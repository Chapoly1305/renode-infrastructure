//
// Copyright (c) 2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Wireless
{
    // In-emulation IEEE 802.15.4 bridge that joins the wireless medium next to a STOCK SiLabs Matter device
    // and carries the device's real RAIL 802.15.4 PHY frames to/from a HOST Thread network -- all in
    // software, sharing the emulator's virtual clock (no host 802.15.4 radio / RCP hardware). This is the
    // Thread-operational counterpart of BleCentralBridge: once chip-tool commissions the device over BLE and
    // pushes the Thread dataset, ThreadNetworkEnable brings up the device's OpenThread stack on 802.15.4;
    // this peripheral relays those frames to a host OpenThread "simulation" air (UDP multicast 224.0.0.116)
    // where an ot-rcp/otbr border router lives, so the device attaches to a real Thread network and chip-tool
    // reaches it operationally.
    //
    // The device firmware is unmodified: its real OpenThread MAC (CSMA, auto-ACK) runs on the emulated radio;
    // this peripheral is only the air-interface bridge. Two format differences are reconciled here:
    //   * PHY framing: the model transmits [syncWord(N)][PHR=len][MPDU incl FCS]; OpenThread's simulation air
    //     carries [channel(1)][MPDU incl FCS]. N (sync-word length) is auto-detected per frame and the raw
    //     sync-word bytes are learned from the device's own transmissions for the reverse direction.
    //   * FCS: the model mocks the 2-byte FCS as 0x0000 (two emulated nodes never actually check it -- only
    //     the sync word gates RX). OpenThread DOES verify CRC-16/KERMIT, so device->host frames get a real
    //     FCS recomputed here; host->device frames keep the host's (valid) FCS since the model ignores it.
    //
    // The medium delivers a frame A->B only when their Channel values match, so this bridge sits on the
    // device's 802.15.4 channel; the model's per-frame sync-word check rejects any BLE cross-talk when the
    // same medium also carries the CHIPoBLE connection.
    public class Ieee802154HostBridge : IDoubleWordPeripheral, IRadio
    {
        public Ieee802154HostBridge(IMachine machine, int channel = 15, int nodeId = 2, int portBase = 9000,
                                    string group = "224.0.0.116", string localAddress = "127.0.0.1", bool verbose = false)
        {
            this.machine = machine;
            this.Channel = channel;
            this.nodeId = nodeId;
            this.portBase = portBase;
            this.groupAddress = IPAddress.Parse(group);
            this.localAddress = IPAddress.Parse(localAddress);
            this.verbose = verbose;
            StartHostSockets();
        }

        public void Reset()
        {
            learnedSync = null;
        }

        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public int Channel { get; set; }

        public event Action<IRadio, byte[]> FrameSent;

        // ---- Device -> host: an 802.15.4 PHY frame the device just transmitted on the medium ----------------
        public void ReceiveFrame(byte[] frame, IRadio sender)
        {
            if(frame == null || frame.Length < MinPhyFrame)
            {
                return;
            }

            // The device's on-air frame is [syncWord(N)][PHR=MPDU length][MPDU (MHR+payload+FCS)]. Find N by
            // the PHR self-consistency: frame[N] must equal the number of bytes after the PHR byte. This makes
            // the bridge self-configuring for whatever sync-word length RAIL programs, and rejects BLE frames
            // (whose leading access-address bytes won't yield a consistent PHR).
            var n = DetectSyncLength(frame);
            if(n < 0)
            {
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: ignoring non-802.15.4 frame ({0} B): {1}",
                         frame.Length, BitConverter.ToString(frame));
                return;
            }

            var phr = frame[n];
            var mpdu = new byte[phr];
            Array.Copy(frame, n + 1, mpdu, 0, phr);

            // Learn the exact sync-word bytes for building host->device frames later.
            if(learnedSync == null || learnedSync.Length != n)
            {
                learnedSync = new byte[n];
                Array.Copy(frame, 0, learnedSync, 0, n);
                this.Log(LogLevel.Warning, "Ieee802154HostBridge: learned {0}-byte 802.15.4 sync word {1} (channel {2})",
                         n, BitConverter.ToString(learnedSync), Channel);
            }

            // The model mocks the FCS as 0x0000; OpenThread verifies CRC-16/KERMIT, so put a real FCS in.
            WriteFcs(mpdu);

            if(verbose)
            {
                this.Log(LogLevel.Warning, "Ieee802154HostBridge: device->host MPDU ({0} B, ch{1}): {2}",
                         mpdu.Length, Channel, BitConverter.ToString(mpdu));
            }

            SendToHost(mpdu);
        }

        // Find the sync-word length N (0..MaxSyncBytes) for which the PHR byte is self-consistent with the
        // frame length. Prefer the previously-learned length when it still fits, for stability.
        private int DetectSyncLength(byte[] frame)
        {
            if(learnedSync != null)
            {
                var n = learnedSync.Length;
                if(n < frame.Length && frame[n] == frame.Length - 1 - n && frame[n] >= MinMpdu && frame[n] <= MaxMpdu)
                {
                    return n;
                }
            }
            for(var n = 0; n <= MaxSyncBytes && n < frame.Length; ++n)
            {
                var phr = frame[n];
                if(phr == frame.Length - 1 - n && phr >= MinMpdu && phr <= MaxMpdu)
                {
                    return n;
                }
            }
            return -1;
        }

        // ---- Host -> device: an MPDU received from the host Thread air, injected onto the medium ------------
        private void DeliverFromHost(byte[] mpdu)
        {
            if(mpdu.Length < MinMpdu || mpdu.Length > MaxMpdu)
            {
                return;
            }
            if(learnedSync == null)
            {
                // We haven't seen a device transmission yet, so we don't know the PHY sync word to prepend.
                // In Thread attach the device always transmits first (MLE Parent Request / MAC beacon
                // request), so this only drops host chatter that precedes the device coming up.
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: dropping host->device frame (sync word not yet learned)");
                return;
            }

            var frame = new byte[learnedSync.Length + 1 + mpdu.Length];
            Array.Copy(learnedSync, 0, frame, 0, learnedSync.Length);
            frame[learnedSync.Length] = (byte)mpdu.Length; // PHR
            Array.Copy(mpdu, 0, frame, learnedSync.Length + 1, mpdu.Length);

            if(verbose)
            {
                this.Log(LogLevel.Warning, "Ieee802154HostBridge: host->device MPDU ({0} B, ch{1}): {2}",
                         mpdu.Length, Channel, BitConverter.ToString(mpdu));
            }

            // Transmit onto the medium like the real radio: register in the shared InterferenceQueue BEFORE
            // raising FrameSent (the receiver drops frames whose sender is absent there -- "TX was aborted"),
            // then remove shortly after so the entry doesn't linger past the frame's air time.
            InterferenceQueue.Add(this, RadioPhyId.Phy_802154_2_4GHz_OQPSK, Channel, 0, frame);
            FrameSent?.Invoke(this, frame);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(FrameOnAirMicroseconds), _ => InterferenceQueueRemoveSafe(), "ieee802154-tx-done");
        }

        private void InterferenceQueueRemoveSafe()
        {
            try
            {
                InterferenceQueue.Remove(this);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: InterferenceQueue.Remove: {0}", e.Message);
            }
        }

        // ---- Host UDP transport: OpenThread "simulation" real-time virtual radio (multicast) ----------------
        // Every node joins multicast group 224.0.0.116 and receives on portBase; a node transmits from source
        // port portBase+nodeId (peers derive the sender node id from the source port) to group:portBase, with
        // multicast loopback on. We filter out our own echo by source port. Frame = [channel(1)][MPDU].

        private void StartHostSockets()
        {
            running = true;
            try
            {
                rxSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                rxSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                TrySetReusePort(rxSocket);
                rxSocket.Bind(new IPEndPoint(IPAddress.Any, portBase));
                rxSocket.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership,
                                         new MulticastOption(groupAddress, localAddress));
                rxSocket.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastLoopback, true);

                txSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
                txSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                TrySetReusePort(txSocket);
                txSocket.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastInterface,
                                         BitConverter.ToInt32(localAddress.GetAddressBytes(), 0));
                txSocket.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastLoopback, true);
                txSocket.Bind(new IPEndPoint(localAddress, portBase + nodeId));
                groupEndpoint = new IPEndPoint(groupAddress, portBase);

                this.Log(LogLevel.Warning, "Ieee802154HostBridge: joined OT-sim air {0}:{1} as node {2} (tx port {3}), channel {4}",
                         groupAddress, portBase, nodeId, portBase + nodeId, Channel);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "Ieee802154HostBridge: failed to open OT-sim sockets: {0}", e.Message);
                return;
            }

            rxThread = new Thread(HostRxLoop) { IsBackground = true, Name = "ieee802154-host-rx" };
            rxThread.Start();
        }

        private static void TrySetReusePort(Socket s)
        {
            try
            {
                // SO_REUSEPORT is not in the .NET enum on all platforms; value 0x0200 on BSD/macOS + Linux.
                s.SetSocketOption(SocketOptionLevel.Socket, (SocketOptionName)0x0200, true);
            }
            catch
            {
                // Not fatal -- ReuseAddress is usually enough for multicast RX.
            }
        }

        private void HostRxLoop()
        {
            var buffer = new byte[2048];
            EndPoint remote = new IPEndPoint(IPAddress.Any, 0);
            while(running)
            {
                int len;
                IPEndPoint from;
                try
                {
                    len = rxSocket.ReceiveFrom(buffer, ref remote);
                    from = (IPEndPoint)remote;
                }
                catch(Exception e)
                {
                    if(running)
                    {
                        this.Log(LogLevel.Noisy, "Ieee802154HostBridge: rx error: {0}", e.Message);
                    }
                    continue;
                }

                if(len < 2 || from.Port == portBase + nodeId)
                {
                    continue; // too short, or our own multicast echo
                }

                var channel = buffer[0];
                var mpdu = new byte[len - 1];
                Array.Copy(buffer, 1, mpdu, 0, len - 1);

                // Frame carries the sender's channel; only inject frames on the channel the device is using.
                if(channel != Channel)
                {
                    continue;
                }

                // Marshal onto the emulation thread at the current virtual time (thread-safe; same mechanism
                // the wireless medium uses to deliver inbound frames).
                var vts = TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp();
                machine.HandleTimeDomainEvent<byte[]>(DeliverFromHost, mpdu, vts);
            }
        }

        private void SendToHost(byte[] mpdu)
        {
            var s = txSocket;
            if(s == null)
            {
                return;
            }
            try
            {
                var datagram = new byte[1 + mpdu.Length];
                datagram[0] = (byte)Channel;
                Array.Copy(mpdu, 0, datagram, 1, mpdu.Length);
                s.SendTo(datagram, groupEndpoint);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: tx error: {0}", e.Message);
            }
        }

        // ---- IEEE 802.15.4 FCS: CRC-16/KERMIT (reflected 0x8408, init 0x0000), low byte first --------------
        private static void WriteFcs(byte[] mpdu)
        {
            if(mpdu.Length < 2)
            {
                return;
            }
            var crc = Crc16Kermit(mpdu, mpdu.Length - 2);
            mpdu[mpdu.Length - 2] = (byte)(crc & 0xFF);
            mpdu[mpdu.Length - 1] = (byte)((crc >> 8) & 0xFF);
        }

        private static ushort Crc16Kermit(byte[] data, int length)
        {
            ushort crc = 0;
            for(var i = 0; i < length; ++i)
            {
                crc ^= data[i];
                for(var b = 0; b < 8; ++b)
                {
                    crc = (ushort)((crc & 1) != 0 ? (crc >> 1) ^ 0x8408 : crc >> 1);
                }
            }
            return crc;
        }

        private readonly IMachine machine;
        private readonly int nodeId;
        private readonly int portBase;
        private readonly IPAddress groupAddress;
        private readonly IPAddress localAddress;
        private readonly bool verbose;

        private volatile bool running;
        private Socket rxSocket;
        private Socket txSocket;
        private IPEndPoint groupEndpoint;
        private Thread rxThread;
        private byte[] learnedSync;

        // 802.15.4: min MPDU = FC(2)+seq(1)+FCS(2) = 5; max PSDU = 127. Sync word is a few bytes.
        private const int MinMpdu = 5;
        private const int MaxMpdu = 127;
        private const int MaxSyncBytes = 8;
        private const int MinPhyFrame = MinMpdu + 1; // at least PHR + a minimal MPDU
        private const ulong FrameOnAirMicroseconds = 400;
    }
}
