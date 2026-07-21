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

            // The bridge synthesizes MAC ACKs locally (see the ack-request handling below and in HostRxLoop),
            // so a real ACK the device transmitted is redundant. Relaying it would only deliver a stray ACK
            // to the host well after its ACK window -- drop it.
            if(IsAckFrame(mpdu))
            {
                return;
            }

            // The model mocks the FCS as 0x0000; OpenThread verifies CRC-16/KERMIT, so put a real FCS in.
            WriteFcs(mpdu);

            if(verbose)
            {
                this.Log(LogLevel.Warning, "Ieee802154HostBridge: device->host MPDU ({0} B, ch{1}): {2}",
                         mpdu.Length, Channel, BitConverter.ToString(mpdu));
            }

            // FrameSent fires at the device's TX START, but a real receiver only holds the frame once it is
            // fully on air. Estimate the on-air time (PHY frame length at the 802.15.4 O-QPSK rate of
            // 32us/byte; the received frame omits the 4-byte preamble, so add it back).
            var airTimeUs = (ulong)(frame.Length + PreambleBytes) * MicrosecondsPerByte;

            // Half-duplex: while the device is transmitting a LARGE frame it cannot receive; the inject pump
            // should defer so a host->device fragment isn't lost colliding with it. This is the dominant loss
            // source for the large fragmented CASE messages. IMPORTANT: only gate on LARGE device frames
            // (>kHalfDuplexGateBytes) -- deferring during every small MLE frame (advertisements etc.) delays
            // otbr's join requests past its (patched-short) MLE timeouts and breaks the otbr<->device attach.
            if(mpdu.Length > kHalfDuplexGateBytes)
            {
                deviceTxInProgress = true;
                machine.ScheduleAction(TimeInterval.FromMicroseconds(airTimeUs + AckTurnaroundMicroseconds),
                                       _ => deviceTxInProgress = false, "ieee802154-devtx-busy");
            }

            if(IsUnicastAckRequest(mpdu))
            {
                // A unicast frame the device expects to be ACKed. Two things must be true or the MLE attach
                // never completes:
                //   * the ACK must land in the device's ~864us MAC ACK window -- the real host ACK can't
                //     round-trip that fast across the virtual/real-time boundary (~10ms lag), so synthesize
                //     it locally and deliver it after the device finishes transmitting (air time) + the
                //     standard turnaround, i.e. right when the device starts listening for its ACK; and
                //   * the ACK must arrive BEFORE the host's data reply. The device waits for the MAC ACK
                //     before accepting a reply; if the (real-time) host's reply is injected first, the device
                //     treats its frame as un-ACKed and retransmits instead of processing the reply. So deliver
                //     the ACK and only THEN relay the frame to the host, chaining them in one action so the
                //     host's reply is necessarily ordered after the ACK.
                var sequenceNumber = mpdu[2];
                machine.ScheduleAction(TimeInterval.FromMicroseconds(airTimeUs + AckTurnaroundMicroseconds),
                                       _ => { InjectToDevice(BuildAck(sequenceNumber)); SendToHost(mpdu); },
                                       "ieee802154-ack-then-relay");
            }
            else
            {
                // Broadcast / no-ACK frame: just relay it, after the device finishes transmitting so the
                // real-time host's reply isn't injected back while the half-duplex device is still on air.
                machine.ScheduleAction(TimeInterval.FromMicroseconds(airTimeUs), _ => SendToHost(mpdu), "ieee802154-relay-to-host");
            }
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

        // ---- Inject an MPDU onto the medium toward the device (host frames, and locally-synthesized ACKs) --
        private void InjectToDevice(byte[] mpdu)
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

            // Register in the shared InterferenceQueue BEFORE raising FrameSent (the receiver drops frames
            // whose sender is absent there -- "TX was aborted") and keep the entry until the frame's air
            // time elapses: the receiver validates the sender against the queue DURING RX, i.e. after
            // FrameSent returns, so a synchronous removal would make it drop the frame. The queue holds a
            // single entry per sender, so reference-count overlapping injections (e.g. a data frame and a
            // synthesized ACK) rather than letting one removal clobber the other's still-in-flight entry.
            if(inFlightInjections++ == 0)
            {
                InterferenceQueue.Add(this, RadioPhyId.Phy_802154_2_4GHz_OQPSK, Channel, 0, frame);
            }
            FrameSent?.Invoke(this, frame);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(FrameOnAirMicroseconds), _ => EndInjection(), "ieee802154-tx-done");
        }

        private void EndInjection()
        {
            if(--inFlightInjections == 0)
            {
                InterferenceQueueRemoveSafe();
            }
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

        // ---- Serialized host->device delivery (collision avoidance) -------------------------------------
        // Enqueue a host frame for delivery to the device and make sure the drain pump is running. Called
        // from the host RX thread; the pump itself runs on the emulation thread.
        private void EnqueueToDevice(byte[] mpdu)
        {
            bool startPump = false;
            lock(injectLock)
            {
                deviceInjectQueue.Enqueue(mpdu);
                if(!injectPumpRunning)
                {
                    injectPumpRunning = true;
                    startPump = true;
                }
            }
            if(startPump)
            {
                var vts = TimeDomainsManager.Instance.GetEffectiveVirtualTimeStamp();
                machine.HandleTimeDomainEvent<object>(_ => PumpDeviceInject(), null, vts);
            }
        }

        // Inject one queued frame, then re-arm after its air time (+ a guard gap) so the device has fully
        // received it before the next frame starts -- no two host frames are ever on air at the device at once.
        private void PumpDeviceInject()
        {
            lock(injectLock)
            {
                if(deviceInjectQueue.Count == 0)
                {
                    injectPumpRunning = false;
                    return;
                }
            }
            // Half-duplex + RX-readiness guard: the emulated radio only accepts a frame while it is in RAC
            // RxSearch (actively listening). If we inject while it is transmitting (its own frame or a MAC ACK
            // for the previous frame), warming up, or still mid-RX of another frame, the frame is dropped
            // "not in RXSEARCH" -- and because we synthesize the host-side MAC-ACK immediately (so otbr believes
            // delivery succeeded), otbr never retransmits it. That silently loses e.g. the 2nd 6LoWPAN fragment
            // of the Child ID Response, so Thread attach never completes. Defer (don't dequeue) until the radio
            // is back in RxSearch so every injected fragment is truly received. deviceTxInProgress is a fast
            // path for the known large-TX case; the RxSearch check covers ACK-TX / warm / mid-RX too.
            if(deviceTxInProgress || !DeviceRadioReadyToReceive())
            {
                if(++injectDeferCount <= MaxInjectDefers)
                {
                    machine.ScheduleAction(TimeInterval.FromMicroseconds(DeviceTxDeferMicroseconds), _ => PumpDeviceInject(), "ieee802154-inject-defer");
                    return;
                }
                // Safety valve: after ~MaxInjectDefers*DeviceTxDeferMicroseconds the radio still isn't listening
                // (device stuck/idle); inject best-effort rather than stall the queue forever.
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: RxSearch gate timed out ({0} defers); injecting best-effort", injectDeferCount);
            }
            injectDeferCount = 0;
            byte[] mpdu;
            lock(injectLock)
            {
                mpdu = deviceInjectQueue.Dequeue();
            }
            InjectToDevice(mpdu);
            var airUs = (ulong)(mpdu.Length + PreambleBytes) * MicrosecondsPerByte + InterFrameGapMicroseconds;
            machine.ScheduleAction(TimeInterval.FromMicroseconds(airUs), _ => PumpDeviceInject(), "ieee802154-device-inject-pump");
        }

        // Is the emulated device radio in RAC RxSearch (ready to accept an injected frame)? Resolve the radio
        // peripheral lazily from the machine (the SiLabs_xG24_LPW sharing this medium). If it can't be found,
        // return true so the bridge falls back to its previous unconditional-inject behaviour (never worse).
        private bool DeviceRadioReadyToReceive()
        {
            if(deviceRadio == null)
            {
                deviceRadio = machine.GetPeripheralsOfType<SiLabs_xG24_LPW>().FirstOrDefault();
                if(deviceRadio == null)
                {
                    return true;
                }
            }
            return deviceRadio.IsRadioInRxSearch;
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

                // The bridge synthesizes ACKs locally, so drop the host's real ACKs (they would otherwise
                // reach the device as stray frames long after its ACK window).
                if(IsAckFrame(mpdu))
                {
                    continue;
                }

                // If the host unicast a frame that requests an ACK, ACK it immediately from this RX thread --
                // which runs in the host's real-time domain, so the host sees the ACK within microseconds.
                // (The device's own real ACK would be delayed ~10ms by the virtual/real-time boundary and
                // miss the host's ACK window.) Then deliver the actual frame to the emulated device.
                if(IsUnicastAckRequest(mpdu))
                {
                    SendToHost(BuildAck(mpdu[2]));
                }

                // Serialize delivery to the device instead of injecting immediately: the emulated radio
                // drops any frame that arrives while it is already receiving another (ReceiveFrame ->
                // "RX already ongoing" collision). The host side has TWO responders (leader + border router)
                // plus periodic MLE advertisements, so their frames routinely overlap the device's multi-ms
                // reception of a large Parent/Child-ID/Data Response -> the MLE attach handshake frame is
                // silently lost and the device never completes attach (it self-partitions). Queue host->device
                // frames and inject them one at a time, spaced by each frame's air time, so the device hears
                // them cleanly, one after another. Device firmware stays stock; this is purely bridge behavior.
                EnqueueToDevice(mpdu);
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
                // Called both from the emulation thread (relaying device frames) and the host RX thread
                // (synthesizing prompt ACKs back to the host), so serialize the sends.
                lock(txLock)
                {
                    s.SendTo(datagram, groupEndpoint);
                }
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Noisy, "Ieee802154HostBridge: tx error: {0}", e.Message);
            }
        }

        // ---- Local MAC-ACK synthesis --------------------------------------------------------------------
        // The device (virtual time) and the host (real time) can't meet each other's ~864us 802.15.4 ACK
        // window across the UDP relay -- Renode aligns the two clocks only in coarse (~ms) steps. So the
        // bridge generates the ACKs locally in each peer's own time domain and suppresses the (late) real
        // ACKs, instead of relaying them. Device firmware stays stock; this is purely bridge behavior.

        private static bool IsAckFrame(byte[] mpdu)
        {
            // 802.15.4 frame type (FCF bits 0-2) == 2 is an acknowledgement.
            return mpdu.Length >= 1 && (mpdu[0] & 0x07) == 0x02;
        }

        private static bool IsUnicastAckRequest(byte[] mpdu)
        {
            // A data (1) or MAC-command (3) frame with the Ack Request bit (FCF bit 5, 0x20) set. Broadcast
            // MLE frames carry AR=0, so they are naturally excluded (and need no ACK).
            return mpdu.Length >= 3 && (mpdu[0] & 0x07) != 0x02 && (mpdu[0] & 0x20) != 0;
        }

        private static byte[] BuildAck(byte sequenceNumber)
        {
            // 802.15.4 Imm-Ack: FCF = 0x0002 (Ack frame, no addressing), the acked sequence number, FCS.
            var ack = new byte[] { 0x02, 0x00, sequenceNumber, 0x00, 0x00 };
            WriteFcs(ack);
            return ack;
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
        private readonly object txLock = new object();
        private int inFlightInjections; // reference count for the single per-sender InterferenceQueue entry
        private readonly object injectLock = new object();
        private readonly System.Collections.Generic.Queue<byte[]> deviceInjectQueue = new System.Collections.Generic.Queue<byte[]>();
        private bool injectPumpRunning; // true while the host->device drain pump is scheduling itself
        private volatile bool deviceTxInProgress; // true while the device is transmitting (half-duplex; can't RX)
        private SiLabs_xG24_LPW deviceRadio; // lazily-resolved device radio, for the RxSearch injection gate
        private int injectDeferCount; // consecutive RxSearch-gate defers for the current frame (safety-valve bound)

        // 802.15.4: min MPDU = FC(2)+seq(1)+FCS(2) = 5; max PSDU = 127. Sync word is a few bytes.
        private const int MinMpdu = 5;
        private const int MaxMpdu = 127;
        private const int MaxSyncBytes = 8;
        private const int MinPhyFrame = MinMpdu + 1; // at least PHR + a minimal MPDU
        private const ulong FrameOnAirMicroseconds = 400; // keep the sender in the InterferenceQueue this long
        // 802.15.4 aTurnaroundTime (12 symbols * 16us): the TX->RX turnaround after the device finishes
        // transmitting, before it listens for the ACK.
        private const ulong AckTurnaroundMicroseconds = 192;
        // 802.15.4 2.4GHz O-QPSK PHY: 250 kbps = 32us/byte; SHR preamble is 4 bytes (not carried in the
        // frame the bridge receives). Used to estimate the device's frame air time for ACK timing.
        private const ulong MicrosecondsPerByte = 32;
        private const int PreambleBytes = 4;
        // Guard gap between serialized host->device frames so the device's RX of one frame fully completes
        // (rxTimer + processing) before the next starts. A few symbol times is plenty.
        private const ulong InterFrameGapMicroseconds = 300;
        // How often the inject pump re-checks when it finds the device mid-transmission (half-duplex defer)
        // or not yet in RxSearch. Also the poll interval for the RxSearch injection gate.
        private const ulong DeviceTxDeferMicroseconds = 200;
        // Safety-valve bound on RxSearch-gate defers per frame (~MaxInjectDefers*DeviceTxDeferMicroseconds of
        // virtual time). A well-behaved rx-on device returns to RxSearch within a few ms after each TX; this
        // just prevents a permanent queue stall if the radio never reports RxSearch.
        private const int MaxInjectDefers = 250; // ~50ms

        // Only large device TX frames (data/6LoWPAN fragments) gate host->device injection; small MLE control
        // frames (~70B advertisements) do NOT, so the otbr<->device MLE attach isn't delayed past its timeouts.
        private const int kHalfDuplexGateBytes = 90;
    }
}
