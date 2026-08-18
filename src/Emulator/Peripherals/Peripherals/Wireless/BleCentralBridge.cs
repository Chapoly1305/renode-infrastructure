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

namespace Antmicro.Renode.Peripherals.Wireless
{
    // In-emulation BLE CENTRAL that joins a BLEMedium next to a STOCK SiLabs Matter device, receives its
    // CHIPoBLE advertising, drives a BLE connection, and (eventually) bridges the CHIPoBLE C1/C2 GATT
    // characteristics to a host chip-tool over a socket -- all in software, sharing the emulator's virtual
    // clock so connection-event timing is deterministic (no host BLE hardware / OS Bluetooth stack).
    //
    // The device firmware is unmodified: its real sl_bt link layer + GATT server run on the emulated radio;
    // this peripheral is the *central* side of the air interface. The on-medium byte[] is a raw BLE PHY PDU
    // framed as [4B access address, little-endian][PDU header+payload][CRC(ignored by the model)]; the
    // WirelessMedium delivers a frame from radio A to radio B only when their Channel values match.
    //
    // STEP 1 (this revision): passively receive + log the device's advertising frames to validate the
    // medium plumbing for a lightweight custom radio. Connection/GATT/socket bridging follow.
    public class BleCentralBridge : IDoubleWordPeripheral, IRadio
    {
        public BleCentralBridge(IMachine machine, int port = 3500)
        {
            this.machine = machine;
            this.port = port;
            // BLE primary advertising channels are 37/38/39; in this radio model's channel numbering those
            // are 0 (2402 MHz), 12 (2426 MHz) and 39 (2480 MHz). Scan on 37 by default.
            Channel = 0;
            StartHostSocket();
        }

        public void Reset()
        {
            state = State.Scanning;
            connectIndScheduled = false;
            connectAttempt = 0;
            suppressReconnect = false;
        }

        public uint ReadDoubleWord(long offset)
        {
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
        }

        public void ReceiveFrame(byte[] frame, IRadio sender)
        {
            if(frame == null || frame.Length < 6)
            {
                return;
            }

            // Advertising PDU on the medium: [AA(4)=D6-BE-89-8E][header(2)][AdvA(6)][AdvData...][CRC].
            var isAdv = frame[0] == 0xD6 && frame[1] == 0xBE && frame[2] == 0x89 && frame[3] == 0x8E;
            if(!isAdv)
            {
                // A data-channel PDU: is it on OUR connection's access address?
                var recvAa = (uint)(frame[0] | (frame[1] << 8) | (frame[2] << 16) | (frame[3] << 24));
                if(state == State.Connected && recvAa == ConnectionAccessAddress)
                {
                    ProcessDataPdu(frame);
                }
                return;
            }

            if(state != State.Scanning || suppressReconnect)
            {
                return;
            }

            var pduType = frame[4] & 0x0F;          // 0x00 = ADV_IND (connectable, undirected)
            var isConnectable = pduType == 0x00 || pduType == 0x01;
            var isChipoble = false;
            for(var i = 4; i + 1 < frame.Length; ++i)
            {
                if(frame[i] == 0x16 && frame[i + 1] == 0xF6) // AD type Service Data, UUID 0xFFF6
                {
                    isChipoble = true;
                    break;
                }
            }
            if(!isConnectable || !isChipoble)
            {
                return;
            }

            // Capture the device (peripheral) address + address type from this ADV_IND.
            deviceAddress = new byte[6];
            Array.Copy(frame, 6, deviceAddress, 0, 6); // AdvA is right after the 2-byte header at offset 6
            deviceAddressRandom = (frame[4] & 0x40) != 0; // TxAdd

            this.Log(LogLevel.Warning, "BleCentralBridge: connectable CHIPoBLE ADV_IND from {0} on ch{1} -> sending CONNECT_IND",
                string.Join(":", deviceAddress.Reverse().Select(b => b.ToString("X2"))), Channel);

            // The peripheral opens a short RX window (~T_IFS) after its ADV_IND on the SAME channel. The
            // medium delivers synchronously in virtual time, so schedule the CONNECT_IND to land inside that
            // window (~450 us after the ADV start, per the model's Tx->Tx2Rx->RxSearch timing).
            //
            // That constant is measured from ReceiveFrame, which fires at the sender's TX-START, so the
            // right value depends on the ADV's air time -- i.e. on its length. Measured from the
            // SiLabs_xG24_LPW RSM trace (add `logLevel 1 sysbus.radio` to the scenario):
            //
            //   Tuya 4701/79   ADV 35 B -> 284 us air; RxSearch [ADVstart+500, +600] us
            //   LG  4142/8704  ADV 46 B -> 377 us air; RxSearch [ADVstart+613, +900] us
            //
            // 450 us lands at ADVstart+550 (ScheduleAction always adds one 100 us default quantum): inside
            // Tuya's window, 13 us short of LG's, which is dropped as "not in RXSEARCH". No single constant
            // serves both. ATTEMPT 0 THEREFORE KEEPS THE HISTORICAL CONSTANT UNCHANGED -- only a retry, which
            // by definition means the constant did not work, switches to the air-time-derived aim and sweeps.
            connectIndScheduled = true;
            state = State.Connecting;
            if(connectAttempt == 0)
            {
                machine.ScheduleAction(TimeInterval.FromMicroseconds(ConnectIndDelayMicroseconds), _ => SendConnectInd(), "ble-central-connect-ind");
                return;
            }
            var advAirTimeUs = (frame.Length + 1) * 8;    // 1 Msym/s, 8 us/byte, + 1 preamble byte
            var sweepStep = (connectAttempt / 2) * ConnectIndSweepStepMicroseconds;
            if(sweepStep > ConnectIndSweepCapMicroseconds)
            {
                sweepStep = ConnectIndSweepCapMicroseconds;
            }
            var aimUs = advAirTimeUs + (int)ConnectIndAimMicroseconds - (int)SchedulerQuantumMicroseconds
                        + ((connectAttempt % 2 == 1) ? sweepStep : -sweepStep);
            if(aimUs < 50)
            {
                aimUs = 50;
            }
            this.Log(LogLevel.Warning,
                "BleCentralBridge: CONNECT_IND retry {0} -- ADV {1} B ({2} us on air), aiming {3} us after ADV start",
                connectAttempt, frame.Length, advAirTimeUs, aimUs + (int)SchedulerQuantumMicroseconds);
            machine.ScheduleAction(TimeInterval.FromMicroseconds((ulong)aimUs), _ => SendConnectInd(), "ble-central-connect-ind");
        }

        private void SendConnectInd()
        {
            var pdu = BuildConnectInd();
            this.Log(LogLevel.Warning, "BleCentralBridge: TX CONNECT_IND on ch{0} ({1} B): {2}",
                Channel, pdu.Length, string.Join("-", pdu.Select(b => b.ToString("X2"))));
            SendFrame(pdu);

            // Start the connection as MASTER. Init CSA#1 + LL sequence state, then schedule the first
            // connection event inside the peripheral's transmit window (CONNECT_IND end + 1.25ms +
            // WinOffset). The peripheral locks its anchor to the first master packet, so subsequent events
            // are simply +connInterval from each other (shared virtual clock -> exact alignment).
            state = State.Connected;
            connEventCounter = 0;
            lastUnmappedChannel = 0;
            centralSn = 0;
            centralNesn = 0;
            lastSentWasData = false;
            versionSent = false;
            l2capReassembly = null;
            attMtu = 23;
            c1Handle = 0;
            c2Handle = 0;
            c2CccdHandle = 0;
            gattReady = false;
            discoveryStarted = false;
            missStreak = 0;
            peerAnsweredLastEvent = false;
            searchEngaged = false;
            // currentIntervalUs and searchEverEngaged deliberately SURVIVE a reconnect: one connection only
            // affords ~34 trials before the peripheral's own supervision timeout, which is not enough to
            // cover the widest error measured (Tuya 4701/436, +1121 us). Continuing the sweep across
            // reconnects covers it in a few seconds instead. Reset() clears them.
            previousEventIntervalUs = currentIntervalUs;
            anchorEvent = 0;
            anchorTxUs = NowMicroseconds() + Anchor0DelayMicroseconds;
            // A connection generation, so a timer left over from a previous connection cannot inject a stray
            // event into this one after a rescan.
            connectionGeneration++;
            var generation = connectionGeneration;
            machine.ScheduleAction(TimeInterval.FromMicroseconds(Anchor0DelayMicroseconds), _ => ConnectionEvent(generation, currentIntervalUs), "ble-central-conn-event");
            // After the connection settles (LL control drains over the first few events), kick off GATT with
            // an ATT MTU exchange to validate the ATT path end-to-end.
            machine.ScheduleAction(TimeInterval.FromMicroseconds(200000), _ => { if(generation == connectionGeneration) { SendAttMtuRequest(); } }, "ble-central-att-mtu");
        }

        // One connection event as the master: hop to the data channel, transmit our PDU (queued LL/L2CAP
        // data, else an empty PDU), then schedule the next event one connection interval later. The device's
        // real sl_bt responds each event; its response arrives in ReceiveFrame -> ProcessDataPdu.
        private void ConnectionEvent(uint generation, double intervalThatPlacedThisEvent)
        {
            if(state != State.Connected || generation != connectionGeneration)
            {
                return;
            }

            // Close the books on the PREVIOUS event, whose outcome only becomes knowable now. The peripheral
            // answers ~T_IFS (150 us) after our transmission -- i.e. ~150 us into a 30 ms interval -- so by
            // the time the next event starts, its answer has either arrived or it never will. Judging it at
            // the END of the previous event, before the reply could physically have arrived, is precisely
            // what made an earlier version of this change believe a perfectly healthy link had missed every
            // single event. Do not move this.
            if(connEventCounter > 0)
            {
                if(peerAnsweredLastEvent)
                {
                    // The peer heard the PREVIOUS event, which was placed by previousEventIntervalUs. If the
                    // search was engaged, that value is the peripheral's real connection interval: adopt it
                    // and disengage. This is the ONLY place currentIntervalUs is ever written, and it cannot
                    // be reached unless the search was engaged, which needs SearchEngageEvents CONFIRMED
                    // consecutive misses first.
                    if(searchEngaged)
                    {
                        this.Log(LogLevel.Warning,
                            "BleCentralBridge: peer re-acquired at event {0} -- adopting connection interval {1:F0} us (was {2:F0})",
                            connEventCounter - 1, previousEventIntervalUs, currentIntervalUs);
                        currentIntervalUs = previousEventIntervalUs;
                        searchEngaged = false;
                        searchSweepBase = 0;
                    }
                    missStreak = 0;
                }
                else
                {
                    missStreak++;
                }
            }
            peerAnsweredLastEvent = false;
            previousEventIntervalUs = intervalThatPlacedThisEvent;

            // Bridge-side link supervision. Without it the bridge stays in State.Connected for ever:
            // ReceiveFrame drops every ADV_IND while state != Scanning, so ONE missed CONNECT_IND or one lost
            // link is permanent and the run simply goes silent. Every stalled image in the deployed corpus
            // ends exactly there -- the DEVICE gives up at its own 1 s supervision timeout and goes back to
            // advertising (RSM channel 0/12/39 in the radio trace), while the bridge keeps hopping data
            // channels and talking to nobody. SupervisionEvents matches the 1 s SupervisionTimeout the
            // CONNECT_IND itself advertises, so the peripheral always gives up first and is already
            // advertising again by the time we rescan.
            if(missStreak > SupervisionEvents)
            {
                this.Log(LogLevel.Warning,
                    "BleCentralBridge: no peer PDU for {0} consecutive connection events -- link lost, rescanning (next CONNECT_IND attempt {1})",
                    missStreak, connectAttempt + 1);
                AbandonConnection();
                return;
            }

            Channel = DataChannelToRf(NextDataChannel());

            int llid;
            byte[] payload;
            lock(txLock)
            {
                if(txQueue.Count > 0)
                {
                    var item = txQueue.Peek(); // keep it queued until the device ACKs it
                    llid = item.Llid;
                    payload = item.Payload;
                    lastSentWasData = true;
                }
                else
                {
                    llid = 0x01;               // LL Data PDU, empty -> keep-alive / poll
                    payload = EmptyPayload;
                    lastSentWasData = false;
                }
            }

            lastTxUs = NowMicroseconds();
            SendFrame(BuildDataPdu(llid, payload));
            connEventCounter++;

            // THE GATE. Until the link has missed SearchEngageEvents CONSECUTIVE, CONFIRMED events, this is
            // the original line, unmodified, with the original constant. A healthy link -- and a link that
            // drops the odd packet -- never reaches the else branch, never reads an anchor, and never writes
            // currentIntervalUs.
            if(missStreak < SearchEngageEvents && !searchEverEngaged)
            {
                machine.ScheduleAction(TimeInterval.FromMicroseconds(ConnIntervalMicroseconds), _ => ConnectionEvent(generation, ConnIntervalMicroseconds), "ble-central-conn-event");
                return;
            }
            if(missStreak < SearchEngageEvents)
            {
                // The search already ran on this link: keep the interval it found, and keep timing events
                // from the last transmission the peer actually heard rather than from our own clock, so a
                // single missed event cannot let the two timelines walk apart.
                ScheduleFromAnchor(generation, currentIntervalUs);
                return;
            }

            // Engaged. The measured cause of every remaining stall is that the EMULATED peripheral's
            // connection interval is not 30.000 ms, and the error is not always the crystal:
            //
            //   HEIMAN 4619/4101 (reaches GATT)  30.0000 ms  widening  -0.1 us/event  <- matched pair,
            //   HEIMAN 4619/4097 (stalls)        30.3306 ms  widening +30.6 us/event     same board
            //   IKEA   4476/32774                30.3304 ms   (crystal already correct)
            //   Tuya   4701/436                  31.1209 ms   (crystal already correct)
            //
            // A central fixed at 30.000 ms drifts 330..1120 us per event out of a window that widens only
            // ~30 us per event. From the last event the peer demonstrably heard, re-time event k to
            // anchor + k * trial and step the TRIAL INTERVAL through +/-150, +/-300 ... us. Trialling an
            // interval (not an offset) makes the offset grow with k exactly as the drift does; a fixed
            // offset sweep cannot track it. The peripheral re-anchors on any packet it hears, so one hit
            // both proves the interval and ends the search.
            searchEverEngaged = true;
            searchEngaged = true;
            // Trial n = 0, +1, -1, +2, -2, ... steps away from the interval we currently believe in.
            // Starting at ZERO matters: it re-tries the current interval first, so a healthy link that
            // simply went quiet for a few events is re-acquired instead of being walked off its own grid.
            //
            // Step size is set by the peripheral, not by us. Its RX window half-width grows ~30 us per
            // event since its last sync, while a wrong trial's offset grows k * error over the same k
            // events, so it hears us iff |error| <= 30 + 40/k -- i.e. ~30 us regardless of k. A coarser
            // step simply steps over every value that would have worked.
            var n = searchSweepBase + (missStreak - SearchEngageEvents);
            var trialStep = ((n + 1) / 2) * IntervalTrialStepMicroseconds;
            if(trialStep > IntervalTrialCapMicroseconds)
            {
                trialStep = IntervalTrialCapMicroseconds;
            }
            var trialUs = currentIntervalUs + ((n % 2 == 1) ? trialStep : -trialStep);
            this.Log(LogLevel.Debug, "BleCentralBridge: event {0}, {1} consecutive misses -- trialling interval {2:F0} us",
                connEventCounter, missStreak, trialUs);
            ScheduleFromAnchor(generation, trialUs);
        }

        // Place the next connection event at anchor + k * intervalUs, where the anchor is the last
        // transmission the peer demonstrably heard. Absolute rather than relative, so missed events do not
        // accumulate error into our own timeline.
        private void ScheduleFromAnchor(uint generation, double intervalUs)
        {
            var targetUs = anchorTxUs + (connEventCounter - anchorEvent) * intervalUs;
            var delayUs = targetUs - NowMicroseconds();
            if(delayUs < MinimumEventDelayMicroseconds)
            {
                delayUs = MinimumEventDelayMicroseconds;
            }
            machine.ScheduleAction(TimeInterval.FromMicroseconds((ulong)delayUs), _ => ConnectionEvent(generation, intervalUs), "ble-central-conn-event");
        }

        private double NowMicroseconds()
        {
            return machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
        }

        // Give up on this connection and go back to scanning, so the next ADV_IND -- the device is already
        // advertising again by then -- gets a fresh CONNECT_IND with a different aim.
        private void AbandonConnection()
        {
            if(missStreak > SearchEngageEvents)
            {
                // Carry the sweep position into the next connection attempt instead of restarting it.
                searchSweepBase += missStreak - SearchEngageEvents;
            }
            state = State.Scanning;
            // Put the receiver back on an ADVERTISING channel. ConnectionEvent hops
            // Channel across the 37 data channels via DataChannelToRf, which maps
            // 0..36 -> 1..11,13..38 and can therefore NEVER return 0, 12 or 39 -- the
            // only three channels a peripheral advertises on. WirelessMedium delivers
            // a frame only when receiver.Channel == sender.Channel, so a bridge left
            // parked on a data channel is deaf to every subsequent ADV_IND and
            // "rescanning" never rescans. It is also the TX side: the next CONNECT_IND
            // would go out on a channel nobody is listening to.
            // Measured across all 106 corpus workspaces before this fix: not one log
            // contained a second accepted ADV_IND, and the CONNECT_IND retry below had
            // never executed in any run.
            Channel = 0;
            connectIndScheduled = false;
            connectAttempt++;
            missStreak = 0;
            peerAnsweredLastEvent = false;
            gattReady = false;
            discoveryStarted = false;
            l2capReassembly = null;
            lock(txLock)
            {
                txQueue.Clear();
            }
        }

        // BLE Channel Selection Algorithm #1 (all 37 data channels enabled -> mapped == unmapped).
        private int NextDataChannel()
        {
            lastUnmappedChannel = (lastUnmappedChannel + HopIncrement) % 37;
            return lastUnmappedChannel;
        }

        // BLE data-channel index (0..36) -> this radio model's RF channel index (adv 37/38/39 = RF 0/12/39).
        private static int DataChannelToRf(int dataChannel)
        {
            return dataChannel <= 10 ? dataChannel + 1 : dataChannel + 2;
        }

        // Build a connection data-channel PDU: [connAA(4,LE)][header0][len][payload][CRC(3, ignored)].
        private byte[] BuildDataPdu(int llid, byte[] payload)
        {
            payload = payload ?? EmptyPayload;
            var header0 = (byte)((llid & 0x03) | (centralNesn << 2) | (centralSn << 3)); // MD=0
            var pdu = new byte[4 + 2 + payload.Length + 3];
            pdu[0] = (byte)(ConnectionAccessAddress & 0xFF);
            pdu[1] = (byte)((ConnectionAccessAddress >> 8) & 0xFF);
            pdu[2] = (byte)((ConnectionAccessAddress >> 16) & 0xFF);
            pdu[3] = (byte)((ConnectionAccessAddress >> 24) & 0xFF);
            pdu[4] = header0;
            pdu[5] = (byte)payload.Length;
            Array.Copy(payload, 0, pdu, 6, payload.Length);
            return pdu;
        }

        // Process a received connection data PDU: LL acknowledgement (SN/NESN) + dispatch payload.
        private void ProcessDataPdu(byte[] frame)
        {
            // Any PDU on our access address means the peer heard the packet we just sent: a BLE peripheral
            // only ever transmits in reply to its master inside a connection event. Recorded here and acted
            // on at the START of the next event (see ConnectionEvent); nothing about the current event's
            // scheduling depends on it.
            peerAnsweredLastEvent = true;
            connectAttempt = 0;
            if(connEventCounter > 0)
            {
                // The anchor: the last transmission the peer demonstrably heard. Written unconditionally
                // because it is cheap; read only by the engaged search path.
                anchorTxUs = lastTxUs;
                anchorEvent = connEventCounter - 1;
            }

            var header0 = frame[4];
            var len = frame[5];
            var llid = header0 & 0x03;
            var peerSn = (header0 >> 3) & 1;
            var peerNesn = (header0 >> 2) & 1;

            // Did the peer acknowledge our last transmission? (its NESN advanced past our SN)
            if(peerNesn != centralSn)
            {
                centralSn ^= 1;
                lock(txLock)
                {
                    if(lastSentWasData && txQueue.Count > 0)
                    {
                        txQueue.Dequeue();
                    }
                }
                lastSentWasData = false;
            }

            // Is this a new PDU from the peer? (its SN matches what we expect next)
            if(peerSn == centralNesn)
            {
                centralNesn ^= 1;
                if(len > 0)
                {
                    var payload = new byte[len];
                    Array.Copy(frame, 6, payload, 0, Math.Min(len, frame.Length - 6));
                    if(llid == 0x03)
                    {
                        HandleLlControl(payload);
                    }
                    else
                    {
                        HandleL2cap(llid, payload);
                    }
                }
            }
            // else: peer retransmission -- already processed, just re-acked above.
        }

        // Minimal LL control responder -- enough to satisfy the device's sl_bt so the connection stays up.
        private void HandleLlControl(byte[] p)
        {
            var opcode = p[0];
            this.Log(LogLevel.Warning, "BleCentralBridge: RX LL_CONTROL opcode=0x{0:X2}", opcode);
            switch(opcode)
            {
            case 0x0C: // LL_VERSION_IND -> reply once with ours (VersNr 5.1, CompId 0xFFFF)
                if(!versionSent)
                {
                    EnqueueLlControl(new byte[] { 0x0C, 0x0A, 0xFF, 0xFF, 0x00, 0x00 });
                    versionSent = true;
                }
                break;
            case 0x08: // LL_FEATURE_REQ
            case 0x0E: // LL_SLAVE_FEATURE_REQ
                EnqueueLlControl(new byte[] { 0x09, 0, 0, 0, 0, 0, 0, 0, 0 }); // LL_FEATURE_RSP (no optional features)
                break;
            case 0x14: // LL_LENGTH_REQ -> LL_LENGTH_RSP (251 octets / 2120 us)
                EnqueueLlControl(new byte[] { 0x15, 0xFB, 0x00, 0x48, 0x08, 0xFB, 0x00, 0x48, 0x08 });
                break;
            case 0x0F: // LL_CONNECTION_PARAM_REQ -> accept by echoing as LL_CONNECTION_PARAM_RSP
                {
                    var rsp = (byte[])p.Clone();
                    rsp[0] = 0x10;
                    EnqueueLlControl(rsp);
                }
                break;
            case 0x02: // LL_TERMINATE_IND
                this.Log(LogLevel.Warning, "BleCentralBridge: peer terminated the connection (reason 0x{0:X2})", p.Length > 1 ? p[1] : 0);
                state = State.Scanning;
                suppressReconnect = true;
                break;
            default:   // anything we don't implement -> LL_UNKNOWN_RSP
                EnqueueLlControl(new byte[] { 0x07, opcode });
                break;
            }
        }

        private void EnqueueLlControl(byte[] controlPayload)
        {
            lock(txLock)
            {
                txQueue.Enqueue(new TxItem { Llid = 0x03, Payload = controlPayload });
            }
        }

        // Reassemble L2CAP (LLID 0x02 = start, 0x01 = continuation) and dispatch the ATT channel (CID 0x0004).
        private void HandleL2cap(int llid, byte[] fragment)
        {
            if(llid == 0x02) // start of a new L2CAP PDU
            {
                l2capReassembly = fragment;
            }
            else if(l2capReassembly != null) // continuation
            {
                var combined = new byte[l2capReassembly.Length + fragment.Length];
                Array.Copy(l2capReassembly, 0, combined, 0, l2capReassembly.Length);
                Array.Copy(fragment, 0, combined, l2capReassembly.Length, fragment.Length);
                l2capReassembly = combined;
            }
            else
            {
                return;
            }

            if(l2capReassembly.Length < 4)
            {
                return; // need the 4-byte L2CAP header
            }
            var l2capLen = l2capReassembly[0] | (l2capReassembly[1] << 8);
            if(l2capReassembly.Length < 4 + l2capLen)
            {
                return; // more fragments to come
            }
            var cid = l2capReassembly[2] | (l2capReassembly[3] << 8);
            var sdu = new byte[l2capLen];
            Array.Copy(l2capReassembly, 4, sdu, 0, l2capLen);
            l2capReassembly = null;

            if(cid == 0x0004) // ATT
            {
                HandleAtt(sdu);
            }
        }

        private void HandleAtt(byte[] att)
        {
            if(att.Length == 0)
            {
                return;
            }
            this.Log(LogLevel.Warning, "BleCentralBridge: RX ATT ({0} B): {1}", att.Length,
                string.Join("-", att.Select(b => b.ToString("X2"))));
            var opcode = att[0];
            switch(opcode)
            {
            case 0x02: // ATT_EXCHANGE_MTU_REQ *from the peripheral*
                // Not every stack answers our MTU request; some open the exchange
                // themselves. Without this case the opcode fell to `default:`,
                // discovery was never started, and both sides waited forever --
                // observed on Tuya firmware that was otherwise a fully booted Matter
                // device, and which looked like a boot failure because of it.
                // Answer, then proceed exactly as for the response path.
                attMtu = att.Length >= 3
                    ? Math.Min((ushort)(att[1] | (att[2] << 8)), LocalAttMtu)
                    : LocalAttMtu;
                SendAtt(new byte[] { 0x03, (byte)(LocalAttMtu & 0xFF), (byte)(LocalAttMtu >> 8) });
                this.Log(LogLevel.Warning, "BleCentralBridge: peer-initiated ATT MTU exchange -> {0}; discovering CHIPoBLE service", attMtu);
                if(!discoveryStarted)
                {
                    discoveryStarted = true;
                    DiscoverChipobleService();
                }
                break;
            case 0x03: // ATT_EXCHANGE_MTU_RSP -> begin CHIPoBLE service discovery
                attMtu = Math.Min((ushort)(att[1] | (att[2] << 8)), LocalAttMtu);
                this.Log(LogLevel.Warning, "BleCentralBridge: ATT MTU exchanged -> {0}; discovering CHIPoBLE service", attMtu);
                // Guarded so a peer that both requests and responds cannot start
                // two overlapping discovery procedures.
                if(!discoveryStarted)
                {
                    discoveryStarted = true;
                    DiscoverChipobleService();
                }
                break;
            case 0x07: // ATT_FIND_BY_TYPE_VALUE_RSP -> CHIPoBLE (0xFFF6) primary service handle range
                if(att.Length >= 5)
                {
                    serviceStart = (ushort)(att[1] | (att[2] << 8));
                    serviceEnd = (ushort)(att[3] | (att[4] << 8));
                    this.Log(LogLevel.Warning, "BleCentralBridge: CHIPoBLE service 0x{0:X4}-0x{1:X4}; discovering characteristics", serviceStart, serviceEnd);
                    DiscoverCharacteristics((ushort)serviceStart);
                }
                break;
            case 0x09: // ATT_READ_BY_TYPE_RSP -> characteristic declarations (0x2803)
                ParseCharacteristicDeclarations(att);
                break;
            case 0x05: // ATT_FIND_INFORMATION_RSP -> looking for C2's CCCD (0x2902)
                ParseFindInformation(att);
                break;
            case 0x01: // ATT_ERROR_RSP (e.g. ATTRIBUTE_NOT_FOUND ends a discovery sub-procedure)
                HandleAttError(att);
                break;
            case 0x1B: // ATT_HANDLE_VALUE_NOTIFICATION
            case 0x1D: // ATT_HANDLE_VALUE_INDICATION
                HandleValueNotification(opcode, att);
                break;
            case 0x13: // ATT_WRITE_RSP
                break;
            default:
                break;
            }
        }

        // Discover the CHIPoBLE primary service (16-bit UUID 0xFFF6) directly by value.
        private void DiscoverChipobleService()
        {
            // ATT_FIND_BY_TYPE_VALUE_REQ: [0x06][start(2)][end(2)][type=0x2800][value=0xFFF6 LE]
            SendAtt(new byte[] { 0x06, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28, 0xF6, 0xFF });
        }

        // ATT_READ_BY_TYPE_REQ for characteristic declarations (0x2803) within [start, serviceEnd].
        private void DiscoverCharacteristics(ushort start)
        {
            SendAtt(new byte[] { 0x08, (byte)(start & 0xFF), (byte)(start >> 8),
                                 (byte)(serviceEnd & 0xFF), (byte)(serviceEnd >> 8), 0x03, 0x28 });
        }

        private void ParseCharacteristicDeclarations(byte[] att)
        {
            var itemLen = att[1]; // 7 for 16-bit char UUID, 21 for 128-bit
            ushort lastHandle = 0;
            for(var i = 2; i + itemLen <= att.Length; i += itemLen)
            {
                var declHandle = (ushort)(att[i] | (att[i + 1] << 8));
                var valueHandle = (ushort)(att[i + 3] | (att[i + 4] << 8));
                lastHandle = declHandle;
                if(itemLen == 21) // 128-bit UUID at att[i+5 .. i+21], little-endian
                {
                    // CHIPoBLE char UUID 18EE2EF5-263D-4559-959F-4F9C429F9D1x -> LE = [1x][9D]...[EE][18].
                    // Match the fixed base (LE bytes: [1]=0x9D, [14]=0xEE, [15]=0x18) and use byte[0] for C1/C2.
                    var u0 = att[i + 5];
                    if(att[i + 20] == 0x18 && att[i + 19] == 0xEE && att[i + 6] == 0x9D)
                    {
                        if(u0 == 0x11)
                        {
                            c1Handle = valueHandle;
                            this.Log(LogLevel.Warning, "BleCentralBridge: found C1 (write) valueHandle=0x{0:X4}", c1Handle);
                        }
                        else if(u0 == 0x12)
                        {
                            c2Handle = valueHandle;
                            this.Log(LogLevel.Warning, "BleCentralBridge: found C2 (indicate) valueHandle=0x{0:X4}", c2Handle);
                        }
                    }
                }
            }
            // Continue reading characteristics after the last one, unless we already have C1 + C2.
            if((c1Handle == 0 || c2Handle == 0) && lastHandle != 0 && lastHandle < serviceEnd)
            {
                DiscoverCharacteristics((ushort)(lastHandle + 1));
            }
            else if(c2Handle != 0)
            {
                DiscoverCccd();
            }
        }

        // Find C2's Client Characteristic Configuration Descriptor (0x2902) after the C2 value handle.
        private void DiscoverCccd()
        {
            var start = (ushort)(c2Handle + 1);
            SendAtt(new byte[] { 0x04, (byte)(start & 0xFF), (byte)(start >> 8),
                                 (byte)(serviceEnd & 0xFF), (byte)(serviceEnd >> 8) });
        }

        private void ParseFindInformation(byte[] att)
        {
            var format = att[1]; // 1 = 16-bit UUIDs
            if(format != 0x01)
            {
                return;
            }
            for(var i = 2; i + 4 <= att.Length; i += 4)
            {
                var handle = (ushort)(att[i] | (att[i + 1] << 8));
                var uuid = (ushort)(att[i + 2] | (att[i + 3] << 8));
                if(uuid == 0x2902)
                {
                    c2CccdHandle = handle;
                    this.Log(LogLevel.Warning, "BleCentralBridge: found C2 CCCD handle=0x{0:X4} -- CHIPoBLE GATT ready (C1=0x{1:X4} C2=0x{2:X4})",
                        c2CccdHandle, c1Handle, c2Handle);
                    gattReady = true;
                    return;
                }
            }
        }

        private void HandleAttError(byte[] att)
        {
            var reqOpcode = att.Length > 1 ? att[1] : 0;
            // ATTRIBUTE_NOT_FOUND on Read-By-Type ends characteristic discovery -> move on to the CCCD.
            if(reqOpcode == 0x08 && c2Handle != 0 && c2CccdHandle == 0)
            {
                DiscoverCccd();
            }
        }

        private void HandleValueNotification(byte opcode, byte[] att)
        {
            var handle = att.Length >= 3 ? (ushort)(att[1] | (att[2] << 8)) : (ushort)0;
            var value = att.Length > 3 ? att.Skip(3).ToArray() : EmptyPayload;
            if(handle == c2Handle)
            {
                // Forward the CHIPoBLE C2 payload to chip-tool as an INDICATION (0x06) frame.
                SendHostFrame(HostFrameIndication, value);
            }
            if(opcode == 0x1D)
            {
                SendAtt(new byte[] { 0x1E }); // ATT_HANDLE_VALUE_CONFIRMATION
            }
        }

        // ---- Host (chip-tool) side: a socket speaking the fake CHIPoBLE C1/C2 frame protocol -------------
        // Frame = [1B type][2B len LE][payload]. Types: 1 CONNECT, 2 DISCONNECT, 3 WRITE_REQUEST(C1),
        // 4 SUBSCRIBE, 5 UNSUBSCRIBE, 6 INDICATION(C2). chip-tool's existing host-side FakeBleTransport
        // connects here (CHIP_FAKE_BLE_PORT); we translate its C1 writes / subscribe into real GATT and its
        // C2 indications back, so the device runs entirely stock.

        private void StartHostSocket()
        {
            running = true;
            socketThread = new Thread(HostSocketLoop) { IsBackground = true, Name = "ble-central-host-socket" };
            socketThread.Start();
        }

        private void HostSocketLoop()
        {
            try
            {
                listener = new TcpListener(IPAddress.Loopback, port);
                listener.Start();
                this.Log(LogLevel.Warning, "BleCentralBridge: host CHIPoBLE socket listening on 127.0.0.1:{0}", port);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "BleCentralBridge: failed to open host socket on port {0}: {1}", port, e.Message);
                return;
            }

            while(running)
            {
                try
                {
                    using(var c = listener.AcceptTcpClient())
                    {
                        c.NoDelay = true;
                        hostStream = c.GetStream();
                        this.Log(LogLevel.Warning, "BleCentralBridge: chip-tool connected to host socket");
                        ReadHostFrames(hostStream);
                        hostStream = null;
                    }
                }
                catch(Exception e)
                {
                    if(running)
                    {
                        this.Log(LogLevel.Warning, "BleCentralBridge: host socket error: {0}", e.Message);
                    }
                }
            }
        }

        private void ReadHostFrames(NetworkStream s)
        {
            var header = new byte[3];
            while(running && ReadExact(s, header, 3))
            {
                var type = header[0];
                var len = header[1] | (header[2] << 8);
                var payload = new byte[len];
                if(len > 0 && !ReadExact(s, payload, len))
                {
                    break;
                }
                HandleHostFrame(type, payload);
            }
        }

        private static bool ReadExact(NetworkStream s, byte[] buf, int len)
        {
            var got = 0;
            while(got < len)
            {
                var n = s.Read(buf, got, len - got);
                if(n <= 0)
                {
                    return false;
                }
                got += n;
            }
            return true;
        }

        private void HandleHostFrame(byte type, byte[] payload)
        {
            switch(type)
            {
            case HostFrameConnect:
                this.Log(LogLevel.Warning, "BleCentralBridge: host CONNECT (BLE {0})", gattReady ? "ready" : "connecting");
                break;
            case HostFrameSubscribe:
                // Enable CHIPoBLE C2 indications by writing 0x0002 to its CCCD.
                EnqueueAtt(new byte[] { 0x12, (byte)(c2CccdHandle & 0xFF), (byte)(c2CccdHandle >> 8), 0x02, 0x00 });
                this.Log(LogLevel.Warning, "BleCentralBridge: host SUBSCRIBE -> write C2 CCCD 0x{0:X4}", c2CccdHandle);
                break;
            case HostFrameWriteRequest:
                // chip-tool writes C1 -> ATT Write Request (C1 has write-with-response).
                {
                    var att = new byte[3 + payload.Length];
                    att[0] = 0x12; att[1] = (byte)(c1Handle & 0xFF); att[2] = (byte)(c1Handle >> 8);
                    Array.Copy(payload, 0, att, 3, payload.Length);
                    EnqueueAtt(att);
                }
                break;
            case HostFrameUnsubscribe:
                EnqueueAtt(new byte[] { 0x12, (byte)(c2CccdHandle & 0xFF), (byte)(c2CccdHandle >> 8), 0x00, 0x00 });
                break;
            case HostFrameDisconnect:
                // RENODE: the host (chip-tool) closed the BLE connection (it does this right after
                // ThreadNetworkEnable). Previously this was a no-op, so the DEVICE never learned of the
                // disconnect and kept its BLE link up forever -- and an open BLE connection monopolises the
                // emulated EFR32 multiprotocol radio, starving 15.4 RX so operational CASE over Thread can
                // never complete ("commissioning deadlock"). Tell the device to tear down its link via
                // LL_TERMINATE_IND so its radio is freed for Thread.
                this.Log(LogLevel.Warning, "BleCentralBridge: host closed BLE -> LL_TERMINATE_IND to device (free its radio for 15.4/Thread)");
                // Deliberate teardown: the rescan above must NOT fire, or the bridge would re-seize the
                // device's shared radio and recreate the commissioning deadlock this handler exists to avoid.
                suppressReconnect = true;
                EnqueueLlControl(new byte[] { 0x02, 0x13 }); // LL_TERMINATE_IND, reason 0x13 = remote user terminated
                // Give the terminate a few connection events to reach the device, THEN stop our own
                // connection-event loop. Otherwise the bridge keeps hopping + transmitting BLE connection
                // PDUs into the shared wireless medium forever; once the device has switched to 15.4 (Thread)
                // those stray BLE frames flood its RX and are dropped as "SYNC MISMATCH" (observed 600+ of
                // them), degrading the Thread link so merge/operational-CASE fail. Stopping the loop frees
                // the medium for 15.4. ScheduleAction marshals the state change onto the emulation thread
                // (this runs on the host-socket RX thread), consistent with ConnectionEvent().
                machine.ScheduleAction(TimeInterval.FromMicroseconds(ConnIntervalMicroseconds * 4), _ =>
                {
                    if(state == State.Connected)
                    {
                        state = State.Scanning;
                        this.Log(LogLevel.Warning, "BleCentralBridge: connection torn down -> stopped BLE connection events (medium free for 15.4)");
                    }
                }, "ble-central-terminate");
                break;
            default:
                this.Log(LogLevel.Warning, "BleCentralBridge: unknown host frame type 0x{0:X2}", type);
                break;
            }
        }

        // Enqueue an ATT PDU to the device from the host-socket thread -- thread-safe against the emulation
        // thread that drains txQueue in ConnectionEvent/ProcessDataPdu.
        private void EnqueueAtt(byte[] att)
        {
            lock(txLock)
            {
                var l2cap = new byte[4 + att.Length];
                l2cap[0] = (byte)(att.Length & 0xFF);
                l2cap[1] = (byte)(att.Length >> 8);
                l2cap[2] = 0x04; l2cap[3] = 0x00;
                Array.Copy(att, 0, l2cap, 4, att.Length);
                txQueue.Enqueue(new TxItem { Llid = 0x02, Payload = l2cap });
            }
        }

        private void SendHostFrame(byte type, byte[] payload)
        {
            var s = hostStream;
            if(s == null)
            {
                return;
            }
            try
            {
                var frame = new byte[3 + payload.Length];
                frame[0] = type;
                frame[1] = (byte)(payload.Length & 0xFF);
                frame[2] = (byte)(payload.Length >> 8);
                Array.Copy(payload, 0, frame, 3, payload.Length);
                lock(hostWriteLock)
                {
                    s.Write(frame, 0, frame.Length);
                }
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning, "BleCentralBridge: host socket write failed: {0}", e.Message);
            }
        }

        // Send an ATT PDU over L2CAP CID 0x0004 as a (single-fragment) LL Data PDU.
        private void SendAtt(byte[] att)
        {
            var l2cap = new byte[4 + att.Length];
            l2cap[0] = (byte)(att.Length & 0xFF);
            l2cap[1] = (byte)(att.Length >> 8);
            l2cap[2] = 0x04; l2cap[3] = 0x00; // CID = 0x0004 (ATT)
            Array.Copy(att, 0, l2cap, 4, att.Length);
            // For now ATT PDUs fit a single LL PDU (we negotiated a 251-octet data length): LLID 0x02 = start/complete.
            lock(txLock)
            {
                txQueue.Enqueue(new TxItem { Llid = 0x02, Payload = l2cap });
            }
        }

        private void SendAttMtuRequest()
        {
            if(state != State.Connected)
            {
                return;
            }
            this.Log(LogLevel.Warning, "BleCentralBridge: -> ATT_EXCHANGE_MTU_REQ ({0})", LocalAttMtu);
            SendAtt(new byte[] { 0x02, (byte)(LocalAttMtu & 0xFF), (byte)(LocalAttMtu >> 8) });
        }

        // Transmit a PHY frame onto the medium. Like the real radio (TransmitFrame), we must register the TX
        // in the shared InterferenceQueue BEFORE raising FrameSent -- the receiver's ReceiveFrame looks the
        // sender's TX up there for RSSI/timing and DROPS the frame ("TX was aborted") if it is absent. The
        // entry is removed shortly after, once the receiver has consumed it (delivery is same-virtual-time).
        private void SendFrame(byte[] pdu)
        {
            InterferenceQueue.Add(this, RadioPhyId.Phy_BLE_2_4GHz_GFSK, Channel, 0, pdu);
            FrameSent?.Invoke(this, pdu);
            machine.ScheduleAction(TimeInterval.FromMicroseconds(FrameOnAirMicroseconds), _ => InterferenceQueue.Remove(this), "ble-central-tx-done");
        }

        // Build a CONNECT_IND PDU framed for the medium: [AA=0x8E89BED6 LE][header(2)][InitA(6)][AdvA(6)][LLData(22)][CRC(3)].
        private byte[] BuildConnectInd()
        {
            var payload = new byte[6 + 6 + 22];
            // InitA (our random central address, little-endian)
            Array.Copy(centralAddress, 0, payload, 0, 6);
            // AdvA (device address, already little-endian as received)
            Array.Copy(deviceAddress, 0, payload, 6, 6);
            // LLData (22 bytes)
            var d = 12;
            // Access Address (4, LE) for the connection
            payload[d++] = (byte)(ConnectionAccessAddress & 0xFF);
            payload[d++] = (byte)((ConnectionAccessAddress >> 8) & 0xFF);
            payload[d++] = (byte)((ConnectionAccessAddress >> 16) & 0xFF);
            payload[d++] = (byte)((ConnectionAccessAddress >> 24) & 0xFF);
            // CRCInit (3)
            payload[d++] = 0x56; payload[d++] = 0x34; payload[d++] = 0x12;
            payload[d++] = WinSize;                 // WinSize (x1.25ms)
            payload[d++] = 0x00; payload[d++] = 0x00; // WinOffset (x1.25ms), LE
            payload[d++] = (byte)(ConnInterval & 0xFF); payload[d++] = (byte)(ConnInterval >> 8); // Interval, LE
            payload[d++] = 0x00; payload[d++] = 0x00; // Latency
            payload[d++] = (byte)(SupervisionTimeout & 0xFF); payload[d++] = (byte)(SupervisionTimeout >> 8); // Timeout, LE
            // Channel map (5): all 37 data channels enabled
            payload[d++] = 0xFF; payload[d++] = 0xFF; payload[d++] = 0xFF; payload[d++] = 0xFF; payload[d++] = 0x1F;
            payload[d++] = (byte)(HopIncrement & 0x1F); // Hop (5 bits) + SCA (upper 3 bits) = 0

            var header0 = (byte)(0x05 | 0x40 | (deviceAddressRandom ? 0x80 : 0x00)); // CONNECT_IND, TxAdd=random, RxAdd=dev
            var header1 = (byte)payload.Length;

            var pdu = new byte[4 + 2 + payload.Length + 3];
            // Advertising access address 0x8E89BED6, little-endian
            pdu[0] = 0xD6; pdu[1] = 0xBE; pdu[2] = 0x89; pdu[3] = 0x8E;
            pdu[4] = header0; pdu[5] = header1;
            Array.Copy(payload, 0, pdu, 6, payload.Length);
            // CRC (3) -- ignored by the model
            return pdu;
        }

        public int Channel { get; set; }

        public event Action<IRadio, byte[]> FrameSent;

        private enum State
        {
            Scanning,
            Connecting,
            Connected,
        }

        private sealed class TxItem
        {
            public int Llid;
            public byte[] Payload;
        }

        private State state = State.Scanning;
        private bool connectIndScheduled;
        private byte[] deviceAddress;
        private bool deviceAddressRandom;

        // Connection (master) state.
        private uint connEventCounter;
        private int lastUnmappedChannel;
        private int centralSn;              // sequence number of our next/last transmission
        private int centralNesn;            // next sequence number we expect from the peer
        private bool lastSentWasData;       // whether our last TX carried a queued (ackable) PDU
        private bool versionSent;

        // Link-supervision state. All of it is write-only on a healthy link except missStreak, which stays 0.
        private bool peerAnsweredLastEvent;  // set by ProcessDataPdu, consumed at the start of the next event
        private uint missStreak;             // CONFIRMED consecutive events with no reply
        private uint connectionGeneration;   // bumped per connection; stale timers check it and return
        private int connectAttempt;          // CONNECT_INDs sent since the peer last answered
        private bool suppressReconnect;      // link was torn down on purpose; do not re-connect
        private bool searchEngaged;          // a trial interval is in flight
        private bool searchEverEngaged;      // the search has run at least once on this connection
        private double currentIntervalUs = ConnIntervalMicroseconds;
        private double previousEventIntervalUs = ConnIntervalMicroseconds;
        private double anchorTxUs;           // virtual time (us) of the last transmission the peer heard
        private double lastTxUs;             // virtual time (us) of our most recent transmission
        private uint anchorEvent;            // connEventCounter index of that transmission
        private uint searchSweepBase;        // trials already tried on earlier connections
        private readonly System.Collections.Generic.Queue<TxItem> txQueue = new System.Collections.Generic.Queue<TxItem>();
        private static readonly byte[] EmptyPayload = new byte[0];

        // ATT/GATT client state.
        private byte[] l2capReassembly;
        private ushort attMtu = 23;
        private int serviceStart;
        private int serviceEnd;
        private ushort c1Handle;         // CHIPoBLE C1 (client writes) value handle
        private ushort c2Handle;         // CHIPoBLE C2 (indications) value handle
        private ushort c2CccdHandle;     // C2 Client Characteristic Configuration Descriptor
        private bool gattReady;
        private bool discoveryStarted;
        private const ushort LocalAttMtu = 247;

        // Our (central/initiator) random address, little-endian.
        private readonly byte[] centralAddress = new byte[] { 0x11, 0x22, 0x33, 0x44, 0x55, 0xC0 };

        // Delay from receiving the ADV_IND to sending CONNECT_IND, to land in the peripheral's post-ADV RX
        // window (device reaches RxSearch ~435us after ADV start and closes ~550us -- see model RSM timing).
        private const ulong ConnectIndDelayMicroseconds = 450;
        // How long our TX stays registered in the InterferenceQueue so the receiver can consume it.
        private const ulong FrameOnAirMicroseconds = 400;
        // Delay from sending CONNECT_IND to the first connection event, landing in the peripheral's transmit
        // window (CONNECT_IND air-time ~364us + transmitWindowDelay 1.25ms; WinSize=2 gives ~2.5ms slack).
        private const ulong Anchor0DelayMicroseconds = 1650;
        // Connection interval in microseconds (must equal ConnInterval * 1.25ms = 24 * 1.25ms = 30ms).
        private const ulong ConnIntervalMicroseconds = 30000;

        // Connection parameters advertised in the CONNECT_IND LLData.
        private const uint ConnectionAccessAddress = 0xAF9AB135; // a valid (non-adv) connection access address
        private const byte WinSize = 0x02;                       // transmit window size (x1.25ms)
        private const ushort ConnInterval = 0x0018;              // 24 * 1.25ms = 30ms
        private const ushort SupervisionTimeout = 0x0064;        // 100 * 10ms = 1s
        private const byte HopIncrement = 0x07;                  // 5..16

        // Supervision / CONNECT_IND-retry tuning. SupervisionEvents * 30 ms = 1.02 s, just past the 1 s
        // SupervisionTimeout the CONNECT_IND advertises. ConnectIndAim is microseconds after the END of the
        // ADV; SchedulerQuantumMicroseconds is TimeSourceBase.DefaultQuantum, which ScheduleAction adds.
        private const uint SupervisionEvents = 34;
        private const ulong ConnectIndAimMicroseconds = 275;
        private const ulong SchedulerQuantumMicroseconds = 100;
        private const int ConnectIndSweepStepMicroseconds = 60;
        private const int ConnectIndSweepCapMicroseconds = 300;
        // The gate: consecutive CONFIRMED misses before the interval search may engage at all. A healthy
        // link sits at missStreak == 0; the corpus's booting images never exceed 1.
        private const uint SearchEngageEvents = 4;
        private const double IntervalTrialStepMicroseconds = 30.0;
        private const double IntervalTrialCapMicroseconds = 1500.0;
        private const double MinimumEventDelayMicroseconds = 100.0;

        // Host CHIPoBLE socket (chip-tool's fake C1/C2 frame transport connects here).
        private const byte HostFrameConnect = 0x01;
        private const byte HostFrameDisconnect = 0x02;
        private const byte HostFrameWriteRequest = 0x03; // C1 write
        private const byte HostFrameSubscribe = 0x04;
        private const byte HostFrameUnsubscribe = 0x05;
        private const byte HostFrameIndication = 0x06;   // C2 indication
        private readonly int port;
        private TcpListener listener;
        private Thread socketThread;
        private volatile bool running;
        private NetworkStream hostStream;
        private readonly object txLock = new object();
        private readonly object hostWriteLock = new object();

        private readonly IMachine machine;
    }
}
