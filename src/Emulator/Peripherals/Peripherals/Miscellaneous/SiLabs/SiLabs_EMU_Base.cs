//
// Copyright (c) 2010-2025 Silicon Labs
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Debugging;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.IRQControllers;
using Range = Antmicro.Renode.Core.Range;

namespace Antmicro.Renode.Peripherals.Miscellaneous.SiLabs
{
    public class SiLabs_EMU_Base : IEmulationElement
    {
        public SiLabs_EMU_Base(IMachine machine, IBusPeripheral parent = null)
        {
            this.machine = machine;
            this.parent = parent;

            isWFIHookAdded = false;
            inDeepSleep = false;
            em2CoreRetentionEnabled = true;
            noRetain = new List<IPeripheral>();
            currentBusFaulters = new Dictionary<IPeripheral, List<BusFaulter>>();
            customPowerDownRanges = new Dictionary<IPeripheral, List<Range>>();
        }

        public void AddEnterDeepSleepHook(Action hook)
        {
            AddWFIHook();
            EnterDeepSleep += hook;
        }

        public void AddExitDeepSleepHook(Action hook)
        {
            AddWFIHook();
            ExitDeepSleep += hook;
        }

        public void UpdatePeripheralEnabledStatus(IPeripheral peripheral, bool enabled)
        {
            if (peripheral == null)
            {
                throw new Exception("UpdatePeripheralEnabledStatus: peripheral is null");
            }
            
            if(enabled) 
            {
                PowerUpPeripheral(peripheral);
            }
            else 
            {
                PowerDownPeripheral(peripheral);
            }
        }

        private Action<bool> WFIStateChanged(CortexM cpu)
        {
            return (WFI) =>
            {
                CpuWfiState[cpu] = WFI;
                
                IEnumerable<NVIC> nvic = machine.GetPeripheralsOfType<NVIC>();        
                if(nvic.All(n => n.DeepSleepEnabled))
                {
                    if(CpuWfiState.Values.All(wfi => wfi))
                    {
                        // all CPUs in WFI state
                        EnterDeepSleep?.Invoke();
                        inDeepSleep = true;
                    }
                    else if(!WFI && inDeepSleep)
                    {
                        // invoke the exit deep sleep hooks on the
                        // first CPU state change after deep sleep
                        ExitDeepSleep?.Invoke();
                        // The reset of the core must be done in the exit hook to
                        // keep the WFI state and the NVIC state for the wakeup sources
                        if(!em2CoreRetentionEnabled) {
                            ResetCortexMCores();
                        }
                        // This must be done after the reset of the cores because this modifies the NVIC state
                        PowerDownAllNoRetainPeripherals();
                        inDeepSleep = false;
                    }
                }
            };
        }

        public void AddWFIHook()
        {
            if(!isWFIHookAdded)
            {
                CpuWfiState.Keys.ToList().ForEach(cpu => cpu.AddHookAtWfiStateChange(WFIStateChanged(cpu)));
            }
            isWFIHookAdded = true;
        }

        public void SetNoRetainPeripheralsList(List<IPeripheral> noRetain)
        {
            AddWFIHook();
            this.noRetain = noRetain;
        }

        public List<IPeripheral> GetNoRetainPeripherals()
        {
            return noRetain;
        }

        private void PowerDownAllNoRetainPeripherals()
        {
            foreach(IPeripheral peripheral in noRetain)
            {
                string peripheralName = peripheral.GetName().Split('.')[1];
                var powerDownRegisters = peripheral.GetPowerDownRegisters();
                if (powerDownRegisters.Count == 0)
                {
                    this.Log(LogLevel.Noisy, "Powering down NoRetain peripheral {0}", peripheralName);
                    PowerDownPeripheral(peripheral);
                    peripheral.PowerDown();
                    continue;
                }

                this.Log(LogLevel.Noisy, "Powering down NoRetain peripheral {0} with partial retention", peripheralName);
                foreach(var register in powerDownRegisters)
                {
                    register.Reset();
                }

                // Update power down ranges for unretained registers
                customPowerDownRanges[peripheral] = CombineRanges(peripheral.AddRangesToPowerDown());
                PowerDownPeripheral(peripheral);
            }
        }

        // Merges adjacent ranges into a single range.
        private static List<Range> CombineRanges(List<Range> ranges)
        {
            if (ranges.Count == 0)
            {
                return ranges;
            }
            var result = new List<Range> { ranges[0] };
            for(var i = 1; i < ranges.Count; i++)
            {
                var last = result[result.Count - 1];
                if(last.CanBeExpandedBy(ranges[i]))
                {
                    result[result.Count - 1] = last.Expand(ranges[i]);
                }
                else
                {
                    result.Add(ranges[i]);
                }
            }
            return result;
        }

        // emulate power down by adding a BusFaulter peripheral at
        // the same address which will intercept all transactions
        // targeting the peripheral.  Note, for a multi core
        // implementation (i.e. Logan) there could possibly be
        // collisions between the BusFaulter registrations (added
        // for em2 emulation) and the Interceptor registrations
        // (added for bus isolation modeling).  my idea here is,
        // if necessary, add a API that allows the IMU to register
        // a callback.  the PowerDown / PowerUp methods can then
        // invoke the callback to coordinate any overlap between
        // the BusFaulters and the Interceptors.
        private void PowerDownPeripheral(IPeripheral peripheral)
        {
            IBusController sysbus = machine.GetSystemBus(parent);
            IRegisterablePeripheral<IBusPeripheral, BusRangeRegistration> sysbusRegistration = sysbus as IRegisterablePeripheral<IBusPeripheral, BusRangeRegistration>;

            foreach(CortexM cpu in CpuWfiState.Keys)
            {
                string cpuName = cpu.GetName().Split('.')[1];
                string cpuConditionString = "initiator == " + cpuName;

                // don't create a new busfaulter if it already exists
                if(currentBusFaulters.ContainsKey(peripheral))
                {
                    continue;
                }

                // foreach registered address for the specified
                // peripheral, register a temporary busFaulter
                // peripheral into the memory map that overlaps
                // the same address range
                //
                // it is registered with a condition "initiator == cpu0"
                // so that the original peripheral does not need to be
                // unregistered from the memory map.
                //
                // renode allows a peripheral registered with a condition
                // to overlap the same address range as an existing
                // peripheral, and the peripheral with the condition
                // will supersede the original peripheral for any
                // transaction that meets the condition (in this case,
                // if "cpu" is the initiator that targets this address
                // range)
                //
                IEnumerable<Range> rangesToFault;
                if(customPowerDownRanges.TryGetValue(peripheral, out var customRanges))
                {
                    rangesToFault = customRanges;
                }
                else
                {
                    rangesToFault = sysbus.GetRegistrationPoints(peripheral as IBusPeripheral)
                        .OfType<BusRangeRegistration>()
                        .Select(registration => registration.Range);
                }

                foreach(var range in rangesToFault)
                {
                    BusFaulter newBusFaulter = new BusFaulter(machine, cpu, cpu.Nvic, range.StartAddress);
                    BusRangeRegistration busFaulterRegistration = new BusRangeRegistration(new Range(range.StartAddress, range.Size), cpuConditionString);
                    sysbusRegistration.Register(newBusFaulter, busFaulterRegistration.WithInitiatorAndStateMask(cpu, StateMask.AllAccess) as BusRangeRegistration);

                    string busFaulterName = string.Format("busfaulter_{0}_0x{1:X}", cpuName, range.StartAddress);
                    machine.SetLocalName(newBusFaulter, busFaulterName);
                    this.Log(LogLevel.Noisy, "busFaulter peripheral {0} added at {1}", busFaulterName, busFaulterRegistration);

                    // keep track of all busFaulters added to the memory map
                    if(currentBusFaulters.ContainsKey(peripheral))
                    {
                        currentBusFaulters[peripheral].Add(newBusFaulter);
                    }
                    else
                    {
                        currentBusFaulters.Add(peripheral, new List<BusFaulter>() { newBusFaulter });
                    }
                }
            }
        }

        private void PowerUpPeripheral(IPeripheral peripheral)
        {
            IBusController sysbus = machine.GetSystemBus(parent);
            IRegisterablePeripheral<IBusPeripheral, BusRangeRegistration> sysbusRegistration = sysbus as IRegisterablePeripheral<IBusPeripheral, BusRangeRegistration>;

            if(currentBusFaulters.ContainsKey(peripheral))
            {
                // remove all busfaulters that overlap the peripheral in the memory map
                currentBusFaulters[peripheral].ForEach(x => sysbusRegistration.Unregister(x));
                currentBusFaulters.Remove(peripheral);
            }
        }

        private void ResetCortexMCores()
        {
            var cortexMCpus = machine.GetPeripheralsOfType<CortexM>().ToList();

            foreach(var cpu in cortexMCpus)
            {
                var vectorTableOffset = cpu.VectorTableOffset;
                var savedPendingByNvic = new Dictionary<NVIC, int[]>();

                cpu.CpuWaitSignal.Set(true);

                // Save pending interrupts before reset. On real hardware the IRQ line stays on
                // and the NVIC has IRQs pending when waking from EM2; we restore them after
                // reset to match that behavior. We read the NVIC's pending set via reflection
                // so we don't need to change the NVIC model.
                savedPendingByNvic[cpu.Nvic] = GetPendingIRQsFromNvic(cpu.Nvic);

                // Reset the NVIC and the core
                // This resets CPU internal states that are not retained during EM2
                cpu.Nvic.Reset();
                cpu.Reset();

                // Reset vector table to the original value to have the valid initial PC
                cpu.VectorTableOffset = vectorTableOffset;

                // Re-apply pending interrupts so wake-from-EM2 matches hardware (IRQ stays pending)
                foreach(var kv in savedPendingByNvic)
                {
                    var nvic = kv.Key;
                    foreach(var irq in kv.Value)
                    {
                        nvic.SetPendingIRQ(irq);
                    }
                }

                cpu.CpuWaitSignal.Set(false);
            }
        }

        /// <summary>
        /// Reads the set of pending IRQ numbers from an NVIC via reflection.
        /// Used to save/restore pending state across EM2 core reset without modifying the NVIC model.
        /// </summary>
        private static int[] GetPendingIRQsFromNvic(NVIC nvic)
        {
            var field = typeof(NVIC).GetField("pendingIRQs", BindingFlags.NonPublic | BindingFlags.Instance);
            if(field == null)
            {
                return Array.Empty<int>();
            }
            var pendingSet = field.GetValue(nvic) as System.Collections.Generic.ICollection<int>;
            if(pendingSet == null)
            {
                return Array.Empty<int>();
            }
            var snapshot = new int[pendingSet.Count];
            pendingSet.CopyTo(snapshot, 0);
            return snapshot;
        }

        // keys are reference to the CPU(s) values are WFI state of the CPU
        private IDictionary<CortexM, bool> CpuWfiState
        {
            get
            {
                if(_cpuWfiState == null)
                {
                    _cpuWfiState = machine.SystemBus.GetCPUs().Where(cpu => cpu is CortexM).ToDictionary(cpu => cpu as CortexM, wfiState => false);
                }
                return _cpuWfiState;
            }
        }

        protected event Action EnterDeepSleep;
        protected event Action ExitDeepSleep;
        public bool em2CoreRetentionEnabled;
        private readonly IMachine machine;
        private readonly IBusPeripheral parent;
        private bool isWFIHookAdded;
        private bool inDeepSleep;
        private List<IPeripheral> noRetain;
        private IDictionary<CortexM, bool> _cpuWfiState;

        // dictionary for keeping track of all busFaulters that
        // have been added to the memory map
        private Dictionary<IPeripheral, List<BusFaulter>> currentBusFaulters;

        // optional custom address ranges per peripheral for power-down bus fault;
        // when set, only these ranges are faulted instead of the full registration
        private Dictionary<IPeripheral, List<Range>> customPowerDownRanges;
 
         // dummy peripheral that always generates a bus fault on any read/write
        private class BusFaulter : IBytePeripheral, IWordPeripheral, IDoubleWordPeripheral
        {
            public BusFaulter(IMachine machine, CortexM cpu, NVIC nvic, ulong baseAddr)
            {
                this.machine = machine;
                this.cpu = cpu;
                this.nvic = nvic;
                this.baseAddr = baseAddr;
            }

            private void TriggerBusFault(long offset)
            {
                // set bus fault state bits

                // BFAR = address that caused fault
                uint bfar = checked((uint)baseAddr + (uint)offset);
                nvic.RegisterCollection.Write(0xD38, bfar);

                // CFSR update the BFSR subregister
                uint cfsr = cpu.FaultStatus;
                cfsr = (cfsr & 0xffff00ff) // preserve MMFSR and UFSR
                    | 1 << 15 // BFSR.BFARVALID = 1
                    | 1 << 9 // BFSR.PRECISERR = 1
                ; // all other BFSR bits zero
                cpu.FaultStatus = cfsr;

                // trigger bus fault
                //
                // note the SystemException enum in NVIC.cs
                // contains "BusFault = 5"
                // but the enum is private to the NVIC class, so
                // just using the hardcoded int value (5) here
                nvic.SetPendingIRQ(5);
            }

            public void Reset()
            {
            }
            public uint ReadDoubleWord(long offset)
            {
                TriggerBusFault(offset);
                return 0;
            }
            public void WriteDoubleWord(long offset, uint value)
            {
                TriggerBusFault(offset);
            }
            public ushort ReadWord(long offset)
            {
                TriggerBusFault(offset);
                return 0;
            }
            public void WriteWord(long offset, ushort value)
            {
                TriggerBusFault(offset);
            }
            public byte ReadByte(long offset)
            {
                TriggerBusFault(offset);
                return 0;
            }
            public void WriteByte(long offset, byte value)
            {
                TriggerBusFault(offset);
            }

            private readonly IMachine machine;
            private readonly CortexM cpu;
            private readonly NVIC nvic;
            private readonly ulong baseAddr;
        }
    }
}