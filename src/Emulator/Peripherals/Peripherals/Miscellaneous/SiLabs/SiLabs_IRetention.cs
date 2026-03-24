//
// Copyright (c) 2010-2025 Silicon Labs
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Range = Antmicro.Renode.Core.Range;

namespace Antmicro.Renode.Peripherals.Miscellaneous.SiLabs
{
    /// <summary>
    /// Optional: implement this to report a custom retention state. If not implemented, the entire peripheral is treated as not retained.
    /// </summary>
    public interface SiLabs_IRetention
    {
        bool Enabled { get; }
        void PowerDown();
    }

    /// <summary>
    /// Optional: implement this to report registers and ranges to power down. If not implemented, the entire peripheral is treated as not retained.
    /// </summary>
    public interface SiLabs_IPartialRetention
    {
        List<DoubleWordRegister> GetPowerDownRegisters();
        List<Range> AddRangesToPowerDown();
    }

    /// <summary>
    /// Extension methods for IPeripheral used by SiLabs peripherals.
    /// </summary>
    public static class SiLabs_IPeripheralExtensions
    {
        /// <summary>
        /// Returns the enabled state of the peripheral if it implements SiLabs_IRetention; otherwise returns false.
        /// </summary>
        public static bool Enabled(this IPeripheral @this)
        {
            return @this is SiLabs_IRetention peripheral ? peripheral.Enabled : false;
        }

        /// <summary>
        /// Power down the peripheral if it implements SiLabs_IRetention; otherwise reset it.
        /// Only used when GetPowerDownRegisters returns an empty list.
        /// </summary>
        public static void PowerDown(this IPeripheral @this)
        {
            if (@this is SiLabs_IRetention peripheral) peripheral.PowerDown(); else @this.Reset();
        }

        /// <summary>
        /// Returns a list of the registers to power down if it implements SiLabs_IPartialRetention; otherwise returns an empty list.
        /// Returning an empty list means that the entire peripheral is not retained when the system powers down.
        /// </summary>
        public static List<DoubleWordRegister> GetPowerDownRegisters(this IPeripheral @this)
        {
            return @this is SiLabs_IPartialRetention peripheral ? peripheral.GetPowerDownRegisters() : new List<DoubleWordRegister>();
        }

        /// <summary>
        /// Adds the ranges to the power down list if the peripheral implements SiLabs_IPartialRetention; otherwise returns an empty list.
        /// </summary>
        public static List<Range> AddRangesToPowerDown(this IPeripheral @this)
        {
            return @this is SiLabs_IPartialRetention peripheral ? peripheral.AddRangesToPowerDown() : new List<Range>();
        }
    }

    /// <summary>
    /// Helper methods for retention.
    /// </summary>
    public static class SiLabs_Retention
    {
                // Gets the system bus ranges for the registers of a peripheral.
        public static List<Range> GetRegisterRangesFromSystemBus(IPeripheral peripheral, List<ulong> registerAddresses)
        {
            var busPeripheral = peripheral as IBusPeripheral;
            var registrations = peripheral.GetMachine().GetSystemBus(busPeripheral).GetRegistrationPoints(busPeripheral).OfType<BusRangeRegistration>().ToList();
            return GetRegisterRangesFromRegistrations(registrations, registerAddresses);
        }

        // Gets the system bus ranges for the registration.
        public static List<Range> GetRegisterRangesFromRegistrations(List<BusRangeRegistration> registrations, List<ulong> registerAddresses)
        {
            var ranges = new List<Range>();
            foreach(var registration in registrations)
            {
                ulong baseAddress = registration.Range.StartAddress;
                if (registration.Range.Size != 0x4000)
                {
                    continue;
                }
                ranges.AddRange(registerAddresses.Select(registerAddress => new Range(baseAddress + registerAddress, 4)));
            }
            return ranges;
        }
    }
}
