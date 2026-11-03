//
// Copyright (c) 2010-2023 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class EFR32xG2_I2CController : EFR32_GenericI2CController, IDoubleWordPeripheral
    {
        public EFR32xG2_I2CController(IMachine machine) : base(machine)
        {
            var map = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Registers.IpVersion, new DoubleWordRegister(this)
                    .WithTag("IPVERSION", 0, 32)
                },
                {(long)Registers.Enable, new DoubleWordRegister(this)
                    .WithTaggedFlag("ENABLE", 0)
                    .WithReservedBits(1, 31)
                },
                {(long)Registers.Control, GenerateControlRegister()},
                {(long)Registers.Command, GenerateCommandRegister()},
                {(long)Registers.State, GenerateStateRegister()},
                {(long)Registers.Status, GenerateStatusRegister()},
                {(long)Registers.ClockDivision, GenerateClockDivisionRegister()},
                {(long)Registers.SlaveAddress, GenerateSlaveAddressRegister()},
                {(long)Registers.SlaveAddressMask, GenerateSlaveAddressMaskRegister()},
                {(long)Registers.ReceiveBufferData, GenerateReceiveBufferDataRegister()},
                {(long)Registers.ReceiveBufferDoubleData, GenerateReceiveBufferDoubleDataRegister()},
                {(long)Registers.ReceiveBufferDataPeek, GenerateReceiveBufferDataPeekRegister()},
                {(long)Registers.ReceiveBufferDoubleDataPeek, GenerateReceiveBufferDoubleDataPeekRegister()},
                {(long)Registers.TransmitBufferData, GenerateTransmitBufferDataRegister()},
                {(long)Registers.TransmitBufferDoubleData, GenerateTransmitBufferDoubleDataRegister()},
                {(long)Registers.InterruptFlag, GenerateInterruptFlagRegister(maskByEnable: false)},
                {(long)Registers.InterruptEnable, GenerateInterruptEnableRegister()},
                {(long)Registers.InterruptFlagSet, GenerateSeries2InterruptFlagAliasRegister(set: true)},
                {(long)Registers.InterruptFlagClear, GenerateSeries2InterruptFlagAliasRegister(set: false)},
            };
            registers = new DoubleWordRegisterCollection(this, map);
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset & 0xFFF);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            machine.ClockSource.ExecuteInLock(() => WriteRegister(offset, value));
        }

        private void WriteRegister(long offset, uint value)
        {
            var address = offset & 0xFFF;
            var operation = offset & 0x3000;
            if(address == (long)Registers.InterruptFlag && operation != 0)
            {
                // IF uses flag-specific set/clear registers. A generic RMW on
                // the base W1C register would clear unrelated pending events.
                var pending = registers.Read(address);
                if(operation == 0x2000 || operation == 0x3000)
                {
                    registers.Write((long)Registers.InterruptFlagClear, operation == 0x3000 ? value & pending : value);
                }
                if(operation == 0x1000 || operation == 0x3000)
                {
                    registers.Write((long)Registers.InterruptFlagSet, operation == 0x3000 ? value & ~pending : value);
                }
                return;
            }
            switch(operation)
            {
            case 0x1000:
                value |= registers.Read(address);
                break;
            case 0x2000:
                value = registers.Read(address) & ~value;
                break;
            case 0x3000:
                value ^= registers.Read(address);
                break;
            }
            registers.Write(address, value);
        }

        public override void Reset()
        {
            base.Reset();
            registers.Reset();
        }

        private readonly DoubleWordRegisterCollection registers;

        private enum Registers
        {
            IpVersion = 0x0,
            Enable = 0x4,
            Control = 0x08,
            Command = 0x0C,
            State = 0x10,
            Status = 0x14,
            ClockDivision = 0x18,
            SlaveAddress = 0x1C,
            SlaveAddressMask = 0x20,
            ReceiveBufferData = 0x24,
            ReceiveBufferDoubleData = 0x28,
            ReceiveBufferDataPeek = 0x2C,
            ReceiveBufferDoubleDataPeek = 0x30,
            TransmitBufferData = 0x34,
            TransmitBufferDoubleData = 0x38,
            InterruptFlag = 0x3C,
            InterruptEnable = 0x40,
            InterruptFlagSet = 0x103C,
            InterruptFlagClear = 0x203C
        }
    }
}
