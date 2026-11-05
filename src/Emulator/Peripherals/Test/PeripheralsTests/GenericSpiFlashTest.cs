//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.SPI;

using NUnit.Framework;

namespace Antmicro.Renode.PeripheralsTests
{
    [TestFixture]
    public class GenericSpiFlashTest
    {
        [SetUp]
        public void SetUp()
        {
            memory = new MappedMemory(new Machine(), FlashSize);
            flash = new InspectableFlash(memory);
        }

        [TearDown]
        public void TearDown()
        {
            memory.Dispose();
        }

        [Test]
        public void ShouldOnlyClearBitsWhenProgramming()
        {
            ProgramThreeByte(0x200, 0xF0);
            ProgramThreeByte(0x200, 0x0F);
            Assert.AreEqual(0, ReadThreeByte(0x200));
            ProgramThreeByte(0x200, 0xFF);
            Assert.AreEqual(0, ReadThreeByte(0x200));
        }

        [Test]
        public void ShouldUseExtendedAddressForThreeByteProgramReadAndErase()
        {
            ProgramThreeByte(0x200, 0xA5);
            SetExtendedAddress(1);
            ProgramThreeByte(0x200, 0x5A);
            Assert.AreEqual(0x5A, ReadThreeByte(0x200));

            WriteEnable();
            Transaction(0x20, 0, 2, 0);
            Assert.AreEqual(0xFF, ReadThreeByte(0x200));

            SetExtendedAddress(0);
            Assert.AreEqual(0xA5, ReadThreeByte(0x200));
        }

        [Test]
        public void ShouldIgnoreExtendedAddressForFourByteCommandsAndMode()
        {
            ProgramThreeByte(0x200, 0xA5);
            SetExtendedAddress(1);
            ProgramThreeByte(0x200, 0x5A);

            Assert.AreEqual(0xA5, ReadFourByte(0x200));
            WriteEnable();
            Transaction(0x12, 0, 0, 2, 0, 0xF0);
            Assert.AreEqual(0xA0, ReadFourByte(0x200));
            Assert.AreEqual(0x5A, ReadThreeByte(0x200));

            Transaction(0xB7);
            Assert.AreEqual(0xA0, Transaction(0x03, 0, 0, 2, 0, 0)[5]);
        }

        [Test]
        public void ShouldIgnoreExtendedAddressForFourByteErase()
        {
            ProgramThreeByte(0x200, 0xA5);
            SetExtendedAddress(1);
            ProgramThreeByte(0x200, 0x5A);

            WriteEnable();
            Transaction(0x21, 0, 0, 2, 0);
            Assert.AreEqual(0xFF, ReadFourByte(0x200));
            Assert.AreEqual(0x5A, ReadThreeByte(0x200));
        }

        [Test]
        public void ShouldContinueReadingAcrossSegmentAndArrayBoundaries()
        {
            ProgramFourByte(0x00FFFFFF, 0x11);
            ProgramFourByte(0x01000000, 0x22);
            ProgramFourByte(0x01FFFFFF, 0x33);
            ProgramFourByte(0, 0x44);

            var lowerBoundary = Transaction(0x03, 0xFF, 0xFF, 0xFF, 0, 0);
            Assert.AreEqual(0x11, lowerBoundary[4]);
            Assert.AreEqual(0x22, lowerBoundary[5]);

            SetExtendedAddress(1);
            var upperBoundary = Transaction(0x03, 0xFF, 0xFF, 0xFF, 0, 0);
            Assert.AreEqual(0x33, upperBoundary[4]);
            Assert.AreEqual(0x44, upperBoundary[5]);
        }

        [Test]
        public void ShouldRejectCommandsStartingOutsideTheArray()
        {
            Assert.AreEqual(0, ReadFourByte(FlashSize));
            WriteEnable();
            try
            {
                foreach(var value in new byte[] { 0x12, 2, 0, 0, 0 })
                {
                    flash.Transmit(value);
                }
                Assert.IsFalse(flash.CanWriteNextByte(out var position));
                Assert.AreEqual(FlashSize, position);
                flash.Transmit(0x12);
            }
            finally
            {
                flash.FinishTransmission();
            }
            Assert.AreEqual(0xFF, ReadFourByte(0));
        }

        private void WriteEnable()
        {
            Transaction(0x06);
        }

        private void SetExtendedAddress(byte value)
        {
            WriteEnable();
            Transaction(0xC5, value);
        }

        private void ProgramThreeByte(int address, byte value)
        {
            WriteEnable();
            Transaction(0x02, (byte)(address >> 16), (byte)(address >> 8), (byte)address, value);
        }

        private void ProgramFourByte(int address, byte value)
        {
            WriteEnable();
            Transaction(0x12, (byte)(address >> 24), (byte)(address >> 16), (byte)(address >> 8), (byte)address, value);
        }

        private byte ReadThreeByte(int address)
        {
            return Transaction(0x03, (byte)(address >> 16), (byte)(address >> 8), (byte)address, 0)[4];
        }

        private byte ReadFourByte(int address)
        {
            return Transaction(0x13, (byte)(address >> 24), (byte)(address >> 16), (byte)(address >> 8), (byte)address, 0)[5];
        }

        private byte[] Transaction(params byte[] data)
        {
            var result = new byte[data.Length];
            try
            {
                for(var i = 0; i < data.Length; i++)
                {
                    result[i] = flash.Transmit(data[i]);
                }
                return result;
            }
            finally
            {
                flash.FinishTransmission();
            }
        }

        private MappedMemory memory;
        private InspectableFlash flash;

        private sealed class InspectableFlash : GenericSpiFlash
        {
            public InspectableFlash(MappedMemory underlyingMemory)
                : base(underlyingMemory, manufacturerId: 0xC2, memoryType: 0x20)
            {
            }

            public bool CanWriteNextByte(out long position)
            {
                return TryVerifyWriteToMemory(out position);
            }
        }

        private const int FlashSize = 0x2000000;
    }
}
