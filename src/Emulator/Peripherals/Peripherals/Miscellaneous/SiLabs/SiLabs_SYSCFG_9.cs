//
// Copyright (c) 2010-2025 Silicon Labs
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous.SiLabs
{
    public partial class SiLabs_SYSCFG_9
    {
        public SiLabs_SYSCFG_9(Machine machine) : base(machine)
        {
            Define_Registers();
        }

        partial void SYSCFG_Reset()
        {
            registers.Reset();
        }

        partial void RootSeswversion_Swversion_ValueProvider(ulong a)
        {
            rootseswversion_swversion_field.Value = 0xFFFFFF;
        }
    }
}