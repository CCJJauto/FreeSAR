/*
 * Copyright (C) 2022 CCAuto.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* Standard includes. */


/*---------------------------------------------------------------------------*/
void vMMUTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vInternalProtectionTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vInstructionErrorTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vContextManagementTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vSystemBusAndPeripheralsTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vAssertionTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );
void vNonMaskableInterruptTrap( int iTrapIdentification ) __attribute__( ( longcall, weak ) );

/*-----------------------------------------------------------*/
#define IFX_CFG_CPU_TRAP_MME_HOOK(trapWatch)            vMMUTrap( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_IPE_HOOK(trapWatch)            vInternalProtectionTrap( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_IE_HOOK(trapWatch)             vInstructionErrorTrap( trapWatch.tId )
#define FX_CFG_CPU_TRAP_CME_HOOK(trapWatch)             vContextManagementTrap( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_BE_HOOK(trapWatch)             vSystemBusAndPeripheralsTrap( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_ASSERT_HOOK(trapWatch)         vAssertionTrap( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU0_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU1_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU2_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU3_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU4_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_SYSCALL_CPU5_HOOK(trapWatch)   prvTrapYield( trapWatch.tId )
#define IFX_CFG_CPU_TRAP_NMI_HOOK(trapWatch)            vNonMaskableInterruptTrap( trapWatch.tId )

