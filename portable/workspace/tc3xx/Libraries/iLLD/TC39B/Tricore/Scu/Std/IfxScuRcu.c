/**
 * \file IfxScuRcu.c
 * \brief SCU  basic functionality
 *
 * \version iLLD_1_0_1_15_0_1
 * \copyright Copyright (c) 2019 Infineon Technologies AG. All rights reserved.
 *
 *
 *                                 IMPORTANT NOTICE
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees. If and as long as no such terms
 * of use are agreed, use of this file is subject to following:
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer, must
 * be included in all copies of the Software, in whole or in part, and all
 * derivative works of the Software, unless such copies or derivative works are
 * solely in the form of machine-executable object code generated by a source
 * language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include "IfxScuRcu.h"

/******************************************************************************/
/*----------------------------------Macros------------------------------------*/
/******************************************************************************/

#define IFXSCURCU_PERFORM_RESET_DELAY (90000U)

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

IfxScuRcu_ResetCode IfxScuRcu_evaluateReset(void)
{
    Ifx_SCU_RSTCON      Rstcon;
    Ifx_SCU_RSTSTAT     RstStat;
    IfxScuRcu_ResetCode resetCode;
    resetCode.cpuSafeState = (((MODULE_SCU.RSTCON2.U >> IFX_SCU_RSTCON2_CSSX_OFF) & IFX_SCU_RSTCON2_CSSX_MSK) == IFX_SCU_RSTCON2_CSSX_MSK);
    resetCode.resetType    = IfxScuRcu_ResetType_undefined;
    resetCode.resetTrigger = IfxScuRcu_Trigger_undefined;
    resetCode.resetReason  = 0;

    RstStat.U              = MODULE_SCU.RSTSTAT.U;
    Rstcon.U               = MODULE_SCU.RSTCON.U;

    /* Evaluate the warm reset conditions first */
    if (RstStat.B.ESR0)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.ESR0;
        resetCode.resetTrigger = IfxScuRcu_Trigger_esr0;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.ESR1)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.ESR1;
        resetCode.resetTrigger = IfxScuRcu_Trigger_esr1;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.SMU)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.SMU;
        resetCode.resetTrigger = IfxScuRcu_Trigger_smu;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.SW)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.SW;
        resetCode.resetTrigger = IfxScuRcu_Trigger_sw;
        resetCode.resetReason  = MODULE_SCU.RSTCON2.B.USRINFO;
    }
    else if (RstStat.B.STM0)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM0;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm0;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.STM1)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM1;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm1;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.STM2)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM2;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm2;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.STM3)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM3;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm3;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.STM4)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM4;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm4;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.STM5)
    {
        resetCode.resetType    = (IfxScuRcu_ResetType)Rstcon.B.STM5;
        resetCode.resetTrigger = IfxScuRcu_Trigger_stm5;
        resetCode.resetReason  = 0;
    }

    else if (RstStat.B.CB0)
    {
        resetCode.resetType    = IfxScuRcu_ResetType_system;
        resetCode.resetTrigger = IfxScuRcu_Trigger_cb0;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.CB1)
    {
        resetCode.resetType    = IfxScuRcu_ResetType_debug;
        resetCode.resetTrigger = IfxScuRcu_Trigger_cb1;
        resetCode.resetReason  = 0;
    }
    else if (RstStat.B.CB3)
    {
        resetCode.resetType    = IfxScuRcu_ResetType_application;
        resetCode.resetTrigger = IfxScuRcu_Trigger_cb3;
        resetCode.resetReason  = 0;
    }
    else
    {
        /* Now evaluate for Cold reset conditions */
        if (RstStat.B.EVRC)
        {
            resetCode.resetType    = IfxScuRcu_ResetType_coldpoweron;
            resetCode.resetTrigger = IfxScuRcu_Trigger_evrc;
            resetCode.resetReason  = 0;
        }
        else if (RstStat.B.EVR33)
        {
            resetCode.resetType    = IfxScuRcu_ResetType_coldpoweron;
            resetCode.resetTrigger = IfxScuRcu_Trigger_evr33;
            resetCode.resetReason  = 0;
        }
        else if (RstStat.B.SWD)
        {
            resetCode.resetType    = IfxScuRcu_ResetType_coldpoweron;
            resetCode.resetTrigger = IfxScuRcu_Trigger_swd;
            resetCode.resetReason  = 0;
        }
        else if (RstStat.B.STBYR)
        {
            resetCode.resetType    = IfxScuRcu_ResetType_coldpoweron;
            resetCode.resetTrigger = IfxScuRcu_Trigger_stbyr;
            resetCode.resetReason  = 0;
        }
    }

/* Finally - Evaluate selectively for PORST */
    if (RstStat.B.PORST)
    {
        if (resetCode.resetType != IfxScuRcu_ResetType_coldpoweron)
        {
            resetCode.resetType    = IfxScuRcu_ResetType_warmpoweron;
            resetCode.resetTrigger = IfxScuRcu_Trigger_portst;
        }

        resetCode.resetReason = 0;
    }

    return resetCode;
}


void IfxScuRcu_performReset(IfxScuRcu_ResetType resetType, uint16 userResetInfo)
{
    uint32 index;
    uint16 password;

    password = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinitInline(password);

    /* Write the Reset Type - Application or System Reset */
    if (IfxScuRcu_ResetType_system == resetType)
    {
        MODULE_SCU.RSTCON.B.SW = 1; /* System Reset */
    }
    else
    {
        MODULE_SCU.RSTCON.B.SW = 2; /* Application Reset */
    }

    /* SWRSTCON and RSTCON2 are CPU endinit protected - clear end init protection */
    password = IfxScuWdt_getCpuWatchdogPasswordInline(&MODULE_SCU.WDTCPU[IfxCpu_getCoreIndex()]);

    IfxScuWdt_clearCpuEndinitInline(&MODULE_SCU.WDTCPU[IfxCpu_getCoreIndex()], password);
    /* Write the user DATA to reset evaluation */
    MODULE_SCU.RSTCON2.B.USRINFO = userResetInfo;

    /* software Reset can be performed by writing to Reset Request register  SWRSTCON */
    MODULE_SCU.SWRSTCON.B.SWRSTREQ = 1U;

    /* Add some delay for HW to reset */
    for (index = 0U; index < (uint32)IFXSCURCU_PERFORM_RESET_DELAY; index++)
    {}

    /*IfxScu_Wdt_enableSafetyEndinit() is not needed, as the micro would RESET */
    /* IfxScuWdt_setCpuEndinitInline() is not needed, as the micro would RESET */
}
