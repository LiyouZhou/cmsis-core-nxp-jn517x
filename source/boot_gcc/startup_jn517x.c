/****************************************************************************
 *
 * MODULE:             Application Hardware API - Init For JN517x
 *
 * COMPONENT:          $HeadURL: https://www.collabnet.nxp.com/svn/lprf_sware/Projects/Components/HardwareApi/Branches/JN517x-new/Source/AHI_Init.c $
 *
 * DATED:              $Date: 2014-01-29 10:37:57 +0000 (Wed, 29 Jan 2014) $
 *
 * STATUS:             $Id: AHI_Init.c 58928 2014-01-29 10:37:57Z nxp29789 $
 *
 * AUTHOR:             IL
 *
 * DESCRIPTION:
 * Provides an API for access to the hardware peripherals on the device, i.e.
 * timers, UARTs, SPI master, etc.
 *
 * REVISION:           $Revision: 58928 $
 *
 * LAST MODIFIED BY:   $Author: nxp29789 $
 *
 *
 ****************************************************************************
 *
 *  (c) Copyright 2012 NXP Semiconductors B.V.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include "jendefs.h"
//#include "MicroSpecific.h"
#include "AppHardwareApi.h"
#include "JN517x.h"

//#include "Debug.h"
/****************************************************************************/
/***        Imported Definitions                                          ***/
/****************************************************************************/

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define WEAK                       __attribute__ ((weak))
#define ALIAS(f)                   __attribute__ ((weak, alias (#f)))

/* TRAP instruction */
#define MICRO_TRAP()                                                        \
    {                                                                       \
        __asm volatile("BKPT 0;");                                            \
    }

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************
 * Forward declaration of the specific IRQ handlers. These are aliased
 * to the vIntDefaultHandler, which is a 'forever' loop. When the application
 * defines a handler (with the same name), this will automatically take
 * precedence over these weak definitions
 ****************************************************************************/
PUBLIC void vIntDefaultHandler(void);

extern void vResetISR(void)                            ALIAS(vIntDefaultHandler);
extern void vNMI_Handler(void)                         ALIAS(vIntDefaultHandler);
extern void vHardFault_Handler(void)                   ALIAS(vIntDefaultHandler);
extern void vMemManage_Handler(void)                   ALIAS(vIntDefaultHandler);
extern void vBusFault_Handler(void)                    ALIAS(vIntDefaultHandler);
extern void vUsageFault_Handler(void)                  ALIAS(vIntDefaultHandler);
extern void vSVCall_Handler(void)                      ALIAS(vIntDefaultHandler);
extern void vDebugMon_Handler(void)                    ALIAS(vIntDefaultHandler);
extern void vPendSV_Handler(void)                      ALIAS(vIntDefaultHandler);
extern void vAHI_ExceptionHandlerTickTimer(void)       ALIAS(vIntDefaultHandler);
extern void vReserved_Handler(void)                    ALIAS(vIntDefaultHandler);

/****************************************************************************
 * Forward declaration of the specific IRQ handlers. These are aliased
 * to the vIntDefaultHandler, which is a 'forever' loop. When the application
 * defines a handler (with the same name), this will automatically take
 * precedence over these weak definitions
 ****************************************************************************/
extern void vAHI_IntHandlerTimer2(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer3(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer4(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer5(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer6(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer7(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerSysCtrl(void)               ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerBbc_select(void)            ALIAS(vIntDefaultHandler);
extern void vACI_IntHandlerAes(void)                   ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerPhy_select(void)            ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerUart0(void)                 ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerUart1(void)                 ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerSpiSlave(void)              ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerSpiMaster(void)             ALIAS(vIntDefaultHandler);
extern void vAHI_I2CIntHandler(void)                   ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer0(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer1(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerTimer8(void)                ALIAS(vIntDefaultHandler);
extern void vAHI_IntHandlerAnalogPeriph(void)          ALIAS(vIntDefaultHandler);
extern void vAHI_ExceptionHandlerWatchDog(void)        ALIAS(vIntDefaultHandler);

/****************************************************************************
 * External declaration for the pointer to the stack top from the linker
 * script
 ****************************************************************************/
extern void __stack_top(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
__attribute__ ((section(".isr_handlers")))
PUBLIC void ( *isr_handlers[])(void) = {
    /* Core Level - CM3 */
    &__stack_top,                            /* The initial stack pointer */
    vResetISR,                               /* The reset handler         */
    vNMI_Handler,                            /* The NMI handler           */
    vHardFault_Handler,                      /* The hard fault handler    */
    vMemManage_Handler,                      /* The MPU fault handler     */
    vBusFault_Handler,                       /* The bus fault handler     */
    vUsageFault_Handler,                     /* The usage fault handler   */
    vReserved_Handler,                       /* Reserved                  */
    vReserved_Handler,                       /* Reserved                  */
    vReserved_Handler,                       /* Reserved                  */
    vReserved_Handler,                       /* Reserved                  */
    vSVCall_Handler,                         /* SVCall handler            */
    vDebugMon_Handler,                       /* Debug monitor handler     */
    vReserved_Handler,                       /* Reserved                  */
    vPendSV_Handler,                         /* The PendSV handler        */
    vAHI_ExceptionHandlerTickTimer,   		 /* The SysTick handler       */

    /* Chip Level - JN5172 */
    vAHI_IntHandlerTimer2,
    vAHI_IntHandlerTimer3,
    vAHI_IntHandlerTimer4,
    vAHI_IntHandlerTimer5,
    vAHI_IntHandlerTimer6,
    vAHI_IntHandlerTimer7,
    vAHI_IntHandlerSysCtrl,
    vAHI_IntHandlerBbc_select,
    vACI_IntHandlerAes,
    vAHI_IntHandlerPhy_select,
    vAHI_IntHandlerUart0,
    vAHI_IntHandlerUart1,
    vAHI_IntHandlerSpiSlave,
    vAHI_IntHandlerSpiMaster,
    vAHI_I2CIntHandler,
    vAHI_IntHandlerTimer0,
    vAHI_IntHandlerTimer1,
    vAHI_IntHandlerTimer8,
    vAHI_IntHandlerAnalogPeriph,
    vAHI_ExceptionHandlerWatchDog
};

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME:       vAHI_Init
 *
 * DESCRIPTION:
 * Initialisation function for AHI. In practice this does nothing as each
 * peripheral has an individual enable function
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
#if 0
PUBLIC uint32 u32AHI_Init(void)
{

    /* Set up NVIC */
    vAHI_InitialiseInterruptController((uint32 *)isr_handlers);

    /* Enable individual interrupts for chip in PIC */
    MICRO_SET_PIC_ENABLE(MICRO_ISR_EN_MASK);

//vDebug("MICRO_ISR_EN_MASK = 0x");
//vDebugHex(MICRO_ISR_EN_MASK, 8);
//vDebug("\r\n");
    /* Enable interrupts in CPU */
    MICRO_ENABLE_INTERRUPTS();

    return AHI_VERSION;
}
#endif

/****************************************************************************
 * NAME:        vIntDefaultHandler
 *
 * DESCRIPTION:
 * Processor ends up here if an unexpected interrupt occurs or a specific
 * handler is not present in the application code.
 *
 * RETURNS:
 * Never returns.
 *
 * NOTES:
 * Diagnostic code (such as UART print, trace logging) can be added here to
 * aid debugging.
 *
 ****************************************************************************/
__attribute__ ((section(".exceptions")))
WEAK PUBLIC void vIntDefaultHandler(void)
{
//    vDebugNonInt("vIntDefaultHandler ..... Ouch !\r\n");

    if(u32REG_Read(REG_DEBUG_HALTING_STATUS_AND_CONTROL) & 1)
    {
        // if C_DEBUGEN == 1, debugger connected halt program execution
        MICRO_TRAP();
    }

    while (1); // enter endless loop otherwise
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
