/*###############################################################################
#
# MODULE:              Definitions specific to a particular processor
#
# COMPONENT:          
#
# AUTHOR:             
#
# DESCRIPTION:
# Definitions for a specific processor, i.e. functions that can only be
# resolved by op codes
#              
#
# $HeadURL:  $
#
# REVISION:           
#
# LAST MODIFIED BY:   
#
# $LastChangedDate: $
#
# $Id:  $
#
###############################################################################
#
# This software is owned by NXP B.V. and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on NXP products [NXP Microcontrollers such as JN514x, JN516x, JN517x].
# You, and any third parties must reproduce the copyright and warranty notice 
# and any other legend of ownership on each  copy or partial copy of the software.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"  
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE. 
# 
# Copyright NXP B.V. 2015-2016. All rights reserved
#
###############################################################################*/

#ifndef  MICRO_SPECIFIC_INCLUDED
#define  MICRO_SPECIFIC_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

extern void (*isr_handlers[])(void);

#if (defined JENNIC_CHIP_JN5179_MRA1)
extern uint8 au8JENNICtoARMinterruptPriorityMapping[];
extern uint8 au8ARMtoJENNICinterruptPriorityMapping[];
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/** @{ Defined system call numbers */
#define SYSCALL_SEMIHOSTING   						0xAB

#define SEMIHOSTING_WRITE0      					0x04
#define SEMIHOSTING_READC       0x07
/** @} */

#define MICRO_INTERRUPT_EXCEPTION_OFFSET            16

// number of bits are defined by the hardware
#if (defined JENNIC_CHIP_JN5179_MRA1)
#define MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS     3   // was 4 at one point - waiting for a Bangalore confirm
#else
#define MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS     4
#endif
// this macro depends on the setting of the priority group in the NVIC, setting G=3 in this case
#define MICRO_INTERRUPT_MAX_PRIORITY                ((1 << MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS) - 1)
// half way
#define MICRO_INTERRUPT_MID_PRIORITY                (MICRO_INTERRUPT_MAX_PRIORITY/2)

// Priority levels in the arm are higher for lower values - B-Semi chips were the other way around
#define MICRO_INTERRUPT_ELEVATED_PRIORITY           (11)
#define MICRO_INTERRUPT_MEDIUM_PRIORITY             (12)

#if (defined JENNIC_CHIP_JN5179_MRA1)
#define MICRO_JENNIC_TO_ARM_PRIORITY_MAP(J)         ( J > 12 ? 0 : (au8JENNICtoARMinterruptPriorityMapping[J]))
#define MICRO_ARM_TO_JENNIC_PRIORITY_MAP(A)         (au8ARMtoJENNICinterruptPriorityMapping[A])
#else
#define MICRO_JENNIC_TO_ARM_PRIORITY_MAP(J)         ((16 - J) & 0xF)
#define MICRO_ARM_TO_JENNIC_PRIORITY_MAP(A)         ((16 - A) & 0xF)
#endif

// priority/sub priority register 8-bits wide
// read/write priority
#define MICRO_INTERRUPT_WRITE_PRIORITY_VALUE(W)     ((W) << (8 - MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS))
#define MICRO_INTERRUPT_READ_PRIORITY_VALUE(R)      ((R) >> (8 - MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS))
// read/write sub-priority
#define MICRO_INTERRUPT_SUBPRIORITY_MASK            ((1  << (8 - MICRO_INTERRUPT_NUMBER_OF_PRIORITY_BITS)) -1)
#define MICRO_INTERRUPT_SUBPRIORITY_VALUE(S)        ((S) & (MICRO_INTERRUPT_SUBPRIORITY_MASK))

// JN5172 interrupt mappings

#define MICRO_ISR_NUM_TMR2                          0
#define MICRO_ISR_NUM_TMR3                          1
#define MICRO_ISR_NUM_TMR4                          2
#define MICRO_ISR_NUM_TMR5                          3
#define MICRO_ISR_NUM_TMR6                          4
#define MICRO_ISR_NUM_TMR7                          5
#define MICRO_ISR_NUM_SYSCTRL                       6
#define MICRO_ISR_NUM_BBC                           7       // MAC
#define MICRO_ISR_NUM_AES                           8
#define MICRO_ISR_NUM_PHY                           9
#define MICRO_ISR_NUM_UART0                         10
#define MICRO_ISR_NUM_UART1                         11
#define MICRO_ISR_NUM_SPIS                          12
#define MICRO_ISR_NUM_SPIM                          13
#define MICRO_ISR_NUM_I2C                           14
#define MICRO_ISR_NUM_TMR0                          15
#define MICRO_ISR_NUM_TMR1                          16
#define MICRO_ISR_NUM_TMR8                          17     // TIMER_ADC
#define MICRO_ISR_NUM_ANPER                         18
#define MICRO_ISR_NUM_WDOG                          19
// number of interrupts
#define MICRO_ISR_NUM                               20
// mask
#define MICRO_ISR_EN_MASK                           ((1 << MICRO_ISR_NUM) - 1)

/* NOTE: Following are based on values above. Not all are defined for all
   chips. So if an error is raised by the MICRO_ISR_MASK_* macros below, look
   to see if the corresponding MICRO_ISR_NUM_* macro is defined above. If it
   isn't, that peripheral is not present on the defined chip */
#define MICRO_ISR_MASK_SYSCTRL  (1 << MICRO_ISR_NUM_SYSCTRL)
#define MICRO_ISR_MASK_BBC      (1 << MICRO_ISR_NUM_BBC)
#define MICRO_ISR_MASK_AES      (1 << MICRO_ISR_NUM_AES)
#define MICRO_ISR_MASK_PHY      (1 << MICRO_ISR_NUM_PHY)
#define MICRO_ISR_MASK_UART0    (1 << MICRO_ISR_NUM_UART0)
#define MICRO_ISR_MASK_UART1    (1 << MICRO_ISR_NUM_UART1)
#define MICRO_ISR_MASK_TMR0     (1 << MICRO_ISR_NUM_TMR0)
#define MICRO_ISR_MASK_TMR1     (1 << MICRO_ISR_NUM_TMR1)
#define MICRO_ISR_MASK_TMR2     (1 << MICRO_ISR_NUM_TMR2)
#define MICRO_ISR_MASK_TMR3     (1 << MICRO_ISR_NUM_TMR3)
#define MICRO_ISR_MASK_TMR4     (1 << MICRO_ISR_NUM_TMR4)
#define MICRO_ISR_MASK_I2C      (1 << MICRO_ISR_NUM_I2C)
#define MICRO_ISR_MASK_SPIM     (1 << MICRO_ISR_NUM_SPIM)
#define MICRO_ISR_MASK_SPIS     (1 << MICRO_ISR_NUM_SPIS)
#define MICRO_ISR_MASK_INTPER   (1 << MICRO_ISR_NUM_INTPER)
#define MICRO_ISR_MASK_ANPER    (1 << MICRO_ISR_NUM_ANPER)
#define MICRO_ISR_MASK_WDOG     (1 << MICRO_ISR_NUM_WDOG)

/* Handy macros for controlling interrupts, PIC, interrupt levels */

#define MICRO_ENABLE_INTERRUPTS()                                           \
        __asm volatile ("CPSIE I;" : : );

#define MICRO_DISABLE_INTERRUPTS()                                          \
        __asm volatile ("CPSID I;" : : );

extern void vAHI_InterruptSetPriority(uint32 u32Mask, uint8 u8Level);
extern uint8 u8AHI_InterruptGetPriority(uint32 u32InterruptNumber);
extern void vAHI_InterruptDisable(uint32 u32EnableMask);
extern void vAHI_TickTimerIntEnable(bool_t bIntEnable);
extern void vAHI_InterruptSetActivePriorityLevel(uint8 u8Level);
extern uint8 u8AHI_InterruptReadActivePriorityLevel(void);

#define MICRO_ENABLE_TICK_TIMER_INTERRUPT();                                \
        vAHI_TickTimerIntEnable(TRUE);

// use same value as Jennic/BA devices
#define MICRO_SET_PIC_ENABLE(A)                                             \
    vAHI_InterruptSetPriority(A, 8);

#define MICRO_CLEAR_PIC_ENABLE(A)                                           \
    vAHI_InterruptDisable(A)

#define MICRO_SET_PIC_PRIORITY_LEVEL(A,B)                                   \
    vAHI_InterruptSetPriority(A, B);

#define MICRO_GET_PIC_PRIORITY_LEVEL(A)                                     \
    u8AHI_InterruptGetPriority(A);

/* MRS Move to register from status */
/* MSR Move to status register */
#define MICRO_SET_ACTIVE_INT_LEVEL(A)                                       \
   ({                                                                       \
       register uint32 __u32interruptLevelStore  = A;                       \
        __asm volatile ("MSR BASEPRI, %[intlevelstore];"                      \
            :                                                               \
            :[intlevelstore] "r"(__u32interruptLevelStore)                  \
            );                                                              \
   })

#define MICRO_GET_ACTIVE_INT_LEVEL()                                        \
    ({                                                                      \
        register uint32 __u32interruptActiveLevel;                          \
        __asm volatile ("MRS %[activelevelstore], BASEPRI;"                   \
            :[activelevelstore] "=r"(__u32interruptActiveLevel)             \
            : );                                                            \
        __u32interruptActiveLevel;                                          \
    })

#define MICRO_SET_PRIMASK_LEVEL(A)                                          \
   ({                                                                       \
        register uint32 __u32primaskLevelStore  = A;                        \
        __asm volatile ("MSR PRIMASK, %[primasklevelstore];"                  \
            :                                                               \
            :[primasklevelstore] "r"(__u32primaskLevelStore)                \
            );                                                              \
   })

#define MICRO_GET_PRIMASK_LEVEL()                                           \
   ({                                                                       \
        register uint32 __u32primaskLevelStore;                             \
        __asm volatile ("MRS %[primasklevelstore], PRIMASK;"                  \
            :[primasklevelstore] "=r"(__u32primaskLevelStore)               \
            :                                                               \
            );                                                              \
        __u32primaskLevelStore;                                             \
   })

// read back PRIMASK status into u32Store variable then disable the interrupts
#define MICRO_DISABLE_AND_SAVE_INTERRUPTS(u32Store)                         \
   ({                                                                       \
        __asm volatile ("MRS %[primasklevelstore], PRIMASK;"                  \
                :[primasklevelstore] "=r"(u32Store)                         \
                :                                                           \
                );                                                          \
        __asm volatile ("CPSID I;" : : );                                     \
   })

#define MICRO_RESTORE_INTERRUPTS(u32Store)                                  \
   ({                                                                       \
        __asm volatile ("MSR PRIMASK, %[primasklevelstore];"                  \
                :                                                           \
                :[primasklevelstore] "r"(u32Store)                          \
                );                                                          \
   })

// using AAPCS the parameter (the stack frame) will map to r0
#define MICRO_GET_EXCEPTION_STACK_FRAME()                                   \
   {                                                                        \
        __asm volatile("MRS R0, MSP");                                        \
   }

// macro using privilege/non–privilege model
#define MICRO_GET_EXCEPTION_STACK_FRAME_PNPM()                              \
   ({                                                                       \
        __asm volatile("TST LR, #4");                                         \
        __asm volatile("ITE EQ");                                             \
        __asm volatile("MRSEQ R0, MSP");                                      \
        __asm volatile("MRSNE R0, PSP");                                      \
    })

#define FF1(__input)                                                        \
     ({ register uint32 __reverse, __result, __return;                      \
        __asm volatile ("RBIT %[reverse], %[input];"                          \
            : [reverse] "=r" (__reverse)                                    \
            : [input]  "r"  (__input)                                       \
                     );                                                     \
        __asm volatile ("CLZ %[result], %[reverse];"                          \
            : [result] "=r" (__result)                                      \
            : [reverse]  "r"  (__reverse)                                   \
            );                                                              \
        __return = ((__result == 32) ? 0 : __result+1);                  \
        __return; })

#define MICRO_GET_LX()                                                      \
   ({                                                                       \
        register uint32 __u32lxRegister;                                    \
        __asm volatile ("MOV %[lxRegister], R14;"                             \
            :[lxRegister] "=r"(__u32lxRegister)                             \
            :                                                               \
            );                                                              \
        __u32lxRegister;                                                    \
   })

#define MICRO_GET_STACK_LEVEL()                                             \
   ({                                                                       \
        register uint32 __u32stackRegister;                                 \
        __asm volatile ("MOV %[stackRegister], SP;"                           \
            :[stackRegister] "=r"(__u32stackRegister)                       \
            :                                                               \
            );                                                              \
        __u32stackRegister;                                                 \
   })

/* Interrupt Handler registration - only useful if you're putting the handlers
 * in RAM */

/* Location of isr_handlers is no longer at a known location, but we can link
   to it directly instead */
#define MICRO_SET_INT_HANDLER(INT, FUNC);                                   \
    isr_handlers[(MICRO_INTERRUPT_EXCEPTION_OFFSET + INT)] = (void *)(FUNC);

#define MICRO_GET_INT_HANDLER(INT)                                          \
    (isr_handlers[(MICRO_INTERRUPT_EXCEPTION_OFFSET + INT)])

/* Nested interrupt control */
#define MICRO_INT_STORAGE         tsMicroIntStorage sIntStorage
#define MICRO_INT_ENABLE_ONLY(A)  vMicroIntEnableOnly(&sIntStorage, A)
#define MICRO_INT_RESTORE_STATE() vMicroIntRestoreState(&sIntStorage)

/* Exception Handlers */
#define MICRO_ESR_NUM_RESETISR                    1
#define MICRO_ESR_NUM_NMI                         2
#define MICRO_ESR_NUM_HARDFAULT                   3
#define MICRO_ESR_NUM_MEMMANAGE                   4
#define MICRO_ESR_NUM_BUSFAULT                    5
#define MICRO_ESR_NUM_USGFAULT                    6
// 4 reserved handlers here
#define MICRO_ESR_NUM_SVCALL                      11
#define MICRO_ESR_NUM_DEBUGMON                    12
// 1 reserved handler here
#define MICRO_ESR_NUM_PENDSV                      14
#define MICRO_ESR_NUM_SYSTICK                     15

/* Location of exception_handlers is no longer at a known location, but we can link
   to it directly instead - only useful if you're putting the handlers
 * in RAM */
#define MICRO_SET_EXCEPTION_HANDLER(EXCEPTION, FUNC)                        \
    isr_handlers[EXCEPTION] = (void *)(FUNC);

#define MICRO_GET_EXCEPTION_HANDLER(INT)                                    \
    (isr_handlers[EXCEPTION])

/* NOP instruction */
#define MICRO_NOP()                                                         \
    {                                                                       \
        __asm volatile ("nop;");                                              \
    }

/* TRAP instruction */
#define MICRO_TRAP()                                                        \
    {                                                                       \
        __asm volatile("BKPT 0;");                                            \
    }

#define MICRO_JUMP_TO_ADDRESS(ADDRESS)                                      \
   ({                                                                       \
       register uint32 __u32programAddressStore = ADDRESS | 0x1;            \
       __asm volatile ("BLX %[programAddressStore];"                           \
           :                                                                \
           :[programAddressStore] "r"(__u32programAddressStore));           \
    })

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* Nested interrupt control */
typedef struct
{
    uint8 u8Level;
} tsMicroIntStorage;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void vAHI_InitialiseInterruptController(uint32 *pu32InterruptVectorTable);

/* Nested interrupt control */
PUBLIC void vMicroIntSetGlobalEnable(uint32 u32EnableMask);
PUBLIC void vMicroIntEnableOnly(tsMicroIntStorage *, uint32 u32EnableMask);
PUBLIC void vMicroIntRestoreState(tsMicroIntStorage *);
/* Default Exception Handler */
PUBLIC void vIntDefaultHandler(void);

PUBLIC void __attribute__((noinline)) vMicroSyscall(volatile uint32 u32SysCallNumber, ...);
PUBLIC void __attribute__((noinline)) vMicroSemihost(volatile uint32 u32SemihostNumber, ...);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* MICRO_SPECIFIC_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

