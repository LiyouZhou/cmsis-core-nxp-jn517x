/*###############################################################################
#
# MODULE:             Application Hardware API
#
# COMPONENT:          
#
# AUTHOR:             
#
# DESCRIPTION:        Abstraction of the hardware peripherals available on the
#                     802.15.4 chip that are not used directly for 802.15.4,
#                     such as UARTs and timers.
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

#ifndef  AHI_H_INCLUDED
#define  AHI_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* System Controller RND Generator */
#define E_AHI_RND_SINGLE_SHOT           (TRUE)
#define E_AHI_RND_CONTINUOUS            (FALSE)
#define E_AHI_INTS_ENABLED              (TRUE)
#define E_AHI_INTS_DISABLED             (FALSE)

/* System Controller CPU clock control */
#define E_AHI_XTAL_OFF                  (TRUE)
#define E_AHI_XTAL_ON                   (FALSE)
#define E_AHI_XTAL_1MHZ                 (0x06)
#define E_AHI_XTAL_2MHZ                 (0x07)
#define E_AHI_XTAL_4MHZ                 (0x00)
#define E_AHI_XTAL_8MHZ                 (0x01)
#define E_AHI_XTAL_16MHZ                (0x02)
#define E_AHI_XTAL_32MHZ                (0x03)

/* System Controller crystal clock adjustment (use with vAHI_ClockXtalPull) */
#define E_AHI_XTAL_PULL_NONE            (0U)
#define E_AHI_XTAL_PULL_50PC            (1U)
#define E_AHI_XTAL_PULL_100PC           (3U)

/* System Controller 32Khz Clock Mode Control */
#define E_AHI_INTERNAL_RC               (0)
#define E_AHI_EXTERNAL_RC               (1)
#define E_AHI_XTAL                      (2)

/* Device enumerations */
#define E_AHI_WAKE_TIMER_0              (0)
#define E_AHI_WAKE_TIMER_1              (1)
#define E_AHI_AP_COMPARATOR_1           (0)
#define E_AHI_UART_0                    (0)
#define E_AHI_UART_1                    (1)
#define E_AHI_TIMER_0                   (0)
#define E_AHI_TIMER_1                   (1)
#define E_AHI_TIMER_2                   (2)
#define E_AHI_TIMER_3                   (3)
#define E_AHI_TIMER_4                   (4)
#define E_AHI_TIMER_5                   (5)
#define E_AHI_TIMER_6                   (6)
#define E_AHI_TIMER_7                   (7)
#define E_AHI_TIMER_8                   (8) // ADC Timer
#define E_AHI_DEVICE_WATCHDOG           (9)

/* Value enumerations: Pulse Counters */
#define E_AHI_PC_0                      (0)
#define E_AHI_PC_1                      (1)
#define E_AHI_PC_COMBINE_OFF            (0)
#define E_AHI_PC_COMBINE_ON0            (1)
#define E_AHI_PC_COMBINE_ON1            (2)

/* Value enumerations: VBO */
#define E_AHI_VBOREF_1V95               (0)
#define E_AHI_VBOREF_2V0                (1)
#define E_AHI_VBOREF_2V1                (2)
#define E_AHI_VBOREF_2V2                (3)
#define E_AHI_VBOREF_2V3                (4)
#define E_AHI_VBOREF_2V4                (5)
#define E_AHI_VBOREF_2V7                (6)
#define E_AHI_VBOREF_3V0                (7)

/* Value enumerations: Phy Controller (RF Power Settings) */
#define E_AHI_ZERO_dB                   (3)
#define E_AHI_MINUS_12_dB               (2)
#define E_AHI_MINUS_24_dB               (1)
#define E_AHI_MINUS_32_dB               (0)

/* Value enumerations: Wake Timer */
#define E_AHI_WAKE_TIMER_MASK_0         (1)
#define E_AHI_WAKE_TIMER_MASK_1         (2)

/* Value enumerations: Analogue Peripherals */
#define E_AHI_ADC_SRC_ADC5              (2)
#define E_AHI_ADC_SRC_ADC4              (3)
#define E_AHI_ADC_SRC_ADC3              (4)
#define E_AHI_ADC_SRC_ADC2              (5)
#define E_AHI_ADC_SRC_ADC1_VREF_EXT     (6)
#define E_AHI_ADC_SRC_ADC0_TEST1        (7)
#define E_AHI_ADC_SRC_TEMP              (8)
#define E_AHI_ADC_SRC_VOLT              (9)
#define E_AHI_ADC_SRC_TM_0              (10)
#define E_AHI_ADC_SRC_TM_1              (11)

#define E_AHI_ADC_DMA_SRC_ADC5_MASK              (1 << E_AHI_ADC_SRC_ADC5)
#define E_AHI_ADC_DMA_SRC_ADC4_MASK              (1 << E_AHI_ADC_SRC_ADC4)
#define E_AHI_ADC_DMA_SRC_ADC3_MASK              (1 << E_AHI_ADC_SRC_ADC3)
#define E_AHI_ADC_DMA_SRC_ADC2_MASK              (1 << E_AHI_ADC_SRC_ADC2)
#define E_AHI_ADC_DMA_SRC_ADC1_VREF_EXT_MASK     (1 << E_AHI_ADC_SRC_ADC1_VREF_EXT)
#define E_AHI_ADC_DMA_SRC_ADC0_TEST1_MASK        (1 << E_AHI_ADC_SRC_ADC0_TEST1)
#define E_AHI_ADC_DMA_SRC_TEMP_MASK              (1 << E_AHI_ADC_SRC_TEMP)
#define E_AHI_ADC_DMA_SRC_VOLT_MASK              (1 << E_AHI_ADC_SRC_VOLT)
#define E_AHI_ADC_DMA_SRC_TM_0_MASK              (1 << E_AHI_ADC_SRC_TM_0)
#define E_AHI_ADC_DMA_SRC_TM_1_MASK              (1 << E_AHI_ADC_SRC_TM_1)

#define E_AHI_AP_REGULATOR_ENABLE       (TRUE)
#define E_AHI_AP_REGULATOR_DISABLE      (FALSE)
#define E_AHI_AP_SAMPLE_2               (0)
#define E_AHI_AP_SAMPLE_4               (1)
#define E_AHI_AP_SAMPLE_6               (2)
#define E_AHI_AP_SAMPLE_8               (3)
#define E_AHI_AP_CLOCKDIV_2MHZ          (0)
#define E_AHI_AP_CLOCKDIV_1MHZ          (1)
#define E_AHI_AP_CLOCKDIV_500KHZ        (2)
#define E_AHI_AP_CLOCKDIV_250KHZ        (3)
#define E_AHI_AP_INPUT_RANGE_2          (TRUE)
#define E_AHI_AP_INPUT_RANGE_1          (FALSE)
#define E_AHI_AP_GAIN_2                 (TRUE)
#define E_AHI_AP_GAIN_1                 (FALSE)
#define E_AHI_AP_EXTREF                 (TRUE)
#define E_AHI_AP_INTREF                 (FALSE)
#define E_AHI_ADC_CONVERT_ENABLE        (TRUE)
#define E_AHI_ADC_CONVERT_DISABLE       (FALSE)
#define E_AHI_ADC_CONTINUOUS            (TRUE)
#define E_AHI_ADC_SINGLE_SHOT           (FALSE)
#define E_AHI_AP_CAPT_INT_STATUS_MASK   (0x01)
#define E_AHI_AP_ACC_INT_STATUS_MASK    (0x2)
#define E_AHI_ADC_ACC_SAMPLE_2          (0x00)
#define E_AHI_ADC_ACC_SAMPLE_4          (0x01)
#define E_AHI_ADC_ACC_SAMPLE_8          (0x02)
#define E_AHI_ADC_ACC_SAMPLE_16         (0x03)
#define E_AHI_AP_ADCACC_INT_ENABLE      (0x2U)
#define E_AHI_AP_CAPT_INT_ENABLE        (0x1U)
#define E_AHI_AP_INT_ENABLE             (TRUE)
#define E_AHI_AP_INT_DISABLE            (FALSE)
#define E_AHI_AP_BANDGAP_ENABLE         (TRUE)
#define E_AHI_AP_BANDGAP_DISABLE        (FALSE)
#define E_AHI_AP_SLOW_CONVERSIONS_ENABLE  (TRUE)
#define E_AHI_AP_SLOW_CONVERSIONS_DISABLE (FALSE)

/* Value enumerations: Comparator */
#define E_AHI_COMP_HYSTERESIS_0MV       (0)
#define E_AHI_COMP_HYSTERESIS_10MV      (1)
#define E_AHI_COMP_HYSTERESIS_20MV      (2)
#define E_AHI_COMP_HYSTERESIS_40MV      (3)
#define E_AHI_AP_COMPARATOR_MASK_1      (1)

#define E_AHI_COMP_SEL_EXT              (0x00)
#define E_AHI_COMP_SEL_BANDGAP          (0x02)
#define E_AHI_COMP_SEL_EXT_INVERSE      (0x81)
#define E_AHI_COMP_SEL_BANDGAP_INVERSE  (0x82)

/* Value enumerations: UART */
#define E_AHI_UART_RATE_4800            (0)
#define E_AHI_UART_RATE_9600            (1)
#define E_AHI_UART_RATE_19200           (2)
#define E_AHI_UART_RATE_38400           (3)
#define E_AHI_UART_RATE_76800           (4)
#define E_AHI_UART_RATE_115200          (5)
#define E_AHI_UART_WORD_LEN_5           (0)
#define E_AHI_UART_WORD_LEN_6           (1)
#define E_AHI_UART_WORD_LEN_7           (2)
#define E_AHI_UART_WORD_LEN_8           (3)
#define E_AHI_UART_FIFO_LEVEL_1         (0)
#define E_AHI_UART_FIFO_LEVEL_4         (1)
#define E_AHI_UART_FIFO_LEVEL_8         (2)
#define E_AHI_UART_FIFO_LEVEL_14        (3)
#define E_AHI_UART_LS_ERROR             (0x80)
#define E_AHI_UART_LS_TEMT              (0x40)
#define E_AHI_UART_LS_THRE              (0x20)
#define E_AHI_UART_LS_BI                (0x10)
#define E_AHI_UART_LS_FE                (0x08)
#define E_AHI_UART_LS_PE                (0x04)
#define E_AHI_UART_LS_OE                (0x02)
#define E_AHI_UART_LS_DR                (0x01)
#define E_AHI_UART_MS_CTS               (0x10)
#define E_AHI_UART_MS_DCTS              (0x01)
#define E_AHI_UART_INT_MODEM            (0)
#define E_AHI_UART_INT_TX               (1)
#define E_AHI_UART_INT_RXDATA           (2)
#define E_AHI_UART_INT_RXLINE           (3)
#define E_AHI_UART_INT_TIMEOUT          (6)
#define E_AHI_UART_TX_RESET             (TRUE)
#define E_AHI_UART_RX_RESET             (TRUE)
#define E_AHI_UART_TX_ENABLE            (FALSE)
#define E_AHI_UART_RX_ENABLE            (FALSE)
#define E_AHI_UART_EVEN_PARITY          (TRUE)
#define E_AHI_UART_ODD_PARITY           (FALSE)
#define E_AHI_UART_PARITY_ENABLE        (TRUE)
#define E_AHI_UART_PARITY_DISABLE       (FALSE)
#define E_AHI_UART_1_STOP_BIT           (TRUE)
#define E_AHI_UART_2_STOP_BITS          (FALSE)
#define E_AHI_UART_RTS_HIGH             (TRUE)
#define E_AHI_UART_RTS_LOW              (FALSE)
#define E_AHI_UART_FIFO_ARTS_LEVEL_8    (0)
#define E_AHI_UART_FIFO_ARTS_LEVEL_11   (1)
#define E_AHI_UART_FIFO_ARTS_LEVEL_13   (2)
#define E_AHI_UART_FIFO_ARTS_LEVEL_15   (3)

/* Value enumerations: SPI Master */
#define E_AHI_SPIM_MSB_FIRST            (FALSE)
#define E_AHI_SPIM_LSB_FIRST            (TRUE)
#define E_AHI_SPIM_TXPOS_EDGE           (FALSE)
#define E_AHI_SPIM_TXNEG_EDGE           (TRUE)
#define E_AHI_SPIM_RXPOS_EDGE           (FALSE)
#define E_AHI_SPIM_RXNEG_EDGE           (TRUE)
#define E_AHI_SPIM_INT_ENABLE           (TRUE)
#define E_AHI_SPIM_INT_DISABLE          (FALSE)
#define E_AHI_SPIM_AUTOSLAVE_ENBL       (TRUE)
#define E_AHI_SPIM_AUTOSLAVE_DSABL      (FALSE)
#define E_AHI_SPIM_SLAVE_ENBLE_0        (0x1)
#define E_AHI_SPIM_SLAVE_ENBLE_1        (0x2)
#define E_AHI_SPIM_SLAVE_ENBLE_2        (0x4)
#define E_AHI_SPIM_SLAVE_ENBLE_3        (0x8)
#define E_AHI_SPISEL_1                  (1)
#define E_AHI_SPISEL_2                  (2)

/* Value enumerations: Intelligent Peripheral */
#define E_AHI_IP_MAX_MSG_SIZE           (0x3E)
#define E_AHI_IP_TXPOS_EDGE             (FALSE)
#define E_AHI_IP_TXNEG_EDGE             (TRUE)
#define E_AHI_IP_RXPOS_EDGE             (FALSE)
#define E_AHI_IP_RXNEG_EDGE             (TRUE)
#define E_AHI_IP_BIG_ENDIAN             (TRUE)
#define E_AHI_IP_LITTLE_ENDIAN          (FALSE)

/* Value enumerations: Timer */
#define E_AHI_TIMER_INT_PERIOD          (1)
#define E_AHI_TIMER_INT_RISE            (2)

/* Value enumerations: Tick Timer */
#define E_AHI_TICK_TIMER_DISABLE        (0x00) /* Disable tick timer */
#define E_AHI_TICK_TIMER_RESTART        (0x01) /* Restart timer when match occurs */
#define E_AHI_TICK_TIMER_STOP           (0x02) /* Stop timer when match occurs */
#define E_AHI_TICK_TIMER_CONT           (0x03) /* Timer does not stop when match occurs */

/* Value enumerations: DIO */
#define E_AHI_DIO0_INT                  (0x00000001)
#define E_AHI_DIO1_INT                  (0x00000002)
#define E_AHI_DIO2_INT                  (0x00000004)
#define E_AHI_DIO3_INT                  (0x00000008)
#define E_AHI_DIO4_INT                  (0x00000010)
#define E_AHI_DIO5_INT                  (0x00000020)
#define E_AHI_DIO6_INT                  (0x00000040)
#define E_AHI_DIO7_INT                  (0x00000080)
#define E_AHI_DIO8_INT                  (0x00000100)
#define E_AHI_DIO9_INT                  (0x00000200)
#define E_AHI_DIO10_INT                 (0x00000400)
#define E_AHI_DIO11_INT                 (0x00000800)
#define E_AHI_DIO12_INT                 (0x00001000)
#define E_AHI_DIO13_INT                 (0x00002000)
#define E_AHI_DIO14_INT                 (0x00004000)
#define E_AHI_DIO15_INT                 (0x00008000)
#define E_AHI_DIO16_INT                 (0x00010000)
#define E_AHI_DIO17_INT                 (0x00020000)
#define E_AHI_DIO18_INT                 (0x00040000)
#define E_AHI_DIO19_INT                 (0x00080000)
#define E_AHI_DIO20_INT                 (0x00100000)
#define E_AHI_DIO21_INT                 (0x00200000)
#define E_AHI_DIO22_INT                 (0x00400000)
#define E_AHI_DIO23_INT                 (0x00800000)
#define E_AHI_DIO24_INT                 (0x01000000)

/* Interrupt and Status Item Bitmap Masks */
    /* System Control */
#define E_AHI_SYSCTRL_CKEM_MASK         (1 << E_AHI_SYSCTRL_CKES)
#define E_AHI_SYSCTRL_RNDEM_MASK        (1 << E_AHI_SYSCTRL_RNDES)
#define E_AHI_SYSCTRL_FEC_MASK          (1 << E_AHI_SYSCTRL_FEC)
#define E_AHI_SYSCTRL_COMP0_MASK        (1 << E_AHI_SYSCTRL_COMP0)
#define E_AHI_SYSCTRL_WK1_MASK          (1 << E_AHI_SYSCTRL_WK1)
#define E_AHI_SYSCTRL_WK0_MASK          (1 << E_AHI_SYSCTRL_WK0)
#define E_AHI_SYSCTRL_VREM_MASK         (1 << E_AHI_SYSCTRL_VRES)
#define E_AHI_SYSCTRL_VFEM_MASK         (1 << E_AHI_SYSCTRL_VFES)
#define E_AHI_SYSCTRL_PC1_MASK          (1 << E_AHI_SYSCTRL_PC1)
#define E_AHI_SYSCTRL_PC0_MASK          (1 << E_AHI_SYSCTRL_PC0)
    /* UART */
#define E_AHI_UART_TIMEOUT_MASK         (1 << E_AHI_UART_INT_TIMEOUT)
#define E_AHI_UART_RXLINE_MASK          (1 << E_AHI_UART_INT_RXLINE)
#define E_AHI_UART_RXDATA_MASK          (1 << E_AHI_UART_INT_RXDATA)
#define E_AHI_UART_TX_MASK              (1 << E_AHI_UART_INT_TX)
#define E_AHI_UART_MODEM_MASK           (1 << E_AHI_UART_INT_MODEM)
    /* Timer */
#define E_AHI_TIMER_RISE_MASK           E_AHI_TIMER_INT_RISE
#define E_AHI_TIMER_PERIOD_MASK         E_AHI_TIMER_INT_PERIOD
    /* Serial Interface Master */
#define E_AHI_SIM_RXACK_MASK            (1 << 7)
#define E_AHI_SIM_BUSY_MASK             (1 << 6)
#define E_AHI_SIM_AL_MASK               (1 << 5)
#define E_AHI_SIM_ICMD_MASK             (1 << 2)
#define E_AHI_SIM_TIP_MASK              (1 << 1)
#define E_AHI_SIM_INT_STATUS_MASK       (1 << 0)
    /* Serial Interface Slave */
#define E_AHI_SIS_ERROR_MASK            (1 << 4)
#define E_AHI_SIS_LAST_DATA_MASK        (1 << 3)
#define E_AHI_SIS_DATA_WA_MASK          (1 << 2)
#define E_AHI_SIS_DATA_RTKN_MASK        (1 << 1)
#define E_AHI_SIS_DATA_RR_MASK          (1 << 0)
    /* SPI Master */
#define E_AHI_SPIM_TX_MASK              (1 << 0)
    /* SPI Slave */
#define E_AHI_SPIS_STAT_TX_ABOVE_MASK   (1 << 3)
#define E_AHI_SPIS_STAT_RX_ABOVE_MASK   (1 << 2)
#define E_AHI_SPIS_STAT_TX_PENDING_MASK (1 << 1)
#define E_AHI_SPIS_STAT_RX_AVAIL_MASK   (1 << 0)
#define E_AHI_SPIS_INT_RX_TIMEOUT_MASK  (1 << 8)
#define E_AHI_SPIS_INT_TX_UNDER_MASK    (1 << 7)
#define E_AHI_SPIS_INT_RX_UNDER_MASK    (1 << 6)
#define E_AHI_SPIS_INT_TX_OVER_MASK     (1 << 5)
#define E_AHI_SPIS_INT_RX_OVER_MASK     (1 << 4)
#define E_AHI_SPIS_INT_TX_FALL_MASK     (1 << 3)
#define E_AHI_SPIS_INT_RX_CLIMB_MASK    (1 << 2)
#define E_AHI_SPIS_INT_TX_LAST_MASK     (1 << 1)
#define E_AHI_SPIS_INT_RX_FIRST_MASK    (1 << 0)
    /* Analogue Peripherals */
#define E_AHI_AP_INT_DC_LOW_MASK        (1 << 8)
#define E_AHI_AP_INT_DC_HIGH_MASK       (1 << 7)
#define E_AHI_AP_INT_DC_INRANGE_MASK    (1 << 6)
#define E_AHI_AP_INT_DC_OFFRANGE_MASK   (1 << 5)
#define E_AHI_AP_INT_DMA_OVER_MASK      (1 << 4)
#define E_AHI_AP_INT_DMA_END_MASK       (1 << 3)
#define E_AHI_AP_INT_DMA_MID_MASK       (1 << 2)
#define E_AHI_AP_INT_ADCACC_MASK        (1 << 1)
#define E_AHI_AP_INT_CAPT_MASK          (1 << 0)
    /* Infrared Remote Control Transmitter */
#define E_AHI_INFRARED_TX_MASK          (1 << 0)
    /* Antenna Diversity */
#define E_AHI_ANTDIV_STAT_ANT_MASK      (1 << 2)
#define E_AHI_ANTDIV_STAT_RX_MASK       (1 << 1)
#define E_AHI_ANTDIV_STAT_TX_MASK       (1 << 0)

/* Version number of module */
/*****************************************************************
[31:24] = Family : 4 (JN516x) / 3 (JN514x) / 0 (JN513x)
[23:16] = Part : 8 (JN5168) / 5 (JN5142) / 4 (JN5148) / 1 (JN5139)
[15: 0] = AHI Version (increment for each new release/patch)
******************************************************************/

#define AHI_VERSION                0x04080000

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#ifndef AHI_DEVICE_ENUM
#define AHI_DEVICE_ENUM

/* Device types, used to identify interrupt source */
typedef enum
{
    E_AHI_DEVICE_TIMER1           = 0,    /* Timer 1 */
    E_AHI_DEVICE_TIMER2           = 1,    /* Timer 2 */
    E_AHI_DEVICE_SYSCTRL          = 2,    /* System controller */
    E_AHI_DEVICE_BBC              = 3,    /* Baseband controller */
    E_AHI_DEVICE_AES              = 4,    /* Encryption engine */
    E_AHI_DEVICE_PHYCTRL          = 5,    /* Phy controller */
    E_AHI_DEVICE_UART0            = 6,    /* UART 0 */
    E_AHI_DEVICE_UART1            = 7,    /* UART 1 */
    E_AHI_DEVICE_TIMER0           = 8,    /* Timer 0 */
    E_AHI_DEVICE_SI               = 10,   /* Serial Interface (2 wire) */
    E_AHI_DEVICE_SPIM             = 11,   /* SPI master */
    E_AHI_DEVICE_SPIS             = 12,   /* SPI slave */
    E_AHI_DEVICE_ANALOGUE         = 13,   /* Analogue peripherals */
    E_AHI_DEVICE_TIMER3           = 14,   /* Timer 3 */
    E_AHI_DEVICE_TICK_TIMER       = 15,   /* Tick timer */
    E_AHI_DEVICE_TIMER4           = 16,   /* Timer 4 */
    E_AHI_DEVICE_FEC              = 17,   /* Flash and EEPROM Controller */
    E_AHI_DEVICE_INFRARED         = 19,   /* Infrared */
    E_AHI_DEVICE_TIMER5           = 20,   /* Timer 5 */
    E_AHI_DEVICE_TIMER6           = 21,   /* Timer 6 */
    E_AHI_DEVICE_TIMER7           = 22,   /* Timer 7 */
    E_AHI_DEVICE_TIMER8           = 23,   /* Timer 8 */
    E_AHI_DEVICE_SYSCTRL_GPIO_EXT = 24,   /* System controller 'virtual'
                                             device for additional GPIOs */
} teAHI_Device;

/* Individual interrupts. GPIO defined elswhere in this file */
typedef enum
{
    E_AHI_SYSCTRL_PC0   = 22,   /* Pulse Counter 0 */
    E_AHI_SYSCTRL_PC1   = 23,   /* Pulse Counter 1 */
    E_AHI_SYSCTRL_VFES  = 24,   /* VBO Falling  */
    E_AHI_SYSCTRL_VRES  = 25,   /* VBO Rising */
    E_AHI_SYSCTRL_WK0   = 26,   /* Wake timer 0 */
    E_AHI_SYSCTRL_WK1   = 27,   /* Wake timer 1 */
    E_AHI_SYSCTRL_COMP0 = 28,   /* Comparator 0 */
    E_AHI_SYSCTRL_FEC   = 29,   /* Flash and EEPROM Controller */
    E_AHI_SYSCTRL_RNDES = 30,   /* Random number generator */
    E_AHI_SYSCTRL_CKES  = 31    /* Clock change  */
} teAHI_Item;

#endif /* AHI_DEVICE_ENUM */

/* Application interrupt callback */
typedef void (*PR_HWINT_APPCALLBACK)(uint32 u32Device, uint32 u32ItemBitmap);

/* Spi Master Configuration */
typedef uint32 tSpiConfiguration;

/* Sleep Modes */
typedef enum
{
    E_AHI_SLEEP_OSCON_RAMON,     /*32Khz Osc on and Ram On*/
    E_AHI_SLEEP_OSCON_RAMOFF,    /*32Khz Osc on and Ram off*/
    E_AHI_SLEEP_OSCOFF_RAMON,    /*32Khz Osc off and Ram on*/
    E_AHI_SLEEP_OSCOFF_RAMOFF,   /*32Khz Osc off and Ram off*/
    E_AHI_SLEEP_DEEP,            /*Deep Sleep*/
} teAHI_SleepMode;


/*Flash Chips*/
typedef enum {
    E_FL_CHIP_ST_M25P10_A,
    E_FL_CHIP_SST_25VF010,
    E_FL_CHIP_ATMEL_AT25F512,
    E_FL_CHIP_ST_M25P40_A,
    E_FL_CHIP_ST_M25P05_A,
    E_FL_CHIP_ST_M25P20_A,
    E_FL_CHIP_CUSTOM,
    E_FL_CHIP_AUTO,
    E_FL_CHIP_INTERNAL
} teFlashChipType;


/* Type definitions for SPI Flash access functions */
typedef void    (*tpfvZSPIflashInit)(int iDivisor, uint8 u8SlaveSel);
typedef void    (*tpfvZSPIflashSetSlaveSel)(uint8 u8SlaveSel);
typedef void    (*tpfvZSPIflashWREN)(void);
typedef void    (*tpfvZSPIflashEWRSR)(void);
typedef uint8   (*tpfu8ZSPIflashRDSR)(void);
typedef uint16  (*tpfu16ZSPIflashRDID)(void);
typedef void    (*tpfvZSPIflashWRSR)(uint8 u8Data);
typedef void    (*tpfvZSPIflashPP)(uint32 u32Addr, uint16 u16Len, uint8* pu8Data);
typedef void    (*tpfvZSPIflashRead)(uint32 u32Addr,uint16 u16Len,uint8* pu8Data);
typedef void    (*tpfvZSPIflashBE)(void);
typedef void    (*tpfvZSPIflashSE)(uint8 u8Sector);

/* Table of SPI Flash access functions */
typedef struct tagSPIflashFncTable {
    uint32                      u32Signature;
    uint16                      u16FlashId;
    uint16                      u16Reserved;
    tpfvZSPIflashInit           vZSPIflashInit;
    tpfvZSPIflashSetSlaveSel    vZSPIflashSetSlaveSel;
    tpfvZSPIflashWREN           vZSPIflashWREN;
    tpfvZSPIflashEWRSR          vZSPIflashEWRSR;
    tpfu8ZSPIflashRDSR          u8ZSPIflashRDSR;
    tpfu16ZSPIflashRDID         u16ZSPIflashRDID;
    tpfvZSPIflashWRSR           vZSPIflashWRSR;
    tpfvZSPIflashPP             vZSPIflashPP;
    tpfvZSPIflashRead           vZSPIflashRead;
    tpfvZSPIflashBE             vZSPIflashBE;
    tpfvZSPIflashSE             vZSPIflashSE;
} tSPIflashFncTable;

/* Flash Powerdown */
typedef enum
{
    E_AHI_EE_POWERDOWN,
    E_AHI_FLASH_BANDGAP_ENABLE,
    E_AHI_FLASH_POWERDOWN
} AHI_FlashControl_e;

typedef struct {
    uint16      u16SlaveAddress;
    bool_t      boExtendAddr;
    uint8      *pu8DataBuffer;
    uint16      u16DataLength;
    bool_t      boMasterWriteData;
} tsAHII2CBCTentry;


#if defined __cplusplus
}
#endif

#endif  /* AHI_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
