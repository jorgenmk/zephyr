
/****************************************************************************************************//**
 * @file     nrf9120_mlm1.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           nrf9120_MLM1 from Nordic Semiconductor.
 *
 * @version  V1
 * @date     8. March 2018
 *
 * @note     Generated with SVDConv V2.81d
 *           from CMSIS SVD File 'nrf9120_MLM1.svd' Version 1,
 *
 * @par      Copyright (c) 2016, Nordic Semiconductor ASA
 *           All rights reserved.
 *
 *           Redistribution and use in source and binary forms, with or without
 *           modification, are permitted provided that the following conditions are met:
 *
 *           * Redistributions of source code must retain the above copyright notice, this
 *           list of conditions and the following disclaimer.
 *
 *           * Redistributions in binary form must reproduce the above copyright notice,
 *           this list of conditions and the following disclaimer in the documentation
 *           and/or other materials provided with the distribution.
 *
 *           * Neither the name of Nordic Semiconductor ASA nor the names of its
 *           contributors may be used to endorse or promote products derived from
 *           this software without specific prior written permission.
 *
 *           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *           AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *           IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *           DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *           FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *           DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *           SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *           CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *           OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *           OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************************************/



/** @addtogroup Nordic Semiconductor
  * @{
  */

/** @addtogroup nrf9120_MLM1
  * @{
  */

#ifndef NRF9120_MLM1_H
#define NRF9120_MLM1_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M4 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn         = -12,              /*!<   4  Memory Management, MPU mismatch, including Access Violation
                                                         and No Match                                                          */
  BusFault_IRQn                 = -11,              /*!<   5  Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                         related Fault                                                         */
  UsageFault_IRQn               = -10,              /*!<   6  Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* -------------------  nrf9120_MLM1 Specific Interrupt Numbers  ------------------ */
  MWU_IRQn                      =   1,              /*!<   1  MWU                                                              */
  CLOCK_POWER_IRQn              =   5,              /*!<   5  CLOCK_POWER                                                      */
  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn=   8,         /*!<   8  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0                                   */
  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn=   9,         /*!<   9  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1                                   */
  GPIOTE_IRQn                   =  10,              /*!<  10  GPIOTE                                                           */
  SAADC_IRQn                    =  11,              /*!<  11  SAADC                                                            */
  TIMER0_IRQn                   =  12,              /*!<  12  TIMER0                                                           */
  TIMER1_IRQn                   =  13,              /*!<  13  TIMER1                                                           */
  TIMER2_IRQn                   =  14,              /*!<  14  TIMER2                                                           */
  RTC0_IRQn                     =  15,              /*!<  15  RTC0                                                             */
  NFCT_IRQn                     =  17,              /*!<  17  NFCT                                                             */
  WDT_IRQn                      =  18,              /*!<  18  WDT                                                              */
  QDEC_IRQn                     =  19,              /*!<  19  QDEC                                                             */
  COMP_LPCOMP_IRQn              =  20,              /*!<  20  COMP_LPCOMP                                                      */
  SWI0_EGU0_IRQn                =  21,              /*!<  21  SWI0_EGU0                                                        */
  PWM0_IRQn                     =  22,              /*!<  22  PWM0                                                             */
  PDM_IRQn                      =  23,              /*!<  23  PDM                                                              */
  I2S_IRQn                      =  24,              /*!<  24  I2S                                                              */
  IPC_IRQn                      =  25,              /*!<  25  IPC                                                              */
  EXTFLASH_IRQn                 =  26,              /*!<  26  EXTFLASH                                                         */
  RTC1_IRQn                     =  27,              /*!<  27  RTC1                                                             */
  USBD_IRQn                     =  31               /*!<  31  USBD                                                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M4 Processor and Core Peripherals---------------- */
#define __CM4_REV                 0x0001            /*!< Cortex-M4 Core Revision                                               */
#define __MPU_PRESENT                  1            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               3            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
#define __FPU_PRESENT                  1            /*!< FPU present or not                                                    */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm4.h"                               /*!< Cortex-M4 processor and core peripherals                              */
#include "system_nrf9120_mlm1.h"                    /*!< nrf9120_MLM1 System                                                   */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif


typedef struct {
  __IO uint32_t  TRACECLK;                          /*!< Pin number configuration for TRACECLK                                 */
  __IO uint32_t  TRACEDATA0;                        /*!< Pin number configuration for TRACEDATA[0] and SWO                     */
  __IO uint32_t  TRACEDATA1;                        /*!< Pin number configuration for TRACEDATA[1]                             */
  __IO uint32_t  TRACEDATA2;                        /*!< Pin number configuration for TRACEDATA[2]                             */
  __IO uint32_t  TRACEDATA3;                        /*!< Pin number configuration for TRACEDATA[3]                             */
} TAD_PSEL_Type;

typedef struct {
  __I  uint32_t  CONFIGID;                          /*!< Configuration identifier                                              */
  __I  uint32_t  DEVICEID[2];                       /*!< Description collection[0]: Device identifier                          */
  __I  uint32_t  PART;                              /*!< Part code                                                             */
  __I  uint32_t  VARIANT;                           /*!< Part Variant, Hardware version and Production configuration           */
  __I  uint32_t  PACKAGE;                           /*!< Package option                                                        */
  __I  uint32_t  RAM;                               /*!< RAM variant                                                           */
  __I  uint32_t  FLASH;                             /*!< Flash variant                                                         */
  __I  uint32_t  CODEPAGESIZE;                      /*!< Code memory page size                                                 */
  __I  uint32_t  CODESIZE;                          /*!< Code memory size                                                      */
  __I  uint32_t  DEVICETYPE;                        /*!< Device type                                                           */
} FICR_INFO_Type;

typedef struct {
  __I  uint32_t  ADDR;                              /*!< Description cluster[0]: Address                                       */
  __I  uint32_t  DATA;                              /*!< Description cluster[0]: Data                                          */
} FICR_TRIMCNF_Type;

typedef struct {
  __IO uint32_t  CPU;                               /*!< AHB bus master priority register for CPU                              */
  __IO uint32_t  EXTRAM[2];                         /*!< Description collection[0]: AHB bus master priority register
                                                         for external RAM slave port (EXTRAMs)                                 */
} AMLI_RAMPRI_Type;

typedef struct {
  __IO uint32_t  PROTECT[3];                        /*!< Description collection[0]: Control access for Master connected
                                                         to AMLI master port EXTPERI[0]                                        */
} DCNF_EXTPERI0_Type;

typedef struct {
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
} DCNF_EXTPERI1_Type;

typedef struct {
  __IO uint32_t  PROTECT[3];                        /*!< Description collection[0]: Control access from Master connected
                                                         to AMLI master port EXTRAM[0]                                         */
} DCNF_EXTRAM_Type;

typedef struct {
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
} DCNF_EXTPERI2_Type;

typedef struct {
  __IO uint32_t  PROTECT;                           /*!< Description cluster[0]: Control access from Master connected
                                                         to AMLI master port EXTCODE[0]                                        */
} DCNF_EXTCODE_Type;

typedef struct {
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
} DCNF_EXTPERI3_Type;

typedef struct {
  __IO uint32_t  WA;                                /*!< Description cluster[0]: Write access to region 0 detected             */
  __IO uint32_t  RA;                                /*!< Description cluster[0]: Read access to region 0 detected              */
} MWU_EVENTS_REGION_Type;

typedef struct {
  __IO uint32_t  WA;                                /*!< Description cluster[0]: Write access to peripheral region 0
                                                         detected                                                              */
  __IO uint32_t  RA;                                /*!< Description cluster[0]: Read access to peripheral region 0 detected   */
} MWU_EVENTS_PREGION_Type;

typedef struct {
  __IO uint32_t  WA;                                /*!< Description cluster[0]: Publish configuration for EVENTS_REGION[0].WA */
  __IO uint32_t  RA;                                /*!< Description cluster[0]: Publish configuration for EVENTS_REGION[0].RA */
} MWU_PUBLISH_REGION_Type;

typedef struct {
  __IO uint32_t  WA;                                /*!< Description cluster[0]: Publish configuration for EVENTS_PREGION[0].WA */
  __IO uint32_t  RA;                                /*!< Description cluster[0]: Publish configuration for EVENTS_PREGION[0].RA */
} MWU_PUBLISH_PREGION_Type;

typedef struct {
  __IO uint32_t  SUBSTATWA;                         /*!< Description cluster[0]: Source of event/interrupt in region
                                                         0, write access detected while corresponding subregion was enabled
                                                          for watching                                                         */
  __IO uint32_t  SUBSTATRA;                         /*!< Description cluster[0]: Source of event/interrupt in region
                                                         0, read access detected while corresponding subregion was enabled
                                                          for watching                                                         */
} MWU_PERREGION_Type;

typedef struct {
  __IO uint32_t  START;                             /*!< Description cluster[0]: Start address for region 0                    */
  __IO uint32_t  END;                               /*!< Description cluster[0]: End address of region 0                       */
  __I  uint32_t  RESERVED0[2];
} MWU_REGION_Type;

typedef struct {
  __I  uint32_t  START;                             /*!< Description cluster[0]: Reserved for future use                       */
  __I  uint32_t  END;                               /*!< Description cluster[0]: Reserved for future use                       */
  __IO uint32_t  SUBS;                              /*!< Description cluster[0]: Subregions of region 0                        */
  __I  uint32_t  RESERVED1;
} MWU_PREGION_Type;

typedef struct {
  __IO uint32_t  RESET;                             /*!< Reset LTE modem                                                       */
  __IO uint32_t  FORCEOFF;                          /*!< Force off power and clock in LTE modem                                */
  __IO uint32_t  FORCEON;                           /*!< Force on LTE modem                                                    */
} POWER_LTEMODEM_Type;

typedef struct {
  __IO uint32_t  SPIS1;                             /*!< AHB bus master priority register for SPIM1, SPIS1, TWIM1 and
                                                         TWIS1                                                                 */
  __IO uint32_t  SAADC;                             /*!< AHB bus master priority register for SAADC                            */
  __IO uint32_t  UARTE;                             /*!< AHB bus master priority register for UARTE                            */
  __IO uint32_t  SERIAL0;                           /*!< AHB bus master priority register for SPIM0, SPIS0, TWIM0 and
                                                         TWIS0                                                                 */
  __IO uint32_t  SERIAL2;                           /*!< AHB bus master priority register for SPIM2 and SPIS2                  */
  __IO uint32_t  NFCT;                              /*!< AHB bus master priority register for NFCT                             */
  __IO uint32_t  I2S;                               /*!< AHB bus master priority register for I2S                              */
  __IO uint32_t  PDM;                               /*!< AHB bus master priority register for PDM                              */
  __IO uint32_t  PWM;                               /*!< AHB bus master priority register for PWM0, PWM1 and PWM2              */
} PAMLI_RAMPRI_Type;

typedef struct {
  __IO uint32_t  RTS;                               /*!< Pin select for RTS signal                                             */
  __IO uint32_t  TXD;                               /*!< Pin select for TXD signal                                             */
  __IO uint32_t  CTS;                               /*!< Pin select for CTS signal                                             */
  __IO uint32_t  RXD;                               /*!< Pin select for RXD signal                                             */
} UARTE_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in receive buffer                             */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
} UARTE_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in transmit buffer                            */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
} UARTE_TXD_Type;

typedef struct {
  __IO uint32_t  SCK;                               /*!< Pin select for SCK                                                    */
  __IO uint32_t  MOSI;                              /*!< Pin select for MOSI signal                                            */
  __IO uint32_t  MISO;                              /*!< Pin select for MISO signal                                            */
} SPIM_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in receive buffer                             */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
  __IO uint32_t  LIST;                              /*!< EasyDMA list type                                                     */
} SPIM_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in transmit buffer                            */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
  __IO uint32_t  LIST;                              /*!< EasyDMA list type                                                     */
} SPIM_TXD_Type;

typedef struct {
  __IO uint32_t  SCK;                               /*!< Pin select for SCK                                                    */
  __IO uint32_t  MISO;                              /*!< Pin select for MISO signal                                            */
  __IO uint32_t  MOSI;                              /*!< Pin select for MOSI signal                                            */
  __IO uint32_t  CSN;                               /*!< Pin select for CSN signal                                             */
} SPIS_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< RXD data pointer                                                      */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in receive buffer                             */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes received in last granted transaction                  */
} SPIS_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< TXD data pointer                                                      */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in transmit buffer                            */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transmitted in last granted transaction               */
} SPIS_TXD_Type;

typedef struct {
  __IO uint32_t  SCL;                               /*!< Pin select for SCL signal                                             */
  __IO uint32_t  SDA;                               /*!< Pin select for SDA signal                                             */
} TWIM_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in receive buffer                             */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
  __IO uint32_t  LIST;                              /*!< EasyDMA list type                                                     */
} TWIM_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in transmit buffer                            */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last transaction                   */
  __IO uint32_t  LIST;                              /*!< EasyDMA list type                                                     */
} TWIM_TXD_Type;

typedef struct {
  __IO uint32_t  SCL;                               /*!< Pin select for SCL signal                                             */
  __IO uint32_t  SDA;                               /*!< Pin select for SDA signal                                             */
} TWIS_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< RXD Data pointer                                                      */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in RXD buffer                                 */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last RXD transaction               */
} TWIS_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< TXD Data pointer                                                      */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of bytes in TXD buffer                                 */
  __I  uint32_t  AMOUNT;                            /*!< Number of bytes transferred in the last TXD transaction               */
} TWIS_TXD_Type;

typedef struct {
  __IO uint32_t  LIMITH;                            /*!< Description cluster[0]: Last results is equal or above CH[0].LIMIT.HIGH */
  __IO uint32_t  LIMITL;                            /*!< Description cluster[0]: Last results is equal or below CH[0].LIMIT.LOW */
} SAADC_EVENTS_CH_Type;

typedef struct {
  __IO uint32_t  LIMITH;                            /*!< Description cluster[0]: Publish configuration for EVENTS_CH[0].LIMITH */
  __IO uint32_t  LIMITL;                            /*!< Description cluster[0]: Publish configuration for EVENTS_CH[0].LIMITL */
} SAADC_PUBLISH_CH_Type;

typedef struct {
  __IO uint32_t  PSELP;                             /*!< Description cluster[0]: Input positive pin selection for CH[0]        */
  __IO uint32_t  PSELN;                             /*!< Description cluster[0]: Input negative pin selection for CH[0]        */
  __IO uint32_t  CONFIG;                            /*!< Description cluster[0]: Input configuration for CH[0]                 */
  __IO uint32_t  LIMIT;                             /*!< Description cluster[0]: High/low limits for event monitoring
                                                         a channel                                                             */
} SAADC_CH_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Data pointer                                                          */
  __IO uint32_t  MAXCNT;                            /*!< Maximum number of buffer words to transfer                            */
  __I  uint32_t  AMOUNT;                            /*!< Number of buffer words transferred since last START                   */
} SAADC_RESULT_Type;

typedef struct {
  __O  uint32_t  EN;                                /*!< Description cluster[0]: Enable channel group 0                        */
  __O  uint32_t  DIS;                               /*!< Description cluster[0]: Disable channel group 0                       */
} DPPIC_TASKS_CHG_Type;

typedef struct {
  __IO uint32_t  EN;                                /*!< Description cluster[0]: Subscribe configuration for TASKS_CHG[0].EN   */
  __IO uint32_t  DIS;                               /*!< Description cluster[0]: Subscribe configuration for TASKS_CHG[0].DIS  */
} DPPIC_SUBSCRIBE_CHG_Type;

typedef struct {
  __IO uint32_t  RX;                                /*!< Result of last incoming frames                                        */
} NFCT_FRAMESTATUS_Type;

typedef struct {
  __IO uint32_t  FRAMECONFIG;                       /*!< Configuration of outgoing frames                                      */
  __IO uint32_t  AMOUNT;                            /*!< Size of outgoing frame                                                */
} NFCT_TXD_Type;

typedef struct {
  __IO uint32_t  FRAMECONFIG;                       /*!< Configuration of incoming frames                                      */
  __I  uint32_t  AMOUNT;                            /*!< Size of last incoming frame                                           */
} NFCT_RXD_Type;

typedef struct {
  __IO uint32_t  LED;                               /*!< Pin select for LED signal                                             */
  __IO uint32_t  A;                                 /*!< Pin select for A signal                                               */
  __IO uint32_t  B;                                 /*!< Pin select for B signal                                               */
} QDEC_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Beginning address in Data RAM of this
                                                         sequence                                                              */
  __IO uint32_t  CNT;                               /*!< Description cluster[0]: Amount of values (duty cycles) in this
                                                         sequence                                                              */
  __IO uint32_t  REFRESH;                           /*!< Description cluster[0]: Amount of additional PWM periods between
                                                         samples loaded into compare register                                  */
  __IO uint32_t  ENDDELAY;                          /*!< Description cluster[0]: Time added after the sequence                 */
  __I  uint32_t  RESERVED2[4];
} PWM_SEQ_Type;

typedef struct {
  __IO uint32_t  OUT[4];                            /*!< Description collection[0]: Output pin select for PWM channel
                                                         0                                                                     */
} PWM_PSEL_Type;

typedef struct {
  __IO uint32_t  HPPOLE;                            /*!< Settings for the high-pass filter                                     */
  __IO uint32_t  HPDISABLE;                         /*!< High pass filter disable                                              */
  __IO uint32_t  SOFTMUTE;                          /*!< Soft mute function                                                    */
  __IO uint32_t  SOFTCYCLES;                        /*!< Soft mute settings                                                    */
  __IO uint32_t  SAMPLEDELAY;                       /*!< Input Data Sampling with Number of PDM_CLK Clock Cycle Delay          */
} PDM_FILTER_Type;

typedef struct {
  __IO uint32_t  CLK;                               /*!< Pin number configuration for PDM CLK signal                           */
  __IO uint32_t  DIN;                               /*!< Pin number configuration for PDM DIN signal                           */
} PDM_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< RAM address pointer to write samples to with EasyDMA                  */
  __IO uint32_t  MAXCNT;                            /*!< Number of samples to allocate memory for in EasyDMA mode              */
  __I  uint32_t  AMOUNT;                            /*!< Number of samples transferred into Data RAM since last START
                                                         task                                                                  */
} PDM_SAMPLE_Type;

typedef struct {
  __IO uint32_t  MODE;                              /*!< I2S mode.                                                             */
  __IO uint32_t  RXEN;                              /*!< Reception (RX) enable.                                                */
  __IO uint32_t  TXEN;                              /*!< Transmission (TX) enable.                                             */
  __IO uint32_t  MCKEN;                             /*!< Master clock generator enable.                                        */
  __IO uint32_t  MCKFREQ;                           /*!< Master clock generator frequency.                                     */
  __IO uint32_t  RATIO;                             /*!< MCK / LRCK ratio.                                                     */
  __IO uint32_t  SWIDTH;                            /*!< Sample width.                                                         */
  __IO uint32_t  ALIGN;                             /*!< Alignment of sample within a frame.                                   */
  __IO uint32_t  FORMAT;                            /*!< Frame format.                                                         */
  __IO uint32_t  CHANNELS;                          /*!< Enable channels.                                                      */
} I2S_CONFIG_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Receive buffer RAM start address.                                     */
} I2S_RXD_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Transmit buffer RAM start address.                                    */
} I2S_TXD_Type;

typedef struct {
  __IO uint32_t  MAXCNT;                            /*!< Size of RXD and TXD buffers.                                          */
  __I  uint32_t  AMOUNT;                            /*!< Number of 32 bit words sent and received since the previous
                                                         END event. Nice to have only. TBD.                                    */
} I2S_RXTXD_Type;

typedef struct {
  __IO uint32_t  MCK;                               /*!< Pin select for MCK signal.                                            */
  __IO uint32_t  SCK;                               /*!< Pin select for SCK signal.                                            */
  __IO uint32_t  LRCK;                              /*!< Pin select for LRCK signal.                                           */
  __IO uint32_t  SDIN;                              /*!< Pin select for SDIN signal.                                           */
  __IO uint32_t  SDOUT;                             /*!< Pin select for SDOUT signal.                                          */
} I2S_PSEL_Type;

typedef struct {
  __IO uint32_t  SRC;                               /*!< Flash memory source address.                                          */
  __IO uint32_t  DST;                               /*!< RAM destination address.                                              */
  __IO uint32_t  CNT;                               /*!< Read transfer length.                                                 */
} QSPI_READ_Type;

typedef struct {
  __IO uint32_t  DST;                               /*!< Flash destination address.                                            */
  __IO uint32_t  SRC;                               /*!< RAM source address.                                                   */
  __IO uint32_t  CNT;                               /*!< Write transfer length.                                                */
} QSPI_WRITE_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Start address of Flash block to be erased.                            */
  __IO uint32_t  LEN;                               /*!< Size of block to be erased.                                           */
} QSPI_ERASE_Type;

typedef struct {
  __IO uint32_t  SCK;                               /*!< Pin select for serial clock SCK. Connected to SPI interface
                                                         pins of SPI-MEM-CTRL.                                                 */
  __IO uint32_t  CSN;                               /*!< Pin select for chip select signal CSN. Connected to SPI interface
                                                         pins of SPI-MEM-CTRL.                                                 */
  __IO uint32_t  CSN1;                              /*!< Pin select for chip select signal CSN1. Not implemented!.             */
  __IO uint32_t  IO0;                               /*!< Pin select for serial data MOSI/IO0. Connected to SPI interface
                                                         pins of SPI-MEM-CTRL                                                  */
  __IO uint32_t  IO1;                               /*!< Pin select for serial data MISO/IO1. Connected to SPI interface
                                                         pins of SPI-MEM-CTRL.                                                 */
  __IO uint32_t  IO2;                               /*!< Pin select for serial data IO2. Connected to SPI interface pins
                                                         of SPI-MEM-CTRL.                                                      */
  __IO uint32_t  IO3;                               /*!< Pin select for serial data IO3. Connected to SPI interface pins
                                                         of SPI-MEM-CTRL.                                                      */
} QSPI_PSEL_Type;

typedef struct {
  __I  uint32_t  EPIN[8];                           /*!< Description collection[0]: IN endpoint halted status. Can be
                                                         used as is as response to a GetStatus() request to endpoint.          */
  __I  uint32_t  RESERVED3;
  __I  uint32_t  EPOUT[8];                          /*!< Description collection[0]: OUT endpoint halted status. Can be
                                                         used as is as response to a GetStatus() request to endpoint.          */
} USBD_HALTED_Type;

typedef struct {
  __IO uint32_t  EPOUT[8];                          /*!< Description collection[0]: Amount of bytes received last in
                                                         the data stage of this OUT endpoint                                   */
  __IO uint32_t  ISOOUT;                            /*!< Description collection[0]: Amount of bytes received last on
                                                         this iso OUT data endpoint                                            */
} USBD_SIZE_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Data pointer                                  */
  __IO uint32_t  MAXCNT;                            /*!< Description cluster[0]: Maximum number of bytes to transfer           */
  __I  uint32_t  AMOUNT;                            /*!< Description cluster[0]: Number of bytes transferred in the last
                                                         transaction                                                           */
  __IO uint32_t  CONFIG;                            /*!< Description cluster[0]: Endpoint EasyDMA configuration                */
  __IO uint32_t  LIST;                              /*!< Description cluster[0]: EasyDMA list type                             */
} USBD_EPIN_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Data pointer                                  */
  __IO uint32_t  MAXCNT;                            /*!< Description cluster[0]: Maximum number of bytes to transfer           */
  __I  uint32_t  AMOUNT;                            /*!< Description cluster[0]: Number of bytes transferred in the last
                                                         transaction                                                           */
  __IO uint32_t  CONFIG;                            /*!< Description cluster[0]: Isochronous endpoint EasyDMA configuration    */
} USBD_ISOIN_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Data pointer                                  */
  __IO uint32_t  MAXCNT;                            /*!< Description cluster[0]: Maximum number of bytes to transfer           */
  __I  uint32_t  AMOUNT;                            /*!< Description cluster[0]: Number of bytes transferred in the last
                                                         transaction                                                           */
  __IO uint32_t  CONFIG;                            /*!< Description cluster[0]: Endpoint EasyDMA configuration                */
  __IO uint32_t  LIST;                              /*!< Description cluster[0]: EasyDMA list type                             */
} USBD_EPOUT_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Data pointer                                  */
  __IO uint32_t  MAXCNT;                            /*!< Description cluster[0]: Maximum number of bytes to transfer           */
  __I  uint32_t  AMOUNT;                            /*!< Description cluster[0]: Number of bytes transferred in the last
                                                         transaction                                                           */
  __IO uint32_t  CONFIG;                            /*!< Description cluster[0]: Isochronous endpoint EasyDMA configuration    */
} USBD_ISOOUT_Type;

typedef struct {
  __I  uint32_t  TESTTC;                            /*!< Description cluster[0]: Observe the TC output from the flash
                                                         macro                                                                 */
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __IO uint32_t  TESTDATA128[4];                    /*!< Description collection[0]: Direct flash test data word 0              */
  __I  uint32_t  TESTDATA128INC[4];                 /*!< Description collection[0]: Direct flash test data word 0. Post-increments
                                                         TESTCONTROLADDR                                                       */
  __I  uint32_t  TESTCMPINC;                        /*!< Description cluster[0]: Reads and compares last flash row to
                                                         TESTDATA128 content. Post-increments TESTCONTROLADDR.                 */
  __I  uint32_t  TESTNCMPINC;                       /*!< Description cluster[0]: Reads and compares last flash row to
                                                         complement of TESTDATA128 content. Post-increments TESTCONTROLADDR.   */
  __IO uint32_t  TESTCONTROLLINES;                  /*!< Description cluster[0]: Direct flash test, control lines. Refer
                                                         to the flash IP test documentation for more details.                  */
  __IO uint32_t  TESTCONTROLADDR;                   /*!< Description cluster[0]: Direct flash test control, address lines      */
  __IO uint32_t  TESTCOUNTERRELOAD;                 /*!< Description cluster[0]: Counter reload value, used when counting
                                                         from SE or YE rising edge to latching of data                         */
  __I  uint32_t  TESTCOUNTERRUNNING;                /*!< Description cluster[0]: Returns the state (counting or not)
                                                         of the counter                                                        */
  __IO uint32_t  TESTCONFIG;                        /*!< Description cluster[0]: Mode of operation of the counter              */
  __IO uint32_t  TESTTMVPPANAEN;                    /*!< Description cluster[0]: Controls analog signals for flash measurements */
  __IO uint32_t  UNUSED1[14];                       /*!< Description collection[0]: Unspecified                                */
} NVMC_TEST_Type;

typedef struct {
  __IO uint32_t  POWER;                             /*!< Description cluster[0]: RAM0 power control register                   */
  __O  uint32_t  POWERSET;                          /*!< Description cluster[0]: RAM0 power control set register               */
  __O  uint32_t  POWERCLR;                          /*!< Description cluster[0]: RAM0 power control clear register             */
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
} VMC_RAM_Type;

typedef struct {
  __IO uint32_t  OUT;                               /*!< Description cluster[0]: Pin 0 direct access output register           */
  __IO uint32_t  IN;                                /*!< Description cluster[0]: Pin 0 direct access input register            */
} GPIO_PIN_Type;


/* ================================================================================ */
/* ================                       TAD                      ================ */
/* ================================================================================ */


/**
  * @brief Trace and debug control (TAD)
  */

typedef struct {                                    /*!< TAD Structure                                                         */
  __I  uint32_t  RESERVED0[320];
  __IO uint32_t  ENABLE;                            /*!< Enable debug domain and aquire selected GPIOs                         */
  TAD_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
  __IO uint32_t  TRACEPORTSPEED;                    /*!< Clocking options for the Trace Port debug interface                   */
} NRF_TAD_Type;


/* ================================================================================ */
/* ================                      FICR                      ================ */
/* ================================================================================ */


/**
  * @brief Factory Information Configuration Registers (FICR)
  */

typedef struct {                                    /*!< FICR Structure                                                        */
  __I  uint32_t  CHIPCONF00;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF01;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF02;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF03;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF04;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF05;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  CHIPCONF06;                        /*!< Miscellaneous analog configuration. Will be set during production
                                                         test.                                                                 */
  __I  uint32_t  RESERVED0[53];
  __I  uint32_t  FLASHPWRUP0;                       /*!< Power-up pattern for flash                                            */
  __I  uint32_t  FLASHPWRUP1;                       /*!< Power-up pattern for flash                                            */
  __I  uint32_t  FLASHPWRUP2;                       /*!< Power-up pattern for flash                                            */
  __I  uint32_t  FLASHPWRUP3;                       /*!< Power-up pattern for flash                                            */

  union {
    __I  uint32_t  TESTSTATUS;                      /*!< Code memory test status. Will be set during production test.          */
    __I  uint32_t  RAM;                             /*!< RAM variant                                                           */
  };

  union {
    __I  uint32_t  CPTEST0;                         /*!< Result from Circuit Probe test                                        */
    __I  uint32_t  FLASH;                           /*!< Flash variant                                                         */
  };
  __I  uint32_t  CPTEST00;                          /*!< Result from Circuit Probe test                                        */
  __I  uint32_t  CPTEST1;                           /*!< Result from Circuit Probe test                                        */
  __I  uint32_t  CPTEST2;                           /*!< Result from Circuit Probe test                                        */
  __I  uint32_t  CPTEST3;                           /*!< Result from Circuit Probe test                                        */
  __I  uint32_t  CPTEST4;                           /*!< Result from Circuit Probe test                                        */
  __I  uint32_t  CPTEST5;                           /*!< Result from Circuit Probe test, flash repair information              */
  __I  uint32_t  CPTEST6;                           /*!< Result from Circuit Probe test, flash repair information              */
  __I  uint32_t  CPTEST7;                           /*!< Result from Circuit Probe test, flash repair information              */
  __I  uint32_t  CPTEST8;                           /*!< Result from Circuit Probe test, flash repair information              */
  __I  uint32_t  RESERVED1[53];
  FICR_INFO_Type INFO;                              /*!< Device info                                                           */
  __I  uint32_t  RESERVED2[53];
  FICR_TRIMCNF_Type TRIMCNF[32];                    /*!< Unspecified                                                           */
} NRF_FICR_Type;


/* ================================================================================ */
/* ================                      UICR                      ================ */
/* ================================================================================ */


/**
  * @brief User Information Configuration Registers (UICR)
  */

typedef struct {                                    /*!< UICR Structure                                                        */
  __IO uint32_t  APPROTECT;                         /*!< Access port protection                                                */
  __IO uint32_t  PSELRESET[2];                      /*!< Description collection[0]: Mapping of the nRESET function (see
                                                         POWER chapter for details)                                            */
  __IO uint32_t  EXTSUPPLY;                         /*!< Enable external circuitry to be supplied from VDD pin. Applicable
                                                         in "High voltage mode" only.                                          */
  __IO uint32_t  REGOUT0;                           /*!< GPIO reference voltage / external output supply voltage in "High
                                                         voltage mode".                                                        */
  __IO uint32_t  NFCPINS;                           /*!< Setting of pins dedicated to NFC functionality: NFC antenna
                                                         or GPIO                                                               */
  __IO uint32_t  XOSC32M;                           /*!< Oscillator Control                                                    */
  __I  uint32_t  RESERVED0[57];
  __IO uint32_t  NRFHW[32];                         /*!< Description collection[0]: Reserved for Nordic hardware design        */
  __I  uint32_t  RESERVED1[32];
  __IO uint32_t  NRFFW[32];                         /*!< Description collection[0]: Reserved for Nordic firmware design        */
  __I  uint32_t  RESERVED2[32];
  __IO uint32_t  CUSTOMER[32];                      /*!< Description collection[0]: Reserved for customer                      */
} NRF_UICR_Type;


/* ================================================================================ */
/* ================                 PMU_REGULATORS                 ================ */
/* ================================================================================ */


/**
  * @brief PMU_REGULATORS (PMU_REGULATORS)
  */

typedef struct {                                    /*!< PMU_REGULATORS Structure                                              */
  __I  uint32_t  UNUSED;                            /*!< Unused.                                                               */
} NRF_PMU_REGULATORS_Type;


/* ================================================================================ */
/* ================                 PMU_OSCILLATORS                ================ */
/* ================================================================================ */


/**
  * @brief PMU_OSCILLATORS (PMU_OSCILLATORS)
  */

typedef struct {                                    /*!< PMU_OSCILLATORS Structure                                             */
  __I  uint32_t  UNUSED;                            /*!< Unused.                                                               */
} NRF_PMU_OSCILLATORS_Type;


/* ================================================================================ */
/* ================                    RADIOLTE                    ================ */
/* ================================================================================ */


/**
  * @brief LTE-M Radio (RADIOLTE)
  */

typedef struct {                                    /*!< RADIOLTE Structure                                                    */
  __I  uint32_t  UNUSED;                            /*!< Unused.                                                               */
} NRF_RADIOLTE_Type;


/* ================================================================================ */
/* ================                      AMLI                      ================ */
/* ================================================================================ */


/**
  * @brief AHB Multi-Layer Interface (AMLI)
  */

typedef struct {                                    /*!< AMLI Structure                                                        */
  __I  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                          /*!< RAM configurable priority configuration structure                     */
} NRF_AMLI_Type;


/* ================================================================================ */
/* ================                      DCNF                      ================ */
/* ================================================================================ */


/**
  * @brief Domain Configuration Management (DCNF)
  */

typedef struct {                                    /*!< DCNF Structure                                                        */
  __I  uint32_t  RESERVED0[264];
  __I  uint32_t  CPUID;                             /*!< CPU number in the system                                              */
  __I  uint32_t  RESERVED1[7];
  DCNF_EXTPERI0_Type EXTPERI0;                      /*!< Unspecified                                                           */
  DCNF_EXTPERI1_Type EXTPERI1[5];                   /*!< Unspecified                                                           */
  DCNF_EXTRAM_Type EXTRAM;                          /*!< Unspecified                                                           */
  DCNF_EXTPERI2_Type EXTPERI2[5];                   /*!< Unspecified                                                           */
  DCNF_EXTCODE_Type EXTCODE[2];                     /*!< Unspecified                                                           */
  DCNF_EXTPERI3_Type EXTPERI3[6];                   /*!< Unspecified                                                           */
} NRF_DCNF_Type;


/* ================================================================================ */
/* ================                       MWU                      ================ */
/* ================================================================================ */


/**
  * @brief Memory Watch Unit (MWU)
  */

typedef struct {                                    /*!< MWU Structure                                                         */
  __I  uint32_t  RESERVED0[64];
  MWU_EVENTS_REGION_Type EVENTS_REGION[4];          /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED1[16];
  MWU_EVENTS_PREGION_Type EVENTS_PREGION[2];        /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED2[4];
  MWU_PUBLISH_REGION_Type PUBLISH_REGION[4];        /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED3[16];
  MWU_PUBLISH_PREGION_Type PUBLISH_PREGION[2];      /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED4[68];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[5];
  __IO uint32_t  NMIEN;                             /*!< Enable or disable non-maskable interrupt                              */
  __IO uint32_t  NMIENSET;                          /*!< Enable non-maskable interrupt                                         */
  __IO uint32_t  NMIENCLR;                          /*!< Disable non-maskable interrupt                                        */
  __I  uint32_t  RESERVED6[53];
  MWU_PERREGION_Type PERREGION[2];                  /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[64];
  __IO uint32_t  REGIONEN;                          /*!< Enable/disable regions watch                                          */
  __IO uint32_t  REGIONENSET;                       /*!< Enable regions watch                                                  */
  __IO uint32_t  REGIONENCLR;                       /*!< Disable regions watch                                                 */
  __I  uint32_t  RESERVED8[57];
  MWU_REGION_Type REGION[4];                        /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED9[32];
  MWU_PREGION_Type PREGION[2];                      /*!< Unspecified                                                           */
} NRF_MWU_Type;


/* ================================================================================ */
/* ================                   REGULATORS                   ================ */
/* ================================================================================ */


/**
  * @brief Voltage Regulators control (REGULATORS)
  */

typedef struct {                                    /*!< REGULATORS Structure                                                  */
  __I  uint32_t  RESERVED0[269];
  __IO uint32_t  DISABLEHPBOR;                      /*!< Enable or disable HPBOR                                               */
  __I  uint32_t  RESERVED1[50];
  __O  uint32_t  SYSTEMOFF;                         /*!< System OFF register                                                   */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  POFCON;                            /*!< Power failure comparator configuration                                */
  __I  uint32_t  RESERVED4[13];
  __IO uint32_t  FORCEOFFNVM;                       /*!< Force off NVM supply. See also the internal section in the NVMC
                                                         chapter.                                                              */
  __I  uint32_t  RESERVED5[11];
  __IO uint32_t  DCDCEN;                            /*!< DC/DC enable register for REG1 (DVDD_0V9).                            */
  __IO uint32_t  DCDCEN2;                           /*!< DC/DC enable register for REG2 (AVDD_1V3)                             */
  __IO uint32_t  DCDCENMIR;                         /*!< Enable mirroring of DCDCEN register into DCDCEN2 register.            */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  ATEPOWER;                          /*!< Analog test bus control for power modules                             */
  __I  uint32_t  RESERVED7[17];
  __IO uint32_t  DCDCCONFREG1;                      /*!< Configuration register for REG1 DCDC regulator (DVDD_0V9).            */
  __IO uint32_t  DCDCCONFREG2;                      /*!< Configuration register for REG2 DCDC regulator (AVDD_1V3).            */
  __I  uint32_t  RESERVED8[268];
  __IO uint32_t  UNUSED1;                           /*!< Unspecified                                                           */
  __IO uint32_t  UNUSED2;                           /*!< Unspecified                                                           */
  __IO uint32_t  FORCEMODEREG1;                     /*!< A backdoor register for manual selection of REG1 regulator mode
                                                         (DVDD_0V9)                                                            */
  __I  uint32_t  RESERVED9[3];
  __O  uint32_t  THRESHOLDREG1;                     /*!< Set threshold value of REG1 DCDC regulator                            */
  __O  uint32_t  ENFORCEMODEREG1;                   /*!< Enable forcing of power mode in REG1 voltage regulator                */
  __O  uint32_t  IGNOREDAPCMODE;                    /*!< Ignore DAPCP power mode                                               */
  __I  uint32_t  CURRENTMODEREG1;                   /*!< Currently used power mode of the REG1 voltage regulator               */
  __IO uint32_t  FORCEMODEREG2;                     /*!< A backdoor register for manual selection of REG2 regulator mode
                                                         (DVDD_1V3)                                                            */
  __I  uint32_t  RESERVED10;
  __O  uint32_t  THRESHOLDREG2;                     /*!< Set threshold value of REG2 DCDC regulator                            */
  __O  uint32_t  ENFORCEMODEREG2;                   /*!< Enable forcing of power mode in REG2 voltage regulator                */
  __IO uint32_t  DISABLEHSLIMITREG2;                /*!< Disable HSLIMIT for REG2 DCDC regulator                               */
  __I  uint32_t  CURRENTMODEREG2;                   /*!< Currently used power mode of the REG2 voltage regulator               */
  __IO uint32_t  DISABLEHSLIMITREG1;                /*!< Disable HSLIMIT for REG1 DCDC regulator                               */
  __I  uint32_t  RESERVED11[2];
  __IO uint32_t  LDOCONFREG1;                       /*!< Configuration register for REG1 LDO regulator (DVDD_0V9).             */
  __IO uint32_t  LDOTESTLOADREG1;                   /*!< Test load setting for REG1 LDO regulator (DVDD_0V9).                  */
  __IO uint32_t  LDOCONFREG2;                       /*!< Configuration register for REG2 LDO regulator (AVDD_1V3).             */
  __IO uint32_t  LDOTESTLOADREG2;                   /*!< Test load setting for REG2 LDO regulator (AVDD_1V3).                  */
  __IO uint32_t  ULPCONFREG1;                       /*!< Configuration register for REG1 ULP regulator (DVDD_0V9).             */
  __IO uint32_t  ULPCONFREG2;                       /*!< Configuration register for REG2 ULP regulator (AVDD_1V3).             */
  __IO uint32_t  VREG_HIGH_REG1;                    /*!< Increase voltage to AVDD_1V3 levels for REG1 LDO regulator (DVDD_0V9) */
  __IO uint32_t  ULP_HIGH_REG1;                     /*!< Increase voltage to AVDD_1V3 ULP levels for REG1 ULP regulator
                                                         (DVDD_0V9)                                                            */
  __I  uint32_t  RESERVED12[285];
  __IO uint32_t  DISABLESRAMSYSTEMOFFACK;           /*!< Enable or disable System OFF ack from SRAM                            */
  __IO uint32_t  DISABLENVMSYSTEMOFFACK;            /*!< Enable or disable System OFF ack from NVM                             */
} NRF_REGULATORS_Type;


/* ================================================================================ */
/* ================                   OSCILLATORS                  ================ */
/* ================================================================================ */


/**
  * @brief Oscillators control (OSCILLATORS)
  */

typedef struct {                                    /*!< OSCILLATORS Structure                                                 */
  __I  uint32_t  RESERVED0[353];
  __IO uint32_t  ATECLOCK;                          /*!< Analog test bus control for clock modules                             */
  __I  uint32_t  RESERVED1[15];
  __IO uint32_t  XOSC32MACCPL;                      /*!< HFXO bypass mode selection.                                           */
  __I  uint32_t  RESERVED2[14];
  __IO uint32_t  BYPASS32KI;                        /*!< Enable or disable bypass of LFCLK crystal oscillator with external
                                                         clock source                                                          */
  __IO uint32_t  POWERDOWN32KI;                     /*!< Enable/Disable forced power power down of LFXO                        */
  __IO uint32_t  HFQSTRT;                           /*!< TBD                                                                   */
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __IO uint32_t  HFCTRL;                            /*!< Temporary for CLOCK HFCTRL                                            */
  __IO uint32_t  BYPASS32M;                         /*!< Enable or disable bypass of HFCLK crystal oscillator with external
                                                         clock source                                                          */
  __IO uint32_t  XOSC32MCTRL;                       /*!< Reserved for future use                                               */
  __IO uint32_t  BBDLL;                             /*!< BBDLL override register in case BBDLL does not behave as expected     */
} NRF_OSCILLATORS_Type;


/* ================================================================================ */
/* ================                      CLOCK                     ================ */
/* ================================================================================ */


/**
  * @brief Clock Management (CLOCK)
  */

typedef struct {                                    /*!< CLOCK Structure                                                       */
  __O  uint32_t  TASKS_HFCLKSTART;                  /*!< Start HFCLK crystal oscillator                                        */
  __O  uint32_t  TASKS_HFCLKSTOP;                   /*!< Stop HFCLK crystal oscillator                                         */
  __O  uint32_t  TASKS_LFCLKSTART;                  /*!< Start LFCLK source                                                    */
  __O  uint32_t  TASKS_LFCLKSTOP;                   /*!< Stop LFCLK source                                                     */
  __I  uint32_t  RESERVED0[28];
  __IO uint32_t  SUBSCRIBE_HFCLKSTART;              /*!< Subscribe configuration for TASKS_HFCLKSTART                          */
  __IO uint32_t  SUBSCRIBE_HFCLKSTOP;               /*!< Subscribe configuration for TASKS_HFCLKSTOP                           */
  __IO uint32_t  SUBSCRIBE_LFCLKSTART;              /*!< Subscribe configuration for TASKS_LFCLKSTART                          */
  __IO uint32_t  SUBSCRIBE_LFCLKSTOP;               /*!< Subscribe configuration for TASKS_LFCLKSTOP                           */
  __I  uint32_t  RESERVED1[28];
  __IO uint32_t  EVENTS_HFCLKSTARTED;               /*!< HFCLK oscillator started                                              */
  __IO uint32_t  EVENTS_LFCLKSTARTED;               /*!< LFCLK started                                                         */
  __I  uint32_t  RESERVED2[30];
  __IO uint32_t  PUBLISH_HFCLKSTARTED;              /*!< Publish configuration for EVENTS_HFCLKSTARTED                         */
  __IO uint32_t  PUBLISH_LFCLKSTARTED;              /*!< Publish configuration for EVENTS_LFCLKSTARTED                         */
  __I  uint32_t  RESERVED3[94];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  INTPEND;                           /*!< Pending status of interrupt                                           */
  __I  uint32_t  RESERVED4[62];
  __I  uint32_t  HFCLKRUN;                          /*!< Status indicating that HFCLKSTART task has been triggered             */
  __I  uint32_t  HFCLKSTAT;                         /*!< Which HFCLK source is running                                         */
  __I  uint32_t  RESERVED5;
  __I  uint32_t  LFCLKRUN;                          /*!< Status indicating that LFCLKSTART task has been triggered             */
  __I  uint32_t  LFCLKSTAT;                         /*!< Which LFCLK source is running                                         */
  __I  uint32_t  LFCLKSRCCOPY;                      /*!< Copy of LFCLKSRC register, set when LFCLKSTART task was triggered
                                                         LFULP is not treat as a separate clock source on Alta MLM1,
                                                          where default clock source (0) is LFRC.                              */
  __I  uint32_t  RESERVED6[4];
  __IO uint32_t  HFCLKCURRFREQ;                     /*!< Current frequency of HFCLK                                            */
  __I  uint32_t  RESERVED7[56];
  __IO uint32_t  HFCLKSRC;                          /*!< Clock source for the HFCLK oscillator, and configuration of
                                                         XTAL oscillator. This register shall only be written while no
                                                          oscillation is present on XC1. Failing to do so may cause unexpected
                                                          behaviour.                                                           */
  __IO uint32_t  LFCLKSRC;                          /*!< Clock source for the LFCLK. LFCLKSTART task starts starts a
                                                         clock source selected with this register. LFULP is not treat
                                                          as a separate clock source on Alta MLM1, where default clock
                                                          source (0) is LFRC.                                                  */
  __I  uint32_t  RESERVED8[15];
  __IO uint32_t  HFCLKCTRL;                         /*!< HFCLK frequency configuration. The feature is not implemented
                                                         on Alta MLM1. Specification of this feature is under discussion.
                                                          See SAMPO-2263.                                                      */
  __IO uint32_t  TRACECONFIG;                       /*!< Clocking options for the Trace Port debug interface                   */
} NRF_CLOCK_Type;


/* ================================================================================ */
/* ================                      POWER                     ================ */
/* ================================================================================ */


/**
  * @brief Power Control (POWER)
  */

typedef struct {                                    /*!< POWER Structure                                                       */
  __I  uint32_t  RESERVED0[30];
  __O  uint32_t  TASKS_CONSTLAT;                    /*!< Enable constant latency mode. When going to sleep: CPU is clock
                                                         gated, but regulators, oscillators and clock tree is kept on.         */
  __O  uint32_t  TASKS_LOWPWR;                      /*!< Enable low power mode (variable latency) When going to sleep:
                                                         Oscillators are switched off (if nothing else is requesting
                                                          clocks). Regulators are switched off (if nothing else is requesting
                                                          power).                                                              */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  SUBSCRIBE_CONSTLAT;                /*!< Subscribe configuration for TASKS_CONSTLAT                            */
  __IO uint32_t  SUBSCRIBE_LOWPWR;                  /*!< Subscribe configuration for TASKS_LOWPWR                              */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  EVENTS_POFWARN;                    /*!< Power failure warning                                                 */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  EVENTS_SLEEPENTER;                 /*!< CPU entered WFI/WFE sleep                                             */
  __IO uint32_t  EVENTS_SLEEPEXIT;                  /*!< CPU exited WFI/WFE sleep                                              */
  __I  uint32_t  RESERVED4[27];
  __IO uint32_t  PUBLISH_POFWARN;                   /*!< Publish configuration for EVENTS_POFWARN                              */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  PUBLISH_SLEEPENTER;                /*!< Publish configuration for EVENTS_SLEEPENTER                           */
  __IO uint32_t  PUBLISH_SLEEPEXIT;                 /*!< Publish configuration for EVENTS_SLEEPEXIT                            */
  __I  uint32_t  RESERVED6[89];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[61];
  __IO uint32_t  RESETREAS;                         /*!< Reset reason                                                          */
  __I  uint32_t  RESERVED8[70];
  __IO uint32_t  GPREGRET[2];                       /*!< Description collection[0]: General purpose retention register         */
  __I  uint32_t  RESERVED9[59];
  POWER_LTEMODEM_Type LTEMODEM;                     /*!< LTE Modem                                                             */
} NRF_POWER_Type;


/* ================================================================================ */
/* ================                      PAMLI                     ================ */
/* ================================================================================ */


/**
  * @brief Peripheral AHB Multi-Layer Interface (PAMLI)
  */

typedef struct {                                    /*!< PAMLI Structure                                                       */
  __I  uint32_t  RESERVED0[360];
  __IO uint32_t  BRIDGETYPE;                        /*!< Select AHB2AHB Sync Bridge Type                                       */
  __I  uint32_t  RESERVED1[535];
  PAMLI_RAMPRI_Type RAMPRI;                         /*!< RAM configurable priority configuration structure                     */
} NRF_PAMLI_Type;


/* ================================================================================ */
/* ================                      UARTE                     ================ */
/* ================================================================================ */


/**
  * @brief UART with EasyDMA 0 (UARTE)
  */

typedef struct {                                    /*!< UARTE Structure                                                       */
  __O  uint32_t  TASKS_STARTRX;                     /*!< Start UART receiver                                                   */
  __O  uint32_t  TASKS_STOPRX;                      /*!< Stop UART receiver                                                    */
  __O  uint32_t  TASKS_STARTTX;                     /*!< Start UART transmitter                                                */
  __O  uint32_t  TASKS_STOPTX;                      /*!< Stop UART transmitter                                                 */
  __I  uint32_t  RESERVED0[3];
  __O  uint32_t  TASKS_SUSPEND;                     /*!< Suspend UART transaction                                              */
  __O  uint32_t  TASKS_RESUME;                      /*!< Resume UART transaction                                               */
  __I  uint32_t  RESERVED1[2];
  __O  uint32_t  TASKS_FLUSHRX;                     /*!< Flush RX FIFO into RX buffer                                          */
  __I  uint32_t  RESERVED2[20];
  __IO uint32_t  SUBSCRIBE_STARTRX;                 /*!< Subscribe configuration for TASKS_STARTRX                             */
  __IO uint32_t  SUBSCRIBE_STOPRX;                  /*!< Subscribe configuration for TASKS_STOPRX                              */
  __IO uint32_t  SUBSCRIBE_STARTTX;                 /*!< Subscribe configuration for TASKS_STARTTX                             */
  __IO uint32_t  SUBSCRIBE_STOPTX;                  /*!< Subscribe configuration for TASKS_STOPTX                              */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  SUBSCRIBE_SUSPEND;                 /*!< Subscribe configuration for TASKS_SUSPEND                             */
  __IO uint32_t  SUBSCRIBE_RESUME;                  /*!< Subscribe configuration for TASKS_RESUME                              */
  __I  uint32_t  RESERVED4[2];
  __IO uint32_t  SUBSCRIBE_FLUSHRX;                 /*!< Subscribe configuration for TASKS_FLUSHRX                             */
  __I  uint32_t  RESERVED5[20];
  __IO uint32_t  EVENTS_CTS;                        /*!< CTS is activated (set low). Clear To Send.                            */
  __IO uint32_t  EVENTS_NCTS;                       /*!< CTS is deactivated (set high). Not Clear To Send.                     */
  __IO uint32_t  EVENTS_RXDRDY;                     /*!< Data received in RXD (but potentially not yet transferred to
                                                         Data RAM)                                                             */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  EVENTS_ENDRX;                      /*!< Receive buffer is filled up                                           */
  __I  uint32_t  RESERVED7[2];
  __IO uint32_t  EVENTS_TXDRDY;                     /*!< Data sent from TXD                                                    */
  __IO uint32_t  EVENTS_ENDTX;                      /*!< Last TX byte transmitted                                              */
  __IO uint32_t  EVENTS_ERROR;                      /*!< Error detected                                                        */
  __I  uint32_t  RESERVED8[7];
  __IO uint32_t  EVENTS_RXTO;                       /*!< Receiver timeout                                                      */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< UART receiver has started                                             */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< UART transmitter has started                                          */
  __IO uint32_t  EVENTS_RTS;                        /*!< RX FIFO has only room for four more bytes before it overflows         */
  __IO uint32_t  EVENTS_TXSTOPPED;                  /*!< Transmitter stopped                                                   */
  __I  uint32_t  RESERVED10[9];
  __IO uint32_t  PUBLISH_CTS;                       /*!< Publish configuration for EVENTS_CTS                                  */
  __IO uint32_t  PUBLISH_NCTS;                      /*!< Publish configuration for EVENTS_NCTS                                 */
  __IO uint32_t  PUBLISH_RXDRDY;                    /*!< Publish configuration for EVENTS_RXDRDY                               */
  __I  uint32_t  RESERVED11;
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __I  uint32_t  RESERVED12[2];
  __IO uint32_t  PUBLISH_TXDRDY;                    /*!< Publish configuration for EVENTS_TXDRDY                               */
  __IO uint32_t  PUBLISH_ENDTX;                     /*!< Publish configuration for EVENTS_ENDTX                                */
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED13[7];
  __IO uint32_t  PUBLISH_RXTO;                      /*!< Publish configuration for EVENTS_RXTO                                 */
  __I  uint32_t  RESERVED14;
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __IO uint32_t  PUBLISH_RTS;                       /*!< Publish configuration for EVENTS_RTS                                  */
  __IO uint32_t  PUBLISH_TXSTOPPED;                 /*!< Publish configuration for EVENTS_TXSTOPPED                            */
  __I  uint32_t  RESERVED15[9];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED16[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED17[93];
  __IO uint32_t  ERRORSRC;                          /*!< Error source Note : this register is read / write one to clear.       */
  __I  uint32_t  RESERVED18[31];
  __IO uint32_t  ENABLE;                            /*!< Enable UART                                                           */
  __I  uint32_t  RESERVED19;
  UARTE_PSEL_Type PSEL;                             /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED20[3];
  __IO uint32_t  BAUDRATE;                          /*!< Baud rate. Accuracy depends on the HFCLK source selected.             */
  __I  uint32_t  RESERVED21[3];
  UARTE_RXD_Type RXD;                               /*!< RXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED22;
  UARTE_TXD_Type TXD;                               /*!< TXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED23[7];
  __IO uint32_t  CONFIG;                            /*!< Configuration of parity and hardware flow control                     */
  __I  uint32_t  RESERVED24[675];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_UARTE_Type;


/* ================================================================================ */
/* ================                      SPIM                      ================ */
/* ================================================================================ */


/**
  * @brief Serial Peripheral Interface Master with EasyDMA 0 (SPIM)
  */

typedef struct {                                    /*!< SPIM Structure                                                        */
  __I  uint32_t  RESERVED0[4];
  __O  uint32_t  TASKS_START;                       /*!< Start SPI transaction                                                 */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop SPI transaction                                                  */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  TASKS_SUSPEND;                     /*!< Suspend SPI transaction                                               */
  __O  uint32_t  TASKS_RESUME;                      /*!< Resume SPI transaction                                                */
  __I  uint32_t  RESERVED2[27];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  SUBSCRIBE_SUSPEND;                 /*!< Subscribe configuration for TASKS_SUSPEND                             */
  __IO uint32_t  SUBSCRIBE_RESUME;                  /*!< Subscribe configuration for TASKS_RESUME                              */
  __I  uint32_t  RESERVED4[24];
  __IO uint32_t  EVENTS_STOPPED;                    /*!< SPI transaction has stopped                                           */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  EVENTS_ENDRX;                      /*!< End of RXD buffer reached                                             */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  EVENTS_END;                        /*!< End of RXD buffer and TXD buffer reached                              */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  EVENTS_ENDTX;                      /*!< End of TXD buffer reached                                             */
  __I  uint32_t  RESERVED8[10];
  __IO uint32_t  EVENTS_STARTED;                    /*!< Transaction started                                                   */
  __I  uint32_t  RESERVED9[13];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED10[2];
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __I  uint32_t  RESERVED11;
  __IO uint32_t  PUBLISH_END;                       /*!< Publish configuration for EVENTS_END                                  */
  __I  uint32_t  RESERVED12;
  __IO uint32_t  PUBLISH_ENDTX;                     /*!< Publish configuration for EVENTS_ENDTX                                */
  __I  uint32_t  RESERVED13[10];
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __I  uint32_t  RESERVED14[12];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED15[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED16[125];
  __IO uint32_t  ENABLE;                            /*!< Enable SPIM                                                           */
  __I  uint32_t  RESERVED17;
  SPIM_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED18;
  __I  uint32_t  RXDDATA;                           /*!< RXD register                                                          */
  __IO uint32_t  TXDDATA;                           /*!< TXD register                                                          */
  __I  uint32_t  RESERVED19;
  __IO uint32_t  FREQUENCY;                         /*!< SPI frequency                                                         */
  __I  uint32_t  RESERVED20[3];
  SPIM_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  SPIM_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __I  uint32_t  RESERVED21[26];
  __IO uint32_t  ORC;                               /*!< Over-read character. Character clocked out in case and over-read
                                                         of the TXD buffer.                                                    */
  __I  uint32_t  RESERVED22[654];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_SPIM_Type;


/* ================================================================================ */
/* ================                      SPIS                      ================ */
/* ================================================================================ */


/**
  * @brief SPI Slave 0 (SPIS)
  */

typedef struct {                                    /*!< SPIS Structure                                                        */
  __I  uint32_t  RESERVED0[9];
  __O  uint32_t  TASKS_ACQUIRE;                     /*!< Acquire SPI semaphore                                                 */
  __O  uint32_t  TASKS_RELEASE;                     /*!< Release SPI semaphore, enabling the SPI slave to acquire it           */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  SUBSCRIBE_ACQUIRE;                 /*!< Subscribe configuration for TASKS_ACQUIRE                             */
  __IO uint32_t  SUBSCRIBE_RELEASE;                 /*!< Subscribe configuration for TASKS_RELEASE                             */
  __I  uint32_t  RESERVED2[22];
  __IO uint32_t  EVENTS_END;                        /*!< Granted transaction completed                                         */
  __I  uint32_t  RESERVED3[2];
  __IO uint32_t  EVENTS_ENDRX;                      /*!< End of RXD buffer reached                                             */
  __I  uint32_t  RESERVED4[5];
  __IO uint32_t  EVENTS_ACQUIRED;                   /*!< Semaphore acquired                                                    */
  __I  uint32_t  RESERVED5[22];
  __IO uint32_t  PUBLISH_END;                       /*!< Publish configuration for EVENTS_END                                  */
  __I  uint32_t  RESERVED6[2];
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __I  uint32_t  RESERVED7[5];
  __IO uint32_t  PUBLISH_ACQUIRED;                  /*!< Publish configuration for EVENTS_ACQUIRED                             */
  __I  uint32_t  RESERVED8[21];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED9[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED10[61];
  __I  uint32_t  SEMSTAT;                           /*!< Semaphore status register                                             */
  __I  uint32_t  RESERVED11[15];
  __IO uint32_t  STATUS;                            /*!< Status from last transaction                                          */
  __I  uint32_t  RESERVED12[47];
  __IO uint32_t  ENABLE;                            /*!< Enable SPI slave                                                      */
  __I  uint32_t  RESERVED13;
  SPIS_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED14[7];
  SPIS_RXD_Type RXD;                                /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED15;
  SPIS_TXD_Type TXD;                                /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED16;
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __I  uint32_t  RESERVED17;
  __IO uint32_t  DEF;                               /*!< Default character. Character clocked out in case of an ignored
                                                         transaction.                                                          */
  __I  uint32_t  RESERVED18[24];
  __IO uint32_t  ORC;                               /*!< Over-read character                                                   */
  __I  uint32_t  RESERVED19[654];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_SPIS_Type;


/* ================================================================================ */
/* ================                      TWIM                      ================ */
/* ================================================================================ */


/**
  * @brief I2C compatible Two-Wire Master Interface with EasyDMA 0 (TWIM)
  */

typedef struct {                                    /*!< TWIM Structure                                                        */
  __O  uint32_t  TASKS_STARTRX;                     /*!< Start TWI receive sequence                                            */
  __I  uint32_t  RESERVED0;
  __O  uint32_t  TASKS_STARTTX;                     /*!< Start TWI transmit sequence                                           */
  __I  uint32_t  RESERVED1[2];
  __O  uint32_t  TASKS_STOP;                        /*!< Stop TWI transaction. Must be issued while the TWI master is
                                                         not suspended.                                                        */
  __I  uint32_t  RESERVED2;
  __O  uint32_t  TASKS_SUSPEND;                     /*!< Suspend TWI transaction                                               */
  __O  uint32_t  TASKS_RESUME;                      /*!< Resume TWI transaction                                                */
  __I  uint32_t  RESERVED3[23];
  __IO uint32_t  SUBSCRIBE_STARTRX;                 /*!< Subscribe configuration for TASKS_STARTRX                             */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  SUBSCRIBE_STARTTX;                 /*!< Subscribe configuration for TASKS_STARTTX                             */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  SUBSCRIBE_SUSPEND;                 /*!< Subscribe configuration for TASKS_SUSPEND                             */
  __IO uint32_t  SUBSCRIBE_RESUME;                  /*!< Subscribe configuration for TASKS_RESUME                              */
  __I  uint32_t  RESERVED7[24];
  __IO uint32_t  EVENTS_STOPPED;                    /*!< TWI stopped                                                           */
  __I  uint32_t  RESERVED8[2];
  __IO uint32_t  EVENTS_ENDRX;                      /*!< End of RXD buffer access by EasyDMA                                   */
  __I  uint32_t  RESERVED9[3];
  __IO uint32_t  EVENTS_ENDTX;                      /*!< END of TXD buffer access by EasyDMA                                   */
  __IO uint32_t  EVENTS_ERROR;                      /*!< TWI error                                                             */
  __I  uint32_t  RESERVED10[4];
  __IO uint32_t  EVENTS_BB;                         /*!< TWI byte boundary, generated before each byte that is sent or
                                                         received                                                              */
  __I  uint32_t  RESERVED11[3];
  __IO uint32_t  EVENTS_SUSPENDED;                  /*!< Last byte has been sent out after the SUSPEND task has been
                                                         issued, TWI traffic is now suspended.                                 */
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< Receive sequence started                                              */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< Transmit sequence started                                             */
  __I  uint32_t  RESERVED12[2];
  __IO uint32_t  EVENTS_LASTRX;                     /*!< Byte boundary, starting to receive the last byte                      */
  __IO uint32_t  EVENTS_LASTTX;                     /*!< Byte boundary, starting to transmit the last byte                     */
  __I  uint32_t  RESERVED13[8];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED14[2];
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __I  uint32_t  RESERVED15[3];
  __IO uint32_t  PUBLISH_ENDTX;                     /*!< Publish configuration for EVENTS_ENDTX                                */
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED16[4];
  __IO uint32_t  PUBLISH_BB;                        /*!< Publish configuration for EVENTS_BB                                   */
  __I  uint32_t  RESERVED17[3];
  __IO uint32_t  PUBLISH_SUSPENDED;                 /*!< Publish configuration for EVENTS_SUSPENDED                            */
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __I  uint32_t  RESERVED18[2];
  __IO uint32_t  PUBLISH_LASTRX;                    /*!< Publish configuration for EVENTS_LASTRX                               */
  __IO uint32_t  PUBLISH_LASTTX;                    /*!< Publish configuration for EVENTS_LASTTX                               */
  __I  uint32_t  RESERVED19[7];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED20[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED21[110];
  __IO uint32_t  ERRORSRC;                          /*!< Error source                                                          */
  __I  uint32_t  RESERVED22[14];
  __IO uint32_t  ENABLE;                            /*!< Enable TWIM                                                           */
  __I  uint32_t  RESERVED23;
  TWIM_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED24[5];
  __IO uint32_t  FREQUENCY;                         /*!< TWI frequency                                                         */
  __I  uint32_t  RESERVED25[3];
  TWIM_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  TWIM_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED26[13];
  __IO uint32_t  ADDRESS;                           /*!< Address used in the TWI transfer                                      */
  __I  uint32_t  RESERVED27[668];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_TWIM_Type;


/* ================================================================================ */
/* ================                      TWIS                      ================ */
/* ================================================================================ */


/**
  * @brief I2C compatible Two-Wire Slave Interface with EasyDMA 0 (TWIS)
  */

typedef struct {                                    /*!< TWIS Structure                                                        */
  __I  uint32_t  RESERVED0[5];
  __O  uint32_t  TASKS_STOP;                        /*!< Stop TWI transaction                                                  */
  __I  uint32_t  RESERVED1;
  __O  uint32_t  TASKS_SUSPEND;                     /*!< Suspend TWI transaction                                               */
  __O  uint32_t  TASKS_RESUME;                      /*!< Resume TWI transaction                                                */
  __I  uint32_t  RESERVED2[3];
  __O  uint32_t  TASKS_PREPARERX;                   /*!< Prepare the TWI slave to respond to a write command                   */
  __O  uint32_t  TASKS_PREPARETX;                   /*!< Prepare the TWI slave to respond to a read command                    */
  __I  uint32_t  RESERVED3[23];
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  SUBSCRIBE_SUSPEND;                 /*!< Subscribe configuration for TASKS_SUSPEND                             */
  __IO uint32_t  SUBSCRIBE_RESUME;                  /*!< Subscribe configuration for TASKS_RESUME                              */
  __I  uint32_t  RESERVED5[3];
  __IO uint32_t  SUBSCRIBE_PREPARERX;               /*!< Subscribe configuration for TASKS_PREPARERX                           */
  __IO uint32_t  SUBSCRIBE_PREPARETX;               /*!< Subscribe configuration for TASKS_PREPARETX                           */
  __I  uint32_t  RESERVED6[19];
  __IO uint32_t  EVENTS_STOPPED;                    /*!< TWI stopped                                                           */
  __I  uint32_t  RESERVED7[7];
  __IO uint32_t  EVENTS_ERROR;                      /*!< TWI error                                                             */
  __I  uint32_t  RESERVED8[3];
  __IO uint32_t  EVENTS_NACKTX;                     /*!< NACK on TX                                                            */
  __IO uint32_t  EVENTS_BB;                         /*!< TWI byte boundary, generated before each byte that is sent or
                                                         received                                                              */
  __I  uint32_t  RESERVED9[4];
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< Receive sequence started                                              */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< Transmit sequence started                                             */
  __I  uint32_t  RESERVED10[4];
  __IO uint32_t  EVENTS_WRITE;                      /*!< Write command received                                                */
  __IO uint32_t  EVENTS_READ;                       /*!< Read command received                                                 */
  __IO uint32_t  EVENTS_CSSTARTED;                  /*!< TWI started stretching the clock                                      */
  __IO uint32_t  EVENTS_CSSTOPPED;                  /*!< TWI stopped stretching the clock                                      */
  __I  uint32_t  RESERVED11[4];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED12[7];
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED13[3];
  __IO uint32_t  PUBLISH_NACKTX;                    /*!< Publish configuration for EVENTS_NACKTX                               */
  __IO uint32_t  PUBLISH_BB;                        /*!< Publish configuration for EVENTS_BB                                   */
  __I  uint32_t  RESERVED14[4];
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __I  uint32_t  RESERVED15[4];
  __IO uint32_t  PUBLISH_WRITE;                     /*!< Publish configuration for EVENTS_WRITE                                */
  __IO uint32_t  PUBLISH_READ;                      /*!< Publish configuration for EVENTS_READ                                 */
  __IO uint32_t  PUBLISH_CSSTARTED;                 /*!< Publish configuration for EVENTS_CSSTARTED                            */
  __IO uint32_t  PUBLISH_CSSTOPPED;                 /*!< Publish configuration for EVENTS_CSSTOPPED                            */
  __I  uint32_t  RESERVED16[3];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED17[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED18[113];
  __IO uint32_t  ERRORSRC;                          /*!< Error source                                                          */
  __I  uint32_t  MATCH;                             /*!< Status register indicating which address had a match                  */
  __I  uint32_t  PINSTATUS;                         /*!< Returns state of signals reflecting the SDA and SCL line status.
                                                         All fields will return a '0' if DEBUGENABLE is not Enabled.           */
  __I  uint32_t  RESERVED19[9];
  __IO uint32_t  ENABLE;                            /*!< Enable TWIS                                                           */
  __I  uint32_t  RESERVED20;
  TWIS_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED21[9];
  TWIS_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED22;
  TWIS_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED23[14];
  __IO uint32_t  ADDRESS[2];                        /*!< Description collection[0]: TWI slave address 0                        */
  __I  uint32_t  RESERVED24;
  __IO uint32_t  CONFIG;                            /*!< Configuration register for the address match mechanism                */
  __I  uint32_t  RESERVED25[2];
  __IO uint32_t  INPUTMODE;                         /*!< Input mode                                                            */
  __IO uint32_t  DEBUGENABLE;                       /*!< Enable TWIS debug features (PINSTATUS register)                       */
  __I  uint32_t  RESERVED26[6];
  __IO uint32_t  ORC;                               /*!< Over-read character. Character sent out in case of an over-read
                                                         of the transmit buffer.                                               */
  __I  uint32_t  RESERVED27[654];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_TWIS_Type;


/* ================================================================================ */
/* ================                     GPIOTE                     ================ */
/* ================================================================================ */


/**
  * @brief GPIO Tasks and Events (GPIOTE)
  */

typedef struct {                                    /*!< GPIOTE Structure                                                      */
  __O  uint32_t  TASKS_OUT[8];                      /*!< Description collection[0]: Task for writing to pin specified
                                                         in CONFIG[0].PSEL. Action on pin is configured in CONFIG[0].POLARITY. */
  __I  uint32_t  RESERVED0[4];
  __O  uint32_t  TASKS_SET[8];                      /*!< Description collection[0]: Task for writing to pin specified
                                                         in CONFIG[0].PSEL. Action on pin is to set it high.                   */
  __I  uint32_t  RESERVED1[4];
  __O  uint32_t  TASKS_CLR[8];                      /*!< Description collection[0]: Task for writing to pin specified
                                                         in CONFIG[0].PSEL. Action on pin is to set it low.                    */
  __IO uint32_t  SUBSCRIBE_OUT[8];                  /*!< Description collection[0]: Subscribe configuration for TASKS_OUT[0]   */
  __I  uint32_t  RESERVED2[4];
  __IO uint32_t  SUBSCRIBE_SET[8];                  /*!< Description collection[0]: Subscribe configuration for TASKS_SET[0]   */
  __I  uint32_t  RESERVED3[4];
  __IO uint32_t  SUBSCRIBE_CLR[8];                  /*!< Description collection[0]: Subscribe configuration for TASKS_CLR[0]   */
  __IO uint32_t  EVENTS_IN[8];                      /*!< Description collection[0]: Event generated from pin specified
                                                         in CONFIG[0].PSEL                                                     */
  __I  uint32_t  RESERVED4[23];
  __IO uint32_t  EVENTS_PORT;                       /*!< Event generated from multiple input GPIO pins with SENSE mechanism
                                                         enabled                                                               */
  __IO uint32_t  PUBLISH_IN[8];                     /*!< Description collection[0]: Publish configuration for EVENTS_IN[0]     */
  __I  uint32_t  RESERVED5[23];
  __IO uint32_t  PUBLISH_PORT;                      /*!< Publish configuration for EVENTS_PORT                                 */
  __I  uint32_t  RESERVED6[64];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[129];
  __IO uint32_t  CONFIG[8];                         /*!< Description collection[0]: Configuration for OUT[n], SET[n]
                                                         and CLR[n] tasks and IN[n] event                                      */
  __I  uint32_t  RESERVED8[52];
  __IO uint32_t  DEBOUNCE[8];                       /*!< Description collection[0]: Debounce mode for GPIOTE channel
                                                         n                                                                     */
  __I  uint32_t  RESERVED9[631];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_GPIOTE_Type;


/* ================================================================================ */
/* ================                      SAADC                     ================ */
/* ================================================================================ */


/**
  * @brief Analog to Digital Converter (SAADC)
  */

typedef struct {                                    /*!< SAADC Structure                                                       */
  __O  uint32_t  TASKS_START;                       /*!< Start the ADC and prepare the result buffer in RAM                    */
  __O  uint32_t  TASKS_SAMPLE;                      /*!< Take one ADC sample, if scan is enabled all channels are sampled      */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop the ADC and terminate any on-going conversion                    */
  __O  uint32_t  TASKS_CALIBRATEOFFSET;             /*!< Starts offset auto-calibration                                        */
  __O  uint32_t  TASKS_CALIBRATEGAIN;               /*!< Starts gain auto-calibration                                          */
  __I  uint32_t  RESERVED0[27];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_SAMPLE;                  /*!< Subscribe configuration for TASKS_SAMPLE                              */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_CALIBRATEOFFSET;         /*!< Subscribe configuration for TASKS_CALIBRATEOFFSET                     */
  __IO uint32_t  SUBSCRIBE_CALIBRATEGAIN;           /*!< Subscribe configuration for TASKS_CALIBRATEGAIN                       */
  __I  uint32_t  RESERVED1[27];
  __IO uint32_t  EVENTS_STARTED;                    /*!< The ADC has started                                                   */
  __IO uint32_t  EVENTS_END;                        /*!< The ADC has filled up the Result buffer                               */
  __IO uint32_t  EVENTS_DONE;                       /*!< A conversion task has been completed. Depending on the mode,
                                                         multiple conversions might be needed for a result to be transferred
                                                          to RAM.                                                              */
  __IO uint32_t  EVENTS_RESULTDONE;                 /*!< A result is ready to get transferred to RAM. Result is available
                                                         in REGRESULT register                                                 */
  __IO uint32_t  EVENTS_CALIBRATEDONE;              /*!< Calibration is complete                                               */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< The ADC has stopped                                                   */
  SAADC_EVENTS_CH_Type EVENTS_CH[8];                /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED2[10];
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __IO uint32_t  PUBLISH_END;                       /*!< Publish configuration for EVENTS_END                                  */
  __IO uint32_t  PUBLISH_DONE;                      /*!< Publish configuration for EVENTS_DONE                                 */
  __IO uint32_t  PUBLISH_RESULTDONE;                /*!< Publish configuration for EVENTS_RESULTDONE                           */
  __IO uint32_t  PUBLISH_CALIBRATEDONE;             /*!< Publish configuration for EVENTS_CALIBRATEDONE                        */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  SAADC_PUBLISH_CH_Type PUBLISH_CH[8];              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED3[10];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[61];
  __I  uint32_t  STATUS;                            /*!< Status                                                                */
  __I  uint32_t  RESERVED6[63];
  __IO uint32_t  ENABLE;                            /*!< Enable or disable ADC                                                 */
  __I  uint32_t  RESERVED7[3];
  SAADC_CH_Type CH[8];                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED8[23];
  __I  uint32_t  REGRESULT;                         /*!< Last conversion result                                                */
  __IO uint32_t  RESOLUTION;                        /*!< Resolution configuration                                              */
  __IO uint32_t  OVERSAMPLE;                        /*!< Oversampling configuration. OVERSAMPLE should not be combined
                                                         with SCAN. The RESOLUTION is applied before averaging, thus
                                                          for high OVERSAMPLE a higher RESOLUTION should be used.              */
  __IO uint32_t  SAMPLERATE;                        /*!< Controls normal or continuous sample rate                             */
  __I  uint32_t  RESERVED9[12];
  SAADC_RESULT_Type RESULT;                         /*!< RESULT EasyDMA channel                                                */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  TESTCTRL;                          /*!< Control signals used during test of ADC                               */
  __IO uint32_t  CALOFFSET;                         /*!< Calibration control for offset error                                  */
  __IO uint32_t  CALGAIN;                           /*!< Calibration control for gain error                                    */
  __IO uint32_t  CALVREF;                           /*!< Calibration control for reference voltage                             */
  __I  uint32_t  RESERVED11[620];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_SAADC_Type;


/* ================================================================================ */
/* ================                      TIMER                     ================ */
/* ================================================================================ */


/**
  * @brief Timer/Counter 0 (TIMER)
  */

typedef struct {                                    /*!< TIMER Structure                                                       */
  __O  uint32_t  TASKS_START;                       /*!< Start Timer                                                           */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop Timer                                                            */
  __O  uint32_t  TASKS_COUNT;                       /*!< Increment Timer (Counter mode only)                                   */
  __O  uint32_t  TASKS_CLEAR;                       /*!< Clear time                                                            */
  __O  uint32_t  TASKS_SHUTDOWN;                    /*!< Deprecated register - Shut down timer                                 */
  __I  uint32_t  RESERVED0[11];
  __O  uint32_t  TASKS_CAPTURE[6];                  /*!< Description collection[0]: Capture Timer value to CC[0] register      */
  __I  uint32_t  RESERVED1[10];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_COUNT;                   /*!< Subscribe configuration for TASKS_COUNT                               */
  __IO uint32_t  SUBSCRIBE_CLEAR;                   /*!< Subscribe configuration for TASKS_CLEAR                               */
  __IO uint32_t  SUBSCRIBE_SHUTDOWN;                /*!< Subscribe configuration for TASKS_SHUTDOWN                            */
  __I  uint32_t  RESERVED2[11];
  __IO uint32_t  SUBSCRIBE_CAPTURE[6];              /*!< Description collection[0]: Subscribe configuration for TASKS_CAPTURE[0] */
  __I  uint32_t  RESERVED3[26];
  __IO uint32_t  EVENTS_COMPARE[6];                 /*!< Description collection[0]: Compare event on CC[0] match               */
  __I  uint32_t  RESERVED4[26];
  __IO uint32_t  PUBLISH_COMPARE[6];                /*!< Description collection[0]: Publish configuration for EVENTS_COMPARE[0] */
  __I  uint32_t  RESERVED5[10];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED6[64];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[126];
  __IO uint32_t  MODE;                              /*!< Timer mode selection                                                  */
  __IO uint32_t  BITMODE;                           /*!< Configure the number of bits used by the TIMER                        */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  PRESCALER;                         /*!< Timer prescaler register                                              */
  __I  uint32_t  RESERVED9[11];
  __IO uint32_t  CC[6];                             /*!< Description collection[0]: Capture/Compare register 0                 */
  __I  uint32_t  RESERVED10[681];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_TIMER_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief Real time counter 0 (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Start RTC COUNTER                                                     */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop RTC COUNTER                                                      */
  __O  uint32_t  TASKS_CLEAR;                       /*!< Clear RTC COUNTER                                                     */
  __O  uint32_t  TASKS_TRIGOVRFLW;                  /*!< Set COUNTER to 0xFFFFF0                                               */
  __I  uint32_t  RESERVED0[28];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_CLEAR;                   /*!< Subscribe configuration for TASKS_CLEAR                               */
  __IO uint32_t  SUBSCRIBE_TRIGOVRFLW;              /*!< Subscribe configuration for TASKS_TRIGOVRFLW                          */
  __I  uint32_t  RESERVED1[28];
  __IO uint32_t  EVENTS_TICK;                       /*!< Event on COUNTER increment                                            */
  __IO uint32_t  EVENTS_OVRFLW;                     /*!< Event on COUNTER overflow                                             */
  __I  uint32_t  RESERVED2[14];
  __IO uint32_t  EVENTS_COMPARE[4];                 /*!< Description collection[0]: Compare event on CC[0] match               */
  __I  uint32_t  RESERVED3[12];
  __IO uint32_t  PUBLISH_TICK;                      /*!< Publish configuration for EVENTS_TICK                                 */
  __IO uint32_t  PUBLISH_OVRFLW;                    /*!< Publish configuration for EVENTS_OVRFLW                               */
  __I  uint32_t  RESERVED4[14];
  __IO uint32_t  PUBLISH_COMPARE[4];                /*!< Description collection[0]: Publish configuration for EVENTS_COMPARE[0] */
  __I  uint32_t  RESERVED5[76];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED6[13];
  __IO uint32_t  EVTEN;                             /*!< Enable or disable event routing                                       */
  __IO uint32_t  EVTENSET;                          /*!< Enable event routing                                                  */
  __IO uint32_t  EVTENCLR;                          /*!< Disable event routing                                                 */
  __I  uint32_t  RESERVED7[110];
  __I  uint32_t  COUNTER;                           /*!< Current COUNTER value                                                 */
  __IO uint32_t  PRESCALER;                         /*!< 12 bit prescaler for COUNTER frequency (32768/(PRESCALER+1)).Must
                                                         be written when RTC is stopped                                        */
  __I  uint32_t  RESERVED8[13];
  __IO uint32_t  CC[4];                             /*!< Description collection[0]: Compare register 0                         */
  __I  uint32_t  RESERVED9[683];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_RTC_Type;


/* ================================================================================ */
/* ================                      DPPIC                     ================ */
/* ================================================================================ */


/**
  * @brief Distributed Programmable Peripheral Interconnect Controller (DPPIC)
  */

typedef struct {                                    /*!< DPPIC Structure                                                       */
  DPPIC_TASKS_CHG_Type TASKS_CHG[6];                /*!< Channel group tasks                                                   */
  __I  uint32_t  RESERVED0[20];
  DPPIC_SUBSCRIBE_CHG_Type SUBSCRIBE_CHG[6];        /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED1[276];
  __IO uint32_t  CHEN;                              /*!< Channel enable register                                               */
  __IO uint32_t  CHENSET;                           /*!< Channel enable set register                                           */
  __IO uint32_t  CHENCLR;                           /*!< Channel enable clear register                                         */
  __I  uint32_t  RESERVED2[189];
  __IO uint32_t  CHG[6];                            /*!< Description collection[0]: Channel group 0                            */
} NRF_DPPIC_Type;


/* ================================================================================ */
/* ================                      NFCT                      ================ */
/* ================================================================================ */


/**
  * @brief NFC-A compatible radio (NFCT)
  */

typedef struct {                                    /*!< NFCT Structure                                                        */
  __O  uint32_t  TASKS_ACTIVATE;                    /*!< Activate NFC peripheral for incoming and outgoing frames, change
                                                         state to activated                                                    */
  __O  uint32_t  TASKS_DISABLE;                     /*!< Disable NFC peripheral                                                */
  __O  uint32_t  TASKS_SENSE;                       /*!< Enable NFC sense field mode, change state to sense mode               */
  __O  uint32_t  TASKS_STARTTX;                     /*!< Start transmission of a outgoing frame, change state to transmit      */
  __O  uint32_t  TASKS_STOPTX;                      /*!< Stops a issued transmission of a frame                                */
  __O  uint32_t  TASKS_START_ROSCCAL;               /*!< Starts calibration of ring oscillator                                 */
  __O  uint32_t  TASKS_FREQMEASURE;                 /*!< Measures the 13.56 MHz clock frequency                                */
  __O  uint32_t  TASKS_ENABLERXDATA;                /*!< Initializes the EasyDMA for receive.                                  */
  __O  uint32_t  TASKS_DISABLERXDATA;               /*!< Ends current EasyDMA transfer and stops waiting for SoF               */
  __O  uint32_t  TASKS_GOIDLE;                      /*!< Force state machine to IDLE state                                     */
  __O  uint32_t  TASKS_GOSLEEP;                     /*!< Force state machine to SLEEP_A state                                  */
  __I  uint32_t  RESERVED0[21];
  __IO uint32_t  SUBSCRIBE_ACTIVATE;                /*!< Subscribe configuration for TASKS_ACTIVATE                            */
  __IO uint32_t  SUBSCRIBE_DISABLE;                 /*!< Subscribe configuration for TASKS_DISABLE                             */
  __IO uint32_t  SUBSCRIBE_SENSE;                   /*!< Subscribe configuration for TASKS_SENSE                               */
  __IO uint32_t  SUBSCRIBE_STARTTX;                 /*!< Subscribe configuration for TASKS_STARTTX                             */
  __IO uint32_t  SUBSCRIBE_STOPTX;                  /*!< Subscribe configuration for TASKS_STOPTX                              */
  __IO uint32_t  SUBSCRIBE_START_ROSCCAL;           /*!< Subscribe configuration for TASKS_START_ROSCCAL                       */
  __IO uint32_t  SUBSCRIBE_FREQMEASURE;             /*!< Subscribe configuration for TASKS_FREQMEASURE                         */
  __IO uint32_t  SUBSCRIBE_ENABLERXDATA;            /*!< Subscribe configuration for TASKS_ENABLERXDATA                        */
  __IO uint32_t  SUBSCRIBE_DISABLERXDATA;           /*!< Subscribe configuration for TASKS_DISABLERXDATA                       */
  __IO uint32_t  SUBSCRIBE_GOIDLE;                  /*!< Subscribe configuration for TASKS_GOIDLE                              */
  __IO uint32_t  SUBSCRIBE_GOSLEEP;                 /*!< Subscribe configuration for TASKS_GOSLEEP                             */
  __I  uint32_t  RESERVED1[21];
  __IO uint32_t  EVENTS_READY;                      /*!< The NFC peripheral is ready to receive and send frames                */
  __IO uint32_t  EVENTS_FIELDDETECTED;              /*!< Remote NFC field detected                                             */
  __IO uint32_t  EVENTS_FIELDLOST;                  /*!< Remote NFC field lost                                                 */
  __IO uint32_t  EVENTS_TXFRAMESTART;               /*!< Marks the start of the first symbol of a transmitted frame            */
  __IO uint32_t  EVENTS_TXFRAMEEND;                 /*!< Marks the end of the last transmitted on-air symbol of a frame
                                                         data bit on the last positive edge ckTxNfc with nfcTransmit
                                                          = 1                                                                  */
  __IO uint32_t  EVENTS_RXFRAMESTART;               /*!< Marks the end of the first symbol of a received frame                 */
  __IO uint32_t  EVENTS_RXFRAMEEND;                 /*!< Received data have been checked (CRC, parity) and transferred
                                                         to RAM, and EasyDMA has ended accessing the RX buffer                 */
  __IO uint32_t  EVENTS_ERROR;                      /*!< NFC error reported. The ERRORSTATUS register contains details
                                                         on the source of the error.                                           */
  __IO uint32_t  EVENTS_FREQMEASUREDONE;            /*!< Done with one frequency measurement, result available in the
                                                         MEASUREDFREQ register. This event will fire several times during
                                                          calibration.                                                         */
  __IO uint32_t  EVENTS_CALCOMPLETE;                /*!< NFC ring oscillator calibration complete.                             */
  __IO uint32_t  EVENTS_RXERROR;                    /*!< NFC RX frame error reported. The FRAMESTATUS.RX register contains
                                                         details on the source of the error.                                   */
  __IO uint32_t  EVENTS_ENDRX;                      /*!< RX buffer (as defined by PACKETPTR and MAXLEN) in Data RAM full.      */
  __IO uint32_t  EVENTS_ENDTX;                      /*!< Transmission of data in RAM has ended, and EasyDMA has ended
                                                         accessing the TX buffer                                               */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  EVENTS_AUTOCOLRESSTARTED;          /*!< Auto collision resolution process has started Event generated
                                                         when ALL_REQ or SENS_REQ has been received while is IDLE state        */
  __IO uint32_t  EVENTS_AUTOCOLRES1;                /*!< Auto collision resolution cascade level 1 succeeded                   */
  __IO uint32_t  EVENTS_AUTOCOLRES2;                /*!< Auto collision resolution cascade level 2 succeeded                   */
  __IO uint32_t  EVENTS_AUTOCOLRES3;                /*!< Auto collision resolution cascade level 3 succeeded                   */
  __IO uint32_t  EVENTS_COLLISION;                  /*!< NFC Auto collision resolution error reported. The AUTOCOLRESSTATUS
                                                         register contains details on the source of the error.                 */
  __IO uint32_t  EVENTS_SELECTED;                   /*!< NFC Auto collision resolution successfully completed                  */
  __IO uint32_t  EVENTS_STARTED;                    /*!< EasyDMA is ready to receive or send frames.                           */
  __I  uint32_t  RESERVED3[11];
  __IO uint32_t  PUBLISH_READY;                     /*!< Publish configuration for EVENTS_READY                                */
  __IO uint32_t  PUBLISH_FIELDDETECTED;             /*!< Publish configuration for EVENTS_FIELDDETECTED                        */
  __IO uint32_t  PUBLISH_FIELDLOST;                 /*!< Publish configuration for EVENTS_FIELDLOST                            */
  __IO uint32_t  PUBLISH_TXFRAMESTART;              /*!< Publish configuration for EVENTS_TXFRAMESTART                         */
  __IO uint32_t  PUBLISH_TXFRAMEEND;                /*!< Publish configuration for EVENTS_TXFRAMEEND                           */
  __IO uint32_t  PUBLISH_RXFRAMESTART;              /*!< Publish configuration for EVENTS_RXFRAMESTART                         */
  __IO uint32_t  PUBLISH_RXFRAMEEND;                /*!< Publish configuration for EVENTS_RXFRAMEEND                           */
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __IO uint32_t  PUBLISH_FREQMEASUREDONE;           /*!< Publish configuration for EVENTS_FREQMEASUREDONE                      */
  __IO uint32_t  PUBLISH_CALCOMPLETE;               /*!< Publish configuration for EVENTS_CALCOMPLETE                          */
  __IO uint32_t  PUBLISH_RXERROR;                   /*!< Publish configuration for EVENTS_RXERROR                              */
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __IO uint32_t  PUBLISH_ENDTX;                     /*!< Publish configuration for EVENTS_ENDTX                                */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  PUBLISH_AUTOCOLRESSTARTED;         /*!< Publish configuration for EVENTS_AUTOCOLRESSTARTED                    */
  __IO uint32_t  PUBLISH_AUTOCOLRES1;               /*!< Publish configuration for EVENTS_AUTOCOLRES1                          */
  __IO uint32_t  PUBLISH_AUTOCOLRES2;               /*!< Publish configuration for EVENTS_AUTOCOLRES2                          */
  __IO uint32_t  PUBLISH_AUTOCOLRES3;               /*!< Publish configuration for EVENTS_AUTOCOLRES3                          */
  __IO uint32_t  PUBLISH_COLLISION;                 /*!< Publish configuration for EVENTS_COLLISION                            */
  __IO uint32_t  PUBLISH_SELECTED;                  /*!< Publish configuration for EVENTS_SELECTED                             */
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __I  uint32_t  RESERVED5[11];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED6[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[61];
  __I  uint32_t  PWRUPSENSE;                        /*!< Tells if the NFCT peripheral is in SENSE mode                         */
  __IO uint32_t  ERRORSTATUS;                       /*!< NFC Error Status register                                             */
  __IO uint32_t  AUTOCOLRESSTATUS;                  /*!< NFC Auto collision resolution Error Status register                   */
  NFCT_FRAMESTATUS_Type FRAMESTATUS;                /*!< Unspecified                                                           */
  __I  uint32_t  NFCTAGSTATE;                       /*!< NfcTag state register                                                 */
  __I  uint32_t  RESERVED8[3];
  __I  uint32_t  DEFAULTSTATESLEEP;                 /*!< Reflects the default state during automatic collision resolution      */
  __I  uint32_t  BYTESWRITTENTORAM;                 /*!< Number of bytes written to RAM                                        */
  __I  uint32_t  NFCFRAMINGCORESTATE;               /*!< NFC Framing Core State                                                */
  __IO uint32_t  NFCANTICOLRESSTATE;                /*!< Automatic collision resolution (anti-collision) state                 */
  __I  uint32_t  RESERVED9;
  __I  uint32_t  MEASUREDFREQ;                      /*!< The frequency measured on the 13.56 MHz NFC carrier signal            */
  __I  uint32_t  ROSCCALVALUE;                      /*!< The current value used by the ring oscillator                         */
  __I  uint32_t  FIELDPRESENT;                      /*!< Indicates the presence or not of a valid field                        */
  __I  uint32_t  CURRENTPEAKDETECTREF;              /*!< Current peak detect reference                                         */
  __I  uint32_t  RESERVED10[47];
  __IO uint32_t  ENABLE;                            /*!< Register to enable NFC functionality                                  */
  __IO uint32_t  FRAMEDELAYMIN;                     /*!< Minimum frame delay                                                   */
  __IO uint32_t  FRAMEDELAYMAX;                     /*!< Maximum frame delay                                                   */
  __IO uint32_t  FRAMEDELAYMODE;                    /*!< Configuration register for the Frame Delay Timer                      */
  __IO uint32_t  PACKETPTR;                         /*!< Packet pointer for TXD and RXD data storage in Data RAM               */
  __IO uint32_t  MAXLEN;                            /*!< Size of allocated for TXD and RXD data storage buffer in Data
                                                         RAM                                                                   */
  NFCT_TXD_Type TXD;                                /*!< Unspecified                                                           */
  NFCT_RXD_Type RXD;                                /*!< Unspecified                                                           */
  __IO uint32_t  FRAMEDELAYSHIFT;                   /*!< Correction for the frame delay reference                              */
  __I  uint32_t  RESERVED11;
  __IO uint32_t  CRCPOLY;                           /*!< CRC polynomial                                                        */
  __IO uint32_t  CRCINIT;                           /*!< CRC initial value                                                     */
  __I  uint32_t  RESERVED12[22];
  __IO uint32_t  NFCID1_LAST;                       /*!< Last NFCID1 part (4, 7 or 10 bytes ID)                                */
  __IO uint32_t  NFCID1_2ND_LAST;                   /*!< Second last NFCID1 part (7 or 10 bytes ID)                            */
  __IO uint32_t  NFCID1_3RD_LAST;                   /*!< Third last NFCID1 part (10 bytes ID)                                  */
  __IO uint32_t  AUTOCOLRESCONFIG;                  /*!< Controls the Auto collision resolution function. This setting
                                                         must be done before the NFCT peripheral is enabled.                   */
  __IO uint32_t  SENSRES;                           /*!< NFC-A SENS_RES auto-response settings                                 */
  __IO uint32_t  SELRES;                            /*!< NFC-A SEL_RES auto-response settings                                  */
  __I  uint32_t  RESERVED13[22];
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED14[6];
  __IO uint32_t  DEMODCONFIG;                       /*!< Configuration settings for demodulation                               */
  __IO uint32_t  FIELDDETECTCONFIG;                 /*!< Configuration for NFCT_FIELDDETECT_TSMC55N                            */
  __IO uint32_t  ANALOGTESTBUSEN;                   /*!< Analog Testbus Enable - ATB0 only                                     */
  __IO uint32_t  SYMBOLTOLLOWER;                    /*!< Configuration register for the tolerance of the received NFC
                                                         pulse width                                                           */
  __IO uint32_t  SYMBOLTOLUPPER;                    /*!< Configuration register for the tolerance of the received NFC
                                                         pulse width                                                           */
  __IO uint32_t  CONTINUOUSSUBCARRIER;              /*!< Selects continuous sub-carrier transmission                           */
  __I  uint32_t  RESERVED15[2];
  __IO uint32_t  CLOCKRECCONFIG;                    /*!< Configuration for NFCT_CLOCKREC_TSMC55N                               */
  __IO uint32_t  CLOCKSOURCE;                       /*!< Deprecated register - Clock source                                    */
  __IO uint32_t  FRAMECONTENT1;                     /*!< Deprecated register - Buffer for custom-frame autoresponse frame-content,
                                                         when no DMA is available                                              */
  __IO uint32_t  FRAMECONTENT2;                     /*!< Deprecated register - Buffer for custom-frame autoresponse,
                                                         when no DMA is available                                              */
  __I  uint32_t  RESERVED16;
  __IO uint32_t  OVERRIDEENABLE;                    /*!< Enables override functionality. Override values for the 2V2
                                                         clamp, 3V6 clamp and loadControl value are blocked if the NFCPADS
                                                          bit of the UICR is set to 0. They are also blocked if the design
                                                          is in a testmode other than nfc ana1, nfc ana2, nfc fpga.            */
  __I  uint32_t  RESERVED17[3];
  __IO uint32_t  OVRVALPWRUPNFC;                    /*!< Override value for pwrupNfc                                           */
  __IO uint32_t  OVRVALMODULATION;                  /*!< Override value for modulation on/off                                  */
  __IO uint32_t  OVRVALLOADINACTIVE;                /*!< Override value for the load control when modulation is off            */
  __IO uint32_t  OVRVALLOADACTIVE;                  /*!< Override value for the load control when modulation is on             */
  __IO uint32_t  OVRVALPWRUPRES;                    /*!< Override value for power up resistor in the analog pad                */
  __IO uint32_t  OVRVALROSCCALCODE;                 /*!< Override value for the calibration word for the oscillator            */
  __IO uint32_t  OVRVALROSCCALEN;                   /*!< Override value for the signal CALEN going to the oscillator           */
  __IO uint32_t  OVRVALCLAMPEN2V2;                  /*!< Override value for the 2V2 clamp                                      */
  __IO uint32_t  OVRVALCLAMPEN3V6;                  /*!< Override value for the 3V6 clamp                                      */
  __IO uint32_t  AUTOCAL;                           /*!< Controls auto-calibration at NFCT module enabling                     */
  __I  uint32_t  RESERVED18;
  __IO uint32_t  SHUNTREGCONFIG;                    /*!< Configuration for NFCT_SHUNTREG_TSMC55N                               */
  __IO uint32_t  LOADMODCONFIG;                     /*!< Configuration for load modulation                                     */
  __IO uint32_t  SHUNTNMOSADJUST;                   /*!< Size adjustment for the shunt NMOS in shunt regulator                 */
  __IO uint32_t  RSHUNTCTRL;                        /*!< Adjust internal shunt resistance                                      */
  __IO uint32_t  OVRVALPEAKDETECTREF;               /*!< Override value for the peak detect reference voltage                  */
  __IO uint32_t  LOCKDETECTWINDOW;                  /*!< Adjust window size on falling edge of lock detect filter used
                                                         for demodulation                                                      */
  __I  uint32_t  RESERVED19[598];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_NFCT_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief Watchdog Timer (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Start the watchdog                                                    */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop the watchdog timer. Only available in timer mode.                */
  __I  uint32_t  RESERVED0[30];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  EVENTS_TIMEOUT;                    /*!< Watchdog timeout                                                      */
  __I  uint32_t  RESERVED2[31];
  __IO uint32_t  PUBLISH_TIMEOUT;                   /*!< Publish configuration for EVENTS_TIMEOUT                              */
  __I  uint32_t  RESERVED3[96];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[61];
  __I  uint32_t  RUNSTATUS;                         /*!< Run status                                                            */
  __I  uint32_t  REQSTATUS;                         /*!< Request status                                                        */
  __I  uint32_t  RCNT;                              /*!< Reload count                                                          */
  __I  uint32_t  RESERVED5[62];
  __IO uint32_t  CRV;                               /*!< Counter reload value                                                  */
  __IO uint32_t  RREN;                              /*!< Enable register for reload request registers                          */
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __IO uint32_t  RRKEY;                             /*!< Reload request key                                                    */
  __I  uint32_t  RESERVED6[59];
  __O  uint32_t  RR[8];                             /*!< Description collection[0]: Reload request 0                           */
  __I  uint32_t  RESERVED7[631];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_WDT_Type;


/* ================================================================================ */
/* ================                      QDEC                      ================ */
/* ================================================================================ */


/**
  * @brief Quadrature Decoder (QDEC)
  */

typedef struct {                                    /*!< QDEC Structure                                                        */
  __O  uint32_t  TASKS_START;                       /*!< Task starting the quadrature decoder                                  */
  __O  uint32_t  TASKS_STOP;                        /*!< Task stopping the quadrature decoder                                  */
  __O  uint32_t  TASKS_READCLRACC;                  /*!< Read and clear ACC and ACCDBL                                         */
  __O  uint32_t  TASKS_RDCLRACC;                    /*!< Read and clear ACC                                                    */
  __O  uint32_t  TASKS_RDCLRDBL;                    /*!< Read and clear ACCDBL                                                 */
  __I  uint32_t  RESERVED0[27];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_READCLRACC;              /*!< Subscribe configuration for TASKS_READCLRACC                          */
  __IO uint32_t  SUBSCRIBE_RDCLRACC;                /*!< Subscribe configuration for TASKS_RDCLRACC                            */
  __IO uint32_t  SUBSCRIBE_RDCLRDBL;                /*!< Subscribe configuration for TASKS_RDCLRDBL                            */
  __I  uint32_t  RESERVED1[27];
  __IO uint32_t  EVENTS_SAMPLERDY;                  /*!< Event being generated for every new sample value written to
                                                         the SAMPLE register                                                   */
  __IO uint32_t  EVENTS_REPORTRDY;                  /*!< Non-null report ready                                                 */
  __IO uint32_t  EVENTS_ACCOF;                      /*!< ACC or ACCDBL register overflow                                       */
  __IO uint32_t  EVENTS_DBLRDY;                     /*!< Double displacement(s) detected                                       */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< QDEC has been stopped                                                 */
  __I  uint32_t  RESERVED2[27];
  __IO uint32_t  PUBLISH_SAMPLERDY;                 /*!< Publish configuration for EVENTS_SAMPLERDY                            */
  __IO uint32_t  PUBLISH_REPORTRDY;                 /*!< Publish configuration for EVENTS_REPORTRDY                            */
  __IO uint32_t  PUBLISH_ACCOF;                     /*!< Publish configuration for EVENTS_ACCOF                                */
  __IO uint32_t  PUBLISH_DBLRDY;                    /*!< Publish configuration for EVENTS_DBLRDY                               */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED3[27];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[125];
  __IO uint32_t  ENABLE;                            /*!< Enable the quadrature decoder                                         */
  __IO uint32_t  LEDPOL;                            /*!< LED output pin polarity                                               */
  __IO uint32_t  SAMPLEPER;                         /*!< Sample period                                                         */
  __I  int32_t   SAMPLE;                            /*!< Motion sample value                                                   */
  __IO uint32_t  REPORTPER;                         /*!< Number of samples to be taken before REPORTRDY and DBLRDY events
                                                         can be generated                                                      */
  __I  int32_t   ACC;                               /*!< Register accumulating the valid transitions                           */
  __I  int32_t   ACCREAD;                           /*!< Snapshot of the ACC register, updated by the READCLRACC or RDCLRACC
                                                         task                                                                  */
  QDEC_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __IO uint32_t  DBFEN;                             /*!< Enable input debounce filters                                         */
  __I  uint32_t  RESERVED6[5];
  __IO uint32_t  LEDPRE;                            /*!< Time period the LED is switched ON prior to sampling                  */
  __I  uint32_t  ACCDBL;                            /*!< Register accumulating the number of detected double transitions       */
  __I  uint32_t  ACCDBLREAD;                        /*!< Snapshot of the ACCDBL, updated by the READCLRACC or RDCLRDBL
                                                         task                                                                  */
  __I  uint32_t  RESERVED7[684];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_QDEC_Type;


/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */


/**
  * @brief Comparator (COMP)
  */

typedef struct {                                    /*!< COMP Structure                                                        */
  __O  uint32_t  TASKS_START;                       /*!< Start comparator                                                      */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop comparator                                                       */
  __O  uint32_t  TASKS_SAMPLE;                      /*!< Sample comparator value                                               */
  __I  uint32_t  RESERVED0[29];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_SAMPLE;                  /*!< Subscribe configuration for TASKS_SAMPLE                              */
  __I  uint32_t  RESERVED1[29];
  __IO uint32_t  EVENTS_READY;                      /*!< COMP is ready and output is valid                                     */
  __IO uint32_t  EVENTS_DOWN;                       /*!< Downward crossing                                                     */
  __IO uint32_t  EVENTS_UP;                         /*!< Upward crossing                                                       */
  __IO uint32_t  EVENTS_CROSS;                      /*!< Downward or upward crossing                                           */
  __I  uint32_t  RESERVED2[28];
  __IO uint32_t  PUBLISH_READY;                     /*!< Publish configuration for EVENTS_READY                                */
  __IO uint32_t  PUBLISH_DOWN;                      /*!< Publish configuration for EVENTS_DOWN                                 */
  __IO uint32_t  PUBLISH_UP;                        /*!< Publish configuration for EVENTS_UP                                   */
  __IO uint32_t  PUBLISH_CROSS;                     /*!< Publish configuration for EVENTS_CROSS                                */
  __I  uint32_t  RESERVED3[28];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[61];
  __I  uint32_t  RESULT;                            /*!< Compare result                                                        */
  __I  uint32_t  BGREADY;                           /*!< COMP module is ready                                                  */
  __I  uint32_t  RESERVED6[62];
  __IO uint32_t  ENABLE;                            /*!< COMP enable                                                           */
  __IO uint32_t  PSEL;                              /*!< Pin select                                                            */
  __IO uint32_t  REFSEL;                            /*!< Reference source select                                               */
  __IO uint32_t  EXTREFSEL;                         /*!< External reference select                                             */
  __I  uint32_t  RESERVED7[8];
  __IO uint32_t  TH;                                /*!< Threshold configuration for hysteresis unit                           */
  __IO uint32_t  MODE;                              /*!< Mode configuration                                                    */
  __IO uint32_t  HYST;                              /*!< Comparator hysteresis enable                                          */
  __IO uint32_t  ISOURCE;                           /*!< Current source select on analog input                                 */
  __IO uint32_t  REFTRIM;                           /*!< Trim internal band gap reference, copied from FICR (COMPREFTRIM
                                                         register) at reset                                                    */
  __IO uint32_t  ATECOMP;                           /*!< Analog test bus control for comparator                                */
  __I  uint32_t  RESERVED8[685];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_COMP_Type;


/* ================================================================================ */
/* ================                     LPCOMP                     ================ */
/* ================================================================================ */


/**
  * @brief Low Power Comparator (LPCOMP)
  */

typedef struct {                                    /*!< LPCOMP Structure                                                      */
  __O  uint32_t  TASKS_START;                       /*!< Start comparator                                                      */
  __O  uint32_t  TASKS_STOP;                        /*!< Stop comparator                                                       */
  __O  uint32_t  TASKS_SAMPLE;                      /*!< Sample comparator value                                               */
  __I  uint32_t  RESERVED0[29];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_SAMPLE;                  /*!< Subscribe configuration for TASKS_SAMPLE                              */
  __I  uint32_t  RESERVED1[29];
  __IO uint32_t  EVENTS_READY;                      /*!< LPCOMP is ready and output is valid                                   */
  __IO uint32_t  EVENTS_DOWN;                       /*!< Downward crossing                                                     */
  __IO uint32_t  EVENTS_UP;                         /*!< Upward crossing                                                       */
  __IO uint32_t  EVENTS_CROSS;                      /*!< Downward or upward crossing                                           */
  __I  uint32_t  RESERVED2[28];
  __IO uint32_t  PUBLISH_READY;                     /*!< Publish configuration for EVENTS_READY                                */
  __IO uint32_t  PUBLISH_DOWN;                      /*!< Publish configuration for EVENTS_DOWN                                 */
  __IO uint32_t  PUBLISH_UP;                        /*!< Publish configuration for EVENTS_UP                                   */
  __IO uint32_t  PUBLISH_CROSS;                     /*!< Publish configuration for EVENTS_CROSS                                */
  __I  uint32_t  RESERVED3[28];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[61];
  __I  uint32_t  RESULT;                            /*!< Compare result                                                        */
  __I  uint32_t  RESERVED6[63];
  __IO uint32_t  ENABLE;                            /*!< Enable LPCOMP                                                         */
  __IO uint32_t  PSEL;                              /*!< Input pin select                                                      */
  __IO uint32_t  REFSEL;                            /*!< Reference select                                                      */
  __IO uint32_t  EXTREFSEL;                         /*!< External reference select                                             */
  __I  uint32_t  RESERVED7[4];
  __IO uint32_t  ANADETECT;                         /*!< Analog detect configuration                                           */
  __I  uint32_t  RESERVED8[5];
  __IO uint32_t  HYST;                              /*!< Comparator hysteresis enable                                          */
  __I  uint32_t  RESERVED9[688];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_LPCOMP_Type;


/* ================================================================================ */
/* ================                       SWI                      ================ */
/* ================================================================================ */


/**
  * @brief Software interrupt (SWI)
  */

typedef struct {                                    /*!< SWI Structure                                                         */
  __I  uint32_t  UNUSED;                            /*!< Unused.                                                               */
} NRF_SWI_Type;


/* ================================================================================ */
/* ================                       EGU                      ================ */
/* ================================================================================ */


/**
  * @brief Event Generator Unit (EGU)
  */

typedef struct {                                    /*!< EGU Structure                                                         */
  __O  uint32_t  TASKS_TRIGGER[16];                 /*!< Description collection[0]: Trigger 0 for triggering the corresponding
                                                         TRIGGERED[0] event                                                    */
  __I  uint32_t  RESERVED0[16];
  __IO uint32_t  SUBSCRIBE_TRIGGER[16];             /*!< Description collection[0]: Subscribe configuration for TASKS_TRIGGER[0] */
  __I  uint32_t  RESERVED1[16];
  __IO uint32_t  EVENTS_TRIGGERED[16];              /*!< Description collection[0]: Event number 0 generated by triggering
                                                         the corresponding TRIGGER[0] task                                     */
  __I  uint32_t  RESERVED2[16];
  __IO uint32_t  PUBLISH_TRIGGERED[16];             /*!< Description collection[0]: Publish configuration for EVENTS_TRIGGERED[0] */
  __I  uint32_t  RESERVED3[80];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
} NRF_EGU_Type;


/* ================================================================================ */
/* ================                       PWM                      ================ */
/* ================================================================================ */


/**
  * @brief Pulse Width Modulation Unit (PWM)
  */

typedef struct {                                    /*!< PWM Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Starts PWM pulse generation with the last loaded values               */
  __O  uint32_t  TASKS_STOP;                        /*!< Stops PWM pulse generation on all channels at the end of current
                                                         PWM period, and stops sequence playback                               */
  __O  uint32_t  TASKS_SEQSTART[2];                 /*!< Description collection[0]: Loads the first PWM value on all
                                                         enabled channels from sequence 0, and starts playing that sequence
                                                          at the rate defined in SEQ[0]REFRESH and/or DECODER.MODE. Causes
                                                          PWM generation to start it was not running. Sending a SEQSTART[0]
                                                          task while a sequence is already playing back will gracefully
                                                          (i.e. glitch-free) abort that sequence at the earliest opportunity
                                                          and start sequence 0.                                                */
  __O  uint32_t  TASKS_NEXTSTEP;                    /*!< Steps by one value in the current sequence on all enabled channels
                                                         if DECODER.MODE=NextStep. Does not cause PWM generation to start
                                                          it was not running.                                                  */
  __O  uint32_t  TASKS_SEQABORT;                    /*!< (Gracefully) aborts the playback of the current sequence or
                                                         the current end delay.                                                */
  __I  uint32_t  RESERVED0[26];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_SEQSTART[2];             /*!< Description collection[0]: Subscribe configuration for TASKS_SEQSTART[0] */
  __IO uint32_t  SUBSCRIBE_NEXTSTEP;                /*!< Subscribe configuration for TASKS_NEXTSTEP                            */
  __IO uint32_t  SUBSCRIBE_SEQABORT;                /*!< Subscribe configuration for TASKS_SEQABORT                            */
  __I  uint32_t  RESERVED1[26];
  __IO uint32_t  EVENTS_STARTED;                    /*!< Response to START task, emitted when PWM pulses start to get
                                                         generated                                                             */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< Response to STOP task, emitted when PWM pulses are no longer
                                                         generated                                                             */
  __IO uint32_t  EVENTS_SEQSTARTED[2];              /*!< Description collection[0]: First PWM period started on sequence
                                                         0                                                                     */
  __IO uint32_t  EVENTS_SEQEND[2];                  /*!< Description collection[0]: Emitted at end of every sequence
                                                         0, when last value from RAM has been applied to wave counter          */
  __IO uint32_t  EVENTS_PWMPERIODEND;               /*!< Emitted at the end of each PWM period                                 */
  __IO uint32_t  EVENTS_LOOPSDONE;                  /*!< Concatenated sequences have been played the amount of times
                                                         defined in LOOP.CNT                                                   */
  __IO uint32_t  EVENTS_RAMUNDERFLOW;               /*!< Emitted when a RAM fecth does not complete in time for the PWM
                                                         module                                                                */
  __IO uint32_t  EVENTS_DMAREADY[2];                /*!< Description collection[0]: Emitted when EasyDMA has fetched
                                                         .PTR and .CNT registers for sequence 0 (they can be prepared
                                                          for the next sequence)                                               */
  __IO uint32_t  EVENTS_DMAEND[2];                  /*!< Description collection[0]: Emitted when EasyDMA has finished
                                                         fetching values of sequence 0 from RAM buffer                         */
  __I  uint32_t  RESERVED2[19];
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __IO uint32_t  PUBLISH_SEQSTARTED[2];             /*!< Description collection[0]: Publish configuration for EVENTS_SEQSTARTED[0] */
  __IO uint32_t  PUBLISH_SEQEND[2];                 /*!< Description collection[0]: Publish configuration for EVENTS_SEQEND[0] */
  __IO uint32_t  PUBLISH_PWMPERIODEND;              /*!< Publish configuration for EVENTS_PWMPERIODEND                         */
  __IO uint32_t  PUBLISH_LOOPSDONE;                 /*!< Publish configuration for EVENTS_LOOPSDONE                            */
  __IO uint32_t  PUBLISH_RAMUNDERFLOW;              /*!< Publish configuration for EVENTS_RAMUNDERFLOW                         */
  __IO uint32_t  PUBLISH_DMAREADY[2];               /*!< Description collection[0]: Publish configuration for EVENTS_DMAREADY[0] */
  __IO uint32_t  PUBLISH_DMAEND[2];                 /*!< Description collection[0]: Publish configuration for EVENTS_DMAEND[0] */
  __I  uint32_t  RESERVED3[19];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[125];
  __IO uint32_t  ENABLE;                            /*!< PWM module enable register                                            */
  __IO uint32_t  MODE;                              /*!< Selects operating mode of the wave counter                            */
  __IO uint32_t  COUNTERTOP;                        /*!< Value up to which the pulse generator counter counts                  */
  __IO uint32_t  PRESCALER;                         /*!< Configuration for PWM_CLK                                             */
  __IO uint32_t  DECODER;                           /*!< Configuration of the decoder                                          */
  __IO uint32_t  LOOP;                              /*!< Amount of playback of a loop                                          */
  __I  uint32_t  RESERVED6[2];
  PWM_SEQ_Type SEQ[2];                              /*!< Unspecified                                                           */
  PWM_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
} NRF_PWM_Type;


/* ================================================================================ */
/* ================                       PDM                      ================ */
/* ================================================================================ */


/**
  * @brief Pulse Density Modulation (Digital Microphone) Interface (PDM)
  */

typedef struct {                                    /*!< PDM Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Starts continuous PDM transfer                                        */
  __O  uint32_t  TASKS_STOP;                        /*!< Stops PDM transfer                                                    */
  __I  uint32_t  RESERVED0[30];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  EVENTS_STARTED;                    /*!< PDM transfer has started                                              */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< PDM transfer has finished                                             */
  __IO uint32_t  EVENTS_END;                        /*!< The PDM has written the last sample specified by SAMPLE.MAXCNT
                                                         (or the last sample after a STOP task has been received) to
                                                          Data RAM                                                             */
  __IO uint32_t  EVENTS_OVERRUN;                    /*!< PDM samples lost due to DMA destination not available                 */
  __I  uint32_t  RESERVED2[28];
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __IO uint32_t  PUBLISH_END;                       /*!< Publish configuration for EVENTS_END                                  */
  __IO uint32_t  PUBLISH_OVERRUN;                   /*!< Publish configuration for EVENTS_OVERRUN                              */
  __I  uint32_t  RESERVED3[92];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[125];
  __IO uint32_t  ENABLE;                            /*!< PDM module enable register                                            */
  __IO uint32_t  PDMCLKCTRL;                        /*!< PDM clock generator control                                           */
  __IO uint32_t  MODE;                              /*!< Defines the routing of the connected PDM microphones' signals         */
  __IO uint32_t  FDBYPASS;                          /*!< Bypass the frequency doubler in the PDM clock generation              */
  __I  uint32_t  RXDLDATA;                          /*!< Left sample from filter output                                        */
  __I  uint32_t  RXDRDATA;                          /*!< Right sample from filter output                                       */
  __IO uint32_t  GAINL;                             /*!< Left output gain adjustment                                           */
  __IO uint32_t  GAINR;                             /*!< Right output gain adjustment                                          */
  __I  uint32_t  RESERVED5[2];
  PDM_FILTER_Type FILTER;                           /*!< Unspecified                                                           */
  __IO uint32_t  PHASE;                             /*!< Selection of delay on the clock line (to avoid potential setup&amp;hold
                                                         timing violation on input data)                                       */
  PDM_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
  __IO uint32_t  TSEL;                              /*!< Trim value for ramTSEL[1:0] on the RamBist interface                  */
  __I  uint32_t  RESERVED6[5];
  PDM_SAMPLE_Type SAMPLE;                           /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[676];
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_PDM_Type;


/* ================================================================================ */
/* ================                       I2S                      ================ */
/* ================================================================================ */


/**
  * @brief Inter-IC Sound (I2S)
  */

typedef struct {                                    /*!< I2S Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Starts continuous I2S transfer. Also starts MCK generator when
                                                         this is enabled.                                                      */
  __O  uint32_t  TASKS_STOP;                        /*!< Stops I2S transfer. Also stops MCK generator. Triggering this
                                                         task will cause the {event:STOPPED} event to be generated.            */
  __I  uint32_t  RESERVED0[10];
  __O  uint32_t  TASKS_DMA_START_TX;                /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_TASK_TRIG_START_TX.           */
  __O  uint32_t  TASKS_DMA_START_RX;                /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_TASK_TRIG_START_RX.           */
  __O  uint32_t  TASKS_DMA_STOP_TX;                 /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_TASK_TRIG_STOP_TX.            */
  __O  uint32_t  TASKS_DMA_STOP_RX;                 /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_TASK_TRIG_STOP_RX.            */
  __I  uint32_t  RESERVED1[16];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED2[10];
  __IO uint32_t  SUBSCRIBE_DMA_START_TX;            /*!< Subscribe configuration for TASKS_DMA_START_TX                        */
  __IO uint32_t  SUBSCRIBE_DMA_START_RX;            /*!< Subscribe configuration for TASKS_DMA_START_RX                        */
  __IO uint32_t  SUBSCRIBE_DMA_STOP_TX;             /*!< Subscribe configuration for TASKS_DMA_STOP_TX                         */
  __IO uint32_t  SUBSCRIBE_DMA_STOP_RX;             /*!< Subscribe configuration for TASKS_DMA_STOP_RX                         */
  __I  uint32_t  RESERVED3[16];
  __IO uint32_t  EVENTS_DMA_END_TX;                 /*!< Equivalent to eventDmaEndTx                                           */
  __IO uint32_t  EVENTS_RXPTRUPD;                   /*!< The RXD.PTR register has been copied to internal double-buffers.
                                                         When the I2S module is started and RX is enabled, this event
                                                          will be generated for every RXTXD.MAXCNT words that are received
                                                          on the SDIN pin. Equivalent to eventDmaReadyRx.                      */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< I2S transfer stopped. There are no requirements to how "graceful"
                                                         the I2S module shall be stopped. The reason for this is that
                                                          we are talking about streaming audio data here, and thus loosing
                                                          some data is not critical. Graceful stopping of stream can easily
                                                          be done in FW. Equivalent to eventStopped                            */
  __IO uint32_t  EVENTS_ERROR_TX;                   /*!< DMA read from memory failed. Equivalent to eventUnderflowTx           */
  __IO uint32_t  EVENTS_ERROR_RX;                   /*!< DMA write to memory failed. Equivalent to eventOverflowRx             */
  __IO uint32_t  EVENTS_TXPTRUPD;                   /*!< The TDX.PTR register has been copied to internal double-buffers.
                                                         When the I2S module is started and TX is enabled, this event
                                                          will be generated for every RXTXD.MAXCNT words that are sent
                                                          on the SDOUT pin. Equivalent to eventDmaReadyTx.                     */
  __IO uint32_t  EVENTS_DMA_END_RX;                 /*!< DMA write to memory failed. Equivalent to eventDmaEndRx               */
  __I  uint32_t  RESERVED4[25];
  __IO uint32_t  PUBLISH_DMA_END_TX;                /*!< Publish configuration for EVENTS_DMA_END_TX                           */
  __IO uint32_t  PUBLISH_RXPTRUPD;                  /*!< Publish configuration for EVENTS_RXPTRUPD                             */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __IO uint32_t  PUBLISH_ERROR_TX;                  /*!< Publish configuration for EVENTS_ERROR_TX                             */
  __IO uint32_t  PUBLISH_ERROR_RX;                  /*!< Publish configuration for EVENTS_ERROR_RX                             */
  __IO uint32_t  PUBLISH_TXPTRUPD;                  /*!< Publish configuration for EVENTS_TXPTRUPD                             */
  __IO uint32_t  PUBLISH_DMA_END_RX;                /*!< Publish configuration for EVENTS_DMA_END_RX                           */
  __I  uint32_t  RESERVED5[89];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED6[125];
  __IO uint32_t  ENABLE;                            /*!< Enable I2S module.                                                    */
  I2S_CONFIG_Type CONFIG;                           /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[3];
  I2S_RXD_Type RXD;                                 /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED8;
  I2S_TXD_Type TXD;                                 /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED9[3];
  I2S_RXTXD_Type RXTXD;                             /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED10[2];
  I2S_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED11[99];
  __IO uint32_t  DMA_ENABLE_RX;                     /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_ENABLE_RX.                    */
  __IO uint32_t  DMA_ENABLE_TX;                     /*!< Equivalent to ID_DMA_CHANNEL_PERIPHERAL_ENABLE_TX.                    */
} NRF_I2S_Type;


/* ================================================================================ */
/* ================                       IPC                      ================ */
/* ================================================================================ */


/**
  * @brief Inter Processor Communication (IPC)
  */

typedef struct {                                    /*!< IPC Structure                                                         */
  __O  uint32_t  TASKS_SEND[8];                     /*!< Description collection[0]: Trigger events on channel enabled
                                                         in SEND_CNF[0].                                                       */
  __I  uint32_t  RESERVED0[24];
  __IO uint32_t  SUBSCRIBE_SEND[8];                 /*!< Description collection[0]: Subscribe configuration for TASKS_SEND[0]  */
  __I  uint32_t  RESERVED1[24];
  __IO uint32_t  EVENTS_RECEIVE[8];                 /*!< Description collection[0]: Event received on one or more of
                                                         the enabled channels in RECEIVE_CNF[{n}].                             */
  __I  uint32_t  RESERVED2[24];
  __IO uint32_t  PUBLISH_RECEIVE[8];                /*!< Description collection[0]: Publish configuration for EVENTS_RECEIVE[0] */
  __I  uint32_t  RESERVED3[88];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  INTPEND;                           /*!< Pending status of interrupt                                           */
  __I  uint32_t  RESERVED4[128];
  __IO uint32_t  SEND_CNF[8];                       /*!< Description collection[0]: Send event configuration for TASKS_SEND[0]. */
  __I  uint32_t  RESERVED5[24];
  __IO uint32_t  RECEIVE_CNF[8];                    /*!< Description collection[0]: Receive event configuration for EVENTS_RECEIVE[
                                                         0].                                                                   */
  __I  uint32_t  RESERVED6[24];
  __IO uint32_t  GPMEM[2];                          /*!< Description collection[0]: General purpose memory.                    */
} NRF_IPC_Type;


/* ================================================================================ */
/* ================                      QSPI                      ================ */
/* ================================================================================ */


/**
  * @brief External Flash interface (QSPI)
  */

typedef struct {                                    /*!< QSPI Structure                                                        */
  __O  uint32_t  TASKS_ACTIVATE;                    /*!< Activate QSPI interface.                                              */
  __O  uint32_t  TASKS_READSTART;                   /*!< Start transfer from external Flash memory to internal RAM.            */
  __O  uint32_t  TASKS_WRITESTART;                  /*!< Start transfer from internal RAM to external Flash memory.            */
  __O  uint32_t  TASKS_ERASESTART;                  /*!< Start external Flash memory erase operation.                          */
  __O  uint32_t  TASKS_DEACTIVATE;                  /*!< Deactivate QSPI interface.                                            */
  __I  uint32_t  RESERVED0[27];
  __IO uint32_t  SUBSCRIBE_ACTIVATE;                /*!< Subscribe configuration for TASKS_ACTIVATE                            */
  __IO uint32_t  SUBSCRIBE_READSTART;               /*!< Subscribe configuration for TASKS_READSTART                           */
  __IO uint32_t  SUBSCRIBE_WRITESTART;              /*!< Subscribe configuration for TASKS_WRITESTART                          */
  __IO uint32_t  SUBSCRIBE_ERASESTART;              /*!< Subscribe configuration for TASKS_ERASESTART                          */
  __IO uint32_t  SUBSCRIBE_DEACTIVATE;              /*!< Subscribe configuration for TASKS_DEACTIVATE                          */
  __I  uint32_t  RESERVED1[27];
  __IO uint32_t  EVENTS_READY;                      /*!< QSPI peripheral is ready. This event will be generated as a
                                                         response to any QSPI task.                                            */
  __I  uint32_t  RESERVED2[31];
  __IO uint32_t  PUBLISH_READY;                     /*!< Publish configuration for EVENTS_READY                                */
  __I  uint32_t  RESERVED3[95];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[125];
  __IO uint32_t  ENABLE;                            /*!< Enable QSPI peripheral and acquire pins selected in PSELn registers.  */
  QSPI_READ_Type READ;                              /*!< Unspecified                                                           */
  QSPI_WRITE_Type WRITE;                            /*!< Unspecified                                                           */
  QSPI_ERASE_Type ERASE;                            /*!< Unspecified                                                           */
  QSPI_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __IO uint32_t  XIPOFFSET;                         /*!< Address offset into the external memory for Execute in Place
                                                         operation.                                                            */
  __IO uint32_t  IFCONFIG0;                         /*!< Interface configuration. SPI MEM CTRL Default Memory reg.             */
  __I  uint32_t  RESERVED5[46];
  __IO uint32_t  IFCONFIG1;                         /*!< Interface configuration. SPI MEM CTRL Control reg.                    */
  __I  uint32_t  STATUS;                            /*!< Status register. SPI MEM CTRL Status reg.                             */
  __IO uint32_t  ACCESSREQ0;                        /*!< Access request, word 0. SPI MEM CTRL Access Request reg. 0.           */
  __IO uint32_t  ACCESSREQ1;                        /*!< Access request, word 1. SPI MEM CTRL Access Request reg. 1.           */
  __IO uint32_t  ACCESSREQ2;                        /*!< Access request, word 2. SPI MEM CTRL Access Request reg. 2.           */
  __IO uint32_t  DPMDUR;                            /*!< Set the duration required to enter/exit Deep Power-down Mode
                                                         (DPM). SPI MEM CTRL Duration DPM reg.                                 */
  __IO uint32_t  DATARW;                            /*!< Read data when read, write data when written. SPI MEM CTRL Read/Write
                                                         Data reg.                                                             */
  __I  uint32_t  FIFOSTAT;                          /*!< The number of items in Read and Write FIFOs. SPI MEM CTRL FIFOs
                                                         Status reg.                                                           */
  __IO uint32_t  DEFMEM;                            /*!< Default memory register. SPI MEM Default memory register. Fields
                                                         adn content equivalent to IFCONFIG0.                                  */
  __IO uint32_t  ADDRCONFIG;                        /*!< Extended address configuration. SPI MEM CTRL Extended Addressing
                                                         Mode reg.                                                             */
  __IO uint32_t  MEMSPEC;                           /*!< Byte specification: 1 byte manufacture ID, 1 byte memory type,
                                                         1 byte density. SPI MEM CTRL Memory Specification reg.                */
  __IO uint32_t  IRQMSK;                            /*!< Interrupts enable disable mask. SPI MEM CTRL Interrupt Mask
                                                         reg.                                                                  */
  __IO uint32_t  IRQREQ;                            /*!< Interrupt request register (reason for triggering interrupt
                                                         signal). SPI MEM CTRL Interrupt Request reg.                          */
  __IO uint32_t  CINSTRCONF;                        /*!< Custom instruction configuration register. SPI MEM CTRL Custom
                                                         Instruction Setup reg.                                                */
  __IO uint32_t  CINSTRDAT0;                        /*!< Custom instruction data register 0. SPI MEM CTRL Custom Instruction
                                                         Data reg. 0.                                                          */
  __IO uint32_t  CINSTRDAT1;                        /*!< Custom instruction data register 1. SPI MEM CTRL Custom Instruction
                                                         Data reg. 1.                                                          */
  __IO uint32_t  READCYCLES;                        /*!< Number of dummy cycles for Read Instructions. SPI MEM CTRL Read
                                                         Dummy Cycles reg.                                                     */
} NRF_QSPI_Type;


/* ================================================================================ */
/* ================                      USBD                      ================ */
/* ================================================================================ */


/**
  * @brief Universal Serial Bus device (USBD)
  */

typedef struct {                                    /*!< USBD Structure                                                        */
  __O  uint32_t  TASKS_USBDREINIT;                  /*!< Resets the whole USBD peripheral. This includes disabling the
                                                         pull-up on D+.                                                        */
  __O  uint32_t  TASKS_STARTEPIN[8];                /*!< Description collection[0]: Captures the EPIN[0].PTR, EPIN[0].MAXCNT
                                                         and EPIN[0].CONFIG registers values, and enables endpoint IN
                                                          0 to respond to traffic from host                                    */
  __O  uint32_t  TASKS_STARTISOIN;                  /*!< Description collection[0]: Captures the ISOIN[0].PTR, ISOIN[0].MAXCNT
                                                         and ISOIN[0].CONFIG registers values, and enables sending data
                                                          on iso endpoint                                                      */
  __O  uint32_t  TASKS_STARTEPOUT[8];               /*!< Description collection[0]: Captures the EPOUT[0].PTR, EPOUT[0].MAXCNT
                                                         and EPOUT[0].CONFIG registers values, and enables endpoint 0
                                                          to respond to traffic from host                                      */
  __O  uint32_t  TASKS_STARTISOOUT;                 /*!< Description collection[0]: Captures the ISOOUT[0].PTR, ISOOUT[0].MAXCNT
                                                         and ISOOUT[0].CONFIG registers values, and enables receiving
                                                          of data on iso endpoint                                              */
  __O  uint32_t  TASKS_EP0RCVOUT;                   /*!< Allows OUT data stage on control endpoint 0                           */
  __O  uint32_t  TASKS_EP0STATUS;                   /*!< Allows status stage on control endpoint 0                             */
  __O  uint32_t  TASKS_EP0STALL;                    /*!< STALLs data and status stage on control endpoint 0                    */
  __O  uint32_t  TASKS_DPDMDRIVE;                   /*!< Forces D+ and D-lines to the state defined in the DPDMVALUE
                                                         register                                                              */
  __O  uint32_t  TASKS_DPDMNODRIVE;                 /*!< Stops forcing D+ and D- lines to any state (USB engine takes
                                                         control)                                                              */
  __I  uint32_t  RESERVED0[8];
  __IO uint32_t  SUBSCRIBE_USBDREINIT;              /*!< Subscribe configuration for TASKS_USBDREINIT                          */
  __IO uint32_t  SUBSCRIBE_STARTEPIN[8];            /*!< Description collection[0]: Subscribe configuration for TASKS_STARTEPIN[0] */
  __IO uint32_t  SUBSCRIBE_STARTISOIN;              /*!< Description collection[0]: Subscribe configuration for TASKS_STARTISOIN[0] */
  __IO uint32_t  SUBSCRIBE_STARTEPOUT[8];           /*!< Description collection[0]: Subscribe configuration for TASKS_STARTEPOUT[0] */
  __IO uint32_t  SUBSCRIBE_STARTISOOUT;             /*!< Description collection[0]: Subscribe configuration for TASKS_STARTISOOUT[0
                                                         ]                                                                     */
  __IO uint32_t  SUBSCRIBE_EP0RCVOUT;               /*!< Subscribe configuration for TASKS_EP0RCVOUT                           */
  __IO uint32_t  SUBSCRIBE_EP0STATUS;               /*!< Subscribe configuration for TASKS_EP0STATUS                           */
  __IO uint32_t  SUBSCRIBE_EP0STALL;                /*!< Subscribe configuration for TASKS_EP0STALL                            */
  __IO uint32_t  SUBSCRIBE_DPDMDRIVE;               /*!< Subscribe configuration for TASKS_DPDMDRIVE                           */
  __IO uint32_t  SUBSCRIBE_DPDMNODRIVE;             /*!< Subscribe configuration for TASKS_DPDMNODRIVE                         */
  __I  uint32_t  RESERVED1[8];
  __IO uint32_t  EVENTS_USBRESET;                   /*!< Signals that a USB reset condition has been detected on the
                                                         USB lines                                                             */
  __IO uint32_t  EVENTS_STARTED;                    /*!< Confirms that the EPIN[n].PTR, EPIN[n].MAXCNT, EPIN[n].CONFIG,
                                                         or EPOUT[n].PTR, EPOUT[n].MAXCNT and EPOUT[n].CONFIG registers
                                                          have been captured on all endpoints reported in the EPSTATUS
                                                          register                                                             */
  __IO uint32_t  EVENTS_ENDEPIN[8];                 /*!< Description collection[0]: The whole EPIN[0] buffer has been
                                                         consumed. The RAM buffer can be accessed safely by software.          */
  __IO uint32_t  EVENTS_EP0DATADONE;                /*!< An acknowledged data transfer has taken place on the control
                                                         endpoint                                                              */
  __IO uint32_t  EVENTS_ENDISOIN;                   /*!< Description collection[0]: The whole ISOIN[0] buffer has been
                                                         consumed. The RAM buffer can be accessed safely by software.          */
  __IO uint32_t  EVENTS_ENDEPOUT[8];                /*!< Description collection[0]: The whole EPOUT[0] buffer has been
                                                         consumed. The RAM buffer can be accessed safely by software.          */
  __IO uint32_t  EVENTS_ENDISOOUT;                  /*!< Description collection[0]: The whole ISOOUT[0] buffer has been
                                                         consumed. The RAM buffer can be accessed safely by software.          */
  __IO uint32_t  EVENTS_SOF;                        /*!< Signals that a SOF (start of frame) condition has been detected
                                                         on the USB lines                                                      */
  __IO uint32_t  EVENTS_USBEVENT;                   /*!< An event or an error not covered by specific events has occurred,
                                                         check EVENTCAUSE register to find the cause                           */
  __IO uint32_t  EVENTS_EP0SETUP;                   /*!< A valid SETUP token has been received (and acknowledged) on
                                                         the control endpoint                                                  */
  __IO uint32_t  EVENTS_EPDATA;                     /*!< A data transfer has occurred on a data endpoint, indicated by
                                                         the EPDATASTATUS register                                             */
  __IO uint32_t  EVENTS_ACCESSFAULT;                /*!< Access to an unavailable USB register has been attempted (software
                                                         or EasyDMA). This event can get fired even when USBD is not
                                                          ENABLEd.                                                             */
  __I  uint32_t  RESERVED2[6];
  __IO uint32_t  PUBLISH_USBRESET;                  /*!< Publish configuration for EVENTS_USBRESET                             */
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __IO uint32_t  PUBLISH_ENDEPIN[8];                /*!< Description collection[0]: Publish configuration for EVENTS_ENDEPIN[0] */
  __IO uint32_t  PUBLISH_EP0DATADONE;               /*!< Publish configuration for EVENTS_EP0DATADONE                          */
  __IO uint32_t  PUBLISH_ENDISOIN;                  /*!< Description collection[0]: Publish configuration for EVENTS_ENDISOIN[0] */
  __IO uint32_t  PUBLISH_ENDEPOUT[8];               /*!< Description collection[0]: Publish configuration for EVENTS_ENDEPOUT[0] */
  __IO uint32_t  PUBLISH_ENDISOOUT;                 /*!< Description collection[0]: Publish configuration for EVENTS_ENDISOOUT[0] */
  __IO uint32_t  PUBLISH_SOF;                       /*!< Publish configuration for EVENTS_SOF                                  */
  __IO uint32_t  PUBLISH_USBEVENT;                  /*!< Publish configuration for EVENTS_USBEVENT                             */
  __IO uint32_t  PUBLISH_EP0SETUP;                  /*!< Publish configuration for EVENTS_EP0SETUP                             */
  __IO uint32_t  PUBLISH_EPDATA;                    /*!< Publish configuration for EVENTS_EPDATA                               */
  __IO uint32_t  PUBLISH_ACCESSFAULT;               /*!< Publish configuration for EVENTS_ACCESSFAULT                          */
  __I  uint32_t  RESERVED3[6];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED4[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED5[61];
  __IO uint32_t  EVENTCAUSE;                        /*!< Details on event that caused the USBEVENT event                       */
  __I  uint32_t  BUSSTATE;                          /*!< Provides the logic state of the D+ and D- lines                       */
  __I  uint32_t  RESERVED6[6];
  USBD_HALTED_Type HALTED;                          /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  EPSTATUS;                          /*!< Provides information on which endpoint's EasyDMA registers have
                                                         been captured                                                         */
  __IO uint32_t  EPDATASTATUS;                      /*!< Provides information on which endpoint(s) an acknowledged data
                                                         transfer has occurred (EPDATA event)                                  */
  __I  uint32_t  USBADDR;                           /*!< Device USB address                                                    */
  __I  uint32_t  DMASTATE;                          /*!< Indicates activity state of the DMA                                   */
  __I  uint32_t  RESERVED8[2];
  __I  uint32_t  BMREQUESTTYPE;                     /*!< SETUP data, byte 0, bmRequestType                                     */
  __I  uint32_t  BREQUEST;                          /*!< SETUP data, byte 1, bRequest                                          */
  __I  uint32_t  WVALUEL;                           /*!< SETUP data, byte 2, LSB of wValue                                     */
  __I  uint32_t  WVALUEH;                           /*!< SETUP data, byte 3, MSB of wValue                                     */
  __I  uint32_t  WINDEXL;                           /*!< SETUP data, byte 4, LSB of wIndex                                     */
  __I  uint32_t  WINDEXH;                           /*!< SETUP data, byte 5, MSB of wIndex                                     */
  __I  uint32_t  WLENGTHL;                          /*!< SETUP data, byte 6, LSB of wLength                                    */
  __I  uint32_t  WLENGTHH;                          /*!< SETUP data, byte 7, MSB of wLength                                    */
  USBD_SIZE_Type SIZE;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED9[15];
  __IO uint32_t  ENABLE;                            /*!< Enable USB                                                            */
  __IO uint32_t  USBPULLUP;                         /*!< Control of the USB pull-up                                            */
  __IO uint32_t  DPDMVALUE;                         /*!< State at which the DPDMDRIVE task will force D+ and D-. The
                                                         DPDMNODRIVE task reverts the control of the lines to MAC IP
                                                          (no forcing).                                                        */
  __IO uint32_t  DTOGGLE;                           /*!< Data toggle control and status.                                       */
  __IO uint32_t  EPINEN;                            /*!< Endpoint IN enable                                                    */
  __IO uint32_t  EPOUTEN;                           /*!< Endpoint OUT enable                                                   */
  __O  uint32_t  EPSTALL;                           /*!< STALL endpoints                                                       */
  __IO uint32_t  ISOSPLIT;                          /*!< Controls the split of ISO buffers                                     */
  __I  uint32_t  FRAMECNTR;                         /*!< Returns the current value of the start of frame counter               */
  __IO uint32_t  TESTUSBPULLUP;                     /*!< Control of the USB pull-up value, for test purposes                   */
  __IO uint32_t  TSEL;                              /*!< Trim value for ramTSEL[1:0] on the RamBist interface                  */
  __IO uint32_t  LOWPOWER;                          /*!< First silicon only: Controls USBD peripheral low-power mode
                                                         during USB suspend                                                    */
  __IO uint32_t  ISOINCONFIG;                       /*!< Controls the response of the ISO IN endpoint to an IN token
                                                         when no data is ready to be sent                                      */
  __I  uint32_t  RESERVED10[51];
  USBD_EPIN_Type EPIN[8];                           /*!< Unspecified                                                           */
  USBD_ISOIN_Type ISOIN;                            /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED11[20];
  USBD_EPOUT_Type EPOUT[8];                         /*!< Unspecified                                                           */
  USBD_ISOOUT_Type ISOOUT;                          /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED12[20];
  __IO uint32_t  RAWADDR;                           /*!< Address to be used for accessing the MAC IP4000                       */
  __IO uint32_t  RAWDATA;                           /*!< Actual read or write access to the raw IP, using RAWADDR address      */
  __I  uint32_t  RESERVED13[508];
  __IO uint32_t  REGRESET;                          /*!< Registers reset. Writing any value to this register will reset
                                                         all registers in this peripheral to their reset values. Read
                                                          access to this register has no consequence. Not implemented
                                                          in first silicon.                                                    */
  __IO uint32_t  POWER;                             /*!< Peripheral power control                                              */
} NRF_USBD_Type;


/* ================================================================================ */
/* ================                      NVMC                      ================ */
/* ================================================================================ */


/**
  * @brief Non Volatile Memory Controller (NVMC)
  */

typedef struct {                                    /*!< NVMC Structure                                                        */
  __I  uint32_t  RESERVED0[256];
  __I  uint32_t  READY;                             /*!< Ready flag                                                            */
  __I  uint32_t  VERIFYFAILED;                      /*!< Flag to signal that a flash erase or write operation has failed       */
  __I  uint32_t  RESERVED1[63];
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */

  union {
    __IO uint32_t  ERASEPCR1;                       /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
    __IO uint32_t  ERASEPAGE;                       /*!< Register for erasing a page in Code area                              */
  };
  __IO uint32_t  ERASEALL;                          /*!< Register for erasing all non-volatile user memory                     */
  __IO uint32_t  ERASEPCR0;                         /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
  __IO uint32_t  ERASEUICR;                         /*!< Register for erasing User Information Configuration Registers         */
  __IO uint32_t  ERASEPAGEPARTIAL;                  /*!< Register for partial erase of a page in Code area                     */
  __IO uint32_t  ERASEPAGEPARTIALCFG;               /*!< Register for partial erase configuration                              */
  __I  uint32_t  RESERVED2[4];
  __IO uint32_t  WAITSTATENUM;                      /*!< Register to set the number of waitstate for flash access              */
  __I  uint32_t  RESERVED3[3];
  __IO uint32_t  ICACHECNF;                         /*!< I-Code cache configuration register.                                  */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  IHIT;                              /*!< I-Code cache hit counter.                                             */
  __IO uint32_t  IMISS;                             /*!< I-Code cache miss counter.                                            */
  __I  uint32_t  RESERVED5[44];
  __IO uint32_t  TESTMODE;                          /*!< Register for entering test mode. Refer to the flash IP test
                                                         documentation for more details.                                       */
  __IO uint32_t  LVEN;                              /*!< Low Voltage Read Mode                                                 */
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __IO uint32_t  UNUSED1;                           /*!< Unspecified                                                           */
  __O  uint32_t  TESTMASSERASE;                     /*!< Self-timed mass-erase operation                                       */
  __O  uint32_t  TESTERASEREFCELL;                  /*!< Self-timed erase reference cell operation                             */
  NVMC_TEST_Type TEST;                              /*!< Unspecified                                                           */
} NRF_NVMC_Type;


/* ================================================================================ */
/* ================                       VMC                      ================ */
/* ================================================================================ */


/**
  * @brief Volatile Memory controller (VMC)
  */

typedef struct {                                    /*!< VMC Structure                                                         */
  __I  uint32_t  RESERVED0[384];
  VMC_RAM_Type RAM[8];                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED1[537];
  __IO uint32_t  SRAMTRIM;                          /*!< Backdoor trim values for SRAM                                         */
  __IO uint32_t  RamSSNonRetTSel;                   /*!< Timing control values for TSMC RAMs                                   */
} NRF_VMC_Type;


/* ================================================================================ */
/* ================                      GPIO                      ================ */
/* ================================================================================ */


/**
  * @brief GPIO Port (GPIO)
  */

typedef struct {                                    /*!< GPIO Structure                                                        */
  __I  uint32_t  RESERVED0[321];
  __IO uint32_t  OUT;                               /*!< Write GPIO port                                                       */
  __IO uint32_t  OUTSET;                            /*!< Set individual bits in GPIO port                                      */
  __IO uint32_t  OUTCLR;                            /*!< Clear individual bits in GPIO port                                    */
  __I  uint32_t  IN;                                /*!< Read GPIO port                                                        */
  __IO uint32_t  DIR;                               /*!< Direction of GPIO pins                                                */
  __IO uint32_t  DIRSET;                            /*!< DIR set register                                                      */
  __IO uint32_t  DIRCLR;                            /*!< DIR clear register                                                    */
  __IO uint32_t  LATCH;                             /*!< Latch register indicating what GPIO pins that have met the criteria
                                                         set in the PIN_CNF[n].SENSE registers                                 */
  __IO uint32_t  DETECTMODE;                        /*!< Select between default DETECT signal behaviour and LDETECT mode       */
  __I  uint32_t  RESERVED1[54];
  GPIO_PIN_Type PIN[32];                            /*!< Pin 0 direct access                                                   */
  __IO uint32_t  PIN_CNF[32];                       /*!< Description collection[0]: Configuration of GPIO pins                 */
} NRF_GPIO_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define NRF_TAD_BASE                    0xE0080000UL
#define NRF_FICR_BASE                   0x00FF0000UL
#define NRF_UICR_BASE                   0x00FF8000UL
#define NRF_PMU_REGULATORS_BASE         0x40000000UL
#define NRF_PMU_OSCILLATORS_BASE        0x40000000UL
#define NRF_RADIOLTE_BASE               0x40000000UL
#define NRF_AMLI_BASE                   0x40000000UL
#define NRF_DCNF_BASE                   0x40000000UL
#define NRF_MWU_BASE                    0x40001000UL
#define NRF_REGULATORS_BASE             0x40003000UL
#define NRF_OSCILLATORS_BASE            0x40003000UL
#define NRF_CLOCK_BASE                  0x40005000UL
#define NRF_POWER_BASE                  0x40005000UL
#define NRF_PAMLI_BASE                  0x40007000UL
#define NRF_UARTE0_BASE                 0x40008000UL
#define NRF_SPIM0_BASE                  0x40008000UL
#define NRF_SPIS0_BASE                  0x40008000UL
#define NRF_TWIM0_BASE                  0x40008000UL
#define NRF_TWIS0_BASE                  0x40008000UL
#define NRF_UARTE1_BASE                 0x40009000UL
#define NRF_SPIM1_BASE                  0x40009000UL
#define NRF_SPIS1_BASE                  0x40009000UL
#define NRF_TWIM1_BASE                  0x40009000UL
#define NRF_TWIS1_BASE                  0x40009000UL
#define NRF_GPIOTE_BASE                 0x4000A000UL
#define NRF_SAADC_BASE                  0x4000B000UL
#define NRF_TIMER0_BASE                 0x4000C000UL
#define NRF_TIMER1_BASE                 0x4000D000UL
#define NRF_TIMER2_BASE                 0x4000E000UL
#define NRF_RTC0_BASE                   0x4000F000UL
#define NRF_DPPIC_BASE                  0x40010000UL
#define NRF_NFCT_BASE                   0x40011000UL
#define NRF_WDT_BASE                    0x40012000UL
#define NRF_QDEC_BASE                   0x40013000UL
#define NRF_COMP_BASE                   0x40014000UL
#define NRF_LPCOMP_BASE                 0x40014000UL
#define NRF_SWI0_BASE                   0x40015000UL
#define NRF_EGU0_BASE                   0x40015000UL
#define NRF_PWM0_BASE                   0x40016000UL
#define NRF_PDM_BASE                    0x40017000UL
#define NRF_I2S_BASE                    0x40018000UL
#define NRF_IPC_BASE                    0x40019000UL
#define NRF_EXTFLASH_BASE               0x4001A000UL
#define NRF_RTC1_BASE                   0x4001B000UL
#define NRF_USBD_BASE                   0x4001F000UL
#define NRF_NVMC_BASE                   0x40030000UL
#define NRF_VMC_BASE                    0x40031000UL
#define NRF_GPIO_BASE                   0x40800000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define NRF_TAD                         ((NRF_TAD_Type            *) NRF_TAD_BASE)
#define NRF_FICR                        ((NRF_FICR_Type           *) NRF_FICR_BASE)
#define NRF_UICR                        ((NRF_UICR_Type           *) NRF_UICR_BASE)
#define NRF_PMU_REGULATORS              ((NRF_PMU_REGULATORS_Type *) NRF_PMU_REGULATORS_BASE)
#define NRF_PMU_OSCILLATORS             ((NRF_PMU_OSCILLATORS_Type*) NRF_PMU_OSCILLATORS_BASE)
#define NRF_RADIOLTE                    ((NRF_RADIOLTE_Type       *) NRF_RADIOLTE_BASE)
#define NRF_AMLI                        ((NRF_AMLI_Type           *) NRF_AMLI_BASE)
#define NRF_DCNF                        ((NRF_DCNF_Type           *) NRF_DCNF_BASE)
#define NRF_MWU                         ((NRF_MWU_Type            *) NRF_MWU_BASE)
#define NRF_REGULATORS                  ((NRF_REGULATORS_Type     *) NRF_REGULATORS_BASE)
#define NRF_OSCILLATORS                 ((NRF_OSCILLATORS_Type    *) NRF_OSCILLATORS_BASE)
#define NRF_CLOCK                       ((NRF_CLOCK_Type          *) NRF_CLOCK_BASE)
#define NRF_POWER                       ((NRF_POWER_Type          *) NRF_POWER_BASE)
#define NRF_PAMLI                       ((NRF_PAMLI_Type          *) NRF_PAMLI_BASE)
#define NRF_UARTE0                      ((NRF_UARTE_Type          *) NRF_UARTE0_BASE)
#define NRF_SPIM0                       ((NRF_SPIM_Type           *) NRF_SPIM0_BASE)
#define NRF_SPIS0                       ((NRF_SPIS_Type           *) NRF_SPIS0_BASE)
#define NRF_TWIM0                       ((NRF_TWIM_Type           *) NRF_TWIM0_BASE)
#define NRF_TWIS0                       ((NRF_TWIS_Type           *) NRF_TWIS0_BASE)
#define NRF_UARTE1                      ((NRF_UARTE_Type          *) NRF_UARTE1_BASE)
#define NRF_SPIM1                       ((NRF_SPIM_Type           *) NRF_SPIM1_BASE)
#define NRF_SPIS1                       ((NRF_SPIS_Type           *) NRF_SPIS1_BASE)
#define NRF_TWIM1                       ((NRF_TWIM_Type           *) NRF_TWIM1_BASE)
#define NRF_TWIS1                       ((NRF_TWIS_Type           *) NRF_TWIS1_BASE)
#define NRF_GPIOTE                      ((NRF_GPIOTE_Type         *) NRF_GPIOTE_BASE)
#define NRF_SAADC                       ((NRF_SAADC_Type          *) NRF_SAADC_BASE)
#define NRF_TIMER0                      ((NRF_TIMER_Type          *) NRF_TIMER0_BASE)
#define NRF_TIMER1                      ((NRF_TIMER_Type          *) NRF_TIMER1_BASE)
#define NRF_TIMER2                      ((NRF_TIMER_Type          *) NRF_TIMER2_BASE)
#define NRF_RTC0                        ((NRF_RTC_Type            *) NRF_RTC0_BASE)
#define NRF_DPPIC                       ((NRF_DPPIC_Type          *) NRF_DPPIC_BASE)
#define NRF_NFCT                        ((NRF_NFCT_Type           *) NRF_NFCT_BASE)
#define NRF_WDT                         ((NRF_WDT_Type            *) NRF_WDT_BASE)
#define NRF_QDEC                        ((NRF_QDEC_Type           *) NRF_QDEC_BASE)
#define NRF_COMP                        ((NRF_COMP_Type           *) NRF_COMP_BASE)
#define NRF_LPCOMP                      ((NRF_LPCOMP_Type         *) NRF_LPCOMP_BASE)
#define NRF_SWI0                        ((NRF_SWI_Type            *) NRF_SWI0_BASE)
#define NRF_EGU0                        ((NRF_EGU_Type            *) NRF_EGU0_BASE)
#define NRF_PWM0                        ((NRF_PWM_Type            *) NRF_PWM0_BASE)
#define NRF_PDM                         ((NRF_PDM_Type            *) NRF_PDM_BASE)
#define NRF_I2S                         ((NRF_I2S_Type            *) NRF_I2S_BASE)
#define NRF_IPC                         ((NRF_IPC_Type            *) NRF_IPC_BASE)
#define NRF_EXTFLASH                    ((NRF_QSPI_Type           *) NRF_EXTFLASH_BASE)
#define NRF_RTC1                        ((NRF_RTC_Type            *) NRF_RTC1_BASE)
#define NRF_USBD                        ((NRF_USBD_Type           *) NRF_USBD_BASE)
#define NRF_NVMC                        ((NRF_NVMC_Type           *) NRF_NVMC_BASE)
#define NRF_VMC                         ((NRF_VMC_Type            *) NRF_VMC_BASE)
#define NRF_GPIO                        ((NRF_GPIO_Type           *) NRF_GPIO_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group nrf9120_MLM1 */
/** @} */ /* End of group Nordic Semiconductor */

#ifdef __cplusplus
}
#endif


#endif  /* nrf9120_MLM1_H */

