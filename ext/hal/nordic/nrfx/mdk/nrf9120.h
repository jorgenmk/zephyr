
/****************************************************************************************************//**
 * @file     nrf9120.h
 *
 * @brief    CMSIS Cortex-M4 Peripheral Access Layer Header File for
 *           nrf9120 from Nordic Semiconductor.
 *
 * @version  V1
 * @date     8. March 2018
 *
 * @note     Generated with SVDConv V2.81d 
 *           from CMSIS SVD File 'nrf9120.svd' Version 1,
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

/** @addtogroup nrf9120
  * @{
  */

#ifndef NRF9120_H
#define NRF9120_H

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
/* ---------------------  nrf9120 Specific Interrupt Numbers  --------------------- */
  SPU_IRQn                      =   3,              /*!<   3  SPU                                                              */
  CLOCK_POWER_IRQn              =   5,              /*!<   5  CLOCK_POWER                                                      */
  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn=   8,         /*!<   8  UARTE0_SPIM0_SPIS0_TWIM0_TWIS0                                   */
  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn=   9,         /*!<   9  UARTE1_SPIM1_SPIS1_TWIM1_TWIS1                                   */
  UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn=  10,         /*!<  10  UARTE2_SPIM2_SPIS2_TWIM2_TWIS2                                   */
  UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn=  11,         /*!<  11  UARTE3_SPIM3_SPIS3_TWIM3_TWIS3                                   */
  GPIOTE0_IRQn                  =  13,              /*!<  13  GPIOTE0                                                          */
  SAADC_IRQn                    =  14,              /*!<  14  SAADC                                                            */
  TIMER0_IRQn                   =  15,              /*!<  15  TIMER0                                                           */
  TIMER1_IRQn                   =  16,              /*!<  16  TIMER1                                                           */
  TIMER2_IRQn                   =  17,              /*!<  17  TIMER2                                                           */
  RTC0_IRQn                     =  20,              /*!<  20  RTC0                                                             */
  RTC1_IRQn                     =  21,              /*!<  21  RTC1                                                             */
  WDT_IRQn                      =  24,              /*!<  24  WDT                                                              */
  EGU0_IRQn                     =  27,              /*!<  27  EGU0                                                             */
  EGU1_IRQn                     =  28,              /*!<  28  EGU1                                                             */
  EGU2_IRQn                     =  29,              /*!<  29  EGU2                                                             */
  EGU3_IRQn                     =  30,              /*!<  30  EGU3                                                             */
  EGU4_IRQn                     =  31,              /*!<  31  EGU4                                                             */
  EGU5_IRQn                     =  32,              /*!<  32  EGU5                                                             */
  PWM0_IRQn                     =  33,              /*!<  33  PWM0                                                             */
  PWM1_IRQn                     =  34,              /*!<  34  PWM1                                                             */
  PWM2_IRQn                     =  35,              /*!<  35  PWM2                                                             */
  PWM3_IRQn                     =  36,              /*!<  36  PWM3                                                             */
  PDM_IRQn                      =  38,              /*!<  38  PDM                                                              */
  I2S_IRQn                      =  40,              /*!<  40  I2S                                                              */
  FPU_IRQn                      =  44,              /*!<  44  FPU                                                              */
  GPIOTE1_IRQn                  =  49,              /*!<  49  GPIOTE1                                                          */
  KMU_IRQn                      =  57,              /*!<  57  KMU                                                              */
  CRYPTOCELL_IRQn               =  64               /*!<  64  CRYPTOCELL                                                       */
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
#include "system_nrf9120.h"                         /*!< nrf9120 System                                                        */


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
  __IO uint32_t  TRACEDATA0;                        /*!< Pin number configuration for TRACEDATA[0]                             */
  __IO uint32_t  TRACEDATA1;                        /*!< Pin number configuration for TRACEDATA[1]                             */
  __IO uint32_t  TRACEDATA2;                        /*!< Pin number configuration for TRACEDATA[2]                             */
  __IO uint32_t  TRACEDATA3;                        /*!< Pin number configuration for TRACEDATA[3]                             */
} TAD_PSEL_Type;

typedef struct {
  __I  uint32_t  RESERVED0;
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
  __I  uint32_t  BYTES;                             /*!< Amount of bytes for the required entropy bits                         */
  __I  uint32_t  RCCUTOFF;                          /*!< Repetition counter cutoff                                             */
  __I  uint32_t  APCUTOFF;                          /*!< Adaptive proportion cutoff                                            */
  __I  uint32_t  STARTUP;                           /*!< Amount of bytes for the startup tests                                 */
  __I  uint32_t  ROSC1;                             /*!< Sample count for ring oscillator 1                                    */
  __I  uint32_t  ROSC2;                             /*!< Sample count for ring oscillator 2                                    */
  __I  uint32_t  ROSC3;                             /*!< Sample count for ring oscillator 3                                    */
  __I  uint32_t  ROSC4;                             /*!< Sample count for ring oscillator 4                                    */
} FICR_TRNG90B_Type;

typedef struct {
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
} UICR_RFU_Type;

typedef struct {
  __IO uint32_t  DEST;                              /*!< Destination address where content of registers KEYSLOT1.VALUE[0-3]
                                                         will be pushed by KMU. Note that this address MUST match that
                                                          of a peripheral's APB mapped write-only key registers, otherwise
                                                          the KMU can push this key value into an address range which
                                                          the CPU can potentially read!                                        */
  __IO uint32_t  PERM;                              /*!< Define permissions for key slot 1. Bits 0-15 and 16-31 can only
                                                         be written once.                                                      */
} UICR_KEYSLOT1_Type;

typedef struct {
  __IO uint32_t  DEST;                              /*!< Destination address where content of registers KEYSLOT2.VALUE[0-3]
                                                         will be pushed by KMU. Note that this address MUST match that
                                                          of a peripheral's APB mapped write-only key registers, otherwise
                                                          the KMU can push this key value into an address range which
                                                          the CPU can potentially read!                                        */
  __IO uint32_t  PERM;                              /*!< Define permissions for key slot 2. Bits 0-15 and 16-31 can only
                                                         be written once.                                                      */
} UICR_KEYSLOT2_Type;

typedef struct {
  __IO uint32_t  DEST;                              /*!< Destination address where content of registers KEYSLOT3.VALUE[0-3]
                                                         will be pushed by KMU. Note that this address MUST match that
                                                          of a peripheral's APB mapped write-only key registers, otherwise
                                                          the KMU can push this key value into an address range which
                                                          the CPU can potentially read!                                        */
  __IO uint32_t  PERM;                              /*!< Define permissions for key slot 3. Bits 0-15 and 16-31 can only
                                                         be written once.                                                      */
} UICR_KEYSLOT3_Type;

typedef struct {
  __IO uint32_t  DEST;                              /*!< Destination address where content of registers KEYSLOT4.VALUE[0-3]
                                                         will be pushed by KMU. Note that this address MUST match that
                                                          of a peripheral's APB mapped write-only key registers, otherwise
                                                          the KMU can push this key value into an address range which
                                                          the CPU can potentially read!                                        */
  __IO uint32_t  PERM;                              /*!< Define permissions for key slot 4. Bits 0-15 and 16-31 can only
                                                         be written once.                                                      */
} UICR_KEYSLOT4_Type;

typedef struct {
  __IO uint32_t  VALUE[4];                          /*!< Description collection[0]: Define bits [31:0] of value assigned
                                                         to key slot 1                                                         */
} UICR_KEY1_Type;

typedef struct {
  __IO uint32_t  VALUE[4];                          /*!< Description collection[0]: Define bits [31:0] of value assigned
                                                         to key slot 2                                                         */
} UICR_KEY2_Type;

typedef struct {
  __IO uint32_t  VALUE[4];                          /*!< Description collection[0]: Define bits [31:0] of value assigned
                                                         to key slot 3                                                         */
} UICR_KEY3_Type;

typedef struct {
  __IO uint32_t  VALUE[4];                          /*!< Description collection[0]: Define bits [31:0] of value assigned
                                                         to key slot 4                                                         */
} UICR_KEY4_Type;

typedef struct {
  __IO uint32_t  CPU;                               /*!< AHB bus master priority register for CPU                              */
  __IO uint32_t  EXTRAM[2];                         /*!< Description collection[0]: AHB bus master priority register
                                                         for external RAM slave port (EXTRAMs)                                 */
} AMLI_RAMPRI_Type;

typedef struct {
  __IO uint32_t  PROTECT[2];                        /*!< Description collection[0]: Control access for Master connected
                                                         to AMLI master port EXTPERI[0]                                        */
} DCNF_EXTPERI0_Type;

typedef struct {
  __IO uint32_t  PROTECT[2];                        /*!< Description collection[0]: Control access from Master connected
                                                         to AMLI master port EXTRAM[0]                                         */
} DCNF_EXTRAM_Type;

typedef struct {
  __IO uint32_t  PROTECT;                           /*!< Description cluster[0]: Control access from Master connected
                                                         to AMLI master port EXTCODE[0]                                        */
} DCNF_EXTCODE_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: Access for bus access generated from
                                                         the external domain 0 List capabilities of the external domain
                                                          0                                                                    */
} SPU_EXTDOMAIN_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: Select between Secure and Non-Secure
                                                         attribute for DPPI channels 0 to 31. Only a Secure peripheral
                                                          can publish or subscribe to a channel with Secure attribute
                                                          set.                                                                 */
  __IO uint32_t  LOCK;                              /*!< Description cluster[0]: Prevent further modification of the
                                                         corresponding PERM register                                           */
} SPU_DPPI_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: Select between Secure and Non-Secure
                                                         attribute for pins 0 to 31 of port 0. Only a Secure peripheral
                                                          access a pin with Secure attribute set.                              */
  __IO uint32_t  LOCK;                              /*!< Description cluster[0]: Prevent further modification of the
                                                         corresponding PERM register                                           */
} SPU_GPIOPORT_Type;

typedef struct {
  __IO uint32_t  REGION;                            /*!< Description cluster[0]: Define which FLASH region can contains
                                                         the Non-Secure Callable (NSC) region 0                                */
  __IO uint32_t  SIZE;                              /*!< Description cluster[0]: Define the size of the Non-Secure Callable
                                                         (NSC) region 0                                                        */
} SPU_FLASHNSC_Type;

typedef struct {
  __IO uint32_t  REGION;                            /*!< Description cluster[0]: Define which RAM region can contains
                                                         the Non-Secure Callable (NSC) region 0                                */
  __IO uint32_t  SIZE;                              /*!< Description cluster[0]: Define the size of the Non-Secure Callable
                                                         (NSC) region 0                                                        */
} SPU_RAMNSC_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: Access permissions for FLASH region
                                                         0                                                                     */
} SPU_FLASHREGION_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: Access permissions for RAM region 0           */
} SPU_RAMREGION_Type;

typedef struct {
  __IO uint32_t  PERM;                              /*!< Description cluster[0]: List capabilities and Access permissions
                                                         for the peripheral with ID 0                                          */
} SPU_PERIPHID_Type;

typedef struct {
  __IO uint32_t  SERIO[4];                          /*!< Description collection[0]: AHB bus master priority register
                                                         for SerIOBox0                                                         */
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
  __IO uint32_t  PTR;                               /*!< Description cluster[0]: Beginning address in Data RAM of this
                                                         sequence                                                              */
  __IO uint32_t  CNT;                               /*!< Description cluster[0]: Amount of values (duty cycles) in this
                                                         sequence                                                              */
  __IO uint32_t  REFRESH;                           /*!< Description cluster[0]: Amount of additional PWM periods between
                                                         samples loaded into compare register                                  */
  __IO uint32_t  ENDDELAY;                          /*!< Description cluster[0]: Time added after the sequence                 */
  __I  uint32_t  RESERVED1[4];
} PWM_SEQ_Type;

typedef struct {
  __IO uint32_t  OUT[4];                            /*!< Description collection[0]: Output pin select for PWM channel
                                                         0                                                                     */
} PWM_PSEL_Type;

typedef struct {
  __IO uint32_t  CLK;                               /*!< Pin number configuration for PDM CLK signal                           */
  __IO uint32_t  DIN;                               /*!< Pin number configuration for PDM DIN signal                           */
} PDM_PSEL_Type;

typedef struct {
  __IO uint32_t  PTR;                               /*!< RAM address pointer to write samples to with EasyDMA                  */
  __IO uint32_t  MAXCNT;                            /*!< Number of samples to allocate memory for in EasyDMA mode              */
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
} I2S_RXTXD_Type;

typedef struct {
  __IO uint32_t  MCK;                               /*!< Pin select for MCK signal.                                            */
  __IO uint32_t  SCK;                               /*!< Pin select for SCK signal.                                            */
  __IO uint32_t  LRCK;                              /*!< Pin select for LRCK signal.                                           */
  __IO uint32_t  SDIN;                              /*!< Pin select for SDIN signal.                                           */
  __IO uint32_t  SDOUT;                             /*!< Pin select for SDOUT signal.                                          */
} I2S_PSEL_Type;

typedef struct {
  __IO uint32_t  POWER;                             /*!< Description cluster[0]: RAM0 power control register                   */
  __O  uint32_t  POWERSET;                          /*!< Description cluster[0]: RAM0 power control set register               */
  __O  uint32_t  POWERCLR;                          /*!< Description cluster[0]: RAM0 power control clear register             */
  __I  uint32_t  RESERVED2;
} VMC_RAM_Type;


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
  __I  uint32_t  RESERVED0[128];
  FICR_INFO_Type INFO;                              /*!< Device info                                                           */
  __I  uint32_t  RESERVED1[630];
  FICR_TRNG90B_Type TRNG90B;                        /*!< NIST800-90B RNG calibration data                                      */
} NRF_FICR_Type;


/* ================================================================================ */
/* ================                      UICR                      ================ */
/* ================================================================================ */


/**
  * @brief User information configuration registers (UICR)
  */

typedef struct {                                    /*!< UICR Structure                                                        */
  __IO uint32_t  APPROTECT;                         /*!< Access port protection                                                */
  __IO uint32_t  UNUSED0;                           /*!< Unspecified                                                           */
  __IO uint32_t  UNUSED1;                           /*!< Unspecified                                                           */
  __IO uint32_t  UNUSED2;                           /*!< Unspecified                                                           */
  __IO uint32_t  UNUSED3;                           /*!< Unspecified                                                           */
  __IO uint32_t  XOSC32M;                           /*!< Oscillator control                                                    */
  __I  uint32_t  RESERVED0;
  __IO uint32_t  HFXOSRC;                           /*!< HFXO clock source selection                                           */
  __IO uint32_t  HFXOCNT;                           /*!< HFXO startup counter                                                  */
  __I  uint32_t  RESERVED1[2];
  __IO uint32_t  SECUREAPPROTECT;                   /*!< Secure access port protection                                         */
  __IO uint32_t  ERASEPROTECT;                      /*!< Erase protection                                                      */
  __I  uint32_t  RESERVED2[51];
  UICR_RFU_Type RFU[2];                             /*!< Unspecified                                                           */
  __IO uint32_t  OTP[4];                            /*!< Description collection[0]: OTP bits [31:0].                           */
  __I  uint32_t  RESERVED3[186];
  UICR_KEYSLOT1_Type KEYSLOT1;                      /*!< Unspecified                                                           */
  UICR_KEYSLOT2_Type KEYSLOT2;                      /*!< Unspecified                                                           */
  UICR_KEYSLOT3_Type KEYSLOT3;                      /*!< Unspecified                                                           */
  UICR_KEYSLOT4_Type KEYSLOT4;                      /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED4[248];
  UICR_KEY1_Type KEY1;                              /*!< Unspecified                                                           */
  UICR_KEY2_Type KEY2;                              /*!< Unspecified                                                           */
  UICR_KEY3_Type KEY3;                              /*!< Unspecified                                                           */
  UICR_KEY4_Type KEY4;                              /*!< Unspecified                                                           */
} NRF_UICR_Type;


/* ================================================================================ */
/* ================                     GPIOTE                     ================ */
/* ================================================================================ */


/**
  * @brief GPIO Tasks and Events 0 (GPIOTE)
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
  __I  uint32_t  RESERVED6[65];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[129];
  __IO uint32_t  CONFIG[8];                         /*!< Description collection[0]: Configuration for OUT[n], SET[n]
                                                         and CLR[n] tasks and IN[n] event                                      */
} NRF_GPIOTE_Type;


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
  __I  uint32_t  RESERVED2[6];
  DCNF_EXTRAM_Type EXTRAM;                          /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED3[6];
  DCNF_EXTCODE_Type EXTCODE[2];                     /*!< Unspecified                                                           */
} NRF_DCNF_Type;


/* ================================================================================ */
/* ================                       SPU                      ================ */
/* ================================================================================ */


/**
  * @brief System Protection Unit (SPU)
  */

typedef struct {                                    /*!< SPU Structure                                                         */
  __I  uint32_t  RESERVED0[64];
  __IO uint32_t  EVENTS_RAMACCERR;                  /*!< A security violation has been detected for the RAM memory space       */
  __IO uint32_t  EVENTS_FLASHACCERR;                /*!< A security violation has been detected for the Flash memory
                                                         space                                                                 */
  __IO uint32_t  EVENTS_PERIPHACCERR;               /*!< A security violation has been detected on one or several peripherals  */
  __I  uint32_t  RESERVED1[29];
  __IO uint32_t  PUBLISH_RAMACCERR;                 /*!< Publish configuration for EVENTS_RAMACCERR                            */
  __IO uint32_t  PUBLISH_FLASHACCERR;               /*!< Publish configuration for EVENTS_FLASHACCERR                          */
  __IO uint32_t  PUBLISH_PERIPHACCERR;              /*!< Publish configuration for EVENTS_PERIPHACCERR                         */
  __I  uint32_t  RESERVED2[93];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED3[61];
  __I  uint32_t  CAP;                               /*!< Show implemented features for the current device                      */
  __I  uint32_t  RESERVED4[15];
  SPU_EXTDOMAIN_Type EXTDOMAIN[1];                  /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED5[15];
  SPU_DPPI_Type DPPI[1];                            /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED6[14];
  SPU_GPIOPORT_Type GPIOPORT[1];                    /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[14];
  SPU_FLASHNSC_Type FLASHNSC[2];                    /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED8[12];
  SPU_RAMNSC_Type RAMNSC[2];                        /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED9[44];
  SPU_FLASHREGION_Type FLASHREGION[32];             /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED10[32];
  SPU_RAMREGION_Type RAMREGION[32];                 /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED11[32];
  SPU_PERIPHID_Type PERIPHID[67];                   /*!< Unspecified                                                           */
} NRF_SPU_Type;


/* ================================================================================ */
/* ================                   REGULATORS                   ================ */
/* ================================================================================ */


/**
  * @brief Voltage Regulators control (REGULATORS)
  */

typedef struct {                                    /*!< REGULATORS Structure                                                  */
  __I  uint32_t  RESERVED0[320];
  __O  uint32_t  SYSTEMOFF;                         /*!< System OFF register                                                   */
  __I  uint32_t  RESERVED1[3];
  __IO uint32_t  POFCON;                            /*!< Power failure comparator configuration                                */
  __I  uint32_t  RESERVED2[25];
  __IO uint32_t  DCDCEN;                            /*!< Enable DC/DC mode of the main voltage regulator.                      */
} NRF_REGULATORS_Type;


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
  __I  uint32_t  HFCLKSTAT;                         /*!< The register shows if HFXO has been requested by triggering
                                                         HFCLKSTART task and if it has been started (STATE).                   */
  __I  uint32_t  RESERVED5;
  __I  uint32_t  LFCLKRUN;                          /*!< Status indicating that LFCLKSTART task has been triggered             */
  __I  uint32_t  LFCLKSTAT;                         /*!< The register shows which LFCLK source has been requested (SRC)
                                                         when triggering LFCLKSTART task and if the source has been started
                                                          (STATE).                                                             */
  __I  uint32_t  LFCLKSRCCOPY;                      /*!< Copy of LFCLKSRC register, set after LFCLKSTART task has been
                                                         triggered.                                                            */
  __I  uint32_t  RESERVED6[62];
  __IO uint32_t  LFCLKSRC;                          /*!< Clock source for the LFCLK. LFCLKSTART task starts starts a
                                                         clock source selected with this register.                             */
} NRF_CLOCK_Type;


/* ================================================================================ */
/* ================                      POWER                     ================ */
/* ================================================================================ */


/**
  * @brief Power control (POWER)
  */

typedef struct {                                    /*!< POWER Structure                                                       */
  __I  uint32_t  RESERVED0[30];
  __O  uint32_t  TASKS_CONSTLAT;                    /*!< Enable constant latency mode.                                         */
  __O  uint32_t  TASKS_LOWPWR;                      /*!< Enable low power mode (variable latency)                              */
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
  __I  uint32_t  RESERVED8[15];
  __I  uint32_t  POWERSTATUS;                       /*!< Modem domain power status                                             */
  __I  uint32_t  RESERVED9[54];
  __IO uint32_t  GPREGRET[2];                       /*!< Description collection[0]: General purpose retention register         */
} NRF_POWER_Type;


/* ================================================================================ */
/* ================                   CTRLAPPERI                   ================ */
/* ================================================================================ */


/**
  * @brief Control access port (CTRLAPPERI)
  */

typedef struct {                                    /*!< CTRLAPPERI Structure                                                  */
  __I  uint32_t  RESERVED0[256];
  __I  uint32_t  CTRLDAP2COREDATA;                  /*!< Data sent from the Debugger to the Core                               */
  __I  uint32_t  CTRLDAP2CORESTATUS;                /*!< Status to indicate if Data sent from the Debugger to the Core
                                                         has been read                                                         */
  __I  uint32_t  RESERVED1[30];
  __IO uint32_t  CORE2CTRLDAPDATA;                  /*!< Data sent from the Core to the Debugger                               */
  __I  uint32_t  CORE2CTRLDAPSTATUS;                /*!< Status to indicate if Data sent from the Core to the Debugger
                                                         status has been read                                                  */
  __I  uint32_t  RESERVED2[30];
  __O  uint32_t  CPURETURNFIELD_LOCK;               /*!< Lock Mail Box erase all mechanism                                     */
  __O  uint32_t  CPURETURNFIELD;                    /*!< Mail box erase all control register. A write once register.           */
} NRF_CTRLAPPERI_Type;


/* ================================================================================ */
/* ================                      PAMLI                     ================ */
/* ================================================================================ */


/**
  * @brief Peripheral AHB Multi-Layer Interface (PAMLI)
  */

typedef struct {                                    /*!< PAMLI Structure                                                       */
  __I  uint32_t  RESERVED0[896];
  PAMLI_RAMPRI_Type RAMPRI;                         /*!< RAM configurable priority configuration structure                     */
  __IO uint32_t  I2S;                               /*!< AHB bus master priority register for I2S                              */
  __I  uint32_t  RESERVED1;
  __IO uint32_t  PDM;                               /*!< AHB bus master priority register for PDM                              */
  __I  uint32_t  RESERVED2;
  __IO uint32_t  PWM[4];                            /*!< Description collection[0]: AHB bus master priority register
                                                         for PWM0                                                              */
  __IO uint32_t  SAADC;                             /*!< AHB bus master priority register for SAADC                            */
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
  __I  uint32_t  RESERVED0[7];
  __O  uint32_t  TASKS_FLUSHRX;                     /*!< Flush RX FIFO into RX buffer                                          */
  __I  uint32_t  RESERVED1[20];
  __IO uint32_t  SUBSCRIBE_STARTRX;                 /*!< Subscribe configuration for TASKS_STARTRX                             */
  __IO uint32_t  SUBSCRIBE_STOPRX;                  /*!< Subscribe configuration for TASKS_STOPRX                              */
  __IO uint32_t  SUBSCRIBE_STARTTX;                 /*!< Subscribe configuration for TASKS_STARTTX                             */
  __IO uint32_t  SUBSCRIBE_STOPTX;                  /*!< Subscribe configuration for TASKS_STOPTX                              */
  __I  uint32_t  RESERVED2[7];
  __IO uint32_t  SUBSCRIBE_FLUSHRX;                 /*!< Subscribe configuration for TASKS_FLUSHRX                             */
  __I  uint32_t  RESERVED3[20];
  __IO uint32_t  EVENTS_CTS;                        /*!< CTS is activated (set low). Clear To Send.                            */
  __IO uint32_t  EVENTS_NCTS;                       /*!< CTS is deactivated (set high). Not Clear To Send.                     */
  __IO uint32_t  EVENTS_RXDRDY;                     /*!< Data received in RXD (but potentially not yet transferred to
                                                         Data RAM)                                                             */
  __I  uint32_t  RESERVED4;
  __IO uint32_t  EVENTS_ENDRX;                      /*!< Receive buffer is filled up                                           */
  __I  uint32_t  RESERVED5[2];
  __IO uint32_t  EVENTS_TXDRDY;                     /*!< Data sent from TXD                                                    */
  __IO uint32_t  EVENTS_ENDTX;                      /*!< Last TX byte transmitted                                              */
  __IO uint32_t  EVENTS_ERROR;                      /*!< Error detected                                                        */
  __I  uint32_t  RESERVED6[7];
  __IO uint32_t  EVENTS_RXTO;                       /*!< Receiver timeout                                                      */
  __I  uint32_t  RESERVED7;
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< UART receiver has started                                             */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< UART transmitter has started                                          */
  __I  uint32_t  RESERVED8;
  __IO uint32_t  EVENTS_TXSTOPPED;                  /*!< Transmitter stopped                                                   */
  __I  uint32_t  RESERVED9[9];
  __IO uint32_t  PUBLISH_CTS;                       /*!< Publish configuration for EVENTS_CTS                                  */
  __IO uint32_t  PUBLISH_NCTS;                      /*!< Publish configuration for EVENTS_NCTS                                 */
  __IO uint32_t  PUBLISH_RXDRDY;                    /*!< Publish configuration for EVENTS_RXDRDY                               */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  PUBLISH_ENDRX;                     /*!< Publish configuration for EVENTS_ENDRX                                */
  __I  uint32_t  RESERVED11[2];
  __IO uint32_t  PUBLISH_TXDRDY;                    /*!< Publish configuration for EVENTS_TXDRDY                               */
  __IO uint32_t  PUBLISH_ENDTX;                     /*!< Publish configuration for EVENTS_ENDTX                                */
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED12[7];
  __IO uint32_t  PUBLISH_RXTO;                      /*!< Publish configuration for EVENTS_RXTO                                 */
  __I  uint32_t  RESERVED13;
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __I  uint32_t  RESERVED14;
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
  __I  uint32_t  RESERVED15[64];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED16[125];
  __IO uint32_t  ENABLE;                            /*!< Enable SPIM                                                           */
  __I  uint32_t  RESERVED17;
  SPIM_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED18[4];
  __IO uint32_t  FREQUENCY;                         /*!< SPI frequency. Accuracy depends on the HFCLK source selected.         */
  __I  uint32_t  RESERVED19[3];
  SPIM_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  SPIM_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __I  uint32_t  RESERVED20[26];
  __IO uint32_t  ORC;                               /*!< Over-read character. Character clocked out in case and over-read
                                                         of the TXD buffer.                                                    */
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
  __I  uint32_t  RESERVED9[64];
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
  __I  uint32_t  RESERVED8[7];
  __IO uint32_t  EVENTS_ERROR;                      /*!< TWI error                                                             */
  __I  uint32_t  RESERVED9[8];
  __IO uint32_t  EVENTS_SUSPENDED;                  /*!< Last byte has been sent out after the SUSPEND task has been
                                                         issued, TWI traffic is now suspended.                                 */
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< Receive sequence started                                              */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< Transmit sequence started                                             */
  __I  uint32_t  RESERVED10[2];
  __IO uint32_t  EVENTS_LASTRX;                     /*!< Byte boundary, starting to receive the last byte                      */
  __IO uint32_t  EVENTS_LASTTX;                     /*!< Byte boundary, starting to transmit the last byte                     */
  __I  uint32_t  RESERVED11[8];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED12[7];
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED13[8];
  __IO uint32_t  PUBLISH_SUSPENDED;                 /*!< Publish configuration for EVENTS_SUSPENDED                            */
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __I  uint32_t  RESERVED14[2];
  __IO uint32_t  PUBLISH_LASTRX;                    /*!< Publish configuration for EVENTS_LASTRX                               */
  __IO uint32_t  PUBLISH_LASTTX;                    /*!< Publish configuration for EVENTS_LASTTX                               */
  __I  uint32_t  RESERVED15[7];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED16[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED17[110];
  __IO uint32_t  ERRORSRC;                          /*!< Error source                                                          */
  __I  uint32_t  RESERVED18[14];
  __IO uint32_t  ENABLE;                            /*!< Enable TWIM                                                           */
  __I  uint32_t  RESERVED19;
  TWIM_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED20[5];
  __IO uint32_t  FREQUENCY;                         /*!< TWI frequency. Accuracy depends on the HFCLK source selected.         */
  __I  uint32_t  RESERVED21[3];
  TWIM_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  TWIM_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED22[13];
  __IO uint32_t  ADDRESS;                           /*!< Address used in the TWI transfer                                      */
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
  __I  uint32_t  RESERVED8[9];
  __IO uint32_t  EVENTS_RXSTARTED;                  /*!< Receive sequence started                                              */
  __IO uint32_t  EVENTS_TXSTARTED;                  /*!< Transmit sequence started                                             */
  __I  uint32_t  RESERVED9[4];
  __IO uint32_t  EVENTS_WRITE;                      /*!< Write command received                                                */
  __IO uint32_t  EVENTS_READ;                       /*!< Read command received                                                 */
  __I  uint32_t  RESERVED10[6];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED11[7];
  __IO uint32_t  PUBLISH_ERROR;                     /*!< Publish configuration for EVENTS_ERROR                                */
  __I  uint32_t  RESERVED12[9];
  __IO uint32_t  PUBLISH_RXSTARTED;                 /*!< Publish configuration for EVENTS_RXSTARTED                            */
  __IO uint32_t  PUBLISH_TXSTARTED;                 /*!< Publish configuration for EVENTS_TXSTARTED                            */
  __I  uint32_t  RESERVED13[4];
  __IO uint32_t  PUBLISH_WRITE;                     /*!< Publish configuration for EVENTS_WRITE                                */
  __IO uint32_t  PUBLISH_READ;                      /*!< Publish configuration for EVENTS_READ                                 */
  __I  uint32_t  RESERVED14[5];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED15[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED16[113];
  __IO uint32_t  ERRORSRC;                          /*!< Error source                                                          */
  __I  uint32_t  MATCH;                             /*!< Status register indicating which address had a match                  */
  __I  uint32_t  RESERVED17[10];
  __IO uint32_t  ENABLE;                            /*!< Enable TWIS                                                           */
  __I  uint32_t  RESERVED18;
  TWIS_PSEL_Type PSEL;                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED19[9];
  TWIS_RXD_Type RXD;                                /*!< RXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED20;
  TWIS_TXD_Type TXD;                                /*!< TXD EasyDMA channel                                                   */
  __I  uint32_t  RESERVED21[14];
  __IO uint32_t  ADDRESS[2];                        /*!< Description collection[0]: TWI slave address 0                        */
  __I  uint32_t  RESERVED22;
  __IO uint32_t  CONFIG;                            /*!< Configuration register for the address match mechanism                */
  __I  uint32_t  RESERVED23[10];
  __IO uint32_t  ORC;                               /*!< Over-read character. Character sent out in case of an over-read
                                                         of the transmit buffer.                                               */
} NRF_TWIS_Type;


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
  __I  uint32_t  RESERVED0[28];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_SAMPLE;                  /*!< Subscribe configuration for TASKS_SAMPLE                              */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_CALIBRATEOFFSET;         /*!< Subscribe configuration for TASKS_CALIBRATEOFFSET                     */
  __I  uint32_t  RESERVED1[28];
  __IO uint32_t  EVENTS_STARTED;                    /*!< The ADC has started                                                   */
  __IO uint32_t  EVENTS_END;                        /*!< The ADC has filled up the Result buffer                               */
  __IO uint32_t  EVENTS_DONE;                       /*!< A conversion task has been completed. Depending on the mode,
                                                         multiple conversions might be needed for a result to be transferred
                                                          to RAM.                                                              */
  __IO uint32_t  EVENTS_RESULTDONE;                 /*!< A result is ready to get transferred to RAM.                          */
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
  __I  uint32_t  RESERVED3[74];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[61];
  __I  uint32_t  STATUS;                            /*!< Status                                                                */
  __I  uint32_t  RESERVED5[63];
  __IO uint32_t  ENABLE;                            /*!< Enable or disable ADC                                                 */
  __I  uint32_t  RESERVED6[3];
  SAADC_CH_Type CH[8];                              /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[24];
  __IO uint32_t  RESOLUTION;                        /*!< Resolution configuration                                              */
  __IO uint32_t  OVERSAMPLE;                        /*!< Oversampling configuration. OVERSAMPLE should not be combined
                                                         with SCAN. The RESOLUTION is applied before averaging, thus
                                                          for high OVERSAMPLE a higher RESOLUTION should be used.              */
  __IO uint32_t  SAMPLERATE;                        /*!< Controls normal or continuous sample rate                             */
  __I  uint32_t  RESERVED8[12];
  SAADC_RESULT_Type RESULT;                         /*!< RESULT EasyDMA channel                                                */
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
  __I  uint32_t  RESERVED5[77];
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
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief Watchdog Timer (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  __O  uint32_t  TASKS_START;                       /*!< Start the watchdog                                                    */
  __I  uint32_t  RESERVED0[31];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __I  uint32_t  RESERVED1[31];
  __IO uint32_t  EVENTS_TIMEOUT;                    /*!< Watchdog timeout                                                      */
  __I  uint32_t  RESERVED2[31];
  __IO uint32_t  PUBLISH_TIMEOUT;                   /*!< Publish configuration for EVENTS_TIMEOUT                              */
  __I  uint32_t  RESERVED3[96];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[61];
  __I  uint32_t  RUNSTATUS;                         /*!< Run status                                                            */
  __I  uint32_t  REQSTATUS;                         /*!< Request status                                                        */
  __I  uint32_t  RESERVED5[63];
  __IO uint32_t  CRV;                               /*!< Counter reload value                                                  */
  __IO uint32_t  RREN;                              /*!< Enable register for reload request registers                          */
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __I  uint32_t  RESERVED6[60];
  __O  uint32_t  RR[8];                             /*!< Description collection[0]: Reload request 0                           */
} NRF_WDT_Type;


/* ================================================================================ */
/* ================                       EGU                      ================ */
/* ================================================================================ */


/**
  * @brief Event Generator Unit 0 (EGU)
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
  * @brief Pulse Width Modulation Unit 0 (PWM)
  */

typedef struct {                                    /*!< PWM Structure                                                         */
  __I  uint32_t  RESERVED0;
  __O  uint32_t  TASKS_STOP;                        /*!< Stops PWM pulse generation on all channels at the end of current
                                                         PWM period, and stops sequence playback                               */
  __O  uint32_t  TASKS_SEQSTART[2];                 /*!< Description collection[0]: Loads the first PWM value on all
                                                         enabled channels from sequence 0, and starts playing that sequence
                                                          at the rate defined in SEQ[0]REFRESH and/or DECODER.MODE. Causes
                                                          PWM generation to start it was not running.                          */
  __O  uint32_t  TASKS_NEXTSTEP;                    /*!< Steps by one value in the current sequence on all enabled channels
                                                         if DECODER.MODE=NextStep. Does not cause PWM generation to start
                                                          it was not running.                                                  */
  __I  uint32_t  RESERVED1[28];
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __IO uint32_t  SUBSCRIBE_SEQSTART[2];             /*!< Description collection[0]: Subscribe configuration for TASKS_SEQSTART[0] */
  __IO uint32_t  SUBSCRIBE_NEXTSTEP;                /*!< Subscribe configuration for TASKS_NEXTSTEP                            */
  __I  uint32_t  RESERVED2[28];
  __IO uint32_t  EVENTS_STOPPED;                    /*!< Response to STOP task, emitted when PWM pulses are no longer
                                                         generated                                                             */
  __IO uint32_t  EVENTS_SEQSTARTED[2];              /*!< Description collection[0]: First PWM period started on sequence
                                                         0                                                                     */
  __IO uint32_t  EVENTS_SEQEND[2];                  /*!< Description collection[0]: Emitted at end of every sequence
                                                         0, when last value from RAM has been applied to wave counter          */
  __IO uint32_t  EVENTS_PWMPERIODEND;               /*!< Emitted at the end of each PWM period                                 */
  __IO uint32_t  EVENTS_LOOPSDONE;                  /*!< Concatenated sequences have been played the amount of times
                                                         defined in LOOP.CNT                                                   */
  __I  uint32_t  RESERVED3[25];
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __IO uint32_t  PUBLISH_SEQSTARTED[2];             /*!< Description collection[0]: Publish configuration for EVENTS_SEQSTARTED[0] */
  __IO uint32_t  PUBLISH_SEQEND[2];                 /*!< Description collection[0]: Publish configuration for EVENTS_SEQEND[0] */
  __IO uint32_t  PUBLISH_PWMPERIODEND;              /*!< Publish configuration for EVENTS_PWMPERIODEND                         */
  __IO uint32_t  PUBLISH_LOOPSDONE;                 /*!< Publish configuration for EVENTS_LOOPSDONE                            */
  __I  uint32_t  RESERVED4[24];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED5[63];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED6[125];
  __IO uint32_t  ENABLE;                            /*!< PWM module enable register                                            */
  __IO uint32_t  MODE;                              /*!< Selects operating mode of the wave counter                            */
  __IO uint32_t  COUNTERTOP;                        /*!< Value up to which the pulse generator counter counts                  */
  __IO uint32_t  PRESCALER;                         /*!< Configuration for PWM_CLK                                             */
  __IO uint32_t  DECODER;                           /*!< Configuration of the decoder                                          */
  __IO uint32_t  LOOP;                              /*!< Amount of playback of a loop                                          */
  __I  uint32_t  RESERVED7[2];
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
  __I  uint32_t  RESERVED2[29];
  __IO uint32_t  PUBLISH_STARTED;                   /*!< Publish configuration for EVENTS_STARTED                              */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __IO uint32_t  PUBLISH_END;                       /*!< Publish configuration for EVENTS_END                                  */
  __I  uint32_t  RESERVED3[93];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED4[125];
  __IO uint32_t  ENABLE;                            /*!< PDM module enable register                                            */
  __IO uint32_t  PDMCLKCTRL;                        /*!< PDM clock generator control                                           */
  __IO uint32_t  MODE;                              /*!< Defines the routing of the connected PDM microphones' signals         */
  __I  uint32_t  RESERVED5[3];
  __IO uint32_t  GAINL;                             /*!< Left output gain adjustment                                           */
  __IO uint32_t  GAINR;                             /*!< Right output gain adjustment                                          */
  __IO uint32_t  RATIO;                             /*!< Selects the ratio between PDM_CLK and output sample rate. Change
                                                         PDMCLKCTRL accordingly.                                               */
  __I  uint32_t  RESERVED6[7];
  PDM_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
  __I  uint32_t  RESERVED7[6];
  PDM_SAMPLE_Type SAMPLE;                           /*!< Unspecified                                                           */
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
  __I  uint32_t  RESERVED0[30];
  __IO uint32_t  SUBSCRIBE_START;                   /*!< Subscribe configuration for TASKS_START                               */
  __IO uint32_t  SUBSCRIBE_STOP;                    /*!< Subscribe configuration for TASKS_STOP                                */
  __I  uint32_t  RESERVED1[31];
  __IO uint32_t  EVENTS_RXPTRUPD;                   /*!< The RXD.PTR register has been copied to internal double-buffers.
                                                         When the I2S module is started and RX is enabled, this event
                                                          will be generated for every RXTXD.MAXCNT words that are received
                                                          on the SDIN pin.                                                     */
  __IO uint32_t  EVENTS_STOPPED;                    /*!< I2S transfer stopped.                                                 */
  __I  uint32_t  RESERVED2[2];
  __IO uint32_t  EVENTS_TXPTRUPD;                   /*!< The TDX.PTR register has been copied to internal double-buffers.
                                                         When the I2S module is started and TX is enabled, this event
                                                          will be generated for every RXTXD.MAXCNT words that are sent
                                                          on the SDOUT pin.                                                    */
  __I  uint32_t  RESERVED3[27];
  __IO uint32_t  PUBLISH_RXPTRUPD;                  /*!< Publish configuration for EVENTS_RXPTRUPD                             */
  __IO uint32_t  PUBLISH_STOPPED;                   /*!< Publish configuration for EVENTS_STOPPED                              */
  __I  uint32_t  RESERVED4[2];
  __IO uint32_t  PUBLISH_TXPTRUPD;                  /*!< Publish configuration for EVENTS_TXPTRUPD                             */
  __I  uint32_t  RESERVED5[90];
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
  __I  uint32_t  RESERVED10[3];
  I2S_PSEL_Type PSEL;                               /*!< Unspecified                                                           */
} NRF_I2S_Type;


/* ================================================================================ */
/* ================                       FPU                      ================ */
/* ================================================================================ */


/**
  * @brief FPU (FPU)
  */

typedef struct {                                    /*!< FPU Structure                                                         */
  __I  uint32_t  UNUSED;                            /*!< Unused.                                                               */
} NRF_FPU_Type;


/* ================================================================================ */
/* ================                      NVMC                      ================ */
/* ================================================================================ */


/**
  * @brief Non Volatile Memory Controller (NVMC)
  */

typedef struct {                                    /*!< NVMC Structure                                                        */
  __I  uint32_t  RESERVED0[256];
  __I  uint32_t  READY;                             /*!< Ready flag                                                            */
  __I  uint32_t  RESERVED1;
  __I  uint32_t  READYNEXT;                         /*!< Ready flag                                                            */
  __I  uint32_t  RESERVED2[62];
  __IO uint32_t  CONFIG;                            /*!< Configuration register                                                */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  ERASEALL;                          /*!< Register for erasing all non-volatile user memory                     */
  __I  uint32_t  RESERVED4[3];
  __IO uint32_t  ERASEPAGEPARTIALCFG;               /*!< Register for partial erase configuration                              */
  __I  uint32_t  RESERVED5[8];
  __IO uint32_t  ICACHECNF;                         /*!< I-code cache configuration register.                                  */
  __I  uint32_t  RESERVED6;
  __IO uint32_t  IHIT;                              /*!< I-code cache hit counter.                                             */
  __IO uint32_t  IMISS;                             /*!< I-code cache miss counter.                                            */
  __I  uint32_t  RESERVED7[13];
  __IO uint32_t  CONFIGNS;                          /*!< Non SecureConfiguration register                                      */
  __O  uint32_t  WRITEUICRNS;                       /*!< Non Secure APPROTECT enable register                                  */
  __I  uint32_t  RESERVED8[93];
  __IO uint32_t  FORCEONNVM;                        /*!< Force on all NVM supplies. See also the internal section in
                                                         the NVMC chapter.                                                     */
  __I  uint32_t  RESERVED9[9];
  __IO uint32_t  FORCEOFFNVM;                       /*!< Force off NVM supply. See also the internal section in the NVMC
                                                         chapter.                                                              */
} NRF_NVMC_Type;


/* ================================================================================ */
/* ================                       KMU                      ================ */
/* ================================================================================ */


/**
  * @brief Key Management Unit (KMU)
  */

typedef struct {                                    /*!< KMU Structure                                                         */
  __O  uint32_t  TASKS_PUSH_KEYSLOT;                /*!< Push a key slot over secure APB.                                      */
  __I  uint32_t  RESERVED0[31];
  __IO uint32_t  SUBSCRIBE_PUSH_KEYSLOT;            /*!< Subscribe configuration for TASKS_PUSH_KEYSLOT                        */
  __I  uint32_t  RESERVED1[31];
  __IO uint32_t  EVENTS_KEYSLOT_PUSHED;             /*!< Key successfully pushed over secure APB.                              */
  __IO uint32_t  EVENTS_KEYSLOT_REVOKED;            /*!< Key has been revoked and cannot be tasked for selection.              */
  __IO uint32_t  EVENTS_KEYSLOT_ERROR;              /*!< No key slot selected or no destination address defined or error
                                                         during push mechanism.                                                */
  __I  uint32_t  RESERVED2[29];
  __IO uint32_t  PUBLISH_KEYSLOT_PUSHED;            /*!< Publish configuration for EVENTS_KEYSLOT_PUSHED                       */
  __IO uint32_t  PUBLISH_KEYSLOT_REVOKED;           /*!< Publish configuration for EVENTS_KEYSLOT_REVOKED                      */
  __IO uint32_t  PUBLISH_KEYSLOT_ERROR;             /*!< Publish configuration for EVENTS_KEYSLOT_ERROR                        */
  __I  uint32_t  RESERVED3[93];
  __IO uint32_t  INTEN;                             /*!< Enable or disable interrupt                                           */
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  INTPEND;                           /*!< Pending status of interrupt                                           */
  __I  uint32_t  RESERVED4[63];
  __I  uint32_t  STATUS;                            /*!< Status bits for KMU operation.                                        */
  __I  uint32_t  RESERVED5[60];
  __IO uint32_t  SELECTKEYSLOT;                     /*!< Select key slot ID to be read over AHB or pushed over secure
                                                         APB when TASKS_PUSH_KEYSLOT is started.                               */
} NRF_KMU_Type;


/* ================================================================================ */
/* ================                       VMC                      ================ */
/* ================================================================================ */


/**
  * @brief Volatile Memory controller (VMC)
  */

typedef struct {                                    /*!< VMC Structure                                                         */
  __I  uint32_t  RESERVED0[384];
  VMC_RAM_Type RAM[8];                              /*!< Unspecified                                                           */
} NRF_VMC_Type;


/* ================================================================================ */
/* ================                   CRYPTOCELL                   ================ */
/* ================================================================================ */


/**
  * @brief ARM TrustZone CryptoCell register interface (CRYPTOCELL)
  */

typedef struct {                                    /*!< CRYPTOCELL Structure                                                  */
  __I  uint32_t  RESERVED0[320];
  __IO uint32_t  ENABLE;                            /*!< Control power and clock for CRYPTOCELL subsystem                      */
} NRF_CRYPTOCELL_Type;


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
  __IO uint32_t  DETECTMODE;                        /*!< Select between default DETECT signal behaviour and LDETECT mode
                                                         (For non-secure pin only)                                             */
  __IO uint32_t  DETECTMODE_SEC;                    /*!< Select between default DETECT signal behaviour and LDETECT mode
                                                         (For secure pin only)                                                 */
  __I  uint32_t  RESERVED1[117];
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
#define NRF_GPIOTE1_BASE                0x40031000UL
#define NRF_AMLI_BASE                   0x50000000UL
#define NRF_DCNF_BASE                   0x50000000UL
#define NRF_SPU_BASE                    0x50003000UL
#define NRF_REGULATORS_BASE             0x50004000UL
#define NRF_CLOCK_BASE                  0x50005000UL
#define NRF_POWER_BASE                  0x50005000UL
#define NRF_CTRL_AP_PERI_BASE           0x50006000UL
#define NRF_PAMLI_BASE                  0x50007000UL
#define NRF_UARTE0_BASE                 0x50008000UL
#define NRF_SPIM0_BASE                  0x50008000UL
#define NRF_SPIS0_BASE                  0x50008000UL
#define NRF_TWIM0_BASE                  0x50008000UL
#define NRF_TWIS0_BASE                  0x50008000UL
#define NRF_UARTE1_BASE                 0x50009000UL
#define NRF_SPIM1_BASE                  0x50009000UL
#define NRF_SPIS1_BASE                  0x50009000UL
#define NRF_TWIM1_BASE                  0x50009000UL
#define NRF_TWIS1_BASE                  0x50009000UL
#define NRF_UARTE2_BASE                 0x5000A000UL
#define NRF_SPIM2_BASE                  0x5000A000UL
#define NRF_SPIS2_BASE                  0x5000A000UL
#define NRF_TWIM2_BASE                  0x5000A000UL
#define NRF_TWIS2_BASE                  0x5000A000UL
#define NRF_UARTE3_BASE                 0x5000B000UL
#define NRF_SPIM3_BASE                  0x5000B000UL
#define NRF_SPIS3_BASE                  0x5000B000UL
#define NRF_TWIM3_BASE                  0x5000B000UL
#define NRF_TWIS3_BASE                  0x5000B000UL
#define NRF_GPIOTE0_BASE                0x5000D000UL
#define NRF_SAADC_BASE                  0x5000E000UL
#define NRF_TIMER0_BASE                 0x5000F000UL
#define NRF_TIMER1_BASE                 0x50010000UL
#define NRF_TIMER2_BASE                 0x50011000UL
#define NRF_RTC0_BASE                   0x50014000UL
#define NRF_RTC1_BASE                   0x50015000UL
#define NRF_DPPIC_BASE                  0x50017000UL
#define NRF_WDT_BASE                    0x50018000UL
#define NRF_EGU0_BASE                   0x5001B000UL
#define NRF_EGU1_BASE                   0x5001C000UL
#define NRF_EGU2_BASE                   0x5001D000UL
#define NRF_EGU3_BASE                   0x5001E000UL
#define NRF_EGU4_BASE                   0x5001F000UL
#define NRF_EGU5_BASE                   0x50020000UL
#define NRF_PWM0_BASE                   0x50021000UL
#define NRF_PWM1_BASE                   0x50022000UL
#define NRF_PWM2_BASE                   0x50023000UL
#define NRF_PWM3_BASE                   0x50024000UL
#define NRF_PDM_BASE                    0x50026000UL
#define NRF_I2S_BASE                    0x50028000UL
#define NRF_FPU_BASE                    0x5002C000UL
#define NRF_NVMC_BASE                   0x50039000UL
#define NRF_KMU_BASE                    0x50039000UL
#define NRF_VMC_BASE                    0x5003A000UL
#define NRF_CRYPTOCELL_BASE             0x50840000UL
#define NRF_GPIO_BASE                   0x50842000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define NRF_TAD                         ((NRF_TAD_Type            *) NRF_TAD_BASE)
#define NRF_FICR                        ((NRF_FICR_Type           *) NRF_FICR_BASE)
#define NRF_UICR                        ((NRF_UICR_Type           *) NRF_UICR_BASE)
#define NRF_GPIOTE1                     ((NRF_GPIOTE_Type         *) NRF_GPIOTE1_BASE)
#define NRF_AMLI                        ((NRF_AMLI_Type           *) NRF_AMLI_BASE)
#define NRF_DCNF                        ((NRF_DCNF_Type           *) NRF_DCNF_BASE)
#define NRF_SPU                         ((NRF_SPU_Type            *) NRF_SPU_BASE)
#define NRF_REGULATORS                  ((NRF_REGULATORS_Type     *) NRF_REGULATORS_BASE)
#define NRF_CLOCK                       ((NRF_CLOCK_Type          *) NRF_CLOCK_BASE)
#define NRF_POWER                       ((NRF_POWER_Type          *) NRF_POWER_BASE)
#define NRF_CTRL_AP_PERI                ((NRF_CTRLAPPERI_Type     *) NRF_CTRL_AP_PERI_BASE)
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
#define NRF_UARTE2                      ((NRF_UARTE_Type          *) NRF_UARTE2_BASE)
#define NRF_SPIM2                       ((NRF_SPIM_Type           *) NRF_SPIM2_BASE)
#define NRF_SPIS2                       ((NRF_SPIS_Type           *) NRF_SPIS2_BASE)
#define NRF_TWIM2                       ((NRF_TWIM_Type           *) NRF_TWIM2_BASE)
#define NRF_TWIS2                       ((NRF_TWIS_Type           *) NRF_TWIS2_BASE)
#define NRF_UARTE3                      ((NRF_UARTE_Type          *) NRF_UARTE3_BASE)
#define NRF_SPIM3                       ((NRF_SPIM_Type           *) NRF_SPIM3_BASE)
#define NRF_SPIS3                       ((NRF_SPIS_Type           *) NRF_SPIS3_BASE)
#define NRF_TWIM3                       ((NRF_TWIM_Type           *) NRF_TWIM3_BASE)
#define NRF_TWIS3                       ((NRF_TWIS_Type           *) NRF_TWIS3_BASE)
#define NRF_GPIOTE0                     ((NRF_GPIOTE_Type         *) NRF_GPIOTE0_BASE)
#define NRF_SAADC                       ((NRF_SAADC_Type          *) NRF_SAADC_BASE)
#define NRF_TIMER0                      ((NRF_TIMER_Type          *) NRF_TIMER0_BASE)
#define NRF_TIMER1                      ((NRF_TIMER_Type          *) NRF_TIMER1_BASE)
#define NRF_TIMER2                      ((NRF_TIMER_Type          *) NRF_TIMER2_BASE)
#define NRF_RTC0                        ((NRF_RTC_Type            *) NRF_RTC0_BASE)
#define NRF_RTC1                        ((NRF_RTC_Type            *) NRF_RTC1_BASE)
#define NRF_DPPIC                       ((NRF_DPPIC_Type          *) NRF_DPPIC_BASE)
#define NRF_WDT                         ((NRF_WDT_Type            *) NRF_WDT_BASE)
#define NRF_EGU0                        ((NRF_EGU_Type            *) NRF_EGU0_BASE)
#define NRF_EGU1                        ((NRF_EGU_Type            *) NRF_EGU1_BASE)
#define NRF_EGU2                        ((NRF_EGU_Type            *) NRF_EGU2_BASE)
#define NRF_EGU3                        ((NRF_EGU_Type            *) NRF_EGU3_BASE)
#define NRF_EGU4                        ((NRF_EGU_Type            *) NRF_EGU4_BASE)
#define NRF_EGU5                        ((NRF_EGU_Type            *) NRF_EGU5_BASE)
#define NRF_PWM0                        ((NRF_PWM_Type            *) NRF_PWM0_BASE)
#define NRF_PWM1                        ((NRF_PWM_Type            *) NRF_PWM1_BASE)
#define NRF_PWM2                        ((NRF_PWM_Type            *) NRF_PWM2_BASE)
#define NRF_PWM3                        ((NRF_PWM_Type            *) NRF_PWM3_BASE)
#define NRF_PDM                         ((NRF_PDM_Type            *) NRF_PDM_BASE)
#define NRF_I2S                         ((NRF_I2S_Type            *) NRF_I2S_BASE)
#define NRF_FPU                         ((NRF_FPU_Type            *) NRF_FPU_BASE)
#define NRF_NVMC                        ((NRF_NVMC_Type           *) NRF_NVMC_BASE)
#define NRF_KMU                         ((NRF_KMU_Type            *) NRF_KMU_BASE)
#define NRF_VMC                         ((NRF_VMC_Type            *) NRF_VMC_BASE)
#define NRF_CRYPTOCELL                  ((NRF_CRYPTOCELL_Type     *) NRF_CRYPTOCELL_BASE)
#define NRF_GPIO                        ((NRF_GPIO_Type           *) NRF_GPIO_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group nrf9120 */
/** @} */ /* End of group Nordic Semiconductor */

#ifdef __cplusplus
}
#endif


#endif  /* nrf9120_H */

