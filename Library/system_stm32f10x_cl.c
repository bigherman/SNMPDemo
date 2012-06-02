/******************************************************************************
 * @file:    system_stm32f10x_cl.c
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Source File for the
 *           ST STM32F10x Connectivity Line Device Series
 * @version: V1.00
 * @date:    18. Jun. 2009
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#include <stm32f10x_cl.h>

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//=========================================================================== Clock Configuration
// <e0> Clock Configuration
//   <h> Clock Control Register Configuration (RCC_CR)
//     <e1.28> PLL3ON: PLL3 enable
//       <o3.12..15> PLL3MUL: PLL3 Multiplication Factor
//         <i> Default: PLL3SRC * 8
//                       <6=> PLL3SRC * 8
//                       <7=> PLL3SRC * 9
//                       <8=> PLL3SRC * 10
//                       <9=> PLL3SRC * 11
//                       <10=> PLL3SRC * 12
//                       <11=> PLL3SRC * 13
//                       <12=> PLL3SRC * 14
//                       <14=> PLL3SRC * 16
//                       <15=> PLL3SRC * 20
//     </e>
//     <e1.26> PLL2ON: PLL2 enable
//       <o3.4..7> PREDIV2: PREDIV2 division factor 
//         <i> Default: PREDIV2CLK not divided
//                       <0=> PREDIV2CLK not divided
//                       <1=> PREDIV2CLK / 2
//                       <2=> PREDIV2CLK / 3
//                       <3=> PREDIV2CLK / 4
//                       <4=> PREDIV2CLK / 5
//                       <5=> PREDIV2CLK / 6
//                       <6=> PREDIV2CLK / 7
//                       <7=> PREDIV2CLK / 8
//                       <8=> PREDIV2CLK / 9
//                       <9=> PREDIV2CLK / 10
//                       <10=> PREDIV2CLK / 11
//                       <11=> PREDIV2CLK / 12
//                       <12=> PREDIV2CLK / 13
//                       <13=> PREDIV2CLK / 14
//                       <14=> PREDIV2CLK / 15
//                       <15=> PREDIV2CLK / 16
//       <o3.8..11> PLL2MUL: PLL2 Multiplication Factor
//         <i> Default: PLL2SRC * 8
//                       <6=> PLL2SRC * 8
//                       <7=> PLL2SRC * 9
//                       <8=> PLL2SRC * 10
//                       <9=> PLL2SRC * 11
//                       <10=> PLL2SRC * 12
//                       <11=> PLL2SRC * 13
//                       <12=> PLL2SRC * 14
//                       <14=> PLL2SRC * 16
//                       <15=> PLL2SRC * 20
//     </e>
//     <e1.24> PLL1ON: PLL1 enable         
//       <i> Default: PLL1 Disabled
//       <o3.16> PREDIV1SRC: PREDIV1 entry clock source 
//         <i> Default: HSE selected as PREDIV1 clock entry
//                       <0=> HSE selected as PREDIV1 clock entry 
//                       <1=> PLL2 selected as PREDIV1 clock entry
//       <o3.0..3> PREDIV1: PREDIV1 division factor
//         <i> Default: PREDIV1CLK not divided
//         <i> Bit(0) is the same as bit(17) in the RCC_CFGR register, so modifying bit(17) in the RCC_CFGR register changes Bit(0) accordingly 
//                       <0=> PREDIV1CLK not divided
//                       <1=> PREDIV1CLK / 2
//                       <2=> PREDIV1CLK / 3
//                       <3=> PREDIV1CLK / 4
//                       <4=> PREDIV1CLK / 5
//                       <5=> PREDIV1CLK / 6
//                       <6=> PREDIV1CLK / 7
//                       <7=> PREDIV1CLK / 8
//                       <8=> PREDIV1CLK / 9
//                       <9=> PREDIV1CLK / 10
//                       <10=> PREDIV1CLK / 11
//                       <11=> PREDIV1CLK / 12
//                       <12=> PREDIV1CLK / 13
//                       <13=> PREDIV1CLK / 14
//                       <14=> PREDIV1CLK / 15
//                       <15=> PREDIV1CLK / 16
//       <o2.18..21> PLL1MUL: PLL1 Multiplication Factor
//         <i> Default: PLL1SRC * 4
//         <i> Caution: The PLL1 output frequency must not exceed 72 MHz.
//                       <2=> PLL1SRC * 4
//                       <3=> PLL1SRC * 5
//                       <4=> PLL1SRC * 6
//                       <5=> PLL1SRC * 7
//                       <6=> PLL1SRC * 8
//                       <7=> PLL1SRC * 9
//                       <13=> PLL1SRC * 6.5
//       <o2.17> PLL1XTPRE: LSB of division factor PREDIV1
//         <i> Default: 0
//                       <0=> 0
//                       <1=> 1
//       <o2.16> PLL1SRC: PLL1 entry clock source
//         <i> Default: PLL1SRC = HSI / 2
//                       <0=> PLL1SRC = HSI / 2
//                       <1=> PLL1SRC = PREDIV1 clock
//     </e>
//     <o1.19> CSSON: Clock Security System enable
//       <i> Default: Clock detector OFF
//     <o1.18> HSEBYP: External High Speed clock Bypass
//       <i> Default: HSE oscillator not bypassed
//     <o1.16> HSEON: External High Speed clock enable 
//       <i> Default: HSE oscillator OFF
//     <o1.3..7> HSITRIM: Internal High Speed clock trimming  <0-31>
//       <i> Default: 0
//     <o1.0> HSION: Internal High Speed clock enable
//       <i> Default: internal 8MHz RC oscillator OFF
//   </h>
//   <h> Clock Configuration Register Configuration (RCC_CFGR)
//     <o2.24..27> MCO: Microcontroller Clock Output   
//       <i> Default: MCO = noClock
//                     <0=> MCO = noClock
//                     <4=> MCO = SYSCLK
//                     <5=> MCO = HSI
//                     <6=> MCO = HSE
//                     <7=> MCO = PLL1CLK / 2
//                     <8=> MCO = PLL2CLK
//                     <9=> MCO = PLL3CLK /2
//                     <10=> XT1 external 3-25 MHz oscillator clock selected (for Ethernet)
//                     <11=> MCO = PLL3CLK (for Ethernet)
//     <o2.22> OTGFSPRE: USB OTG FS prescaler
//       <i> Default: USBCLK = PLL1CLK / 3
//                     <0=> USBCLK = PLL1CLK / 3
//                     <1=> USBCLK = PLL1CLK / 2
//     <o2.14..15> ADCPRE: ADC prescaler
//       <i> Default: ADCCLK=PCLK2 / 2
//                     <0=> ADCCLK = PCLK2 / 2
//                     <1=> ADCCLK = PCLK2 / 4
//                     <2=> ADCCLK = PCLK2 / 6
//                     <3=> ADCCLK = PCLK2 / 8
//     <o2.11..13> PPRE2: APB High speed prescaler (APB2)
//       <i> Default: PCLK2 = HCLK
//                     <0=> PCLK2 = HCLK
//                     <4=> PCLK2 = HCLK / 2 
//                     <5=> PCLK2 = HCLK / 4 
//                     <6=> PCLK2 = HCLK / 8 
//                     <7=> PCLK2 = HCLK / 16 
//     <o2.8..10> PPRE1: APB Low speed prescaler (APB1) 
//       <i> Default: PCLK1 = HCLK
//       <i> Caution: Software must configure these bits ensure that the frequency in this domain does not exceed 36 MHz.
//                     <0=> PCLK1 = HCLK
//                     <4=> PCLK1 = HCLK / 2 
//                     <5=> PCLK1 = HCLK / 4 
//                     <6=> PCLK1 = HCLK / 8 
//                     <7=> PCLK1 = HCLK / 16 
//     <o2.4..7> HPRE: AHB prescaler 
//       <i> Default: HCLK = SYSCLK
//                     <0=> HCLK = SYSCLK
//                     <8=> HCLK = SYSCLK / 2
//                     <9=> HCLK = SYSCLK / 4
//                     <10=> HCLK = SYSCLK / 8
//                     <11=> HCLK = SYSCLK / 16
//                     <12=> HCLK = SYSCLK / 64
//                     <13=> HCLK = SYSCLK / 128
//                     <14=> HCLK = SYSCLK / 256
//                     <15=> HCLK = SYSCLK / 512
//     <o2.0..1> SW: System Clock Switch
//       <i> Default: SYSCLK = HSE
//                     <0=> SYSCLK = HSI
//                     <1=> SYSCLK = HSE
//                     <2=> SYSCLK = PLL1CLK
//   </h>
//   <h> Clock Configuration Register2 Configuration (RCC_CFGR2)
//     <o3.18> I2S3SRC: I2S3 clock source
//       <i> Default: ADCCLK=PCLK2 / 2
//                     <0=> SYSCLK selected as I2S3 clock entry
//                     <1=> PLL3 VCO clock selected as I2S3 clock entry
//     <o3.17> I2S2SRC: I2S2 clock source
//       <i> Default: SYSCLK selected as I2S2 clock entry
//                     <0=> SYSCLK selected as I2S2 clock entry
//                     <1=> PLL3 VCO clock selected as I2S2 clock entry
//   </h>
//   <o4>HSE: External High Speed Clock [Hz] <3000000-25000000>
//   <i> clock value for the used External High Speed Clock (3MHz <= HSE <= 25MHz).
//   <i> Default: 25000000  (25MHz)
// </e> End of Clock Configuration
#define __CLOCK_SETUP              1
#define __RCC_CR_VAL               0x15010082
#define __RCC_CFGR_VAL             0x0B1D8402
#define __RCC_CFGR2_VAL            0x00018644
#define __HSE                      25000000

//=========================================================================== Embedded Flash Configuration
// <e0> Embedded Flash Configuration
//   <h> Flash Access Control Configuration (FLASH_ACR)
//     <o1.0..2> LATENCY: Latency
//       <i> Default: 2 wait states
//                     <0=> 0 wait states
//                     <1=> 1 wait states
//                     <2=> 2 wait states
//     <o1.3> HLFCYA: Flash Half Cycle Access Enable
//     <o1.4> PRFTBE: Prefetch Buffer Enable
//     <o1.5> PRFTBS: Prefetch Buffer Status Enable
//   </h>
// </e>
#define __EFI_SETUP               1
#define __EFI_ACR_Val             0x00000012


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
  RCC Defines
 *----------------------------------------------------------------------------*/
/* register RCC_CR -----------------------------------------------------------*/
#define RCC_CR_HSION       (0x00000001)              /* Internal High Speed clock enable     */
#define RCC_CR_HSIRDY      (0x00000002)              /* Internal High Speed clock ready flag */
#define RCC_CR_HSEON       (0x00010000)              /* External High Speed clock enable     */
#define RCC_CR_HSERDY      (0x00020000)              /* External High Speed clock ready flag */
#define RCC_CR_PLL1ON      (0x01000000)              /* PLL1 enable                          */
#define RCC_CR_PLL1RDY     (0x02000000)              /* PLL1 clock ready flag                */
#define RCC_CR_PLL2ON      (0x04000000)              /* PLL2 enable                          */
#define RCC_CR_PLL2RDY     (0x08000000)              /* PLL2 clock ready flag                */
#define RCC_CR_PLL3ON      (0x10000000)              /* PLL3 enable                          */
#define RCC_CR_PLL3RDY     (0x20000000)              /* PLL3 clock ready flag                */

/* register RCC_CFGR ---------------------------------------------------------*/
#define RCC_CFGR_SWS       (0x0000000C)              /* System Clock Switch Status           */

#define __HSI (8000000UL)   

/*----------------------------------------------------------------------------
  Clock Definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency;                            /*!< System Clock Frequency (Core Clock) */
uint32_t SystemFrequency_SysClk;                     /*!< System clock                        */
uint32_t SystemFrequency_AHBClk;                     /*!< AHB System bus speed                */
uint32_t SystemFrequency_APB1Clk;                    /*!< APB Peripheral bus 1 (low)  speed   */
uint32_t SystemFrequency_APB2Clk;                    /*!< APB Peripheral bus 2 (high) speed   */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the Embedded Flash Interface,  initialize the PLL and update th SystemFrequency variable
 */
void SystemInit (void) {
  uint32_t rccCfgr, rccCfgr2;
   int32_t pll1InClk, pll2OutClk;

#if __EFI_SETUP
  FLASH->ACR = __EFI_ACR_Val;                        /* set access control register */
#endif

#if __CLOCK_SETUP

  RCC->CFGR  = __RCC_CFGR_VAL;                       /* set clock configuration register */
  RCC->CFGR2 = __RCC_CFGR2_VAL;                      /* set clock configuration register 2 */
  RCC->CR = __RCC_CR_VAL & 0x000FFFFF;               /* do not start PLLs yet */

  if (__RCC_CR_VAL & RCC_CR_HSION) {                 /* if HSI enabled*/
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);          /* Wait for HSIRDY = 1 (HSI is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_HSEON) {                 /* if HSE enabled*/
    while ((RCC->CR & RCC_CR_HSERDY) == 0);          /* Wait for HSERDY = 1 (HSE is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_PLL3ON) {                /* if PLL3 enabled*/
    RCC->CR |= RCC_CR_PLL3ON;                              /* PLL3 On */
    while ((RCC->CR & RCC_CR_PLL3RDY) == 0);         /* Wait for PLL3RDY = 1 (PLL is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_PLL2ON) {                /* if PLL2 enabled*/
    RCC->CR |= RCC_CR_PLL2ON;                              /* PLL6 On */
    while ((RCC->CR & RCC_CR_PLL2RDY) == 0);         /* Wait for PLL2RDY = 1 (PLL is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_PLL1ON) {                /* if PLL1 enabled*/
    RCC->CR |= RCC_CR_PLL1ON;                              /* PLL3 On */
    while ((RCC->CR & RCC_CR_PLL1RDY) == 0);         /* Wait for PLL1RDY = 1 (PLL is ready)*/
  }

  /* Wait till SYSCLK is stabilized (depending on selected clock) */
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((__RCC_CFGR_VAL<<2) & RCC_CFGR_SWS));
#endif

  /* Determine clock frequency according to clock register values */
  rccCfgr  = RCC->CFGR;
  rccCfgr2 = RCC->CFGR2;

  switch ((rccCfgr) & 0x03) {                        /* SW System clock Switch */
    case 0:                                          /* HSI selected as system clock */
      SystemFrequency_SysClk  = (uint32_t)(__HSI);
      break;
    case 1:                                          /* HSE selected as system clock */
      SystemFrequency_SysClk  = (uint32_t)(__HSE);
      break;
    case 2:                                          /* PLL selected as system clock */
      if ((rccCfgr >> 16) & 0x01) {                  /* PLLSRC PLL1 entry clock source */
        if ((rccCfgr2 >> 16) & 0x01) {               /* PLL2Clk selected as PLL1 entry clock source */
          pll2OutClk = (int32_t)(__HSE / (((rccCfgr2 >> 4) & 0x0F)+1));
          if (((rccCfgr2 >> 8) & 0x0F) < 0x0B) {
            pll2OutClk = pll2OutClk * (((rccCfgr2 >> 8) & 0x0F) + 2);
          } else {
            pll2OutClk = pll2OutClk * (((rccCfgr2 >> 8) & 0x0F) + 4);
          }
          pll1InClk = pll2OutClk;
        } else {
          pll1InClk = (int32_t)(__HSE); 
        }

        if (rccCfgr2 & 0x0E) {                       /* PREDIV1 configured */
          pll1InClk = (int32_t)(pll1InClk / ((rccCfgr2 & 0x0F)+1));
        } else {
          if ((rccCfgr >> 17) & 0x01) {              /* PLLXTPRE HSE divider for PLL entry */
            pll1InClk = (uint32_t)(pll1InClk/2);     /* PLL1 clock divided by 2 */
          } else {
            pll1InClk = (uint32_t)(pll1InClk);       /* PLL1CLK clock not divided */
          }
        }
      } else {
        pll1InClk = (uint32_t)(__HSI/2);             /* HSI oscillator clock / 2 selected as PLL1 input clock */
      }

      if (((rccCfgr >> 18) & 0x0F) == 0x0D) {
        SystemFrequency_SysClk  = pll1InClk * 6.5;
      } else {
        SystemFrequency_SysClk  = pll1InClk * (((rccCfgr >> 18) & 0x0F) + 2);
      }

      break;
  }

  if (rccCfgr & (1<< 7)) {
    SystemFrequency = SystemFrequency_SysClk >> (((rccCfgr >> 4) & 0x07) + 1);
  } else {
    SystemFrequency = SystemFrequency_SysClk;
  }

  SystemFrequency_AHBClk = SystemFrequency;

  if (rccCfgr & (1<<10)) {
    SystemFrequency_APB1Clk = SystemFrequency >> (((rccCfgr >> 8) & 0x03) + 1);
  } else {
    SystemFrequency_APB1Clk = SystemFrequency;
  }

  if (rccCfgr & (1<<13)) {
    SystemFrequency_APB2Clk = SystemFrequency >> (((rccCfgr >>11) & 0x03) + 1);
  } else {
    SystemFrequency_APB2Clk = SystemFrequency;
  }
}
