/*----------------------------------------------------------------------------
 *      RL-ARM - FlashFS
 *----------------------------------------------------------------------------
 *      Name:    SPI_STM32F107.c
 *      Purpose: Serial Peripheral Interface Driver for ST STM32F105/7
 *      Rev.:    V4.20
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <File_Config.h>
#include <stm32f10x_cl.h>


/*----------------------------------------------------------------------------
  SPI Driver instance definition
   spi0_drv: First SPI driver
   spi1_drv: Second SPI driver
 *---------------------------------------------------------------------------*/

#define __DRV_ID  spi0_drv
#define __FPCLK    72000000	

/* SPI Driver Interface functions */
static BOOL Init (void);
static BOOL UnInit (void);
static U8   Send (U8 outb);
static BOOL SendBuf (U8 *buf, U32 sz);
static BOOL RecBuf (U8 *buf, U32 sz);
static BOOL BusSpeed (U32 kbaud);
static BOOL SetSS (U32 ss);
static U32  CheckMedia (void);        /* Optional function for SD card check */

/* SPI Device Driver Control Block */
SPI_DRV __DRV_ID = {
  Init,
  UnInit,
  Send,
  SendBuf,
  RecBuf,
  BusSpeed,
  SetSS,
  CheckMedia                          /* Can be NULL if not existing         */
};

#ifdef STM3210C
 #define SPIx   SPI3
#else
 #define SPIx   SPI1
#endif

/* SPI_SR - bit definitions. */
#define RXNE    0x01
#define TXE     0x02
#define BSY     0x80
#define FPCLK   __FPCLK/1000


/*--------------------------- spi_init --------------------------------------*/

static BOOL Init (void) {
  /* Initialize and enable the SSP Interface module. */

#ifdef STM3210C
  /* Use SPI3 on STM3210C-EVAL board. */

  /* Enable clock for GPIOA,C,E, AFIO and SPI3. */
  RCC->APB2ENR |= 0x00000055;
  RCC->APB1ENR |= 0x00008000;

  /* Set SPI3 remap (use PC10..PC12). */
  AFIO->MAPR   |= 0x10000000;

  /* SPI3_NSS is GPIO, output set to high. */
  GPIOA->CRL = (GPIOA->CRL & 0xFFF0FFFF) | 0x00030000;
  GPIOA->BSRR = 0x00000010;

  /* SPI3_SCK, SPI3_MISO, SPI3_MOSI are SPI pins. */
  GPIOC->CRH = (GPIOC->CRH & 0xFFF000FF) | 0x000B8B00;
#else
  /* Use SPI1 on MCBSTM32C evaluation board. */

  /* Enable clock for GPIOA,E, AFIO and SPI1. */
  RCC->APB2ENR |= 0x00001045;

  /* No SPI1 remap (use PA5..PA7). */
  AFIO->MAPR   &= 0xFFFFFFFE;

  /* SPI1_NSS is GPIO, output set to high. */
  /* SPI1_SCK, SPI1_MISO, SPI1_MOSI are SPI pins. */
  GPIOA->CRL = (GPIOA->CRL & 0x0000FFFF) | 0xB8B30000;
  GPIOA->BSRR = 0x00000010;
#endif

  /* Card Sensor PE.0 input */
  /* 1 = NO Card, 0 = Card plugged. */
  GPIOE->CRL  = (GPIOE->CRL & 0xFFFFFFF0) | 0x00000008;
  GPIOE->ODR |= 0x01; 

  /* Enable SPI in Master Mode, CPOL=0, CPHA=0. */
  /* Clock speed = fPCLK1 / 256 = 280 kHz at 72 MHz PCLK1 clk. */
  SPIx->CR1  = 0x037C;
  SPIx->CR2  = 0x0000;

  return (__TRUE);
}


/*--------------------------- UnInit ----------------------------------------*/

static BOOL UnInit (void) {
  /* Return SSP interface to default state. */
  
#ifdef STM3210C
  GPIOA->CRL = (GPIOA->CRL & 0xFFF0FFFF) | 0x00040000;
  GPIOC->CRH = (GPIOC->CRH & 0xFFF000FF) | 0x00044400;
#else 
  GPIOA->CRL = (GPIOA->CRL & 0x0000FFFF) | 0x44440000;
#endif 
  
  GPIOE->ODR &= ~0x01; 
  GPIOE->CRL  = (GPIOE->CRL & 0xFFFFFFF0) | 0x00000004;
  
  SPIx->CR1  = 0x0000;
  SPIx->CR2  = 0x0000;    

  return (__TRUE);
}



/*--------------------------- Send ------------------------------------------*/

static U8 Send (U8 outb) {
  /* Write and Read a byte on SPI interface. */

  SPIx->DR = outb;	 
  /* Wait if RNE cleared, Rx FIFO is empty. */
  while (!(SPIx->SR & RXNE));
  return (SPIx->DR);
}


/*--------------------------- SendBuf ---------------------------------------*/

static BOOL SendBuf (U8 *buf, U32 sz) {
  /* Send buffer to SPI interface. */
  U32 i;										   

  for (i = 0; i < sz; i++) {    
    SPIx->DR = buf[i];
	/* Wait if TXE cleared, Tx FIFO is full. */
    while (!(SPIx->SR & TXE));
    
    SPIx->DR;
  }
  /* Wait until Tx finished, drain Rx FIFO. */
  while (SPIx->SR & (BSY | RXNE)) {
    SPIx->DR;
  }
  return (__TRUE);							 
}


/*--------------------------- RecBuf ----------------------------------------*/

static BOOL RecBuf (U8 *buf, U32 sz) {
  /* Receive SPI data to buffer. */
  U32 i;

  for (i = 0; i < sz; i++) {
    SPIx->DR = 0xFF;
    /* Wait if RNE cleared, Rx FIFO is empty. */
    while (!(SPIx->SR & RXNE));
    buf[i] = SPIx->DR;
  }
  return (__TRUE);
}


/*--------------------------- BusSpeed --------------------------------------*/

static BOOL BusSpeed (U32 kbaud) {
  /* Set an SPI clock to required baud rate. */
  U8  br;

  if      (kbaud >= FPCLK / 2)   br = 0;                       /* FPCLK/2    */
  else if (kbaud >= FPCLK / 4)   br = 1;                       /* FPCLK/4    */
  else if (kbaud >= FPCLK / 8)   br = 2;                       /* FPCLK/8    */
  else if (kbaud >= FPCLK / 16)  br = 3;                       /* FPCLK/16   */
  else if (kbaud >= FPCLK / 32)  br = 4;                       /* FPCLK/32   */
  else if (kbaud >= FPCLK / 64)  br = 5;                       /* FPCLK/64   */
  else if (kbaud >= FPCLK / 128) br = 6;                       /* FPCLK/128  */
  else                           br = 7;                       /* FPCLK/256  */

  SPIx->CR1 = (SPIx->CR1 & ~(7 << 3)) | (br << 3); 
  
  return (__TRUE);
}


/*--------------------------- SetSS -----------------------------------------*/

static BOOL SetSS (U32 ss) {
  /* Enable/Disable SPI Chip Select (drive it high or low). */
  GPIOA->BSRR = ss ? 0x00000010 : 0x00100000;
  return (__TRUE);
}


/*--------------------------- CheckMedia ------------------------------------*/

static U32 CheckMedia (void) {
  /* Read CardDetect and WriteProtect SD card socket pins. */
  U32 stat = 0;
 
  if (!(GPIOE->IDR & 1)) {
    /* Card is inserted (CD=0). */
    stat |= M_INSERTED;
  }

#if 0
  if ((GPIOA->IDR  & 0x20)) {
    /* Write Protect switch is active (WP=1). */
    stat |= M_PROTECTED;
  }
#endif

  return (stat);
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
