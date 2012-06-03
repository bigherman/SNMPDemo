/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SNMP_DEMO.C 
 *      Purpose: SNMP Agent demo example
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <RTL.h>
#include <Net_Config.h>
#include <stm32f10x_cl.h>
#include "GLCD.h"


BOOL tick;
U32  dhcp_tout;
BOOL LCDupdate;
U8   lcd_text[2][16+1] = {" ",                /* Buffer for LCD text         */
                          "Waiting for DHCP"};

extern LOCALM localm[];                       /* Local Machine Settings      */
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   5                        /* DHCP timeout 0.5 seconds      */

/*--------------------------- init ------------------------------------------*/

static void init () {
  /* Add System initialisation code here */

  /* Set the clocks. */
  SystemInit();
  RCC->APB2ENR |= 0x00000261;

  /* Configure the GPIO for Push Buttons */
  GPIOA->CRL &= 0xFFFFFFF0;
  GPIOA->CRL |= 0x00000004;
  GPIOB->CRL &= 0x0FFFFFFF;
  GPIOB->CRL |= 0x40000000;
  GPIOC->CRH &= 0xFF0FFFFF;
  GPIOC->CRH |= 0x00400000;

  /* Configure GPIO for JOYSTICK */
  GPIOD->CRH &= 0x00000FFF;
  GPIOD->CRH |= 0x44444000;

  /* Configure the GPIO for LEDs. */
  GPIOE->CRH  = 0x33333333;

  /* Configure UART2 for 115200 baud. */
  AFIO->MAPR |= 0x00000008;
  GPIOD->CRL &= 0xF00FFFFF;
  GPIOD->CRL |= 0x04B00000;

  RCC->APB1ENR |= 0x00020000;
  USART2->BRR = 0x0135;
  USART2->CR3 = 0x0000;
  USART2->CR2 = 0x0000;
  USART2->CR1 = 0x200C;

  /* Setup and enable the SysTick timer for 100ms. */
  SysTick->LOAD = (SystemFrequency / 10) - 1;
  SysTick->CTRL = 0x05;

  /******************************************************************************/
  /* Initialises the Analog/Digital converter */
  /* PC4 (ADC Channel14) is used as analog input */
  /******************************************************************************/
  RCC->APB2ENR |= (1<<4); /* enable clock for GPIOC */
  GPIOC->CRL &= 0xFFF0FFFF; /* Configure ADC.14 input. */
  RCC->APB2ENR |= (1<<9); /* enable clock for ADC1 */
  ADC1->SQR1 = 0x00000000; /* one conversion */
  ADC1->SQR3 = (14<< 0); /* set order to chn14 */
  ADC1->SMPR1 = ( 5<<12); /* sample time (55,5 cycles) */
  ADC1->CR1 = 0x00000100; /* independant mode, SCAN mode */
  ADC1->CR2 = 0x000E0003; /* data align right */
  /* continuous conversion */
  /* EXTSEL = SWSTART */
  /* enable ADC */
  ADC1->CR2 |= 0x00000008; /* Reset calibration */
  while (ADC1->CR2 & 0x00000008);
  ADC1->CR2 |= 0x00000004; /* Start calibration */
  while (ADC1->CR2 & 0x00000004);
  ADC1->CR2 |= 0x00500000; /* start SW conversion */ 
}


/*--------------------------- fputc -----------------------------------------*/

int fputc (int ch, FILE *f)  {
  /* Debug output to serial port. */

  if (ch == '\n')  {
    while (!(USART2->SR & 0x0080));
    USART2->DR = 0x0D;
  }
  while (!(USART2->SR & 0x0080));
  USART2->DR = (ch & 0xFF);
  return (ch);
}


/*--------------------------- LED_out ---------------------------------------*/

void LED_out (U32 val) {
  val <<= 8;
  GPIOE->BSRR = val;
  GPIOE->BRR  = val ^ 0xFF00;
}


/*--------------------------- get_button ------------------------------------*/

U8 get_button (void) {
  /* Read ARM Digital Input */
  U32 val = 0;

  if ((GPIOB->IDR & (1 << 7)) == 0) {
    /* User button */
    val |= 0x01;
  }
  if ((GPIOC->IDR & (1 << 13)) == 0) {
    /* Tamper button */
    val |= 0x02;
  }
  if ((GPIOA->IDR & (1 << 0)) != 0) {
    /* Wakeup button */
    val |= 0x04;
  }
  if ((GPIOD->IDR & (1 << 15)) == 0) {
    /* Joystick left */
    val |= 0x08;
  }
  if ((GPIOD->IDR & (1 << 13)) == 0) {
    /* Joystick right */
    val |= 0x10;
  }
  if ((GPIOD->IDR & (1 << 12)) == 0) {
    /* Joystick up */
    val |= 0x20;
  }
  if ((GPIOD->IDR & (1 << 14)) == 0) {
    /* Joystick down */
    val |= 0x40;
  }
  if ((GPIOD->IDR & (1 << 11)) == 0) {
    /* Joystick select */
    val |= 0x80;
  }

  return (val);
}


/*--------------------------- upd_display -----------------------------------*/

static void upd_display () {
  /* Update GLCD Module display text. */

  GLCD_ClearLn (5, 1);
  GLCD_DisplayString (5, 0, 1, lcd_text[0]);
  GLCD_ClearLn (6, 1);
  GLCD_DisplayString (6, 0, 1, lcd_text[1]);

  LCDupdate =__FALSE;
}

/*--------------------------- get_adc ---------------------------------------*/

U16 get_adc(void) {
  /* Get ADC Value. */

  //U32 adc_value = 1024;
  U16	adcValue = (ADC1->DR & 0x0FFF); /* AD value (12 bit) */

  return (adcValue);
}


/*--------------------------- init_display ----------------------------------*/

static void init_display () {
  /* LCD Module init */

  GLCD_Init ();
  GLCD_Clear (White);
  GLCD_SetTextColor (Blue);
  GLCD_DisplayString (2, 4, 1, "   RL-ARM");
  GLCD_DisplayString (3, 4, 1, "SNMP example");
}

/*---------------------------- send_trap ------------------------------------*/ 

void send_trap_message (void) {
  /* Send a trap message using MIB entries as per SNMP_MIB.c.*/
  U16 obj[6];

  /* obj[0] is the number of bound variables in a message,
  obj[n] where n!=0 is the index for the total MIB table, as laid out in SNMP_MIB.c. */
  obj[0] = 5;
  obj[1] = 7;
  obj[2] = 8;
  obj[3] = 9;
  obj[4] = 10;
  obj[5] = 11;

  /* Use Experimental MIB entries. snmp_trap binds the variables requested in obj[n] to the trap message.
  Make a Wireshark capture if you don't believe me! ;-)  */
  snmp_trap (NULL, 6, 3, obj);
}


/*--------------------------- timer_poll ------------------------------------*/

static void timer_poll () {
  /* System tick timer running in poll mode */

  if (SysTick->CTRL & 0x10000) {
    /* Timer tick every 100 ms */
    timer_tick ();
    tick  = __TRUE;
	send_trap_message();
  }
}


/*--------------------------- dhcp_check ------------------------------------*/

static void dhcp_check () {
  /* Monitor DHCP IP address assignment. */

  if (tick == __FALSE || dhcp_tout == 0) {
    return;
  }
  tick = __FALSE;
  if (mem_test (&MY_IP, 0, IP_ADRLEN) == __FALSE && !(dhcp_tout & 0x80000000)) {
    /* Success, DHCP has already got the IP address. */
    dhcp_tout = 0;
    sprintf((char *)lcd_text[0]," IP address:");
    sprintf((char *)lcd_text[1]," %d.%d.%d.%d", MY_IP[0], MY_IP[1],
                                                MY_IP[2], MY_IP[3]);
    LCDupdate = __TRUE;
    return;
  }
  if (--dhcp_tout == 0) {
    /* A timeout, disable DHCP and use static IP address. */
    dhcp_disable ();
    sprintf((char *)lcd_text[1]," DHCP failed    " );
    LCDupdate = __TRUE;
    dhcp_tout = 30 | 0x80000000;
    return;
  }
  if (dhcp_tout == 0x80000000) {
    dhcp_tout = 0;
    sprintf((char *)lcd_text[0]," IP address:");
    sprintf((char *)lcd_text[1]," %d.%d.%d.%d", MY_IP[0], MY_IP[1],
                                                MY_IP[2], MY_IP[3]);
    LCDupdate = __TRUE;
  }
}


/*---------------------------------------------------------------------------*/

int main (void) {
  /* Main Thread of the TcpNet */

  init ();
  init_display ();
  init_TcpNet ();

  dhcp_tout = DHCP_TOUT;
  LED_out (0x55);

  while (1) {
    timer_poll ();
    main_TcpNet ();
    dhcp_check ();
    if (LCDupdate == __TRUE) {
      upd_display ();
    }
  }
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
