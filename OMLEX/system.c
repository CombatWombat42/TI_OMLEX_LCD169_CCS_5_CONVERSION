/**********************************************************************************/
/*                                                                                */
/*    Copyright (C) 2005 OLIMEX  LTD.                                             */
/*                                                                                */
/*    Module Name    :  system module                                             */
/*    File   Name    :  system.c                                                  */
/*    Revision       :  01.00                                                     */
/*    Date           :  2006/01/26 initial version                                */
/*                                                                                */
/**********************************************************************************/
#include "system.h"
#include <msp430.h>


/****************************************************************************/
/*  Init system frequency                                                   */
/*  Function : InitFreq                                                     */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/*  Description :                                                           */
/*  Tune DCOCLK around 800KHz and it is use for MCLK                        */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void InitFreq(void)
{
  // External oscilator 32kHz

  // Watchdog
  WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

  // DCO Control Register
  // register:
  // MODx   4-0
  // DCOx   7-5
  // set:
  // MODx = 0, DCOx = 3
  // description:
  // modulation off, DCOx and RSELx ~800kHz
  DCOCTL = 0x60;


  // Basic Clock System Control Register 1
  // register:
  // RSELx  2-0
  // XT5V   3
  // DIVAx  5-4
  // XTS    6
  // XT20FF 7
  // set:
  // RSELx = 4, DIVAx = 4, XTS = 0, XT20FF = 1
  // description:
  // 1/4 for ACLK, set Lower mode, XT2 off
  BCSCTL1 = 0xA4;


  // Basic Clock System Control Register 2
  // register:
  // DCOR   0
  // DIVSx  1-2
  // SELS   3
  // DIVMx  4-5
  // SELMx  6-7
  // set:
  // DCOR = 0, DIVSx = 0, SELS = 0, DIVMx = 0, SELMx = 0
  // description:
  // internal resistor for DCO, no divider for MCLK and SCLK, MCLK and SCLK source -> DCOCLK
  BCSCTL2 = 0x00;

}


/****************************************************************************/
/*  Init system frequency                                                   */
/*  Function : InitFreq                                                     */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/*  Description :                                                           */
/*  LFXTCLK 32kHz is use for MCLK                                           */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void InitFreq32kHz(void)
{
  // External oscilator 32kHz

  // Watchdog
  WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

  // DCO Control Register
  // register:
  // MODx   4-0
  // DCOx   7-5
  // set:
  // MODx = 0, DCOx = 3
  // description:
  // modulation off, DCOx and RSELx ~800kHz
  DCOCTL = 0x60;


  // Basic Clock System Control Register 1
  // register:
  // RSELx  2-0
  // XT5V   3
  // DIVAx  5-4
  // XTS    6
  // XT20FF 7
  // set:
  // RSELx = 4, DIVAx = 0, XTS = 0, XT20FF = 1
  // description:
  // no divider for ACLK, set Lower mode, XT2 off
  BCSCTL1 = 0x84;


  // Basic Clock System Control Register 2
  // register:
  // DCOR   0
  // DIVSx  1-2
  // SELS   3
  // DIVMx  4-5
  // SELMx  6-7
  // set:
  // DCOR = 0, DIVSx = 0, SELS = 0, DIVMx = 0, SELMx = 0
  // description:
  // internal resistor for DCO, no divider for MCLK and SCLK, MCLK and SCLK source -> LFXTCLK (32kHz)
  BCSCTL2 = 0xC0;

}



/****************************************************************************/
/*  Init pheriphery                                                         */
/*  Function : InitPeriph                                                   */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void InitPeriph(void)
{
  // set first functionality to all ports
  P1SEL = 0x0;
  P2SEL = 0x0;
  P3SEL = 0x0;
/*
  // JTAG Pins as output and low
  P1OUT_bit.P1OUT_4 = 0;
  P1OUT_bit.P1OUT_5 = 0;
  P1OUT_bit.P1OUT_6 = 0;
  P1OUT_bit.P1OUT_7 = 0;
  P1DIR_bit.P1DIR_4 = 1;
  P1DIR_bit.P1DIR_5 = 1;
  P1DIR_bit.P1DIR_6 = 1;
  P1DIR_bit.P1DIR_7 = 1;

  // VCC pin -> P2.5
  // P2.5 as output
  P2OUT_bit.P2OUT_5 = 1;
  P2DIR_bit.P2DIR_5 = 1;


  // Pin2.3 as output and low
  P2OUT_bit.P2OUT_3 = 0;
  P2DIR_bit.P2DIR_3 = 1;


  // PROG 7.5 Pin3.5 as output and low
  P3OUT_bit.P3OUT_5 = 0;
  P3DIR_bit.P3DIR_5 = 1;
*/

}


