/**********************************************************************************/
/*                                                                                */
/*    Copyright (C) 2005 OLIMEX  LTD.                                             */
/*                                                                                */
/*    Module Name    :  main module                                               */
/*    File   Name    :  main.c                                                    */
/*    Revision       :  01.00                                                     */
/*    Date           :  2006/01/26 initial version                                */
/*                                                                                */
/**********************************************************************************/
#include "system.h"
#include "lcd_new.h"
#include "mmc.h"
#include <io430x16x.h>
#include <string.h>

#define DELAY_1   0

unsigned int X = 0;
unsigned int Y = 0;

unsigned char i = 0;

// mmc variable
extern char mmc_buffer[512];
char mmc_buffer_test_1[512];
char state_mmc = 1;

// menu variable
unsigned char pos = 0;
char press_up   = 1;
char press_down = 1;
char press_joys = 1;

// simple delay
void DelayN(unsigned long a) { while (--a!=0); }
void Stop(void);

// Restart
void Restart(void) {

  typedef void (* Fpnt_t)(void);
  Fpnt_t Fpnt;
  Fpnt = (Fpnt_t)*(unsigned int *)0xFFFE;
  Fpnt();
}

// move simple bitmap
void Move(void) {

  int i,j;

  for(i=0; i<16; i++) {
    for(j=0; j<18; j++) {
        LCDPixel(X+j, Y+i, BitmapSmall[i][j]);
    }
  }

  LCDUpdate();
}

// vizualize menu
void Menu(void) {

  LCDClear();
  LCDStr ( 0, " Test EXT port" );
  LCDStr ( 1, " Test MMC card" );
  LCDStr ( 2, " Test Graphics" );
  LCDStr ( 5, "press joystick" );
  LCDStr ( pos-1, ">" );
  LCDUpdate();

  //if joystick was pressed
  while((P2IN&BIT0) == 0) {
  }

  // loop for choose
  while(P2IN&BIT0) {

    // Up
    if((!(P1IN&BIT6))&&(press_up==1)) {


      pos--;
      if(pos<1) pos = 1;

      LCDClear();
      LCDStr ( 0, " Test EXT port" );
      LCDStr ( 1, " Test MMC card" );
      LCDStr ( 2, " Test Graphics" );
      LCDStr ( 5, "press joystick" );
      LCDStr ( pos-1, ">" );
      LCDUpdate();

      press_up = 0;
    }
    if((P1IN&BIT6)&&(press_up==0)) {

      press_up = 1;
    }

    // Down
    if((!(P1IN&BIT5))&&(press_down==1)) {

      pos++;
      if(pos>3) pos = 3;

      LCDClear();
      LCDStr ( 0, " Test EXT port" );
      LCDStr ( 1, " Test MMC card" );
      LCDStr ( 2, " Test Graphics" );
      LCDStr ( 5, "press joystick" );
      LCDStr ( pos-1, ">" );
      LCDUpdate();

      press_down = 0;
    }
    if((P1IN&BIT5)&&(press_down==0)) {

      press_down = 1;
    }

  }

}

/****************************************************************************/
/*  Main function                                                           */
/*  Function : main                                                         */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
int main( void )
{

  /**** INITIALIZATION ****/
  Y = 16;
  X = 33;
  pos = 1;


  //all as input
  P1DIR = 0x0;
  P2DIR = 0x0;
  P3DIR = 0x0;
  P4DIR = 0x0;
  P5DIR = 0x0;
  P6DIR = 0x0;


  // Frequency
  InitFreq();


  // LCD init
  LCDInit();
  LCDContrast(0x45);

  // Show menu
  Menu();

  // choose test ext
  if(pos == 1) {

    // THIS IS ONLY WITH EXTENSION PORT
    // CODE IS NOT PRESENT HERE

  }


  // choose mmc card
  if(pos == 2) {

    // pressed joystick
    press_joys = 0;
    DelayN(1000);
    // if joystick was pressed
    while((P2IN&BIT0) == 0) {
    }

    // CP and WP as input
    P5DIR_bit.P5DIR_5 = 0;
    P5DIR_bit.P5DIR_6 = 0;

    // card present
    while(P5IN&BIT6) {
      LCDClear();
      LCDStr ( 0, "Card not      " );
      LCDStr ( 1, "present!      " );
      LCDStr ( 3, "Please put it " );
      LCDStr ( 4, "and press to  " );
      LCDStr ( 5, "continue!     " );
      LCDUpdate();
    }

    while(P2IN&BIT0);


    // write protect
    while(P5IN&BIT5) {
      LCDClear();
      LCDStr ( 0, "Card is write " );
      LCDStr ( 1, "protect!      " );
      LCDStr ( 3, "Please correct" );
      LCDStr ( 4, "it and press  " );
      LCDStr ( 5, "to continue!  " );
      LCDUpdate();
    }

    while(P2IN&BIT0);

    if (initMMC() == MMC_SUCCESS)	// card found
    {
      //card_state |= 1;
      memset(&mmc_buffer,0,512);
      mmcReadRegister (10, 16);
      mmc_buffer[7]=0;

      // Fill first Block (0) with 'A'
      memset(&mmc_buffer,'0',512);    //set breakpoint and trace mmc_buffer contents
      mmcWriteBlock(0);
      // Fill second Block (1)-AbsAddr 512 with 'B'
      memset(&mmc_buffer,'1',512);
      mmcWriteBlock(512);

      // Read first Block back to buffer
      memset(&mmc_buffer,0x00,512);
      mmcReadBlock(0,512);
      memset(&mmc_buffer_test_1,'0',512);
      if(strncmp(&mmc_buffer[0], &mmc_buffer_test_1[0], 512)) state_mmc=0;

      // Read first Block back to buffer
      memset(&mmc_buffer,0x00,512);
      mmcReadBlock(512,512);
      memset(&mmc_buffer_test_1,'1',512);
      if(strncmp(&mmc_buffer[0], &mmc_buffer_test_1[0], 512)) state_mmc=0;

      memset(&mmc_buffer,0x00,512);
    }



      LCDClear();
      LCDStr ( 0, "Test MMC card " );

      if(state_mmc == 1)
        LCDStr ( 1, "  SUCCESS!!!  " );
      if(state_mmc == 0)
        LCDStr ( 1, "  FAILED!!!   " );


      LCDStr ( 5, " back to menu " );
      LCDUpdate();

      while(1) {
        if(((P2IN&BIT0) == 0)&&(press_joys == 1)) {
          press_joys = 0;
          Restart();
        }
        if(((P2IN&BIT0) == 1)&&(press_joys == 0)) {
          press_joys = 1;
        }
      }
  }

  // choose Graphics test
  if(pos == 3) {

    // pressed joystick
    press_joys = 0;
    DelayN(1000);
    // if joystick was pressed
    while((P2IN&BIT0) == 0) {
    }

    // visualize
    LCDClear();
    Move();
    LCDStr ( 5, " back to menu " );
    LCDUpdate();

    // led - P2.1
    P2OUT &= ~BIT1;   // low
    P2DIR |= BIT1;    // output

    // loop forever
    while(1) {
      // left
      if(!(P1IN&BIT7)) { X++; Move(); }

      // right
      if(!(P1IN&BIT4)) { X--; Move(); }

      // Up
      if(!(P1IN&BIT6)) { Y--; Move(); }

      // Down
      if(!(P1IN&BIT5)) { Y++; Move(); }

      // press joystick
      if(((P2IN&BIT0) == 0)&&(press_joys == 1)) {
        press_joys = 0;
        Restart();
      }
      if(((P2IN&BIT0) == 1)&&(press_joys == 0)) {
        press_joys = 1;
      }

    }

  }
}
