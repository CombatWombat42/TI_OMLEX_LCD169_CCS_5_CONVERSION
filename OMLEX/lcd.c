//lcd_nokia.h
#include "lcd.h"
#include <io430x16x.h>

static int   LcdCacheIdx;
static int   LoWaterMark;
static int   HiWaterMark;

// represent lcd matrix
unsigned char  LcdCache [ LCD_CACHE_SIZE ];

// simple delay
void Delay(unsigned long a) { while (--a!=0); }

/****************************************************************************/
/*  Set LCD Contrast                                                        */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChr(unsigned char size, unsigned char ch ) {

  unsigned char i, c;
    unsigned char b1, b2;
    int  tmpIdx;

    if ( LcdCacheIdx < LoWaterMark )
    {
        //  Update low marker.
        LoWaterMark = LcdCacheIdx;
    }

    // if size 1X
    if ( size == FONT_1X )
    {
        for ( i = 0; i < 5; i++ )
        {
            LcdCache[LcdCacheIdx++] = FontLookup[ch - 32][i] << 1;
        }
    }
    // if size 2X
    else if ( size == FONT_2X )
    {
        tmpIdx = LcdCacheIdx - 84;

        if ( tmpIdx < LoWaterMark )
        {
            LoWaterMark = tmpIdx;
        }

        if ( tmpIdx < 0 ) return;

        for ( i = 0; i < 5; i++ )
        {
            c = FontLookup[ch - 32][i] << 1;
            b1 =  (c & 0x01) * 3;
            b1 |= (c & 0x02) * 6;
            b1 |= (c & 0x04) * 12;
            b1 |= (c & 0x08) * 24;

            c >>= 4;
            b2 =  (c & 0x01) * 3;
            b2 |= (c & 0x02) * 6;
            b2 |= (c & 0x04) * 12;
            b2 |= (c & 0x08) * 24;

            LcdCache[tmpIdx++] = b1;
            LcdCache[tmpIdx++] = b1;
            LcdCache[tmpIdx + 82] = b2;
            LcdCache[tmpIdx + 83] = b2;
        }

        //  Update x cursor position.
        LcdCacheIdx += 11;
    }


    if ( LcdCacheIdx > HiWaterMark )
    {
        //  Update high marker.
        HiWaterMark = LcdCacheIdx;
    }

    //  Horizontal gap between characters.
    LcdCache[LcdCacheIdx++] = 0x00;
}

/****************************************************************************/
/*  Clear LCD                                                               */
/*  Function : LCDClear                                                     */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDClear(void) {

    int i;

    // loop all cashe array
    for ( i = 0; i < LCD_CACHE_SIZE; i++ )
    {
        LcdCache[i] = 0x00;
    }

    //  Reset watermark pointers.
    LoWaterMark = 0;
    HiWaterMark = LCD_CACHE_SIZE - 1;
}

/****************************************************************************/
/*  Set LCD Contrast                                                        */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDUpdate ( void )
{
    int i;

    if ( LoWaterMark < 0 )
        LoWaterMark = 0;
    else if ( LoWaterMark >= LCD_CACHE_SIZE )
        LoWaterMark = LCD_CACHE_SIZE - 1;

    if ( HiWaterMark < 0 )
        HiWaterMark = 0;
    else if ( HiWaterMark >= LCD_CACHE_SIZE )
        HiWaterMark = LCD_CACHE_SIZE - 1;

    //  Set base address according to LoWaterMark.
    LCDSend((0x80 | (LoWaterMark % LCD_X_RES)), SEND_CMD );
    LCDSend((0x40 | (LoWaterMark / LCD_X_RES)), SEND_CMD );

    //  Serialize the video buffer.
    for ( i = LoWaterMark; i <= HiWaterMark; i++ )
    {
      LCDSend( LcdCache[i], SEND_CHR );
    }

    //  Reset watermark pointers.
    LoWaterMark = 0;
    HiWaterMark = LCD_CACHE_SIZE - 1;

}

/****************************************************************************/
/*  Send to LCD                                                             */
/*  Function : LCDSend                                                      */
/*      Parameters                                                          */
/*          Input   :  data and  SEND_CHR or SEND_CMD                       */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDSend(unsigned char data, unsigned char cd) {

  // Enable display controller (active low).
  P3OUT_bit.P3OUT_0 = 0;  // STE0 (CE)

  // command or data
  if(cd == SEND_CHR) {
    P3OUT_bit.P3OUT_2 = 1;  // SOMI0 (D/S)
  }
  else {
    P3OUT_bit.P3OUT_2 = 0;  // SOMI0 (D/S)
  }

  ///// SEND SPI /////

  //send data
  U0TXBUF = data;

  //Wait for ready U0TXBUF
  while((U0TCTL & TXEPT) == 0);

  // Disable display controller.
  P3OUT_bit.P3OUT_0 = 1;  // STE0 (CE)

}

/****************************************************************************/
/*  Init LCD Controler                                                      */
/*  Function : LCDInit                                                      */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDInit(void)
{

  //  Pull-up on reset pin.
  P5OUT_bit.P5OUT_4 = 1;

  // Pin configuration - all as output
  P3DIR_bit.P3DIR_0 = 1;  // STE0
  P3DIR_bit.P3DIR_1 = 1;  // SIMO0
  P3DIR_bit.P3DIR_2 = 1;  // SOMI0 (D/S)
  P3DIR_bit.P3DIR_3 = 1;  // ULCK0
  P5DIR_bit.P5DIR_4 = 1;  // RES

  // Pin select function
  P3SEL_bit.P3SEL_0 = 0;  // STEO0
  P3SEL_bit.P3SEL_1 = 1;  // SIMO0
  P3SEL_bit.P3SEL_2 = 0;  // SOMI0 (D/S)
  P3SEL_bit.P3SEL_3 = 1;  // ULCK


  //  Toggle display reset pin.
  P5OUT_bit.P5OUT_4 = 0;
  Delay(10000);
  P5OUT_bit.P5OUT_4 = 1;

  // Init SPI
  U0CTL   = 0x16;   // SPI Mode, 8bit, Master mode
  U0TCTL  = 0xB2;   // 3pin Mode, clock->SMCLK, no CKPL (poliarity), no CKPH (phase)
  //U0TCTL  = 0x30;   // 4pin Mode, clock->SMCLK, no CKPL (poliarity), no CKPH (phase)

  //U0BR0   = 0x2A;   // 19200 -> 19200 = ~800Khz/46
  U0BR0   = 0x15;     // 38000 -> 38000 = ~800Khz/21
  U0BR1   = 0x00;
  UMCTL0  = 0x00;   // in spi mode don't used

  ME1     = 0x40;   // Enable SPI0
  ME2     = 0x01;   // Enable SPI0

  // Disable display controller.
  P3OUT_bit.P3OUT_0 = 1;  // STE0

  Delay(100);

  // Send sequence of command
  LCDSend( 0x21, SEND_CMD );  // LCD Extended Commands.
  LCDSend( 0xC8, SEND_CMD );  // Set LCD Vop (Contrast).
  LCDSend( 0x06, SEND_CMD );  // Set Temp coefficent.
  LCDSend( 0x13, SEND_CMD );  // LCD bias mode 1:48.
  LCDSend( 0x20, SEND_CMD );  // LCD Standard Commands, Horizontal addressing mode.
  LCDSend( 0x08, SEND_CMD );  // LCD blank
  LCDSend( 0x0C, SEND_CMD );  // LCD in normal mode.

  //  Reset watermark pointers.
  LoWaterMark = 0;
  HiWaterMark = LCD_CACHE_SIZE - 1;

  // Clear and Update
  LCDClear();
  LCDUpdate();

}

/****************************************************************************/
/*  Change LCD Pixel mode                                                   */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDPixel (unsigned char x, unsigned char y, unsigned char mode )
{
    unsigned int    index   = 0;
    unsigned char   offset  = 0;
    unsigned char   data    = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    index = ((y / 8) * 84) + x;
    offset  = y - ((y / 8) * 8);

    data = LcdCache[index];

    if ( mode == PIXEL_OFF )
    {
        data &= (~(0x01 << offset));
    }
    else if ( mode == PIXEL_ON )
    {
        data |= (0x01 << offset);
    }
    else if ( mode  == PIXEL_XOR )
    {
        data ^= (0x01 << offset);
    }

    LcdCache[index] = data;

    if ( index < LoWaterMark )
    {
        //  Update low marker.
        LoWaterMark = index;
    }

    if ( index > HiWaterMark )
    {
        //  Update high marker.
        HiWaterMark = index;
    }

}

/****************************************************************************/
/*  Change LCD Pixel mode                                                   */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDChrXY (unsigned char x, unsigned char y, unsigned char ch )
{
    unsigned int    index   = 0;
    unsigned int    offset  = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

    //index = ((y / 8) * 84) + x;
    //offset  = y - ((y / 8) * 8);

    //index = y*84+x;
    index = (x*48+y*84)/8;



    for ( i = 0; i < 5; i++ )
    {
      LcdCache[index] = FontLookup[ch - 32][i] << 1;
      index++;
    }

}

/****************************************************************************/
/*  Set LCD Contrast                                                        */
/*  Function : LcdContrast                                                  */
/*      Parameters                                                          */
/*          Input   :  contrast                                             */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDContrast(unsigned char contrast) {

    //  LCD Extended Commands.
    LCDSend( 0x21, SEND_CMD );

    // Set LCD Vop (Contrast).
    LCDSend( 0x80 | contrast, SEND_CMD );

    //  LCD Standard Commands, horizontal addressing mode.
    LCDSend( 0x20, SEND_CMD );
}


/****************************************************************************/
/*  Send string to LCD                                                      */
/*  Function : LCDStr                                                       */
/*      Parameters                                                          */
/*          Input   :  size text, text                                      */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void LCDStr(unsigned char size, unsigned char *dataPtr ) {

  // loop to the and of string
  while ( *dataPtr ) {
    LCDChr( size, *dataPtr++ );
  }
}



