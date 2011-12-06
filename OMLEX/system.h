/**********************************************************************************/
/*                                                                                */
/*    Copyright (C) 2005 OLIMEX  LTD.                                             */
/*                                                                                */
/*    Module Name    :  system module                                             */
/*    File   Name    :  system.h                                                  */
/*    Revision       :  01.00                                                     */
/*    Date           :  2006/01/26 initial version                                */
/*                                                                                */
/**********************************************************************************/

#ifndef system_def
#define system_def

/* definitions */

// POWER MODE
#define MODE_ACTIVE  0
#define MODE_LPM0    1
#define MODE_LPM1    2
#define MODE_LPM2    3
#define MODE_LPM3    4
#define MODE_LPM4    5



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
void InitFreq(void);

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
void InitFreq32kHz(void);


/****************************************************************************/
/*  Init pheriphery                                                         */
/*  Function : InitPeriph                                                   */
/*      Parameters                                                          */
/*          Input   :  Nothing                                              */
/*          Output  :  Nothing                                              */
/****************************************************************************/
void InitPeriph(void);

#endif

