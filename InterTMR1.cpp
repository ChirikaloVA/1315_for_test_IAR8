/********************************************************************************************/
/*        пп обработки  TIMR1 (IRQ) прерывани€ - счетчик живого времени набора спектра      */
/*******************************************************************************************/

#include <intrinsics.h>
#include <NXP/iolpc2294.h>
#include "my_macros.h"
/**************************************************************************/
#define TCR_ENABLE    0x1       /* timer TCR bit 0 =1-enable counting */
#define TCR_RESET     0x2       /* timer TCR bit 1 =1-reset and sinch count and pres */
#define MATCH_0_INTERRUPT 0x1       /* bit enable interrapt for MR0  */
#define MATCH_0_RESET     0x2       /* bit reset TC  for MR0  */

extern unsigned int TIMEIZMsek;//заданное врем€ измерени€
extern unsigned int PCLK_VPB;
extern void STOPEINT0(void);
extern __BITFLAG FLAG;   //тип в macros

static void TimerInterrupt1(void);
unsigned char gpi1;//отладка
extern unsigned short TEST_pause;
unsigned int T1TC_stop;

/*****************************************************
 * Initialise timer 1
 ****************************************************/
void InitialiseTimer1(void)
{
//   extern void handle_timer1(void);


   VICVectCntl2  = 0x25;    			  		/* vector timer 1 as highest priority and enabled */
   VICVectAddr2  = (int) TimerInterrupt1;	  		/* vector timer 1 interrupt address */

   T1TCR = TCR_RESET; //сбросили счетчик
   T1TCR = 0;         //остановили счетчик

   T1PC = 0;   // Initialise the prescale counter to 0...

   //~1uSek при (1000000:14 745 M√ц=67,82[*4убрали] )nSek*3686/1000000=~0,999938..mSek ; =4608*-при 18 432 ћ√ц=1,85..uSek;
   //(т.к. Pclk = Cclk сейчас так задано VPB)
   T1PR = PCLK_VPB;  //что соответствует 999991,294uSek~~1mSek

   T1MCR = MATCH_0_INTERRUPT | MATCH_0_RESET ;
   T1MR0 = TIMEIZMsek*1000; //  = msek задано
   T1TC_stop = 1;
//   gpi1=0;//отладка
}
/*****************************************************/
void TimerInterrupt1(void)
{
//  IO0CLR = 0xF0;   /*дл€ отладки*/
//  gpi1=~gpi1 & 0x80;
//  IO0SET = gpi1;   /*дл€ отладки*/

//  T1TCR = TCR_RESET;
// Clear the interrupt bit within the IR register...
  STOPEINT0();  //остановили набор спектра кончилось врем€,остановили счетчик
  FLAG.SPECTR = 0; //запретить накопление спектра
  T1TC_stop = T1TC;
  T1IR = 0xFF;    //сброс прерывани€
//  T1TCR = TCR_ENABLE;

}
/*****************************************************/
