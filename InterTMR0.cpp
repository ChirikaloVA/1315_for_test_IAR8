/*****************************************************************************/
/* �� ���������  TIMR0 (IRQ) ����������-����� �� 2 ��� � 2/3 ���??????       */
/****************************************************************************/

#include <intrinsics.h>
#include <NXP/iolpc2294.h>
#include "my_macros.h"

// The following is the timer interval required in ms. Change as required.
/*********** tmr0***************/
#define TCR_ENABLE        0x1       /* timer TCR bit 0 =1-enable counting */
#define TCR_RESET         0x2       /* timer TCR bit 1 =1-reset and sinch count and pres */
#define MATCH_0_INTERRUPT 0x1       /* bit enable interrapt for MR0  */
#define MATCH_0_RESET     0x2       /* bit reset TC  for MR0  */
/*==============================*/
unsigned long ulGlobalMaxTimer0Value;
long TH,TL;
int Temp_i;           //���������� �������� ������� ��� ���� ����� �� �����  ����� ���� ��� 10.07.2008
float TEMPERFLOAT;    //������� ������� �����������
float TEMPERFLOATtek;    // ������� �����������
float MasTemper[20];
int IndMasTemper;
float TIMER0_INTERVAL;     //120000.0�������� � msek // ������ 2 ������  - ��������� �����������
__BITFLAG FLAGrez;   //��� � macros
/**************************************************************************/
void MIDTEMPER(void);
void TEMPERATURA(void);

void RESETTMR0(void);

void TimerInterrupt0(void);
/******************************************************/
//extern unsigned char gpi1;//�������
extern __BITFLAG FLAG;   //��� � macros
extern  double dPeriod;
extern unsigned int PCLK_VPB;
extern unsigned  short KODADC;
extern unsigned  short KODADCEEPROM;
extern unsigned char TABLTemperCORR[126]; //������� ������ ��������� 61 ���� �� 2 ����� +CRC 2�+1������+ 1� ����
extern void STOPEINT0(void);
extern void STARTEINT0(void);
extern void ZAPFACTOR (unsigned short factor,unsigned char nf);

extern unsigned short TEST_pause;

/**********************************************************
 * Initialise timer 0, ������ �������� � �������� ���������
 *********************************************************/
void InitialiseTimer0(void)
{
   VICVectCntl1  = 0x24;    			  		/* vector timer 0 as highest priority and enabled */
   VICVectAddr1  = (int) TimerInterrupt0;	  		/* vector timer 0 interrupt address */
   T0TCR = TCR_RESET;     //reset counter
   T0TCR = 0;             //���������� �������
   T0PC  = 0;            // Initialise the prescale counter to 0...
   T0PR  = 0;
   T0MCR = MATCH_0_INTERRUPT | MATCH_0_RESET; //int generate and counter reset
   ulGlobalMaxTimer0Value = (TIMER0_INTERVAL/dPeriod);
   //ulGlobalMaxTimer0Value *= 1000000; // max �������� �������� ��� ����
   ulGlobalMaxTimer0Value *= 50000; // max �������� �������� ��� ����
   T0MR0 = ulGlobalMaxTimer0Value;
}
/*****************************************************/
void RESETTMR0(void)
{
  T0TCR = TCR_RESET;     //reset counter
  T0IR  = 0xFF;          // Clear the interrupt bit within the IR register...
  T0TCR = TCR_ENABLE;   //start counter
}
/*****************************************************/
void TimerInterrupt0(void)
{
   //============ ���� (12.08.2009)========
    //IO0SET_bit.P0_14 = 1;
    //IO0CLR_bit.P0_14 = 1;
    //=======================================

  //if( TEST_pause )
   // ++ TEST_pause;
  FLAG.temperature = 0; // ������� ��� ���� ������ ��������
  FLAG.noEINT0 = 1;   //���� �������� ������� ���������� �� ������� ������������ � EINT0 ����������
//  FLAG.temperature = 0; // ������� ��� ���� ������ ��������
  if (!FLAG.temperWaitHL)   //���� ������ ��������
        FLAG.temperIZM = 1; //���� �������� �����������
  else
  {//��� ��������� �������
   T0TCR = 0;                      //���������� �������
 //  T0MR0 = ulGlobalMaxTimer0Value; //������������  ������ ��������� �����������
   FLAG.temperature = 1; //error temper ������ �� ��������
  }
  FLAGrez =FLAG;
  RESETTMR0();
   //============ ���� (12.08.2009)========
    //IO0SET_bit.P0_14 = 1;
    //IO0CLR_bit.P0_14 = 1;
    //=======================================
}
/***************************************************************************/
/*                    �� ��������� �����������                             */
/***************************************************************************/
void TEMPERATURA(void)
{
  struct{
        unsigned pin07 :1;
        } zP07 ;
  float tdiv;

  STOPEINT0();
  T0TCR = 0;                      //���������� �������
  T0MR0 = (500 *1000000)/dPeriod; // 500msek max ���� ��������� ����� ������ �������
  FLAG.temperWaitHL =1;
  zP07.pin07 = IO0PIN_bit.P0_7;
  RESETTMR0();      //���� ��� �������?????
  while ((IO0PIN_bit.P0_7 == zP07.pin07) && (FLAGrez.temperature == 0));//���� ���������z .pin07

  RESETTMR0();
  while ((IO0PIN_bit.P0_7 == !zP07.pin07) && (FLAGrez.temperature == 0)) ; //���� ���������
  T0TCR = 0;
  TH = T0TC;
  RESETTMR0();
  while ((IO0PIN_bit.P0_7 == zP07.pin07) && (FLAGrez.temperature == 0)) ; //���� ���������
  T0TCR = 0;
  TL    = T0TC;
  STARTEINT0();
  FLAG.temperWaitHL = 0;
  T0MR0 = ulGlobalMaxTimer0Value; //������������  ������ ��������� �����������
  RESETTMR0();
  if (TL > TH)
        tdiv = (float)TH  / (float) TL ;
  else
        tdiv = (float)TL  / (float) TH ;
  MasTemper[IndMasTemper] = 421.0 - (751.0 * (tdiv)); //������� ����������� � ������
  TEMPERFLOATtek = MasTemper[IndMasTemper];    // ������� �����������
  if ((TEMPERFLOATtek < 9.5) && (TEMPERFLOATtek > 50.5))
    FLAG.minmaxTemper = 1;      //������� ����������� ��� ���������
  IndMasTemper++;
  if (IndMasTemper == 20)
    IndMasTemper =0;      //������ ��������� �� �����
}
/****************************************************************************/
/* ���������� ������� ������� ����������� ������� ���������� */
/****************************************************************************/
void MIDTEMPER(void)
{
  float tsum;
  int i;
  tsum = 0.0;
  for (i=20; --i>=0;)
            tsum+= MasTemper[i];
  TEMPERFLOAT = tsum/20.0;  //������� ����������� �� ��������� 40 �����
}
/****************************************************************************/
/*������������� ������������, ������ ��� ������� �����*/
/*������� ���� ���� ���������: �� +10 ���� �� +50 ����*/
/* �������� ���  -10...+9 ���� � �������*/
/*���� ����������� �� ������ � ����� ������ �� ������������*/
/****************************************************************************/
void SETKOEFF(void)
{
   int i;
   if ((FLAG.temperCORR) && (!FLAG.minmaxTemper))
  {
   i = ((int)(TEMPERFLOAT+0.5)+10)<<1;  //������� ������ ����� -10 ����,����������
   //--- 25.10.2010 ---------------------
    if( (i >= 0) & (i <= 122) )
    {
      //�� ���� � ������� ������?????
      KODADC = KODADCEEPROM +(TABLTemperCORR[i]<< 8 | TABLTemperCORR[i+1]);
    }
   //--- 20.10.2010 ---------------------
   if( KODADC > 4000 )
     KODADC = 4000;
   //-----------------------------------
   /*
   if(Temp_i != i)
    ZAPFACTOR(KODADC,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA
   */
   //--- 12.01.2010 ---------------------
   if(Temp_i != i)
    ZAPFACTOR(KODADC,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA
   //-----------------------------------
   Temp_i=i;

  }//if (FLAG.temperCORR)

}//end SETKOEFF
/****************************************************************************/
/* ���������� ������� ���������� 1-� ���������� ��� �������������*/
/****************************************************************************/
void TEMPERATURA1(void)
{
  int i;
  IndMasTemper = 0;  //�������� ��������� ������ � 0
  TEMPERATURA();
  for (i=20; --i>=1;)
            MasTemper[i]= MasTemper[0];
  MIDTEMPER();
}
/****************************************************************************/
