/******************************************************/
/*  �� ��������� ��������� (FIQ) ���������� �� P0.16 */
/*  �� ������ � ���� ������� � �����. ��������      */
/****************************************************/
#include <intrinsics.h>
#include <NXP/iolpc2294.h>
#include "my_macros.h"

//���� ���������� ���������� (VICENABLE...)
#define INTERENT0     0x4000   /* bit 14h int*/
#define TCR_DISABLE   0x0       /* timer TCR bit 0 =0-disable counting */
#define TCR_ENABLE    0x1       /* timer TCR bit 0 =1-enable counting */

extern void STOPEINT0(void);
//unsigned char SPECTR[3075],BUFFSPECTR[3075];

unsigned char BUFFSPECTR[3075];
__no_init unsigned long SPECTR[1025];


#define EMCDAC  0x83000000;
#define EMCACP  0x82000000;
//extern __BITFLAG FLAG;   //��� � macros
extern __BITFLAG FLAG;   //��� � macros

extern unsigned int T1TC_temp;

unsigned int DAC;
__fiq __arm  void IntEINT0(void);
unsigned char FLAGdac128;
unsigned char FLAGOver3byte;

unsigned short tmp;
/*****************************************************
* Initialise EINT0
*****************************************************/
void InitialiseEINT0(void)
{
    PINSEL1 = 0x1;               /* select P0:16 as EINT0 */
    //VPBDIV   = 0;
    EXTMODE_bit.EXTMODE0 = 1;   // ������ �� �����
    EXTINT = 0x1;                /* clear EINT0 interrupt status */
    VICIntSelect  = INTERENT0;   /* FIQ  */
    FLAG.SPECTR = 1;             /* ������������� ���� ���������� �������*/
}
/********************************************************/
/*��������� ���������� EINT0 - ������ �� ����� � startup*/
/*******************************************************/
__fiq __arm void IntEINT0(void)
{
   unsigned int ADRCANAL;
   unsigned int IZM;
//   int irab;  /*��� �����*/
   unsigned int  DACtek;

   unsigned int* pulMemory;

   FLAGOver3byte=0;      //FALSE-����� ����� ������������ 3-� ����
/**********************************************************
     ������ �������� �������� DAC -> CS3 �� ������� ������*/
   pulMemory = (unsigned int*) EMCDAC;
   //*pulMemory = DAC<<16;
   //DAC=0x55;
   //--11.10.2011-- ��� ������������ ���� ��� ��� �������������
    *pulMemory = DAC;
   //*pulMemory = 0;

     DACtek=DAC;
   //DACtek = 0;
/***************************************************************/
/* �����������/���������� DAC */
     if (FLAGdac128 == 0)
            { DAC++;
              if (DAC == 0x7F)
                    FLAGdac128=1;
              }
      else
            { DAC--;
              if (DAC==0)
                    FLAGdac128=0;
              }
//     DELAYTACKT(irab,43);         //�������� �� -3uSek ��� ������������ �������

//----- ��� WDT ----------------------------
    T1TC_temp = T1TC;
/*******************************************/
     IO0CLR_bit.P0_24 = 1;        //������� ���� P0.24=0
     T1TCR = 0;         //���������� ������� TMR1 - ������� �����
     EXTPOLAR_bit.EXTPOLAR0 = 1;
     EXTINT = 0x1;
  /************************************************************************
 ���� �� ������������ �� ���������, ��������� ������� ���������� */
      if  (IO2PIN_bit.P2_29)    //((FLAG.STAB) ||)
//STARTACP:
     { //������ ����� ��������������
     IO0CLR_bit.P0_23 = 1; //P0.23=0
     IO0SET_bit.P0_23 = 1;        //����� ��������������
/*************************************************************/
/*  ������ ������(������ ������) �� ������� ������ - CS2*/
     pulMemory = (unsigned int*) EMCACP;
     ADRCANAL =*pulMemory;
     ADRCANAL&=0x00000FFF;
    // ADRCANAL = ((ADRCANAL-DACtek)>>2)*3;// >>16
     ADRCANAL = ((ADRCANAL-DACtek)>>2);
/**********************************************************/
/* ������������ ��������� � ������ ADRCANAL *****/
     //if (ADRCANAL < 3072)
     if (ADRCANAL < 1024)
     {
     //IZM = SPECTR[ADRCANAL] | (SPECTR[ADRCANAL+1]<<8) | (SPECTR[ADRCANAL+2]<<16);
        IZM = SPECTR[ADRCANAL];
        IZM++;
      if ( IZM < 0xFFFFFF && FLAG.SPECTR )
     //if (IZM < 0xFFFFFF)
      {//����� ��������
      //SPECTR[ADRCANAL]   = IZM;
      //SPECTR[ADRCANAL+1] = IZM >> 8;
      //SPECTR[ADRCANAL+2] = IZM >> 16;
      //============ ���� (30.09.2011)========
        /*
        if( ADRCANAL == 262)
        {
          IO0SET_bit.P0_14 = 1;
          IO0CLR_bit.P0_14 = 1;
        }
        */
      //=======================================
        SPECTR[ADRCANAL]   = IZM;
      }
/**************************************************************/
     else
         {
         FLAGOver3byte = 1;       //������������ 3-� ����
         //---- ������� 14.10.2011------
         /*
         STOPEINT0();
         FLAG.SPECTR = 0;
         */
         FLAG.testerror = 1;
         //------------------------------
         }
     }//if ADRCANAL < 3072
}//����� ����� ��������������
/**********************************************************/
/* ����� ���������, ������ �� ��������� ��������� */
     //RESETDET:
     if (!FLAGOver3byte)             //���� ������������ 4-� ����??-���������� ���������
        { //������ ����� ������ ��������� � ������ ��������������
        IO0CLR_bit.P0_5 = 1;  //P0.5=0
        asm("nop");
        asm("nop");
        IO0SET_bit.P0_5 = 1;         //����� ���������
//        while (!IO0PIN_bit.P0_16);   //���� �����  �� �������
/**********************************************************/
        while (!(EXTINT && 1))
        {
          if( IO0PIN_bit.P0_16 )
            break;
        }
        EXTINT = 0x1;             //����� ����������
        EXTPOLAR_bit.EXTPOLAR0 = 0;
        //while (EXTINT & 1)
        //    EXTINT = 0x1;             //����� ����������
        IO0SET_bit.P0_24 = 1;        //������� ����
        T1TCR = TCR_ENABLE;          //��������� ������� TMR0
        } //����� ����� ������ ��������� � ������ ��������������
 FLAG.noEINT0 = 0; //����� ����� �������� ������� ���������� �� ������� ����� ������ 2 ������ � TMR0 ����������
}//����� ��
/**************************************************************************/
/*  ������ � ���� ������� ������, �������� ������, ������������ ��������  */
/**************************************************************************/
/**************************************************************************/
/*  ������ � ���� ������� ������, �������� ������, ������������ ��������  */
/**************************************************************************/
void ZAPFACTOR (unsigned short factor,unsigned char nf)
//unsigned short factor; /* ����� ��� ����������� */
//unsigned char nf; /*������� 0 - ������ �����, 1 = ������� �����,2=����������� �������� */
{

  int i,irab,pausacik;
  pausacik = 43;
  __disable_interrupt();
  IO0DIR_bit.P0_3 = 1;  /*P0.3 Di/o ����������� �� �����*/
  DELAYTACKT(irab,pausacik);
  //tmp=factor;
  //factor = factor << 3;  /* �������� ����� - ����� 12-��������*/

  switch( nf )
  {
  case 0:
    {
      factor= factor & 0x7FFF; //��� 16 � 0 ��� ������ �
      break;
    }
  case 1:
    {
      factor= factor | 0x8000; //��� 16 � 1 ��� ������ �
      break;
    }
  case 2:
    {
      factor = factor << 2;  /* �������� ����� - ����� 12-��������*/
      factor= factor & 0x3FFF;
      break;
    }
  }
  if (nf==2)
        IO2CLR_bit.P2_26 =1;  /*CS6 = 0 ����������� ��������*/
    else
    {
    IO2CLR_bit.P2_25 =1;  /*CS5 = 0 ������*/
    factor = factor | 0x4000;  //��������� ��������������
    }
  DELAYTACKT(irab,pausacik);
 // for (i = 12; --i >=0;)

  for (i = 16; --i >=0;)
  {
//--11.10.2011--- ����������� �������� ��������� ��� ----------------
     if ((factor & 0x8000) == 0)
      IO0CLR_bit.P0_3 = 1;   //��� DI/O = 0
      else                   // ���
        IO0SET_bit.P0_3 = 1; //��� DI/O = 1
//--11.10.2011--- ����������� �������� ��������� ��� ----------------
    DELAYTACKT(irab,pausacik);

    IO0SET_bit.P0_2 = 1;      //SCK = 1

    DELAYTACKT(irab,pausacik);
    //----11.10.2011-----------------
    /*
    if ((factor & 0x8000) == 0)
      IO0CLR_bit.P0_3 = 1;   //��� DI/O = 0
      else                   // ���
        IO0SET_bit.P0_3 = 1; //��� DI/O = 1
    */
   //------------------------------------
    DELAYTACKT(irab,pausacik);
    IO0CLR_bit.P0_2 = 1;      //SCK = 0
    DELAYTACKT(irab,pausacik);
    factor = factor << 1; //�������� �� 1 ������
  }

  //IO0SET_bit.P0_2 = 1;      //SCK = 1   9.07.2008
  IO0CLR_bit.P0_2 = 1;      //SCK = 0
  DELAYTACKT(irab,pausacik);

  if (nf==2)
  {
        IO2SET_bit.P2_26 =1;  /*CS6 = 1 ����������� ��������*/
        IO2SET_bit.P2_25 =1;  /*CS5 = 1 ������*/
  }
    else
    {
    IO2SET_bit.P2_25 =1;  /*CS5 = 1 ������*/
    IO2SET_bit.P2_26 =1;  /*CS6 = 1 ����������� ��������*/
    }
  DELAYTACKT(irab,pausacik);
  if (nf==0 || nf==1)
  {
  IO2CLR_bit.P2_30 = 1;      //LDA = 0 - ������ ����� ��� ����������� ��������
  DELAYTACKT(irab,pausacik);
  IO2SET_bit.P2_30 = 1;      //LDA = 1 - ������ ����� ��� ����������� ��������
  DELAYTACKT(irab,pausacik);
  }
 /* if (nf == 1)
  {
    IO2CLR_bit.P2_31 = 1;      //LDB = 0 - ������� �����
    DELAYTACKT(irab,pausacik);
    IO2SET_bit.P2_31 = 1;      //LDB = 1 - ������� �����
  }
  else
  {
    IO2CLR_bit.P2_30 = 1;      //LDA = 0 - ������ ����� ��� ����������� ��������
    DELAYTACKT(irab,pausacik);
    IO2SET_bit.P2_30 = 1;      //LDA = 1 - ������ ����� ��� ����������� ��������
    DELAYTACKT(irab,pausacik);
  }*/
  IO0DIR_bit.P0_3 = 0;  /*P0.3 Di/o ����������� �� ����*/
  __enable_interrupt();

}

void ZAPFACTOR2 (unsigned short factor,unsigned char nf) //������ ����
//unsigned short factor; /* ����� ��� ����������� */
//unsigned char nf; /*������� 0 - ������ �����, 1 = ������� �����,2=����������� �������� */
{
  int i,irab,pausacik;
  pausacik = 43;
  IO0DIR_bit.P0_3 = 1;  /*P0.3 Di/o ����������� �� �����*/
  DELAYTACKT(irab,pausacik);
  factor = factor << 3;  /* �������� ����� - ����� 12-��������*/
  if (nf==2)
        IO2CLR_bit.P2_26 =1;  /*CS6 = 0 ����������� ��������*/
    else
    IO2CLR_bit.P2_25 =1;  /*CS5 = 0 ������*/
  DELAYTACKT(irab,pausacik);
  for (i = 12; --i >=0;)
  {
    IO0SET_bit.P0_2 = 1;      //SCK = 1
    factor = factor << 1; //�������� �� 1 ������
    DELAYTACKT(irab,pausacik);
    if ((factor & 0x8000) == 0)
      IO0CLR_bit.P0_3 = 1;   //��� DI/O = 0
      else                   // ���
        IO0SET_bit.P0_3 = 1; //��� DI/O = 1
    DELAYTACKT(irab,pausacik);
    IO0CLR_bit.P0_2 = 1;      //SCK = 0
    DELAYTACKT(irab,pausacik);
  }

  IO0SET_bit.P0_2 = 1;      //SCK = 1
  DELAYTACKT(irab,pausacik);

  if (nf==2)
        IO2SET_bit.P2_26 =1;  /*CS6 = 1 ����������� ��������*/
    else
    IO2SET_bit.P2_25 =1;  /*CS5 = 1 ������*/
  DELAYTACKT(irab,pausacik);

  if (nf == 1)
  {
    IO2CLR_bit.P2_31 = 1;      //LDB = 0 - ������� �����
    DELAYTACKT(irab,pausacik);
    IO2SET_bit.P2_31 = 1;      //LDB = 1 - ������� �����
  }
  else
  {
    IO2CLR_bit.P2_30 = 1;      //LDA = 0 - ������ ����� ��� ����������� ��������
    DELAYTACKT(irab,pausacik);
    IO2SET_bit.P2_30 = 1;      //LDA = 1 - ������ ����� ��� ����������� ��������
    DELAYTACKT(irab,pausacik);
  }
  IO0DIR_bit.P0_3 = 0;  /*P0.3 Di/o ����������� �� ����*/

}
/*****************************************************/

__irq __arm void prefetch_handler(void)
{
   if( IO2PIN_bit.P2_17 )
      {
        IO2CLR_bit.P2_17 = 1;
      }
      else
      {
        IO2SET_bit.P2_17 = 1;
      }
	
}

__irq __arm void data_handler(void)
{
  if( IO2PIN_bit.P2_18 )
      {
        IO2CLR_bit.P2_18 = 1;
      }
      else
      {
        IO2SET_bit.P2_18 = 1;
      }
	
}

