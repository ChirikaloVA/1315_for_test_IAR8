
#include <intrinsics.h>
#include <NXP/iolpc2294.h>
//#include <iolpc2294.h>
#include <string.h>
#include <setjmp.h>
#include "my_macros.h"
/********************************/
/* ��� �������
#define TARGET_BOARD_FREQ 18432
#define VPBDIV01  0x00000001
#define DELAYTACKT(CJ,CI) for (CJ = (CI-1/3);  --CJ >= 0;);
 typedef struct
{
  unsigned noEINT0      :1;
  unsigned temperature  :1;
  unsigned ErrtempTABL  :1;
  unsigned testerror    :1;
  unsigned SPECTR       :1;
  unsigned STAB         :1;
  unsigned UART_RS      :1;
  unsigned error        :1;
  unsigned tuning       :1;
  unsigned EEPROMAW     :1;
  unsigned EEPROMDW     :1;
  unsigned EEPROMAR     :1;
  unsigned EEPROMDR     :1;
  unsigned temperWaitHL :1;
  unsigned temperIZM    :1;
  unsigned temperCORR   :1;
  unsigned CRCkoeff     :1;
  unsigned minmaxTemper :1;
  unsigned EEPROMSP     :1;
} __BITFLAG;*/
///////////////////////////////

/*==============================*/
#define PowerPCONP  0x80E   /*- ��� TMR0; 0x8E ������� - PCONP bit 1-T0,2-T1,3-urt0,7-I2C, 11-EMC*/
#define INTERENT0   0x4000 /* bit 14h int*/
#define INTERTMRO   0x10   /* bit 4 int*/
#define INTERTMR1   0x20   /* bit 5 int*/
#define TCR_ENABLE  0x1    /* timer TCR bit 0 =1-enable counting */
#define INTERUART0  0x40   /* bit 6 int*/
/****************************************/
void ClearCOMMAND (void);
/****************************************/
 __BITFLAG FLAG;   //��� � macros
extern unsigned int DAC;
//extern unsigned char SPECTR[3075];

extern unsigned char BUFFSPECTR[3075];
extern unsigned long SPECTR[1025];

extern unsigned short CRChi,CRClo;
extern int ByteCurrentIzm;
extern float MasTemper[20];
extern unsigned char lock;
extern unsigned char flagTIME2;  //���� ��e��� ������� �� 2 �����
extern unsigned char MasEEPROMKoef[17];
extern unsigned char TimeByte;    //���-�� ���� � ������������� ������� 4 ��� 2


extern int Temp_i;

/****************************************/
extern void InitialiseEINT0(void);
extern void InitialiseEINT2LoHi(void);
extern void InitialiseTimer0(void);
extern void InitialiseTimer1(void);
extern void InitialiseUART0(unsigned char speed);
extern void COM3(void);
extern void COM4(void);
extern void COM5(void);
extern void COM6(void);
extern void COM7(void);
extern void COM8(void);
extern void COMB(void);
extern void COM10(void);
extern void COM11(void);
extern void COM12(void);
extern void COM13(void);
extern void COM15(void);
extern void COM16(void);
extern void COM17(void);

extern void COM20(void);

extern void COMerr(char kod);
extern void CRC16(unsigned char mas[],short usDataLen,char flFF,char flzapCOMMand);
extern void STOPEINT0(void);
extern void ZAPFACTOR (unsigned short factor,unsigned char nf);

extern void InitialiseI2C(unsigned char nn,unsigned char Flread);
extern void ReadEEPROMkoef(void);
extern void ReadTABLtemper(void);
extern void TEMPERATURA1(void);
extern void MIDTEMPER(void);
extern void TEMPERATURA(void);
extern void SETKOEFF(void);
extern void ReadEEPROMIntervalTemper(void);

extern void i2c_lpc_init(void);


/****************************************/
void ClearSPECTR (void);
void DeInitialiseI2C(void);

unsigned char COMMAND[16];
unsigned int ByteCOMMAND;
unsigned int TIMEIZMsek;//�������� ����� ���������
unsigned  short KODADC;
unsigned  short KODADCEEPROM;
unsigned  short MINPOROG;
unsigned  short MAXPOROG;
unsigned int PCLK_VPB;
double dPeriod;
unsigned short TEST_pause;
unsigned int SoftTimerCT;
unsigned int SoftTimerCT_save;
unsigned int SoftTimerCT_save1;
unsigned int SoftTimerVal;
__no_init unsigned int T1TC_temp;
int icikl_n; //��� ��� ��������
jmp_buf JmpMain;

unsigned  short KODADC_temp;
unsigned char FL_test;

//======== ��� ������������ (14.12.2015) ==============
unsigned char Test_WDT_cnt = 0;
unsigned char WDMOD_temp;
//======== ��� ������������ (14.12.2015) ==============

/************************************************/
__irq __nested __arm void Pirq (void)
{
  unsigned int vector;
  void (*interrupt_task)();

  // Called at 1000 Hz rate.
  vector = VICVectAddr; // Get interrupt vector.
  interrupt_task = (void(*)())vector;
  //(*interrupt_function)(); // Call vectored interrupt function.

  //VICVectAddr = 0; // Clear interrupt in VIC.
 __enable_interrupt(); // Allow other IRQ interrupts
                        //to be serviced from this
                        //point.
  (*interrupt_task)();  // Execute the task associated
                        //with this interrupt.
  __disable_interrupt();
   VICVectAddr = 0; // Clear interrupt in VIC.
}
/*****************************************************/
void ClearCOMMAND (void)
{
  ByteCOMMAND = 0;
  FLAG.UART_RS = 0;
}
/*****************************************************/
//void ClearSPECTR (void)
//{
//  int i;
//
//  for (i=3072; --i >=0;)
//              SPECTR[i] = 0;
//}
void ClearSPECTR (void)
{
  memset(SPECTR,0,sizeof(SPECTR));
}
/*****************************************************/
void InitialiseEMC (void)
{
PINSEL2 = 0x00014114; //0x0E6141E4;
BCFG3_bit.IDCY = 0;
BCFG3_bit.WST1=0;
BCFG3_bit.WST2=0;
BCFG3_bit.RBLE = 1; //��� DAC
BCFG3_bit.MW = 2;

BCFG2_bit.IDCY = 0;
BCFG2_bit.WST1=0;
BCFG2_bit.WST2=0;
BCFG2_bit.RBLE = 1; //��� ���
BCFG2_bit.MW = 1;
}

//-------------���������� �� WDT ---------------------
//----------------------------------------------------

void WDT_Interupt(void)
{
  //WDMOD_bit.WDTOF = 0;
  //WDMOD_bit.WDEN = 1;
  // __disable_interrupt();
  //WDFEED = 0xAA;
  //WDFEED = 0x55;
  //__enable_interrupt();
  //longjmp(JmpMain,1);
  ++icikl_n;

}
/*****************************************************/
/* ��������������� I2C, ����� ������� � ����������  */
/****************************************************/
void DeInitialiseI2C(void)
{
   //while(lock == 1);	//Wait for interrupt to signal end of I2C activity????????????
   //PINSEL0   = (PINSEL0 & 0xFFFFFF0F);   /* enable GPIO Port0.2,Port0.3 bus pins */
   //PCONP =PowerPCONP; //����� ������� ��� I2C
   //VICIntEnClear = 0x00000200; /*���������� ���������� �� I2C*/
}
int main()
{
  int i,adrcom15;
  void (*AdrCOMMAND)();

  __disable_interrupt();
  PINSEL0 = 0;
  MEMMAP = 1;  //1-to ROM-flash, 2-to ram-ozy;
  MAMCR = 2;  //�������� ������ ������������� MAM
  MAMTIM = 3; //3 ����� �������� ��� ������� �� ����

// ��������� FLACH � 3 ����
//  � 1 ��� �� ������ - �������� ��������� ���� �� �������� �������� ����� �� �������
/*  PLLCFG =  0x00000022;   //��������� � 3 ���� �=2+1, �= 2+1
  PLLCON =  0x00000001;
  PLLFEED = 0x000000AA;
  PLLFEED = 0x00000055;
  while (!(PLLSTAT & 0x00000400));
  PLLCON =  0x00000003;
  PLLFEED = 0x000000AA;
  PLLFEED = 0x00000055;*/

//  VPBDIV   = VPBDIV01;    //VPB - Pclk=Cclk - 01
  APBDIV_bit.APBDIV   = 0;    //VPB - Pclk=Cclk - 01
  APBDIV_bit.XCLKDIV = 1;
  //POWER
  PCONP    = PowerPCONP;
  PCLK_VPB = TARGET_BOARD_FREQ;
  // Remap interrupt vectors to R0M...
  VICIntEnClear = 0xFFFFFFFF;  //����� ���������
  VICIntEnable  = 0x00000000;


//������������� ����� ����������""""""""""""""""""""""""""""""

  *(int*)&FLAG = 0; //������ ��� ������//////////////////////////////////////////////////////
  DAC = 0;
  ByteCurrentIzm = 0;
  flagTIME2 = 0;  //���� ������ ������� �� 2 �����
  TimeByte = 4; // ������� ��� �� 4-� ������� �����

  Temp_i=0;     //10.07.2008

  ClearCOMMAND();

  TIMEIZMsek = 0x418937;  //=4294967 ������ - ������������ ����� ��������� � ���� ~ 49,7����
// �������� ������� ��� ��������~~~~~~~~~~~~~~~~~~~~~~~~~~
   dPeriod = 1000000.0 / (float)PCLK_VPB; // 54,253...nsek � ��� Pclk = Cclk


   //******* ���� ***************
   //PINSEL0_bit.P0_3 = 0;
   //PINSEL0_bit.P0_2 = 0;
   //IO0DIR_bit.P0_3 = 1;
   //IO0DIR_bit.P0_2 = 1;
   //IO0DIR_bit.P0_0 = 1;
   //while(1)
   //{
   //  IO0SET_bit.P0_0 = 1;
   //  IO0SET_bit.P0_3 = 1;
   //  IO0SET_bit.P0_2 = 1;
   //  IO0CLR_bit.P0_3 = 1;
   //  IO0CLR_bit.P0_2 = 1;
   //  IO0CLR_bit.P0_0 = 1;
   //}

   //***************************
//======== ��� ������������ (1.12.2015) ==============
  IO2DIR_bit.P2_16 = 0;
  IO2DIR_bit.P2_17 = 0;
  IO2DIR_bit.P2_18 = 0;
  IO2DIR_bit.P2_19 = 0;
  WDMOD_temp = WDMOD;
//======== ��� ������������ (1.12.2015) ==============
//������ ��� ������������� EEPROM""""""""""""""""""""""""
 // __enable_interrupt();
  //==== ������������ ����� ������ ������ � ������ (21.04.2009) ===
 // i2c_lpc_init();
 // ReadEEPROMkoef();
 // ReadTABLtemper();
 // ReadEEPROMIntervalTemper();
  //==================================================
  //ReadEEPROMkoef();
  //ReadTABLtemper();
  //ReadEEPROMIntervalTemper();
  //DeInitialiseI2C();
// �����""""""""""""""""""""""""""""""""""""""""
//�������  IO0DIR = 0x018000F4;  // 1-�� ����� - ����������(������� P0.4..P0.7������) P0.23,P0.24-�����_RB0,RB1,P0.16-RA0-����
  IO0DIR = 0x01800024; // ����� - 2,5,23,24 P0.3-���� ��� ����� � ����������� ��� ������
 // IO0SET = 0xFFFFFFFF; // �� ���� ������� 1
  IO0DIR_bit.P0_3=1;
  IO0SET = 0xFFFFFFF9; // �� ���� ������� 1
  IO0CLR = 0x6; // �� ���� ������� 0
  IO2DIR = 0xCFFF0000; // ����� - 24:27,30,31,16:23
  IO2SET = 0xFFFFFFFF; // �� ���� ������� 1

  //IO2CLR_bit.P2_26 = 1;


  IO3DIR = 0x0B000000; // ����� - 24,25,27
  IO3SET = 0xFFFFFFFF; // �� ���� ������� 1

  //======== ��� ������������ (30.09.2011) ==============
  IO0DIR_bit.P0_14 = 0;
  //=====================================================
 // IO0SET_bit.P0_23 = 1;         //����� ��������������
 // IO0SET_bit.P0_5  = 1;         //����� ���������
 // IO0SET_bit.P0_24 = 1;        //������� ����
//==== ����� ��������� �� ���� ��������� ���� (29.04.2009) ===
  ZAPFACTOR(1200,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA
  ZAPFACTOR(2000,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA
  i2c_lpc_init();
  ReadEEPROMkoef();
  ReadTABLtemper();
  ReadEEPROMIntervalTemper();

// Enable the interrupt""""""""""""""""""""""""""""""""""""""""
  InitialiseTimer0();
  InitialiseTimer1();
  InitialiseUART0(MasEEPROMKoef[7]);
  InitialiseEINT0();
  InitialiseEMC();

// Clear the interrupt bit within the IR register...
  T0IR = 0xFF;
  T1IR = 0xFF;
  T1TC =0;  //����� �������� TMR1
  T1TCR = TCR_ENABLE; //���������� �� ���� ��R1
  T0TC =0;  //����� �������� TMR0
  T0TCR = TCR_ENABLE; //���������� �� ���� ��R0

  //if( WDMOD_bit.WDTOF )
  if( WDMOD && 0x04 )
  {
    //IO2SET_bit.P2_19 = 1;
    WDMOD_bit.WDTOF = 0;
    T1TC = T1TC_temp;
    //IO2CLR_bit.P2_19 = 1;
  }
  else
  {
    icikl_n = 0;
    T1TC_temp = 0;
    ClearSPECTR();
    ZAPFACTOR(MAXPOROG,1);
    ZAPFACTOR(MINPOROG,0);
    ZAPFACTOR(KODADC,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA
  }

   /* ������������"""""""""""""""""""""""""""""""""*/



  VICIntEnable =VICIntEnable | INTERENT0 | INTERTMRO | INTERUART0 | INTERTMR1;  //���������� EINT0 � TMR0, UART0
  __enable_interrupt();

  //**********===================******************
  //TEST_pause=0;

  //for(TEST_pause=0;TEST_pause < 2000; ++TEST_pause)
  //{
  //  DELAYTACKT(icikl_n,10000); //�������� �� 100 ������-54uSek
  //}
  //IO0DIR_bit.P0_3=0;
  //IO0SET = 0xFFFFFFFF; // �� ���� ������� 1
  //IO2SET_bit.P2_26 = 1;
  /* ������������"""""""""""""""""""""""""""""""""*/
 //ZAPFACTOR(MAXPOROG,1);
 //ZAPFACTOR(MINPOROG,0);
 //ZAPFACTOR(KODADC,2); //����� ������� KODADC-���������� ������ ����� ��� �������  LDA

/* ������ ����������"""""""""""""""""""""""""""""""""*/

/*�����������"""""""""""""""""""""""""""""""""""*/
  TEMPERATURA1();
  //setjmp(JmpMain);
  //=========== WDT ===================

  VICVectCntl3_bit.NUMBER = VIC_WDT;
  VICVectCntl3_bit.ENABLED = 1;
  VICVectAddr3 = (int) WDT_Interupt;
  WDTC = WDT_value;
  WDMOD_bit.WDEN = 1; //��� ���������� �� WDT ��� ������
  WDMOD_bit.WDRESET = 1;
  VICIntEnable_bit.INT0 = 1;
  __disable_interrupt();
  WDFEED = 0xAA;
  WDFEED = 0x55;
  __enable_interrupt();

  //------------------3.10.2011-----------------------

  ZAPFACTOR(KODADC,2);

  //--------------------------------------------------


  //���� �������� ������
  while(1)
  {
    //============ ���� (1.12.2015)========

      if( IO2PIN_bit.P2_16 )
      {
        FL_test = 1;
        IO2CLR_bit.P2_16 = 1;
      }
      else
      {
        FL_test = 0;
        IO2SET_bit.P2_16 = 1;
      //IO0CLR_bit.P0_14 = 1;
      }
      //=======================================

    SoftTimerCT_save = T0TC;
    if( SoftTimerCT_save > SoftTimerCT_save1 )
      SoftTimerCT = (SoftTimerCT_save - SoftTimerCT_save1) + SoftTimerCT;
    else
      SoftTimerCT = (T0MR0 - SoftTimerCT_save1 + SoftTimerCT_save) + SoftTimerCT;
    SoftTimerCT_save1 = SoftTimerCT_save;
    if( SoftTimerCT >= 1843200) //~100ms (12.08.2009)
    {

      __disable_interrupt();
      WDFEED = 0xAA;
      WDFEED = 0x55;
      __enable_interrupt();
      ++SoftTimerVal;
      SoftTimerCT = 0;
      if( SoftTimerVal > 5 )
      {
        if( ByteCOMMAND )
        {
          ByteCOMMAND = 0;
          FLAG.UART_RS = 0;
        }
        SoftTimerVal = 0;
      }
      //============ ���� (11.10.2011)========
      //IO0SET_bit.P0_14 = 1;
      //ZAPFACTOR(KODADC,2);
      //IO0CLR_bit.P0_14 = 1;
      //=======================================
    }

    if (FLAG.temperIZM)
    {//��������� ��� � 2 ���
      STOPEINT0();            //�������������� ������� ��� � 2 ��� 14.11.07
      //ZAPFACTOR(MAXPOROG,1);
      //ZAPFACTOR(MINPOROG,0);
      FLAG.temperIZM = 0;
      TEMPERATURA();
      MIDTEMPER();  //��� ������� ������
      SETKOEFF(); //���� ��� ����� ����� FLAG.temperCORR
      //--12.01.2010 ���� ������� ��������
     /*
      if ( FL_test > 10 )
      {
        FL_test = 0;
        KODADC_temp = 0;
      }
      else
      {
        ++FL_test;
        KODADC_temp = KODADC;

      }
      ZAPFACTOR(KODADC,2);
      */
      //----------------------------
    } //9.07.2008 ��� �������� ����
    if ((FLAG.UART_RS) && (ByteCOMMAND >= 4))
    {// �������� �������   - minimum 4 byte
      i= 0x08;
      if (COMMAND[1] > 0x08)
                       i=0x09;

      // ���������� �� ��������� ������
      switch (COMMAND[1]){
        case 0x03:  AdrCOMMAND = *COM3; break;
        case 0x04:  AdrCOMMAND = *COM4; break;
        case 0x05:  AdrCOMMAND = *COM5; break;
        case 0x06:  /*if ((COMMAND[2]==0) && (COMMAND[3]==0))
                          i=10;*/
                    AdrCOMMAND = *COM6; break;
        case 0x07:  AdrCOMMAND = *COM7;
                    i=4; break;
        case 0x08:  AdrCOMMAND = *COM8; break;

        case 0x0B:  AdrCOMMAND = *COMB; break;
        case 0x10:  AdrCOMMAND = *COM10;
                     i=COMMAND[2]+5; break;
        case 0x11:  AdrCOMMAND = *COM11;
                     i=4; break;
        case 0x12:  AdrCOMMAND = *COM12; break;
        case 0x13:  AdrCOMMAND = *COM13; /* �������� ������� ��� ���������*/
                    i= 7; ;break;
        case 0x15:  AdrCOMMAND = *COM15;
                    if (ByteCurrentIzm == 0)
                              i = TimeByte + 9;// ������ ������ � ��������
                    break;/*��� ������ � ������������ ������*/
        case 0x16:  AdrCOMMAND = *COM16;
                    i=4; break;
        case 0x17:  AdrCOMMAND = *COM17;
                    i=4; break;
        //------------ 27.11.2015  ��� ������������ WDT---------------------
        case 0x20:  AdrCOMMAND = *COM20;
                    i=4; break;
         //------------ 27.11.2015  ��� ������������ WDT---------------------
        default:   i=0;
                    COMerr(0x01);
                    break;    //��� i == 0 -������ -������������ �������

        }//switch
        if ( (i!=0) && (ByteCOMMAND >= i))
        {
        if ((COMMAND[1]!=0x13) && (COMMAND[1] !=0x15) && (COMMAND[1] !=0x016))
              {// ��������� CRC, ���� �� ��������� �� ��������
              CRC16(COMMAND,i-2,0xFF,0);
              if( (CRClo == COMMAND[i-2]) && (CRChi == COMMAND[i-1]) )
                    (*AdrCOMMAND)(); //����� �� ������ �� �������
              ClearCOMMAND ();  //�������� ������ �������
              }//(COMANND[1]!=0x13) && (COMMAND[1] !=0x15)
          else  //(COMANND[1]=0x13) ��� (COMMAND[1] =0x15 (COMMAND[1] !=0x016))
          {
            if (COMMAND[1] == 0x15)
              if (((COMMAND[5] << 8) | COMMAND[6]) == 0)
                adrcom15 = ByteCurrentIzm+TimeByte; /*��� �������� ���-�� ���� �������*/
              else
                adrcom15 = ByteCurrentIzm;

            if ( (( ByteCurrentIzm >= COMMAND[6]+2) && (COMMAND[1]==0x13)) || //��������� ������
                 (( adrcom15 == COMMAND[8]+2)       && (COMMAND[1]==0x15)) || //������ ��� �������
                 (( ByteCurrentIzm >= COMMAND[2]+2) && (COMMAND[1]==0x16))  ) //���� ������ ���������
            {
              (*AdrCOMMAND)(); //����� �� ������ �� ������� ����� ��������� ��� ������
              ClearCOMMAND ();  //�������� ������ �������
              }
          }
        }//if i!=0...
    }//if FLAG..., Byte...
  }//while(1)

 // return 0;
}
