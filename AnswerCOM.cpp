/******************************************************/
/*             �� ���������  ���������� ������       */
/****************************************************/
#include <NXP/iolpc2294.h>
#include <intrinsics.h>
#include "my_macros.h"

extern unsigned char COMMAND[16];
extern unsigned int ByteCOMMAND;
//extern unsigned char SPECTR[3075],BUFFSPECTR[3075];


extern unsigned char BUFFSPECTR[3075];
extern unsigned long SPECTR[1025];

extern unsigned char MasEEPROMKoef[17];
extern unsigned char LiveTime[4];
extern __BITFLAG FLAG;   //��� � macros
extern unsigned char FLAGOver3byte;
extern unsigned char lock;
extern unsigned char MasInt[6];

extern int Temp_i;

extern unsigned char Test_WDT_cnt;
extern unsigned char WDMOD_temp;

#define INTERENT0   0x4000 /* bit 14h int*/
unsigned char CurrentDIzm [226]; //������� ������ ��������� 223 + CRC 2 �����
int ByteCurrentIzm;
unsigned char flagTIME2;  //���� ������ ������� �� 2 �����
unsigned char TimeByte;    //���-�� ���� � ������������� ������� 4 ��� 2

unsigned char COMMAND_temp[8];
/////////////////
void STOPEINT0(void);
void STARTEINT0(void);
void I2CTABLTempFlagW(void);

/***********************************************************************/
extern void ClearSPECTR (void);
extern void ClearCOMMAND (void);
extern void InitialiseTimer1(void); //������ ������� ������ �������
extern void InitialiseUART0(unsigned char speed);
extern void ZAPFACTOR (unsigned short factor,unsigned char nf);
extern unsigned  short I2CKOEFF(int IndMas);
extern void I2CTABLTempW(void);
extern void I2CTABLTempR(void);
extern void DeInitialiseI2C(void);
extern void I2CMasKoefW (void);
extern void I2CMasKoefR (void);
extern unsigned int TIMEIZMsek;//�������� ����� ���������
extern unsigned short KODADC;   //����������� �������� �������
extern unsigned short KODADCEEPROM;   //����������� ��������  � EEPROM
extern unsigned short MINPOROG; //������ �����
extern unsigned short MAXPOROG; //������� �����
extern float TEMPERFLOATtek;
extern float TEMPERFLOAT; //�������
extern unsigned char TABLTemperCORR[126]; //������� ������ ��������� 61 ���� �� 2 ����� +CRC 2�+ 1� ����
extern void I2CIntevarTemperW(void);
extern void ReadEEPROMIntervalTemper(void);
extern float TIMER0_INTERVAL;     //120000.0�������� � msek // ������ 2 ������  - ��������� �����������
extern void InitialiseTimer0(void);
extern void SETKOEFF(void);
extern void RESETTMR0(void);
extern unsigned int T1TC_stop;
extern unsigned char FL_test;
extern unsigned int T1TC_temp;

/**********************************************************************/
/* Table of CRC values for high-order byte */
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

/* Table of CRC values for low-order byte */
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;

void COM3(void);
void COM4(void);
void COM5(void);
void COM6(void);
void COM7(void);
void COM8(void);
void COMB(void);
void COM11(void);
void COM12(void);
void COM13(void);
void COM15(void);
void COMerr(char kod);
unsigned short CRChi,CRClo;
void COM20(void);
/****************************************************************************/
struct
      {unsigned char FractionPart;      //��. ����� - ������� ������
       unsigned char IntegerPart;      //�� ����� - ����� ����� ��������
      }TEMPERATURE;
/****************************************************************************/
/****** ������� ����������� ����� ************/
void CRC16(unsigned char mas[],short usDataLen,char flFF,char flzapCOMMand)
//(mas,usDataLen,flFF,flzapCOMMand) // �� ������ ����� ��������� -extern
//unsigned char *mas;
//unsigned short usDataLen; /*���-�� ����*/
//unsigned char flFF,flzapCOMMand; /* ���� ������ crc, ���� ������ crc � ��� COMMAND*/
{
  unsigned uIndex,kolData;

  kolData = usDataLen;  //���������� ���� ���
  if (flFF)
  {  CRChi = CRClo = 0xFF;
      } //if flFF

  while (usDataLen--)
  {
    uIndex = CRChi ^ *mas++;
    CRChi = CRClo ^ auchCRCHi[uIndex];
    CRClo = auchCRCLo[uIndex];
  }// while
  if (flzapCOMMand)
  {  COMMAND[kolData]  = CRClo;
     kolData++;
     COMMAND[kolData] = CRChi;
  } //if (flzapCOMMand)
}//CRC16
/***���������� ������ �� imas � rmas ��� ������**/
//void MOVSPECTR(imas,rmas)
//unsigned char imas[3074],rmas[3074];
//{
//  int i;
// for (i=3071;i >= 0; i-=3)
// {
// rmas[i]   = imas[i];
// rmas[i-1] = imas[i-1];
// rmas[i-2] = imas[i-2];
// }
//}//TURNSPECTR end
void MOVSPECTR_B_S(void)
//unsigned char imas[3074],rmas[3074];
{
  int i;
  for (i = 0; i < 1024; ++i)   //������ ��� ������� � �������� ���
  {
    //BUFFSPECTR[i] = SPECTR[i];
    BUFFSPECTR[i*3 + 2] = *((char*)&SPECTR[i] + 2);
    BUFFSPECTR[i*3 + 1] = *((char*)&SPECTR[i] + 1);
    BUFFSPECTR[i*3 + 0] = *((char*)&SPECTR[i] + 0);
  }


}
void MOVSPECTR_S_B(void)
//unsigned char imas[3074],rmas[3074];
{
  int i;
  for (i = 0; i < 1024; ++i)   //������ ��� ������� � �������� ���
  {
    //BUFFSPECTR[i] = SPECTR[i];
    *((char*)&SPECTR[i] + 2) = BUFFSPECTR[i*3 + 2];
    *((char*)&SPECTR[i] + 1) = BUFFSPECTR[i*3 + 1];
    *((char*)&SPECTR[i] + 0) = BUFFSPECTR[i*3 + 0];
  }


}
/****************************************************************************/

/***���������� ������ �� imas C ���������������� �������� � rmas ��� ������**/
void TURNSPECTR(imas,rmas)
unsigned char imas[3074],rmas[3074];
{
  int i;
 for (i=3071;i >= 0; i-=3)
 {
 rmas[i]   = imas[i-2];
 rmas[i-1] = imas[i-1];
 rmas[i-2] = imas[i];
 }
}//TURNSPECTR end

void TURNSPECTR_B_S(void)
//unsigned char imas[3074],rmas[3074];
{
  int i;
  for (i = 0; i < 1024; ++i)   //������ ��� ������� � �������� ���
  {
    //BUFFSPECTR[i] = SPECTR[i];
    BUFFSPECTR[i*3 + 0] = *((char*)&SPECTR[i] + 2);
    BUFFSPECTR[i*3 + 1] = *((char*)&SPECTR[i] + 1);
    BUFFSPECTR[i*3 + 2] = *((char*)&SPECTR[i] + 0);
  }


}

void TURNSPECTR_S_B(void)
//unsigned char imas[3074],rmas[3074];
{
  int i;
  for (i = 0; i < 1024; ++i)   //������ ��� ������� � �������� ���
  {
    //BUFFSPECTR[i] = SPECTR[i];
    *((char*)&SPECTR[i] + 0) = BUFFSPECTR[i*3 + 2];
    *((char*)&SPECTR[i] + 1) = BUFFSPECTR[i*3 + 1];
    *((char*)&SPECTR[i] + 2) = BUFFSPECTR[i*3 + 0];
  }


}

/****************************************************************************/
/********* ��������� ����������� *********/
void TakingTemperature(void)
{
  float ttt;
  if (FLAG.temperCORR)
    ttt = TEMPERFLOAT;    //���� ����� ������ ��������� �������� �������
  else
        ttt = TEMPERFLOATtek;

  if (ttt > 0)
  {
  TEMPERATURE.IntegerPart = (char) ttt;
  TEMPERATURE.FractionPart = (char) (ttt*10.0) - (TEMPERATURE.IntegerPart*10.0);
  }
  else
  {//������������� ����������� �� ������ ������
  TEMPERATURE.IntegerPart = (char) (- ttt);
  TEMPERATURE.FractionPart = (char) (-ttt*10.0) - (TEMPERATURE.IntegerPart*10.0);
  TEMPERATURE.IntegerPart = 0xFF - (char) (ttt);
  }
}//TakingTemperature END
/****************************************************************************/
/****** ������ ������ � UART0 ********/
void BYTEtoUART(unsigned char Charbyte)
{
    while (!(U0LSR & 0x020));
    U0THR= Charbyte;
}//BYTEtoUART end
/****** ������ ������ � UART0 ********/
void COMMANDtoUART(unsigned int kolbyte)
{
 int i;
// STOPEINT0();
 for (i = -1; ++i < kolbyte;)
     BYTEtoUART(COMMAND[i]);
// STARTEINT0();
}//COMMANDtoUART


//----------11.08.2009----------------------

void COMMANDtoUARTnew(unsigned int kolbyte)
{
 int i;
 float Temp,Temp1;
 Temp1 = 10;
 Temp = 2;
// STOPEINT0();
 for (i = -1; ++i < kolbyte;)
 {
   Temp1 = Temp1 * Temp;
     //BYTEtoUART(COMMAND[i]);
   while (!(U0LSR & 0x020));
    U0THR= COMMAND[i];
 }
// STARTEINT0();
}//COMMANDtoUART


/****************************************************************************/
/****** ��������� 2-� ���� � COMMAND ********/
void toCOMMAND2byte(unsigned short nombyte,unsigned short datavalue)
{
  COMMAND[nombyte] = datavalue >> 8;
  nombyte++;
  COMMAND[nombyte] = datavalue;
}//toCOMMAND2byte
/****************************************************************************/
/****** ��������� 2-� ���� �� COMMAND � short ********/
short toShort2COMMAND(nombyte)
unsigned short nombyte;
{
return (COMMAND[nombyte] << 8 | COMMAND[nombyte+1]);
}//toCOMMAND2byte
/****************************************************************************/
/**** ����� �� ������� � ������� *****/
void COMerr02(void)
{
COMerr(0x02); //������������ ����� ������
FLAG.error = 1;
}
/********/
void COMerr( char kod)
//unsigned char kod;
{

  COMMAND[1] = COMMAND[1] | 0x80;
  COMMAND[2] = kod;

  CRC16(COMMAND,3,0xFF,1);

  COMMANDtoUART(5);

  ClearCOMMAND ();  //�������� ������ �������

}
/****************************************************************************/
/********** STOP EINT0 **********/
void STOPEINT0(void)
{
            if (FLAG.SPECTR)
                            { while (EXTINT & 1)
                                    EXTINT = 0x1;             //����� ���������� EINT0
                            T1TCR = 0;         //���������� ������� TMR1 - ������� �����
                            VICIntEnClear = VICIntEnClear | INTERENT0;  //������ ���������
                            }
}//STOPEINT0
/****************************************************************************/
/********** START EINT0 **********/
void STARTEINT0(void)
{
            if (FLAG.SPECTR)
            { VICIntEnable =VICIntEnable | INTERENT0; //���������� ���������
              T1TCR = 1;         //��������� ������� TMR1
             }
}//STARTEINT0
/****************************************************************************/
/************COM3  ������� ��������� ��������� ���������� **********/
void COM3(void)
{ short adr,kol;
  float InterTemp;
      adr = COMMAND[2] << 8 | COMMAND[3]; //����� � ��������� ������� �� 0...4
      kol = COMMAND[4] << 8 | COMMAND[5]; //���-�� ��� �������
       //�������� ����� ��������� ������� 4-�����(2 ��������) ��������� �� 2 �����(1 �������)
       //�������� ���-�� ��������� �������
      if (((adr == 0) && ((kol == 2) || (kol == 1))) || ((adr >0) && (kol == 1)) || ((adr==5) && (kol == 2)))
          {//������ ������ �������� �������, ������������ ������
            COMMAND[2] = kol << 1; //���-�� ���� ������ ������ = ���-�� ��� * 2
            FLAG.error = 0;  //���� ��������� ������

            switch (adr){
            case 0x00: if (flagTIME2 == 0)
                      {
                       COMMAND[3] = TIMEIZMsek >> 24;
                       COMMAND[4] = TIMEIZMsek >> 16;  //� �������� 2 �������
                       if (kol == 2)
                           {
                            toCOMMAND2byte(5,TIMEIZMsek);  //�������� ����� ���������� ������� 4�����
                            TimeByte= 4;
                            }
                          else
                          {
                            flagTIME2 = 1; //������ �� 2 �����
                            TimeByte= 2;
                          }
                      }
                      else
                      {
                       toCOMMAND2byte(3,TIMEIZMsek);  //�������� ����� ���������� ������� 2 �������
                       flagTIME2 = 0;
                      }
                       break;
            case 0x02:  toCOMMAND2byte(3,KODADC);     //����������� ��������
                        break;
            case 0x03:  toCOMMAND2byte(3,MINPOROG);   //������ �����
                        break;
            case 0x04:  toCOMMAND2byte(3,MAXPOROG);   //������� �����
                        break;
            case 0x05:



                        COMMAND[2] = 4;               //�������� ��������� ����������� ����.
                        InterTemp = TIMER0_INTERVAL / 60.0 / 1000.0;
                        COMMAND[6] =  *(unsigned char*)  &  InterTemp;
                        COMMAND[5] =  *((unsigned char*) &  InterTemp+1);
                        COMMAND[4] =  *((unsigned char*) &  InterTemp+2);
                        COMMAND[3] =  *((unsigned char*) &  InterTemp+3);


                        break;
            default:    COMerr02();
                        break; //��� ���>5 ��� =1 - ������������ ����� ������
            }//swith adr
          if (!FLAG.error)
              {// �����
                  kol = COMMAND[2]+3;
                  CRC16(COMMAND,kol,0xFF,1);
                  kol+=2;
                  COMMANDtoUART(kol);
              }//!FLAG.error
          }//if adr | kol
                    else //if adr | kol
                    COMerr(0x03); //������ ���-�� ��������� - ������������ �������� ������
}//END COM3
/****************************************************************************/
/************* COM4 ������� ��������� ��������� ������ **************/
void COM4(void)
{ short adr,kol;
  int LivingTime;
      adr = COMMAND[2] << 8 | COMMAND[3]; //����� � ��������� ������� �� 0...4
      kol = COMMAND[4] << 8 | COMMAND[5]; //���-�� ��� �������
       //����� ����� 4-�����(2 ��������) ��������� �� 2 �����(1 �������)
       //�������� ���-�� ��������� �������
      if (((adr == 0) && ((kol == 2)|| (kol == 1)) || ((adr ==2) && (kol == 1))) || ((adr == 3) && (kol == 2)))
          {//������ ������ �������� �������, ������������ ������
            COMMAND[2] = kol << 1; //���-�� ���� ������ ������ = ���-�� ��� * 2
            FLAG.error = 0;  //���� ��������� ������
            //LivingTime = T1TC;  //����� ����� - mSek
            //---25.04.2011-----------

            if( FLAG.SPECTR )
            {
              LivingTime = T1TC;  //����� ����� - mSek

            }
            else
            {
              LivingTime = T1TC_stop;
            }

            //-----------------------------------------

            switch (adr){
            case 0x00:  LivingTime/=1000; //����� ������
            case 0x03:
                        if (flagTIME2 == 0)
                        {
                        COMMAND[3] = LivingTime >> 24;
                        COMMAND[4] = LivingTime >> 16;  //1-� �� ��� �����
                        if (kol == 2)
                        {
                          toCOMMAND2byte(5,LivingTime);   //� ������ -  mSek 4 �����
                          TimeByte =4;
                        }
                          else
                          {
                            flagTIME2 = 1; //������ �� 2 �����
                            TimeByte =2;
                          }
                      }
                      else
                      {
                       toCOMMAND2byte(3,LivingTime);  //�������� ����� ���������� ������� 2 �������
                       flagTIME2 = 0;
                      }
                        break;
            case 0x02:  TakingTemperature();            //������� �������� �����������
                        COMMAND[3] = TEMPERATURE.IntegerPart;
                        COMMAND[4] = TEMPERATURE.FractionPart;


                        break;
            default:    COMerr02();
                        break; //��� ���>3 ��� =1 - ������������ ����� ������
            }//swith adr
          if (!FLAG.error)
              {// �����
                  kol = COMMAND[2]+3;
                  CRC16(COMMAND,kol,0xFF,1);
                  kol+=2;
                  COMMANDtoUART(kol);
                  //======== ��� ������������ WDT (14.12.2015) ==============

                        ++Test_WDT_cnt;
                        if( Test_WDT_cnt > 10)
                        {
                           __disable_interrupt();
                          while(1)
                          {
                          }
                        }

                        //======== ��� ������������ WDT (14.12.2015) ==============
              }//!FLAG.error
           }//if adr | kol
                    else //if adr | kol
                    COMerr(0x03); //������ ���-�� ��������� - ������������ �������� ������
}//END COM4
/****************************************************************************/
/******* COM5  ������ ����������� ������ **************/
void COM5(void)
{ int i;
  if ( (COMMAND[2] == 0) && (COMMAND[5]==0) && ((COMMAND[4] == 0xFF) || (COMMAND[4] == 0)))
    {//������ ������ �������� �������, ����� - ���������� �������
     STOPEINT0();    //���������� ���������
     switch (COMMAND[3]){
            case 0x00:
                        T1TC_temp = 0;
                        ClearSPECTR();  //�������� ������ ���������� �������
                        InitialiseTimer1(); // � ������ ������ �������
                        break;
            case 0x01:  if (COMMAND[4]==0x0)
                            {
                              STOPEINT0();
                              FLAG.SPECTR = 0; //��������� ���������� �������
                             }
                         else
                            {
                              //--������������ ������� 14.10.2011
                              //if( FL_test == 1 )
                                //-------------------------------
                                FLAG.SPECTR = 1;    //��������� ���������� �������
                              STARTEINT0();
                              //--������������ ������� 14.10.2011
                              //if( FL_test == 0 )
                                // FLAG.SPECTR = 0;    //��������� ���������� �������
                                //-------------------------------

                            }
                        break;
            case 0x02:  //for (i=1024; --i>=0;)   //������ ��� ������� � �������� ���
                        //      BUFFSPECTR[i] = SPECTR[i];
                        for ( i=1024; --i>=0; )   //������ ��� ������� � �������� ���
                            {
                              //BUFFSPECTR[i] = SPECTR[i];
                              BUFFSPECTR[i*4] = *((char*)&SPECTR[i] + 3);
                              BUFFSPECTR[i*4 + 1] = *((char*)&SPECTR[i] + 2);
                              BUFFSPECTR[i*4 + 2] = *((char*)&SPECTR[i] + 1);
                            }
                        break;
            case 0x05:
                        FLAG.temperCORR = !FLAG.temperCORR; //�������� - ��������� ������������� ���������
                        I2CTABLTempFlagW(); //������ c ����� ������� � EEPROM
                        break;
            case 0x07:  FLAG.tuning = !FLAG.tuning; //�������� - ��������� ������������� ���������
                        break;
            default:    COMerr02();
                        break; //��� ���>2  - ������������ ����� ������
            }//swith adr
     STARTEINT0(); //��������� ���������
     if (!FLAG.error)
             // �����
              COMMANDtoUART(8);
     }//if COMMAND
     else // COMMAND
               COMerr(0x03); //������  - ������������ �������� ������
}//end COM5
/****************************************************************************/
/****** COM6 �������� ��������� ��������� ���������� *******/
void COM6(void)
{
short adr,kol;
  adr = COMMAND[2]<<8 | COMMAND[3];
  //�������� ����� ��������� ������� 4-�����(2 ��������) ��������� �� 2 �����(1 �������)
  //�������� ���-�� ��������� �������
  //������ ������ �������� �������, ������������ ������
   FLAG.error = 0;  //���� ��������� ������
   kol = 8;
   STOPEINT0();  //���������� ����������, ���� ���� ����������
   switch (adr){
            case 0x00:
                        if (TimeByte == 4)  //ATAS
                          TIMEIZMsek =COMMAND[4]<<24 | COMMAND[5]<<16 ;  //������ ����� ���������� ������� �� �����
                        else
                          if (flagTIME2 == 0) //MOODE
                          {
                          TIMEIZMsek =COMMAND[4]<<24 | COMMAND[5]<<16 ;  //������ ����� ���������� ������� �� �����
                          flagTIME2 = 1;
                          break;
                          }
                          flagTIME2 = 0;
            case 0x01:  TIMEIZMsek |=  COMMAND[4]<<8 | COMMAND[5];  //������ ����� ���������� ������� �� �����
                        STOPEINT0();
                        T1MR0 = TIMEIZMsek*1000; //  = msek ������
                        STARTEINT0();
                        break;
            case 0x02:  KODADCEEPROM = I2CKOEFF(0); //� ���������������� I2C
                        if (FLAG.temperCORR)
                        {
                            Temp_i=0;       //11.07.2008
                            SETKOEFF(); //���� ��� ����� ����� FLAG.temperCORR
                        }
                        else
                        {
                         KODADC = KODADCEEPROM;
                         ZAPFACTOR(KODADCEEPROM,2); //����������� ��������
                        }
                        break;
            case 0x03:  MINPOROG = I2CKOEFF(2);
                        ZAPFACTOR(MINPOROG,0);   //������ �����
                        break;
            case 0x04:  MAXPOROG = I2CKOEFF(4);
                        ZAPFACTOR(MAXPOROG,1);  //������� �����
                        ZAPFACTOR(MINPOROG,0);  //����� ��������� ��������� LDA ���
                        break;
            case 0x05:  MasInt[3] = COMMAND[4]; //�� �����
                        MasInt[2] = COMMAND[5];
                        break;
            case 0x06:  MasInt[1] = COMMAND[4]; //�� �����
                        MasInt[0] = COMMAND[5];
                        I2CIntevarTemperW();
                        ReadEEPROMIntervalTemper();
                        //DeInitialiseI2C();
                        InitialiseTimer0();
                        RESETTMR0();
                        break;
            default:    COMerr02();
                        break; //��� ���>5 ��� =1 - ������������ ����� ������
            }//swith adr
    if (!FLAG.error)
              {// �����
                CRC16(COMMAND,6,0xFF,1);
                COMMANDtoUART(kol);
              }//!FLAG.error
    STARTEINT0(); //��������� ����������,  ���� ���� ����������
}// END COM6
/****************************************************************************/
void COM7(void)
{
   COMMAND[2] = 0;
    if (FLAG.SPECTR)
         COMMAND[2] = 2; //���� ���������� �������
    if (FLAG.temperCORR)
         COMMAND[2] |= 4; //������������� ��������� ��������
    if ( FLAG.testerror )
         COMMAND[2] |= 8;//������������ ������ �������
    if (FLAG.tuning)
         COMMAND[2] |= 0x80; //����� ���������
    CRC16(COMMAND,3,0xFF,1);
    COMMANDtoUART(5);
}// END COM7
/****************************************************************************/
/******* ����������� ********/
void COM8(void)
{
unsigned short adr;
 int icikl; //��� ��� ��������

 adr = COMMAND[2]<<8 | COMMAND[3];
  switch (adr){
            case 0x00:  // ������� ������ �������
                        COMMANDtoUART(8); break;
            case 0x01: //������������� ������� ������������
                        if ((COMMAND[4] == 0) && ((COMMAND[5] ==1) || (COMMAND[5] == 2)))
                        {
                          COMMANDtoUART(8); //��������
                          DELAYTACKT(icikl,100);         //�������� �� 100 ������-5,4uSek
                          InitialiseUART0(COMMAND[5]);  //���������������� UART0 - ����� ����� ������������� ��� �������� ������
                        }
                        else
                          COMerr(0x03); //������  - ������������ �������� ������ -��� �������� �� 1 ��� 2
                        break;
            case 0x02: // ������� ���������� �������� �����������
                        COMMAND[4]=0;
                        COMMAND[5]=0;
                        if (FLAG.noEINT0)
                                COMMAND[5] = COMMAND[5] | 0x02; //��� ���� �� ��������� � ������� 2-� �����
                        if ((FLAG.EEPROMAW) || (FLAG.EEPROMAR) || (FLAG.EEPROMDW) || (FLAG.EEPROMDR))
                                COMMAND[5] = COMMAND[5] | 0x04; //������ ������ ������ EEPROM
                        if (FLAGOver3byte == 1)
                                COMMAND[5] = COMMAND[5] | 0x10; //������������ 3-� ���� ��������� �������
                        if (FLAG.ErrtempTABL)
                                COMMAND[5] = COMMAND[5] | 0x40; //������ CRC ���� ���� ����
                        if (FLAG.ErrKoef)
                                COMMAND[5] = COMMAND[5] | 0x80; //������ CRC �������������
                        if (FLAG.minmaxTemper)
                                COMMAND[5] = COMMAND[4] | 0x04; //����������� ��� ��������� +9.5...+50.5
                        if (FLAG.temperature)
                                COMMAND[5] = COMMAND[4] | 0x08; // ������ ����������� �� ��������
                        CRC16(COMMAND,6,0xFF,1);
                        COMMANDtoUART(8); //��������
                        break;
            //---- 10.08.2009 ----------------------
                case 0x81: // ������� ���������� �������� �����������
                        COMMAND[4]=0;
                        COMMAND[5]=0;
                        if (FLAG.noEINT0)
                                COMMAND[5] = COMMAND[5] | 0x02; //��� ���� �� ��������� � ������� 2-� �����
                        if ((FLAG.EEPROMAW) || (FLAG.EEPROMAR) || (FLAG.EEPROMDW) || (FLAG.EEPROMDR))
                                COMMAND[5] = COMMAND[5] | 0x04; //������ ������ ������ EEPROM
                        if (FLAGOver3byte == 1)
                                COMMAND[5] = COMMAND[5] | 0x10; //������������ 3-� ���� ��������� �������
                        if (FLAG.ErrtempTABL)
                                COMMAND[5] = COMMAND[5] | 0x40; //������ CRC ���� ���� ����
                        if (FLAG.ErrKoef)
                                COMMAND[5] = COMMAND[5] | 0x80; //������ CRC �������������
                        if (FLAG.minmaxTemper)
                                COMMAND[5] = COMMAND[4] | 0x04; //����������� ��� ��������� +9.5...+50.5
                        if (FLAG.temperature)
                                COMMAND[5] = COMMAND[4] | 0x08; // ������ ����������� �� ��������
                        CRC16(COMMAND,6,0xFF,1);
                        COMMANDtoUART(8); //��������
                        break;

            default:    COMerr02();
                        break; //��� ���>2  - ������������ ����� ������
          }//swith adr
}// END COM8
/****************************************************************************/
/******* ������ ������� �������������� ������� ********/
void COMB(void)
{
  unsigned int adr,kol;
  int i;
  adr = (COMMAND[3] << 8) | COMMAND[4];
  kol = (COMMAND[5] << 8) | COMMAND[6];
  if (adr > 0xC00)
            COMerr02(); //������ ������
  else
    if ((adr+kol) > 3072 || (COMMAND[5] !=0))
            COMerr(0x03); //������ ������
         else
         {
           if (adr == 0)
           { if (TimeByte == 2)
                 //TURNSPECTR(SPECTR,BUFFSPECTR);  //��������������
                  TURNSPECTR_B_S();
                 else
                   //MOVSPECTR(SPECTR,BUFFSPECTR);  //���������� ������� ������ � ������
                   MOVSPECTR_B_S();
           }
           COMMAND[2] = COMMAND[6]; //���-��
           CRC16(COMMAND,3,0xFF,0);
           CRC16(BUFFSPECTR+adr,COMMAND[2],0,0);
           COMMANDtoUART(3);        //������ �������

           for (i=adr; i<(adr+kol); i++) //������
                  BYTEtoUART(BUFFSPECTR[i]);

           BYTEtoUART(CRClo);   //crc
           BYTEtoUART(CRChi);
         }

}// END COMB
/****************************************************************************/
/******* �������� ����������������� ��� ********/
void COM10(void)
{

  switch (COMMAND[4])
  {
           case 0x00:  // ����������������� ��� �������
                         MasEEPROMKoef[12]  = COMMAND[8];     //1���� - ����������������� ���
                         COMMAND_temp[0] = COMMAND[8];
                         break;
           case 0x01: // ����� ������
                         MasEEPROMKoef[14]  = COMMAND[8];  //������ �� ����
                         COMMAND_temp[1] =  COMMAND[8];
                         COMMAND_temp[2] =  COMMAND[7];
                         break;
           case 0x02: // ��������� �����
                         MasEEPROMKoef[9]  = COMMAND[7];    //�� ����
                         MasEEPROMKoef[8]  = COMMAND[8];    //�� ����
                         COMMAND_temp[3] =  COMMAND[8];
                         COMMAND_temp[4] =  COMMAND[7];
                         break;
           case 0x03: // ���� ������������
                         MasEEPROMKoef[11]  = COMMAND[7];    //���
                         MasEEPROMKoef[10]  = COMMAND[8];    //������
                         COMMAND_temp[5] =  COMMAND[8];
                         COMMAND_temp[6] =  COMMAND[7];
                         break;
           case 0x04: // ������������ ����� �������
                         MasEEPROMKoef[13]  = COMMAND[8];
                         break;
            default:    COMerr02();
                        break; //��� ���>2  - ������������ ����� ������
          }//swith
   //COMMAND[3] = 2;
   CRC16(COMMAND,6,0xFF,1);
   COMMANDtoUART(8);
  //COMMANDtoUARTnew(11);
  //COMMANDtoUARTnew(11);
  if (COMMAND[4] == 4)
  {

    //------------- 14.08.2009 ----------------------
    if( COMMAND_temp[0] != 1)
    {
      // ����� ������
      MasEEPROMKoef[14] = COMMAND_temp[3];
      // ��������� �����
      MasEEPROMKoef[9]  = COMMAND_temp[6];    //�� ����
      MasEEPROMKoef[8]  = COMMAND_temp[5];
      // ���� ������������
      MasEEPROMKoef[11] = COMMAND_temp[2];    //���
      MasEEPROMKoef[10] = COMMAND_temp[1];
    }
    MasEEPROMKoef[12] = 1;
    I2CMasKoefW ();
   //DeInitialiseI2C();
  }
}// END COM10
/****************************************************************************/
/******* ������ ����������������� ��� ********/
void COM11(void)
{
    I2CMasKoefR ();
    //DeInitialiseI2C();
    COMMAND[2] = 0x0A;  // 10-���- ����
    COMMAND[3] = MasEEPROMKoef[12];     //1���� - ����������������� ���
    COMMAND[4] = MasEEPROMKoef[14];     //����� ������
    COMMAND[5] = MasEEPROMKoef[9];  //c� ���� ��������� �����
    COMMAND[6] = MasEEPROMKoef[8];
    COMMAND[7] = MasEEPROMKoef[11];  // ���
    COMMAND[8] = MasEEPROMKoef[10];  // ������ ������������
    if (MasEEPROMKoef[6] == 2) //���� ������ 1024!!!!!
    {
      toCOMMAND2byte(9,1024); // ���-�� �������
      toCOMMAND2byte(11,1024*3); // ���-�� ���� �������
    }
    CRC16(COMMAND,13,0xFF,1);
    COMMANDtoUART(15);
}// END COM11
/****************************************************************************/
/******* ������ ������� ������ ��������� ********/
void COM12(void)
{
  unsigned int adr;
  int i;
/* ��� ���� �������   ��� ���� ��� �� ����� ��� ���� ��� �� �����*/
if ((COMMAND[2] != 4) || (COMMAND[5]!=0) || (COMMAND[6] > 223))
              COMerr(0x03); //������  - ������������ �������� ������
    else
    {
      if ((COMMAND[3] != 0) || (COMMAND[4] >223))
                              COMerr(0x02); //������  - ������������ ����� ������
       else /*������ ������*/
       {
         COMMAND[2] = COMMAND[6]; //���-�� ����
         adr = COMMAND[4];    //����� 1-�� �����
         CRC16(COMMAND,3,0xFF,0);
         COMMANDtoUART(3);
         CRC16(CurrentDIzm+adr,COMMAND[2],0,0);

         for (i = adr-1; ++i < (adr+COMMAND[2]);)
                 BYTEtoUART(CurrentDIzm[i]);/*���������� ������ � CRC*/
         BYTEtoUART(CRClo);   //crc
         BYTEtoUART(CRChi);

        }/*������ ������*/
    } /*else*/

}// END COM12
/****************************************************************************/
/******* �������� ������� ������ ��������� ********/
void COM13(void)
{
int i;
/* ��� ���� ��� �� ����� ��� ���� ��� �� �����*/
if ( (COMMAND[5]!=0) || (COMMAND[6] > 223))
              COMerr(0x03); //������  - ������������ �������� ������

  else
    {
      if ((COMMAND[3] != 0) || (COMMAND[4] >=223))
                              COMerr(0x02); //������  - ������������ ����� ������
       else /* �������� ����������� �����*/
       {
        CRC16(COMMAND,7,0xFF,0);
        CRC16(CurrentDIzm+COMMAND[4],COMMAND[6],0,0);
        if( (CRClo == CurrentDIzm[COMMAND[4]+ByteCurrentIzm-2]) && (CRChi == CurrentDIzm[COMMAND[4]+ByteCurrentIzm-1]) )
          {/*���������� ������� */
            COMMANDtoUART(7);
             for (i = COMMAND[4]-1; ++i < (COMMAND[4]+COMMAND[6]+2);)
                  BYTEtoUART(CurrentDIzm[i]); /*���������� ������ � CRC*/
          } /*������� ������� � ������*/

       }/*�������� ���������� �����- ���������*/
    } /*else*/
//CurrentDIzm[ByteCurrentIzm-2] = 0; //������ CRC
//CurrentDIzm[ByteCurrentIzm-1] = 0;
ByteCurrentIzm = 0;
}// END COM13
/****************************************************************************/
/******* �������� ������ � ������������ ������ ********/
void COM15(void)
{
int i;
unsigned int kol,adr,acrc;
adr = (COMMAND[5] << 8) | COMMAND[6]; /*����� �������*/
/* ��� ���� ��� �� ����� ��� ���� ��� �� �����*/
kol =(COMMAND[7] << 8) | COMMAND[8];
acrc =adr+kol-TimeByte; /*����� ����� 4(2)-�����*/
if (adr > 0xC00)
    COMerr(0x02); //������  - ������������ ����� ������
else
  if ( acrc > 3072)
            COMerr(0x03); //������  - ������������ �������� ������
  else
    {
        CRC16(COMMAND,9,0xFF,0);
        if (adr == 0)
              {/* ������ ������ � �������� */
                CRC16(LiveTime,TimeByte,0,0);
                CRC16(BUFFSPECTR,kol-TimeByte,0,0);
              }
        else
          CRC16(BUFFSPECTR+adr-TimeByte,kol,0,0);
        if( (CRClo == BUFFSPECTR[acrc]) && (CRChi == BUFFSPECTR[acrc+1]) )
          {/*���������� ������� */
            if (adr == 0)
            {
              COMMANDtoUART(TimeByte+9);// ������ ������ � ��������
              kol-=TimeByte;
            }
            else
            {
              COMMANDtoUART(9);
              adr-=TimeByte;
            }

            for (i = -1; ++i < kol+2;)
                {/*���������� ������ � CRC*/
                    BYTEtoUART(BUFFSPECTR[adr+i]);
                }//for (i = -1; ++i < kol+2;)

        if ((acrc) >= 3070)
        { /* �������� ���� ������*/
//          BUFFSPECTR[acrc] = 0; //������ CRC
//          BUFFSPECTR[acrc+1] = 0;
          /* ���������*/
          STOPEINT0();      //���������� ������� TMR1 � ��������� EINT0-�������� ����� ������
          if (TimeByte == 2) //���� ���������� ����� �� 2 ����� - ������ ��������������
                //TURNSPECTR(BUFFSPECTR,SPECTR);
                TURNSPECTR_S_B();
          else
                //MOVSPECTR(BUFFSPECTR,SPECTR);
                MOVSPECTR_S_B();
         kol = ((LiveTime[3] << 24)| (LiveTime[2] << 16)| (LiveTime[1] << 8)| LiveTime[0])*1000; /*� �����*/
         T1TC  = kol;
         T1PC  = 0;
        }//�������� ���� ������
  }// CRC ���������
  else
    LiveTime[1] = 0;
}// ��� ������
STARTEINT0(); //��������� ���������, ���� ���������
ByteCurrentIzm = 0;
}// END COM15
/****************************************************************************/
/******* �������� ������� ������������� ���������  ********/
/* CRC ��������� ��������*/
void TABLTempUART(void)
{
 int i;

 COMMANDtoUART(3);
 for (i = -1; ++i < 122;)
                  BYTEtoUART(TABLTemperCORR[i]); /* ������ */
  BYTEtoUART(CRClo);
  BYTEtoUART(CRChi);
}
/****************************************************************************/
/******* �������� ������� ������������� ��������� ********/
void COM16(void)
{

 CRC16(COMMAND,3,0xFF,0);
 CRC16(TABLTemperCORR,122,0,0);
 if( (CRClo == TABLTemperCORR[122]) && (CRChi == TABLTemperCORR[123]) )
  {//CRC ���������
   I2CTABLTempW();  //������ ����� ������� � EEPROM + CRC + flag
   //DeInitialiseI2C();
   CRC16(COMMAND,3,0xFF,0);
   CRC16(TABLTemperCORR,122,0,0);  //������� CRC
   TABLTempUART();  //���������� ������� +CRC (�� �������)
  }//CRC ���������
ByteCurrentIzm = 0;
}// end COM16
/****************************************************************************/
/******* ������ ������� ������������� ��������� ********/
void COM17(void)
{
 COMMAND[2] = 0x7A; // ���������� 61(122�����) �������� ��� �������� � ����
 CRC16(COMMAND,3,0xFF,0);
 CRC16(TABLTemperCORR,122,0,0);
 TABLTempUART();  //�������� +CRC
}// end COM17
/****************************************************************************/


 //------------ 27.11.2015  ��� ������������ WDT---------------------
/****************************************************************************/
// ��� ������� WDT �������� ��������� !!!!!!!!!
void COM20(void)
{
   COMMAND[2] = WDMOD_temp;

    CRC16(COMMAND,3,0xFF,1);
    COMMANDtoUART(5);
    //------------ 27.11.2015  ��� ������������ WDT---------------------
    while(1)
    {


    }
}// END COM20
/****************************************************************************/
