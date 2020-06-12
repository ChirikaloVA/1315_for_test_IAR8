/******************************************************/
/*             пп обработки  полученных команд       */
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
extern __BITFLAG FLAG;   //тип в macros
extern unsigned char FLAGOver3byte;
extern unsigned char lock;
extern unsigned char MasInt[6];

extern int Temp_i;

extern unsigned char Test_WDT_cnt;
extern unsigned char WDMOD_temp;

#define INTERENT0   0x4000 /* bit 14h int*/
unsigned char CurrentDIzm [226]; //текущие данные измерения 223 + CRC 2 байта
int ByteCurrentIzm;
unsigned char flagTIME2;  //флаг чтания времени по 2 байта
unsigned char TimeByte;    //кол-во байт в запрашиваемом времени 4 или 2

unsigned char COMMAND_temp[8];
/////////////////
void STOPEINT0(void);
void STARTEINT0(void);
void I2CTABLTempFlagW(void);

/***********************************************************************/
extern void ClearSPECTR (void);
extern void ClearCOMMAND (void);
extern void InitialiseTimer1(void); //чистка таймера живого времени
extern void InitialiseUART0(unsigned char speed);
extern void ZAPFACTOR (unsigned short factor,unsigned char nf);
extern unsigned  short I2CKOEFF(int IndMas);
extern void I2CTABLTempW(void);
extern void I2CTABLTempR(void);
extern void DeInitialiseI2C(void);
extern void I2CMasKoefW (void);
extern void I2CMasKoefR (void);
extern unsigned int TIMEIZMsek;//заданное время измерения
extern unsigned short KODADC;   //коэффициент усиления текущий
extern unsigned short KODADCEEPROM;   //коэффициент усиления  в EEPROM
extern unsigned short MINPOROG; //нижний порог
extern unsigned short MAXPOROG; //верхний порог
extern float TEMPERFLOATtek;
extern float TEMPERFLOAT; //средняя
extern unsigned char TABLTemperCORR[126]; //ТАБЛИЦА ТЕМПЕР КОРРЕКЦИИ 61 знач по 2 байта +CRC 2б+ 1б флаг
extern void I2CIntevarTemperW(void);
extern void ReadEEPROMIntervalTemper(void);
extern float TIMER0_INTERVAL;     //120000.0интервал в msek // каждые 2 минуты  - измерение температуры
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
      {unsigned char FractionPart;      //мл. часть - дробный градус
       unsigned char IntegerPart;      //ст часть - целая часть градусов
      }TEMPERATURE;
/****************************************************************************/
/****** подсчет контрольной суммы ************/
void CRC16(unsigned char mas[],short usDataLen,char flFF,char flzapCOMMand)
//(mas,usDataLen,flFF,flzapCOMMand) // не менять место положения -extern
//unsigned char *mas;
//unsigned short usDataLen; /*кол-во байт*/
//unsigned char flFF,flzapCOMMand; /* флаг чистки crc, флаг записи crc в мас COMMAND*/
{
  unsigned uIndex,kolData;

  kolData = usDataLen;  //количество байт для
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
/***ПЕРЕГОНЯЕМ СПЕКТР из imas в rmas при обмене**/
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
  for (i = 0; i < 1024; ++i)   //запись тек спектра в буферное ОЗУ
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
  for (i = 0; i < 1024; ++i)   //запись тек спектра в буферное ОЗУ
  {
    //BUFFSPECTR[i] = SPECTR[i];
    *((char*)&SPECTR[i] + 2) = BUFFSPECTR[i*3 + 2];
    *((char*)&SPECTR[i] + 1) = BUFFSPECTR[i*3 + 1];
    *((char*)&SPECTR[i] + 0) = BUFFSPECTR[i*3 + 0];
  }


}
/****************************************************************************/

/***ПЕРЕГОНЯЕМ СПЕКТР из imas C ПЕРЕВОРАЧИВАНИЕМ ЗНАЧЕНИЙ в rmas при обмене**/
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
  for (i = 0; i < 1024; ++i)   //запись тек спектра в буферное ОЗУ
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
  for (i = 0; i < 1024; ++i)   //запись тек спектра в буферное ОЗУ
  {
    //BUFFSPECTR[i] = SPECTR[i];
    *((char*)&SPECTR[i] + 0) = BUFFSPECTR[i*3 + 2];
    *((char*)&SPECTR[i] + 1) = BUFFSPECTR[i*3 + 1];
    *((char*)&SPECTR[i] + 2) = BUFFSPECTR[i*3 + 0];
  }


}

/****************************************************************************/
/********* измерение температуры *********/
void TakingTemperature(void)
{
  float ttt;
  if (FLAG.temperCORR)
    ttt = TEMPERFLOAT;    //если режим темпер коррекции передаем среднюю
  else
        ttt = TEMPERFLOATtek;

  if (ttt > 0)
  {
  TEMPERATURE.IntegerPart = (char) ttt;
  TEMPERATURE.FractionPart = (char) (ttt*10.0) - (TEMPERATURE.IntegerPart*10.0);
  }
  else
  {//отрицательная температура на всякий случай
  TEMPERATURE.IntegerPart = (char) (- ttt);
  TEMPERATURE.FractionPart = (char) (-ttt*10.0) - (TEMPERATURE.IntegerPart*10.0);
  TEMPERATURE.IntegerPart = 0xFF - (char) (ttt);
  }
}//TakingTemperature END
/****************************************************************************/
/****** запись строки в UART0 ********/
void BYTEtoUART(unsigned char Charbyte)
{
    while (!(U0LSR & 0x020));
    U0THR= Charbyte;
}//BYTEtoUART end
/****** запись строки в UART0 ********/
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
/****** пересылка 2-х байт в COMMAND ********/
void toCOMMAND2byte(unsigned short nombyte,unsigned short datavalue)
{
  COMMAND[nombyte] = datavalue >> 8;
  nombyte++;
  COMMAND[nombyte] = datavalue;
}//toCOMMAND2byte
/****************************************************************************/
/****** пересылка 2-х байт из COMMAND в short ********/
short toShort2COMMAND(nombyte)
unsigned short nombyte;
{
return (COMMAND[nombyte] << 8 | COMMAND[nombyte+1]);
}//toCOMMAND2byte
/****************************************************************************/
/**** ответ на команду с ошибкой *****/
void COMerr02(void)
{
COMerr(0x02); //недопустимый адрес данных
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

  ClearCOMMAND ();  //обнулили массив команды

}
/****************************************************************************/
/********** STOP EINT0 **********/
void STOPEINT0(void)
{
            if (FLAG.SPECTR)
                            { while (EXTINT & 1)
                                    EXTINT = 0x1;             //сброс прерывания EINT0
                            T1TCR = 0;         //остановили счетчик TMR1 - МЕРТВОЕ ВРЕМЯ
                            VICIntEnClear = VICIntEnClear | INTERENT0;  //запрет перывания
                            }
}//STOPEINT0
/****************************************************************************/
/********** START EINT0 **********/
void STARTEINT0(void)
{
            if (FLAG.SPECTR)
            { VICIntEnable =VICIntEnable | INTERENT0; //разрешение перывания
              T1TCR = 1;         //запустили счетчик TMR1
             }
}//STARTEINT0
/****************************************************************************/
/************COM3  Считать состояние регистров управления **********/
void COM3(void)
{ short adr,kol;
  float InterTemp;
      adr = COMMAND[2] << 8 | COMMAND[3]; //номер в регистрах выборки от 0...4
      kol = COMMAND[4] << 8 | COMMAND[5]; //кол-во рег выборки
       //заданное время измерения спектра 4-байта(2 регистра) остальные по 2 байта(1 регистр)
       //проверка кол-ва регистров выборки
      if (((adr == 0) && ((kol == 2) || (kol == 1))) || ((adr >0) && (kol == 1)) || ((adr==5) && (kol == 2)))
          {//анализ номера регистра выборки, формирование ответа
            COMMAND[2] = kol << 1; //кол-во байт данных ответа = кол-во рег * 2
            FLAG.error = 0;  //флаг локальной ошибки

            switch (adr){
            case 0x00: if (flagTIME2 == 0)
                      {
                       COMMAND[3] = TIMEIZMsek >> 24;
                       COMMAND[4] = TIMEIZMsek >> 16;  //в секундах 2 старших
                       if (kol == 2)
                           {
                            toCOMMAND2byte(5,TIMEIZMsek);  //заданное время накопления спектра 4байта
                            TimeByte= 4;
                            }
                          else
                          {
                            flagTIME2 = 1; //читают по 2 байта
                            TimeByte= 2;
                          }
                      }
                      else
                      {
                       toCOMMAND2byte(3,TIMEIZMsek);  //заданное время накопления спектра 2 младших
                       flagTIME2 = 0;
                      }
                       break;
            case 0x02:  toCOMMAND2byte(3,KODADC);     //коэффициент усиления
                        break;
            case 0x03:  toCOMMAND2byte(3,MINPOROG);   //Нижний порог
                        break;
            case 0x04:  toCOMMAND2byte(3,MAXPOROG);   //верхний порог
                        break;
            case 0x05:



                        COMMAND[2] = 4;               //интервал измерения температуры плав.
                        InterTemp = TIMER0_INTERVAL / 60.0 / 1000.0;
                        COMMAND[6] =  *(unsigned char*)  &  InterTemp;
                        COMMAND[5] =  *((unsigned char*) &  InterTemp+1);
                        COMMAND[4] =  *((unsigned char*) &  InterTemp+2);
                        COMMAND[3] =  *((unsigned char*) &  InterTemp+3);


                        break;
            default:    COMerr02();
                        break; //ном рег>5 или =1 - недопустимый адрес данных
            }//swith adr
          if (!FLAG.error)
              {// ОТВЕТ
                  kol = COMMAND[2]+3;
                  CRC16(COMMAND,kol,0xFF,1);
                  kol+=2;
                  COMMANDtoUART(kol);
              }//!FLAG.error
          }//if adr | kol
                    else //if adr | kol
                    COMerr(0x03); //ошибка кол-ва регистров - недопустимое значение данных
}//END COM3
/****************************************************************************/
/************* COM4 Считать состояние регистров данных **************/
void COM4(void)
{ short adr,kol;
  int LivingTime;
      adr = COMMAND[2] << 8 | COMMAND[3]; //номер в регистрах выборки от 0...4
      kol = COMMAND[4] << 8 | COMMAND[5]; //кол-во рег выборки
       //живое время 4-байта(2 регистра) остальные по 2 байта(1 регистр)
       //проверка кол-ва регистров выборки
      if (((adr == 0) && ((kol == 2)|| (kol == 1)) || ((adr ==2) && (kol == 1))) || ((adr == 3) && (kol == 2)))
          {//анализ номера регистра выборки, формирование ответа
            COMMAND[2] = kol << 1; //кол-во байт данных ответа = кол-во рег * 2
            FLAG.error = 0;  //флаг локальной ошибки
            //LivingTime = T1TC;  //живое время - mSek
            //---25.04.2011-----------

            if( FLAG.SPECTR )
            {
              LivingTime = T1TC;  //живое время - mSek

            }
            else
            {
              LivingTime = T1TC_stop;
            }

            //-----------------------------------------

            switch (adr){
            case 0x00:  LivingTime/=1000; //целых секунд
            case 0x03:
                        if (flagTIME2 == 0)
                        {
                        COMMAND[3] = LivingTime >> 24;
                        COMMAND[4] = LivingTime >> 16;  //1-е ст два байта
                        if (kol == 2)
                        {
                          toCOMMAND2byte(5,LivingTime);   //в ответе -  mSek 4 байта
                          TimeByte =4;
                        }
                          else
                          {
                            flagTIME2 = 1; //читают по 2 байта
                            TimeByte =2;
                          }
                      }
                      else
                      {
                       toCOMMAND2byte(3,LivingTime);  //заданное время накопления спектра 2 младших
                       flagTIME2 = 0;
                      }
                        break;
            case 0x02:  TakingTemperature();            //текущее значение температуры
                        COMMAND[3] = TEMPERATURE.IntegerPart;
                        COMMAND[4] = TEMPERATURE.FractionPart;


                        break;
            default:    COMerr02();
                        break; //ном рег>3 или =1 - недопустимый адрес данных
            }//swith adr
          if (!FLAG.error)
              {// ОТВЕТ
                  kol = COMMAND[2]+3;
                  CRC16(COMMAND,kol,0xFF,1);
                  kol+=2;
                  COMMANDtoUART(kol);
                  //======== Для тестирования WDT (14.12.2015) ==============

                        ++Test_WDT_cnt;
                        if( Test_WDT_cnt > 10)
                        {
                           __disable_interrupt();
                          while(1)
                          {
                          }
                        }

                        //======== Для тестирования WDT (14.12.2015) ==============
              }//!FLAG.error
           }//if adr | kol
                    else //if adr | kol
                    COMerr(0x03); //ошибка кол-ва регистров - недопустимое значение данных
}//END COM4
/****************************************************************************/
/******* COM5  подать управляющий сигнал **************/
void COM5(void)
{ int i;
  if ( (COMMAND[2] == 0) && (COMMAND[5]==0) && ((COMMAND[4] == 0xFF) || (COMMAND[4] == 0)))
    {//анализ номера регистра выборки, ответ - полученная команда
     STOPEINT0();    //ОСТАНОВИЛИ ИЗМЕРЕНИЯ
     switch (COMMAND[3]){
            case 0x00:
                        T1TC_temp = 0;
                        ClearSPECTR();  //обнулить память накопления спектра
                        InitialiseTimer1(); // и таймер живого времени
                        break;
            case 0x01:  if (COMMAND[4]==0x0)
                            {
                              STOPEINT0();
                              FLAG.SPECTR = 0; //запретить накопление спектра
                             }
                         else
                            {
                              //--Тестирование запуска 14.10.2011
                              //if( FL_test == 1 )
                                //-------------------------------
                                FLAG.SPECTR = 1;    //разрешить накопление спектра
                              STARTEINT0();
                              //--Тестирование запуска 14.10.2011
                              //if( FL_test == 0 )
                                // FLAG.SPECTR = 0;    //разрешить накопление спектра
                                //-------------------------------

                            }
                        break;
            case 0x02:  //for (i=1024; --i>=0;)   //запись тек спектра в буферное ОЗУ
                        //      BUFFSPECTR[i] = SPECTR[i];
                        for ( i=1024; --i>=0; )   //запись тек спектра в буферное ОЗУ
                            {
                              //BUFFSPECTR[i] = SPECTR[i];
                              BUFFSPECTR[i*4] = *((char*)&SPECTR[i] + 3);
                              BUFFSPECTR[i*4 + 1] = *((char*)&SPECTR[i] + 2);
                              BUFFSPECTR[i*4 + 2] = *((char*)&SPECTR[i] + 1);
                            }
                        break;
            case 0x05:
                        FLAG.temperCORR = !FLAG.temperCORR; //включить - выключить температурную коррекцию
                        I2CTABLTempFlagW(); //запись c контр чтением в EEPROM
                        break;
            case 0x07:  FLAG.tuning = !FLAG.tuning; //включить - выключить температурную коррекцию
                        break;
            default:    COMerr02();
                        break; //ном рег>2  - недопустимый адрес данных
            }//swith adr
     STARTEINT0(); //ЗАПУСТИЛИ ИЗМЕРЕНИЯ
     if (!FLAG.error)
             // ОТВЕТ
              COMMANDtoUART(8);
     }//if COMMAND
     else // COMMAND
               COMerr(0x03); //ошибка  - недопустимое значение данных
}//end COM5
/****************************************************************************/
/****** COM6 изменить состояние регистров управления *******/
void COM6(void)
{
short adr,kol;
  adr = COMMAND[2]<<8 | COMMAND[3];
  //заданное время измерения спектра 4-байта(2 регистра) остальные по 2 байта(1 регистр)
  //проверка кол-ва регистров выборки
  //анализ номера регистра выборки, формирование ответа
   FLAG.error = 0;  //флаг локальной ошибки
   kol = 8;
   STOPEINT0();  //ОСТАНОВИЛИ ПРЕРЫВАНИЯ, если идет накопление
   switch (adr){
            case 0x00:
                        if (TimeByte == 4)  //ATAS
                          TIMEIZMsek =COMMAND[4]<<24 | COMMAND[5]<<16 ;  //задают время накопления спектра ст часть
                        else
                          if (flagTIME2 == 0) //MOODE
                          {
                          TIMEIZMsek =COMMAND[4]<<24 | COMMAND[5]<<16 ;  //задают время накопления спектра ст часть
                          flagTIME2 = 1;
                          break;
                          }
                          flagTIME2 = 0;
            case 0x01:  TIMEIZMsek |=  COMMAND[4]<<8 | COMMAND[5];  //задают время накопления спектра мл часть
                        STOPEINT0();
                        T1MR0 = TIMEIZMsek*1000; //  = msek задано
                        STARTEINT0();
                        break;
            case 0x02:  KODADCEEPROM = I2CKOEFF(0); //с деинициализацией I2C
                        if (FLAG.temperCORR)
                        {
                            Temp_i=0;       //11.07.2008
                            SETKOEFF(); //корр при устан флаге FLAG.temperCORR
                        }
                        else
                        {
                         KODADC = KODADCEEPROM;
                         ZAPFACTOR(KODADCEEPROM,2); //коэффициент усиления
                        }
                        break;
            case 0x03:  MINPOROG = I2CKOEFF(2);
                        ZAPFACTOR(MINPOROG,0);   //Нижний порог
                        break;
            case 0x04:  MAXPOROG = I2CKOEFF(4);
                        ZAPFACTOR(MAXPOROG,1);  //верхний порог
                        ZAPFACTOR(MINPOROG,0);  //чтобы последним выбирался LDA рег
                        break;
            case 0x05:  MasInt[3] = COMMAND[4]; //ст часть
                        MasInt[2] = COMMAND[5];
                        break;
            case 0x06:  MasInt[1] = COMMAND[4]; //мл часть
                        MasInt[0] = COMMAND[5];
                        I2CIntevarTemperW();
                        ReadEEPROMIntervalTemper();
                        //DeInitialiseI2C();
                        InitialiseTimer0();
                        RESETTMR0();
                        break;
            default:    COMerr02();
                        break; //ном рег>5 или =1 - недопустимый адрес данных
            }//swith adr
    if (!FLAG.error)
              {// ОТВЕТ
                CRC16(COMMAND,6,0xFF,1);
                COMMANDtoUART(kol);
              }//!FLAG.error
    STARTEINT0(); //ЗАПУСТИЛИ ПРЕРЫВАНИЯ,  если идет накопление
}// END COM6
/****************************************************************************/
void COM7(void)
{
   COMMAND[2] = 0;
    if (FLAG.SPECTR)
         COMMAND[2] = 2; //ИДЕТ НАКОПЛЕНИЕ СПЕКТРА
    if (FLAG.temperCORR)
         COMMAND[2] |= 4; //температурная коррекция включена
    if ( FLAG.testerror )
         COMMAND[2] |= 8;//переполнение канала спектра
    if (FLAG.tuning)
         COMMAND[2] |= 0x80; //режим настройки
    CRC16(COMMAND,3,0xFF,1);
    COMMANDtoUART(5);
}// END COM7
/****************************************************************************/
/******* диагностика ********/
void COM8(void)
{
unsigned short adr;
 int icikl; //раб для задержки

 adr = COMMAND[2]<<8 | COMMAND[3];
  switch (adr){
            case 0x00:  // вернуть данные запроса
                        COMMANDtoUART(8); break;
            case 0x01: //перезапустить систему коммуникации
                        if ((COMMAND[4] == 0) && ((COMMAND[5] ==1) || (COMMAND[5] == 2)))
                        {
                          COMMANDtoUART(8); //ответили
                          DELAYTACKT(icikl,100);         //задержка на 100 тактов-5,4uSek
                          InitialiseUART0(COMMAND[5]);  //инициализировали UART0 - здесь можно предусмотреть изм скорости обмена
                        }
                        else
                          COMerr(0x03); //ошибка  - недопустимое значение данных -код скорости не 1 или 2
                        break;
            case 0x02: // Считать содержимое регистра диагностики
                        COMMAND[4]=0;
                        COMMAND[5]=0;
                        if (FLAG.noEINT0)
                                COMMAND[5] = COMMAND[5] | 0x02; //нет прер от детектора в течение 2-х минут
                        if ((FLAG.EEPROMAW) || (FLAG.EEPROMAR) || (FLAG.EEPROMDW) || (FLAG.EEPROMDR))
                                COMMAND[5] = COMMAND[5] | 0x04; //ошибка чтения записи EEPROM
                        if (FLAGOver3byte == 1)
                                COMMAND[5] = COMMAND[5] | 0x10; //переполнение 3-х байт измерения спектра
                        if (FLAG.ErrtempTABL)
                                COMMAND[5] = COMMAND[5] | 0x40; //ошибка CRC табл темп корр
                        if (FLAG.ErrKoef)
                                COMMAND[5] = COMMAND[5] | 0x80; //ошибка CRC коэффициентов
                        if (FLAG.minmaxTemper)
                                COMMAND[5] = COMMAND[4] | 0x04; //температура вне диапазона +9.5...+50.5
                        if (FLAG.temperature)
                                COMMAND[5] = COMMAND[4] | 0x08; // датчик температуры не работает
                        CRC16(COMMAND,6,0xFF,1);
                        COMMANDtoUART(8); //ответили
                        break;
            //---- 10.08.2009 ----------------------
                case 0x81: // Считать содержимое регистра диагностики
                        COMMAND[4]=0;
                        COMMAND[5]=0;
                        if (FLAG.noEINT0)
                                COMMAND[5] = COMMAND[5] | 0x02; //нет прер от детектора в течение 2-х минут
                        if ((FLAG.EEPROMAW) || (FLAG.EEPROMAR) || (FLAG.EEPROMDW) || (FLAG.EEPROMDR))
                                COMMAND[5] = COMMAND[5] | 0x04; //ошибка чтения записи EEPROM
                        if (FLAGOver3byte == 1)
                                COMMAND[5] = COMMAND[5] | 0x10; //переполнение 3-х байт измерения спектра
                        if (FLAG.ErrtempTABL)
                                COMMAND[5] = COMMAND[5] | 0x40; //ошибка CRC табл темп корр
                        if (FLAG.ErrKoef)
                                COMMAND[5] = COMMAND[5] | 0x80; //ошибка CRC коэффициентов
                        if (FLAG.minmaxTemper)
                                COMMAND[5] = COMMAND[4] | 0x04; //температура вне диапазона +9.5...+50.5
                        if (FLAG.temperature)
                                COMMAND[5] = COMMAND[4] | 0x08; // датчик температуры не работает
                        CRC16(COMMAND,6,0xFF,1);
                        COMMANDtoUART(8); //ответили
                        break;

            default:    COMerr02();
                        break; //ном рег>2  - недопустимый адрес данных
          }//swith adr
}// END COM8
/****************************************************************************/
/******* считаь выборку накапливаемого спектра ********/
void COMB(void)
{
  unsigned int adr,kol;
  int i;
  adr = (COMMAND[3] << 8) | COMMAND[4];
  kol = (COMMAND[5] << 8) | COMMAND[6];
  if (adr > 0xC00)
            COMerr02(); //ошибка адреса
  else
    if ((adr+kol) > 3072 || (COMMAND[5] !=0))
            COMerr(0x03); //ошибка данных
         else
         {
           if (adr == 0)
           { if (TimeByte == 2)
                 //TURNSPECTR(SPECTR,BUFFSPECTR);  //переворачиваем
                  TURNSPECTR_B_S();
                 else
                   //MOVSPECTR(SPECTR,BUFFSPECTR);  //ПЕРЕГОНЯЕМ ТЕКУЩИЙ СПЕКТР В БУФФЕР
                   MOVSPECTR_B_S();
           }
           COMMAND[2] = COMMAND[6]; //кол-во
           CRC16(COMMAND,3,0xFF,0);
           CRC16(BUFFSPECTR+adr,COMMAND[2],0,0);
           COMMANDtoUART(3);        //начало команды

           for (i=adr; i<(adr+kol); i++) //спектр
                  BYTEtoUART(BUFFSPECTR[i]);

           BYTEtoUART(CRClo);   //crc
           BYTEtoUART(CRChi);
         }

}// END COMB
/****************************************************************************/
/******* изменить идентификационный код ********/
void COM10(void)
{

  switch (COMMAND[4])
  {
           case 0x00:  // идентификационный код прибора
                         MasEEPROMKoef[12]  = COMMAND[8];     //1пока - идентификационный код
                         COMMAND_temp[0] = COMMAND[8];
                         break;
           case 0x01: // номер версии
                         MasEEPROMKoef[14]  = COMMAND[8];  //только мл байт
                         COMMAND_temp[1] =  COMMAND[8];
                         COMMAND_temp[2] =  COMMAND[7];
                         break;
           case 0x02: // заводской номер
                         MasEEPROMKoef[9]  = COMMAND[7];    //ст байт
                         MasEEPROMKoef[8]  = COMMAND[8];    //мл байт
                         COMMAND_temp[3] =  COMMAND[8];
                         COMMAND_temp[4] =  COMMAND[7];
                         break;
           case 0x03: // дата изготовления
                         MasEEPROMKoef[11]  = COMMAND[7];    //год
                         MasEEPROMKoef[10]  = COMMAND[8];    //неделя
                         COMMAND_temp[5] =  COMMAND[8];
                         COMMAND_temp[6] =  COMMAND[7];
                         break;
           case 0x04: // интерфейсный адрес прибора
                         MasEEPROMKoef[13]  = COMMAND[8];
                         break;
            default:    COMerr02();
                        break; //ном рег>2  - недопустимый адрес данных
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
      // номер версии
      MasEEPROMKoef[14] = COMMAND_temp[3];
      // заводской номер
      MasEEPROMKoef[9]  = COMMAND_temp[6];    //ст байт
      MasEEPROMKoef[8]  = COMMAND_temp[5];
      // дата изготовления
      MasEEPROMKoef[11] = COMMAND_temp[2];    //год
      MasEEPROMKoef[10] = COMMAND_temp[1];
    }
    MasEEPROMKoef[12] = 1;
    I2CMasKoefW ();
   //DeInitialiseI2C();
  }
}// END COM10
/****************************************************************************/
/******* считаь идентификационный код ********/
void COM11(void)
{
    I2CMasKoefR ();
    //DeInitialiseI2C();
    COMMAND[2] = 0x0A;  // 10-кол- байт
    COMMAND[3] = MasEEPROMKoef[12];     //1пока - идентификационный код
    COMMAND[4] = MasEEPROMKoef[14];     //номер версии
    COMMAND[5] = MasEEPROMKoef[9];  //cт байт заводской номер
    COMMAND[6] = MasEEPROMKoef[8];
    COMMAND[7] = MasEEPROMKoef[11];  // год
    COMMAND[8] = MasEEPROMKoef[10];  // неделя изготовление
    if (MasEEPROMKoef[6] == 2) //пока только 1024!!!!!
    {
      toCOMMAND2byte(9,1024); // кол-во каналов
      toCOMMAND2byte(11,1024*3); // кол-во байт спектра
    }
    CRC16(COMMAND,13,0xFF,1);
    COMMANDtoUART(15);
}// END COM11
/****************************************************************************/
/******* считаь текущие данные измерения ********/
void COM12(void)
{
  unsigned int adr;
  int i;
/* кол байт запроса   кол байт выб ст часть кол байт выб мл часть*/
if ((COMMAND[2] != 4) || (COMMAND[5]!=0) || (COMMAND[6] > 223))
              COMerr(0x03); //ошибка  - недопустимое значение данных
    else
    {
      if ((COMMAND[3] != 0) || (COMMAND[4] >223))
                              COMerr(0x02); //ошибка  - недопустимый адрес данных
       else /*читаем данные*/
       {
         COMMAND[2] = COMMAND[6]; //кол-во байт
         adr = COMMAND[4];    //адрес 1-го байта
         CRC16(COMMAND,3,0xFF,0);
         COMMANDtoUART(3);
         CRC16(CurrentDIzm+adr,COMMAND[2],0,0);

         for (i = adr-1; ++i < (adr+COMMAND[2]);)
                 BYTEtoUART(CurrentDIzm[i]);/*возвращаем данные и CRC*/
         BYTEtoUART(CRClo);   //crc
         BYTEtoUART(CRChi);

        }/*читаем данные*/
    } /*else*/

}// END COM12
/****************************************************************************/
/******* записать текущие данные измерения ********/
void COM13(void)
{
int i;
/* кол байт выб ст часть кол байт выб мл часть*/
if ( (COMMAND[5]!=0) || (COMMAND[6] > 223))
              COMerr(0x03); //ошибка  - недопустимое значение данных

  else
    {
      if ((COMMAND[3] != 0) || (COMMAND[4] >=223))
                              COMerr(0x02); //ошибка  - недопустимый адрес данных
       else /* проверка контрольной суммы*/
       {
        CRC16(COMMAND,7,0xFF,0);
        CRC16(CurrentDIzm+COMMAND[4],COMMAND[6],0,0);
        if( (CRClo == CurrentDIzm[COMMAND[4]+ByteCurrentIzm-2]) && (CRChi == CurrentDIzm[COMMAND[4]+ByteCurrentIzm-1]) )
          {/*возвращаем команду */
            COMMANDtoUART(7);
             for (i = COMMAND[4]-1; ++i < (COMMAND[4]+COMMAND[6]+2);)
                  BYTEtoUART(CurrentDIzm[i]); /*возвращаем данные и CRC*/
          } /*вернули команду и данные*/

       }/*проверка контольной суммы- совпадает*/
    } /*else*/
//CurrentDIzm[ByteCurrentIzm-2] = 0; //убрали CRC
//CurrentDIzm[ByteCurrentIzm-1] = 0;
ByteCurrentIzm = 0;
}// END COM13
/****************************************************************************/
/******* записать спектр в инкрементную память ********/
void COM15(void)
{
int i;
unsigned int kol,adr,acrc;
adr = (COMMAND[5] << 8) | COMMAND[6]; /*адрес выборки*/
/* кол байт выб ст часть кол байт выб мл часть*/
kol =(COMMAND[7] << 8) | COMMAND[8];
acrc =adr+kol-TimeByte; /*живое время 4(2)-байта*/
if (adr > 0xC00)
    COMerr(0x02); //ошибка  - недопустимый адрес данных
else
  if ( acrc > 3072)
            COMerr(0x03); //ошибка  - недопустимое значение данных
  else
    {
        CRC16(COMMAND,9,0xFF,0);
        if (adr == 0)
              {/* первая проция с временем */
                CRC16(LiveTime,TimeByte,0,0);
                CRC16(BUFFSPECTR,kol-TimeByte,0,0);
              }
        else
          CRC16(BUFFSPECTR+adr-TimeByte,kol,0,0);
        if( (CRClo == BUFFSPECTR[acrc]) && (CRChi == BUFFSPECTR[acrc+1]) )
          {/*возвращаем команду */
            if (adr == 0)
            {
              COMMANDtoUART(TimeByte+9);// первая порция с временем
              kol-=TimeByte;
            }
            else
            {
              COMMANDtoUART(9);
              adr-=TimeByte;
            }

            for (i = -1; ++i < kol+2;)
                {/*возвращаем данные и CRC*/
                    BYTEtoUART(BUFFSPECTR[adr+i]);
                }//for (i = -1; ++i < kol+2;)

        if ((acrc) >= 3070)
        { /* получили весь спектр*/
//          BUFFSPECTR[acrc] = 0; //убрали CRC
//          BUFFSPECTR[acrc+1] = 0;
          /* измерения*/
          STOPEINT0();      //остановили счетчик TMR1 и запретили EINT0-получаем новый спектр
          if (TimeByte == 2) //если спрашивали время по 2 байта - спектр переворачиваем
                //TURNSPECTR(BUFFSPECTR,SPECTR);
                TURNSPECTR_S_B();
          else
                //MOVSPECTR(BUFFSPECTR,SPECTR);
                MOVSPECTR_S_B();
         kol = ((LiveTime[3] << 24)| (LiveTime[2] << 16)| (LiveTime[1] << 8)| LiveTime[0])*1000; /*в млсек*/
         T1TC  = kol;
         T1PC  = 0;
        }//получили весь спектр
  }// CRC совпадают
  else
    LiveTime[1] = 0;
}// нет ошибок
STARTEINT0(); //запускаем измерение, если разрешено
ByteCurrentIzm = 0;
}// END COM15
/****************************************************************************/
/******* передача таблицы температурной коррекции  ********/
/* CRC считается отдельно*/
void TABLTempUART(void)
{
 int i;

 COMMANDtoUART(3);
 for (i = -1; ++i < 122;)
                  BYTEtoUART(TABLTemperCORR[i]); /* данные */
  BYTEtoUART(CRClo);
  BYTEtoUART(CRChi);
}
/****************************************************************************/
/******* записать таблицу температурной коррекции ********/
void COM16(void)
{

 CRC16(COMMAND,3,0xFF,0);
 CRC16(TABLTemperCORR,122,0,0);
 if( (CRClo == TABLTemperCORR[122]) && (CRChi == TABLTemperCORR[123]) )
  {//CRC совпадает
   I2CTABLTempW();  //запись всего массива в EEPROM + CRC + flag
   //DeInitialiseI2C();
   CRC16(COMMAND,3,0xFF,0);
   CRC16(TABLTemperCORR,122,0,0);  //считаем CRC
   TABLTempUART();  //возвращаем команду +CRC (не считает)
  }//CRC совпадает
ByteCurrentIzm = 0;
}// end COM16
/****************************************************************************/
/******* читать таблицу температурной коррекции ********/
void COM17(void)
{
 COMMAND[2] = 0x7A; // пересылаем 61(122байта) значение для стыковки с ПЭВМ
 CRC16(COMMAND,3,0xFF,0);
 CRC16(TABLTemperCORR,122,0,0);
 TABLTempUART();  //передаем +CRC
}// end COM17
/****************************************************************************/


 //------------ 27.11.2015  для тестирования WDT---------------------
/****************************************************************************/
// Для отладки WDT вызывает зависание !!!!!!!!!
void COM20(void)
{
   COMMAND[2] = WDMOD_temp;

    CRC16(COMMAND,3,0xFF,1);
    COMMANDtoUART(5);
    //------------ 27.11.2015  для тестирования WDT---------------------
    while(1)
    {


    }
}// END COM20
/****************************************************************************/
