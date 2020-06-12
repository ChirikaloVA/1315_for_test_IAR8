/*******************************************************************/
/*    �� I2C - EEPROM        */
/*******************************************************************/
#include <iolpc2214.h>
#include <stdarg.h>
#include "my_macros.h"

#define I2CAddr  0x000000A0      //SLA ����� EEPROM
/*bit Freq = (TARGED_BOARD_FREQ)/4/(I2SCLH+I2SCLL) =~ 330 K�� (263 ��� 14745)*/
extern unsigned  short KODADC;
extern unsigned  short KODADCEEPROM;
extern unsigned  short MINPOROG;
extern unsigned  short MAXPOROG;
extern __BITFLAG FLAG;   //��� � macros
extern unsigned char COMMAND[16];
extern unsigned short CRChi,CRClo;
extern void CRC16(unsigned char mas[],short usDataLen,char flFF,char flzapCOMMand);
extern void toCOMMAND2byte(unsigned short nombyte,unsigned short datavalue);
extern void DeInitialiseI2C(void);
extern float TIMER0_INTERVAL;     //120000.0�������� � msek // ������ 2 ������  - ��������� �����������
extern float TEMPERFLOAT;    //������� ������� �����������


unsigned char *I2CData,     //����� ������� ������ ��� EEPROM
               I2CCounter,  //���-�� ���� ������
               I2CAddress,  //SLA(CONTROL BYTE) ����� ���������� EEPROM (� ��� 1010 000r/w, 1-r,0-w)
               MemAddress[2], //����� ����� � EEPROM
               lock;        //���� ������ ������ � I2C(EEPROM)
unsigned char Flmembyte2;  //���� ��������� 2-�� ����� ������ � EEPROM
unsigned char FlerrRW;    //���� ���-�� ��������� ��������� � EEPROM <5
unsigned char MasEEPROMKoef[17];
unsigned char TABLTemperCORR[126]; //������� ������ ��������� 61 ���� �� 2 ����� +CRC 2�+ 1� ����
void I2CInterruptW(void);
void I2CInterruptR(void);
unsigned char MasInt[6];

/******************************************************************************************
 * Initialise I2C bus used to access
 ****************************************************************************************/
void InitialiseI2C(unsigned char nn,unsigned char FlRead)
/*nn=0 1-� ���� ��� ������������� �������,������������� ��������, nn=1 ����������� ����� */
/*FlRead = 0 ���� ��� ������ �� ����������, FlRead=1 ��� ������ �� ���������� */
{
//   extern void handle_i2c(void);
   while (lock == 1) ;
   lock       = 0;
   PCONP =PCONP | 0x80;
   PINSEL0   = (PINSEL0 & 0xFFFFFF0F) | 0x50;   /* enable I2C bus pins */
   if (nn == 0)
      {
        VICVectCntl3  = 0x29;    			/* vector I2C as highest priority and enabled */
        I2SCLH    = 22;   //*��������� ������� � 3 ���� /* set I2C bus freq (330Khz)*/
        I2SCLL    = 38;    			        /**3 set I2C bus freq (330Khz)*/
      }
   if (FlRead == 0)
            {
            VICVectAddr3  = (int) I2CInterruptW;	  	/* vector I2C interrupt address ��� ������*/
            I2CAddress = I2CAddr;			//Place address and data in Globals to be used by the interrupt
            }
    else
            {
            VICVectAddr3  = (int) I2CInterruptR;	  	/* vector I2C interrupt address ��� ������*/
            I2CAddress = I2CAddr | 1;			//Place address and data in Globals to be used by the interrupt
            }
   VICIntEnable =  VICIntEnable | 0x00000200; /*���������� ���������� �� I2C*/
   Flmembyte2 = 0;
   FlerrRW    = 5;
   FLAG.EEPROMAW =0;
   FLAG.EEPROMAR =0;
   FLAG.EEPROMDW =0;
   FLAG.EEPROMDR =0;
}// end  InitialiseI2C
/****************************************************************************/
/* stop bit to I2C (EEPROM)*/
/****************************************************************************/
void I2CStopBit (void)
{
I2CONSET=0x10; //_bit.STO = 1;         //Stop condition
Flmembyte2 = 0;
lock = 0;
}// end I2CStopBit
/****************************************************************************/
/* Interrupt handler for I2C status change ������ � EEPROM*/
/****************************************************************************/
void I2CInterruptW(void)
{
	switch(I2STAT)
	{
	case 0x08:                  			 // a START condition has been transmitted
		I2CONCLR=0x20;  //  _bit.STAC = 1;	 // Clear the START bit
		I2DAT = I2CAddress; 			 // Address and data direction
		break;
	case 0x18:		     			 // high SLA+W has been transmitted, ACK has been received
		I2DAT = MemAddress[0]; 			 // Data to transmit.
		break;
	// ������ � EEPROM
        case 0x20:                  			 // SLA+W not ACK
		  if (FlerrRW != 0)
                    {
                    I2DAT = I2CAddress; 			 // Resend Address ????????
                    FlerrRW--;
                    }
                  else
                                {
                                I2CStopBit();         //Stop condition
                                FLAG.EEPROMAW=1; //���� ������ ������ ������ � EEPROM
                                }
		break;
        case 0x28:                                        // Data sent,ACK
                if (Flmembyte2 != 0)
                {
                    if (I2CCounter-- > 0)
                    {
                        I2DAT =*I2CData;                      // Write data
                        I2CData++;
                    }
                    else
                      I2CStopBit();        //Stop condition
                }
                else
                {
                 I2DAT = MemAddress[1];   //low 2-� ���� ������
                 Flmembyte2++;
                }
                break;
        case 0x30:                                       //Data send, Not ACK
                if ((I2CCounter==0) || (FlerrRW == 0))
                                {
                                I2CStopBit();  //Stop condition
                                FLAG.EEPROMDW=1; //���� ������ ������ ������ � EEPROM
                                }
                else
                  {
                  I2DAT = *I2CData;                           //Write data - ????????
                  FlerrRW--;
                  }
                break;
	default:
                I2CStopBit();  //Stop condition
		break;
	}//switch end
        I2CONCLR=0x08; //_bit.SIC = 1;                  //Clear I2C interrapt
}// end I2CInterruptW

/****************************************************************************/
/* Interrupt handler for I2C status change ������ �� EEPROM*/
/****************************************************************************/
void I2CInterruptR(void)
{
	switch(I2STAT)
	{
	case 0x08:                  			 // a START condition has been transmitted
		I2CONCLR=0x20;  //  _bit.STAC = 1;	 // Clear the START bit
		I2DAT = I2CAddress; 			 // Address and data direction
		break;
        // ������ �� EEPROM
        case 0x40:                   		// SLA+R has been transmitted,
	        I2CONSET=0x04; //_bit.AA = 1;	// ACK has been received
		break;
        case 0x48:                   		// SLA+R not ACK,
                if (FlerrRW !=0)
                  {
	          I2CONSET=0x20; //_bit.STA = 1;	        // resend start
                  FlerrRW--;
                  }
                else
                {
                  I2CStopBit();  //Stop condition
                  FLAG.EEPROMAR = 1;
                }

		break;
        case 0x50:                                        // Data read,ACK
                if (--I2CCounter > 0)
                {
                  *I2CData = I2DAT;                      // read data
                  I2CData++;
               }
                else
                {
                  I2CONCLR=0x04;  //_bit.AA = 0;         //��������� ���� ������ ��� ������� ACK
                  *I2CData = I2DAT;
                }
                break;
	case 0x58:				// Data  received, NOT ACK has not been returned
                if (I2CCounter == 0)
                      I2CStopBit();        //Stop condition
                 else
                {
                  if (FlerrRW !=0)
                  {
                  *I2CData = I2DAT;                      // read data
                  I2CData++;
                  FlerrRW--;
                  }
                  else
                  {
                  I2CStopBit();  //Stop condition
                  FLAG.EEPROMDR =1;
                  }
                }
                break;
	default:
                I2CStopBit();  //Stop condition
		break;
	}//switch end
        I2CONCLR=0x08; //_bit.SIC = 1;                  //Clear I2C interrapt
}// end I2CInterruptR
/****************************************************************************/
/*������ ���������� �� ������ / ������ � EEPROM*/
/****************************************************************************/
void I2CTransferByte(unsigned char MemAddr,unsigned char count,...)
{//���� count=0  ��������������� ������ �����
va_list ap;
va_start(ap,count);

while(lock == 1)				//Wait for interrupt to signal end of I2C activity
{
;
}
lock 	   = 1;                   		//Set I2C bus as active
Flmembyte2 = 0;
if(count >0)
{
I2CData  	= va_arg(ap,unsigned char *);
}
I2CCounter	= count;
MemAddress[0]	= MemAddr >> 8; //������ ������������ ��. ���� ������
MemAddress[1]	= MemAddr;
I2CONCLR 	= 0x000000FF;			//Clear all I2C settings
I2CONSET 	= 0x00000004; 			//Enable ACK
I2CONSET 	= 0x00000040; 			//Enable the I2C interface
I2CONSET 	= 0x00000020; 			//Start condition
va_end(ap);
}//end I2CTransferByte
/****************************************************************************/
/*������ ������� ������������� + CRC*/
void I2CMasKoefW_ (void)
{
  CRC16(MasEEPROMKoef,15,0xFF,0);
  MasEEPROMKoef[15] = CRClo;   //�� ����
  MasEEPROMKoef[16] = CRChi;  //c� ����
  InitialiseI2C(1,0); //������������� ��� ������
  I2CTransferByte(0,17,MasEEPROMKoef);
  while(lock == 1); //����
}
/****************************************************************************/
/*������ ������� ������������� + CRC*/
void I2CMasKoefR_ (void)
{
  InitialiseI2C(0,0); //������������� 1-� ��� ������
  I2CTransferByte(0,0);  //��������� ������ ������ 0
  InitialiseI2C(1,1); //�������������   ��� ������
  I2CTransferByte(0,17,MasEEPROMKoef);
  while(lock == 1); //����
}
/****************************************************************************/
/*������ ��������� ��������� ����������� �� EEPROM+CRC*/
/****************************************************************************/
void I2CIntevarTemperR()
{
  InitialiseI2C(1,0); //�������������  ��� ������
  I2CTransferByte(17,0);  //��������� ������ ������ 17-�  ������
  InitialiseI2C(1,1); //�������������   ��� ������
  I2CTransferByte(17,6,MasInt); //4����� ���� �������� +CRC
  while(lock == 1); //����
}// end 2CIntevarTemperR
/**************************************************************/
/*������ ����������� ��������� � EEPROM, ��� ����� ������ */
/* � ���������������� I2C*/
/*************************************************************/
void I2CIntevarTemperW_(void)
{
int irab;
CRC16(MasInt,4,0xFF,0);
MasInt[4] = CRClo;   //�� ����
MasInt[5] = CRChi;  //c� ����

InitialiseI2C(1,0); //������������� ��� ������
I2CTransferByte(17,6,MasInt); //������ �������

while (lock == 1); //���� �� �������� ������ ������� CRC

DELAYTACKT(irab,100006);

I2CIntevarTemperR();
while (lock == 1); //���� �� ��������
}//end 2CIntevarTemperW
/*********************************************************/
/****************************************************************************/
/*������ ��������� ��������� ����������� �� EEPROM,���� �� ��������� CRC ������ � EEPROM*/
/*�������� �� ��������� = 2 �������*/
/****************************************************************************/
void ReadEEPROMIntervalTemper_(void)
{
  int irab;
  float Inter2min;

  I2CIntevarTemperR(); /*������ ���������*/
  while (lock == 1); //���� �� �������� ������ ������� CRC
  CRC16(MasInt,4,0xFF,0);
  if( (CRClo != MasInt[4]) || (CRChi != MasInt[5]) )
  {
   Inter2min = 2.0;
   MasInt[0] =  *(unsigned char*) & Inter2min; //�� ����
   MasInt[1] =  *((unsigned char*) & Inter2min+1);
   MasInt[2] =  *((unsigned char*) & Inter2min+2);
   MasInt[3] =  *((unsigned char*) & Inter2min+3);
   DELAYTACKT(irab,100006);
   I2CIntevarTemperW_(); /*������ ��������� = 2 ������� � CRC*/
   while (lock == 1); //���� �� �������� ������ ������� CRC
 }
* (unsigned char*) & Inter2min   = MasInt[0];
*((unsigned char*) & Inter2min+1)= MasInt[1];
*((unsigned char*) & Inter2min+2)= MasInt[2] ;
*((unsigned char*) & Inter2min+3)= MasInt[3];
TIMER0_INTERVAL=Inter2min * 60.0 * 1000.0;     //�������� � msek
}
/****************************************************************************/
/*������ ������� ������������� �� EEPROM,���� �� ��������� CRC ������ � EEPROM*/
/*���� �������� �� ���������*/
/****************************************************************************/
void ReadEEPROMkoef_(void)
{
  int irab;
  I2CMasKoefR_(); /*������ ������� �����*/
  CRC16(MasEEPROMKoef,15,0xFF,0);
  if( (CRClo != MasEEPROMKoef[15]) || (CRChi != MasEEPROMKoef[16]) )
  {
    //�����������  ����� ������������� �� ���������- ������������� (�������������� �� ���������]
   /*����� 1,0 - KODADC   ����������� �������� [�� ��������� 1300]
           3,2 - MINPOROG ������ ����� [�� ��������� 64]
           5,4 - MAXPOROG ������� ����� [�� ��������� 4060]
           6   - ���-�� ������� ���� 1,0 1024 =10 (2)
           7   - ��� �������� ������ 115200��� = 1
           9,8 - ��������� ����� [�� ��������� 1]
           11,10 - ���� ������������ ���[07] , ������[01]
           12  - ����������������� ��� = 0
           13  - ������������ ����� = 1
           14  - ����� ������ =1
           16,15 - CRC
           20,19,18,17  - �������� ��������� ����������� � ������� 4 ����� ���� �����
           22,21 - CRC
           */
   MasEEPROMKoef[1] = 0x05; //c� ���� ����������� ��������1300>>8;
   MasEEPROMKoef[0] = 0x14;    //�� ���� 1300

   MasEEPROMKoef[3] = 0;      //c� ���� ������ �����
   MasEEPROMKoef[2] = 0x40;    //�� ����=64;

   MasEEPROMKoef[5] = 0x0F; //c� ���� ������� �����4060>>8;
   MasEEPROMKoef[4] = 0xDC;    //�� ���� 4060;

   MasEEPROMKoef[6] = 0x02;    //���-�� ������� ���� 1,0 1024 =10 (2)

   MasEEPROMKoef[7] = 0x01;    //��� �������� ������ 115200���

   MasEEPROMKoef[9] = 0x0;    //c� ���� ��������� �����
   MasEEPROMKoef[8] = 0x0;    //�� ����

   MasEEPROMKoef[11] = 0x07;  //c� ���� -���
   MasEEPROMKoef[10] = 0x01;    //�� ����  -������

   MasEEPROMKoef[12] = 0x1;    //����������������� ���

   MasEEPROMKoef[13] = 0x01;   //������������ �����

   MasEEPROMKoef[14] = 0x01;   //����� ������

   //������ ������������� �� ���������
  DELAYTACKT(irab,100006);
  I2CMasKoefW_(); /*������ ������� �������������*/
  } //end if( (CRClo != MasEEPRO......
  KODADC   = MasEEPROMKoef[1] << 8 | MasEEPROMKoef[0];  //����������� �������� ��� short
  KODADCEEPROM = KODADC;
  MINPOROG = MasEEPROMKoef[3] << 8 | MasEEPROMKoef[2];  //������ ����� ��� short
  MAXPOROG = MasEEPROMKoef[5] << 8 | MasEEPROMKoef[4];  //������� ����� ��� short
 }//end ReadEEPROMkoef
/**************************************************************/
/*������ ����������� ������������� � EEPROM, ��� ����� ������ */
/* � ���������������� I2C*/
/*************************************************************/
unsigned  short  I2CKOEFF_( int IndMas)
{
unsigned short NameKoef;
int irab,i;
if ((IndMas == 0) && (FLAG.temperCORR))
      {//��� ������ ��������� - ��������� ����� �������� �� ������� ������ ��� �����������
      i = ((int)(TEMPERFLOAT+0.5)+10)<<1;  //������� ������ ����� -10 ����,����������
      irab = (COMMAND[4]<<8 | COMMAND [5]) - (TABLTemperCORR[i]<< 8 | TABLTemperCORR[i+1]);
      MasEEPROMKoef[IndMas]   = irab;
      MasEEPROMKoef[IndMas+1] = irab >> 8;
      }
else
{
MasEEPROMKoef[IndMas+1] = COMMAND[4];
MasEEPROMKoef[IndMas]   = COMMAND[5];
}

InitialiseI2C(1,0); //������������� ��� ������
I2CTransferByte(IndMas,2,MasEEPROMKoef+IndMas); //������ �������

while (lock == 1); //���� �� �������� ������ ������� CRC

CRC16(MasEEPROMKoef,15,0xFF,0);

MasEEPROMKoef[15] = CRClo;   //�� ����
MasEEPROMKoef[16] = CRChi;  //c� ����


DELAYTACKT(irab,100006);
I2CTransferByte(15,2,MasEEPROMKoef+15); //������ ������ CRC

DELAYTACKT(irab,100006);
I2CTransferByte(IndMas,0);  //��������� ������ ������ �� IndMas

InitialiseI2C(1,1); //������������� ��� ������
I2CTransferByte(IndMas,2,MasEEPROMKoef+IndMas); //����������� ������
DeInitialiseI2C();
NameKoef  = MasEEPROMKoef[IndMas+1] << 8 | MasEEPROMKoef[IndMas];  //�����������
toCOMMAND2byte(4,NameKoef);     //����������� ����� � �������
return NameKoef;
}//end I2CKOEFF
/*********************************************************/
/*****************************************************/
void ClearTABLtemperCORR (void)
{
  int i;

  for (i=122; --i >=0;)
              TABLTemperCORR[i] = 0;
}
/****************************************************************************/
/*������ ����� ������������� ��������� � EEPROM -128+125 ���� */
/* � ���������������� I2C*/
/****************************************************************************/
void I2CTABLTempFlagW(void)
{
    InitialiseI2C(1,0); //������������� ��� ������

    TABLTemperCORR[125]=0;
    if (FLAG.temperCORR == 1)
          TABLTemperCORR[125]=1;  //���� ������ ��������� 1- ���������

    I2CTransferByte(128+125,1,TABLTemperCORR+125);
    while(lock == 1); //���� ������

    I2CTransferByte(0,0);  //��������� ������ ������ �� 0 ����� ���� ������ ���

    I2CTransferByte(128+125,0);  //��������� ������ ������ �����
    InitialiseI2C(1,1); //������������� ��� ������
    I2CTransferByte(128+125,1,TABLTemperCORR+125);
    DeInitialiseI2C();

    FLAG.temperCORR = 0;
    if (TABLTemperCORR[125])
          FLAG.temperCORR = 1;  //���� ������ ��������� - ���������
}
/****************************************************************************/
/*������ ������� ������������� ��������� + CRC � ����� � EEPROM -125 ���� */
/****************************************************************************/
void I2CTABLTempR(void)
{
  InitialiseI2C(1,0);     //������������� ��� ������
  I2CTransferByte(128,0);  //��������� ������ ������ ������� = 128
  InitialiseI2C(1,1); //�������������   ��� ������
  I2CTransferByte(128,126,TABLTemperCORR);//������ ������� ����� 128!!!
  while(lock == 1); //���� ������
}
/****************************************************************************/
/*������ ������� ������������� ��������� + CRC � ����� � EEPROM -125 ���� */
/****************************************************************************/
void I2CTABLTempW_(void)
{
    CRC16(TABLTemperCORR,122,0xFF,0);//CRC ������� ������ ��� ����
    TABLTemperCORR[122] = CRClo;
    TABLTemperCORR[123] = CRChi;
    InitialiseI2C(1,0); //������������� ��� ������
    I2CTransferByte(128,126,TABLTemperCORR);
    while(lock == 1); //���� ������
}
/****************************************************************************/
/*������ ������� ������������� ������������� ���������  �� EEPROM,*/
/*���� �� ��������� CRC ������ � EEPROM c���������� �������� =0 */
/****************************************************************************/
void ReadTABLtemper_(void)
{
  I2CTABLTempR(); //������ ����� ������� + CRC + ����
  CRC16(TABLTemperCORR,122,0xFF,0);
  if( (CRClo != TABLTemperCORR[122]) || (CRChi != TABLTemperCORR[123]) )
  {
    ClearTABLtemperCORR();  // ������ ��� �������
    FLAG.ErrtempTABL = 1; //���� ������ CRC ���� ���� ���� - ��� 1-� �������������
    TABLTemperCORR[125] = 1; //���� ������ ��������� - 0- ��������� 1 - ��������
    I2CTABLTempW_();
}
  FLAG.temperCORR = 0;
  if (TABLTemperCORR[125])
          FLAG.temperCORR = 1;  //���� ������ ��������� - ���������
}//end ReadTABLtemper
/****************************************************************************/
