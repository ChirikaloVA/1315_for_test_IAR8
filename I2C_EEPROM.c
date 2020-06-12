

#include <NXP/iolpc2294.h>
#include "my_macros.h"
#include <string.h>

#define  I2C_FLAG_AA    (1<<2)
#define  I2C_FLAG_SI    (1<<3)
#define  I2C_FLAG_STO   (1<<4)
#define  I2C_FLAG_STA   (1<<5)
#define  I2C_FLAG_I2EN  (1<<6)

#define  I2C_NO_ERR                    0
#define  I2C_ERR_NO_RESPONSE           1
#define  I2C_ERR_WRONG_PARAM           2
#define  I2C_ERR_24XX_WR_TIMEOUT       3

#define  I2C_WR_24XX_TIMEOUT     10000
#define I2CAddr  0x000000A0      //SLA ����� EEPROM
#define PAGE_SIZE 128

#define Adr_Koeff_EEPROM 0
#define Adr_TempKorr_EEPROM 128
#define Adr_TempInterval_EEPROM 17

//extern unsigned char MasEEPROMKoef[17];
extern unsigned short CRChi,CRClo;
extern unsigned  short KODADC;
extern unsigned  short KODADCEEPROM;
extern unsigned  short MINPOROG;
extern unsigned  short MAXPOROG;
extern __BITFLAG FLAG;   //��� � macros
extern float TEMPERFLOAT;    //������� ������� �����������
extern unsigned char COMMAND[16];


extern void CRC16(unsigned char mas[],short usDataLen,char flFF,char flzapCOMMand);
extern void toCOMMAND2byte(unsigned short nombyte,unsigned short datavalue);
//extern unsigned char MasInt[6];
extern float TIMER0_INTERVAL;     //120000.0�������� � msek // ������ 2 ������  - ��������� �����������


//EEPROM_Meas EEPROM_Meas_;
//EEPROM_Wind EEPROM_Wind_;
//EEPROM_ID EEPROM_ID_;


void ReadEEPROM_test(char * buf, unsigned short Adr);

char I2C_m24xx_wr( char * buf, unsigned short eeprom_addr, unsigned short num);
void I2CTABLTempW(void);

unsigned char MasInt[6];
unsigned char MasEEPROMKoef[17];
unsigned char  TABLTemperCORR[126];

//=====================================================
// �������������� I2C
//=====================================================
void i2c_lpc_init(void)
{
  PCONP =PCONP | 0x80;
  PINSEL0   = (PINSEL0 & 0xFFFFFF0F) | 0x50;   /* enable I2C bus pins */

  I2SCLH    = 22;   //*��������� ������� � 3 ���� /* set I2C bus freq (330Khz)*/
  I2SCLL    = 38;    			        /**3 set I2C bus freq (330Khz)*/
  I2CONCLR = 0xFF;           //-- Clear all flags
  I2CONSET_bit.I2EN = 1;           //-- Set Master Mode
 //  I2CONSET |= I2C_FLAG_I2EN; //--- Enable I2C
}

//---------------------------------------------------------------------------
 void i2c_lpc_wr_byte(int byte)
{
   I2DAT = byte;
   //rI2C_I2CONCLR = I2C_FLAG_SI;                //-- Clear SI
   //while(!(rI2C_I2CONSET & I2C_FLAG_SI));      //-- End wr POINT
   I2CONCLR = I2C_FLAG_SI;                //-- Clear SI
   while(!(I2CONSET & I2C_FLAG_SI));
   //while(! I2CONSET_bit.SI );      //-- End wr POINT
}

//---------------------------------------------------------------------------
 void i2c_lpc_stop()
{
    //-- Set STOP condition
   //rI2C_I2CONCLR = I2C_FLAG_SI;                  //-- Clear SI
  I2CONCLR = I2C_FLAG_SI;                //-- Clear SI
  //I2CONSET |=  I2C_FLAG_AA | I2C_FLAG_STO; //-- Clear NO ASK
  I2CONSET |=  I2C_FLAG_AA | I2C_FLAG_STO;
  //I2CONSET_bit.AA = 1;
  //I2CONSET_bit.STO = 1;
}

//---------------------------------------------------------------------------
 int i2c_lpc_ctrl(int ctrl)
{
   int chk;
   //-- Set START
   I2CONCLR = 0xFF; // Clear all bits
   //I2CONSET |= I2C_FLAG_I2EN | I2C_FLAG_STA;
   I2CONSET_bit.I2EN = 1;
   I2CONSET_bit.STA = 1;
   //while(!(rI2C_I2CONSET & I2C_FLAG_SI));      //--- End START
   //while(! I2CONSET_bit.SI );
   while(!(I2CONSET & I2C_FLAG_SI));
   //-- Set ADDRESS
   I2DAT = ctrl;
   I2CONCLR = I2C_FLAG_STA | I2C_FLAG_SI; //-- Clear START & SI
   //I2CONCLR |= __i2conclr_bits.STAC;
   //I2CONCLR |= __i2conclr_bits.SIC;
   if(ctrl & 1) //-- RD
      chk = 0x40; //-- 40H - SLA+R has been transmitted; ACK has been received
   else
      chk = 0x18; //-- 18H - SLA+W has been transmitted; ACK has been received
  // while(!(rI2C_I2CONSET & I2C_FLAG_SI));      //-- End CTRL
   //while(! I2CONSET_bit.SI );
   while(!(I2CONSET & I2C_FLAG_SI));
   if(I2STAT != chk)
   {
      i2c_lpc_stop();
      return I2C_ERR_NO_RESPONSE;
   }
   return I2C_NO_ERR;
}

//---------------------------------------------------------------------------
 int i2c_lpc_rx_to_buf(char * buf,int num)
{
   int rc;

   if(buf == 0)
      return I2C_ERR_WRONG_PARAM;

   rc = num;
   if(rc > 1)
   {
      I2CONCLR = I2C_FLAG_SI;
      //I2CONCLR_bit.SIC = 1;
      I2CONSET_bit.AA = 1;
      for(;;)
      {
        // while(!(rI2C_I2CONSET & I2C_FLAG_SI));  //-- End Data from slave;
        //while(! I2CONSET_bit.SI );
        while(!(I2CONSET & I2C_FLAG_SI));
         *buf++ = (unsigned char)I2DAT;
         rc--;
         if(rc <= 0)
            break;
         else if(rc == 1)
         {
            //I2CONCLR_bit.AAC = 1;  //-- After next will NO ASK
            //I2CONCLR_bit.SIC = 1;
            I2CONCLR = I2C_FLAG_AA | I2C_FLAG_SI;  //-- After next will NO ASK
         }
         else
         {
            I2CONCLR = I2C_FLAG_SI;                //-- Clear SI
            I2CONSET_bit.AA = 1;
         }
      }
   }
   else if(rc == 1)
   {
      I2CONCLR = I2C_FLAG_AA | I2C_FLAG_SI;  //-- After next will NO ASK
      //while(! I2CONSET_bit.SI );  //-- End Data from slave;
      while(!(I2CONSET & I2C_FLAG_SI));
      *buf = (unsigned char)I2DAT;
   }
   else //err
      return I2C_ERR_WRONG_PARAM;

   return I2C_NO_ERR;
}
//----------------------------------------------------------------------
 int i2c_lpc_ask_polling_op(int ctrl)  //-- wait until write finished
{
   int rc;
   int i;

   for(i=0;i < I2C_WR_24XX_TIMEOUT; i++) //-- actually wr = ~110 but timeout =10000
   {
      I2CONSET_bit.STA = 1;
      I2CONCLR = I2C_FLAG_SI;  //-- Here - clear only SI (not all rI2C_I2CONCLR)
      //while(! I2CONSET_bit.SI );      //wait the ACK
      while(!(I2CONSET & I2C_FLAG_SI));

      I2DAT = ctrl & 0xFE;; // R/WI = 0
      I2CONCLR = I2C_FLAG_SI | I2C_FLAG_STA; //-- Clear START & SI
      //while(! I2CONSET_bit.SI );//wait the ACK
      while(!(I2CONSET & I2C_FLAG_SI));
      rc = I2STAT;
      if(rc == 0x18) //-- got ACK after CLA + W
         break;
      else
      {//�����

      }
   }
   if(i == I2C_WR_24XX_TIMEOUT)
      return I2C_ERR_24XX_WR_TIMEOUT;
   return I2C_NO_ERR;
}
void ReadEEPROM_test(char * buf, unsigned short Adr)
{
  i2c_lpc_ctrl(I2CAddr & 0xFE); //-- Now WR (RD/WI = 0)

  i2c_lpc_wr_byte(*((char*)&Adr + 1));
  i2c_lpc_wr_byte(*((char*)&Adr ));
  i2c_lpc_ctrl(I2CAddr | 0x01); //-- Now RD (RD/WI = 1)
  i2c_lpc_rx_to_buf(buf,16);
  i2c_lpc_stop();     //---- Set STOP ---

}

//----------------------------------------------------------------------------
//static int i2c_lpc_m24xx_wr(
//     int eeprom_type,    //-- EEPROM type
//     int eeprom_addr,    //-- start eeprom addr ( not included Hardware A2,A1,A0)
//     int eeprom_cs_val,  //-- Hardware A2,A1,A0 (valid from 24XX32)
//     char * buf,         //-- Data srs buf
//     int num )            //-- Bytes to write qty
char I2C_EEPROM_wr( char * buf, unsigned short Adr, unsigned short num)
{
   int rc;
   ///int ctrl;
   //int addr_hi;
   //int addr_lo;

   //--- wr START + CONTROL
   rc = i2c_lpc_ctrl(I2CAddr); //-- Now WR (RD/WI = 0)
   if(rc != I2C_NO_ERR)
      return rc;
   //--- wr ADDRESS
   i2c_lpc_wr_byte(*((char*)&Adr + 1));
   i2c_lpc_wr_byte(*((char*)&Adr));
   //---  Write  data
   while(num--)                 //-- transmit data until length>0
   {
      rc = *buf++; //---
      i2c_lpc_wr_byte(rc);
   }
   //-----------------------
   i2c_lpc_stop();

   rc = i2c_lpc_ask_polling_op(I2CAddr);    //-- wait until write finished
   i2c_lpc_stop();
   return I2C_NO_ERR;
}



char I2C_m24xx_wr( char * buf, unsigned short eeprom_addr, unsigned short num)
{
   int page_size = 0;
   int rc;
   int b_to_wr;

   PINSEL0_bit.P0_2 = 1;
   PINSEL0_bit.P0_3 = 1;
   I2CONSET_bit.I2EN = 1;           //-- Set Master Mode

   rc = I2C_NO_ERR;
   for(;;)
   {
     page_size = PAGE_SIZE;

      rc = eeprom_addr % page_size;
      if(rc != 0) //-- not fit on page alignment
      {
         b_to_wr = page_size - rc;
         if(num < b_to_wr)
            b_to_wr = num;
         if(b_to_wr > 0)
         {
             rc = I2C_EEPROM_wr( buf, eeprom_addr, b_to_wr);
             if(rc != I2C_NO_ERR)
                break;
             num -= b_to_wr;
             eeprom_addr += b_to_wr;
             buf += b_to_wr;
         }
      }
       //--- write remainder(by pages,if possible)
      while(num > 0)
      {
          if(num < page_size)
             b_to_wr = num;
          else
             b_to_wr = page_size;

          rc = I2C_EEPROM_wr( buf, eeprom_addr, b_to_wr);
          if(rc != I2C_NO_ERR)
             break;
          num -= b_to_wr;
          eeprom_addr += b_to_wr;
          buf += b_to_wr;
      }

      break;
   }
  I2CONCLR = I2C_FLAG_I2EN;
  PINSEL0_bit.P0_2 = 0;
  PINSEL0_bit.P0_3 = 0;
  return rc;
}

//----------------------------------------------------------------------------

char I2C_m24xx_rd( char * buf, unsigned short eeprom_addr, unsigned short num)
{

   int rc;
   PINSEL0_bit.P0_2 = 1;
   PINSEL0_bit.P0_3 = 1;
   I2CONSET_bit.I2EN = 1;           //-- Set Master Mode

   rc = I2C_NO_ERR;
   for(;;)
   {

       //--- Here - just for addr checking



       //--- wr START + CONTROL
      rc = i2c_lpc_ctrl(I2CAddr & 0xFE); //-- Now WR (RD/WI = 0)
      if(rc != I2C_NO_ERR)
         break;
       //--- wr ADDRESS
      i2c_lpc_wr_byte(*((char*)&eeprom_addr + 1));
      i2c_lpc_wr_byte(*((char*)&eeprom_addr ));

       //--- wr START + CONTROL again - for read start
      rc = i2c_lpc_ctrl(I2CAddr | 0x01); //-- Now RD (RD/WI = 1)
      if(rc != I2C_NO_ERR)
         break;

      rc = i2c_lpc_rx_to_buf(buf,num);
      if(rc != I2C_NO_ERR)
         break;

      i2c_lpc_stop();     //---- Set STOP ---

      break;
   }
   I2CONCLR = I2C_FLAG_I2EN;
   PINSEL0_bit.P0_2 = 0;
   PINSEL0_bit.P0_3 = 0;
   return rc;
}

/****************************************************************************/
/*������ ������� ������������� �� EEPROM,���� �� ��������� CRC ������ � EEPROM*/
/*���� �������� �� ���������*/
/****************************************************************************/
void ReadEEPROMkoef(void)
{

  short idx,flag;

  //I2C_m24xx_rd(MasEEPROMKoef,Adr_Koeff_EEPROM,17); /*������ ������� �����*/
  for(idx = 0; idx < 4; ++idx)
  {
    flag = 0xffff;
    memset(MasEEPROMKoef,0,sizeof(MasEEPROMKoef));
    I2C_m24xx_rd(MasEEPROMKoef,Adr_Koeff_EEPROM,17); /*������ ������� �����*/
    CRC16(MasEEPROMKoef,15,0xFF,0);
    if( (CRClo == MasEEPROMKoef[15]) || (CRChi == MasEEPROMKoef[16]) )
    {
      flag = 0;
      FLAG.ErrKoef = 0;
      break;
    }
  }

  //I2C_m24xx_rd(MasEEPROMKoef,Adr_Koeff_EEPROM,17); /*������ ������� �����*/
  //CRC16(MasEEPROMKoef,15,0xFF,0);
  //if( (CRClo != MasEEPROMKoef[15]) || (CRChi != MasEEPROMKoef[16]) )
  if( flag )
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

   MasEEPROMKoef[14] = 0x02;   //����� ������
   //----- 10.08.2009
   CRC16(MasEEPROMKoef,15,0xFF,0);

   MasEEPROMKoef[15] = CRClo;
   MasEEPROMKoef[16] = CRChi;

   //������ ������������� �� ���������
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! test
  I2C_m24xx_wr(MasEEPROMKoef,Adr_Koeff_EEPROM,17); /*������ ������� �������������*/
  FLAG.ErrKoef = 1;
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
unsigned  short  I2CKOEFF( int IndMas)
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

//InitialiseI2C(1,0); //������������� ��� ������
//I2CTransferByte(IndMas,2,MasEEPROMKoef+IndMas); //������ �������
//****** 10.08.2009
//I2C_m24xx_wr(MasEEPROMKoef+IndMas,0x200+IndMas,2);
I2C_m24xx_wr(MasEEPROMKoef+IndMas,Adr_Koeff_EEPROM+IndMas,2);



CRC16(MasEEPROMKoef,15,0xFF,0);

MasEEPROMKoef[15] = CRClo;   //�� ����
MasEEPROMKoef[16] = CRChi;  //c� ����




//I2CTransferByte(15,2,MasEEPROMKoef+15); //������ ������ CRC
//I2C_m24xx_wr(MasEEPROMKoef+15,0x200+15,2);//������ ������ CRC
//****** 10.08.2009
I2C_m24xx_wr(MasEEPROMKoef+15,Adr_Koeff_EEPROM+15,2);//������ ������ CRC

//I2CTransferByte(IndMas,0);  //��������� ������ ������ �� IndMas


//I2CTransferByte(IndMas,2,MasEEPROMKoef+IndMas); //����������� ������

NameKoef  = MasEEPROMKoef[IndMas+1] << 8 | MasEEPROMKoef[IndMas];  //�����������
toCOMMAND2byte(4,NameKoef);     //����������� ����� � �������
return NameKoef;
}//end I2CKOEFF

//================================================================
//
// ���������� ������ � EEPROM C CRC CRC ����������� � ���� ���������
// ������ ��������
//
//=================================================================


/****************************************************************************/
/*������ ������� ������������� ������������� ���������  �� EEPROM,*/
/*���� �� ��������� CRC ������ � EEPROM c���������� �������� =0 */
/****************************************************************************/
void ReadTABLtemper(void)
{
  int i;
  I2C_m24xx_rd(TABLTemperCORR,Adr_TempKorr_EEPROM,126);//������ ����� ������� + CRC + ����
  //I2CTABLTempR(); //������ ����� ������� + CRC + ����
  CRC16(TABLTemperCORR,122,0xFF,0);
  if( (CRClo != TABLTemperCORR[122]) || (CRChi != TABLTemperCORR[123]) )
  {
    //ClearTABLtemperCORR();  // ������ ��� �������
    for (i=122; --i >=0;)
              TABLTemperCORR[i] = 0;
    FLAG.ErrtempTABL = 1; //���� ������ CRC ���� ���� ���� - ��� 1-� �������������
    TABLTemperCORR[125] = 1; //���� ������ ��������� - 0- ��������� 1 - ��������

    I2CTABLTempW();
}
  FLAG.temperCORR = 0;
  if (TABLTemperCORR[125])
          FLAG.temperCORR = 1;  //���� ������ ��������� - ���������
}//end ReadTABLtemper
/****************************************************************************/


/****************************************************************************/
/*������ ��������� ��������� ����������� �� EEPROM,���� �� ��������� CRC ������ � EEPROM*/
/*�������� �� ��������� = 2 �������*/
/****************************************************************************/
void ReadEEPROMIntervalTemper(void)
{
  int irab;
  float Inter2min;

  //I2CIntevarTemperR(); /*������ ���������*/
  I2C_m24xx_rd(MasInt,Adr_TempInterval_EEPROM,6); //������ ���������
  CRC16(MasInt,4,0xFF,0);
  if( (CRClo != MasInt[4]) || (CRChi != MasInt[5]) )
  {
   Inter2min = 2.0;
   MasInt[0] =  *(unsigned char*) & Inter2min; //�� ����
   MasInt[1] =  *((unsigned char*) & Inter2min+1);
   MasInt[2] =  *((unsigned char*) & Inter2min+2);
   MasInt[3] =  *((unsigned char*) & Inter2min+3);
   DELAYTACKT(irab,100006);
   //I2CIntevarTemperW(); /*������ ��������� = 2 ������� � CRC*/
   //while (lock == 1); //���� �� �������� ������ ������� CRC
 }
* (unsigned char*) & Inter2min   = MasInt[0];
*((unsigned char*) & Inter2min+1)= MasInt[1];
*((unsigned char*) & Inter2min+2)= MasInt[2] ;
*((unsigned char*) & Inter2min+3)= MasInt[3];
TIMER0_INTERVAL=Inter2min * 60.0 * 1000.0;     //�������� � msek
}
/****************************************************************************/

/**************************************************************/
/*������ ����������� ��������� � EEPROM, ��� ����� ������ */
/* � ���������������� I2C*/
/*************************************************************/
void I2CIntevarTemperW(void)
{
int irab;
CRC16(MasInt,4,0xFF,0);
MasInt[4] = CRClo;   //�� ����
MasInt[5] = CRChi;  //c� ����

//InitialiseI2C(1,0); //������������� ��� ������
//I2CTransferByte(17,6,MasInt); //������ �������
I2C_m24xx_wr(MasInt,Adr_TempInterval_EEPROM,6);

//while (lock == 1); //���� �� �������� ������ ������� CRC

//DELAYTACKT(irab,100006);
//I2C_m24xx_rd(MasInt,Adr_TempInterval_EEPROM,6); //������ ���������
//I2CIntevarTemperR();
//while (lock == 1); //���� �� ��������
}//end 2CIntevarTemperW
/*********************************************************/
/****************************************************************************/
/*������ ������� ������������� + CRC*/
void I2CMasKoefW (void)
{
  CRC16(MasEEPROMKoef,15,0xFF,0);
  MasEEPROMKoef[15] = CRClo;   //�� ����
  MasEEPROMKoef[16] = CRChi;  //c� ����
  //InitialiseI2C(1,0); //������������� ��� ������
  //I2CTransferByte(0,17,MasEEPROMKoef);
  //while(lock == 1); //����

  I2C_m24xx_wr(MasEEPROMKoef,Adr_Koeff_EEPROM,17);
}
/****************************************************************************/
/*������ ������� ������������� + CRC*/
void I2CMasKoefR (void)
{
  //InitialiseI2C(0,0); //������������� 1-� ��� ������
  //I2CTransferByte(0,0);  //��������� ������ ������ 0
  //InitialiseI2C(1,1); //�������������   ��� ������
  //I2CTransferByte(0,17,MasEEPROMKoef);
  //while(lock == 1); //����
  I2C_m24xx_rd(MasEEPROMKoef,Adr_Koeff_EEPROM,17); /*������ ������� �����*/

}

/****************************************************************************/
/*������ ������� ������������� ��������� + CRC � ����� � EEPROM -125 ���� */
/****************************************************************************/
void I2CTABLTempW(void)
{
    CRC16(TABLTemperCORR,122,0xFF,0);//CRC ������� ������ ��� ����
    TABLTemperCORR[122] = CRClo;
    TABLTemperCORR[123] = CRChi;
    //InitialiseI2C(1,0); //������������� ��� ������
    //I2CTransferByte(128,126,TABLTemperCORR);
    //while(lock == 1); //���� ������
    I2C_m24xx_wr(TABLTemperCORR,Adr_TempKorr_EEPROM,126);
}
/****************************************************************************/
/*������ ����� ������������� ��������� � EEPROM -128+125 ���� */
/* � ���������������� I2C*/
/****************************************************************************/
void I2CTABLTempFlagW(void)
{
    //InitialiseI2C(1,0); //������������� ��� ������

    TABLTemperCORR[125]=0;
    if (FLAG.temperCORR == 1)
          TABLTemperCORR[125]=1;  //���� ������ ��������� 1- ���������
    I2C_m24xx_wr(TABLTemperCORR + 125,Adr_TempKorr_EEPROM + 125,1);
    //I2CTransferByte(128+125,1,TABLTemperCORR+125);
    //while(lock == 1); //���� ������

    //I2CTransferByte(0,0);  //��������� ������ ������ �� 0 ����� ���� ������ ���

    //I2CTransferByte(128+125,0);  //��������� ������ ������ �����
    //InitialiseI2C(1,1); //������������� ��� ������
    //I2CTransferByte(128+125,1,TABLTemperCORR+125);
    //DeInitialiseI2C();
    I2C_m24xx_rd(TABLTemperCORR + 125,Adr_TempKorr_EEPROM + 125,1);
    FLAG.temperCORR = 0;
    if (TABLTemperCORR[125])
          FLAG.temperCORR = 1;  //���� ������ ��������� - ���������
}
