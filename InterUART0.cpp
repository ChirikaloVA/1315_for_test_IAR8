/******************************************************/
/*        �� ���������  UART0 (IRQ) ����������       */
/****************************************************/
#include <intrinsics.h>
#include <NXP/iolpc2294.h>
#include "my_macros.h"

#define INTERUART0	  0x40    /* bit 6 int*/

extern unsigned char COMMAND[16];
extern unsigned int ByteCOMMAND;
extern __BITFLAG FLAG;   //��� � macros
extern unsigned int PCLK_VPB;

extern unsigned char CurrentDIzm [226]; //������� ������ ��������� 223 + CRC 2 �����
extern int ByteCurrentIzm,counter;
extern unsigned char BUFFSPECTR[3075];
extern unsigned char TABLTemperCORR[126]; //������� ������ ��������� 61 ���� �� 2 ����� +CRC 2�+ 1� ����
extern unsigned char TimeByte;    //���-�� ���� � ������������� ������� 4 ��� 2
extern unsigned int SoftTimerVal; //������� ������ ��������

static void UART0Interrupt(void);
unsigned char LiveTime[4];

/****************************************************************************/
void InitialiseUART0(unsigned char speed)
{
  int SpeedRS232;
  if (speed == 1)
    SpeedRS232 = 115200;   //�������� ������
  else //2
    SpeedRS232 = 19200;    //�������� ������


//   char urLSR;
//   extern void handle_uart0(void);

   PINSEL0 = (PINSEL0 & 0xFFFFFFF0) |0x05; //UART0_PCB_PINSEL_CFG;
   VICVectCntl0  = 0x26;    			   	  /* vector UART0 as 2nd highest priority and enabled */
   VICVectAddr0  = (int) UART0Interrupt;	   	  /* vector UART0 interrupt address */


   // enable access to divisor latch regs
   U0IER = 0;
   U0LCR = 0x83;        //LCR_ENABLE_LATCH_ACCESS;
   // set divisor for desired baud
   U0DLM = 0x00;
   U0DLL = ((PCLK_VPB*1000) / (SpeedRS232*16)); //0x2;  //115200 ->8/4 //���� 0x0A/4->18432000/(16*115200)
   // disable access to divisor latch regs (enable access to xmit/rcv fifos
   // and int enable regs)
   U0LCR =0x3; //LCR_DISABLE_LATCH_ACCESS;
   // setup fifo control reg - trigger level 0 (1 byte fifos), no dma
   // disable fifos (450 mode) ���������� �� ������ 1-�� �����
   U0FCR =0x07; //1byte, clear FIFO
   // disable all UART0 interrupts
   U0IER = 1;
   // setup line control reg - disable break transmittion, even parity,
   // 1 stop bit, 8 bit chars

//   urLSR = U0LSR; //����������� ������
//   urLSR = U0RBR;
}
/****************************************************************************/
/****************************************************************************/
//static
void UART0Interrupt(void)
{
unsigned int adrbyteE,tt;

//   IO0SET_bit.P0_7 = 1; //�������

   FLAG.UART_RS = 1;
  //��������� �� ������� ������
  if (U0IIR == 6)
             {
               FLAG.UART_RS = 0;
               U0FCR =0x07; //1byte, clear FIFO
             }//if U0IIR

   while (!(U0LSR & 0x01))//�������� �������� ����?
      { //��� �����
        if (U0LSR && 0x80)
        {
         FLAG.UART_RS = 0;
         U0FCR =0x07; //1byte, clear FIFO
         break;
        }
      }
/***************************************/
  SoftTimerVal = 0; //������� ������ ������

  if (COMMAND[1] == 0x15)
  {
  adrbyteE = (COMMAND[5] << 8) | COMMAND[6];//������� ����� ������ ������
  tt = 9;
  if (adrbyteE == 0)
        tt += TimeByte;
  }
  if (FLAG.UART_RS)
   //  �� ���� ���� - ������  ���� � ������ ������� ��� � ������ ������
      if ((COMMAND[1] == 0x15) && (ByteCOMMAND == tt)&& (COMMAND[5] < 0x0C))
        { //�������� ������
          if (ByteCurrentIzm == 0) //1-e 4(2) ����� ����� �����??????? � �������
                  {
                  LiveTime[0] = COMMAND[9] ;
                  LiveTime[1] = COMMAND[10] ;
                  if (tt == 13)
                  {
                  LiveTime[2] = COMMAND[11] ;
                  LiveTime[3] = COMMAND[12] ;
                  }
                    else
                  {
                  LiveTime[2] = 0 ;
                  LiveTime[3] = 0 ;
                  }

                  }
          if (adrbyteE != 0)
                adrbyteE-=TimeByte;  //�� ������� ����� �������� �� ����� �������
          adrbyteE+= ByteCurrentIzm; //����� ���������� �����
          BUFFSPECTR[adrbyteE] = U0RBR ;
          if (adrbyteE < 3075)// ������ ������ ��������� ���������� - �� ����������
                      ByteCurrentIzm++;
        }//command 15
/*****************************/
      else
        if ((COMMAND[1] == 0x13) && (ByteCOMMAND == 7) && (COMMAND[4]<223))
        {//�������� ������� ������ ��� �������
          adrbyteE = COMMAND[4]+ByteCurrentIzm;
          CurrentDIzm[adrbyteE]= U0RBR;
          if (adrbyteE<226)//������ ���������� - �� ����������
                      ByteCurrentIzm++;
        }//command 13
      else
        if ((COMMAND[1] == 0x16) && (ByteCOMMAND == 4) && (COMMAND[2]==122))
        {//�������� ������� ������������� ���������
          if (ByteCurrentIzm == 0)
                  {//0-� ���� ��� �������� � ������ COMMAND
                  TABLTemperCORR[0] = COMMAND[3];
                  ByteCurrentIzm++;
                  }
          TABLTemperCORR[ByteCurrentIzm]= U0RBR;
          if (ByteCurrentIzm < 124)//������ ���������� - �� ����������
                      ByteCurrentIzm++;
              else
                ByteCurrentIzm = 124;
        }//command 16
/*****************************/
         else
          {
            COMMAND[ByteCOMMAND]= U0RBR;
            if (ByteCOMMAND<16)//������ ������� ���������� - �� ����������
                      ByteCOMMAND++;//������ ����� � ������� �������
             else
               ByteCOMMAND = 15;
          } //���������

//   IO0SET_bit.P0_7 = 0; //�������
}
/****************************************************************************/
