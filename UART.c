/****************************************************************************/
void InitialiseUART0(unsigned char speed)
{
  int SpeedRS232;
  if (speed == 1)
    SpeedRS232 = 115200;   //скорость обмена
  else //2
    SpeedRS232 = 19200;    //скорость обмена




   PINSEL0 = (PINSEL0 & 0xFFFFFFF0) |0x05; //UART0_PCB_PINSEL_CFG;
   VICVectCntl0  = 0x26;    			   	  /* vector UART0 as 2nd highest priority and enabled */
   VICVectAddr0  = (int) UART0Interrupt;	   	  /* vector UART0 interrupt address */


   // enable access to divisor latch regs
   U0IER = 0;
   U0LCR = 0x83;        //LCR_ENABLE_LATCH_ACCESS;
   // set divisor for desired baud
   U0DLM = 0x00;
   U0DLL = ((PCLK_VPB*1000) / (SpeedRS232*16)); //0x2;  //115200 ->8/4 //надо 0x0A/4->18432000/(16*115200)
   // disable access to divisor latch regs (enable access to xmit/rcv fifos
   // and int enable regs)
   U0LCR =0x3; //LCR_DISABLE_LATCH_ACCESS;
   // setup fifo control reg - trigger level 0 (1 byte fifos), no dma
   // disable fifos (450 mode) прерывание по приему 1-го байта

   //U0FCR =0x07; //1byte, clear FIFO
   U0FCR_bit.FCRFE=1;
   U0FCR_bit.RFR=1;
   U0FCR_bit.TFR=1;
   U0FCR_bit.RTLS=0;

   // disable all UART0 interrupts
   U0IER = 1;
   // setup line control reg - disable break transmittion, even parity,
   // 1 stop bit, 8 bit chars

//   urLSR = U0LSR; //КОНТРОЛЬНОЕ ЧТЕНИЕ
//   urLSR = U0RBR;
}