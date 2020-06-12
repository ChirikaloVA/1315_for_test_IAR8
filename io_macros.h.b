/**************************************************
 *
 * Common macro declarations used for peripheral I/O
 * declarations for AARM and ICCARM.
 *
 * Copyright 1999 IAR Systems. All rights reserved.
 *
 * $Revision: 1.5 $
 *
 * $Log: io_macros.h,v $
 * Revision 1.5  2002/06/14 13:11:28  owi
 * *** empty log message ***
 *
 * Revision 1.5  2002/06/14 13:11:28Z  owi
 * Revision 1.4  2000/06/16 15:35:17Z  xcamilla
 * Revision 1.3  2000/06/15 15:29:08  xcamilla
 * Revision 1.2  2000/06/15 12:58:29  xcamilla
 * Revision 1.1  1999/11/11 14:06:45  apikas
 * Initial revision
 *
 **************************************************/

#ifndef __IO_MACROS_H
#define __IO_MACROS_H
/********************************/
/* мои макросы */
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
  unsigned ErrKoef     :1;
  unsigned minmaxTemper :1;
  unsigned EEPROMSP     :1;
} __BITFLAG;
///////////////////////////////


/***********************************************
 *       C  specific macros
 ***********************************************/

#ifdef __IAR_SYSTEMS_ICC__

#pragma language=extended

/* SFR sizes */
#define __REG8 unsigned char
#define __REG16 unsigned short
#define __REG32 unsigned long


/***********************************************
 * I/O reg attributes
 ***********************************************/
#define __READ_WRITE
#ifdef __cplusplus
#define __READ          /* Not supported */
#else
#define __READ          const
#endif
#define __WRITE         /* Not supported */

/***********************************************
 * I/O reg bits (default names)
 ***********************************************/
typedef struct
  {
    unsigned char no0:1;
    unsigned char no1:1;
    unsigned char no2:1;
    unsigned char no3:1;
    unsigned char no4:1;
    unsigned char no5:1;
    unsigned char no6:1;
    unsigned char no7:1;
  } __BITS8;

typedef struct
  {
    unsigned short no0:1;
    unsigned short no1:1;
    unsigned short no2:1;
    unsigned short no3:1;
    unsigned short no4:1;
    unsigned short no5:1;
    unsigned short no6:1;
    unsigned short no7:1;
    unsigned short no8:1;
    unsigned short no9:1;
    unsigned short no10:1;
    unsigned short no11:1;
    unsigned short no12:1;
    unsigned short no13:1;
    unsigned short no14:1;
    unsigned short no15:1;
  } __BITS16;

typedef struct
  {
    unsigned long no0:1;
    unsigned long no1:1;
    unsigned long no2:1;
    unsigned long no3:1;
    unsigned long no4:1;
    unsigned long no5:1;
    unsigned long no6:1;
    unsigned long no7:1;
    unsigned long no8:1;
    unsigned long no9:1;
    unsigned long no10:1;
    unsigned long no11:1;
    unsigned long no12:1;
    unsigned long no13:1;
    unsigned long no14:1;
    unsigned long no15:1;
    unsigned long no16:1;
    unsigned long no17:1;
    unsigned long no18:1;
    unsigned long no19:1;
    unsigned long no20:1;
    unsigned long no21:1;
    unsigned long no22:1;
    unsigned long no23:1;
    unsigned long no24:1;
    unsigned long no25:1;
    unsigned long no26:1;
    unsigned long no27:1;
    unsigned long no28:1;
    unsigned long no29:1;
    unsigned long no30:1;
    unsigned long no31:1;
  } __BITS32;

/***********************************************
 * Define NAME as an I/O reg
 * Access of 8/16/32 bit reg:  NAME
 ***********************************************/
#define __IO_REG8(NAME, ADDRESS, ATTRIBUTE)              \
                   volatile __no_init ATTRIBUTE unsigned char NAME @ ADDRESS;

#define __IO_REG16(NAME, ADDRESS, ATTRIBUTE)             \
                   volatile __no_init ATTRIBUTE unsigned short NAME @ ADDRESS;

#define __IO_REG32(NAME, ADDRESS, ATTRIBUTE)             \
                   volatile __no_init ATTRIBUTE unsigned long NAME @ ADDRESS;

/***********************************************
 * Define NAME as an I/O reg
 * Access of 8/16/32 bit reg:  NAME
 * Access of bit(s):           NAME_bit.noX  (X=1-31)
 ***********************************************/
#define __IO_REG8_BIT(NAME, ADDRESS, ATTRIBUTE, BIT_STRUCT)\
                       volatile __no_init ATTRIBUTE union \
                        {                                 \
                          unsigned char NAME;             \
                          BIT_STRUCT NAME ## _bit;      \
                        } @ ADDRESS;

#define __IO_REG16_BIT(NAME, ADDRESS, ATTRIBUTE,BIT_STRUCT)\
                        volatile __no_init ATTRIBUTE union \
                         {                                 \
                           unsigned short NAME;            \
                           BIT_STRUCT NAME ## _bit;      \
                         } @ ADDRESS;

#define __IO_REG32_BIT(NAME, ADDRESS, ATTRIBUTE, BIT_STRUCT)\
                        volatile __no_init ATTRIBUTE union \
                         {                                 \
                           unsigned long NAME;             \
                           BIT_STRUCT NAME ## _bit;      \
                         } @ ADDRESS;

#endif /* __IAR_SYSTEMS_ICC__ */


/***********************************************
 *      Assembler specific macros
 ***********************************************/

#ifdef __IAR_SYSTEMS_ASM__

/***********************************************
 * I/O reg attributes (ignored)
 ***********************************************/
#define __READ_WRITE 0
#define __READ 0
#define __WRITE 0

/***********************************************
 * Define NAME as an I/O reg
 ***********************************************/
#define __IO_REG8(NAME, ADDRESS, ATTRIBUTE)      \
                  NAME DEFINE ADDRESS

#define __IO_REG16(NAME, ADDRESS, ATTRIBUTE)     \
                   NAME DEFINE ADDRESS

#define __IO_REG32(NAME, ADDRESS, ATTRIBUTE)     \
                   NAME DEFINE ADDRESS

/***********************************************
 * Define NAME as an I/O reg
 ***********************************************/
#define __IO_REG8_BIT(NAME, ADDRESS, ATTRIBUTE, BIT_STRUCT)  \
                      NAME DEFINE ADDRESS

#define __IO_REG16_BIT(NAME, ADDRESS, ATTRIBUTE, BIT_STRUCT) \
                       NAME DEFINE ADDRESS

#define __IO_REG32_BIT(NAME, ADDRESS, ATTRIBUTE, BIT_STRUCT) \
                       NAME DEFINE ADDRESS

#endif /* __IAR_SYSTEMS_ASM__ */

#endif /* __IO_MACROS_H */
