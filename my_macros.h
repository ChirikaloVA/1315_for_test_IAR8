/********************************/
/* мои макросы */
/*#define TARGET_BOARD_FREQ 18432
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
} __BITFLAG;
///////////////////////////////
*/

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

//#define WDT_value 9216000    //0.5 s
#define WDT_value 5526000      // 0.3 s
///////////////////////////////
