


void i2c_lpc_init(void);
void LSM330DLC_init(void);
void ReadGyroscope_test(char * buf, unsigned short Adr);
void WriteGyroscope_test(unsigned char byte, unsigned short Adr);
void PRS_sens_init(void);
void PRS_sens_read(void);

char TestSection(char i);