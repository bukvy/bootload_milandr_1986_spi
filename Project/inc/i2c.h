#ifndef __i2c__h__
#define __i2c__h__

typedef struct
{
  unsigned char cnt;
  unsigned char ptr;
  unsigned char mode;
  unsigned char *buf;
} Ci2cBuf;

uint8_t ReadFlashBuf(uint32_t adr, uint8_t *buf, uint32_t lng,uint8_t v);
uint8_t WriteFlashBuf(uint32_t adr, uint8_t *buf, uint32_t lng);
void GetI2cTemperatue(void);
void Init_i2c(void);
void Init_ClkDiv(int clk_div);
void I2C_StartTransmission(uint8_t transmissionDirection,  uint8_t slaveAddress,uint8_t cnt,  uint8_t stop);

#endif
