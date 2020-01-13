#ifndef __FLASH__H__
#define __FLASH__H__

#define EE_ADR 0


uint8_t Store_EE(void);
uint8_t ReStore_EE(void);

unsigned char StoreField(unsigned long adr, unsigned char *buf, unsigned long lng);
unsigned char ReStoreField(unsigned long adr, unsigned char *buf, unsigned long lng);

#endif
