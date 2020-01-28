#include "config.h"
extern void testfunc(void);
typedef struct _bootloader_config {
	uint16_t crc;
//	uint16_t timeout;    Remove timeout as never saw where it is used   Room420
	uint16_t autoboot;

} bootloader_config, *p_bootloader_config;

#define NewStackAddr        (*(volatile unsigned long *) 0x08006000)


 #define xyzModem_CHAR_TIMEOUT            2000	/* 2 seconds */
#define xyzModem_MAX_RETRIES             20
#define xyzModem_MAX_RETRIES_WITH_CRC    10
#define xyzModem_CAN_COUNT                3	/* Wait for 3 CAN before quitting */
#define XMODEM_BUF_SIZE 4096

extern bootloader_config  bldr_config;
extern int Program_block(uint32_t address,uint32_t* buffer,uint8_t length);
extern int Erase(uint32_t programadr);
extern int global_retries;
extern int Erase_many_blocks(void);

