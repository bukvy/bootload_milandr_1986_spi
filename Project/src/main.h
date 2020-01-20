extern void testfunc(void);
typedef struct _bootloader_config {
	uint16_t crc;
//	uint16_t timeout;    Remove timeout as never saw where it is used   Room420
	uint16_t autoboot;

} bootloader_config;



extern bootloader_config  bldr_config;

