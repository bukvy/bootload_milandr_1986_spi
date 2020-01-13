#include "config.h"
#include "flash.h"
#include <string.h>
#include "i2c.h"
#include "Serial.h"

#define mask 0x1021

unsigned char Store_EE(void)    /// ���������� �������� ���������� � ���������
{
    return StoreField(EE_ADR,(unsigned char*) &board_set.settings, sizeof(cfg_params));
}

unsigned char ReStore_EE(void)  /// �������������� �������� ���������� � ���������
{
    if (ReStoreField(EE_ADR,(unsigned char*) &board_set.settings, sizeof(cfg_params)))
    {
        return 1;
    }
    else
    {
          memset((unsigned char*) &board_set.settings, 0, sizeof(cfg_params));
    }
    return 0;
}

// ��������� ������� � ���������� �� � ����� (ret 1=OK)
unsigned char StoreField(unsigned long adr, unsigned char *buf, unsigned long lng) {
	unsigned char rez, rep;
	unsigned short crc;
	// ���������� ��
	crc = CRC16(&(buf[2]), lng - 2);
	buf[0] = (crc&0x00ff);
	buf[1] = ((crc>>8)&0x00ff);
	// ������
	rez = 0;
	rep = 0;
	while ((rez == 0) && (rep < 5)) {
		rep++;
		if  (WriteFlashBuf(adr,buf, lng)) {
			rez = 1;
		}
	}
	return rez;
}


// ��������� ������� � ���������� �� � ����� (ret 1=OK)
unsigned char ReStoreField(unsigned long adr, unsigned char *buf, unsigned long lng) {
	unsigned char rep;
	uint16_t crc;
	// ������
	rep = 0;
	while (rep < 5) {
		rep++;
		if (ReadFlashBuf(adr, buf, lng, 0)) {
			crc = buf[0] + (buf[1] << 8);
			if (CRC16(&(buf[2]), lng-2) == crc) {return 1;}
		}
	}
	return 0;
}
