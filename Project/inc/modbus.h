#ifndef _modbus_h
#define _modbus_h



#define MB_BUF_SIZE 256


#define MODBUS_ADDRESS 1
#define MODBUS_PACKET_DETECT_TIME 20L


//структура регистра критических операция модбаса
typedef struct
{
	unsigned short WRIGHT_HR: 1; //непосредственная запись значений в HR выешстоящим контроллером
	unsigned short unused: 15;
}T_MB_CRITICAL_OPERATION;


extern unsigned long last_mb_time;
extern T_MB_CRITICAL_OPERATION MB_CRITICAL_OPERATION; //переменная для обозначения критических операций по модбасу




#ifdef __cplusplus
extern "C" unsigned short ModBusCRC16(unsigned char* p, unsigned short n);
extern "C" unsigned char ModBusLRC(unsigned char *p, unsigned short n);
#else
unsigned short ModBusCRC16(unsigned char* p, unsigned short n);
unsigned char ModBusLRC(unsigned char *p, unsigned short n);
#endif

enum CMB_Functions {	
	mbf_Read_Coils				= 1,
	mbf_Read_Discrete_Inputs	= 2,
	mbf_Read_Holding_Registers	= 3,
	mbf_Read_Input_Register		= 4,
	mbf_Write_Single_Coil		= 5,
	mbf_Write_Single_Register	= 6,
	mbf_Read_Exception_Status	= 7,
	mbf_Diagnostic				= 8,
	mbf_Get_Com_Event_Counter	= 11,
	mbf_Get_Com_Event_Log		= 12,
	mbf_Write_Multiple_Coils	= 15,
	mbf_Write_Multiple_Registers= 16,
	mbf_Report_Slave_ID			= 17,
	mbf_Mask_Write_Register		= 22,
	mbf_Read_File_Record		= 20,
	mbf_Write_File_Record		= 21,
	mbf_Read_Write_Multiple_Registers= 23,
	mbf_Read_FIFO_Queue			= 24,
	mbf_Read_Device_Identification= 43,
	mbf_Encapsulated_Interface_Transport= 43

};

enum CMB_DiagSubFunctions {	
	mbf_Diag_Echo				= 0,
	mbf_Diag_Restart			= 1,
	mbf_Diag_Diag_Register		= 2,
	mbf_Diag_ASCII_Delimiter	= 3,
	mbf_Diag_Force_Listen_Only	= 4,
	mbf_Diag_Clear_Counter		= 10,
	mbf_Diag_Bus_Message_Cnt	= 11,
	mbf_Diag_Bus_Err_Cnt		= 12,
	mbf_Diag_Bus_Exception_Cnt	= 13,
	mbf_Diag_Slave_Cnt			= 14,
	mbf_Diag_Slave_Noresponce_Cnt = 15,
	mbf_Diag_Slave_NAK_Cnt		= 16,
	mbf_Diag_Slave_Busy_Cnt		= 17,
	mbf_Diag_Bus_CharOverrun_Cnt = 18,
	mbf_Diag_Bus_Clear_Overrun  = 20
};

enum CMB_Exception_Codes {	
	mbexc_ILLEGAL_FUNCTION		= 0x01,
	mbexc_ILLEGAL_DATA_ADDRESS	= 0x02,
	mbexc_ILLEGAL_DATA_VALUE	= 0x03,
	mbexc_SLAVE_DEVICE_FAILURE	= 0x04,
	mbexc_ACKNOWLEDGE			= 0x05,
	mbexc_SLAVE_DEVICE_BUSY		= 0x06,
	mbexc_MEMORY_PARITY_ERROR	= 0x08,
	mbexc_GATEWAY_PATH_UNAVAILABLE= 0x0A,
	mbexc_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND= 0x0B
};









#ifndef ULONG
#define ULONG unsigned long
#endif

#ifndef USHORT
#define USHORT unsigned short
#endif

#ifndef SHORT
#define SHORT short
#endif

#ifndef UCHAR
#define UCHAR unsigned char
#endif


#ifndef DWORD
#define DWORD unsigned long
#endif



#define HR_FLAG_SAVE_CONFIG		0x0001
#define HR_FLAG_SAVE_SERIAL	        0x0008

#define HR_FLAG_ERACE_CONFIG		0x0010
#define HR_FLAG_RESET                   0x0080

#define HR_FLAG_TEST_LEDS		0x0100
#define HR_FLAG_TEST_42                 0x0200
#define HR_FLAG_TEST_47                 0x0400

#define HR_FLAG_APPLY_ONLY		0x1000
#define HR_FLAG_EEPROM_TEST             0x0800







#ifndef HIBYTE
#define HIBYTE(arg) ( (unsigned char)(((arg)>>8)&0xFF) )
#define LOBYTE(arg) ( (unsigned char)((arg)&0xFF) )
#endif

#define MBF_INPREG_CNT  (sizeof(cfg_board_settings)/2+1)
#define MBF_HOLDREG_CNT (sizeof(cfg_board_settings)/2+1)

#ifdef MDI_EN
#define MBF_DI_CNT (sizeof(TDI_CPU_CH)*8)
#endif
#ifdef MCO_EN
#define MBF_COIL_CNT (sizeof(TCO_CPU_CH)*8)
#endif
//#define MBF_INPREG_CNT	18		//количество доступных входных регистров
//#define MBF_HOLDREG_CNT	4		//количество доступных хранимых регистров
//#define MBF_DI_CNT		16		//количество доступных дискретных входов
//#define MBF_COIL_CNT	16		//количество доступных флагов

#ifdef __cplusplus
extern "C" void fmb_OnCharacterOverrrun(void); // потери байт во время приема
extern "C" void fmb_OnReceiveError(void);		// ошибка в пакете
extern "C" void fmb_OnCompleteMBPacket(void);	//по завершению приема модбас-пакета,
extern "C" void ProcessModbus(void);
extern "C" unsigned char g_ModbusBuf[MB_BUF_SIZE];
extern "C" unsigned short g_wModbusCnt;
extern "C" unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
extern "C" unsigned short* gmbf_InputRegisters;// в main установить указатель
extern "C" unsigned short* gmbf_HoldingRegisters;// в main установить указатель
extern "C" unsigned char* gmbf_DIs;// в main установить указатель
extern "C" unsigned char* gmbf_Coils;// в main установить указатель
//extern "C" unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus


#else
void fmb_OnCharacterOverrrun(void); // потери байт во время приема
void fmb_OnReceiveError(void);		// ошибка в пакете
void fmb_OnCompleteMBPacket(void);	//по завершению приема модбас-пакета,
void ProcessModbus(void);
extern unsigned char g_ModbusBuf[MB_BUF_SIZE];
extern unsigned short g_wModbusCnt;
extern unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
extern unsigned short* gmbf_InputRegisters;// в main установить указатель
extern unsigned short* gmbf_HoldingRegisters;// в main установить указатель
extern unsigned char* gmbf_DIs;// в main установить указатель
extern unsigned char* gmbf_Coils;// в main установить указатель

#endif


#endif
