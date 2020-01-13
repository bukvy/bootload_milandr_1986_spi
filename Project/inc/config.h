#ifndef CONFIG_H_
#define CONFIG_H_

#include "MDR32F9Qx_it.h"


//Room420  04.10.2019  was 3 
#define Led_group 18

#define PORTS_COUNT 12
#define PORT_DI_COUNT	0
#define PORT_DO_COUNT	0
#define PORT_AO_COUNT	12
#define PORT_AI_COUNT	8
#define PORT_SSW_COUNT	0
#define PORT_HART_COUNT	0
#define GROUPS_COUNT	2
#define GROUP_SIZE 8

#define SERIAL_NUM_COUNT 4
#define MAX_CFG_CHANNELS PORT_AO_COUNT

#define LEDS_PERIOD_SEC 1

#define AIAO8_Type 0x11
#define AI16_Type 0x12
#define DI10DO8AI4AO2_Type 0x13
#define DI10DO8AI4AO2H_Type 0x14
#define DI10DO8SSW5_Type 0x15
#define DI22DO8_Type 0x16





#define LEDS_PERIOD_SEC 1

#define LED_RSK_L   PORT_Pin_4
#define LED_OE_H    PORT_Pin_5
#define LED_SCLR_H  PORT_Pin_6
#define LED_TEST    PORT_Pin_7

//Room420  24.09.2019  Not used here
#define LED_PWR_H   PORT_Pin_3

#define LED_PORTF   (LED_RSK_L | LED_OE_H| LED_SCLR_H)
#define LED_PORTD   (LED_TEST | LED_PWR_H)

enum ch_state {
	CH_STATE_OFF,
	CH_STATE_ON
};

enum ssp_target {
	tNONE	= 0,	//0
	tDI 	= 1,	//1
	tDO		= 2,
	tLEDS 	  = 3
};
  
enum ssp_mode {
	tRECV	= 0,	//0
	tTRANS 	= 1,	//1
};

enum ch_mode {
	CH_MODE_DI_DC_2MA       = 0x00,   //+ цифровой вход типа "сухой контакт" с выдачей 2 мА
	CH_MODE_DI_DC_10MA      = 0x01,   //+ цифровой вход типа "сухой контакт" с выдачей 10 мА
	CH_MODE_DI_IEC 	        = 0x02,   //+ цифровой вход в режиме "потенциальный вход"

        CH_MODE_DI_IC_2MA       = 0x03,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 2 мА в импульсном режиме
        CH_MODE_DI_IC_10MA      = 0x04,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме
        
        CH_MODE_DI_DC_10MA_CC   = 0x05,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 10 мА c контролем обрыва цепи
        CH_MODE_DI_IC_10MA_CC   = 0x06,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме с контролем обрыва цепи
        CH_MODE_DI_IEC_CC       = 0x07,   //+ Цифровой вход в режиме «потенциальный вход» с контролем обрыва цепи
        
        CH_MODE_DI_IEC_CC_KZ    = 0x0A,   //+ Цифровой вход в режиме «потенциальный вход» с контролем обрыва цепи и короткого замыкания

        CH_MODE_DI_DC_10MA_CC_KZ= 0x08,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 10 мА c контролем обрыва цепи и короткого замыкания
        CH_MODE_DI_IC_10MA_CC_KZ= 0x09,   //+ Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме с контролем обрыва цепи и короткого замыкания
};

#define NEED_ON_10MA(TYPE_ID, PORT_POWER)   \
         ((PORT_POWER) &&       \
          ((TYPE_ID == CH_MODE_DI_DC_10MA) ||           \
           (TYPE_ID == CH_MODE_DI_IC_10MA) ||           \
           (TYPE_ID == CH_MODE_DI_DC_10MA_CC) ||        \
           (TYPE_ID == CH_MODE_DI_IC_10MA_CC)  ||       \
           (TYPE_ID == CH_MODE_DI_DC_10MA_CC_KZ) ||     \
           (TYPE_ID == CH_MODE_DI_IC_10MA_CC_KZ)) \
            )


enum ch_status {
	CH_NORMAL	= 0,	//0
	CH_ALARM 	= 1,	//1
	CH_KZ		= 2,
	CH_OBRYV 	= 3,
        CH_FAIL 	= 5,
        
};



#define u8      unsigned char
#pragma pack(push, 1)

typedef struct {
    unsigned short flag_save;
    unsigned short flag_diag;
    unsigned short flag_error;
    unsigned short flag_clear_cntrs;
} cfg_flags;


//================   from AIO8=================================




enum ssp1_target {
	Read_Status	= 0,	//0
	Read_DO 	= 1,	//1
	Write_DO		= 2,
	Read_ADC		= 3
};

enum ssp_slave {
	slaveNone	= 0,	//0
	slaveAI1 	= 1,	//1
	slaveAI2	= 2,
	slaveAO		= 3
};






enum ch_stat {
CHS_NORMAL		  = 0, 
CHS_KZ		      = 1,
CHS_OBRYV 	    = 2,
CHS_PEREGRUZ 	  = 3,
CHS_Ready 	    = 4,
CHS_Link_err 	  = 5
};


    
#pragma pack(push, 1)
typedef struct
{
  float  a,b;
} CDOT;
#pragma pack(pop)




#pragma pack(1)
// ==========================================================================
// ------------------------------------ ::HR:: ------------------------------
typedef struct TConfigModul
{
  uint8_t  Type;            // Тип модулю
  uint8_t  hver;            // Размер динамической базы
  uint8_t  DI;              // кол-во DI
  uint8_t  DO;              // кол-во DO
  uint8_t  AI;              // кол-во AI
  uint8_t  AO;              // кол-во AO
}  CConfigModul;
#pragma pack()

#pragma pack(push, 1)
typedef struct
{
	uint16_t config1:1;        // Значение АЦП 
	uint16_t config2:1;        // Значение АЦП 
	uint16_t info:1;        // Значение АЦП 
  
	uint16_t cdi:3;        // Значение АЦП 
	uint16_t cdo:1;        // Значение АЦП 
  uint16_t unused:9;
  unsigned short radr:12;
  unsigned short rcmd:4;
  unsigned short size_answer;
} CCAN;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
	unsigned short  acp[4]; // Значение АЦП 
  float DO_Upit;          //Питание канала
  float DO_U[2];          //Напряжение канала в В/мВ
  float  DO_Rn;           //Сопротивление канала
  float  DO_In;           //Ток канала
} CDO_meas;
#pragma pack(pop)
#pragma pack(1)
// ==========================================================================
// ------------------------------------ ::HR:: ------------------------------
typedef struct TDIDOState
{
  uint8_t  mode:3;            // Тип модулю
  uint8_t  value:2;            // Размер динамической базы
  uint8_t  status:3;              // кол-во DI
}CDIDOState;
#pragma pack()

#pragma pack(1)
typedef struct TAIAOState
{
  uint32_t  mode:3;            // Тип модулю
  uint32_t  value:16;          // Размер динамической базы
  uint32_t  status:3;          // кол-во DI
  uint32_t  unused:10;         // кол-во DI
}CAIAOState;
#pragma pack()


#pragma pack(push, 1)
  typedef struct {
	uint16_t result;
	uint16_t process;
} memtest_info_t;

// ------------------------------------ ::HR:: ------------------------------

  typedef struct {
	unsigned short  ena:1; // Тест Led
	unsigned short  LED:1; // Тест Led
        unsigned short  eDI:1; // 
        unsigned short  eDO:1; // 
        unsigned short  eAI:1; // 
        unsigned short  tCAN:1; // 
        unsigned short  Fram:1; // 
        unsigned short  Room420Test:9; // Тест  Room420 developer for my own tests during developing

} test_str;

  typedef struct {
  unsigned short  CAN1_link:1; // 
  unsigned short  CAN2_link:1; // 
  unsigned short  Led_button:1; //
  unsigned short    is_leds_on:1;
  unsigned short    is_leds_off:1;
  unsigned short  unused:11;

} leds_str;


typedef struct

{
  unsigned short  kalib; // Тест Led
  test_str test;
  leds_str  leds;
  memtest_info_t memtest_info;
} CDiag;

//=========================  end of from AIO8


typedef struct
{
  unsigned short  crc;  // That is crc field for EEPROM 
  unsigned long fver;         // Версия ПО
  unsigned short addr;         // Адрес CAN/Modbus
  CConfigModul config;
  unsigned long zavnum;       // Зав. номер
  
  unsigned char flag_in;
  unsigned char flag_out;
  
  unsigned long restart_time; // Время с момента рестарта
  unsigned short mode;         // Режим Инициация/Работа
  short  temperature;


  unsigned char ao_status[PORT_AO_COUNT];
  unsigned char ao_mode[PORT_AO_COUNT];

  float ao_value_u[PORT_AO_COUNT];
  float ao_value_i[PORT_AO_COUNT];

  unsigned short ao_value_dac_u[PORT_AO_COUNT];
  unsigned short ao_value_dac_i[PORT_AO_COUNT];

  
  CDOT kal_ao_i[PORT_AO_COUNT];
  CDOT kal_ao_u[PORT_AO_COUNT];
  
  
  unsigned short KalibCH;  //DO_SelectC
  unsigned short CheckCnt; 
  unsigned short LostCnt;   

      
  CDiag diag;
  
  
//unsigned short          Fram_test_result;  inside diag
//unsigned short          Fram_test_process; inside diag

uint8_t         leds_timeout;
uint8_t         user_led_type;

unsigned char   port_block_state[MAX_CFG_CHANNELS]; //Room420  28.12.2019 add as in di48
                                                    

unsigned char   port_block_mode[MAX_CFG_CHANNELS]; //Room420  28.12.2019  add as in di48
                                                   
unsigned char   serial[SERIAL_NUM_COUNT];   //Room420  28.12.2019  add as in di48


unsigned short  save_cfg;
 
} cfg_params;


typedef struct {
    unsigned char   value[MAX_CFG_CHANNELS];  
    short           temperature;
    short           eeprom_test_cnt;
    unsigned long   eeprom_result;
} cfg_values;


typedef struct {
  cfg_params    settings;
  cfg_values    values;
  cfg_flags     flags;
} cfg_board_settings;

#pragma pack(pop)


extern cfg_board_settings board_set;

extern uint8_t DstBuf2[Led_group];
extern uint16_t DstBuf1[100];
extern unsigned char ssp2_cnt,ssp1_cnt;

extern unsigned char SSP_Target;
extern DMA_ChannelInitTypeDef DMA_InitSSP1,DMA_InitSSP2,DMA_InitADC1;
extern DMA_CtrlDataInitTypeDef DMA_PriSSP1,DMA_PriSSP2,DMA_PriADC1;
extern DMA_CtrlDataInitTypeDef DMA_AltADC1;

void init_SPI_Mode(unsigned short mode);
void SendUart2_DMA(unsigned char *ptr,unsigned char cnt);
void InitDMA_SSP1(void);
void InitDMA_SSP1_tx(unsigned char *buf,unsigned char cnt);
void InitDMA_SSP2(unsigned char *buf,unsigned char cnt);
void System_Init(void);
void ReInitDMA(void);
#endif /* CONFIG_H_ */
