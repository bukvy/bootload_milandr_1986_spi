#ifndef INIT_MODULE_H_
#define INIT_MODULE_H_

#include "config.h"


#define MIN_EXCHANGE_PERIOD         10 //мс
extern volatile uint32_t exchange_period;

//#define USE_OLD_RWIO
#ifdef USE_OLD_RWIO
enum type_data
{
	PHYSICAL_VALUE = 0,
	ADC_VALUE = 1,
	PSEUDOPHYSICAL_VALUE = 2,
	VALUS_RANGE = 3,
	PACKED_BITS_DI = 4,
	PACKED_BITS_DI_COUNTER = 5,
	RESERVED_1 = 6,
	RESERVED_2 = 7,
	PHYSICAL_VALUE_FLAGS = 8,
	ADC_VALUE_FLAGS = 9,
	PSEUDOPHYSICAL_VALUE_FLAGS = 10,
	VALUS_RANGE_FLAGS = 11,
	PACKED_BITS_DI_FLAGS = 12,
	PACKED_BITS_DI_COUNTER_FLAGS = 13
};

enum number_channel
{
	CHANNELNUMBER_8 = 0,
	CHANNELNUMBER_12 = 1,
	CHANNELNUMBER_16 = 2,
	CHANNELNUMBER_24 = 3,
	CHANNELNUMBER_32 = 4
};
#else

enum {	//Коды количества каналов для передачи командой RW
	RW_NC_6 = 0,
	RW_NC_8 = 1,
	RW_NC_12 = 2,
	RW_NC_16 = 3,
	RW_NC_24 = 4,
	RW_NC_32 = 5,
	RW_NC_48 = 6,
	RW_NC_64 = 7
};

enum {	//Размеры (в байтах) набора флагов для передачи командой RW
	RW_TF_6 = 2,
	RW_TF_8 = 2,
	RW_TF_12 = 2,
	RW_TF_16 = 2,
	RW_TF_24 = 4,
	RW_TF_32 = 4,
	RW_TF_48 = 8,
	RW_TF_64 = 8
};

enum number_channel
{
	CHANNELNUMBER_6 = RW_NC_6,
	CHANNELNUMBER_8 = RW_NC_8,
	CHANNELNUMBER_12 = RW_NC_12,
	CHANNELNUMBER_16 = RW_NC_16,
	CHANNELNUMBER_24 = RW_NC_24,
	CHANNELNUMBER_32 = RW_NC_32,
	CHANNELNUMBER_48 = RW_NC_48,
	CHANNELNUMBER_64 = RW_NC_64,
};


enum type_data
{
	PHYSICAL_VALUE 			= 0x00,	//Значения в физических единицах
	ADC_VALUE 			= 0x01,	//Значения АЦП
	VALUES_RANGE 			= 0x02,	//Значения от 0..65535 для приведения к подразумеваемому  диапазону (диапазон определяется текущей настройкой канала)
	PACKED_BITS_DI 			= 0x03,	//Упакованные биты DI (16DI)
	PACKED_BITS_DI_COUNTER 		= 0x04,	//Упакованные биты DI + 2 счетчика (DIO16)

	RESERVED_1 			= 0x05,
	RESERVED_2 			= 0x06,
	RESERVED_3 			= 0x07,

	PHYSICAL_VALUE_FLAGS 		= 0x08,	//Значения в физических единицах + набор флагов
	ADC_VALUE_FLAGS 		= 0x09,	//Значения АЦП + набор флагов
	VALUES_RANGE_FLAGS 		= 0x0A,	//Значения от 0..65535 для приведения к подразумеваемому  диапазону (диапазон определяется текущей настройкой канала) + Nf наборов флагов
	PACKED_BITS_DI_FLAGS 		= 0x0B,	//Упакованные биты DI + набор флагов
	PACKED_BITS_DI_COUNTER_FLAGS    = 0x0C,	//Упакованные биты DI + 2 счетчика (DIO16) + Nf наборов флагов

	ONLY_FLAGS			= 0x0D,	//Наборов флагов
	INCREMENTAL_TEST                = 254
};
#endif


extern void module_init(void);
extern void module_deinit(void);


extern int port_get_value(uint8_t num, uint16_t *value) ;

extern void port_set_state(uint8_t num, enum ch_state);
extern enum ch_state port_get_state(uint8_t num);


extern void port_set_mode(uint8_t num, enum ch_mode);
extern enum ch_mode port_get_mode(uint8_t num);



extern void module_apply_block_state(void);

extern void port_calibration(uint8_t num);

extern void port_reset_calibration(uint8_t num);
extern uint16_t g_ai_buf[8];

void aio_timer_u_enable(void);
static void aio_timer_u_disable(void);
static void aio_timer_u_init(uint32_t delay);

/* for void aio_pp_reg_task(uint_32 initial_data); */
struct aio_pp_reg {
	uint8_t num;
	uint32_t delay;
	uint32_t target_u;
	uint16_t k_prop;
	uint16_t k_int;
	uint16_t k_diff;
};

enum mod_status {
	STATUS_NORMAL,
	STATUS_OR,
	STATUS_UR,
	STATUS_WB
};

extern void port_get_status(uint8_t num, enum mod_status *);

typedef struct {
	volatile uint8_t ch_onoff[PORTS_COUNT];
	volatile uint8_t ch_mode[PORTS_COUNT];
	volatile uint8_t ch_onoff_mask[PORTS_COUNT];
	volatile uint8_t ch_mode_mask[PORTS_COUNT];
	volatile uint32_t block_state_enter;
	volatile uint16_t config_change;	// 1 - сбросить, 2 - сохранить, 4 - приминить
	volatile uint16_t want_anything;
} mod_wanted_t;

extern mod_wanted_t mod_wanted;

extern volatile int reset_on;


void module_wanted_parse(void);
void port_queue_set_state(uint8_t adc_index, uint8_t adc_state);
void port_queue_set_mode(uint8_t adc_index, uint8_t adc_mode);

#define CONFIG_ERACE	1
#define CONFIG_SAVE		2
#define CONFIG_PARCE	4
void queue_config_change(uint8_t config_change);

void cfg_parce_config();
void queue_parce_configuration(uint8_t *conf, uint16_t cnt);


#define DBG_STATISTIC_1		0x00000001
#define DBG_STATISTIC_2		0x00000002
#define DBG_LOAD_THREADS	0x00000004
#define DBG_FUNCTIONS		0x00000008
#define DBG_USER		0x10000000

extern uint32_t g_debug_flags;
extern uint32_t g_debug_channel;

extern uint32_t g_mod_buf_raw	[PORTS_COUNT];
extern uint16_t g_mod_ch_period	[PORTS_COUNT];
extern uint16_t   g_mod_buf_val	[PORTS_COUNT];

extern uint8_t data_type; //значение по умолчанию


extern volatile uint8_t bus_show_state;

void main_refresh_task(void);


#define u32 uint32_t
typedef u32 tick;

uint32_t GetTickCount(void);
tick setTimeout(u32 milliseconds);
int  isTimeout(tick timeout);

uint32_t get_ticks(void);



#endif /* INIT_MODULE_H_ */
