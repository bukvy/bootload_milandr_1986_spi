
#include "fp.h"

#include "ferret_proto.h"
#include "spi_ferret_proto.h"

#include "module.h"
#include "leds.h"
#include "config.h"
#include <MDR32Fx.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_uart.h>
#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_dma.h"
#include "MDR32F9Qx_it.h"
#include "modbus.h"

#include "block.h"


#define DI48_COMMAND_BASE_NUM         0x80
#define CMD_BASE                      DI48_COMMAND_BASE_NUM

#define CMD_PORT_SET_STATE             CMD_BASE+0x00	//+
#define CMD_PORT_GET_STATE             CMD_BASE+0x01	//+
#define CMD_PORT_SET_MODE              CMD_BASE+0x02	//+
#define CMD_PORT_GET_MODE              CMD_BASE+0x03	//+

#define CMD_PORT_GET_VALUE             CMD_BASE+0x04	//+

//
#define CMD_PORT_SET_BLOCK_STATE       CMD_BASE+0x05	//+
#define CMD_PORT_GET_BLOCK_STATE       CMD_BASE+0x06	//+
#define CMD_PORT_APPLY_BLOCK_STATE     CMD_BASE+0x07	//???
#define CMD_PORT_GET_STATUS            CMD_BASE+0x08	//+


#define CMD_LEDS_ALL_ON                CMD_BASE+0x09	//+
#define CMD_LEDS_ALL_OFF               CMD_BASE+0x0A 	//+
#define CMD_LEDS_GET_STATE             CMD_BASE+0x0B	//+
#define CMD_LEDS_SET_TIMEOUT           CMD_BASE+0x0C 	//+
#define CMD_LEDS_GET_TIMEOUT           CMD_BASE+0x0D	//+

#define CMD_USER_LED_SET               CMD_BASE+0x0E 	//+
#define CMD_USER_LED_GET               CMD_BASE+0x0F 	//+

#define CMD_PM_SET_RTC_ALARM           CMD_BASE+0x10	//+
#define CMD_PM_GET_RTC_ALARM           CMD_BASE+0x11	//+
#define CMD_PM_STOP                    CMD_BASE+0x12	//+
#define CMD_PM_WAIT                    CMD_BASE+0x13	//+
#define CMD_PM_RUN                     CMD_BASE+0x14	//+

#define CMD_I2C_GET_TEMP               CMD_BASE+0x15	//+

#define CMD_CPLD_FLASH_PREPEARE        CMD_BASE+0x16 	//
#define CMD_CPLD_FLASH_BLOCK	       CMD_BASE+0x17	//
#define CMD_CPLD_FLASH_BLOCK_STATUS    CMD_BASE+0x18	//
#define CMD_CPLD_FLASH_RUN			   CMD_BASE+0x19	//
#define CMD_CPLD_FLASH_RUN_STATUS	   CMD_BASE+0x1A	//

#define CMD_EEPROM_SAVE_CONFIG         CMD_BASE+0x1B	//+
#define CMD_EEPROM_RESET_CONFIG        CMD_BASE+0x1C	//+

#define CMD_SET_BLOCK_STATE		       CMD_BASE+0x1D	//+
#define CMD_GET_BLOCK_STATE		       CMD_BASE+0x1E	//+

#define CMD_ALARM_SET                  CMD_BASE+0x1F	//+
#define CMD_ALARM_GET                  CMD_BASE+0x20	//+

#define CMD_RW_VALUES		       	   CMD_BASE+0x21


#define CMD_PORT_GET_COUNTER           CMD_BASE+0x22
#define CMD_PORT_GET_IS_COUNTER        CMD_BASE+0x23    //??
#define CMD_PORT_RESET_COUNTER         CMD_BASE+0x24    //???


#define CMD_CONFIG_GLOBAL              0x30

#define CMD_RWIO					   0x31	//В CPU: FP_CMD_RWIO
#define CMD_CFG_RWIO				   0x32	//В CPU: FP_CMD_CFG_RWIO







static void register_fp_handlers(void);
static enum fp_error ferret_proto_handler_2(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len,
		uint8_t * ack_data, uint16_t *ack_len);

void mod_fp_init(void)
{
	register_fp_handlers();
}


static void register_fp_handlers(void)
{
	fp_register_handler(CMD_PORT_SET_STATE             , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_STATE             , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_SET_MODE              , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_MODE              , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_VALUE             , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_SET_BLOCK_STATE       , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_BLOCK_STATE       , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_APPLY_BLOCK_STATE     , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_STATUS            , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_STATUS            , ferret_proto_handler_2);

	fp_register_handler(CMD_LEDS_ALL_ON                , ferret_proto_handler_2);
	fp_register_handler(CMD_LEDS_ALL_OFF               , ferret_proto_handler_2);
	fp_register_handler(CMD_LEDS_GET_STATE             , ferret_proto_handler_2);
	fp_register_handler(CMD_LEDS_SET_TIMEOUT           , ferret_proto_handler_2);
	fp_register_handler(CMD_LEDS_GET_TIMEOUT           , ferret_proto_handler_2);
		
	fp_register_handler(CMD_USER_LED_SET		   , ferret_proto_handler_2);
	fp_register_handler(CMD_USER_LED_GET		   , ferret_proto_handler_2);

	fp_register_handler(CMD_PM_SET_RTC_ALARM           , ferret_proto_handler_2);
	fp_register_handler(CMD_PM_GET_RTC_ALARM           , ferret_proto_handler_2);
	fp_register_handler(CMD_PM_STOP                    , ferret_proto_handler_2);
	fp_register_handler(CMD_PM_WAIT                    , ferret_proto_handler_2);
	fp_register_handler(CMD_PM_RUN                     , ferret_proto_handler_2);

	fp_register_handler(CMD_I2C_GET_TEMP               , ferret_proto_handler_2);

	fp_register_handler(CMD_CPLD_FLASH_PREPEARE        , ferret_proto_handler_2);
	fp_register_handler(CMD_CPLD_FLASH_BLOCK	   , ferret_proto_handler_2); 
	fp_register_handler(CMD_CPLD_FLASH_BLOCK_STATUS    , ferret_proto_handler_2); 
	fp_register_handler(CMD_CPLD_FLASH_RUN		   , ferret_proto_handler_2);
	fp_register_handler(CMD_CPLD_FLASH_RUN_STATUS	   , ferret_proto_handler_2);

	fp_register_handler(CMD_EEPROM_SAVE_CONFIG         , ferret_proto_handler_2);
	fp_register_handler(CMD_EEPROM_RESET_CONFIG        , ferret_proto_handler_2);
	
	fp_register_handler(CMD_SET_BLOCK_STATE            , ferret_proto_handler_2);
	fp_register_handler(CMD_GET_BLOCK_STATE            , ferret_proto_handler_2);
	
	fp_register_handler(CMD_ALARM_SET                  , ferret_proto_handler_2);
	fp_register_handler(CMD_ALARM_GET                  , ferret_proto_handler_2);
	//AZ
	fp_register_handler(CMD_RW_VALUES            	   , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_GET_COUNTER       	   , ferret_proto_handler_2);
	fp_register_handler(CMD_PORT_RESET_COUNTER	   , ferret_proto_handler_2);
	
	//Che
	fp_register_handler(CMD_CONFIG_GLOBAL              , ferret_proto_handler_2);

	fp_register_handler(CMD_RWIO            	   , ferret_proto_handler_2);
	fp_register_handler(CMD_CFG_RWIO            	   , ferret_proto_handler_2);		
}

/*
static void altera_fw_update_task(uint32_t initial_data)
{
	uint32_t size;
	memcpy(&size, &altera_jbc[FLASHX_ALTERA_SIZE-4], 4);
	block_sw_watchdog_block();
	cpld_flash_status = altera_jbc_load_fw((uchar_ptr)&altera_jbc[0], size, 0);
	block_sw_watchdog_resume();
}*/


volatile uint32_t job_read_mask = 0, job_read_data, job_read_kz, job_read_obriv; 

void di_process_job(void)
{
	job_read_mask = 0xFFFFFFFF;
	if (job_read_mask) {
		int i;
		uint32_t vals = 0, err1 = 0, err2 = 0;
		uint16_t val;
		
		for (i=0; i<PORTS_COUNT; i++) 
		{
			port_get_value(i,&val);	
			if (val <= CH_ALARM) 	vals|= (val << i); 
			if (val == CH_KZ) 		err1|= (1 << i); 
			if (val == CH_OBRYV) 	err2|= (1 << i); 						
		}
		
		job_read_data 	= vals;
		job_read_kz		= err1;
		job_read_obriv	= err2;
	}
}


ExpBoardParametersDI48_t par_c;


enum ioboard_port_type {
    PORT_TYPE_DI_DC_2MA = 0,	            /**< цифровой вход типа "сухой контакт" с выдачей 2 мА */
    PORT_TYPE_DI_DC_10MA = 1,	            /**< цифровой вход типа "сухой контакт" с выдачей 10 мА */
    PORT_TYPE_DI_IEC = 2,		    /**< цифровой вход в режиме "потенциальный вход" */
    PORT_TYPE_DI_IC_2MA = 103,              /**< Цифровой вход типа «сухой контакт» с выдачей тока 2 мА в импульсном режиме */
    PORT_TYPE_DI_IC_10MA = 104,             /**< Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме */
    PORT_TYPE_DI_DC_10MA_CC = 105,          /**< Цифровой вход типа «сухой контакт» с выдачей тока 10 мА c контролем обрыва цепи */
    PORT_TYPE_DI_IC_10MA_CC = 106,          /**< Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме с контролем обрыва цепи */
    PORT_TYPE_DI_IEC_CC = 107,              /**< Цифровой вход в режиме «потенциальный вход» с контролем обрыва цепи */

    PORT_TYPE_DI_DC_10MA_CC_SC = 109,               /**< Цифровой вход типа «сухой контакт» с выдачей тока 10 мА c контролем обрыва цепи и короткого замыкания */
    PORT_TYPE_DI_IC_10MA_CC_SC = 110,               /**< Цифровой вход типа «сухой контакт» с выдачей тока 10 мА в импульсном режиме с контролем обрыва цепи и короткого замыкания */
    PORT_TYPE_DI_IEC_CC_SC = 111,                   /**< Цифровой вход в режиме «потенциальный вход» с контролем обрыва цепи и короткого замыкания */
};




uint32_t SwitchToBoardType(uint32_t type)
{
    switch (type) {
        case PORT_TYPE_DI_DC_2MA    : return CH_MODE_DI_DC_2MA 	    ;
        case PORT_TYPE_DI_DC_10MA   : return CH_MODE_DI_DC_10MA     ;
        case PORT_TYPE_DI_IEC       : return CH_MODE_DI_IEC 	    ;
        case PORT_TYPE_DI_IC_2MA    : return CH_MODE_DI_IC_2MA      ;
        case PORT_TYPE_DI_IC_10MA   : return CH_MODE_DI_IC_10MA     ;
        case PORT_TYPE_DI_DC_10MA_CC: return CH_MODE_DI_DC_10MA_CC  ;
        case PORT_TYPE_DI_IC_10MA_CC: return CH_MODE_DI_IC_10MA_CC  ;
        case PORT_TYPE_DI_IEC_CC    : return CH_MODE_DI_IEC_CC      ;
        
        case PORT_TYPE_DI_DC_10MA_CC_SC    : return CH_MODE_DI_DC_10MA_CC_KZ      ;
        case PORT_TYPE_DI_IC_10MA_CC_SC    : return CH_MODE_DI_IC_10MA_CC_KZ      ;
        case PORT_TYPE_DI_IEC_CC_SC        : return CH_MODE_DI_IEC_CC_KZ      ;

    }   return CH_MODE_DI_DC_2MA;

}


uint32_t SwitchToCPUType(uint32_t type)
{
    switch (type) {
        case CH_MODE_DI_DC_2MA 	    : return PORT_TYPE_DI_DC_2MA    ;
        case CH_MODE_DI_DC_10MA     : return PORT_TYPE_DI_DC_10MA   ;
        case CH_MODE_DI_IEC 	    : return PORT_TYPE_DI_IEC       ;
        case CH_MODE_DI_IC_2MA      : return PORT_TYPE_DI_IC_2MA    ;
        case CH_MODE_DI_IC_10MA     : return PORT_TYPE_DI_IC_10MA   ;
        case CH_MODE_DI_DC_10MA_CC  : return PORT_TYPE_DI_DC_10MA_CC;
        case CH_MODE_DI_IC_10MA_CC  : return PORT_TYPE_DI_IC_10MA_CC;
        case CH_MODE_DI_IEC_CC      : return PORT_TYPE_DI_IEC_CC    ;
        
        case CH_MODE_DI_DC_10MA_CC_KZ   : return PORT_TYPE_DI_DC_10MA_CC_SC    ;
        case CH_MODE_DI_IC_10MA_CC_KZ   : return PORT_TYPE_DI_IC_10MA_CC_SC    ;
        case CH_MODE_DI_IEC_CC_KZ       : return PORT_TYPE_DI_IEC_CC_SC    ;        
    }   return PORT_TYPE_DI_DC_2MA;
}

enum fp_error ferret_proto_handler_2(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len,
		uint8_t *ack_data, uint16_t *ack_len)
{
	enum fp_error ret = FP_ERR_SUCCESS;
	uint8_t index = 0;
	uint8_t i, j, flags_types;
	uint8_t *pd = (uint8_t*) ack_data;
		
	bus_show_state = LED_GREEN;	
	
	switch (cmd) {
        case CMD_CONFIG_GLOBAL: {
                //parse_configuration(cmd_data,cmd_len);
		queue_parce_configuration(cmd_data,cmd_len);
		*ack_len = 0;
                break;
        }	
	case CMD_PORT_SET_STATE              : {
		//Включение/выключение канала 
		port_queue_set_state(cmd_data[0], cmd_data[1]);
		*ack_len = 0;
		break;	
	}
	case CMD_PORT_GET_STATE              : {
		//запрос состояния канала (включен/выключен)
		uint8_t num = cmd_data[0];
		ack_data[0] = num;
		ack_data[1] = port_get_state(num);
		*ack_len = 2;
		break;	
	}
	case CMD_PORT_SET_MODE               : {
		port_queue_set_mode(cmd_data[0], cmd_data[1]);
		*ack_len = 0;
		break;	
	}
	case CMD_PORT_GET_MODE               : {
		uint8_t num = cmd_data[0];
		ack_data[0] = num;
		ack_data[1] = port_get_mode(num);
		*ack_len = 2;
		break;	
	}
	
	case CMD_PORT_GET_COUNTER			 : {
		uint8_t num = cmd_data[0];
		uint32_t value = 0;
		
		//port_get_counter(num, &value);
		*(uint8_t*)ack_data = num;
		#if (1)		
		*(uint32_t *)(ack_data+sizeof(uint8_t)) = value;
		#else
		*(uint32_t *)(ack_data+sizeof(uint8_t)) = num * 1000 + 777;				
		#endif
		
		*ack_len = 5;		
		break;
	}
	
	case CMD_PORT_RESET_COUNTER			 : {
		uint8_t num = cmd_data[0];			
		//port_reset_counter(num);
		*ack_len = 0;		
		break;
	}
		
		
	case CMD_PORT_GET_VALUE              : {
		uint8_t num = *(uint8_t *)cmd_data;
		uint16_t value = 0;

		port_get_value(num, &value);
		
		*(uint8_t*)ack_data = num;
		*(uint32_t *)(ack_data+sizeof(uint8_t)) = value;
		*ack_len = 5;
		break;	
	}



	case CMD_EEPROM_SAVE_CONFIG         : {
		queue_config_change(CONFIG_SAVE);
		*ack_len = 0;
		break;	
	}
	case CMD_EEPROM_RESET_CONFIG         : {
		queue_config_change(CONFIG_ERACE);
		*ack_len = 0;
		break;	
	}


	case CMD_RW_VALUES : {
		uint32_t rmask = *(uint32_t *)(cmd_data);
#if (1)
		*(uint32_t *)(ack_data+0*sizeof(uint32_t)) = job_read_data;
		*(uint32_t *)(ack_data+1*sizeof(uint32_t)) = job_read_kz;
		*(uint32_t *)(ack_data+2*sizeof(uint32_t)) = job_read_obriv;
		
#else
		*(uint32_t *)(ack_data+0*sizeof(uint32_t)) = 1;
		*(uint32_t *)(ack_data+1*sizeof(uint32_t)) = 2;
		*(uint32_t *)(ack_data+2*sizeof(uint32_t)) = 3;		
#endif
		*ack_len = sizeof(uint32_t) * 3;
		break;
	}
	

	case CMD_CFG_RWIO : { 
		data_type = *(uint8_t *)(cmd_data);//задание типа передаваемых данных
		*ack_len = 0;
		break; 
	}
	
	default: {
		ret = FP_ERR_UNKNOWN_CMD;
		break;
	}
	}

#if TEST_GPIO
	lwgpio_set_value(&t1, LWGPIO_VALUE_LOW);
#endif
	
	return ret;
}
