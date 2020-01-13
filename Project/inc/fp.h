#ifndef FP_H_
#define FP_H_

#include "stdint.h"

extern void mod_fp_init(void);
extern void mod_fp_remove(void);

extern void di_process_job(void);


enum fp_error ferret_proto_handler_2(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len, uint8_t *ack_data, uint16_t *ack_len);





/**
 * @struct ExpboardParametersSetHeader
 * @brief Стандартный заголовок структуры параметров модуля расширения.
 */
#pragma pack(push, 1)
typedef struct ExpboardParametersSetHeader {
	uint16_t  expboard_id; 	/**< @brief Тип модуля */
	uint16_t  version;  	/**< @brief Версия структуры */
	uint32_t  size;    		/**< @brief Размер структуры в байтах */
} ParametersSetHeader_t;


/**
 * @struct  ExpboardCommonParameters
 * @brief   Структура общих параметров модулей
 */
typedef struct ExpboardCommonParameters {
	uint8_t   board_num; 			/**< @brief Слот размещения модуля расширения.*/
	uint8_t   ports_count;  		/**< @brief Количество портов*/

	uint16_t  data_type;  			/**< @brief Тип данных, используемый для задания и получения значений с портов модуля.*/
	uint16_t  poll_period;  		/**< @brief Период группового чтения (сканирования) параметров модуля*/
	uint8_t   leds_timeout;  		/**< @brief Таймаут выключения индикации, с.*/

	uint8_t   pm_state : 3;  		/**< @brief Режим энергосбережения (энергопотребления)*/
	uint8_t   leds_cfg_state : 1;  	/**< @brief Состояние светодиодной индикации*/
	uint8_t   user_led : 2;  		/**< @brief Состояние пользовательского светодиода*/
	uint8_t   enable_polling : 1;  	/**< @brief Флаг периодического  группового чтения (сканирования) параметров модуля*/
	uint8_t   unused : 1;
} ExpboardParameters_t;



/**
 * @struct ChannelParametersDI32_Input
 * @brief Структура параметров канала ввода Di-32
 */
//typedef struct ChannelParametersDI48_Input {
//    uint8_t  pm_state : 1;     	/**< @brief Код состояния питания порта ввода-вывода*/
//    uint8_t  bs_pm_state : 1;  	/**< @brief Состояние питания порта ввода-вывода в режиме блокировки*/
//    uint8_t  unused : 6;
//
//    uint8_t  alarm_value;      	/**< @brief Значение уровня тревоги по порту ввода-вывода*/
//    uint8_t  mode;              /**< @brief Код типа порта ввода-вывода*/
//    uint8_t  bs_mode;          	/**< @brief Код типа порта ввода-вывода в режиме блокировки*/
//
//} ChannelParametersDI48_Input_t;


typedef struct ChannelParametersDI48_Input {
    uint8_t  pm_state : 1;     	/**< @brief Код состояния питания порта ввода-вывода*/
    uint8_t  bs_pm_state : 1;  	/**< @brief Состояние питания порта ввода-вывода в режиме блокировки*/
    uint8_t  unused : 6;

    //uint8_t  alarm_value;      	/**< @brief Значение уровня тревоги по порту ввода-вывода*/
    uint8_t  mode;              /**< @brief Код типа порта ввода-вывода*/
    //uint8_t  mode:4;              /**< @brief Код типа порта ввода-вывода*/
    //uint8_t  bs_mode:4;          	/**< @brief Код типа порта ввода-вывода в режиме блокировки*/

} ChannelParametersDI48_Input_t;




/**
 * @struct ExpBoardParametersDI32
 * @brief Структура параметров параметров модуля ввода-вывода DI32;
 */
typedef struct ExpBoardParametersDI48 {
    ParametersSetHeader_t      		header;   	/*8 *< @brief Заголовок блока параметров.*/
    ExpboardParameters_t       		common; 	/*8 *< @brief Общие параметры.*/
    ChannelParametersDI48_Input_t 	port[48];       /*4*48 *< @brief Параметры каналов модуля.*/
} ExpBoardParametersDI48_t ;

#pragma pack(pop)


extern ExpBoardParametersDI48_t par_c;


#endif /* FP_H_ */
