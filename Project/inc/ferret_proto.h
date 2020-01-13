#ifndef FERRET_PROTO_H_
#define FERRET_PROTO_H_

#include "MDR32Fx.h"
#include "MDR32F9Qx_lib.h"
#include "MDR32F9Qx_config.h"

#define FP_VERBOSE 0

/*
 *   Реализация ответной части протокола (или шины данных) обмена информацией
 * с головным устройством в составе комплекса устройств проекта PLC через SPI
 * и другие шины данных, например UART.
 * 
 *   Основные особенности протокола:
 * - обмен данными идет в режиме запрос-ответ. запрос (или команда) поступает
 *   от головного устройства, обрабатывается, и в случае отсутствия ошибок в данных,
 *   вызывается обработчик входящей команды, который формирует ответ на команду.
 * - ответ на запрос (и сам запрос) может быть нулевой длины (если не считать 
 *   служебных заголовков.
 * - ответ на запрос должен быть обязательно.
 * - обработчик должен быть максимально быстрым, если обработчик не успевает 
 *   сформировать ответ в отведенное время, автоматически отсылается сообщение
 *   об ошибке. повторный запрос данной команды с головного устройства возвратит
 *   подготовленные к тому времени данные без вызова обработчика (только в случае,
 *   если следующая команда та же, что и долго выполнявшаяся. в противном случае,
 *   будет вызван обработчик нового запроса, а посчитанные ранее данные утеряны).
 * - максимальный размер ответа - FP_PAYLOAD_LEN (512) байт.
 * - при запросе присылается номер модуля в кроссе, поэтому имеется отдельная
 *   функция, которая возвращает этот номер. её можно использовать на свое усмотрение.
 * - элементарная единица данных в работе протокола - FP_WORD_SIZE (2) байта. поэтому,
 *   в частности, при вызове обработчика команды передается длина выровненная на 
 *   FP_WORD_SIZE (2) байта, т.е. в общем случае не равная размеру ожидаемого типа.
 *   см. в примере, как с этим работать.
 * - реализация максимально оторвана от аппаратных настроек. но есть и жестко 
 *   используемые параметры, например - таймер PIT0 для отслеживания долго выполняющихся
 *   функций-обработчиков.
 *         
 *   Использование:
 * - инициализация функцией fp_init(), в качестве параметра необходимо передать 
 *   заполненную корректными данными структуру fp_config.
 * - регистрация обработчиков (или одного обработчика на все команды, по желанию).
 * 
 *   Пример обработчика:
 *   struct fp_cmd_test2 {
 *     uint16_t f1;
 *   } __attribute__ ((__packed__));
 *   struct fp_ack_test2 {
 *     uint16_t f1;
 *   } __attribute__ ((__packed__));
 *   
 *   enum fp_error ferret_proto_handler(int cmd, unsigned char_ptr cmd_data, int cmd_len,
 *   		unsigned char_ptr ack_data, int *ack_len)
 *   {
 *   	enum fp_error ret = FP_ERR_SUCCESS;
 *   	switch (cmd) {
 *   	case FP_CMD_TEST2: {
 *   		struct fp_cmd_test2 *cmd;
 *   		struct fp_ack_test2 *ack;
 *   		uint16_t t;
 *   		if (2*((sizeof(struct fp_cmd_test2)+1)/2) != cmd_len) {
 *   			ret = FP_ERR_INVALID_PAYLOAD_LEN;
 *   			break;
 *   		}
 *   		cmd = (struct fp_cmd_test2 *)cmd_data;
 *   		t = cmd->f1;
 *   		ack = (struct fp_ack_test2 *)ack_data;
 *   		ack->f1 = t+1;
 *   		*ack_len = sizeof(struct fp_ack_test2);
 *   		ret = FP_ERR_SUCCESS;
 *   		break;
 *   	}
 *   	case CMD_AIO_AI_SET_STATE              : {
 *   		uint8_t num = *(uint8_t *)cmd_data;
 *   		uint8_t state = *(uint8_t *)(cmd_data+sizeof(uint8_t));
 *   		aio_ai_set_state(num, state);
 *   		*ack_len = 0;
 *   		break;	
 *   	}
 *   	default:
 *   		ret = FP_ERR_UNKNOWN_CMD;
 *   	}
 *   	return ret;
 *   }
 *   
 *   Замечание: После инициализации и регистрации обработчиков в любой момент 
 * следует ожидать запрос от головного устройства. В связи с этим, требуется 
 * перепроверить потокобезопасность используемых функций.
 * 
 *   Требования к оборудованию (SPI related):
 * - нога cs_gpio должна быть заведена отдельно (не на PCS0 SPI модуля), но 
 *   запараллелена с ним. CS используется как прерывание для начала забора данных.
 *   
 *   ID сообщений, используемые при регистрации обработчиков, должны быть такими
 * же, что и в соответствующем драйвере головного модуля. Кроме того, ID должны 
 * быть уникальными в пределах одной совокупности устройств. В связи с этим
 * вводятся следующие правила по использованию ID команд:
 * 
 * 0x000 .. 0x1ff - коды команд-запросов к модулям
 * 0x200 .. 0x3ff - коды ответов на запросы (id_запроса + 0x200)
 * 0x400 .. 0x5ff - ответ на команду содержит код ошибки (id_запроса + 0x400)
 * 
 * 0x000 .. 0x040 - общие для всех модулей команды, зарезервированный диапазон
 * 0x040 .. 0x080 - команды к модулю DIO16 
 * 0x080 .. 0x0C0 - команды к модулю AIO8 
 * 0x0C0 .. 0x100 - команды к модулю RSSW 
 * 0x100 .. 0x140 - команды к модулю FESW 
 * 0x140 .. 0x180 - команды к модулю AIT12
 * 0x180 .. 0x1C0 - команды к модулю DI12F
 */

#define FP_WORD_SIZE   2
#define FP_HEADER_LEN  4
#define FP_PAYLOAD_LEN 512
#define FP_ERR_PAYLOAD_LEN 16
#define FP_PACKET_LEN  (FP_HEADER_LEN + FP_PAYLOAD_LEN)
#define FP_ERR_PACKET_LEN  (FP_HEADER_LEN + FP_ERR_PAYLOAD_LEN)

#define FP_CMD_COUNT_MAX 0x200

/* это определение необходимо для задания размера буфера в ACK на 
 * команду FP_CMD_BOARD_INFO.
 * в конфигурации основной прошивки, где сохраняется/считывается
 * серийный номер, размер должен быть не больше этого.
 */
#define FP_BOARD_SERIAL_LEN (16)

typedef struct {
	/* */
	int  (*fp_read)(void);
	void (*fp_write)(void);

	/* параметры устройства. будут возвращены в ответ на команду с ID 0 */
	uint16_t board_id;
	uint16_t board_hw_vers;
	uint16_t board_pof_vers;
	uint16_t board_sw_vers;

	/* handler, возвращающий серийный номер устройства, требуется для заполнения
	 * соответствующего поля в ответе на команду FP_CMD_BOARD_INFO, не является
	 * обязательным
	 */
	const unsigned char* (*cfg_get_serial)(void);
	/* handler, прописывающий серийный номер устройства. требуется для команды 
	 * FP_CMD_SET_SERIAL_NUM, не является обязательным
	 * предполагается, что в этой функции различные len будут корректно обработаны
	 */
	void (*cfg_set_serial)(unsigned char *, int len);
	
	uint8_t (*pm_is_lowpower_state)(void);
	void   (*pm_lowpower_enter)(void);
	void   (*pm_lowpower_exit)(void);
} FP_CONFIG, *FP_CONFIG_PTR;

typedef union fp_packet {
    uint8_t buf[FP_PACKET_LEN];
    struct {
        uint16_t packet_len : 12;
        uint16_t dev_num : 4;
        uint16_t crc;
        uint16_t cmd_id;
        union {
            uint8_t  data_uchar [FP_PACKET_LEN - FP_HEADER_LEN];
            uint16_t data_ushort[FP_PACKET_LEN/sizeof(uint16_t) - FP_HEADER_LEN];
        } data;
    } packet;
} FP_PACKET, * FP_PACKET_PTR;

typedef union fp_err_packet {
    uint8_t buf[FP_ERR_PACKET_LEN];
    struct {
        uint16_t packet_len : 12;
        uint16_t dev_num : 4;
        uint16_t crc;
        uint16_t cmd_id;
        union {
            uint8_t  data_uchar [FP_ERR_PACKET_LEN - FP_HEADER_LEN];
            uint16_t data_ushort[FP_ERR_PACKET_LEN/sizeof(uint16_t) - FP_HEADER_LEN];
        } data;
    } packet;
} FP_ERR_PACKET, * FP_ERR_PACKET_PTR;

typedef enum fp_error {
	FP_ERR_SUCCESS,
	FP_ERR_UNKNOWN,				/* 1 */
	FP_ERR_UNKNOWN_CMD,			/* 2 */
	FP_ERR_INVALID_PACKET_LEN,	/* 3 */
	FP_ERR_INVALID_MSG_ID,		/* 4 */
	FP_ERR_INVALID_CMD_LEN,		/* 5 */
	FP_ERR_UNEXPECTED_MSG,		/* 6 */
	FP_ERR_CRC_FAIL,			/* 7 */
	FP_ERR_TIMEOUT,				/* 8 */
	FP_ERR_BUSY, 				/* 9 */
	FP_ERR_LOOPBACK_IS_SET = 13      
} FP_ERROR;

typedef FP_ERROR (*FP_HANDLER)(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len,
		uint8_t *ack_data, uint16_t *ack_len);

typedef struct{
	uint8_t  tx_busy;  /* парсинг данных начат */
	uint8_t  tx_ready; /* парсинг данных закончен, остальные поля структуры валидны */
	uint16_t cmd_id;   /* идентификатор команды, при обработке которой произошла err */
	uint16_t dev_num;  /* номер устройства, при работе с которым произошла err */
	FP_ERROR err;	
} FP_PACKET_STATE, * FP_PACKET_STATE_PTR;

typedef struct {
	uint32_t tx_packets;
	uint32_t tx_bytes;
	
	uint32_t rx_packets;
	uint32_t rx_bytes;
	
	uint32_t err_unknown;
	uint32_t err_unknown_cmd;
	uint32_t err_packet_len;
	uint32_t err_msg_id;
	uint32_t err_cmd_len;
	uint32_t err_unexpected_msg;
	uint32_t err_crc;
	uint32_t err_timeout;
	uint32_t err_busy;
} FP_COUNTERS, * FP_COUNTERS_PTR;

typedef struct{
	FP_CONFIG config;  /* конфигурация драйвера */
	
	FP_PACKET rx_buf;  /* буфер приёма данных   */
	FP_PACKET tx_buf;  /* буфер отправки данных */
	
	FP_PACKET_STATE tx_state; /* состояние последней "отправленной" посылки */
	
	void (*recv_start)(void); /* метод, начинающий прием данных */
	
	FP_HANDLER handlers[FP_CMD_COUNT_MAX]; /* список зарегистрированных обработчиков сообщений */
	
	FP_COUNTERS counters;
} FP_DRIVER, * FP_DRIVER_PTR;

void fp_init(void);
void fp_parse_data(void);
void fp_remove(FP_CONFIG config);
int fp_register_handler(uint16_t cmd, FP_HANDLER handler);
void fp_send(void);
int fp_get_module_num(void);
extern FP_DRIVER fp_driver;
#endif /* FERRET_PROTO_H_ */
