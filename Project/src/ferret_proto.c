#include "ferret_proto.h"
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
#include "leds.h"
#include "fp.h"
#include "modbus.h"
#include "spi_ferret_proto.h"
#include "crc_hw.h"
#include "string.h"

/*****************************************************************************/
/* Custom definitions                                                        */
/*****************************************************************************/

#define FP_ACK_SHIFT      0x200

#define PDD_DISABLE       0u
#define PDD_ENABLE        1u

#define PIT_PDD_CHANNEL_0 0U

#define FP_MAX_HANDLER_CALL_TIME (30)

//#define FP_TIMER_DELAY_US 600
//AZ: увеличили скорость ответа модуля ЦПУ
#define FP_TIMER_DELAY_US 300

#define LPM_DEPENDENCY_LEVEL_FERRET_PROTO (BSP_LPM_DEPENDENCY_LEVEL_SERIAL_INT + 1)

/*****************************************************************************/
/* Предварительное объявление функций                                        */
/*****************************************************************************/
static void fp_recv_start(void);
static void fp_prep_err_ans(uint8_t dev_num, uint16_t cmd, FP_ERROR err);
static void fp_register_common_messages(void);

static void fp_tu_init(uint32_t delay);
void fp_tu_isr(void);
static void fp_tu_enable(void);
static void fp_tu_disable(void);

/*  */
FP_DRIVER fp_driver;

void _time_delay (unsigned long ms){
}

void fp_tu_isr(void)
{
	/**
	 * Clears PIT interrupt flag.
	 * @param peripheralBase Peripheral base address.
	 * @param ChannelIdx PIT channel index.
	 */
	#define PIT_PDD_ClearInterruptFlag(peripheralBase, ChannelIdx) ( \
	    (PIT_TFLG_REG(peripheralBase,(ChannelIdx)) = \
	     PIT_TFLG_TIF_MASK), \
	    (void)PIT_LDVAL_REG(peripheralBase,(ChannelIdx)) \
	  )

	//PIT_PDD_ClearInterruptFlag(PIT_BASE_PTR, PIT_PDD_CHANNEL_0);

	fp_tu_disable();

	/* с помощью нижеследующей операции запускается передача
	 * сообщения центральному модулю в случае формирования
	 * ответа от handler'а сообщения, или сообщение о том, что
	 * ответ он еще не успел сформировать (timeout),
	 * сообщения, связанные с ошибками парсинга или
	 * ответом на предидущий запрос.
	 */
	fp_send();
}

static void fp_tu_init(uint32_t delay)
{

}

static void fp_tu_remove(void)
{

}

static void fp_tu_enable(void)
{

}

static void fp_tu_disable(void)
{

}

void fp_init(void)
{
	//fp_hw_init();

	//memset(fp_driver.handlers, NULL, sizeof(fp_driver.handlers));
	memset(fp_driver.rx_buf.buf,  0, FP_PACKET_LEN);
	memset(fp_driver.tx_buf.buf,  0, FP_PACKET_LEN);

	//fp_task0_id = _task_create(0, 0, (unsigned long)(&fp_template_list[0]));

  fp_register_common_messages();

  fp_driver.config.board_id=0x0D48;
  fp_driver.config.board_hw_vers=0x1201;
  fp_driver.config.board_pof_vers=0x0062;
  fp_driver.config.board_sw_vers=0x0003;

  memset(&fp_driver.counters, 0, sizeof(fp_driver.counters));

  fp_driver.recv_start = fp_recv_start;
}

void fp_remove(FP_CONFIG config)
{
	fp_tu_remove();

#if MQX_ENABLE_LOW_POWER
	  _lpm_unregister_driver(fp_tu_lpm_id);
#endif

	//_task_abort(fp_task0_id);
	//_lwsem_destroy(&fp_parse_sem);

	//config.fp_hw_remove();
}

static void fp_recv_start(void)
{
	static uint32_t first_busy_ans_time = 0;
	//static TIME_STRUCT ts;

	/*
	 * нижевызываемые функции могут выполнятся неизвестное количество времени,
	 * поэтому запускаем отправку с таймаутом, которая в любом случае обязана
	 * сработать и ответить центральному модулю.
	 */
	fp_tu_enable();

	/*
	 * если мы оказались в состоянии экономии энергии - выходим из этого состояния
	 * в любом случае, ни принять данные, ни ответить на них мы все равно не успеем,
	 * поэтому просто разгоняем систему в надежде ответить на повторный запрос.
	 */
	if (fp_driver.config.pm_is_lowpower_state) {
		if (fp_driver.config.pm_is_lowpower_state()) {
			if (fp_driver.config.pm_lowpower_exit) {
				/* в сообщении об ошибке отправляем нулевой идентификатор команды, т.к.
				 * мы не в состоянии узнать, какая команда была на входе.
				 * да и отправка скорее всего не произойдет...
				 */
				fp_prep_err_ans(fp_driver.rx_buf.packet.dev_num, 0, FP_ERR_TIMEOUT);
				fp_driver.config.pm_lowpower_exit();
				return;
			}
		}
	}

	first_busy_ans_time = 0;

	/* быстрые проверки выполнены (они выполняются в контексте прерывания, поэтому должны
	 * быть действительно быстрыми), остальное запускаем в отдельном task'е.
	 * запуск указанных выше проверок в этом task'е не позволит обработать состояние BUSY,
	 * т.к. в этом режиме handler еще выполняется и до собственно вызова проверки на
	 * выполнение этого handler'а просто не дойдет, пока он не завершится.
	 */
}


void fp_parse_data(void)
{
	uint8_t dev_num;
	uint16_t cmd;
	uint16_t ack_len = 0;
	FP_ERROR err = FP_ERR_SUCCESS;
	static uint16_t last_rx_crc = 0;
	uint32_t crc;
	uint16_t len;
	int ret = 0;

	ret = spi_fp_read();

	if (ret < 0) { /* если read вернул ошибку, нет смысла делать что то дальше */
		if (ret == -FP_ERR_INVALID_PACKET_LEN) {
			/* ошибка проверки длины пакета */
			fp_prep_err_ans(0, 0, FP_ERR_INVALID_PACKET_LEN);
			return;
		} else if (ret == -FP_ERR_LOOPBACK_IS_SET) {
			/* включен LOOPBACK_MODE в spi_ferret_proto.c */
			//PIT_PDD_ClearInterruptFlag(PIT_BASE_PTR, PIT_PDD_CHANNEL_0);
			fp_tu_disable();
			return;
		}
		fp_prep_err_ans(fp_driver.rx_buf.packet.dev_num, fp_driver.rx_buf.packet.cmd_id, FP_ERR_UNKNOWN);
		return;
	}

	/*
	 * вычисляем CRC по всем принятым данным, кроме первых 2-х байт (там как раз проверочная CRC)
	 */
	len = fp_driver.rx_buf.packet.packet_len * FP_WORD_SIZE - FP_WORD_SIZE;
	crc = ModBusCRC16(&fp_driver.rx_buf.buf[FP_HEADER_LEN], len);
	crc &= 0xFFFF;
	//crc = ((crc & 0xFF00) >> 8) | ((crc & 0x00FF) << 8);

	if (fp_driver.rx_buf.packet.crc != crc) {
#if FP_VERBOSE
		printf("CRC checksum failed, expected 0x%04X, recieved 0x%04X\n", fp_driver.rx_buf.packet.crc, crc);
#endif
		fp_prep_err_ans(fp_driver.rx_buf.packet.dev_num, 0, FP_ERR_CRC_FAIL);
		return;
	}

	dev_num = fp_driver.rx_buf.packet.dev_num;
	fp_driver.tx_buf.packet.dev_num = dev_num;

	cmd = fp_driver.rx_buf.packet.cmd_id;

	if (cmd >= FP_ACK_SHIFT) {
		/* ошибка проверки идентификатора команды, сразу делаем отправку */
		fp_prep_err_ans(dev_num, cmd, FP_ERR_INVALID_MSG_ID);
		return;
	}
#if FP_VERBOSE
	printf("fp recieved message with id: %d\n", cmd);
#endif

	/*
	 * если поле tx_ready равно '1', это означает, что обработка предыдущего запроса
	 * затянулась, был отослан пакет с TIMEOUT, но затем обработка запроса завершилась.
	 * т.е. данные необходимо отправить на центральный модуль. поэтому, дополнительно проверяем
	 * корректность идентификатора команды, и если он именно ожидаемый, то сразу отвечаем.
	 */
	/*
	 * здесь может возникнуть (и возникала) следующая неприятность: посылаем на устройство
	 * "сеттер", устанавливающий параметр в определенное значение. допустим, что ЦПУ отсылается
	 * ответ с timeout. Затем ЦПУ посылает тот же сеттер, но с другим значением параметра. Тогда
	 * ранее контроллер бы ответил ответом на предидущий сеттер, а этот бы отбросил, что как
	 * минимум неочевидно, т.к. ни в каких счетчиках не отображается. С другой стороны, такое
	 * поведение ЦПУ тоже не совсем верно, ибо, если получена ошибка в ответ, то её необходимо
	 * корректно обрабатывать и в случае timeout отсылать перед следующим сеттером запрос на
	 * получение значения. Возможно следующее исправлени ситуации - проверять не только
	 * идентичность CMD/ACK, но и CRC сообщений, которые должны совпадать в случае "перезапроса"
	 * со стороны ЦПУ. Если же со стороны ЦПУ посылается абсолютно идентичный "сеттер" в ответ
	 * на сообщение с timeout'ом, то это явно неверное решение.
	 */
	if (*(volatile uint8_t *)&fp_driver.tx_state.tx_ready == 1) {
		if ((cmd == (fp_driver.tx_buf.packet.cmd_id & (FP_ACK_SHIFT-1)))
				&& (fp_driver.rx_buf.packet.crc == last_rx_crc)) {
#if FP_VERBOSE
			printf("fp send answer for previous asked message\n");
#endif
			/* поля fp_driver.tx_state тут заполнять не нужно, они заполнены ранее
			 * просто завершаем работу, отправка будет выполнена по таймауту */
			return;
		} else {
			/* пришел запрос на неожиданную команду, отвечаем ошибкой */
			/* update: такое поведение может приводить к залипу модуля, убрано */
			/* fp_prep_err_ans(cmd, FP_ERR_UNEXPECTED_MSG); */
			/* return; */
		}
	}

	/* запоминаем CRC входящего пакета для перепроверки возможного повторного запроса на него
	 * в случае ответа с timeout'ом (см. выше по коду использование last_rx_crc).
	 */
	last_rx_crc = fp_driver.rx_buf.packet.crc;

	/*
	 * заранее помечаем идентификатор ответа, если выполнение функции затянется,
	 * сможем сформировать адекватную посылку
	 */
	fp_driver.tx_buf.packet.cmd_id = cmd + FP_ACK_SHIFT;
	fp_driver.tx_state.err      = 0;
	fp_driver.tx_state.tx_busy  = 1;
	fp_driver.tx_state.tx_ready = 0;

	if (fp_driver.handlers[cmd]) {
#if FP_VERBOSE
		printf("fp calling message handler...\n", cmd);
#endif

		err = (*fp_driver.handlers[cmd])(cmd, fp_driver.rx_buf.packet.data.data_uchar,
				fp_driver.rx_buf.packet.packet_len * 2 - FP_HEADER_LEN,
				&fp_driver.tx_buf.packet.data.data_uchar[0], &ack_len);

		ack_len = 2*((ack_len+1)/2);
		fp_driver.tx_buf.packet.packet_len = ack_len/2 + 2;
	} else {
#if FP_VERBOSE
		printf("fp no handler for message\n", cmd);
#endif
		err = FP_ERR_UNKNOWN_CMD;
	}

	/* парсинг и вызов обработчика команды завершен, делаем отправку */
	fp_driver.tx_state.err      = err;
	fp_driver.tx_state.tx_ready = 1;
	fp_driver.tx_state.tx_busy  = 0;


}

static void fp_prep_err_ans(uint8_t dev_num, uint16_t cmd, FP_ERROR err)
{
	fp_driver.tx_state.err = err;

	fp_driver.tx_state.dev_num = dev_num;

	/*
	 * сообщение с cmd >= 0x400 воспринимается центральным устройством, как содержащее
	 * код ошибки с фиксированной длиной
	 */
	fp_driver.tx_state.cmd_id = cmd + ((cmd > 0x200) ? 0x200 : 0x400);

	/* корректируем статистику по ошибкам */
    if (err == FP_ERR_UNKNOWN) {
    	fp_driver.counters.err_unknown += 1;
    } else if (err == FP_ERR_UNKNOWN_CMD) {
    	fp_driver.counters.err_unknown_cmd += 1;
    } else if (err == FP_ERR_INVALID_PACKET_LEN) {
    	fp_driver.counters.err_packet_len += 1;
    } else if (err == FP_ERR_INVALID_MSG_ID) {
    	fp_driver.counters.err_msg_id += 1;
    } else if (err == FP_ERR_INVALID_CMD_LEN) {
    	fp_driver.counters.err_cmd_len += 1;
    } else if (err == FP_ERR_UNEXPECTED_MSG) {
    	fp_driver.counters.err_unexpected_msg += 1;
    } else if (err == FP_ERR_CRC_FAIL) {
    	fp_driver.counters.err_crc += 1;
    } else if (err == FP_ERR_TIMEOUT) {
    	fp_driver.counters.err_timeout += 1;
    } else if (err == FP_ERR_BUSY) {
    	fp_driver.counters.err_busy += 1;
    }
}

static void fp_send_err_ans()
{
	FP_ERR_PACKET err_ans;
#if FP_VERBOSE
	printf("fp send error: %d, cmd: %d\n",
			fp_driver.tx_state.err, fp_driver.tx_state.cmd_id);
#endif

	err_ans.packet.dev_num = fp_driver.tx_state.dev_num;

	err_ans.packet.data.data_uchar[0] = 0xCC;
	err_ans.packet.data.data_uchar[1] = fp_driver.tx_state.err & 0xFF;
	err_ans.packet.packet_len = 3;

	err_ans.packet.cmd_id = fp_driver.tx_state.cmd_id;

	//fp_driver.config.fp_write(&err_ans.buf);
}

void fp_send(void)
{
	uint8_t ready =*(volatile uint8_t *)&fp_driver.tx_state.tx_ready;
	if (ready && (fp_driver.tx_state.err == 0)) {
		/* handler завершил выполнение, без ошибок */
		spi_fp_write();

		fp_driver.tx_state.err = 0;
		fp_driver.tx_state.tx_busy = 0;
		fp_driver.tx_state.tx_ready = 0;
	} else if (ready && (fp_driver.tx_state.err != 0)) {
		/* handler завершил выполнение, с ошибкой */
		fp_prep_err_ans(fp_driver.tx_buf.packet.dev_num,
				fp_driver.tx_buf.packet.cmd_id, fp_driver.tx_state.err);
		fp_send_err_ans();

		fp_driver.tx_state.err = 0;
		fp_driver.tx_state.tx_busy = 0;
		fp_driver.tx_state.tx_ready = 0;
	} else if (!ready && (fp_driver.tx_state.err == 0)) {
		/* handler еще не завершил выполнение */
		fp_prep_err_ans(fp_driver.tx_buf.packet.dev_num,
						fp_driver.tx_buf.packet.cmd_id, FP_ERR_TIMEOUT);
		fp_send_err_ans();
	} else if (!ready && (fp_driver.tx_state.err != 0)) {
		/* handler еще не вызывался, ошибка произошла раньше, т.е. fp_prep_err_ans() делался */
		fp_send_err_ans();

		fp_driver.tx_state.err = 0;
		fp_driver.tx_state.tx_ready = 0;
	}
}

int fp_register_handler(uint16_t cmd, FP_HANDLER handler)
{
#if FP_VERBOSE
	printf("fp register handler for message %d\n", cmd);
#endif

	if (cmd >= FP_CMD_COUNT_MAX)
		return -1;

	fp_driver.handlers[cmd] = handler;
	return 0;
}

int fp_get_module_num(void)
{
	return fp_driver.tx_buf.packet.dev_num;
}

struct fp_cmd_test1 {
	uint8_t f1;
};

struct fp_ack_test1 {
	uint8_t f1;
};

struct fp_cmd_test2 {
	uint16_t f1;
};

struct fp_ack_test2 {
	uint16_t f1;
};

struct fp_cmd_test3 {
	uint8_t f1;
	uint16_t f2;
} ;

struct fp_ack_test3 {
	uint8_t f1;
	uint16_t f2;
};

struct fp_ack_board_info {
	uint16_t id;

	uint16_t hw_vers;
	uint16_t pof_vers;
	uint16_t sw_vers;
	uint8_t serial[FP_BOARD_SERIAL_LEN];
	uint32_t uptime;
};

#define FP_CMD_BOARD_INFO 0

#define FP_CMD_TEST_DATA1          0x01
#define FP_CMD_TEST_DATA2          0x02
#define FP_CMD_TEST_DATA3          0x03
#define FP_CMD_TEST_TIMEOUT        0x04
#define FP_CMD_TEST_ELAPSED_TIME   0x05
#define FP_CMD_TEST_DIFF_SIZES1    0x06
#define FP_CMD_TEST_DIFF_SIZES2    0x07

#define FP_CMD_SET_SERIAL_NUM 0x10

#define FP_CMD_GOTO_BLDR 			0x15
#define FP_CMD_BOARD_RESET 			0x16
#define FP_CMD_BLDR_SET_AUTOBOOT	0x17

#define FP_CMD_GOTO_LOWPOWER 0x40

enum fp_error fp_common_handler(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len,
		uint8_t *ack_data, uint16_t *ack_len)
{
	enum fp_error ret;

#if FP_VERBOSE
	printf("%s: cmd: %d\n", __func__, cmd);
#endif

	switch (cmd) {
	case FP_CMD_TEST_DATA1: {
		struct fp_cmd_test1 *cmd;
		struct fp_ack_test1 *ack;
		uint8_t t;
		if (2*((sizeof(struct fp_cmd_test1)+1)/2) != cmd_len) {
			ret = FP_ERR_INVALID_CMD_LEN;
			break;
		}

		cmd = (struct fp_cmd_test1 *)cmd_data;
		t = cmd->f1;
#if FP_VERBOSE
		printf("cmd_test1: f1: %d (%x)\n", t, t);
#endif
		ack = (struct fp_ack_test1 *)ack_data;
		ack->f1 = t+1;
#if FP_VERBOSE
		printf("ack_test1: f1: %d (%x)\n", ack->f1, ack->f1);
#endif
		*ack_len = sizeof(struct fp_ack_test1);
		ret = FP_ERR_SUCCESS;
		break;
	}
	case FP_CMD_TEST_DATA2: {
		struct fp_cmd_test2 *cmd;
		struct fp_ack_test2 *ack;
		uint16_t t;
		if (2*((sizeof(struct fp_cmd_test2)+1)/2) != cmd_len) {
			ret = FP_ERR_INVALID_CMD_LEN;
			break;
		}
		cmd = (struct fp_cmd_test2 *)cmd_data;
		t = cmd->f1;
#if FP_VERBOSE
		printf("cmd_test2: f1: %d (%x)\n", t, t);
#endif
		ack = (struct fp_ack_test2 *)ack_data;
		ack->f1 = t+1;
#if FP_VERBOSE
		printf("ack_test2: f1: %d (%x)\n", ack->f1, ack->f1);
#endif
		*ack_len = sizeof(struct fp_ack_test2);
		ret = FP_ERR_SUCCESS;
		break;
	}
	case FP_CMD_TEST_DATA3: {
		struct fp_cmd_test3 *cmd;
		struct fp_ack_test3 *ack;
		uint16_t t1, t2;
		if (2*((sizeof(struct fp_cmd_test3)+1)/2) != cmd_len) {
			ret = FP_ERR_INVALID_CMD_LEN;
			break;
		}
		cmd = (struct fp_cmd_test3 *)cmd_data;
		t1 = cmd->f1;
		t2 = cmd->f2;
#if FP_VERBOSE
		printf("cmd_test3: f1: %d (%x), f2: %d (%x)\n", t1, t1, t2, t2);
#endif
		ack = (struct fp_ack_test3 *)ack_data;
		ack->f1 = t1+1;
		ack->f2 = t2+1;
#if FP_VERBOSE
		printf("ack_test3: f1: %d (%x), f2: %d (%x)\n", ack->f1, ack->f1, ack->f2, ack->f2);
#endif
		*ack_len = sizeof(struct fp_ack_test3);
		ret = FP_ERR_SUCCESS;
		break;
	}
	case FP_CMD_TEST_TIMEOUT: {
		uint16_t delay = *(uint16_t *)cmd_data;
		_time_delay(delay);
		*ack_len = 0;
		ret = FP_ERR_SUCCESS;
		break;
	}
	case FP_CMD_TEST_ELAPSED_TIME:
	case FP_CMD_TEST_DIFF_SIZES1:
	case FP_CMD_TEST_DIFF_SIZES2: {
		*ack_len = *(uint8_t *)cmd_data;
		memcpy(ack_data, cmd_data+1, *ack_len);
		ret = FP_ERR_SUCCESS;
		break;
	}

	case FP_CMD_BOARD_INFO: {
		struct fp_ack_board_info *ack;
		//TIME_STRUCT ts;
		ack = (struct fp_ack_board_info *)ack_data;
		ack->id = fp_driver.config.board_id;
		ack->hw_vers = fp_driver.config.board_hw_vers;
		ack->pof_vers = fp_driver.config.board_pof_vers;
		ack->sw_vers = fp_driver.config.board_sw_vers;
		if (fp_driver.config.cfg_get_serial) {
			memcpy(ack->serial, fp_driver.config.cfg_get_serial(), sizeof(ack->serial));
		} else {
			memset(ack->serial, '0', sizeof(ack->serial));
		}
		//_time_get_elapsed(&ts);
		ack->uptime = g_uptime;
		*ack_len = sizeof(struct fp_ack_board_info);
		ret = FP_ERR_SUCCESS;
		break;
	}

	case FP_CMD_SET_SERIAL_NUM: {
		if (fp_driver.config.cfg_set_serial) {
			/* предполагается, что в этой функции различные cmd_len будут корректно обработаны */
			fp_driver.config.cfg_set_serial(cmd_data, cmd_len);
		}
		break;
	}

	case FP_CMD_GOTO_BLDR: {

		break;
	}

	case FP_CMD_BOARD_RESET: {
		uint32_t tmp;
		_time_delay(4);
		//tmp = SCB_AIRCR; /* see http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/Cihehdge.html */
		tmp &= 0x00008700;
		tmp |= 0x05FA0004;
		//SCB_AIRCR = tmp;
		*ack_len = 0;
		ret = FP_ERR_SUCCESS;
		break;
	}

	case FP_CMD_BLDR_SET_AUTOBOOT: {

		break;
	}

	case FP_CMD_GOTO_LOWPOWER: {
		/*
		 * если не делать тут паузу, то частота будет понижена пока тикает таймер,
		 * соответственно, таймер сработает позже, ответ на сообщение отослан не будет,
		 * на пару сообщений протокол подвиснет. :)
		 */
		_time_delay(2);
		if (fp_driver.config.pm_lowpower_enter) {
			fp_driver.config.pm_lowpower_enter();
		}
		*ack_len = 0;
		ret = FP_ERR_SUCCESS;
		break;
	}

	default:
		ret = FP_ERR_UNKNOWN_CMD;
	}

        //return ferret_proto_handler_2(cmd, cmd_data, cmd_len, ack_data, ack_len);
	return ret;
}

static void fp_register_common_messages(void)
{
	fp_register_handler(FP_CMD_BOARD_INFO, fp_common_handler);

	fp_register_handler(FP_CMD_SET_SERIAL_NUM, fp_common_handler);

	fp_register_handler(FP_CMD_TEST_DATA1, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_DATA2, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_DATA3, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_TIMEOUT, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_ELAPSED_TIME, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_DIFF_SIZES1, fp_common_handler);
	fp_register_handler(FP_CMD_TEST_DIFF_SIZES2, fp_common_handler);

	fp_register_handler(FP_CMD_GOTO_BLDR, fp_common_handler);
	fp_register_handler(FP_CMD_BOARD_RESET, fp_common_handler);
	fp_register_handler(FP_CMD_BLDR_SET_AUTOBOOT, fp_common_handler);

	fp_register_handler(FP_CMD_GOTO_LOWPOWER, fp_common_handler);
}

void fp_get_driver(FP_DRIVER *drv)
{
	drv=&fp_driver;
}
