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
#include "modbus.h"
#include "spi_ferret_proto.h"
#include "crc_hw.h"
#include "string.h"

#define SPI_FP_LOOPBACK_MODE 0

#define SPI_FP_DISABLE_RX 0
#define SPI_FP_DISABLE_TX 0

#define SPI_FP_DISABLE_RXTX_MODE_SWAP 0

#define SPI_FP_DISABLE_FP 0

#define SPI_FP_DISABLE_HOLE_SPI 0

#define SPI_FP_RX_MODE SPI_CLK_POL_PHA_MODE1
#define SPI_FP_TX_MODE SPI_CLK_POL_PHA_MODE0

#define SPI_FP_MAX_RX_SHIFT (8)

/* SPI register access defines */
#define DSPI_SR_RFOF_MASK       0x80000u
#define DSPI_SR_TFFF_MASK       0x2000000u
#define DSPI_SR_TFUF_MASK       0x8000000u
#define DSPI_PUSHR_TXDATA_MASK  0xFFFFu
#define DSPI_PUSHR_TXDATA_SHIFT 0
#define DSPI_PUSHR_TXDATA(x)    (((x) << DSPI_PUSHR_TXDATA_SHIFT) & DSPI_PUSHR_TXDATA_MASK)

uint32_t rx_buf[FP_PACKET_LEN + FP_PACKET_LEN/2];
uint16_t tx_buf[FP_PACKET_LEN + FP_PACKET_LEN/2];

void spi_fp_recv_isr(void);
static void spi_fp_dma_setup(uint32_t size);
static void spi_fp_dma_setup_tx(uint32_t size);

void spi_fp_init(struct spi_fp_config config)
{

}



static void spi_fp_dma_setup(uint32_t size)
{

}

static void spi_fp_dma_setup_tx(uint32_t size)
{

}

uint8_t check_is_read_now(void)
{
// Koval this pin is for strobe on shift registers of tDO	return !PORT_ReadInputDataBit(MDR_PORTE, PORT_Pin_3);
}

/*
 * CS сработает как при чтении, так и при записи,
 * поэтому вводим флаг, разделяющий эти операции
 */
void spi_fp_recv_isr(void)
{
	//const FP_DRIVER_PTR fp_driver = fp_get_driver();
	uint32_t param;
	static uint8_t is_read_now = 1;

	if (is_read_now && !check_is_read_now()) {
		/* мы думали, что сейчас CPU должен передавать нам данные, а на самом деле - он их забирает.
		 * как именно такая ситуация может возникать не очень понятно.
		 * отдавать нам CPU нечего, следующая операция должна быть как раз приемом с CPU, поэтому
		 * инвертируем переменную is_read_now и выполним подготовительную работу
		 */
		is_read_now = 0;
	/* } else if (!is_read_now && check_is_read_now()) { */
		/* мы думали, что сейчас CPU должен у нас забирать, а на самом деле - он передает нам данные.
		 * такое может случится, если, к примеру, ранее прерывание ext_spi_int не было выставлено вовремя.
		 * в связи с этим, не выполнялись подготовительные процедуры и отправленные CPU данные не приняты.
		 * поэтому тут сейчас исключительно готовим контроллер к следующей операции передачи данных от CPU
		 * (т.е. в данном случае ничего не делаем, все верно сработает по текущим условиям).
		 * на CPU тогда возникнут 2 ошибки "no answer".
		 */
	}

	/* CS стал неактивным */
	if (is_read_now == 1) {
		/* прием закончился */
		is_read_now = 0;

		/* отключаем DMA канал по приему */
		spi_fp_dma_setup(0);

#if (!SPI_FP_DISABLE_RXTX_MODE_SWAP)
		/* возвращаем настройку SPI модуля на передачу */
		//param = SPI_FP_TX_MODE;
		//ioctl(fp_spi, IO_IOCTL_SPI_SET_MODE, &param);
#endif

		/* отпускаем задачу обработки принятых данных */
		//fp_driver->recv_start();
	} else {
		/* передача закончилась */
		is_read_now = 1;
		/* возвращаем на место сигнал EXT_SPI_INT */
		//lwgpio_set_value(&ext_spi_int, 0);

		/* зануляем буфер по приему не весь, но начало, которое
		 * проверяется потом на нули
		 */
		memset(rx_buf, 0, SPI_FP_MAX_RX_SHIFT);
		/* зануляем буфер по передаче, чтобы на MISO не выдавались
		 * левые данные
		 */
		memset(tx_buf, 0, FP_PAYLOAD_LEN);

		/* возвращаем настройку SPI модуля на прием */
		//param = SPI_FP_RX_MODE;
		//ioctl(fp_spi, IO_IOCTL_SPI_SET_MODE, &param);

		/*
		 * данные отправлены, начинаем ждать нового запроса. сбрасываем DMA канал,
		 * чтобы он начал писать данные опять с нулевого адреса rx_buf
		 */
		spi_fp_dma_setup(FP_PAYLOAD_LEN);

		/* настраиваем передающий DMA канал, дабы при чтении
		 * на MISO выдавался tx_buf (нули)
		 */
		spi_fp_dma_setup_tx(FP_PAYLOAD_LEN);
	}
}

int spi_fp_read(void)
{
	uint32_t t, i, rx_shift = 0;

	fp_driver.counters.rx_packets += 1;

	/*
	 * rx_buf - 32-х битный, поэтому все дальнейшие операции выглядят довольно странно...
	 */

	/*
	 * длина всего пакета указана в словах
	 */
	t = (rx_buf[rx_shift] & 0x0FFF)*2;

	if (t == 0) {
		/*
		 * kinetis может слишком шустро начать принимать данные,
		 * из-за чего в начале буфера окажутся нули,
		 * попробуем это отловить. хоть и костыль -_-
		 */
		for (i = 0; i < SPI_FP_MAX_RX_SHIFT; i++) {
			if (rx_buf[i] != 0) break;
		}
		rx_shift = i;
		t = (rx_buf[rx_shift] & 0x0FFF)*2;
	}
/*
#if (!SPI_FP_DISABLE_RX)
	if ((t == 0) || (t >= FP_PACKET_LEN)
			|| ((FP_PACKET_LEN - DMA_CITER_ELINKNO(dma_channel_rx)) < t)) {
		return -FP_ERR_INVALID_PACKET_LEN;
	}
#endif*/

	/*
	 * копируем принятые DMA данные в буфер на отдачу, надеемся, что данные там таки целые
	 * делаем перестановку байт согласно используемому в системе endian
	 */
	for (i = rx_shift; i < ((t/2)+1); ++i) {
		fp_driver.rx_buf.buf[2*i +1] = (uint8_t)(rx_buf[i] >> 8);
		fp_driver.rx_buf.buf[2*i +0] = (uint8_t)(rx_buf[i]);
	}

	fp_driver.counters.rx_bytes += t;

	return 0;
}

void spi_fp_write(void)
{
	//const FP_DRIVER_PTR fp_driver = fp_get_driver();
	int i;
	uint16_t len, b;
	uint32_t crc;

	/*
	 * Вставляем CRC сумму
	 */
	len = fp_driver.tx_buf.packet.packet_len * FP_WORD_SIZE - FP_WORD_SIZE;
	crc = ModBusCRC16(&fp_driver.tx_buf.buf[FP_HEADER_LEN], len);
	crc &= 0xFFFF;
	//crc = ((crc & 0xFF00) >> 8) | ((crc & 0x00FF) << 8);
	fp_driver.tx_buf.packet.crc = crc;

	for (i = 0; i <= fp_driver.tx_buf.packet.packet_len; ++i){
		tx_buf[i] = (*(uint16_t*)&fp_driver.tx_buf.buf[2*i]);//^0xFFFF;
    tx_buf[i] = tx_buf[i]^0xFFFF;
	}

  while((SSP_GetFlagStatus(MDR_SSP1,SSP_FLAG_RNE))&&(i<100)){
      b=SSP_ReceiveData(MDR_SSP1);
      i++;
    }

	InitDMA_SSP1_tx((unsigned char*)&tx_buf,fp_driver.tx_buf.packet.packet_len + 1);
	//spi_fp_dma_setup_tx(fp_driver.tx_buf.packet.packet_len + 1);

	while (PORT_ReadInputDataBit(MDR_PORTF, PORT_Pin_2) == Bit_RESET) {}
  PORT_ResetBits(MDR_PORTB, PORT_Pin_5);
  PORT_SetBits(MDR_PORTB, PORT_Pin_5);


	//lwgpio_set_value(&ext_spi_int, 0);
	//lwgpio_set_value(&ext_spi_int, 1);

	fp_driver.counters.tx_packets += 1;
	fp_driver.counters.tx_bytes += 2*fp_driver.tx_buf.packet.packet_len;
}
