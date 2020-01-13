#ifndef SPI_FERRET_PROTO_H_
#define SPI_FERRET_PROTO_H_

#include "ferret_proto.h"

#define SPI_FP_HEADER_LEN  6
#define SPI_FP_PAYLOAD_LEN 512
#define SPI_FP_PACKET_LEN  (SPI_FP_HEADER_LEN + SPI_FP_PAYLOAD_LEN)

struct spi_fp_config {
	/* обозначение пина, запараллеленного с CS-ом SPI модуля */
	uint32_t cs_gpio;     /* (GPIO_PORT_C | GPIO_PIN18) */
	uint32_t cs_gpio_mux; /* (LWGPIO_MUX_C18_GPIO) */
	
	char *spi_channel;   /* "spi2:" */
	volatile struct SPI_MemMap *spi_base_ptr; /* SPI2_BASE_PTR */
	uint32_t dma_channel_rx; /* 12 Rx DMA channel (interrupt only) */
	uint32_t dma_source_rx;  /* 20 Rx eDMA source (interrupt only) */
	uint32_t dma_channel_tx; /* 13 Tx DMA channel (interrupt only) */
	uint32_t dma_source_tx;  /* 21 Tx eDMA source (interrupt only) */

	/* сигнал о готовности данных для передачи центральному модулю */
	uint32_t ext_spi_int_gpio;     /* (GPIO_PORT_C | GPIO_PIN16) */
	uint32_t ext_spi_int_gpio_mux; /* (LWGPIO_MUX_C16_GPIO) */
	
	/* сигнал проверки текущей операции прием/передача. 0 - прием, 1 - передача */
	uint32_t ext_spi_chk_gpio;     /* (GPIO_PORT_C | GPIO_PIN17) */
	uint32_t ext_spi_chk_gpio_mux; /* (LWGPIO_MUX_C17_GPIO) */
};


extern uint32_t rx_buf[FP_PACKET_LEN + FP_PACKET_LEN/2];
extern uint16_t tx_buf[FP_PACKET_LEN + FP_PACKET_LEN/2];

extern void spi_fp_init(struct spi_fp_config);
extern void spi_fp_remove(struct spi_fp_config config);

int spi_fp_read(void);
void spi_fp_write(void);

extern int spi_fp_get_module_num(void);

#endif /* SPI_FERRET_PROTO_H_ */
