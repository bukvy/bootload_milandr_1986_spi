#include "block.h"
#include "module.h"
#include "leds.h"
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_ssp.h>
#include "config.h"
#include "serial.h"
#include "modbus.h"
#include "string.h"

/*****************************************************************************/
/* Local variables                                                           */
/*****************************************************************************/
//static LWGPIO_STRUCT block_state_gpio;
//static uint_16 block_state_isr_handle;

/*****************************************************************************/
/* Previous declaration                                                      */
/*****************************************************************************/
static void block_state_ext_handle(void *data);

//static LWSEM_STRUCT block_apply_sem;
static void block_apply_task(uint32_t initial_data);

/*
 * 
 */
void block_state_init(void)
{
	/*LWGPIO_ISR_DESC_STRUCT isr_desc;
	
	lwgpio_init(&block_state_gpio, BSP_GPIO_BLOCK_STATE, LWGPIO_DIR_INPUT, LWGPIO_VALUE_NOCHANGE);
	
	isr_desc.gpio = BSP_GPIO_BLOCK_STATE;
	isr_desc.gpio_mux = BSP_GPIO_BLOCK_STATE_MUX_GPIO;
	isr_desc.gpio_attr = LWGPIO_ATTR_PULL_DOWN;
	isr_desc.int_flags = (LWGPIO_INT_MODE_RISING | LWGPIO_INT_MODE_FALLING);
	isr_desc.int_prio = 5;
	isr_desc.isr = block_state_ext_handle;
	isr_desc.isr_data = &block_state_gpio;

    //TODO: разобраться с режимом блокировки и правильно всё включить
	block_state_isr_handle = lwgpio_install_isr(&isr_desc);
	
	_lwsem_create(&block_apply_sem, 0);
	block_apply_task_id = _task_create(0, 0, (unsigned long)(&block_template_list[0]));
	block_state_ext_handle(&block_state_gpio);*/
}

/*
 * 
 */
void block_state_deinit(void)
{
}

//TODO: Реализовать
/*
static void block_apply_task(uint32_t initial_data)
{
	for (;;) {
		_lwsem_wait(&block_apply_sem);
		mod_wanted.block_state_enter = 1; 
		mod_wanted.want_anything = 1;
		//mod_apply_block_state();
	}
}
*/

void block_state_apply(void)
{
	//_lwsem_post(&block_apply_sem);
}

//TODO: Реализовать
/*
static void led_block_on(void)
{
	uint8_t t;
	spi_cpld_read8(CPLD_REG_CONF2, &t);
	CPLD_CONF2_BLOCK_SET(t);
	spi_cpld_write8(CPLD_REG_CONF2, t);	
}
*/

/*
 * Обработчик выставления внутренней блокировки
 */
void block_state_int_handle()
{
/*	uint8_t t;
	if (what_happened != BLOCK_OFF) {
		led_block_on();
		block_state_apply();
	}
	
#if 0
	if (what_happened == BLOCK_SW_WDT) {
		reset_board();
		//printf("Software watchdog occurred\n");
	} else if (what_happened == BLOCK_HW_WDT) {
		//printf("Hardware watchdog occurred\n");
	} else if (what_happened == BLOCK_EXCEPTION) {
		reset_board();
		//printf("CPU exception occurred\n");
	} else if (what_happened == BLOCK_FORCE_INT) {
		//printf("blockage is set forcibly\n");		
	}
#endif*/
}

/*
 * Обработчик выставления внешней блокировки
 */
//TODO: Реализовать
/*
static void block_state_ext_handle(void *data)
{
	lwgpio_int_clear_flag((LWGPIO_STRUCT_PTR)data);
	if (lwgpio_get_value((LWGPIO_STRUCT_PTR)data)) {
		block_state_apply();
	}
}
*/
/*
 * Принудительно выставить внутреннюю блокировку.
 * (что очень проблематично для IM)
 */
void block_state_set_int(uint8_t block)
{
	/* необходимо отключать прерывания на время выставления значения */
/*	_int_disable();
	if (block) {
		block_set_forcibly();
	} else {
		uint8_t t;
		block_clear_what_happened();
		spi_cpld_read8(CPLD_REG_CONF2, &t);
		CPLD_CONF2_BLOCK_CLR(t);
		spi_cpld_write8(CPLD_REG_CONF2, t);
	}
	_int_enable();
  */  
	/* после этого может сработать прерывание о блокировке */                                                                                                                                                                      
}

/*
 * Возвращает внутреннее состояние блокировки.
 * 0 - если внутренняя блокировка не выставлена, 
 * иначе возвращается причина блокировки (см. BLOCK_WHAT_HAPPENED)
 */
uint8_t block_state_get_int(void)
{
      //return (uint8_t)block_get_what_happened();
      //TODO:  Нужно реализовать
      return 0;
}

/*
 * Возвращает внешнее состояние блокировки.
 * 0 - UNBLOCK_STATE
 * 1 - BLOCK_STATE
 */
uint8_t block_state_get_ext(void)
{
	//if (block_state_get_int())
		return 0;
	//return (uint8_t)lwgpio_get_value(&block_state_gpio);
}
