#ifndef __LEDS_H
#define __LEDS_H

#include "MDR32F9Qx_config.h"
#include <MDR32Fx.h>
#include "types.h"
#include "MDR32F9Qx_board.h"
#include "config.h"

#define CPLD_IND_LED(x, led)			(((x) & 3) << led) 
#define CPLD_IND_LED_SET(v, x, led)		(v = (v & ~CPLD_IND_LED(-1, led)) | CPLD_IND_LED(x, led)) 

#define USER_LEDS	7
enum led_type {
	LED_CHANEL= 0,
	LED_SEL   = 1,
	LED_ALARM = 2,
	LED_ERR   = 3,
	LED_WORK  = 4,
	LED_BUS   = 5,
	LED_USER  = 6,
	LED_PWR   = 7
};

extern uint8_t Led_Buf[Led_group];

enum led_state {
	LED_OFF	  = 0,
	LED_RED   = 1,
	LED_GREEN = 2
};

extern void led_change	(enum led_type led, enum led_state state, uint8_t ch);
extern void leds_all_on	(void);
extern void leds_all_off(void); 
extern void leds_test(void); 
extern int leds_test_apply(void);

extern void leds_set_timeout(int8_t seconds);
uint8_t leds_get_timeout(void);

extern void leds_init(void);	
extern void process_leds(void);	
#endif /* __LEDS_H */

