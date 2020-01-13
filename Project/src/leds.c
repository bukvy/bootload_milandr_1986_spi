#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_dma.h>
#include "leds.h"
#include "module.h"
#include "MDR32F9Qx_it.h"
uint8_t Led_Buf[Led_group];
uint8_t timeout=0;
void led_change	(enum led_type led, enum led_state state, uint8_t ch)
{
    uint8_t i, led_num, t=0;

    if (led == LED_USER) {}

    i = ch/4;
    led_num= (ch % 4)*2;
    if(led==LED_CHANEL)	{
        if(ch<12){
            i = (13-ch)/4; 	led_num = ((13-ch) % 4)*2;
        } else {
            i = (ch+10)/4; 	led_num = ((ch+10) % 4)*2;
        }
        if(ch==10) {i = 0/4; 	led_num = ((0) % 4)*2;		}
        if(ch==11) {i = 3/4; 	led_num = ((3) % 4)*2;		}
        if(ch==22) {i = 2/4; 	led_num = ((2) % 4)*2;		}
        if(ch==23) {i = 1/4; 	led_num = ((1) % 4)*2;		}
    }
    if (led==LED_SEL)	{       i =(15-ch)/4; 	led_num= ((15-ch) % 4)*2;		}
    if (led==LED_ALARM)	{	i =(20+ch)/4; 	led_num= ((20+ch) % 4)*2;		}
    if (led>LED_ALARM)	{	i =(22-led)/4; 	led_num= ((22-led) % 4)*2;		}

    t = Led_Buf[10+i];
    CPLD_IND_LED_SET(t, state, led_num);
    Led_Buf[10+i] = t;
}


void leds_all_on(void){

    board_set.settings.is_leds_on = 1;

}

void leds_all_off(void){

    board_set.settings.is_leds_on = 0;
}

enum led_state leds_get_user_led(void)
{
    return board_set.settings.user_led_type;//cfg_get_user_led_type();
}

uint8_t leds_get_is_all_on(void)
{
    return board_set.settings.is_leds_on;
}

int leds_test_process = 0;
tick tmo = 0;//setTimeout(0);
int led_type = LED_OFF;


int leds_test_apply(void) {

      int swv = 0;
      if (leds_test_process == 0)       return 0;       //Тест завершён
      if (!isTimeout(tmo))              return 1;       //Тест в процессе

#define MAX_LEDS_CNT 32
      leds_test_process++;

      swv = (leds_test_process-1) % MAX_LEDS_CNT;
      switch (swv) {
      case 1:   led_change(LED_ERR,  led_type, 0);      if (led_type!= LED_OFF) tmo = setTimeout(100);    break;
      case 2:   led_change(LED_WORK,  led_type, 0);     if (led_type!= LED_OFF) tmo = setTimeout(100);    break;
      case 3:   led_change(LED_BUS, led_type, 0);       if (led_type!= LED_OFF) tmo = setTimeout(100);    break;
      case 4:   led_change(LED_USER, led_type, 0);      if (led_type!= LED_OFF) tmo = setTimeout(100);    break;

      case 5: case 6:
                led_change(LED_SEL,   led_type, swv-5);
                led_change(LED_ALARM, led_type, swv-5);


                if (led_type!= LED_OFF) tmo = setTimeout(100);
                break;

      case  7: case  8: case  9: case 10: case 11: case 12:  case 13: case 14:
      case 15: case 16: case 17: case 18:
      //case 19: case 20: case 21: case 22:
      //case 23: case 24: case 25: case 26: case 27: case 28: case 29: case 30:
                led_change(LED_CHANEL,  led_type, swv-7);
                led_change(LED_CHANEL,  led_type, swv-7+12);
                if (led_type!= LED_OFF) tmo = setTimeout(100);
                break;

      case 31:
                tmo = setTimeout(3000);
                switch (leds_test_process / MAX_LEDS_CNT) {
                  case 0: led_type = LED_OFF;     break;
                  case 1: led_type = LED_GREEN;   break;
                  case 2: led_type = LED_OFF;     break;
                  case 3: led_type = LED_RED;     break;
                  case 4: led_type = LED_OFF;     break;
                }
                break;
      }

      if (leds_test_process > MAX_LEDS_CNT*5)
          leds_test_process = 0;

      SSP_Target=tLEDS;
      DMA_TX_FLAG = 0;
      DMA_RX_FLAG = 0;
      InitDMA_SSP2(Led_Buf,Led_group);

      return (leds_test_process) ? 1 : 0;

      /*

	for (i = 0; i < GROUPS_COUNT; i++) {
            led_change(LED_SEL, LED_RED, i);
            //led_change(LED_SEL,  LED_RED, i);
	}

	for (i = 0; i < GROUP_SIZE; i++) {
            //led_change(LED_SEL,   LED_RED, i);
            //led_change(LED_SEL,  LED_OFF, i);
	}
	*/
	//_time_delay(1000);


	/* последовательное включение всех светодиодов зеленым с задержкой */
	/*

        leds_all_on_regs();

	led_on_apply(LED_ERR,  LED_GREEN, 0);		_time_delay(300);
	led_on_apply(LED_WORK, LED_GREEN, 0);		_time_delay(300);
	led_on_apply(LED_BUS,  LED_GREEN, 0);		_time_delay(300);
	led_on_apply(LED_USER, LED_GREEN, 0);		_time_delay(300);

	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_CASC_WORK, LED_GREEN, i);	_time_delay(300);
		led_on_apply(LED_CASC_SEL,  LED_GREEN, i);	_time_delay(300);
	}

	for (i = 0; i < GROUP_SIZE; i++) {
		led_on_apply(LED_INOUT,  LED_GREEN, i);		_time_delay(300);
		led_on_apply(LED_STAT,   LED_GREEN, i);		_time_delay(300);
	}

	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_ALARM_PS_ON, LED_GREEN, i);	_time_delay(300);
	}

	_time_delay(3000);
	*/
	/* отключаем все светодиоды */
        /*
	leds_all_off_apply();
	led_on_apply(LED_ERR,  LED_OFF, 0);
	led_on_apply(LED_WORK, LED_OFF, 0);
	led_on_apply(LED_BUS,  LED_OFF, 0);
	led_on_apply(LED_USER, LED_OFF, 0);

	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_CASC_WORK, LED_OFF, i);
		led_on_apply(LED_CASC_SEL,  LED_OFF, i);
	}

	for (i = 0; i < GROUP_SIZE; i++) {
		led_on_apply(LED_STAT,   LED_OFF, i);
		led_on_apply(LED_INOUT,  LED_OFF, i);
	}

	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_ALARM_PS_ON, LED_OFF, i);
	}
        */

	/* последовательное включение всех светодиодов красным с задержкой */
        /*
	leds_all_on_regs();

	led_on_apply(LED_ERR,  LED_RED, 0);		_time_delay(300);
	led_on_apply(LED_WORK, LED_RED, 0);		_time_delay(300);
	//block_state_set_int(1);
	leds_test_block_led(1);					_time_delay(300);
	led_on_apply(LED_BUS,  LED_RED, 0);		_time_delay(300);
	led_on_apply(LED_USER, LED_RED, 0);		_time_delay(300);


	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_CASC_WORK, LED_RED, i);	_time_delay(300);
		led_on_apply(LED_CASC_SEL,  LED_RED, i);	_time_delay(300);
	}

	for (i = 0; i < GROUP_SIZE; i++) {
		led_on_apply(LED_INOUT,  LED_RED, i);		_time_delay(300);
		led_on_apply(LED_STAT,   LED_RED, i);		_time_delay(300);
	}

	for (i = 0; i < GROUPS_COUNT; i++) {
		led_on_apply(LED_ALARM_PS_ON, LED_RED, i);	_time_delay(300);
	}

	_time_delay(3000);

	leds_all_off_apply();
	leds_test_block_led(0);
		*/


}


void leds_test(void){

    leds_test_process = 1;

}

void leds_set_timeout(int8_t seconds){
}

void led_on(enum led_type led, enum led_state state, uint8_t ch)
{
	/*if (led < LEDS_COUNT) {
		leds_wanted.led[led] 	= state;
		leds_wanted.led_ch[led] = ch;
		leds_wanted.want_anything = 1;
	}*/

      led_change(led,state,ch);
}

void led_off(enum led_type led, uint8_t ch)
{
	led_on(led,100,ch);
}

uint8_t leds_get_timeout(void)
{
	return timeout;
}

void leds_init(void){
}

void process_leds(void){

      static uint32_t next_show = 3000;
      static uint8_t prev_bus_show_state = 0;
      static uint8_t sel_casc = 0;

      uint8_t i,j;
      uint32_t p_state[2]={0,0};
      enum ch_state ch_state;
      uint32_t tick = TimerCounter;
      enum led_state led_state = LED_OFF;

      leds_all_on();

      if (!leds_test_apply()) {

          if (board_set.settings.is_leds_on) {
              if (bus_show_state != prev_bus_show_state) {
                  prev_bus_show_state = bus_show_state;
                  led_change(LED_BUS, bus_show_state, 0);
              }   bus_show_state = 0;

// Koval             led_change(LED_WORK, LED_GREEN, 0);
              led_change(LED_WORK, LED_RED, 0);
                            led_change(LED_CHANEL, LED_GREEN, 5);

          } else {
              led_change(LED_WORK, LED_OFF, 0);
              led_change(LED_BUS,  LED_OFF, 0);
              //led_change(LED_PWR,  LED_OFF, 0);
          }


          if ((tick > next_show)&&(SSP_Target<tDO)) {
              next_show = tick + LEDS_PERIOD_SEC * 3000;

              if (board_set.settings.is_leds_on) {

                  for (i = 0; i < 2; i++) {
//Koval                      led_state = LED_OFF;
                        led_state = LED_GREEN;
                      p_state[i] = 0;
                      if (board_set.settings.is_leds_on)
                          for (j = 0; j < 24; j++) {
                              ch_state = port_get_state(i*24+j);
                              if (ch_state != CH_STATE_OFF)	p_state[i]|=  1<<j;
                          }
                  }
              } else {
                  p_state[0] = p_state[1] = 0;
              }



              //Смещение к следующему
              if (p_state [(sel_casc + 1) % 2])
                  sel_casc=(sel_casc + 1) % 2;


              led_change(LED_SEL, LED_OFF, (sel_casc + 1) % 2);
              led_change(LED_SEL, (p_state[sel_casc]) ? LED_GREEN : LED_OFF , sel_casc);



              for (i = 0; i < 24; i++)
              {
                  led_state = LED_OFF;
                  uint16_t value=0;
                  ch_state = port_get_state(sel_casc*24+i);
                  port_get_value(sel_casc*24+i,&value);

                  if (board_set.settings.is_leds_on) {
                      if (ch_state != CH_STATE_OFF) {
                          led_state = LED_GREEN;
                          if (value != CH_NORMAL)
                              led_state = LED_RED;
                      }
                  }
                  led_change(LED_CHANEL, led_state, i);
              }

              //sel_casc++;
              //if(sel_casc>1)sel_casc=0;

              SSP_Target=tLEDS;
              DMA_TX_FLAG = 0;
              DMA_RX_FLAG = 0;
              Led_Buf[13]=0x04;
              Led_Buf[14]=0x04;
              InitDMA_SSP2(Led_Buf,Led_group);
              //led_apply();
          }
      }
}