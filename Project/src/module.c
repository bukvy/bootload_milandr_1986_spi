#include "module.h"
#include "leds.h"
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_ssp.h>
#include <MDR32F9Qx_wwdg.h>
#include "config.h"
#include "serial.h"
#include "modbus.h"
#include "string.h"
#include "ferret_proto.h"
#include "flash.h"
#include "fp.h"
#include "spi_ferret_proto.h"

volatile uint8_t bus_show_state = 0;
uint8_t data_type = PHYSICAL_VALUE; 
uint32_t g_debug_flags = 0x00;
uint32_t g_debug_channel = 0x00;

uint32_t delay_get_dac_status;
uint32_t g_mod_buf_raw[PORTS_COUNT];

float g_mod_buf_val_freq[PORTS_COUNT];

uint16_t g_mod_buf_val[PORTS_COUNT];

uint16_t g_mod_ch_period[PORTS_COUNT]={ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

mod_wanted_t mod_wanted = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 
							{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 
							  0, 0, 0, 0, 0 };



/* Che:
Формат масивов new_mode и last_mode следующий:
1. Начинается масив с конца, то есть с 18 элемента;
2. В масиве лежат структуры по 5 bit:
  2.1. 1 бит - питание группы из 4 входов; [1]
  2.2. 4 бита - источник 10 мА;            [2][3][4][5]
3. Структуры лежат так:
Номер байта             17                                  16              
              [1][2][3][4][5] [1][2][3]         [4][5] [1][2][3][4][5][1] 

*/
uint8_t mode_change=0;
unsigned char new_mode[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char last_mode[18]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};



uint8_t cfg_get_port_mode(uint8_t ch)
{
    return board_set.settings.ao_mode[ch];
}

uint8_t cfg_get_port_state(uint8_t ch)
{
    return board_set.settings.ao_status[ch];
}

uint8_t cfg_get_port_block_state(uint8_t num)
{
	if (num >= PORTS_COUNT) return 0;
	return board_set.settings.port_block_state[num];
}

void cfg_set_port_block_state(uint8_t num, uint8_t val)
{
	if (num >= PORTS_COUNT) return;
	board_set.settings.port_block_state[num] = val;
}



uint8_t cfg_get_port_block_mode(uint8_t num)
{
	if (num >= PORTS_COUNT) return 0;
	return board_set.settings.port_block_mode[num];  
}

void cfg_set_port_block_mode(uint8_t num, uint8_t val)
{
	if (num >= PORTS_COUNT) return;
	board_set.settings.port_block_mode[num] = val;   
} 


void port_set_block_state(uint8_t num, enum ch_state state, enum ch_mode mode) 
{
	cfg_set_port_block_state(num, state);
	cfg_set_port_block_mode (num, mode);
}

void port_get_block_state(uint8_t num, enum ch_state *state, enum ch_mode *mode) 
{
	*mode  = (enum ch_mode)cfg_get_port_block_mode(num);
	*state = (enum ch_state)cfg_get_port_block_state(num);
}

void module_apply_block_state(void) 
{
	uint8_t i;
	for (i = 0; i < PORTS_COUNT; i++) 
	{	
		port_set_state(i, cfg_get_port_block_state(i));
		port_set_mode (i, cfg_get_port_block_mode(i));
	}
}



void cfg_update_config(void);


void cfg_write_config(void)
{
    StoreField(EE_ADR,(unsigned char*) &board_set.settings, sizeof(cfg_params));
    cfg_update_config();
}

void cfg_erase_config(void)
{
    memset(&board_set.settings,0,sizeof(cfg_params));
    StoreField(EE_ADR,(unsigned char*) &board_set.settings,sizeof(cfg_params));
} 

void cfg_update_config(void)
{
      int i=0;

      //Che: добавил переинициализацию при изменении настроект
      board_set.settings.diag.leds.is_leds_on == 0 ?  leds_all_off() : leds_all_on();
      leds_set_timeout(board_set.settings.leds_timeout);
      led_on(LED_USER, board_set.settings.user_led_type, 0);

      for (i=0; i<PORTS_COUNT; i++) {
            port_queue_set_state    (i, board_set.settings.ao_status[i]);
            port_queue_set_mode     (i, board_set.settings.ao_mode[i]);
      }
}



#pragma pack(push, 1)
typedef struct PortProcessDesc {
    uint32_t tmo;
    uint8_t  kind;
} PortProcessDesc_t ;
#pragma pack(pop)

static PortProcessDesc_t port_p[PORTS_COUNT];


uint32_t GetTickCount(void) {   return get_ticks();   }


tick setTimeout(u32 milliseconds)
{
    return milliseconds + GetTickCount();
}
int  isTimeout(tick timeout)
{
    tick t = GetTickCount();
    if (timeout < 0x80000000)
            return ((timeout <= t) && (t < timeout + 0x80000000));
    else    return ((timeout <= t) || (t < timeout - 0x80000000));
}





// ******************* для алгоритма проверки flash *********************
#define SPI_FRAM_PG_SIZE        128 //0x100           // Макс. размер для одной операции записи/чтения
#define SPI_FRAM_TOTAL_SIZE     0x8000          // Размер FRAM
// **********************************************************************
/**/
unsigned long fram_test(unsigned short *eval)
{
    unsigned long i, res;
    unsigned long adr;
    unsigned char buf[SPI_FRAM_PG_SIZE];
    unsigned char buf2[SPI_FRAM_PG_SIZE];
    unsigned long errcnt = 0;

    Init_ClkDiv(20);

    for (adr = 0; adr < SPI_FRAM_TOTAL_SIZE; adr += SPI_FRAM_PG_SIZE) //пишем весь FRAM кусками по (SPI_FRAM_PG_SIZE) байт
    {
        for (unsigned char j = 0; j < 2; j++) //Проверка выполняется два раза что позволяет не разрушить данные во FRAM
        {
                     board_set.settings.diag.memtest_info.process=(adr*100)/SPI_FRAM_TOTAL_SIZE +1;
            //Сброс WDT
            WWDG_SetCounter(0x7F);

            //Чтение куска данных из FRAM
            res = ReadFlashBuf(adr,buf,SPI_FRAM_PG_SIZE);
              
            for (i = 0; i < SPI_FRAM_PG_SIZE; i++)
                buf[i] ^= 0xFF; //инвертируем и сохраняем в буфер
            (*eval) += 1;

            //Запись куска данных из FRAM
            res = WriteFlashBuf(adr, buf, SPI_FRAM_PG_SIZE);
            (*eval) += 1;

            //Чтение куска данных из FRAM
            res = ReadFlashBuf(adr,buf2, SPI_FRAM_PG_SIZE);

            for (i = 0; i < SPI_FRAM_PG_SIZE; i++)
                if (buf[i] != buf2[i])   //Проверяем данные
                    errcnt++;

            (*eval) += 1;
        }
    }

    return errcnt;

}
/**/


void SetOn10ma_toCfg(uint32_t port, uint8_t isOn10mA, uint8_t isOn2mA)
{
    unsigned char bit_pwr=0, bit_ch=0; 
        
    bit_pwr = (port/GROUP_SIZE) * 5;                            //Позиция структуры * размер структуры
    bit_ch  = (port/GROUP_SIZE) * 5 + port%GROUP_SIZE + 1;      //Позиция структуры * размер структуры + смещение в структуре (структура 5 бит)
      
   
    //Если нужно, включаем питание группы. Если нет, выключаем 
    if (isOn2mA)
          new_mode[17-(bit_pwr/8)]|= (1 << (bit_pwr % 8));
    else  new_mode[17-(bit_pwr/8)]&=~(1 << (bit_pwr % 8));
    
     
    //Включаем/выключаем источник на 10мА
    if (isOn10mA == 0)
          new_mode[17-(bit_ch/8)]&=~(1 << (bit_ch % 8));	//выкл
    else  new_mode[17-(bit_ch/8)]|= (1 << (bit_ch % 8));       //вкл
}
 

uint8_t WriteDO(void)
{
    unsigned char pos=0,cs=0; 
    
    memset(new_mode,0,sizeof(new_mode));
    for (int i=0; i<GROUPS_COUNT; i++)
    {
        if (cfg_get_port_state(i*4))
            new_mode[17-cs]|= (0x01<<(pos&0x07));
        
        pos++;
        if(pos>7){ pos=0; cs++;}
        for (int j=0; j<GROUP_SIZE; j++)
        {
            switch (cfg_get_port_mode(i*GROUP_SIZE+j)) {
              case CH_MODE_DI_DC_2MA: 	
              case CH_MODE_DI_IEC: 	
                new_mode[17-cs]|=(0x01<<(pos&0x07));	//выключить источник 10мА для соответствующего канала
                break;
              case CH_MODE_DI_DC_10MA:	                //включить источник 10мА для соответствующего канала
                break;
            }
            pos++;
            if(pos>7) { pos=0; cs++; }
        }
    }
    
    if (SSP_Target<tDO)
        for (int i=0;i<18;i++)
            if (new_mode[i]!=last_mode[i])
            {
                memcpy(last_mode,new_mode,sizeof(new_mode));
                SSP_Target=tDO;
                ssp2_cnt=0;
                InitDMA_SSP2(last_mode,Led_group);
                return 0;
            }
    

    return 0;
}

volatile unsigned char b[18];


volatile unsigned char parced_values[PORTS_COUNT];


uint8_t ReadDI(void)
{
    unsigned char i=0;
    /*static*/
    uint8_t num_byte = 0,num_bit=0, mask = 0;

    if ((SSP_Target<=tDI)&&(!SSP_GetFlagStatus(MDR_SSP2,SSP_FLAG_BSY)))
    {
        while((SSP_GetFlagStatus(MDR_SSP2,SSP_FLAG_RNE))&&(i<18)){  // that is because DMA on recive does not work as Artem said
            DstBuf2[ssp2_cnt++]=SSP_ReceiveData(MDR_SSP2);
            if (ssp2_cnt>17){
                ssp2_cnt=0;
            }
            i++;
        }
        SSP_Target=tDI;
        ssp2_cnt=0;
        InitDMA_SSP2(last_mode,Led_group);
    }
    
    //Разбор полученых значений
    for (i=0; i<PORTS_COUNT; i++)
    {
        if (cfg_get_port_state(i)) 
        {
            num_byte= (i*3)/8;
            num_bit=7-((i*3)-num_byte*8);
            mask = ((DstBuf2[num_byte]>>(num_bit&0x0F))&0x01);
        
            if (num_bit!=0) num_bit--; else {num_bit=7;num_byte++;}
            mask|= ((DstBuf2[num_byte]>>(num_bit&0x0F))&0x01)<<1;
        
            if (num_bit!=0) num_bit--; else {num_bit=7;num_byte++;}
            mask|= ((DstBuf2[num_byte]>>(num_bit&0x0F))&0x01)<<2;
          
            switch (cfg_get_port_mode(i)) {
                case CH_MODE_DI_IEC_CC: {
                      //Только определение неисправности
                      if ((mask&0x07)==0x07)  parced_values[i] = CH_ALARM; else
                      if ((mask&0x07)==0x02)  parced_values[i] = CH_NORMAL; 
                      else                    parced_values[i] = CH_FAIL;
                      break;                  
                }
                case CH_MODE_DI_IEC: {
                      parced_values[i] = ((mask&0x07)==0x07) ? CH_ALARM : CH_NORMAL; 
                      break;                  
                }              
                case CH_MODE_DI_DC_10MA:  
                case CH_MODE_DI_IC_10MA: {
                      parced_values[i] = ((mask&0x07)==0x07) ? CH_NORMAL : CH_ALARM; 
                      break;
                }
                case CH_MODE_DI_DC_2MA:  
                case CH_MODE_DI_IC_2MA: {
                      parced_values[i] = ((mask&0x07)==0x02) ? CH_NORMAL : CH_ALARM; 
                      break;
                }
                case CH_MODE_DI_DC_10MA_CC:  
                case CH_MODE_DI_IC_10MA_CC: {
                      parced_values[i] = CH_ALARM;      //6: n; 7: o; 0: kz,a
                      if ((mask&0x07)==0x07)  parced_values[i] = CH_OBRYV;
                      if ((mask&0x07)==0x06)  parced_values[i] = CH_NORMAL;
                      break;
                }                
                case CH_MODE_DI_DC_10MA_CC_KZ:  
                case CH_MODE_DI_IC_10MA_CC_KZ: {
                      parced_values[i]=CH_KZ;     //6: N, 2: A, 7: O, 0: KZ
                      if ((mask&0x07)==0x07)  parced_values[i] = CH_OBRYV;
                      if ((mask&0x07)==0x02)  parced_values[i] = CH_ALARM;
                      if ((mask&0x07)==0x06)  parced_values[i] = CH_NORMAL;
                      break;
                }
                case CH_MODE_DI_IEC_CC_KZ: {
                      parced_values[i]=CH_KZ;     //6: N, 7: A, 2: O, 0: KZ
                      if ((mask&0x07)==0x02)  parced_values[i] = CH_OBRYV;
                      if ((mask&0x07)==0x07)  parced_values[i] = CH_ALARM;
                      if ((mask&0x07)==0x06)  parced_values[i] = CH_NORMAL;
                      break;
                }
                default: {                        
                    parced_values[i]=CH_KZ;     
                    if ((mask&0x07)==0x07)  parced_values[i] = CH_OBRYV;
                    if ((mask&0x07)==0x02)  parced_values[i] = CH_ALARM;
                    if ((mask&0x07)==0x06)  parced_values[i] = CH_NORMAL;
                    break;
                }
            }
            //parced_values[i] = mask;
	} else 
            parced_values[i] = 0;
    }
    
    return 0;
}

uint32_t get_ticks(void) {
    return TimerCounter;
}



/**/
volatile uint32_t exchange_period = MIN_EXCHANGE_PERIOD;

//Обновление состояния портов
void di_refresh_ports_ic(void)
{
    #define ON_WAIT_TMO             2 //2 мс (судя по осцилографу) нужно на включение, но нужно протестировать!
    #define OFF_WAIT_TMO            ((exchange_period>ON_WAIT_TMO) ? exchange_period-ON_WAIT_TMO : 0)
    #define CALC_ITERATIONS         1

    #define UPDATE_KIND_VALS    0x01
    #define UPDATE_KIND_PULSE   0x02
    #define UPDATE_KIND_ERRORS  0x04
    #define ENABLE_PRINT_TMO    0
    
    static uint8_t start_config = 1;
    static uint8_t vals[CALC_ITERATIONS][PORTS_COUNT];

    static uint8_t on2mA[PORTS_COUNT/4];

    //uint32_t vals_ok = 0;
    int i,p,j;
    //int32_t LMHOUT[3];

    //static uint32_t last_read_adc = 0;


    uint16_t update_kind = 1;
    static uint16_t init_test = 0;

#if ENABLE_PRINT_TMO
    static uint32_t tmo = 0;
    uint32_t hw_ticks_e, hw_ticks = GetTickCount();
#endif

    //memset(on2mA,0,PORTS_COUNT/4);
    
    ///--- Переконфигурирование портов -----------------
    #define SKIP_IT 10
    if (init_test<SKIP_IT) {
        if (++init_test == SKIP_IT)
            for (i=0; i<PORTS_COUNT; i++) {
                port_set_mode(i, cfg_get_port_mode(i));
            }
    }
    ///-------------------------------------------------

    //Читаем значения для всех портов
    for (i=0; i<CALC_ITERATIONS; i++) {
        ReadDI();  
        for (p=0; p<PORTS_COUNT; p++)
            vals[i][p] = parced_values[p];
    }

    for (i=0; i<PORTS_COUNT; i++) {
      
        int ct = cfg_get_port_mode(i);
        int cp = cfg_get_port_state(i);
        switch (ct) {
            default:
                    //Включим/выключим питание группы 2мА и источника 10мА, если нужно
                    if (cp == CH_STATE_ON) 
                          on2mA[i/4] |= (1 << (i%4));
                    else  on2mA[i/4] &=~(1 << (i%4));
                    
                    SetOn10ma_toCfg(i, NEED_ON_10MA(ct,cp) ? 1 : 0, (on2mA[i/4]& 0x0F) ? 1 : 0);
                    update_kind = UPDATE_KIND_VALS;
                    break;

            case CH_MODE_DI_IC_2MA:
            case CH_MODE_DI_IC_10MA:
            case CH_MODE_DI_IC_10MA_CC:     
            case CH_MODE_DI_IC_10MA_CC_KZ:
                    update_kind = UPDATE_KIND_PULSE;    break;
        }


        if (cp == CH_STATE_OFF)
            on2mA[i/4] &=~(1 << (i%4));         //Источник может быть выключен

        
        if (cp == CH_STATE_ON)
        if (isTimeout(port_p[i].tmo))
        {
            //Автомат импульсного режима
            if (update_kind & UPDATE_KIND_PULSE) {
                update_kind&=~UPDATE_KIND_PULSE;

                //Запуск автомата для импульсного режима
                if (exchange_period < MIN_EXCHANGE_PERIOD)
                {   //Не меняем состояние порта, если порт был включен.
                    //Иначе возможена ситуация когда поменяли период и порт оказался выключеным.
                    if (port_p[i].kind!= 0)
                        port_p[i].kind = 1;
                }

                switch (port_p[i].kind) {
                    case 0: default: //Включение порта
                        port_p[i].kind = 1;
                        port_p[i].tmo = setTimeout(ON_WAIT_TMO);
                        
                        // Синхронизация времени включения по первому порту
                        if (i%4 == 0)
                            for (j=1; j<4; j++) 
                              
                              if ((i+j < PORTS_COUNT) &&                //На всякий случай и
                                  (ct == cfg_get_port_mode(i+j)))       //Если режим совпадает
                              {
                                  port_p[i+j].kind = 0;
                                  port_p[i+j].tmo = setTimeout(0);
                              }
                        
                        
                        //Включаем источник на 2мА
                        on2mA[i/4] |= (1 << (i%4));
                        
                        //Включаем источник на 10мА
                        SetOn10ma_toCfg(i, NEED_ON_10MA(ct,cp) ? 1 : 0, (on2mA[i/4] & 0x0F) ? 1 : 0);                        
                        break;

                    case 1: //Измерение значений
                        port_p[i].kind = 2;
                        port_p[i].tmo = setTimeout(0);
                        update_kind|= UPDATE_KIND_VALS;
                        break;

                    case 2: //Выключение порта
                        port_p[i].kind = 0;
                        port_p[i].tmo = setTimeout(OFF_WAIT_TMO);
                        on2mA[i/4] &=~(1 << (i%4));         //Источник может быть выключен
                        
                        SetOn10ma_toCfg(i, 0, on2mA[i/4] & 0x0F);  //Безусловное выключение источника 10мА. И 2 мА в случае если все порты выключены
                        break;
                }
            }
        }

        //Обновление отдаваемых значений
        if (update_kind & UPDATE_KIND_VALS) {
            update_kind&=~UPDATE_KIND_VALS;
            
            //Выставление результирующих значений
            board_set.values.value[i] = vals[0][i];
        }                
    }


    
    //Применение заданной конфигурации
    #define START_ITER  100
    if (SSP_Target<tDO)
        for (int i=0; i<18; i++)
            if ((new_mode[i]!=last_mode[i]) || 
                (start_config == START_ITER))
            {
                memcpy(last_mode,new_mode,sizeof(new_mode));
                SSP_Target = tDO;
                ssp2_cnt = 0;
                InitDMA_SSP2(last_mode,Led_group);
            }
    
    if (start_config<= START_ITER) {
        start_config++;
    } 
    //Che: Если включить переодическое переозадачивание, то будет появляться
    //шум на входах (закономерность не выялена). Нужно выяснить почему так происходит. 
    //Если добавить задержку перед чтением, шум не исчезает. 
    //else 
    //    start_config = 0;
    
#if ENABLE_PRINT_TMO
    hw_ticks_e = GetTickCount();
    if  (hw_ticks<= hw_ticks_e)
         hw_ticks = hw_ticks_e - hw_ticks;
    else hw_ticks = 0xFFFFFFFF - hw_ticks + hw_ticks_e;

    if (hw_ticks > tmo) tmo = hw_ticks;
    //printf("Time: %d, %d, %d\n", tmo, hw_ticks, GetTickCount());
#endif
}
/**/



void druck_wdt(void) {
/*	static int skip = 0;

	if (--skip <= 0) {
		asm("CPSID i");
		WDOG_REFRESH = (uint16_t) 0xA602;
		WDOG_REFRESH = (uint16_t) 0xB480;
		asm("CPSIE i");
		//_int_enable();	
		skip = 400;
	}*/
}

unsigned char ssp1_mode=0;
unsigned char wr_sp1=0;
unsigned long r_enter=0;
void druck_rck(void) 
{
    unsigned char i=0;
    unsigned short b=0;
/* //Room420  18.10.2019
   Now druck rck as you say  we can do after sending buffer right into the function which send this buffer
    as it is blocking untill sending and we can do it after that
   
    if (((!SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_BSY)))&&(SSP_Target==tLEDS)){
        PORT_ResetBits(MDR_PORTF, LED_RSK_L);
        PORT_SetBits(MDR_PORTF, LED_RSK_L);
        SSP_Target=tNONE;
        DMA_TX_FLAG=0;
    }
*/    
/*//Room420  28.12.2019
  

    if (((!SSP_GetFlagStatus(MDR_SSP2, SSP_FLAG_BSY)))&&(SSP_Target==tDO)){
        PORT_ResetBits(MDR_PORTE, PORT_Pin_2);
        PORT_SetBits(MDR_PORTE, PORT_Pin_2);
        SSP_Target=tNONE;
        DMA_TX_FLAG=0;
    }
*/
    if (((!SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_BSY)))&&(ssp1_mode!=tRECV)){
        init_SPI_Mode(SSP_SPH_2Edge);
        while((SSP_GetFlagStatus(MDR_SSP1,SSP_FLAG_RNE))&&(i<100)){
            b=SSP_ReceiveData(MDR_SSP1);
            i++;
        }
        ssp1_cnt=0;
        ssp1_mode=tRECV;
        PORT_ResetBits(MDR_PORTD, LED_TEST);
    }
    
    if (((SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_RNE)))&&(ssp1_mode==tRECV)){
        ssp1_mode=tRECV;
        while((SSP_GetFlagStatus(MDR_SSP1,SSP_FLAG_RNE))&&(i<18)){
            b=SSP_ReceiveData(MDR_SSP1);
            rx_buf[ssp1_cnt]=(b^0xFFFF);
            DstBuf1[ssp1_cnt++]=(b^0xFFFF);
            if (ssp1_cnt>90){
                ssp1_cnt=0;
            }
            i++;
        }
    }
    
    //if(((!SSP_GetFlagStatus(MDR_SSP1, SSP_FLAG_BSY)))&&(ssp1_mode==tRECV)&&(ssp1_cnt)){
    if ((PORT_ReadInputDataBit(MDR_PORTF, PORT_Pin_2) == Bit_SET)&&(ssp1_cnt)&&(ssp1_mode==tRECV)) { 
        PORT_SetBits(MDR_PORTD, LED_TEST);
        r_enter++;
        while ((SSP_GetFlagStatus(MDR_SSP1,SSP_FLAG_RNE))&&(i<18))
        {
            b=SSP_ReceiveData(MDR_SSP1);
            //rx_buf[ssp1_cnt/2]|=((b^0xFFFF)<<16*((ssp1_cnt%2)));
            rx_buf[ssp1_cnt]=(b^0xFFFF);
            DstBuf1[ssp1_cnt++]=(b^0xFFFF);
            if (ssp1_cnt>90){
                ssp1_cnt=0;
            }
            i++;
        }
        
        fp_parse_data();
        if (fp_driver.tx_state.tx_ready == 1){
            fp_send();
            ssp1_mode=tTRANS;
        }
        ssp1_cnt=0;        
    }
}


void process_flags(void)
{
      char sn[SERIAL_NUM_COUNT];
      unsigned short eval;
    
///=============================================================
      if (board_set.flags.flag_save & HR_FLAG_APPLY_ONLY) 	  	{
          cfg_update_config();
      } else {
//Room420  03.10.2019          if (board_set.flags.flag_save & HR_FLAG_SAVE_CONFIG) 	        cfg_write_config();   	
//Room420  03.10.2019           if (board_set.flags.flag_save & HR_FLAG_ERACE_CONFIG)         cfg_erase_config();         
        if (board_set.settings.diag.test.LED == 1) { 
        leds_test();
        board_set.settings.diag.test.LED = 0;
        }
          if (board_set.settings.diag.test.Fram == 1)          {            
//Room420  07.11.2019              board_set.values.eeprom_result = fram_test(&eval);
              board_set.settings.diag.memtest_info.result=fram_test(&eval);

              board_set.values.eeprom_test_cnt++;
              board_set.settings.diag.test.Fram =0;
          }
          
          //
          
     }
}


volatile int reset_on = 0;

void main_refresh_task(void) {
/*
	if (g_debug_flags & DBG_LOAD_THREADS)
		printf("main_refresh_task starting...\n");
*/
    int size;
    unsigned char i;
    
   
  
   ReStore_EE();
     /* Enables the HSI clock for WWDG */
 //Room420  04.10.2019  temporarely TEMPOR1
#define TEMPOR1
#ifdef   TEMPOR1
    WWDG_ClearFlag();
    RST_CLK_PCLKcmd(RST_CLK_PCLK_WWDG,ENABLE);
    /* Set WWDG Prescaler value*/
    WWDG_ClearFlag();
    WWDG_SetPrescaler	(WWDG_Prescaler_8);
    /* Enable WWDG and load start counter value*/
    WWDG_Enable(0x7F);
#endif    
    size = sizeof(cfg_params);
    
    for (;;) {
       module_wanted_parse();
	
        if (reset_on == 0)
          WWDG_SetCounter(0x7F);
        if (i++%2) 
              process_leds();

        GetI2cTemperatue();
        
        //di_process_job();
        //show_stat();
        druck_wdt();
        druck_rck();
       
   
        if(board_set.settings.save_cfg)
        {
          board_set.settings.save_cfg=0;
          Store_EE();        
        }       
        // develop of RoomTest420 ========
     check_command(); 
     if(delay_get_dac_status < GetTickCount() ){
     get_dac_status();
     delay_get_dac_status= GetTickCount() + 1000;              
     }

    }

}

void module_init(void) {
	
}

void module_deinit(void) {

}

int port_get_value(uint8_t num, uint16_t *value) 
{
    if (num > PORTS_COUNT) {
        *value = 0;
        return -1;
    }
    *value = board_set.values.value[num];

    return 0;
}

void port_set_state(uint8_t num, enum ch_state on) 
{
    board_set.settings.ao_status[num] = on;
    mode_change=1;
}

enum ch_state port_get_state(uint8_t num) 
{
    if (num > PORTS_COUNT)
          return (enum ch_state)CH_STATE_OFF;
    else 	
          return (enum ch_state)cfg_get_port_state(num);
}

#define MAX_10_MA_PORTS 24
void port_set_mode(uint8_t num, enum ch_mode mode) 
{
    int cnt, i;
    
    cnt = 0;
    if (NEED_ON_10MA(mode,1)) {
        for (i=0; i<PORTS_COUNT; i++)
            if (NEED_ON_10MA(board_set.settings.ao_mode[i],1)) {
                if (++cnt > MAX_10_MA_PORTS)      //Выключаем режим 10 mA на всякий случай
                    board_set.settings.ao_mode[num] = CH_MODE_DI_DC_2MA;
            }         
    }
                
    if ((cnt < MAX_10_MA_PORTS) || (!NEED_ON_10MA(mode,1)))
          board_set.settings.ao_mode[num] = mode;
    else  board_set.settings.ao_mode[num] = CH_MODE_DI_DC_2MA;
    mode_change = 1;
}


enum ch_mode port_get_mode(uint8_t num) 
{
	if (num > PORTS_COUNT)
			return (enum ch_mode)CH_MODE_DI_IEC;
	else 	return (enum ch_mode)cfg_get_port_mode(num);
}




void port_get_status(uint8_t num, enum mod_status *status) {
  
}


//отложенная обработка озадачиваний сверху
void module_wanted_parse(void) {
	int i;
	//static int once = 1;
	//static int process = 0;
	//static int pro = 0;


	i = mod_wanted.want_anything;
	mod_wanted.want_anything = 0;
	
	/*if (++process > 500) {
		process = 0; 
		if (g_debug_flags & 0x10)
			printf("module_wanted_parse: %i\n",pro++);
	}*/
	
	if (i == 0)
		return;

	if (mod_wanted.ch_onoff_mask)
		for (i = 0; i < PORTS_COUNT; i++)
			if (mod_wanted.ch_onoff_mask[i]) {
                            mod_wanted.ch_onoff_mask[i] = 0;
                            port_set_state(i, mod_wanted.ch_onoff[i]);
			}
	
	if (mod_wanted.ch_mode_mask)
		for (i = 0; i < PORTS_COUNT; i++)
			if (mod_wanted.ch_mode_mask[i]) {
				mod_wanted.ch_mode_mask[i] = 0;
				port_set_mode(i, mod_wanted.ch_mode[i]);
			}
	
        if (mod_wanted.config_change) {
            if (mod_wanted.config_change & CONFIG_ERACE) { cfg_erase_config();   }
            if (mod_wanted.config_change & CONFIG_PARCE) { cfg_parce_config();   }
            if (mod_wanted.config_change & CONFIG_SAVE)  { cfg_write_config();   }
            mod_wanted.config_change = 0;
	}
	
	if (mod_wanted.block_state_enter) {
		mod_wanted.block_state_enter = 0;
		module_apply_block_state();
	}
}


//Постановка в очередь пожелания включить/выключить АЦП
void port_queue_set_state(uint8_t adc_index, uint8_t adc_state) {
	if (adc_index < PORTS_COUNT) {
		if (adc_state != port_get_state(adc_index)) {
			mod_wanted.ch_onoff[adc_index] = adc_state;
			mod_wanted.ch_onoff_mask[adc_index] = 1;
			mod_wanted.want_anything = 1;
		}
	}
}

//Постановка в очередь пожелания сменить режим АЦП
void port_queue_set_mode(uint8_t adc_index, uint8_t adc_mode) {
	if (adc_index < PORTS_COUNT) {
		//if (adc_mode != port_get_mode(adc_index)) {
			mod_wanted.ch_mode[adc_index] = adc_mode;
			mod_wanted.ch_mode_mask[adc_index] = 1;
			mod_wanted.want_anything = 1;
		//}
	}
}



uint8_t conf_process = 0;
ExpBoardParametersDI48_t conf_parced;


void parse_configuration(ExpBoardParametersDI48_t *conf)
{
    char idx = 0;
  
    conf->common.leds_cfg_state ?
				leds_all_on() :
				leds_all_off();

    leds_set_timeout(conf->common.leds_timeout);
    led_on(LED_USER, conf->common.user_led == 0 ? LED_OFF :
                     conf->common.user_led == 1 ? LED_RED : LED_GREEN, 0);
    
    board_set.settings.user_led_type = conf->common.user_led;
   
    for (idx=0; idx<PORTS_COUNT; idx++)
    {
	port_set_state	(idx, conf->port[idx].pm_state ?	CH_STATE_ON	: CH_STATE_OFF);
	port_set_mode	(idx, conf->port[idx].mode);

	//Эти функции тоже отложенные, так как не приминяют изменения сразу.
        //alarm_set_port_value	(idx, conf->port[idx].alarm_value);
 	port_set_block_state	(idx, conf->port[idx].bs_pm_state ? CH_STATE_ON	: CH_STATE_OFF, conf->port[idx].mode);
    }

    queue_config_change(CONFIG_SAVE);
}


void cfg_parce_config()
{
    uint8_t idx = 0;
	int some_change = 0;

    if (conf_parced.common.leds_cfg_state != board_set.settings.diag.leds.is_leds_on ||
        conf_parced.common.leds_timeout   != board_set.settings.leds_timeout ||
        conf_parced.common.user_led       != board_set.settings.user_led_type)
        some_change = 1;

    if (!some_change)
        for (idx=0; idx<PORTS_COUNT; idx++)
        {
            //Преобразуем тип канала в формат модуля
            conf_parced.port[idx].mode = SwitchToBoardType(conf_parced.port[idx].mode);

            if ((conf_parced.port[idx].pm_state    != board_set.settings.ao_status[idx]) ||
                (conf_parced.port[idx].mode        != board_set.settings.ao_mode [idx]) )
            {
                some_change = 1;
            }
        }
    
    if (some_change)
        parse_configuration(&conf_parced);

    conf_process = 0;
}



void queue_parce_configuration(uint8_t *conf, uint16_t cnt)
{
    if (conf_process) {
        //printf("Error! Previous configuration is applied.");
        return;
    }

    if (cnt == sizeof(conf_parced))
    {
        conf_process = 1;
        
        //Эти параметры обработаем сразу, ещё в прерывании
        data_type = ((ExpBoardParametersDI48_t*)conf)->common.data_type;  
        if (((ExpBoardParametersDI48_t*)conf)->common.poll_period >= MIN_EXCHANGE_PERIOD)
          exchange_period = ((ExpBoardParametersDI48_t*)conf)->common.poll_period;

        
        memcpy(&conf_parced,conf,sizeof(conf_parced));
        queue_config_change(CONFIG_PARCE);
    }
    else
    {
        //TODO: Сообщить об ошибке, возможно выставить какой-нибудь флаг
        //printf("Error! Incorrect size of a configuration.");
    }
}

//Постановка в очередь пожелания сбросить или сохранить конфигурацию
void queue_config_change(uint8_t config_change) 
{
	mod_wanted.config_change = config_change;
	mod_wanted.want_anything = 1;
}
