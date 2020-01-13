#ifndef CRC_HW_H
#define CRC_HW_H

/*
 * Модуль вычисления CRC с аппаратным ускорением.
 * 
 * Модуль скопирован практически без изменений из какого-то тестового проекта в MQX.
 * 
 * Способ использования:
 * 0. Где-то в начале работы сделать CRC_Init(), можно не беспокоится о повторном
 *    вызове этой функции.
 * 1. Перед расчетом CRC суммы делать CRC_SetCRCStandard() с нужным параметром.
 * 
 * Замечание1: необходимо учитывать порядок байт в словах в массиве данных, если CRC
 *             не сходится, первым делом лучше удостоверится, что порядок байт одинаков 
 *            
 * Замечание2: работает только режим LDD_CRC_MODBUS_16 ;) 
 */

/* CRC standard  */
typedef enum  {
  LDD_CRC_16,
  LDD_CRC_CCITT,
  LDD_CRC_MODBUS_16,
  LDD_CRC_KERMIT,
  LDD_CRC_DNP,
  LDD_CRC_32,
  LDD_CRC_USER
} CRC_TCRCStandard;

extern void CRC_Init(void);

extern int CRC_SetCRCStandard(CRC_TCRCStandard CRCStandard);

extern uint32_t CRC_Cal_16(uint8_t *msg, uint32_t size);
extern uint32_t CRC_Cal_32(uint8_t *msg, uint32_t size);

#endif
