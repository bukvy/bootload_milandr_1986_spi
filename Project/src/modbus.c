#include "modbus.h"
#include "serial.h"
#include "string.h"
#include "config.h"

unsigned long last_mb_time; //время начала отправки последнего пакета по модбасу

unsigned short ModBusCRC16(unsigned char *p, unsigned short n)
{
    unsigned short w, i;
    unsigned char j;
    for (w = 0xFFFF, i = 0; i < n; i++)
    {
        w ^= p[i];
        for (j = 0; j < 8; j++) if (w & 1)
            {
                w >>= 1;
                w ^= 0xA001;
            }
            else w >>= 1;
    }
    return w;
}

unsigned char ModBusLRC(unsigned char *p, unsigned short n)
{
    unsigned char b = 0;
    while (n--) b += *p++;
    return ((unsigned char)(-((char)b)));
}

//slave (серверная) часть от modbus

#include "modbus.h"

//unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
unsigned char g_bModbusnInListenOnly;	// нахождение в режиме listen-only

//входной/выходной буфер для ModBus
unsigned char g_ModbusBuf[MB_BUF_SIZE];
unsigned short g_wModbusCnt;

//статистика протокола Modbus
typedef struct
{
    unsigned short wTotal;		//количество сообщений, пойманных НА ШИНЕ (ie не только к этому устройству), БЕЗ учета битых
    unsigned short wErr;		//количество полученных битых сообщений: с ошибочной или отсутствующей CRC; отбракованных по ошибке четности; длиннее макс.возможного
    unsigned short wExceptCnt;	//количество сгенерированных исключений с ЭТОГО девайса (необязательно отправленных - в случае broadcast)
    unsigned short wThisCnt;	//количество сообщений, обработанных устройством (включая broadcast)
    unsigned short wNoResponce;	//количество сообщений, обработанных устройством, на которые НЕ отправляется ответ (т.е. broadcast)
    unsigned short wNAKCnt;		//количество отправленных с этого девайса сообщений с исключениями Negative Acknowledge
    unsigned short wBusyCnt;	//количество отправленных с ЭТОГО девайса сообщений с исключениями Slave Device Busy
    unsigned short wBadStream;	//количество принятых битых по причине загруженности устройства пакетов (байты принимаются быстрее, чем мы можем их обработать)
} SMBSerialStat;

//SMBSerialStat MBSerialStat;

void ClearStat(void)
{
    /*MBSerialStat.wBadStream = MBSerialStat.wBusyCnt = MBSerialStat.wErr =
    	MBSerialStat.wExceptCnt = MBSerialStat.wNAKCnt = MBSerialStat.wNoResponce =
    	MBSerialStat.wThisCnt = MBSerialStat.wTotal = 0;*/
    g_bModbusnInListenOnly = 0;
}

// потери байт во время приема
void fmb_OnCharacterOverrrun(void)
{
    //MBSerialStat.wBadStream++;
}

// ошибка в пакете - вызывать в случае ошибок UART'а или плохого fmb_CheckRTU
void fmb_OnReceiveError(void)
{
    //MBSerialStat.wErr++;
}

//проверка RTU-пакета на корректность
unsigned char fmb_CheckRTU(void)
{
    unsigned short crc;
    if (g_wModbusCnt < 3) return 0;	//недостаточная длина для нормального пакета
      crc = ModBusCRC16(g_ModbusBuf, (unsigned short)(g_wModbusCnt - 2));
    if ((LOBYTE(crc) != g_ModbusBuf[g_wModbusCnt-2])
            || (HIBYTE(crc) != g_ModbusBuf[g_wModbusCnt-1]))
        return 0; //плохая CRC
    g_wModbusCnt -= 2; //CRC для обработки не нужна
    return 1;
}

//будет: DP, DPCH, P, T, Tкорп, зав.N, флаги сост-я

typedef unsigned char (* PMBFUNC)(void);	//указатель на выполняемую функцию, результат - код исключения

//Таблица поддерживаемых устройством функций MODBUS
typedef struct
{
    unsigned char	bFuncId;		//номер функции
    unsigned char	bUseInBroadcast;//возможность вызова в броадкасте
    PMBFUNC			pProcessBody;	//указатель на тело обработчика
} SMBFunc;

//unsigned short gmbf_InputRegisters[MBF_INPREG_CNT];	//"входные" регистры
unsigned short *gmbf_InputRegisters;// в main установить указатель

//unsigned short gmbf_HoldingRegisters[MBF_HOLDREG_CNT];	//хранимые регистры
unsigned short *gmbf_HoldingRegisters;// в main установить указатель

//unsigned char gmbf_DIs[(MBF_DI_CNT/8 + 1)];	//дискретные входа
unsigned char *gmbf_DIs;// в main установить указатель

//unsigned char gmbf_Coils[(MBF_COIL_CNT/8 + 1)];	//флаги
unsigned char *gmbf_Coils;// в main установить указатель

unsigned short wRegStart, wRegCnt;

//чтение 16-битных регистров
unsigned char fmbk_ReadInputOrHoldRegister(unsigned short *pData, unsigned short wMaxVal)
{
    unsigned short i;
    //валидация
    if (wRegCnt < 1 || wRegCnt > 0x7D) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= wMaxVal) || (wRegStart + wRegCnt > wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //подготовка корректного ответа - складываем все в буфер
    for (i = 0; i < wRegCnt; i++)
    {
        g_ModbusBuf[3+i*2] = HIBYTE(pData[i+wRegStart]);
        g_ModbusBuf[4+i*2] = LOBYTE(pData[i+wRegStart]);
    }
    g_ModbusBuf[2] = wRegCnt * 2;	//кол-во байт данных
    g_wModbusCnt = wRegCnt * 2 + 3;	//размер посылки
    return 0;
}

//чтение входных регистров
unsigned char fmbk_ReadInputRegister(void)
{
    return fmbk_ReadInputOrHoldRegister(gmbf_InputRegisters, MBF_INPREG_CNT);
}

//чтение хранимых регистров
unsigned char fmbk_ReadHoldingRegister(void)
{
    return fmbk_ReadInputOrHoldRegister(gmbf_HoldingRegisters, MBF_HOLDREG_CNT);
}

//чтение битов
unsigned char fmbk_ReadCoilsOrDI(unsigned char *pData, unsigned short wMaxVal)
{
    unsigned short i, wPosGet;
    unsigned char bMaskGet, bMaskPut, bValue, bPosPut;
    //валидация
    if (wRegCnt < 1 || wRegCnt > 0x7D0) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= wMaxVal) || (wRegStart + wRegCnt > wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;

    //подготовка корректного ответа
    wPosGet = wRegStart >> 3;		//позиция в массиве флагов
    bPosPut = 0;
    bMaskGet = 1 << (wRegStart & 0x07);	//маска текущей позиции в масиве флагов
    bMaskPut = 1;
    bValue = 0;
    for (i = 0; i < wRegCnt; i++)
    {
        if (pData[wPosGet] & bMaskGet)
            bValue |= bMaskPut;
        else bValue &= ~bMaskPut;

        bMaskGet <<= 1;
        if (bMaskGet == 0)
        {
            bMaskGet = 1;
            wPosGet++;
        }
        bMaskPut <<= 1;
        if ((bMaskPut == 0) || (i == wRegCnt - 1)) //пишем очередной байт
        {
            g_ModbusBuf[3 + (bPosPut++)] = bValue;
            bMaskPut = 1;
            bValue = 0;
        }
    }
    g_ModbusBuf[2] = bPosPut;	//кол-во байт данных
    g_wModbusCnt = bPosPut + 3;	//размер посылки
    return 0;
}

//чтение флагов
unsigned char fmbk_ReadCoils(void)
{
  #ifdef MCO_EN
    return fmbk_ReadCoilsOrDI(gmbf_Coils, MBF_COIL_CNT);
   #endif
    return 0;
}

//чтение цифровых входов
unsigned char fmbk_ReadDIs(void)
{
  #ifdef MDI_EN
    return fmbk_ReadCoilsOrDI(gmbf_DIs, MBF_DI_CNT);
  #endif
    return 0;
}

//запись одного флага
unsigned char fmbk_WriteSingleCoil(void)
{
  #ifdef MCO_EN
    //валидация
    if ((wRegCnt != 0) && (wRegCnt != 0xFF00)) return mbexc_ILLEGAL_DATA_VALUE;
    if (wRegStart >= MBF_COIL_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;

    //осуществляем запись во флаг, готовим ответ
    if (wRegCnt) gmbf_Coils[wRegStart>>3] |= (1 << (wRegStart & 7));
    else gmbf_Coils[wRegStart>>3] &= (~(1 << (wRegStart & 7)));
    g_wModbusCnt = 6;	//размер посылки
  #endif
    return 0;
}

//запись одного флага
unsigned char fmbk_WriteSingleRegister(void)
{
    //валидация
    if (wRegStart >= MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
    //осуществляем запись в регистр, готовим ответ
    gmbf_HoldingRegisters[wRegStart] = wRegCnt;
    g_wModbusCnt = 6;	//размер посылки
    return 0;
}

//запись флагов
unsigned char fmbk_WriteCoils(void)
{
  #ifdef MCO_EN
    unsigned short i, wPosPut;
    unsigned char bMaskGet, bMaskPut, bPosGet;

    unsigned char Nn;
    //валидация

    Nn = g_ModbusBuf[6];	//количество байт в области данных
    if ((wRegCnt < 1) || (wRegCnt > 0x7B0) || (Nn != g_wModbusCnt - 7)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_COIL_CNT) || (wRegStart + wRegCnt > MBF_COIL_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;

    //записываем флаги
    wPosPut = wRegStart >> 3;		//позиция в массиве флагов
    bPosGet = 0;
    bMaskPut = 1 << (wRegStart & 0x07);	//маска текущей позиции в масиве флагов
    bMaskGet = 1;
    for (i = 0; i < wRegCnt; i++)
    {
        if (g_ModbusBuf[7 + bPosGet] & bMaskGet)
            gmbf_Coils[wPosPut] |= bMaskPut;
        else gmbf_Coils[wPosPut] &= ~bMaskPut;
        bMaskGet <<= 1;
        if (bMaskGet == 0)
        {
            bMaskGet = 1;
            bPosGet++;
        }
        bMaskPut <<= 1;
        if (bMaskPut == 0)
        {
            bMaskPut = 1;
            wPosPut++;
        }
    }
    g_wModbusCnt = 6;	//размер посылки
  #endif
    return 0;
}

//запись регистров
unsigned char fmbk_WriteHoldingRegisters(void)
{
    unsigned short i;
    unsigned char Nn;
    //валидация
    Nn = g_ModbusBuf[6];	//количество байт в области данных
    if ((wRegCnt < 1) || (wRegCnt > 0x7B) || (Nn != g_wModbusCnt - 7)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_HOLDREG_CNT) || (wRegStart + wRegCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //записываем регистры
    for (i = 0; i < wRegCnt; i++) gmbf_HoldingRegisters[wRegStart+i] =
            (g_ModbusBuf[7+i*2] << 8) | g_ModbusBuf[8+i*2];
    g_wModbusCnt = 6;	//размер посылки
    return 0;
}

//запись регистра по маске
unsigned char fmbk_MaskHoldingRegister(void)
{
    unsigned char wOr;
    //валидация
    if (wRegStart >= MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
    //записываем регистр
    wOr = (g_ModbusBuf[6] << 8) | g_ModbusBuf[7];	//маска "OR", маска "AND" - в RegCnt
    gmbf_HoldingRegisters[wRegStart] = (gmbf_HoldingRegisters[wRegStart] & wRegCnt) | (wOr & (~wRegCnt));
    g_wModbusCnt = 8;	//размер посылки
    return 0;
}

//запись регистров
unsigned char fmbk_ReadWriteHoldingRegisters(void)
{
    unsigned short i, wWriteStart, wWriteCnt;
    unsigned char Nn;
    //валидация
    wWriteStart = (g_ModbusBuf[6] << 8) | g_ModbusBuf[7];
    wWriteCnt = (g_ModbusBuf[8] << 8) | g_ModbusBuf[9];
    Nn = g_ModbusBuf[10];	//количество байт в области данных

    if ((wRegCnt < 1) || (wRegCnt > 0x7D)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wWriteCnt < 1) || (wWriteCnt > 0x79) || (Nn != g_wModbusCnt - 11)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_HOLDREG_CNT) || (wRegStart + wRegCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    if ((wWriteStart >= MBF_HOLDREG_CNT) || (wWriteStart + wWriteCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //записываем регистры
    for (i = 0; i < wWriteCnt; i++)
        gmbf_HoldingRegisters[wWriteStart+i] = (g_ModbusBuf[11+i*2] >> 8) | g_ModbusBuf[12+i*2];
    //читаем регистры
    for (i = 0; i < wRegCnt; i++)
    {
        g_ModbusBuf[3+i*2] = HIBYTE(gmbf_HoldingRegisters[i+wRegStart]);
        g_ModbusBuf[4+i*2] = LOBYTE(gmbf_HoldingRegisters[i+wRegStart]);
    }
    g_ModbusBuf[2] = wRegCnt * 2;	//кол-во байт данных
    g_wModbusCnt = wRegCnt * 2 + 3;	//размер посылки
    return 0;
}

//диагностика уарта
unsigned char fmbk_ReadDiagnostic(void)
{

    return 0;
}

#define MBF_CNT 11	//количество имплементированных MODBUS функций

//список имплементированных MODBUS функций
const SMBFunc MBList[MBF_CNT] =
{
    {mbf_Read_Input_Register, 0, &fmbk_ReadInputRegister},
    {mbf_Read_Coils, 0, &fmbk_ReadCoils},
    {mbf_Read_Discrete_Inputs, 0, &fmbk_ReadDIs},
    {mbf_Read_Holding_Registers, 0, &fmbk_ReadHoldingRegister},
    {mbf_Write_Single_Coil, 0, &fmbk_WriteSingleCoil},
    {mbf_Write_Single_Register, 0, &fmbk_WriteSingleRegister},
    {mbf_Write_Multiple_Coils, 0, &fmbk_WriteCoils},
    {mbf_Write_Multiple_Registers, 0, &fmbk_WriteHoldingRegisters},
    {mbf_Mask_Write_Register, 0, &fmbk_MaskHoldingRegister},
    {mbf_Read_Write_Multiple_Registers, 0, &fmbk_ReadWriteHoldingRegisters},
    {mbf_Diagnostic, 1, &fmbk_ReadDiagnostic }

};

//обработка PDU, результат - код исключения (либо 0, если все OK)
// внутри: проверка на валидность PDU
// внутри: формировать пакет ответа
unsigned char fmb_ValidateAndProcessPDU(void)
{
    unsigned char i, nf;
    nf = 0xFF;
    for (i = 0; i < MBF_CNT; i++) //ищем функцию в списке
    {
        if (MBList[i].bFuncId == g_ModbusBuf[1])
        {
            nf = i;
            break;
        }
    }
    if (nf == 0xFF) return mbexc_ILLEGAL_FUNCTION;	//функция вообще не

    wRegStart = (g_ModbusBuf[2] << 8) | g_ModbusBuf[3];	//аргументы для большинства функций
    wRegCnt = (g_ModbusBuf[4] << 8) | g_ModbusBuf[5];

    if ((MBList[nf].bUseInBroadcast == 0) && (g_ModbusBuf[0] == 0)) return mbexc_ILLEGAL_FUNCTION;	//функция не поддерживается в broadcast

    //если длина данных некорректна, R=mbexc_ILLEGAL_DATA_VALUE
    //если адреса регистров некорректны, R=mbexc_ILLEGAL_DATA_ADDRESS
    //если данные некорректны, R=mbexc_ILLEGAL_DATA_VALUE
    return (MBList[nf].pProcessBody) ? (*MBList[nf].pProcessBody)() : mbexc_ILLEGAL_FUNCTION;	//либо выполним, либо мы еще не дописали функцию
}

//обработка небитого RTU
//после вызова - надо будет отослать пакет, если он непустой
void fmb_OnNormalRTU(void)
{
    unsigned char r;
    //MBSerialStat.wTotal++;	//количество нормальных принятых пакетов увеличилось
    if ((g_ModbusBuf[0] == MODBUS_ADDRESS) || (g_ModbusBuf[0] == 0))   //сообщение этому устройству или broadcast
    {
        //MBSerialStat.wThisCnt++; //количество принятых пакетов для данного устройства увеличилось
        r = fmb_ValidateAndProcessPDU(); //проверка на валидность и обработка
        if (r != 0)  	// таки исключение
        {
            //MBSerialStat.wExceptCnt++;	//одним исключением стало больше
            //if (r==mbexc_ACKNOWLEDGE) MBSerialStat.wNAKCnt++;			//по этому исключению отдельная статистика
            //if (r==mbexc_SLAVE_DEVICE_BUSY) MBSerialStat.wBusyCnt++;	//по этому исключению отдельная статистика
            g_ModbusBuf[1] |= 0x80;	//согласно протоколу - код функции + 0x80
            g_ModbusBuf[2] = r;		//код исключения
            g_wModbusCnt = 3;		//количество байт в ответной посылке
        }
        if (g_ModbusBuf[0] == 0) //если это broadcast
        {
            g_wModbusCnt = 0; //ГАСИМ ответ, т.к.посылать его не надо
        }
    }
    else g_wModbusCnt = 0;
    if (g_wModbusCnt)   //пакет для отправки есть, добавим контрольную сумму
    {
        unsigned short crc = ModBusCRC16(g_ModbusBuf, g_wModbusCnt);
        g_ModbusBuf[g_wModbusCnt++] = LOBYTE(crc);
        g_ModbusBuf[g_wModbusCnt++] = HIBYTE(crc);
    }
    //if (g_bModbusnInListenOnly) g_wModbusCnt=0;
}

void fmb_OnCompleteMBPacket(void)	//по завершению приема модбас-пакета,
{
    if (fmb_CheckRTU())
        fmb_OnNormalRTU();
    else g_wModbusCnt = 0;
}

void ProcessModbus(void)
{
    if (U2.mode == UART_TURN_MODE)
        if (U2.Check_end_pack())
            U2.Stop_TX();

    if (U2.mode == UART_RECEIVE_MODE)
    {
        if (U2.cnt) if (TimerCounter - U2.rx_last_time >= MODBUS_PACKET_DETECT_TIME)
        {
            U2.mode = UART_BUSY_MODE;
            g_wModbusCnt = U2.cnt;
            fmb_OnCompleteMBPacket();
            if (g_wModbusCnt)
            {
                last_mb_time = TimerCounter;
                SendUart2_DMA(g_ModbusBuf,g_wModbusCnt);

            }
            U2.cnt = 0;
            U2.mode = UART_RECEIVE_MODE;
        }
    }
    return;
}
