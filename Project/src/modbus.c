#include "modbus.h"
#include "serial.h"
#include "string.h"
#include "config.h"

unsigned long last_mb_time; //����� ������ �������� ���������� ������ �� �������

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

//slave (���������) ����� �� modbus

#include "modbus.h"

//unsigned char g_bThisModbusAddr;	// ����� ������� ���������� �� ���� Modbus
unsigned char g_bModbusnInListenOnly;	// ���������� � ������ listen-only

//�������/�������� ����� ��� ModBus
unsigned char g_ModbusBuf[MB_BUF_SIZE];
unsigned short g_wModbusCnt;

//���������� ��������� Modbus
typedef struct
{
    unsigned short wTotal;		//���������� ���������, ��������� �� ���� (ie �� ������ � ����� ����������), ��� ����� �����
    unsigned short wErr;		//���������� ���������� ����� ���������: � ��������� ��� ������������� CRC; ������������� �� ������ ��������; ������� ����.����������
    unsigned short wExceptCnt;	//���������� ��������������� ���������� � ����� ������� (������������� ������������ - � ������ broadcast)
    unsigned short wThisCnt;	//���������� ���������, ������������ ����������� (������� broadcast)
    unsigned short wNoResponce;	//���������� ���������, ������������ �����������, �� ������� �� ������������ ����� (�.�. broadcast)
    unsigned short wNAKCnt;		//���������� ������������ � ����� ������� ��������� � ������������ Negative Acknowledge
    unsigned short wBusyCnt;	//���������� ������������ � ����� ������� ��������� � ������������ Slave Device Busy
    unsigned short wBadStream;	//���������� �������� ����� �� ������� ������������� ���������� ������� (����� ����������� �������, ��� �� ����� �� ����������)
} SMBSerialStat;

//SMBSerialStat MBSerialStat;

void ClearStat(void)
{
    /*MBSerialStat.wBadStream = MBSerialStat.wBusyCnt = MBSerialStat.wErr =
    	MBSerialStat.wExceptCnt = MBSerialStat.wNAKCnt = MBSerialStat.wNoResponce =
    	MBSerialStat.wThisCnt = MBSerialStat.wTotal = 0;*/
    g_bModbusnInListenOnly = 0;
}

// ������ ���� �� ����� ������
void fmb_OnCharacterOverrrun(void)
{
    //MBSerialStat.wBadStream++;
}

// ������ � ������ - �������� � ������ ������ UART'� ��� ������� fmb_CheckRTU
void fmb_OnReceiveError(void)
{
    //MBSerialStat.wErr++;
}

//�������� RTU-������ �� ������������
unsigned char fmb_CheckRTU(void)
{
    unsigned short crc;
    if (g_wModbusCnt < 3) return 0;	//������������� ����� ��� ����������� ������
      crc = ModBusCRC16(g_ModbusBuf, (unsigned short)(g_wModbusCnt - 2));
    if ((LOBYTE(crc) != g_ModbusBuf[g_wModbusCnt-2])
            || (HIBYTE(crc) != g_ModbusBuf[g_wModbusCnt-1]))
        return 0; //������ CRC
    g_wModbusCnt -= 2; //CRC ��� ��������� �� �����
    return 1;
}

//�����: DP, DPCH, P, T, T����, ���.N, ����� ����-�

typedef unsigned char (* PMBFUNC)(void);	//��������� �� ����������� �������, ��������� - ��� ����������

//������� �������������� ����������� ������� MODBUS
typedef struct
{
    unsigned char	bFuncId;		//����� �������
    unsigned char	bUseInBroadcast;//����������� ������ � ����������
    PMBFUNC			pProcessBody;	//��������� �� ���� �����������
} SMBFunc;

//unsigned short gmbf_InputRegisters[MBF_INPREG_CNT];	//"�������" ��������
unsigned short *gmbf_InputRegisters;// � main ���������� ���������

//unsigned short gmbf_HoldingRegisters[MBF_HOLDREG_CNT];	//�������� ��������
unsigned short *gmbf_HoldingRegisters;// � main ���������� ���������

//unsigned char gmbf_DIs[(MBF_DI_CNT/8 + 1)];	//���������� �����
unsigned char *gmbf_DIs;// � main ���������� ���������

//unsigned char gmbf_Coils[(MBF_COIL_CNT/8 + 1)];	//�����
unsigned char *gmbf_Coils;// � main ���������� ���������

unsigned short wRegStart, wRegCnt;

//������ 16-������ ���������
unsigned char fmbk_ReadInputOrHoldRegister(unsigned short *pData, unsigned short wMaxVal)
{
    unsigned short i;
    //���������
    if (wRegCnt < 1 || wRegCnt > 0x7D) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= wMaxVal) || (wRegStart + wRegCnt > wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //���������� ����������� ������ - ���������� ��� � �����
    for (i = 0; i < wRegCnt; i++)
    {
        g_ModbusBuf[3+i*2] = HIBYTE(pData[i+wRegStart]);
        g_ModbusBuf[4+i*2] = LOBYTE(pData[i+wRegStart]);
    }
    g_ModbusBuf[2] = wRegCnt * 2;	//���-�� ���� ������
    g_wModbusCnt = wRegCnt * 2 + 3;	//������ �������
    return 0;
}

//������ ������� ���������
unsigned char fmbk_ReadInputRegister(void)
{
    return fmbk_ReadInputOrHoldRegister(gmbf_InputRegisters, MBF_INPREG_CNT);
}

//������ �������� ���������
unsigned char fmbk_ReadHoldingRegister(void)
{
    return fmbk_ReadInputOrHoldRegister(gmbf_HoldingRegisters, MBF_HOLDREG_CNT);
}

//������ �����
unsigned char fmbk_ReadCoilsOrDI(unsigned char *pData, unsigned short wMaxVal)
{
    unsigned short i, wPosGet;
    unsigned char bMaskGet, bMaskPut, bValue, bPosPut;
    //���������
    if (wRegCnt < 1 || wRegCnt > 0x7D0) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= wMaxVal) || (wRegStart + wRegCnt > wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;

    //���������� ����������� ������
    wPosGet = wRegStart >> 3;		//������� � ������� ������
    bPosPut = 0;
    bMaskGet = 1 << (wRegStart & 0x07);	//����� ������� ������� � ������ ������
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
        if ((bMaskPut == 0) || (i == wRegCnt - 1)) //����� ��������� ����
        {
            g_ModbusBuf[3 + (bPosPut++)] = bValue;
            bMaskPut = 1;
            bValue = 0;
        }
    }
    g_ModbusBuf[2] = bPosPut;	//���-�� ���� ������
    g_wModbusCnt = bPosPut + 3;	//������ �������
    return 0;
}

//������ ������
unsigned char fmbk_ReadCoils(void)
{
  #ifdef MCO_EN
    return fmbk_ReadCoilsOrDI(gmbf_Coils, MBF_COIL_CNT);
   #endif
    return 0;
}

//������ �������� ������
unsigned char fmbk_ReadDIs(void)
{
  #ifdef MDI_EN
    return fmbk_ReadCoilsOrDI(gmbf_DIs, MBF_DI_CNT);
  #endif
    return 0;
}

//������ ������ �����
unsigned char fmbk_WriteSingleCoil(void)
{
  #ifdef MCO_EN
    //���������
    if ((wRegCnt != 0) && (wRegCnt != 0xFF00)) return mbexc_ILLEGAL_DATA_VALUE;
    if (wRegStart >= MBF_COIL_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;

    //������������ ������ �� ����, ������� �����
    if (wRegCnt) gmbf_Coils[wRegStart>>3] |= (1 << (wRegStart & 7));
    else gmbf_Coils[wRegStart>>3] &= (~(1 << (wRegStart & 7)));
    g_wModbusCnt = 6;	//������ �������
  #endif
    return 0;
}

//������ ������ �����
unsigned char fmbk_WriteSingleRegister(void)
{
    //���������
    if (wRegStart >= MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
    //������������ ������ � �������, ������� �����
    gmbf_HoldingRegisters[wRegStart] = wRegCnt;
    g_wModbusCnt = 6;	//������ �������
    return 0;
}

//������ ������
unsigned char fmbk_WriteCoils(void)
{
  #ifdef MCO_EN
    unsigned short i, wPosPut;
    unsigned char bMaskGet, bMaskPut, bPosGet;

    unsigned char Nn;
    //���������

    Nn = g_ModbusBuf[6];	//���������� ���� � ������� ������
    if ((wRegCnt < 1) || (wRegCnt > 0x7B0) || (Nn != g_wModbusCnt - 7)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_COIL_CNT) || (wRegStart + wRegCnt > MBF_COIL_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;

    //���������� �����
    wPosPut = wRegStart >> 3;		//������� � ������� ������
    bPosGet = 0;
    bMaskPut = 1 << (wRegStart & 0x07);	//����� ������� ������� � ������ ������
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
    g_wModbusCnt = 6;	//������ �������
  #endif
    return 0;
}

//������ ���������
unsigned char fmbk_WriteHoldingRegisters(void)
{
    unsigned short i;
    unsigned char Nn;
    //���������
    Nn = g_ModbusBuf[6];	//���������� ���� � ������� ������
    if ((wRegCnt < 1) || (wRegCnt > 0x7B) || (Nn != g_wModbusCnt - 7)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_HOLDREG_CNT) || (wRegStart + wRegCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //���������� ��������
    for (i = 0; i < wRegCnt; i++) gmbf_HoldingRegisters[wRegStart+i] =
            (g_ModbusBuf[7+i*2] << 8) | g_ModbusBuf[8+i*2];
    g_wModbusCnt = 6;	//������ �������
    return 0;
}

//������ �������� �� �����
unsigned char fmbk_MaskHoldingRegister(void)
{
    unsigned char wOr;
    //���������
    if (wRegStart >= MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
    //���������� �������
    wOr = (g_ModbusBuf[6] << 8) | g_ModbusBuf[7];	//����� "OR", ����� "AND" - � RegCnt
    gmbf_HoldingRegisters[wRegStart] = (gmbf_HoldingRegisters[wRegStart] & wRegCnt) | (wOr & (~wRegCnt));
    g_wModbusCnt = 8;	//������ �������
    return 0;
}

//������ ���������
unsigned char fmbk_ReadWriteHoldingRegisters(void)
{
    unsigned short i, wWriteStart, wWriteCnt;
    unsigned char Nn;
    //���������
    wWriteStart = (g_ModbusBuf[6] << 8) | g_ModbusBuf[7];
    wWriteCnt = (g_ModbusBuf[8] << 8) | g_ModbusBuf[9];
    Nn = g_ModbusBuf[10];	//���������� ���� � ������� ������

    if ((wRegCnt < 1) || (wRegCnt > 0x7D)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wWriteCnt < 1) || (wWriteCnt > 0x79) || (Nn != g_wModbusCnt - 11)) return mbexc_ILLEGAL_DATA_VALUE;
    if ((wRegStart >= MBF_HOLDREG_CNT) || (wRegStart + wRegCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    if ((wWriteStart >= MBF_HOLDREG_CNT) || (wWriteStart + wWriteCnt > MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
    //���������� ��������
    for (i = 0; i < wWriteCnt; i++)
        gmbf_HoldingRegisters[wWriteStart+i] = (g_ModbusBuf[11+i*2] >> 8) | g_ModbusBuf[12+i*2];
    //������ ��������
    for (i = 0; i < wRegCnt; i++)
    {
        g_ModbusBuf[3+i*2] = HIBYTE(gmbf_HoldingRegisters[i+wRegStart]);
        g_ModbusBuf[4+i*2] = LOBYTE(gmbf_HoldingRegisters[i+wRegStart]);
    }
    g_ModbusBuf[2] = wRegCnt * 2;	//���-�� ���� ������
    g_wModbusCnt = wRegCnt * 2 + 3;	//������ �������
    return 0;
}

//����������� �����
unsigned char fmbk_ReadDiagnostic(void)
{

    return 0;
}

#define MBF_CNT 11	//���������� ������������������ MODBUS �������

//������ ������������������ MODBUS �������
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

//��������� PDU, ��������� - ��� ���������� (���� 0, ���� ��� OK)
// ������: �������� �� ���������� PDU
// ������: ����������� ����� ������
unsigned char fmb_ValidateAndProcessPDU(void)
{
    unsigned char i, nf;
    nf = 0xFF;
    for (i = 0; i < MBF_CNT; i++) //���� ������� � ������
    {
        if (MBList[i].bFuncId == g_ModbusBuf[1])
        {
            nf = i;
            break;
        }
    }
    if (nf == 0xFF) return mbexc_ILLEGAL_FUNCTION;	//������� ������ ��

    wRegStart = (g_ModbusBuf[2] << 8) | g_ModbusBuf[3];	//��������� ��� ����������� �������
    wRegCnt = (g_ModbusBuf[4] << 8) | g_ModbusBuf[5];

    if ((MBList[nf].bUseInBroadcast == 0) && (g_ModbusBuf[0] == 0)) return mbexc_ILLEGAL_FUNCTION;	//������� �� �������������� � broadcast

    //���� ����� ������ �����������, R=mbexc_ILLEGAL_DATA_VALUE
    //���� ������ ��������� �����������, R=mbexc_ILLEGAL_DATA_ADDRESS
    //���� ������ �����������, R=mbexc_ILLEGAL_DATA_VALUE
    return (MBList[nf].pProcessBody) ? (*MBList[nf].pProcessBody)() : mbexc_ILLEGAL_FUNCTION;	//���� ��������, ���� �� ��� �� �������� �������
}

//��������� �������� RTU
//����� ������ - ���� ����� �������� �����, ���� �� ��������
void fmb_OnNormalRTU(void)
{
    unsigned char r;
    //MBSerialStat.wTotal++;	//���������� ���������� �������� ������� �����������
    if ((g_ModbusBuf[0] == MODBUS_ADDRESS) || (g_ModbusBuf[0] == 0))   //��������� ����� ���������� ��� broadcast
    {
        //MBSerialStat.wThisCnt++; //���������� �������� ������� ��� ������� ���������� �����������
        r = fmb_ValidateAndProcessPDU(); //�������� �� ���������� � ���������
        if (r != 0)  	// ���� ����������
        {
            //MBSerialStat.wExceptCnt++;	//����� ����������� ����� ������
            //if (r==mbexc_ACKNOWLEDGE) MBSerialStat.wNAKCnt++;			//�� ����� ���������� ��������� ����������
            //if (r==mbexc_SLAVE_DEVICE_BUSY) MBSerialStat.wBusyCnt++;	//�� ����� ���������� ��������� ����������
            g_ModbusBuf[1] |= 0x80;	//�������� ��������� - ��� ������� + 0x80
            g_ModbusBuf[2] = r;		//��� ����������
            g_wModbusCnt = 3;		//���������� ���� � �������� �������
        }
        if (g_ModbusBuf[0] == 0) //���� ��� broadcast
        {
            g_wModbusCnt = 0; //����� �����, �.�.�������� ��� �� ����
        }
    }
    else g_wModbusCnt = 0;
    if (g_wModbusCnt)   //����� ��� �������� ����, ������� ����������� �����
    {
        unsigned short crc = ModBusCRC16(g_ModbusBuf, g_wModbusCnt);
        g_ModbusBuf[g_wModbusCnt++] = LOBYTE(crc);
        g_ModbusBuf[g_wModbusCnt++] = HIBYTE(crc);
    }
    //if (g_bModbusnInListenOnly) g_wModbusCnt=0;
}

void fmb_OnCompleteMBPacket(void)	//�� ���������� ������ ������-������,
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
