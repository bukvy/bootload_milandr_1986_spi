#ifndef FP_H_
#define FP_H_

#include "stdint.h"

extern void mod_fp_init(void);
extern void mod_fp_remove(void);

extern void di_process_job(void);


enum fp_error ferret_proto_handler_2(uint16_t cmd, uint8_t *cmd_data, uint16_t cmd_len, uint8_t *ack_data, uint16_t *ack_len);





/**
 * @struct ExpboardParametersSetHeader
 * @brief ����������� ��������� ��������� ���������� ������ ����������.
 */
#pragma pack(push, 1)
typedef struct ExpboardParametersSetHeader {
	uint16_t  expboard_id; 	/**< @brief ��� ������ */
	uint16_t  version;  	/**< @brief ������ ��������� */
	uint32_t  size;    		/**< @brief ������ ��������� � ������ */
} ParametersSetHeader_t;


/**
 * @struct  ExpboardCommonParameters
 * @brief   ��������� ����� ���������� �������
 */
typedef struct ExpboardCommonParameters {
	uint8_t   board_num; 			/**< @brief ���� ���������� ������ ����������.*/
	uint8_t   ports_count;  		/**< @brief ���������� ������*/

	uint16_t  data_type;  			/**< @brief ��� ������, ������������ ��� ������� � ��������� �������� � ������ ������.*/
	uint16_t  poll_period;  		/**< @brief ������ ���������� ������ (������������) ���������� ������*/
	uint8_t   leds_timeout;  		/**< @brief ������� ���������� ���������, �.*/

	uint8_t   pm_state : 3;  		/**< @brief ����� ���������������� (�����������������)*/
	uint8_t   leds_cfg_state : 1;  	/**< @brief ��������� ������������ ���������*/
	uint8_t   user_led : 2;  		/**< @brief ��������� ����������������� ����������*/
	uint8_t   enable_polling : 1;  	/**< @brief ���� ��������������  ���������� ������ (������������) ���������� ������*/
	uint8_t   unused : 1;
} ExpboardParameters_t;



/**
 * @struct ChannelParametersDI32_Input
 * @brief ��������� ���������� ������ ����� Di-32
 */
//typedef struct ChannelParametersDI48_Input {
//    uint8_t  pm_state : 1;     	/**< @brief ��� ��������� ������� ����� �����-������*/
//    uint8_t  bs_pm_state : 1;  	/**< @brief ��������� ������� ����� �����-������ � ������ ����������*/
//    uint8_t  unused : 6;
//
//    uint8_t  alarm_value;      	/**< @brief �������� ������ ������� �� ����� �����-������*/
//    uint8_t  mode;              /**< @brief ��� ���� ����� �����-������*/
//    uint8_t  bs_mode;          	/**< @brief ��� ���� ����� �����-������ � ������ ����������*/
//
//} ChannelParametersDI48_Input_t;


typedef struct ChannelParametersDI48_Input {
    uint8_t  pm_state : 1;     	/**< @brief ��� ��������� ������� ����� �����-������*/
    uint8_t  bs_pm_state : 1;  	/**< @brief ��������� ������� ����� �����-������ � ������ ����������*/
    uint8_t  unused : 6;

    //uint8_t  alarm_value;      	/**< @brief �������� ������ ������� �� ����� �����-������*/
    uint8_t  mode;              /**< @brief ��� ���� ����� �����-������*/
    //uint8_t  mode:4;              /**< @brief ��� ���� ����� �����-������*/
    //uint8_t  bs_mode:4;          	/**< @brief ��� ���� ����� �����-������ � ������ ����������*/

} ChannelParametersDI48_Input_t;




/**
 * @struct ExpBoardParametersDI32
 * @brief ��������� ���������� ���������� ������ �����-������ DI32;
 */
typedef struct ExpBoardParametersDI48 {
    ParametersSetHeader_t      		header;   	/*8 *< @brief ��������� ����� ����������.*/
    ExpboardParameters_t       		common; 	/*8 *< @brief ����� ���������.*/
    ChannelParametersDI48_Input_t 	port[48];       /*4*48 *< @brief ��������� ������� ������.*/
} ExpBoardParametersDI48_t ;

#pragma pack(pop)


extern ExpBoardParametersDI48_t par_c;


#endif /* FP_H_ */
