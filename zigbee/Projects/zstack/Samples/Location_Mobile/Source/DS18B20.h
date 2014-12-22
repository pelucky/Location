
/* ����ͷ�ļ� */
/********************************************************************/
#include "hal_lcd.h"
#include "hal_board_cfg.h"
/********************************************************************/



/* �궨�� */
/********************************************************************/
#define DQ P1_2 
#define SET_DQ_OUT P1DIR|=0x04    //����P1_2Ϊ���
#define SET_DQ_IN P1DIR&=~0x04    //����P1_2Ϊ����
#define SquareWaveTimes 10        //���䳬��������Ĵ���



/* ���ر��� */
/********************************************************************/
//static uint16 temperature;
//static uint8 sensor_temperature_value[2];
/********************************************************************/

//���غ���
/********************************************************************/
extern void init_1820();
extern void sendTemperature(void);
extern uint16 read_data(void);
extern uint8 read_1820(void);
extern void write_1820(uint8 x);
extern void halMcuWaitUs(unsigned int usec);

/*********************************************************************/