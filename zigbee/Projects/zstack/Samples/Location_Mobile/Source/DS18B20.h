
/* 包含头文件 */
/********************************************************************/
#include "hal_lcd.h"
#include "hal_board_cfg.h"
/********************************************************************/



/* 宏定义 */
/********************************************************************/
#define DQ P1_2 
#define SET_DQ_OUT P1DIR|=0x04    //设置P1_2为输出
#define SET_DQ_IN P1DIR&=~0x04    //设置P1_2为输入
#define SquareWaveTimes 10        //发射超声波脉冲的次数



/* 本地变量 */
/********************************************************************/
//static uint16 temperature;
//static uint8 sensor_temperature_value[2];
/********************************************************************/

//本地函数
/********************************************************************/
extern void init_1820();
extern void sendTemperature(void);
extern uint16 read_data(void);
extern uint8 read_1820(void);
extern void write_1820(uint8 x);
extern void halMcuWaitUs(unsigned int usec);

/*********************************************************************/