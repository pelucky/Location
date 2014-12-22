
/* 包含头文件 */
/********************************************************************/
#include "hal_lcd.h"
#include "hal_board_cfg.h"
#include "OSAL_Timers.h"
/********************************************************************/

/* 本地变量 */
/********************************************************************/
static bool YelLed_State;      //黄灯状态，显示接收到了超声波
static bool RedLed_State = 0;      //红灯状态，测试用

static unsigned char ucGainNo;
static unsigned int  uiGainTime[11] = {2380,2740,2750,2740,2740,2740,2750,2740,5490,5480,5490};     //计数器增长时间 uint32  ui_UltraArrivalTime;  //超声波传播到的时刻

static bool bFirstRcv = 1;            //判断是否第一次接收

//static bool bRFFirstRcv = TRUE;

static bool bRcvFinish = 0;          //判断是否接收完毕

//static uint32 ui_T4Overflow = 0;      //定时器4的溢出次数
//static unsigned int ui_T4Count;             //定时器4的计数值



typedef struct {
    uint16         timerCapture;
    uint32         overflowCapture;
}MAC_T2_TIME;


/********************************************************************/

//本地函数
/********************************************************************/
//void halSetTimer1Period(uint16 period);
void setGain(unsigned char ucGainNo);
void initP0();
/********************************************************************/


/* 宏定义 */
/********************************************************************/
#define Signal P0_0 
#define GCA    P0_2 
#define GCB    P0_3 
#define GCC    P0_4 
#define GCD    P0_5 
#define Inhi   P0_7

#define TIME_OUT 15     //最长时间，比15还大还未接收到就算超时

#define INHIBIT_TIME 880  //最短允许距离30cm

/*定时器1初始化*/ 
//32分频 modulo模式 一次32M/32=1M 一次计数就是1us
//#define TIMER1_INIT() do{ T1CTL = 0x0A; TIMIF = ~0x40;} while (0)

/*定时器1允许溢出中断*/
//#define TIMER1_ENABLE_OVERFLOW_INT(val)  (TIMIF = (val)?(TIMIF|0x40):(TIMIF&~0x40))

/*定时器1启动或关闭*/  //需要开启定时器1中的通道0
//#define TIMER1_RUN(value) (T1STAT = (value)?T1STAT|0x21:T1STAT&~0x21)

/* 清除TIMER3和4中断标志位 */
/********************************************************************/
//#define CLR_TIMER34_IF( bitMask ) TIMIF=(TIMIF&0x40)|(0x3F&(~bitMask))


//定时器4 设置为  1分频 自由运行 从0-255
//#define TIMER34_INIT(timer) do{ T##timer##CTL = 0x08; TIMIF = 0x00;} while (0)  

// 定时器4使能
//#define TIMER4_RUN(value)      (T4CTL = (value) ? T4CTL | 0x10 : T4CTL & ~0x10)