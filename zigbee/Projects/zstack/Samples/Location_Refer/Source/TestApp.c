/*********************************************************************
  Filename:       RefNode.c
  Revised:        $Date: 2007-02-15 15:11:28 -0800 (Thu, 15 Feb 2007) $
  Revision:       $Revision: 13481 $

  Description: Reference Node for the Z-Stack Location Profile.
				
  Copyright (c) 2006 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reproduce, copy, prepare
  derivative works, modify, distribute, perform, display or sell this
  software and/or its documentation for any purpose is prohibited
  without the express written consent of Texas Instruments, Inc.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_Nv.h"
#include "MT.h"
#include "AF.h"
#include "ZDApp.h"

#include "OnBoard.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
#include "hal_lcd.h"
#include "hal_led.h"
#endif

#include "LocationProfile.h"
#include "TestApp.h"
#include "OSAL_Clock.h"
//#include "hal_timer.h"
#include "mac_mcu.h"

#include "UltraRec.h"
/*********************************************************************
 * CONSTANTS
 */
#define LOCATION_REFER_ID     3 //是参考节点的节点号，烧入不同节点的时候需要更改
#define RECV_TIMES 4            //此处要与sink节点的广播次数相同  之后写到LocationProfile.h中，共享一个变量名
//#define TIMEDIFF_MSG_LENGTH   5  //TIMEDIFF_MSG_LENGTH = （SeqNo RefNo）2bytes + Data（3bytes） ;    时间差的长度 因为数组只能为常量
//#define RECV_TIMEOUT          400 //第一次接收到广播信号后开始计时，时间差应该大于发射次数-1(3)*间隔100此处取值400ms
//#define RECV_DELAY_PERIOD     25  //参考节点延迟接收的时间，25ms，延迟接收3次，此处要与移动节点一致
#define REFER_RECV_DELAY_TIMES      4 //一次广播接收4次超声波信号，用于延迟接收

/* 清除TIMER3和4中断标志位 */
/********************************************************************/
#define CLR_TIMER34_IF( bitMask ) TIMIF=(TIMIF&0x40)|(0x3F&(~bitMask))

//定时器3 设置为  128分频 up-down模式
#define TIMER34_INIT(timer) do{ T##timer##CTL = 0xEB; TIMIF = 0x00;} while (0)  

// 定时器3使能
#define TIMER3_RUN(value)      (T3CTL = (value) ? T3CTL | 0x10 : T3CTL & ~0x10)


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

MAC_T2_TIME RfArrivalTime;
MAC_T2_TIME UltraArrivalTime;
byte Refer_TaskID;
static byte transId;
static unsigned int ui_sequence;  //获取序列号
static unsigned int ui_mobileID;  //获取移动节点序号
unsigned char recTimes = 0;       //接收的次数，用于计算接收的次数
int32 i_timeDifference[RECV_TIMES];  //接收时间差的Ticks数组 *31.25*10**(-7)= 时间差
//int32 i_timeDifference;             //只广播一次超声波，故只计算一次的值
//int32 i_timeDiff[RECV_TIMES];     //接收时间差数组
unsigned char timeDiffMSG[LOCATION_REFER_POSITION_RSP_LENGTH]; 
//unsigned char errorData[LOCATION_REFER_ERROR_POSITION_RSP_LENGTH];
//bool b_ErrorData = 0;             //判断接收的数据是正确的还是错误的

unsigned char recvDelayTimes = 2;      //初始化，因为已经接收了两次了，用于延迟接收的

static unsigned char t3count = 0;
/*********************************************************************
 * LOCAL VARIABLES
 */

static const cId_t Refer_InputClusterList[] =
{
  LOCATION_ULTRA_BLORDCAST
};

static const cId_t Refer_OutputClusterList[] =
{
  LOCATION_REFER_DISTANCE_RSP
};

static const SimpleDescriptionFormat_t Refer_SimpleDesc =
{
  LOCATION_REFER_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_REFER_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,

  sizeof(Refer_InputClusterList),
  (cId_t*)Refer_InputClusterList,

  sizeof(Refer_OutputClusterList),
  (cId_t*)Refer_OutputClusterList
};

static const endPointDesc_t epDesc =
{
  LOCATION_REFER_ENDPOINT,
  &Refer_TaskID,
  (SimpleDescriptionFormat_t *)&Refer_SimpleDesc,
  noLatencyReqs
};





/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processMSGCmd( afIncomingMSGPacket_t *pkt );
static void checkTime();
static MAC_T2_TIME getTime();
//static void startRecWork(void);
static void halProcessKeyInterrupt (void);
static void sendData(void);
void halSetTimer3Period(uint8 period);

/*********************************************************************
 * @fn      Refer_Init
 *
 * @brief   Initialization function for this OSAL task.
 *
 * @param   task_id - the ID assigned by OSAL.
 *
 * @return  none
 */
void Refer_Init( byte task_id )
{
  Refer_TaskID = task_id;

  // Register the endpoint/interface description with the AF.
  afRegister( (endPointDesc_t *)&epDesc );

  // Register for all key events - This app will handle all key events.
  RegisterForKeys( Refer_TaskID );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Location-RefNode", HAL_LCD_LINE_2 );
#endif
  
  
  
}

/*********************************************************************
 * @fn      RefNode_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - Bit map of events to process.
 *
 * @return  none
 */
uint16 Refer_ProcessEvent( byte task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(Refer_TaskID);
    while (MSGpkt)
    {
      switch ( MSGpkt->hdr.event )
      {
      case AF_DATA_CONFIRM_CMD:
        break;
 
      case AF_INCOMING_MSG_CMD:
        {
          processMSGCmd( MSGpkt );
          break;
        }

      case ZDO_STATE_CHANGE:      
        initP0();         //初始化端口和定时器
        break;

      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Refer_TaskID );        
    }

    // Return unprocessed events.
    return ( events ^ SYS_EVENT_MSG );
  }
  
  /*
  // Delay 25ms for receiving the next US signal
  if ( events & RECV_DELAY_EVT )    
  {    
    //记下延迟25ms之后的计数器的值，作为起算时间
    RfArrivalTime = getTime();
    IEN2 |= 0x10;             //打开P1的中断使能
    bFirstRcv = TRUE;         //设置下次还能接受
    
    if(recvDelayTimes == REFER_RECV_DELAY_TIMES)
    {
      recvDelayTimes = 2;
    }
    else
    {
      osal_start_timerEx( Refer_TaskID, RECV_DELAY_EVT, RECV_DELAY_PERIOD );
      recvDelayTimes++;
    }
     
    // return unprocessed events
    return ( events ^ RECV_DELAY_EVT );
  }
  */
    
  // send the data by delaying with 10ms*ID
  if ( events & REFER_DELAYSEND_EVT )    
  {
    afAddrType_t coorAddr;
    coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
    coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
    coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
    AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                             LOCATION_REFER_DISTANCE_RSP, LOCATION_REFER_POSITION_RSP_LENGTH,
                             timeDiffMSG, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );     
    /*
    if (b_ErrorData == 0)
    {
      AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                             LOCATION_REFER_DISTANCE_RSP, LOCATION_REFER_POSITION_RSP_LENGTH,
                             timeDiffMSG, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );      
    }
    else
    {
      AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                             LOCATION_REFER_DISTANCE_RSP, 3,
                             errorData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );  
    }
    */
    
    // return unprocessed events
    return ( events ^ REFER_DELAYSEND_EVT );
  }
  
  return 0;  // Discard unknown events
}

/*********************************************************************
 * @fn      processMSGCmd
 *
 * @brief   Data message processor callback.
 *
 * @param   none
 *
 * @return  none
 */
static void processMSGCmd( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case LOCATION_ULTRA_BLORDCAST:
    {
      //设置定时器
      //osal_start_timerEx( Refer_TaskID, RECV_DELAY_EVT, RECV_DELAY_PERIOD );
      
      //开启定时器T3计时，25ms
      TIMER3_RUN(TRUE);
      
      //记下电磁波到达时间，作为开始计算的时间
      RfArrivalTime = getTime();            
      //IEN2 |= 0x10;             //打开P1的中断使能
      bFirstRcv = TRUE;         //设置下次还能接受
      
      ui_sequence = pkt->cmd.Data[0];         //获取序列号
      ui_mobileID = pkt->cmd.Data[1];         //获取移动节点号
   
      break;
    }
      
    default:
      break;
    }
}

/*********************************************************************
 * @fn      getTime
 *
 * @brief   get the signal arrival time.
 *
 * @param   MAC_T2_TIME - store the signal arrival time.
 *          
 *          
 x
 * @return  none
 */
static MAC_T2_TIME getTime()
{
  MAC_T2_TIME signalArrivalTime;
  //获取MAC层的时间 from树浩论文 原函数在mac_mcu.c
  halIntState_t  s;
  
  // for efficiency, the 32-bit value is encoded using endian abstracted indexing 
  HAL_ENTER_CRITICAL_SECTION(s);
  
  // This T2 access macro allows accessing both T2MOVFx and T2Mx //
  MAC_MCU_T2_ACCESS_OVF_COUNT_VALUE();
  //MAC_MCU_T2_ACCESS_OVF_CAPTURE_VALUE();
  
  // Latch the entire T2MOVFx first by reading T2M0. 
  T2M0;
  ((uint8 *)&signalArrivalTime.overflowCapture)[0] = T2MOVF0;
  ((uint8 *)&signalArrivalTime.overflowCapture)[1] = T2MOVF1;
  ((uint8 *)&signalArrivalTime.overflowCapture)[2] = T2MOVF2;
  ((uint8 *)&signalArrivalTime.overflowCapture)[3] = 0;
  
  MAC_MCU_T2_ACCESS_COUNT_VALUE();
  //MAC_MCU_T2_ACCESS_CAPTURE_VALUE();
  signalArrivalTime.timerCapture = T2M1 << 8;
  signalArrivalTime.timerCapture |= T2M0;
  
  HAL_EXIT_CRITICAL_SECTION(s);
  return signalArrivalTime;
}      
      
/*********************************************************************
 * @fn      checkTime
 *
 * @brief   
 *
 * @param   
 *          
 *          
 x
 * @return  none
 */
static void checkTime()
{
  //接收了4次超声波的方法
  
  if (recTimes < RECV_TIMES)        //判断是否接收完了相应的次数
  {
         
    if (UltraArrivalTime.overflowCapture == 0 & UltraArrivalTime.timerCapture == 0)  //判断超声波是否接收到了，表示超声波未接收到
    {
      i_timeDifference[recTimes] = 0;
    }
    else
    {
      //电磁波先到的计算方法
      i_timeDifference[recTimes] = ((UltraArrivalTime.overflowCapture *10240) + UltraArrivalTime.timerCapture) - ((RfArrivalTime.overflowCapture *10240) + RfArrivalTime.timerCapture);
      
      //超声波先到的计算方法
      //i_timeDifference = ((RfArrivalTime.overflowCapture *10240) + RfArrivalTime.timerCapture) - ((UltraArrivalTime.overflowCapture *10240) + UltraArrivalTime.timerCapture);
    }
  
    if (i_timeDifference[recTimes] > 941177 | i_timeDifference[recTimes] <0)   // 941177*31.25ns *340m/s = 10m  设为测试的极限距离 如比这个还大，则为错误
    {
      i_timeDifference[recTimes] = 0;
    }

    //ui_T4time = ui_T4Overflow * 256 + ui_T4Count;
      
    //UltraArrivalTime.overflowCapture = 0;  //将两个时刻清零
    //UltraArrivalTime.timerCapture = 0;
    //RfArrivalTime.overflowCapture = 0;  
    //RfArrivalTime.timerCapture = 0;

    recTimes++;
  }
  if ( recTimes == RECV_TIMES )        //判断是否接受完了RECV_TIMES次数  
  {    
    //unsigned char timeDiffMSG[REFER_TIMEDIFF_MSG_LEN];  
    //unsigned char timeDiffMSG[8];           //测试T4和T2定时器的差别
    //static unsigned int const timeDiffMsgLength;
    //timeDiffMsgLength = 2 + 4 * RECV_TIMES;      //时间差的长度
    
    //IEN2 &= ~0x10;       //关闭端口1中断使能 
    //发送数据给协调器
    sendData();
    recTimes = 0;         //将接收次数清0  
  }
  
  /*
  //只接收一次超声波的方法
  
  if (UltraArrivalTime.overflowCapture == 0 & UltraArrivalTime.timerCapture == 0)  //判断超声波是否接收到了，表示超声波未接收到
  {
    i_timeDifference = 0;
  }
  else
  {
    //电磁波先到的计算方法
    i_timeDifference = ((UltraArrivalTime.overflowCapture *10240) + UltraArrivalTime.timerCapture) - ((RfArrivalTime.overflowCapture *10240) + RfArrivalTime.timerCapture);
    
    //超声波先到的计算方法
    //i_timeDifference = ((RfArrivalTime.overflowCapture *10240) + RfArrivalTime.timerCapture) - ((UltraArrivalTime.overflowCapture *10240) + UltraArrivalTime.timerCapture);
  }

  if (i_timeDifference > 941177 | i_timeDifference <0)   // 941177*31.25ns *340m/s = 10m  设为测试的极限距离 如比这个还大，则为错误
  {
    i_timeDifference = 0;
  }
  */
  //ui_T4time = ui_T4Overflow * 256 + ui_T4Count;
    
  UltraArrivalTime.overflowCapture = 0;  //将两个时刻清零
  UltraArrivalTime.timerCapture = 0;
  RfArrivalTime.overflowCapture = 0;  
  RfArrivalTime.timerCapture = 0;
  
  //sendData();
}

/*********************************************************************
 * 函数名称：sendData
 * 功    能：启动发射数据函数入口，发至协调器
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void sendData(void)
{
  /*
  //test
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_SEQUENCE] =  ui_sequence;      //序列号  注意此处的帧没有MSG_TYPE
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_FIXID] = LOCATION_REFER_ID;         //参考节点ID
  
  int n;
  for(n=0;n<RECV_TIMES;n++)
  {
    timeDiffMSG[4*n+2] = ((uint8 *)&i_timeDifference[n])[3];      //时间差高最八位 
    timeDiffMSG[4*n+3] = ((uint8 *)&i_timeDifference[n])[2];      //时间差高八位 T2计时器
    timeDiffMSG[4*n+4] = ((uint8 *)&i_timeDifference[n])[1];      //时间差中八位
    timeDiffMSG[4*n+5] = ((uint8 *)&i_timeDifference[n])[0];      //时间差低八位
  }
  timeDiffMSG[4*n+2]='\n';
  */
  
  
  //接收4次的结果计算平均值的方法
  
  uint32 ui_totalTimeDiff = 0;
  int32 i_averageTime = 0;
  int32 i_goodTimeDifference[RECV_TIMES];
  unsigned int ui_goodValue = 0;
  unsigned int goodValueTimes = 0;
  int n;
  for (n=0;n<RECV_TIMES;n++)
  {
    if(i_timeDifference[n] != 0)
    {
      ui_totalTimeDiff += i_timeDifference[n];
      i_goodTimeDifference[ui_goodValue] = i_timeDifference[n];
      ui_goodValue++;
    }
  }
  if (ui_goodValue != 0)
  {
    i_averageTime = ui_totalTimeDiff / ui_goodValue;
    goodValueTimes = ui_goodValue;
    for (n=0;n<goodValueTimes;n++)
    {
      //10cm = Tick * 31.25 * 10**(-7) * 340 ==> Tick = 9412 ,求平均取绝对值，看是否在这个区间之外
      //无语了，不知道怎么求绝对值，自己写
      uint32 compare;
      if((i_goodTimeDifference[n] - i_averageTime) >= 0)
      {
        compare = i_goodTimeDifference[n]-i_averageTime;
      }
      else
      {
        compare = i_averageTime - i_goodTimeDifference[n];
      }
       
      if (compare>=9412)
      {
        i_goodTimeDifference[n] = 0;
        ui_goodValue--;
      }
    }
    if (ui_goodValue ==0)
    {
      //there is no good value;
      i_averageTime = 0;
    }
    else
    {
      ui_totalTimeDiff = 0;
      for(n=0;n<goodValueTimes;n++)
      {
        ui_totalTimeDiff += i_goodTimeDifference[n];
      }
      i_averageTime = ui_totalTimeDiff / ui_goodValue;
    }
  }
  else 
  {
    i_averageTime = 0;
  }
  
  //设置为正确的数据
  //b_ErrorData = 0;
  
  //unsigned char timeDiffMSG[LOCATION_REFER_POSITION_RSP_LENGTH]; 
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_SEQUENCE] =  ui_sequence;      //序列号  注意此处的帧没有MSG_TYPE
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_MOBID] =  ui_mobileID;      //移动节点ID
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_REFID] = LOCATION_REFER_ID;         //参考节点ID
  
  
  if (i_averageTime!=0)
  {  
    //考虑到假设最远为10m，则Tick的最大值为10/340/31.25*10**9=941176,24位就能传完，为了节省传输量，只选了24位。
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_HIGH] = ((uint8 *)&i_averageTime)[2];      //时间差高八位  
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_MIDD] = ((uint8 *)&i_averageTime)[1];      //时间差中八位
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_LOW] = ((uint8 *)&i_averageTime)[0];      //时间差低八位
   }  
  //错误数据将发送为全FF
  else
  {
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_HIGH] = 0xFF;      //时间差高八位  
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_MIDD] = 0xFF;      //时间差中八位
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_LOW] = 0xFF;      //时间差低八位
  }
  
  
  
  int temp;
  for(temp = 0;temp < RECV_TIMES;temp++)
  {
    i_timeDifference[temp] = 0;               //将接收时间差数组清零
  } 
  
  
    
    /*
    int n;
    for(n=0;n<RECV_TIMES;n++)
    {
      timeDiffMSG[4*n+2] = ((uint8 *)&i_timeDifference[n])[3];      //时间差高最八位 
      timeDiffMSG[4*n+3] = ((uint8 *)&i_timeDifference[n])[2];      //时间差高八位 T2计时器
      timeDiffMSG[4*n+4] = ((uint8 *)&i_timeDifference[n])[1];      //时间差中八位
      timeDiffMSG[4*n+5] = ((uint8 *)&i_timeDifference[n])[0];      //时间差低八位
    }
    */
    //timeDiffMSG[4*n+2]='\n';                                       //直接在协调器上添加，减少网络中的传输      //添加\n 终止符，在接收端接收
      
    //timeDiffMSG[2] = ((uint8 *)&ui_T4time)[3];      //时间差最高八位  T4计时器
    //timeDiffMSG[3] = ((uint8 *)&ui_T4time)[2];      //时间差高八位  
    //timeDiffMSG[4] = ((uint8 *)&ui_T4time)[1];      //时间差中八位
    //timeDiffMSG[5] = ((uint8 *)&ui_T4time)[0];      //时间差低八位
    
    //timeDiffMSG[RP_TIMEDIFF_HIGH] = ((uint8 *)&ui_T4time)[2];      //时间差高八位  T4计时器
    //timeDiffMSG[RP_TIMEDIFF_MIDD] = ((uint8 *)&ui_T4time)[1];      //时间差中八位
    //timeDiffMSG[RP_TIMEDIFF_LOW] = ((uint8 *)&ui_T4time)[0];      //时间差低八位
    
    
  //接收一次超声波的方法,增加了移动节点ID
  /*
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_SEQUENCE] =  ui_sequence;      //序列号  注意此处的帧没有MSG_TYPE
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_MOBID] =  ui_mobileID;      //移动节点ID
  timeDiffMSG[LOCATION_REFER_POSITION_RSP_REFID] = LOCATION_REFER_ID;         //参考节点ID
  
  if (i_timeDifference!=0)
  {  
    //考虑到假设最远为10m，则Tick的最大值为10/340/31.25*10**9=941176,24位就能传完，为了节省传输量，只选了24位。
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_HIGH] = ((uint8 *)&i_timeDifference)[2];      //时间差高八位  
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_MIDD] = ((uint8 *)&i_timeDifference)[1];      //时间差中八位
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_LOW] = ((uint8 *)&i_timeDifference)[0];      //时间差低八位
   }  
  //错误数据将发送为全FF
  else
  {
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_HIGH] = 0xFF;      //时间差高八位  
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_MIDD] = 0xFF;      //时间差中八位
    timeDiffMSG[LOCATION_REFER_POSITION_RSP_DSITANCE_LOW] = 0xFF;      //时间差低八位
  }
  */
  
  //开启一个事件，延迟10ms*ID的时间发送消息，避免碰撞
  unsigned int delaySend;
  delaySend = 10 * LOCATION_REFER_ID;
  osal_start_timerEx( Refer_TaskID, REFER_DELAYSEND_EVT, delaySend );
  //i_timeDifference = 0;
  
}

/*********************************************************************
 * 函数名称：startRecWork
 * 功    能：启动接收函数入口
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
//void startRecWork(void)
//{
  //ucGainNo = 11;        //由于T1定时器无法工作，改为直接设置为最大增益，
  /*
  HalTimerInit ();
  HalTimerConfig(HAL_TIMER_3,HAL_TIMER_MODE_CTC,
                 HAL_TIMER_CHANNEL_SINGLE,HAL_TIMER_CH_MODE_OUTPUT_COMPARE,
                 TRUE,TurnP0CallBack);
  HalTimerStart(HAL_TIMER_3,100);
  halTimerSetPrescale(HAL_TIMER_3,0x0A);
  */
  
  //设置定时时间，时间到了系统产生REFER_SETGAIN_EVT事件
  //osal_start_timerEx( Refer_TaskID, REFER_SETGAIN_EVT, uiGainTime[ucGainNo] );
  
  
  //ucGainNo = 0;
  //halSetTimer1Period(uiGainTime[ucGainNo]);
  //setGain(ucGainNo);

  //TIMER1_ENABLE_OVERFLOW_INT(TRUE);
  //IEN1 |=(0x01<<1);         //使能Timer1的中断
  //TIMER1_RUN(TRUE);
  
  
  //HAL_ENABLE_INTERRUPTS();   //使能全局中断
  
  //Inhi = 0;
//}

/*********************************************************************
*********************************************************************/

/*********************************************************************
 * 函数名称：halSetTimer1Period
 * 功    能：设置定时器1定时周期
 * 入口参数：period   定时周期       
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
/*
void halSetTimer1Period(uint16 period)
{
  //给T1CC0写入最终计数值period 
  T1CC0L = period & 0xFF;                      // 把period的低8位写入T1CC0L
  T1CC0H = ((period & 0xFF00)>>8);             // 把period的高8位写入T1CC0H
}
*/

/*********************************************************************
 * 函数名称：setGain
 * 功    能：设置GCA GCB GCC GCD 的增益大小
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void setGain(unsigned char ucGainNo)
{
  switch (ucGainNo)
  {
    case 0:
    {
      GCA = 0;
      GCB = 0;
      GCC = 0;
      GCD = 0;
      break;
    }  
    case 1:
    {
      GCA = 1;
      GCB = 0;
      GCC = 0;
      GCD = 0;
      break;
    }  
    case 2:
    {
      GCA = 0;
      GCB = 1;
      GCC = 0;
      GCD = 0;
      break;
    }  
    case 3:
    {
      GCA = 1;
      GCB = 1;
      GCC = 0;
      GCD = 0;
      break;
    }  
    case 4:
    {
      GCA = 0;
      GCB = 0;
      GCC = 1;
      GCD = 0;
      break;
    }  
    case 5:
    {
      GCA = 1;
      GCB = 0;
      GCC = 1;
      GCD = 0;
      break;
    }  
    case 6:
    {
      GCA = 0;
      GCB = 1;
      GCC = 1;
      GCD = 0;
      break;
    }  
    case 7:
    {
      GCA = 1;
      GCB = 1;
      GCC = 1;
      GCD = 0;
      break;
    }  
    case 8:
    {
      GCA = 0;
      GCB = 0;
      GCC = 0;
      GCD = 1;
      break;
    }  
    case 9:
    {
      GCA = 1;
      GCB = 0;
      GCC = 0;
      GCD = 1;
      break;
    }  
    case 10:
    {
      GCA = 0;
      GCB = 1;
      GCC = 0;
      GCD = 1;
      break;
    }  
    
    default:
    {
      GCA = 1;
      GCB = 1;
      GCC = 0;
      GCD = 1;
      break;
    }  
  }
}

/*********************************************************************
 * 函数名称：T1_IRQ
 * 功    能：定时器1中断服务函数 注意入口函数与裸机中断入口函数不同 实际效果不理想，故舍去
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
/*
_PRAGMA(vector=T1_VECTOR) __near_func __interrupt void halTimer1Isr(void);  
HAL_ISR_FUNCTION( halTimer1Isr, T1_VECTOR)
{
  IEN1 &= ~0x02;    //关闭定时器1中断使能
  if(T1STAT & 0x21) //确认是否产生计时中断
  {
    ucGainNo++;
    if(ucGainNo <11)
    {
      halSetTimer1Period(uiGainTime[ucGainNo]);
      setGain(ucGainNo);
    }
    else 
    {
      if(ucGainNo < TIME_OUT)
      {
        setGain(11);
      }
      else          //超时，未接收到超声波信号
      {
        TIMER1_RUN(FALSE);    //关闭定时器1
        ucGainNo = 0;
        setGain(ucGainNo);
        bRcvFinish = 1;    //设置为接收完成
      }
    }
    
  }
  T1STAT &= ~0x21;  //清除定时器1的标志位
  IRCON &= ~0x02;   //同上
  IEN1 |= 0x02;    //打开定时器1中断使能
  
}
*/

/*********************************************************************
 * 函数名称：P1ISR
 * 功    能：IO1中断服务函数，P1_2检测到下降沿后即会进入 注意入口函数与裸机中断入口函数不同
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
HAL_ISR_FUNCTION(P1_IRQ,P1INT_VECTOR)  
 {
    if(P1IFG>0)
    {
      halProcessKeyInterrupt();
      
      P1IFG = 0;
    }
    P1IF = 0;             //清除中断标记，位于IRCON寄存器中
 }

/*********************************************************************
 * 函数名称：halProcessKeyInterrupt
 * 功    能：定时器3中断服务函数，P0_0检测到下降沿后即会进入 注意入口函数与裸机中断入口函数不同
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halProcessKeyInterrupt (void)
{
    if (bFirstRcv == TRUE)        //判断是否是第一次接收
    {       
      //P1_1 = 1;               //测试使用
      //TIMER4_RUN(TRUE);       //开启定时器4
     
      UltraArrivalTime = getTime();

      if(YelLed_State == 0)
      {
        HAL_TURN_ON_LED3();
        YelLed_State = 1;
      }
      else
      {
        HAL_TURN_OFF_LED3();
        YelLed_State = 0;
      }
      
      bFirstRcv = FALSE;//设置为已经接收过了
      //IEN2 &= ~0x10;       //关闭端口1中断使能  
      checkTime();         //超声波后到计算
      
      
      //TIMER1_RUN(FALSE);//收到超声波后就关闭定时器1，并初始化增益
      //ucGainNo = 0;
      //setGain(ucGainNo);
      
      
    }
}


/*********************************************************************
 * 函数名称：halSetTimer3Period
 * 功    能：设置定时器3定时周期
 * 入口参数：period   定时周期       
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void halSetTimer3Period(uint8 period)
{
  /* 给T3CC0写入最终计数值period */
  T3CC0 = period & 0xFF;             // 把period的值写入T3CC0
}


/*********************************************************************
 * 函数名称：T3_IRQ
 * 功    能：定时器1中断服务函数 注意入口函数与裸机中断入口函数不同 实际效果不理想，故舍去
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/

//_PRAGMA(vector=T1_VECTOR) __near_func __interrupt void halTimer1Isr(void);  
HAL_ISR_FUNCTION( halTimer3Isr, T3_VECTOR)
{
  IEN1 &= ~0x03;    //关闭定时器3中断使能
  if(TIMIF & 0x01) //确认是否产生计时中断
  {
    t3count++;
    if(t3count == 12)     //2*12=24ms
    {
      t3count = 0;
      
      //记下电磁波到达时间，作为开始计算的时间
      RfArrivalTime = getTime(); 
      
      //P1_1 = ~P1_1;               //测试使用
      //IEN2 |= 0x10;             //打开P1的中断使能
      bFirstRcv = TRUE;         //设置下次还能接受
      
      if(recvDelayTimes == REFER_RECV_DELAY_TIMES)
      {
        recvDelayTimes = 2;
        //将定时器T3清0，为下次中断做准备
        T3CTL |= 0x04;       //将计数值清0
        TIMER3_RUN(FALSE);
      }
      else
      {
        recvDelayTimes++;
      }
    }    
  }
  
  TIMIF &= ~0x01;  
  //T1STAT &= ~0x21;  //清除定时器3的标志位
  //IRCON &= ~0x02;   //同上
  IEN1 |= 0x03;    //打开定时器3中断使能
}

/*********************************************************************
 * 函数名称：T4_IRQ
 * 功    能：定时器4中断服务函数 注意入口函数与裸机中断入口函数不同
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
/*
HAL_ISR_FUNCTION( halTimer4Isr, T4_VECTOR)
{
  //TIMER4_RUN(FALSE);    //此处不能关闭计时，因为硬件延迟也包括在此
  EA = FALSE;           //关闭总使能
  if(TIMIF & 0x08)
  {
    ui_T4Overflow++;
  }
  TIMIF &= ~0x08;
  EA = TRUE;
  //TIMER4_RUN(TRUE);
}
*/

/*********************************************************************
 * 函数名称：initP1
 * 功    能：初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void initP0()
{
   P0SEL &= ~0xBD;              //将P0_2 P0_3 P0_4 P0_5 P0_7置为GPIO
   P0DIR |= 0xBC;               //将P0_2、P0_3 P0_4 P0_5 P0_7置为输出
   
   //P1SEL &= ~0x01;              //初始化P1_1 设置为通用I/O输出 示波器测试使用 P4上的4脚
   //P1DIR &= ~0x01;
   //P1_1 = 0;
   
   P1SEL &= ~0x04;              //将P1_2设置为GPIO   
   //P0SEL |= 0x01;             //将P0_0设置为中断的外设   datasheet中如此写的
   P1DIR &= ~0x04;              //将P1_2设置为输入
   P1INP &= ~0x04;                //将P1_2设置为上拉/下拉
   P2INP &= ~0x40;                  //端口1为上拉
      
   P1IEN |= 0x04;               //P1_2设置为中断使能方式
   PICTL |= 0x02;               //P1_2为下降沿触发     
   IEN2 |= 0x10;                //打开端口1中断使能  
   P1IFG  = 0;                  //复位端口1的标志位
   P1IF  = 0; 
   
   //IP1 |= 0x20;                //设置P0INT的优先级最高
   //IP0 |= 0x20;
   
   //初始化接收数组为0
   
   int temp;
    for(temp = 0;temp < RECV_TIMES;temp++)
    {
      i_timeDifference[temp] = 0;               //将接收时间差数组清零
    }
   
   //i_timeDifference = 0;
   
    //ui_MAC_RFArrivalTime = pkt ->timestamp;
    //startRecWork(); 
   
   UltraArrivalTime.overflowCapture = 0;  //初始化将两个时刻清零
   UltraArrivalTime.timerCapture = 0;
   RfArrivalTime.overflowCapture = 0;  
   RfArrivalTime.timerCapture = 0;
   
   //TIMER34_INIT(4);                // 使能定时器4的溢出中断 
   //IEN1 |= (0x01 << 4);             // 使能Timer4的中断T4IE 
   
   //TIMER1_INIT();               //初始化定时器1 使能定时器1的溢出中断
   //T1CCTL0 |= 0x44;             //允许定时器1的通道0中断请求，并且设置为输出比较模式
   
   ucGainNo = 11;        //由于T1定时器无法工作，改为直接设置为最大增益，
   setGain(ucGainNo);
   Inhi = 0;
   
   YelLed_State = 0;          //初始化黄灯状态为关闭   
   
   TIMER34_INIT(3);
   halSetTimer3Period(250); //表示延迟2ms 注意：实际写入T1CC0寄存器的值应小于255
   IEN1 |= (0x01 << 3);             // 使能Timer3的中断

   EA = 1;                      //开总使能
}
