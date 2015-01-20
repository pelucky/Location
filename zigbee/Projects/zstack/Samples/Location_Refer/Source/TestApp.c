/*********************************************************************
  Filename:       RefNode.c
  Revised:        $Date: 2007-02-15 15:11:28 -0800 (Thu, 15 Feb 2007) $
  Revision:       $Revision: 13481 $

  Description: Reference Node for the Z-Stack Location Profile.
				
  Copyright (c) 2006 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reprodue, copy, prepare
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
#include "mac_mcu.h"
//#include "stdlib.h"            //pel+ abs()

#include "UltraRec.h"
/*********************************************************************
 * CONSTANTS
 */

#define LAST_MOBILE_ID         4  //最后一个移动节点的ID，也可以理解为一共有的移动节点的个数

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

static unsigned int ui_sequence;                                       //获取序列号
int32 i_timeDifference[LAST_MOBILE_ID];                                //接收时间差的Ticks数组 *31.25*10**(-7)= 时间差
unsigned char timeDiffMSG[R2C_DIFF_TIME_LENGTH]; 
unsigned char recvMobiID = 1;                                          //用于标志正在接受的移动节点号
unsigned char t3count = 0;

unsigned char refer_id = 1;           //是参考节点的节点号，烧入不同节点的时候需要更改
unsigned char recv_timeout = 160;     //设置一个序列号的超时时间，如果大于此还未发射表示未接满，直接发送
unsigned char delay_time = 18;        //实际延迟是2*delay_time，指每次同发射端的延迟时间

/*********************************************************************
 * LOCAL VARIABLES
 */

static const cId_t Refer_InputClusterList[] =
{
  CID_S2MR_BROADCAST,
  CID_C2A_GET_BASIC_VALUE,
  CID_C2A_SET_BASIC_VALUE   
};

static const cId_t Refer_OutputClusterList[] =
{
  CID_R2C_DIFF_TIME,
  CID_A2C_RP_BASIC_VALUE,
  CID_A2C_SUCCESS_RESPONSE
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
static void halProcessKeyInterrupt (void);
static void sendData(void);
void halSetTimer3Period(uint8 period);
void getBasicValue();
void setBasicValue(afIncomingMSGPacket_t *pkt);
void successResponse(uint8 result);

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
      
   //初始化接收数组为0
   int temp;
   for(temp = 0;temp < LAST_MOBILE_ID;temp++)
   {
     i_timeDifference[temp] = 0;               //将接收时间差数组清零
   }
   
   UltraArrivalTime.overflowCapture = 0;  //初始化将两个时刻清零
   UltraArrivalTime.timerCapture = 0;
   RfArrivalTime.overflowCapture = 0;  
   RfArrivalTime.timerCapture = 0;
   
   
   //由于T1定时器无法工作，改为直接设置为最大增益，
   GCA = 1;
   GCB = 1;
   GCC = 0;
   GCD = 1;
   Inhi = 0;
   
   RedLed_State = 0;          //初始化红灯状态为关闭   
   
   TIMER34_INIT(3);
   halSetTimer3Period(250); //表示延迟2ms 注意：实际写入T1CC0寄存器的值应小于255
   IEN1 |= (0x01 << 3);             // 使能Timer3的中断

   EA = 1;                      //开总使能
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
HAL_ISR_FUNCTION( halTimer3Isr, T3_VECTOR)
{
  IEN1 &= ~0x03;    //关闭定时器3中断使能
  if(TIMIF & 0x01) //确认是否产生计时中断
  {
    t3count++;
    if(t3count == delay_time)     //2*12=24ms
    {
      t3count = 0;
      
      //记下电磁波到达时间，作为开始计算的时间
      RfArrivalTime = getTime(); 
      
      //P1_1 = ~P1_1;               //测试使用
      //IEN2 |= 0x10;             //打开P1的中断使能
      bFirstRcv = TRUE;         //设置下次还能接受
      recvMobiID++;             //开启定时的时候就将接收节点的ID+1
      
      if(recvMobiID == LAST_MOBILE_ID)
      {
        //将定时器T3清0，为下次中断做准备
        TIMER3_RUN(FALSE);
        T3CTL |= 0x04;       //将计数值清0
      }
    }    
  }
  
  TIMIF &= ~0x01;  
  //T1STAT &= ~0x21;  //清除定时器3的标志位
  //IRCON &= ~0x02;   //同上
  IEN1 |= 0x03;    //打开定时器3中断使能
}

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
     
      UltraArrivalTime = getTime();

      if(RedLed_State == 0)
      {
        HAL_TURN_ON_LED2();
        RedLed_State = 1;
      }
      else
      {
        HAL_TURN_OFF_LED2();
        RedLed_State = 0;
      }
      
      bFirstRcv = FALSE;//设置为已经接收过了
      //IEN2 &= ~0x10;       //关闭端口1中断使能  
      checkTime();         //超声波后到计算
            
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
        //初始化时将基本数据发送给协调器   
        getBasicValue();
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
  
  // 超时则发送现有的结果
  if ( events & RECV_TIMEOUT_EVT )    
  {
    sendData();
        
    // return unprocessed events
    return ( events ^ RECV_TIMEOUT_EVT );
  }
  
  
  // send the data by delaying with 10ms*ID
  if ( events & REFER_DELAYSEND_EVT )    
  {
    afAddrType_t coorAddr;
    coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
    coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
    coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
    AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                             CID_R2C_DIFF_TIME, R2C_DIFF_TIME_LENGTH,
                             timeDiffMSG, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );   
    //将发送数组清零
    int i;
    for(i=0;i<R2C_DIFF_TIME_LENGTH;i++)
    {
      timeDiffMSG[i] = 0;     
    }
       
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
    case CID_S2MR_BROADCAST:
    {
      //开启定时器T3计时，24ms
      TIMER3_RUN(TRUE);
            
      //记下电磁波到达时间，作为开始计算的时间
      RfArrivalTime = getTime();            
      //IEN2 |= 0x10;             //打开P1的中断使能
      bFirstRcv = TRUE;         //设置下次还能接受
      recvMobiID = 1;  
      
      ui_sequence = pkt->cmd.Data[0];         //获取序列号
      
      //设置超时时间，将获取的4个移动节点的发送给协调器
        osal_start_timerEx( Refer_TaskID, RECV_TIMEOUT_EVT, recv_timeout );
   
      break;
    }
    
    case CID_C2A_GET_BASIC_VALUE:
    {
      getBasicValue();
      break;
    }
      
    case CID_C2A_SET_BASIC_VALUE:
    {
      setBasicValue(pkt);   
      break;
    }
      
    default:
      break;
    }
}

/*********************************************************************
 * @fn      getBasicValue
 *
 * @brief   get basic value and send to Coor
 *
 * @param   none
 *
 * @return  none
 */
void getBasicValue()
{
  //收到查询信息闪烁绿灯3次，每次300ms
  HalLedBlink(HAL_LED_1, 3, 50, 300);
  
  //将基本信息返回给协调器
  unsigned char theMessageData[R2C_RP_BASIC_VALUE_LENGTH];     
  theMessageData[R2C_RP_BASIC_VALUE_NODE_TYPE] = NT_REF_NODE;
  theMessageData[R2C_RP_BASIC_VALUE_REF_ID] = refer_id;
  theMessageData[R2C_RP_BASIC_VALUE_RECV_TIME_OUT] = recv_timeout;
  theMessageData[R2C_RP_BASIC_VALUE_RECV_DELAY_TIME] = delay_time * 2;     //设定的值实际是*2了的
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_A2C_RP_BASIC_VALUE, R2C_RP_BASIC_VALUE_LENGTH,
                           theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ); 
}

/*********************************************************************
 * @fn      setBasicValue
 *
 * @brief   set basic value and send a success response to Coor
 *
 * @param   none
 *
 * @return  none
 */
void setBasicValue(afIncomingMSGPacket_t *pkt)
{
  //将协调器发送过来的配置写入ref节点中
  refer_id = pkt->cmd.Data[C2R_SET_BASIC_VALUE_REF_ID];
  recv_timeout = pkt->cmd.Data[C2R_SET_BASIC_VALUE_RECV_TIME_OUT];
  delay_time = pkt->cmd.Data[C2R_SET_BASIC_VALUE_RECV_DELAY_TIME] / 2;      //配置的值需要除以2才能使用
  
  //将设置成功信息返回给协调器
  successResponse(1);
}    

/*********************************************************************
 * @fn      successResponse
 *
 * @brief   send the success response to the coor
 *
 * @param   none
 *
 * @return  none
 */
void successResponse(uint8 result)
{
  //收到设置信息闪烁绿灯3次，每次300ms
  HalLedBlink(HAL_LED_1, 3, 50, 300);
  
  //将设置成功信息返回给协调器
  unsigned char theMessageData[A2C_SUCCESS_RESPONSE_LENGTH];     
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_TYPE] = NT_REF_NODE;
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_ID] = refer_id;        
  theMessageData[A2C_SUCCESS_RESPONSE_RESULT] = result;         //1表示配置成功
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_A2C_SUCCESS_RESPONSE, A2C_SUCCESS_RESPONSE_LENGTH,
                           theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
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
  //180ms轮询发射的方法
  if (UltraArrivalTime.overflowCapture == 0 & UltraArrivalTime.timerCapture == 0)  //判断超声波是否接收到了，表示超声波未接收到
  {
    i_timeDifference[recvMobiID-1] = 0;
  }
  else
  {
    //电磁波先到的计算方法
    i_timeDifference[recvMobiID-1] = ((UltraArrivalTime.overflowCapture *10240) + UltraArrivalTime.timerCapture) - ((RfArrivalTime.overflowCapture *10240) + RfArrivalTime.timerCapture);
    
    //去除大于10和小于0的错误数据
    if (i_timeDifference[recvMobiID-1] > 941177 | i_timeDifference[recvMobiID-1] <0)   // 941177*31.25ns *340m/s = 10m  设为测试的极限距离 如比这个还大，则为错误
    {
      i_timeDifference[recvMobiID-1] = 0xFFFFFF;
    }  
  }
 
  if(recvMobiID == LAST_MOBILE_ID)
  {
    osal_stop_timerEx(Refer_TaskID,RECV_TIMEOUT_EVT);
    sendData();
  }
   
  UltraArrivalTime.overflowCapture = 0;  //将两个时刻清零
  UltraArrivalTime.timerCapture = 0;
  RfArrivalTime.overflowCapture = 0;  
  RfArrivalTime.timerCapture = 0;
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
  timeDiffMSG[R2C_DIFF_TIME_SEQ] =  ui_sequence;      //序列号  注意此处的帧没有MSG_TYPE
  timeDiffMSG[R2C_DIFF_TIME_REFID] = refer_id;         //参考节点ID
  
  int n;
  for(n=0;n<LAST_MOBILE_ID;n++)
  {
    timeDiffMSG[3*n+2] = ((uint8 *)&i_timeDifference[n])[2];      //时间差高八位 T2计时器
    timeDiffMSG[3*n+3] = ((uint8 *)&i_timeDifference[n])[1];      //时间差中八位
    timeDiffMSG[3*n+4] = ((uint8 *)&i_timeDifference[n])[0];      //时间差低八位
  }
  
  //开启一个事件，1号参考节点直接发送，其他节点延迟5ms*(ID-1)的时间发送消息，避免碰撞
  if (refer_id == 1)
  {
    afAddrType_t coorAddr;
    coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
    coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
    coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
    AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                             CID_R2C_DIFF_TIME, R2C_DIFF_TIME_LENGTH,
                             timeDiffMSG, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
    
    //将发送数组清零
    int i;
    for(i=0;i<R2C_DIFF_TIME_LENGTH;i++)
    {
      timeDiffMSG[i] = 0;     
    }
  }
  else
  {
    unsigned int delaySend;
    delaySend = 5 * (refer_id-1);
    osal_start_timerEx( Refer_TaskID, REFER_DELAYSEND_EVT, delaySend );
  }
  
   //将接收时间差数组清零
  int temp;
  for(temp = 0;temp < LAST_MOBILE_ID;temp++)
  {
    i_timeDifference[temp] = 0;              
  } 
}
