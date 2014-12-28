/*********************************************************************
  Filename:       RefNode.c
  Revised:        $Date: 2007-02-15 15:11:28 -0800 (Thu, 15 Feb 2007) $
  Revision:       $Revision: 13481 $

  Description: Sinkence Node for the Z-Stack Location Profile.
				
  Copyright (c) 2006 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reproduce, copy, prepare
  derivative works, modify, distribute, perform, display or sell this
  software and/or its documentation for any purpose is prohibited
  without the express written consent of Texas Instruments, Inc.
  修改定位算法，sink节点，用作周期性的广播，现暂定为3s 一个周期内发射10次广播信号
  修改了-DMAX_BCAST = 22 默认为9
  2014_12_20 修改算法，为了适应多个移动节点，需要设置轮询广播
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

/*********************************************************************
 * CONSTANTS
 */
#define SINK_BROADCAST_PERIOD 120 //广播周期一次为120ms发射一次,单位为ms

#define SINK_BROADCAST_TIMES  4     //每次大周期内的发射次数

#define SEQUENCE                200  //序列号，用于判断是否收到的是同一次数据

#define MOBILEN_NO            4       //移动节点的个数


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte Sink_TaskID;
unsigned char uc_sequence = 100;        //改为计数从100-200，和结尾符0x0a分开
unsigned char uc_mobileID = 1;          //第一次广播从1号节点开始
unsigned char broadcastTimes = 0;       //周期广播次数
uint8 transId;
bool GreLedState = 0;
static byte transId;


/*********************************************************************
 * FUNCATION
*/
void initP1();
void startBroadcast();
/*********************************************************************/
 // LOCAL VARIABLES
 
/*
static const cId_t Sink_InputClusterList[] =
{
  LOCATION_ULTRA_BLORDCAST
};
*/
static const cId_t Sink_OutputClusterList[] =
{
  LOCATION_ULTRA_BLORDCAST,
};

static const SimpleDescriptionFormat_t Sink_SimpleDesc =
{
  LOCATION_SINK_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_SINK_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,

  /*sizeof(Sink_InputClusterList),
  (cId_t*)Sink_InputClusterList,*/
  0,
  (cId_t *) NULL,
  sizeof(Sink_OutputClusterList),
  (cId_t*)Sink_OutputClusterList
};

static const endPointDesc_t epDesc =
{
  LOCATION_SINK_ENDPOINT,
  &Sink_TaskID,
  (SimpleDescriptionFormat_t *)&Sink_SimpleDesc,
  noLatencyReqs
};


/*********************************************************************
 * 函数名称：initP1
 * 功    能：初始化
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void initP1()
{
   P1SEL &= ~0x02;              //将P1_1置为GPIO
   P1DIR |= 0x02;               //将P1_1置为输出
   P1_1 = 0;                  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Sink_Init
 *
 * @brief   Initialization function for this OSAL task.
 *
 * @param   task_id - the ID assigned by OSAL.
 *
 * @return  none
 */
void Sink_Init( byte task_id )
{
  Sink_TaskID = task_id;

  // Register the endpoint/interface description with the AF.
  afRegister( (endPointDesc_t *)&epDesc );

  // Register for all key events - This app will handle all key events.
  RegisterForKeys( Sink_TaskID );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Location-RefNode", HAL_LCD_LINE_2 );
#endif
  //设备初始化
  initP1();
  
  HAL_TURN_OFF_LED1();         // 熄灭LED_G、LED_R、LED_Y 不使用绿灯和红灯，因为板子使用了P1_0口和P1_1
  HAL_TURN_OFF_LED2();       
  HAL_TURN_OFF_LED3();
}

/*********************************************************************
 * @fn      SinkNode_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - Bit map of events to process.
 *
 * @return  none
 */
uint16 Sink_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Sink_TaskID );
    while( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
      case KEY_CHANGE:
        break;

      case AF_INCOMING_MSG_CMD:
        break;

      case ZDO_STATE_CHANGE:
        osal_start_timerEx( Sink_TaskID, SINK_BROADCAST_EVT, SINK_BROADCAST_PERIOD );  //设置广播定时                                      
        break;

      default:
        break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Sink_TaskID );
    }
    
    // Return unprocessed events.
    return ( events ^ SYS_EVENT_MSG );  
  }
  
  // Send a message out - This event is generated by a timer
  if ( events & SINK_BROADCAST_EVT )    
  {
    startBroadcast();
    // Setup to send period message again
    osal_start_timerEx( Sink_TaskID, SINK_BROADCAST_EVT, SINK_BROADCAST_PERIOD );
      
    // return unprocessed events
    return ( events ^ SINK_BROADCAST_EVT );
  }
  
  // Discard unknown events.
  return 0;  
}

/*********************************************************************
 * @fn      startBlast
 *
 * @brief   Start a sequence of blasts
 *
 * @param   none
 *
 * @return  none
 */
void startBroadcast( void )
{
  //周期性的广播
  if (GreLedState == 0)
  {
    HAL_TURN_ON_LED1();     //改变一次LED_G的状态，表示正在发射超声波
    GreLedState = 1;
  }
  else 
  {
    HAL_TURN_OFF_LED1();
    GreLedState = 0;
  }  
  
  if (uc_mobileID > MOBILEN_NO)
  {
    uc_mobileID = 1;
    if (uc_sequence < SEQUENCE)     //设置发射序列号
    {
      uc_sequence++;
    }
    else
    {
      uc_sequence = 100;            
    }
  }
  
  //发射电磁波
  unsigned char theMessageData[2];     //发射内容为序列号 移动节点号
  theMessageData[0] = uc_sequence;
  theMessageData[1] = uc_mobileID;
  afAddrType_t broadcast_DstAddr;
  broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  broadcast_DstAddr.endPoint = LOCATION_REFER_ENDPOINT;           //暂时未解决如何广播两种端口的问题，只能将不同类型的节点的ENDPOINT设置为一样的
  broadcast_DstAddr.addr.shortAddr = 0xFFFF;  //表示数据包发往网络中的所有节点广播，设置0xFFFC，路由器会收不到，不知何解 
  if( AF_DataRequest( &broadcast_DstAddr, (endPointDesc_t *)&epDesc,
                 LOCATION_ULTRA_BLORDCAST, 2,
                 theMessageData, &transId,
                 0, 1        //AF_DISCV_ROUTE AF_DEFAULT_RADIUS    //设置为一跳,直接进行广播
                 ) == afStatus_SUCCESS)
  {
    P1_1 = ~P1_1;
  }  
  
  uc_mobileID++;    //将移动节点的ID号自加，发送下一个移动节点的同步信号
}

