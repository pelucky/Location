/*********************************************************************
  Filename:       LocationDongle.c
  Revised:        $Date: 2007-03-26 11:53:55 -0700 (Mon, 26 Mar 2007) $
  Revision:       $Revision: 13853 $

  Description: This application resides in a dongle and enables a PC GUI (or
    other application) to send and recieve OTA location messages.

          Key control:
            SW1:  N/A
            SW2:  N/A
            SW3:  N/A
            SW4:  N/A

  Copyright (c) 2007 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reproduce, copy, prepare
  derivative works, modify, distribute, perform, display or sell this
  software and/or its documentation for any purpose is prohibited
  without the express written consent of Texas Instruments, Inc.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "MT.h"
#include "MT_APP.h"
#include "AF.h"
#include "ZDApp.h"

#include "OnBoard.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
#include "hal_lcd.h"
#include "hal_led.h"//钟明+
#include "mt_uart.h"//钟明+：For SPI
#endif

#include "LocationProfile.h"
#include "TestApp.h"


#if !defined ( MT_TASK )
  #error      // No need for this module if MT_TASK isn't defined
#endif

/*********************************************************************
 * MACROS
 */
//#define RECV_TIMES              4   //后面把该常量写入LocationProfile中，要与sink节点发送一致
//#define TIMEDIFF_MSG_LENGTH   7  //TIMEDIFF_MSG_LENGTH = 3 + 3 + 1;    时间差的长度 因为数组只能为常量
//#define TEMP_MSG_LENGTH       3   //信息类型 温度值(需要除以100) \n
#define MAX_DATA_LENGTH       16 //最长的数据是4*4 参考节点号(1) + 距离数据（3） 总共有4个节点 
#define TOTAL_DATA_LENGTH     16 //MSG_TYPE + SEQ_NO + MOB_ID + 12 + '\n'
#define REFER_NODES_NUMBERS           4  //表示网络中有参考节点的个数
#define MOBILE_NODES_NUMBERS          4  //表示网络中有移动节点的个数
#define DELAY_SEND_TIMES              10 //表示每组数据延迟发送给上位机的时间间隔
/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 LocationDongle_TaskID;
unsigned int lastSeqNo = 0;
unsigned int thisSeqNo = 0;
unsigned int lastMobID = 0;
unsigned int thisMobID = 0;
unsigned char delayTimes = 1;     //记录延迟发送的次数，一共需要延迟发送3次
uint32 referDataBuf[MOBILE_NODES_NUMBERS+1][REFER_NODES_NUMBERS+1];    //5*5的矩阵，其中第一个用来判断是否已经存在
uint8 buf[MOBILE_NODES_NUMBERS][TOTAL_DATA_LENGTH];         //缓存的发射数组
/*********************************************************************
 * CONSTANTS
 */



static const cId_t LocationDongle_InputClusterList[] =
{
  //Location_MOBILE_FIND_RSP,
  //LOCATION_MOBILE_CONFIG_RSP,
  //LOCATION_REFER_CONFIG_RSP,
  LOCATION_REFER_DISTANCE_RSP,           //pel+
  LOCATION_COOR_TEMPURATE,
  //LOCATION_COOR_ERROR
    
  //Location_REFER_TEMPERATURE_RSP
};

/*
static const cId_t LocationDongle_OutputClusterList[] =
{
  //Location_MOBILE_FIND_REQ,
  //LOCATION_MOBILE_CONFIG_REQ,
  //LOCATION_REFER_CONFIG_REQ
  //Location_REFER_TEMPERATURE_REQ
};
*/

static const SimpleDescriptionFormat_t LocationDongle_SimpleDesc =
{
  LOCATION_DONGLE_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_DONGLE_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,

  sizeof(LocationDongle_InputClusterList),
  (cId_t *)LocationDongle_InputClusterList,
  
  0,
  (cId_t *)NULL
  /*sizeof(LocationDongle_OutputClusterList),
  (cId_t *)LocationDongle_OutputClusterList*/
};

static const endPointDesc_t epDesc =
{
  LOCATION_DONGLE_ENDPOINT,
  &LocationDongle_TaskID,
  (SimpleDescriptionFormat_t *)&LocationDongle_SimpleDesc,
  noLatencyReqs
};


/*********************************************************************
 * LOCAL VARIABLES
 */

uint8 LocationDongle_TransID;  // This is the unique message ID (counter)

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void LocationDongle_Init( uint8 task_id );
UINT16 LocationDongle_ProcessEvent( uint8 task_id, UINT16 events );
void LocationDongle_ProcessMSGCmd( afIncomingMSGPacket_t *pckt );
//void LocationDongle_MTMsg( uint8 len, uint8 *msg );  初期阶段未使用协调器输出功能
void SPIMgr_ProcessZToolData( uint8 port,uint8 event);
//void LocationHandleKeys( uint8 keys );

/*********************************************************************
 * @fn      LocationDongle_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void LocationDongle_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  LocationDongle_TaskID = task_id;
  LocationDongle_TransID = 0;

  //钟明+：
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_19200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;                // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;                // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;              // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;                  // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SPIMgr_ProcessZToolData;
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
  // Register the endpoint/interface description with the AF
  afRegister( (endPointDesc_t *)&epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( LocationDongle_TaskID );

  // Update the display
  #if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "Location-Dongle", HAL_LCD_LINE_2 );
  #endif
}

/*********************************************************************
 * @fn      LocationDongle_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 LocationDongle_ProcessEvent( uint8 task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LocationDongle_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case MT_SYS_APP_MSG:  // Z-Architect Messages
          HalLedBlink(HAL_LED_2,1,50,200);
          //LocationDongle_MTMsg( ((mtSysAppMsg_t *)MSGpkt)->appDataLen, ((mtSysAppMsg_t *)MSGpkt)->appData );
          break;
        case AF_INCOMING_MSG_CMD:
          LocationDongle_ProcessMSGCmd( MSGpkt );
          break;
          
        case ZDO_STATE_CHANGE:
          //显示自己的NETID
          HalLcdWriteStringValue( "NetID:", NLME_GetShortAddr(), 16, HAL_LCD_LINE_3 );
          break;

        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LocationDongle_TaskID );
    }

    // Return unprocessed events
    return ( events ^ SYS_EVENT_MSG );
  }

  // send the data by delaying with 10ms*ID
  if ( events & COOR_DELAYSEND_EVT )    
  {
    //由于c中无法获取一行数组，只能遍历获取，发送其他移动节点1的值
    uint8 sendbuf[TOTAL_DATA_LENGTH];
    int temp;
    for(temp=0;temp<TOTAL_DATA_LENGTH;temp++)
    {
      sendbuf[temp] = buf[delayTimes][temp];
    }
    HalUARTWrite(HAL_UART_PORT_0,sendbuf,TOTAL_DATA_LENGTH);
    
    //完成发射后将清零
    if(delayTimes == 3)
    {
      //将发送数组清零
      int m,n;
      for(m=0;m<MOBILE_NODES_NUMBERS;m++)
      {
        for(n=0;n<TOTAL_DATA_LENGTH;n++)
        {
          buf[m][n] = 0;     
        }
      }
      delayTimes = 1;
    }
    //还未完成节点发射则设定定时器
    else
    {
      delayTimes++;
      osal_start_timerEx( LocationDongle_TaskID, COOR_DELAYSEND_EVT, DELAY_SEND_TIMES );
    }
       
    // return unprocessed events
    return ( events ^ COOR_DELAYSEND_EVT );
  }
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      LocationDongle_ProcessMSGCmd
 *
 * @brief   All incoming messages are sent out the serial port
 *          as an MT SYS_APP_MSG.
 *
 * @param   Raw incoming MSG packet structure pointer.
 *
 * @return  none
 */
void LocationDongle_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )
{
  //uint8 buf[TIMEDIFF_MSG_LENGTH];       //接收常数
  switch(pkt->clusterId)
  {
  //pel+ 添加参考节点发射距离位置给协调器  
  case LOCATION_REFER_DISTANCE_RSP:
    thisSeqNo  = pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_SEQUENCE];
    
    //依据序列号和移动节点ID号是否相同来发射，移动节点或序列号不同则把数据交给串口
    if (thisSeqNo != lastSeqNo)
    {
      //将获取的值放进缓存数组
      int mobileNo,refNo;
      for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
      {
        buf[mobileNo-1][TIMEDIFF_MSG_TYPE] = RP_BLOADCAST_TIME;
        buf[mobileNo-1][TIMEDIFF_SEQUENCE] = lastSeqNo;
        buf[mobileNo-1][TIMEDIFF_MOBID]  = mobileNo;
        for(refNo=1;refNo<=REFER_NODES_NUMBERS;refNo++)
        {
          buf[mobileNo-1][refNo*3] = ((uint8 *)&referDataBuf[mobileNo][refNo])[2];        //时间差高八位 
          buf[mobileNo-1][refNo*3+1] = ((uint8 *)&referDataBuf[mobileNo][refNo])[1];      //时间差中八位
          buf[mobileNo-1][refNo*3+2] = ((uint8 *)&referDataBuf[mobileNo][refNo])[0];      //时间差低八位
        } 
        buf[mobileNo-1][15]  = '\n';
      }
      
      //由于c中无法获取一行数组，只能遍历获取，直接发送移动节点1的值
      uint8 sendbuf[TOTAL_DATA_LENGTH];
      int temp;
      for(temp=0;temp<TOTAL_DATA_LENGTH;temp++)
      {
        sendbuf[temp] = buf[0][temp];
      }
      HalUARTWrite(HAL_UART_PORT_0,sendbuf,TOTAL_DATA_LENGTH); 
      //设置定时器，延迟发送给上位机，防止串口处理堵塞
      osal_start_timerEx( LocationDongle_TaskID, COOR_DELAYSEND_EVT, DELAY_SEND_TIMES );
      
      //清空发射缓存数组
      int m,n;
      for (m=0;m<=MOBILE_NODES_NUMBERS;m++)
      {
        for (n=0;n<=REFER_NODES_NUMBERS;n++)
        {
          referDataBuf[m][n] = 0;
        }
      }
            
      //将这次获取不同序列号的存到发射缓存数组中
      referDataBuf[0][pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_REFID]] = 1;
      //将三个uint8的数据合成一个uint32的并存入数组中,直接<<16位会报警告，要将获取的值声明为32位长，否则会出现不够长的情况  
      for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
      {
        referDataBuf[mobileNo][pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_REFID]] = \
          (unsigned long)(pkt->cmd.Data[3*mobileNo-1])<<16 |      \
          (unsigned long)(pkt->cmd.Data[3*mobileNo])<<8 |      \
          (pkt->cmd.Data[3*mobileNo+1]);
      }
    }
    else
    {
      //因为随着网络的增大，参考节点可能发送多条相同的数据信息给协调器，故要略去相同的数据
      bool b_exist = false;
      if(referDataBuf[0][pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_REFID]] == 1)
      {
        b_exist = true;
      }
      
      if (b_exist == false)
      {
        referDataBuf[0][pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_REFID]] = 1;
        //将三个uint8的数据合成一个uint32的并存入数组中
        int mobileNo;
        for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
        {
          referDataBuf[mobileNo][pkt->cmd.Data[LOCATION_REFER_POSITION_RSP_REFID]] = \
            (unsigned long)(pkt->cmd.Data[3*mobileNo-1])<<16|      \
            (unsigned long)(pkt->cmd.Data[3*mobileNo])<<8 |      \
            (pkt->cmd.Data[3*mobileNo+1]);
        }
      }
    }
    
    lastSeqNo = thisSeqNo;     
    break;  
    
    //pel+ 添加 移动节点发射温度信息给协调器  
  case LOCATION_COOR_TEMPURATE:
    {
      uint8 buf[TEMP_MSG_LENGTH];       //接收常数
      buf[TEMP_MSG_TYPE] = RP_TEMPERATURE_DATA;
      buf[TEMP_DATA_LOW] = pkt->cmd.Data[0];
      buf[TEMP_DATA_HIGH] = pkt->cmd.Data[1];
      buf[TEMP_END] = '\n';
  
      HalUARTWrite(HAL_UART_PORT_0,buf,TEMP_MSG_LENGTH);
      
      break; 
    }
  default:
    break;
  }
}

void SPIMgr_ProcessZToolData ( uint8 port, uint8 event )
{
  uint16 rxlen;//接收数据长度
  uint8* psbuf;//接收存储区指针
     
  if (event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    rxlen = Hal_UART_RxBufLen( HAL_UART_PORT_0 );//获得接收缓冲区数据长度
    psbuf = osal_mem_alloc(rxlen);//分配rxlen长度内存并把指针赋给psbuf
    HalUARTRead (HAL_UART_PORT_0, psbuf, rxlen);//读接收缓冲区数据到内
    //LocationDongle_MTMsg(rxlen,psbuf);  初期阶段直接将输出函数屏蔽了
    osal_mem_free( psbuf );//释放分配的内存
  }
}

/*********************************************************************
*********************************************************************/
