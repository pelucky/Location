/*********************************************************************
  Filename:       TestApp.c
  Revised:        $Date: 2007-03-26 11:53:55 -0700 (Mon, 26 Mar 2007) $
  Revision:       $Revision: 13853 $

  Description: This application resides in a coor and enables a PC GUI (or
    other application) to send and recieve location messages.


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

#define MAX_DATA_LENGTH       16 //最长的数据是4*4 参考节点号(1) + 距离数据（3） 总共有4个节点 
#define TOTAL_DATA_LENGTH     16 //MSG_TYPE + SEQ_NO + MOB_ID + 12 + '\n'
#define REFER_NODES_NUMBERS    4 //表示网络中有参考节点的个数
#define MOBILE_NODES_NUMBERS   4 //表示网络中有移动节点的个数
/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 LocationDongle_TaskID;
static byte transId;

unsigned int lastSeqNo = 0;
unsigned int thisSeqNo = 0;
unsigned char delayTimes = 1;                                           //记录延迟发送的次数，一共需要延迟发送3次
uint32 referDataBuf[MOBILE_NODES_NUMBERS+1][REFER_NODES_NUMBERS+1];     //5*5的矩阵，其中第一列用来判断是否已经存在
uint8 buf[MOBILE_NODES_NUMBERS][TOTAL_DATA_LENGTH];                     //缓存的发射数组

unsigned char delay_send_times = 10;                                    //表示每组数据延迟发送给上位机的时间间隔

unsigned char sinkNetAddr[2] = {0,0};                                               //存储sink节点的网络地址
unsigned char mobileNetAddr[MOBILE_NODES_NUMBERS][2] = {{0,0},{0,0},{0,0},{0,0}};   //存储mobile节点的网络地址
unsigned char referNetAddr[REFER_NODES_NUMBERS][2] = {{0,0},{0,0},{0,0},{0,0}};     //存储refer节点的网络地址

/*********************************************************************
 * CONSTANTS
 */

static const cId_t LocationDongle_InputClusterList[] =
{  
  CID_A2C_RP_BASIC_VALUE,
  CID_A2C_SUCCESS_RESPONSE,
  CID_S2C_TEMPERATURE,
  CID_M2C_REQ_POSITION, 
  CID_R2C_DIFF_TIME  
};


static const cId_t LocationDongle_OutputClusterList[] =
{
  CID_C2A_GET_BASIC_VALUE,
  CID_C2A_SET_BASIC_VALUE,
  CID_C2M_RP_POSITION,
  CID_C2M_SET_JUDGE
};


static const SimpleDescriptionFormat_t LocationDongle_SimpleDesc =
{
  LOCATION_DONGLE_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_DONGLE_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,
  sizeof(LocationDongle_InputClusterList),
  (cId_t *)LocationDongle_InputClusterList,
  sizeof(LocationDongle_OutputClusterList),
  (cId_t *)LocationDongle_OutputClusterList
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
uint16 LocationDongle_ProcessEvent( uint8 task_id, UINT16 events );
void LocationDongle_ProcessMSGCmd( afIncomingMSGPacket_t *pckt );
void LocationDongle_MTMsg( uint8 len, uint8 *msg ); 
void SPIMgr_ProcessZToolData( uint8 port,uint8 event);

void successResponse(uint8 nodeType, uint8 nodeID, uint8 result);
void getBasicValue(uint8 nodeType, uint8 nodeID, uint8 netAddrHi, uint8 netAddrLo, byte endPoint);
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
uint16 LocationDongle_ProcessEvent( uint8 task_id, UINT16 events )
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
      osal_start_timerEx( LocationDongle_TaskID, COOR_DELAYSEND_EVT, delay_send_times );
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
    //其他节点发送查询配置响应给协调器
    case CID_A2C_RP_BASIC_VALUE:
    {
      unsigned char nodeType = pkt->cmd.Data[0];  //查询配置响应帧的第一个字节是节点类型
      switch(nodeType)
      {
        //响应节点为sink节点
        case NT_SINK_NODE:
        {
          uint8 buf[C2PC_RP_BASIC_VALUE_LENGTH_S];       //接收常数
          buf[C2PC_RP_BASIC_VALUE_MSG_TYPE_S] = MT_C2PC_RP_BASIC_VALUE_S;
          buf[C2PC_RP_BASIC_VALUE_BROAD_CYC_S] = pkt->cmd.Data[S2C_RP_BASIC_VALUE_BROAD_CYC];
          buf[C2PC_RP_BASIC_VALUE_TEMP_CYC_S] = pkt->cmd.Data[S2C_RP_BASIC_VALUE_TEMP_CYC];
          buf[C2PC_RP_BASIC_VALUE_END_S] = '\n';
          
          //将获取的sink网络地址存储到数组中
          sinkNetAddr[0] = HI_UINT16(pkt->srcAddr.addr.shortAddr);
          sinkNetAddr[1] = LO_UINT16(pkt->srcAddr.addr.shortAddr);
      
          HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_RP_BASIC_VALUE_LENGTH_S);
          break;
        }
        
        //响应节点为mobile节点
        case NT_MOB_NODE:
        {
          uint8 buf[C2PC_RP_BASIC_VALUE_LENGTH_M];       //接收常数
          buf[C2PC_RP_BASIC_VALUE_MSG_TYPE_M] = MT_C2PC_RP_BASIC_VALUE_M;
          buf[C2PC_RP_BASIC_VALUE_MOB_ID_M] = pkt->cmd.Data[M2C_RP_BASIC_VALUE_MOB_ID];
          buf[C2PC_RP_BASIC_VALUE_TEAM_ID_M] = pkt->cmd.Data[M2C_RP_BASIC_VALUE_TEAM_ID];
          buf[C2PC_RP_BASIC_VALUE_SEND_DELAY_TIME_M] = pkt->cmd.Data[M2C_RP_BASIC_VALUE_SEND_DELAY_TIME];
          buf[C2PC_RP_BASIC_VALUE_END_M] = '\n';
      
          //将获取的mobile网络地址存储到数组中
          mobileNetAddr[(pkt->cmd.Data[M2C_RP_BASIC_VALUE_MOB_ID]-1)][0] = HI_UINT16(pkt->srcAddr.addr.shortAddr);
          mobileNetAddr[(pkt->cmd.Data[M2C_RP_BASIC_VALUE_MOB_ID]-1)][1] = LO_UINT16(pkt->srcAddr.addr.shortAddr);
          
          HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_RP_BASIC_VALUE_LENGTH_M);
          break;
        }
        
        //响应节点为refer节点
        case NT_REF_NODE:
        {
          uint8 buf[C2PC_RP_BASIC_VALUE_LENGTH_R];       //接收常数
          buf[C2PC_RP_BASIC_VALUE_MSG_TYPE_R] = MT_C2PC_RP_BASIC_VALUE_R;
          buf[C2PC_RP_BASIC_VALUE_REF_ID_R] = pkt->cmd.Data[R2C_RP_BASIC_VALUE_REF_ID];
          buf[C2PC_RP_BASIC_VALUE_RECV_TIME_OUT_R] = pkt->cmd.Data[R2C_RP_BASIC_VALUE_RECV_TIME_OUT];
          buf[C2PC_RP_BASIC_VALUE_RECV_DELAY_TIME_R] = pkt->cmd.Data[R2C_RP_BASIC_VALUE_RECV_DELAY_TIME];
          buf[C2PC_RP_BASIC_VALUE_END_R] = '\n';
      
          //将获取的refer网络地址存储到数组中
          referNetAddr[(pkt->cmd.Data[R2C_RP_BASIC_VALUE_REF_ID]-1)][0] = HI_UINT16(pkt->srcAddr.addr.shortAddr);
          referNetAddr[(pkt->cmd.Data[R2C_RP_BASIC_VALUE_REF_ID]-1)][1] = LO_UINT16(pkt->srcAddr.addr.shortAddr);
          
          HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_RP_BASIC_VALUE_LENGTH_R);
          break;
        }
        
        default:
        break;
      } 
      break; 
    }
    
    //其他节点发送设置成功响应给协调器,协调器将数据发送至串口
    case CID_A2C_SUCCESS_RESPONSE:
    {
      successResponse(pkt->cmd.Data[A2C_SUCCESS_RESPONSE_NODE_TYPE],
                      pkt->cmd.Data[A2C_SUCCESS_RESPONSE_NODE_ID],
                      pkt->cmd.Data[A2C_SUCCESS_RESPONSE_RESULT]);
      break; 
    }
    
    //Sink节点发送温度信息给协调器，协调器将数据发送至串口  
    case CID_S2C_TEMPERATURE:
    {
      uint8 buf[C2PC_TEMPERATURE_DATA_LENGTH];       //接收常数
      buf[C2PC_TEMPERATURE_DATA_MSG_TYPE] = MT_C2PC_TEMPERATURE_DATA;
      buf[C2PC_TEMPERATURE_DATA_HIGH] = pkt->cmd.Data[S2C_TEMPERATURE_DATA_HIGH];
      buf[C2PC_TEMPERATURE_DATA_LOW] = pkt->cmd.Data[S2C_TEMPERATURE_DATA_LOW];
      buf[C2PC_TEMPERATURE_DATA_END] = '\n';
  
      HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_TEMPERATURE_DATA_LENGTH);
      break; 
    }
    
    //移动节点发送位置请求信息给协调器，协调器将数据发送至串口
    case CID_M2C_REQ_POSITION:
    {
      uint8 buf[C2PC_REQ_POSITION_LENGTH];       //接收常数
      buf[C2PC_REQ_POSITION_MSG_TYPE] = MT_C2PC_REQ_POSITION;
      buf[C2PC_REQ_POSITION_REQ_MOB_ID] = pkt->cmd.Data[M2C_REQ_POSITION_REQ_MOB_ID];
      buf[C2PC_REQ_POSITION_GET_MOB_ID] = pkt->cmd.Data[M2C_REQ_POSITION_GET_MOB_ID];
      buf[C2PC_REQ_POSITION_END] = '\n';
  
      HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_REQ_POSITION_LENGTH);
      break; 
    }
    
    //参考节点发送时间差信息给协调器，协调器打包后将数据发送至串口  
    case CID_R2C_DIFF_TIME:
    {
      thisSeqNo  = pkt->cmd.Data[R2C_DIFF_TIME_SEQ];
      
      //依据序列号和移动节点ID号是否相同来发射，移动节点或序列号不同则把数据交给串口
      if (thisSeqNo != lastSeqNo)
      {
        //将获取的值放进缓存数组
        int mobileNo,refNo;
        for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
        {
          buf[mobileNo-1][C2PC_DIFF_TIME_MSG_TYPE] = MT_C2PC_DIFF_TIME;
          buf[mobileNo-1][C2PC_DIFF_TIME_SEQ] = lastSeqNo;
          buf[mobileNo-1][C2PC_DIFF_TIME_MOBID]  = mobileNo;
          for(refNo=1;refNo<=REFER_NODES_NUMBERS;refNo++)
          {
            buf[mobileNo-1][refNo*3] = ((uint8 *)&referDataBuf[mobileNo][refNo])[2];        //时间差高八位 
            buf[mobileNo-1][refNo*3+1] = ((uint8 *)&referDataBuf[mobileNo][refNo])[1];      //时间差中八位
            buf[mobileNo-1][refNo*3+2] = ((uint8 *)&referDataBuf[mobileNo][refNo])[0];      //时间差低八位
          } 
          buf[mobileNo-1][C2PC_DIFF_TIME_END]  = '\n';
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
        osal_start_timerEx( LocationDongle_TaskID, COOR_DELAYSEND_EVT, delay_send_times );
        
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
        referDataBuf[0][pkt->cmd.Data[R2C_DIFF_TIME_REFID]] = 1;
        //将三个uint8的数据合成一个uint32的并存入数组中,直接<<16位会报警告，要将获取的值声明为32位长，否则会出现不够长的情况  
        for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
        {
          referDataBuf[mobileNo][pkt->cmd.Data[R2C_DIFF_TIME_REFID]] = \
            (unsigned long)(pkt->cmd.Data[3*mobileNo-1])<<16 |      \
            (unsigned long)(pkt->cmd.Data[3*mobileNo])<<8 |      \
            (pkt->cmd.Data[3*mobileNo+1]);
        }
      }
      else
      {
        //因为随着网络的增大，参考节点可能发送多条相同的数据信息给协调器，故要略去相同的数据
        bool b_exist = false;
        if(referDataBuf[0][pkt->cmd.Data[R2C_DIFF_TIME_REFID]] == 1)
        {
          b_exist = true;
        }
        
        if (b_exist == false)
        {
          referDataBuf[0][pkt->cmd.Data[R2C_DIFF_TIME_REFID]] = 1;
          //将三个uint8的数据合成一个uint32的并存入数组中
          int mobileNo;
          for(mobileNo=1;mobileNo<=MOBILE_NODES_NUMBERS;mobileNo++)
          {
            referDataBuf[mobileNo][pkt->cmd.Data[R2C_DIFF_TIME_REFID]] = \
              (unsigned long)(pkt->cmd.Data[3*mobileNo-1])<<16|      \
              (unsigned long)(pkt->cmd.Data[3*mobileNo])<<8 |      \
              (pkt->cmd.Data[3*mobileNo+1]);
          }
        }
      }
      
      lastSeqNo = thisSeqNo;     
      break;  
    } 
       
    default:
      break;
  }
}


/*********************************************************************
 * @fn      LocationDongle_MTMsg
 *
 * @brief   All outcoming messages are sent from the serial port
 *          
 *
 * @param   Raw incoming MSG packet structure pointer.
 *
 * @return  none
 */
void LocationDongle_MTMsg( uint8 len, uint8 *msg )
{
  unsigned char msgType = msg[0];     //所有从上位机通过串口发给协调器的信息第一个字节都是数据帧类型 
  switch(msgType)
  {
    //上位机向协调器发出查询节点配置信息
    case MT_PC2C_GET_BASIC_VALUE:
    {
      unsigned char nodeType = msg[PC2C_GET_BASIC_VALUE_NODE_TYPE];
      switch(nodeType)
      {
        //表示查询的是coor节点，则直接返回响应信息
        case NT_COOR_NODE:
        {
          //发送配置参数给上位机
          uint8 buf[C2PC_RP_BASIC_VALUE_LENGTH_C];       //接收常数
          buf[C2PC_RP_BASIC_VALUE_MSG_TYPE_C] = MT_C2PC_RP_BASIC_VALUE_C;
          buf[C2PC_RP_BASIC_VALUE_DELAY_TIME_C] = delay_send_times;        
          buf[C2PC_RP_BASIC_VALUE_END_C] = '\n';
          
          //收到配置信息闪烁绿灯3次，每次300ms
          HalLedBlink(HAL_LED_1, 3, 50, 300);
      
          HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_RP_BASIC_VALUE_LENGTH_C);
          break;
        }
         
        //表示查询的是sink节点，则向sink节点发送查询信息
        case NT_SINK_NODE:
        {
          //sink节点只有1个故编号为1
          getBasicValue(NT_SINK_NODE,1,sinkNetAddr[0],sinkNetAddr[1],LOCATION_SINK_ENDPOINT);
          break;
        }
        
        //表示查询的是mobile节点，则向mobile节点发送查询信息
        case NT_MOB_NODE:
        {
          getBasicValue(NT_MOB_NODE,msg[PC2C_GET_BASIC_VALUE_NODE_ID],
                        mobileNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][0],
                        mobileNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][1],
                        LOCATION_MOBILE_ENDPOINT);
          break;
        }
        
        //表示查询的是refer节点，则向refer节点发送查询信息
        case NT_REF_NODE:
        {
          getBasicValue(NT_REF_NODE,msg[PC2C_GET_BASIC_VALUE_NODE_ID],
                        referNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][0],
                        referNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][1],
                        LOCATION_REFER_ENDPOINT);
          break;
        }
        
        default:
        break;
      }
      break;
    }
    
    /*所有设置节点配置信息的第二个字节都是帧类型，其中包括了节点类型*/
    //上位机向协调器发出设置协调器节点配置信息  
    case MT_PC2C_SET_BASIC_VALUE_C:
    {  
      //将设置信息写入coor节点
      delay_send_times = msg[PC2C_SET_BASIC_VALUE_DELAY_TIME_C];
      
      //发送配置成功响应给上位机
      uint8 buf[C2PC_SUCCESS_RESPONSE_LENGTH];       //接收常数
      buf[C2PC_SUCCESS_RESPONSE_MSG_TYPE] = MT_C2PC_SUCCESS_RESPONSE;
      buf[C2PC_SUCCESS_RESPONSE_NODE_TYPE] = NT_COOR_NODE;
      buf[C2PC_SUCCESS_RESPONSE_NODE_ID] = 1;         //协调器只有一个，故写1
      buf[C2PC_SUCCESS_RESPONSE_RESULT] = 1;          //结果：1，表示成功，0表示失败
      buf[C2PC_SUCCESS_RESPONSE_END] = '\n';
  
      //收到配置信息闪烁绿灯3次，每次300ms
      HalLedBlink(HAL_LED_1, 3, 50, 300);
      
      HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_SUCCESS_RESPONSE_LENGTH);
      break;
    }
    
    //上位机向协调器发出设置sink节点配置信息  
    case MT_PC2C_SET_BASIC_VALUE_S:
    {
      //将设置节点配置信息发给的sink节点
      unsigned char theMessageData[C2S_SET_BASIC_VALUE_LENGTH];     
      theMessageData[C2S_SET_BASIC_VALUE_BROAD_CYC] = msg[PC2C_SET_BASIC_VALUE_BROAD_CYC_S];
      theMessageData[C2S_SET_BASIC_VALUE_TEMP_CYC] = msg[PC2C_SET_BASIC_VALUE_TEMP_CYC_S];
      
      afAddrType_t sinkAddr;
      sinkAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
      sinkAddr.endPoint = LOCATION_SINK_ENDPOINT; //目的端口号
      sinkAddr.addr.shortAddr = BUILD_UINT16(sinkNetAddr[1],sinkNetAddr[0]);
      AF_DataRequest( &sinkAddr, (endPointDesc_t *)&epDesc,
                               CID_C2A_SET_BASIC_VALUE, C2S_SET_BASIC_VALUE_LENGTH,
                               theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );    
      break;
    }
    
    //上位机向协调器发出设置mobile节点配置信息  
    case MT_PC2C_SET_BASIC_VALUE_M:
    {
      //将设置节点配置信息发给的mobile节点
      unsigned char theMessageData[C2M_SET_BASIC_VALUE_LENGTH];     
      theMessageData[C2M_SET_BASIC_VALUE_MOB_ID] = msg[PC2C_SET_BASIC_VALUE_MOB_ID_M];
      theMessageData[C2M_SET_BASIC_VALUE_TEAM_ID] = msg[PC2C_SET_BASIC_VALUE_TEAM_ID_M];
      theMessageData[C2M_SET_BASIC_VALUE_SEND_DELAY_TIME] = msg[PC2C_SET_BASIC_VALUE_SEND_DELAY_TIME_M];
      
      afAddrType_t mobileAddr;
      mobileAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
      mobileAddr.endPoint = LOCATION_MOBILE_ENDPOINT; //目的端口号
      mobileAddr.addr.shortAddr = BUILD_UINT16(mobileNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][1],
                                               mobileNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][0]);
      AF_DataRequest( &mobileAddr, (endPointDesc_t *)&epDesc,
                               CID_C2A_SET_BASIC_VALUE, C2M_SET_BASIC_VALUE_LENGTH,
                               theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
      break;
    }
        
    //上位机向协调器发出设置refer节点配置信息  
    case MT_PC2C_SET_BASIC_VALUE_R:
    {
      //将设置节点配置信息发给的refer节点
      unsigned char theMessageData[C2R_SET_BASIC_VALUE_LENGTH];     
      theMessageData[C2R_SET_BASIC_VALUE_REF_ID] = msg[PC2C_SET_BASIC_VALUE_REF_ID_R];
      theMessageData[C2R_SET_BASIC_VALUE_RECV_TIME_OUT] = msg[PC2C_SET_BASIC_VALUE_RECV_TIME_OUT_R];
      theMessageData[C2R_SET_BASIC_VALUE_RECV_DELAY_TIME] = msg[PC2C_SET_BASIC_VALUE_RECV_DELAY_TIME_R];
      
      afAddrType_t referAddr;
      referAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
      referAddr.endPoint = LOCATION_REFER_ENDPOINT; //目的端口号
      referAddr.addr.shortAddr = BUILD_UINT16(referNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][1],
                                              referNetAddr[(msg[PC2C_GET_BASIC_VALUE_NODE_ID]-1)][0]);
      AF_DataRequest( &referAddr, (endPointDesc_t *)&epDesc,
                               CID_C2A_SET_BASIC_VALUE, C2R_SET_BASIC_VALUE_LENGTH,
                               theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );

      break;
    }
    
    //上位机向协调器发出响应移动节点位置请求信息
    case MT_PC2C_RP_POSITION:
    {
      //将响应信息发给请求的移动节点
      unsigned char theMessageData[C2M_RP_POSITION_LENGTH];     
      theMessageData[C2M_RP_POSITION_GET_MOB_ID] = msg[PC2C_RP_POSITION_GET_MOB_ID];
      theMessageData[C2M_RP_POSITION_X_HI] = msg[PC2C_RP_POSITION_X_HI];
      theMessageData[C2M_RP_POSITION_X_LO] = msg[PC2C_RP_POSITION_X_LO];
      theMessageData[C2M_RP_POSITION_Y_HI] = msg[PC2C_RP_POSITION_Y_HI];
      theMessageData[C2M_RP_POSITION_Y_LO] = msg[PC2C_RP_POSITION_Y_LO];
      
      afAddrType_t mobileAddr;
      mobileAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
      mobileAddr.endPoint = LOCATION_MOBILE_ENDPOINT; //目的端口号
      mobileAddr.addr.shortAddr = BUILD_UINT16(mobileNetAddr[(msg[PC2C_RP_POSITION_REQ_MOB_ID]-1)][1],
                                               mobileNetAddr[(msg[PC2C_RP_POSITION_REQ_MOB_ID]-1)][0]);
      AF_DataRequest( &mobileAddr, (endPointDesc_t *)&epDesc,
                               CID_C2M_RP_POSITION, C2M_RP_POSITION_LENGTH,
                               theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );     
      break;
    }
    
    //上位机向协调器发出起始信息
    case MT_PC2C_SET_JUDGE:
    {
      //将响应信息发给请求的移动节点
      unsigned char theMessageData[C2M_SET_JUDGE_LENGTH];     
      theMessageData[C2M_SET_JUDGE_ACTION] = msg[PC2C_SET_JUDGE_ACTION];
      
      afAddrType_t mobileAddr;
      mobileAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
      mobileAddr.endPoint = LOCATION_MOBILE_ENDPOINT; //目的端口号
      mobileAddr.addr.shortAddr = BUILD_UINT16(mobileNetAddr[(msg[PC2C_SET_JUDGE_MOB_ID]-1)][1],
                                               mobileNetAddr[(msg[PC2C_SET_JUDGE_MOB_ID]-1)][0]);
      AF_DataRequest( &mobileAddr, (endPointDesc_t *)&epDesc,
                               CID_C2M_SET_JUDGE, C2M_SET_JUDGE_LENGTH,
                               theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
      break;
    }  
    
    default:
    break;
  }
}

/*********************************************************************
 * @fn      SPIMgr_ProcessZToolData
 *
 * @brief   receive and send by serial
 *
 * @param   none
 *
 * @return  none
 */
void SPIMgr_ProcessZToolData ( uint8 port, uint8 event )
{
  uint16 rxlen;//接收数据长度
  uint8* psbuf;//接收存储区指针
     
  if (event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    rxlen = Hal_UART_RxBufLen( HAL_UART_PORT_0 );     //获得接收缓冲区数据长度
    psbuf = osal_mem_alloc(rxlen);                    //分配rxlen长度内存并把指针赋给psbuf
    HalUARTRead (HAL_UART_PORT_0, psbuf, rxlen);      //读接收缓冲区数据到内
    LocationDongle_MTMsg(rxlen,psbuf);                //对接收到的数据进行处理
    osal_mem_free( psbuf );                           //释放分配的内存
  }
}


/*********************************************************************
 * @fn      successResponse
 *
 * @brief   send seccess response to the PC
 *
 * @param   none
 *
 * @return  none
 */
void successResponse(uint8 nodeType, uint8 nodeID, uint8 result)
{
  uint8 buf[C2PC_SUCCESS_RESPONSE_LENGTH];       //接收常数
  buf[C2PC_SUCCESS_RESPONSE_MSG_TYPE] = MT_C2PC_SUCCESS_RESPONSE;
  buf[C2PC_SUCCESS_RESPONSE_NODE_TYPE] = nodeType;
  buf[C2PC_SUCCESS_RESPONSE_NODE_ID] = nodeID;
  buf[C2PC_SUCCESS_RESPONSE_RESULT] = result;
  buf[C2PC_SUCCESS_RESPONSE_END] = '\n';

  HalUARTWrite(HAL_UART_PORT_0,buf,C2PC_SUCCESS_RESPONSE_LENGTH);
}


/*********************************************************************
 * @fn      getBasicValue
 *
 * @brief   PC send to C for getting the basic value of any other type of nodes
 *
 * @param   none
 *
 * @return  none
 */
void getBasicValue(uint8 nodeType, uint8 nodeID, uint8 netAddrHi, uint8 netAddrLo, byte endPoint)
{
  //判断节点是否已经开启了，开启了才能进行查询
  if(netAddrHi != 0 || netAddrLo != 0)
  {
    //将查询节点配置信息发给的sink节点            
    afAddrType_t Addr;
    Addr.addrMode = (afAddrMode_t)Addr16Bit;  //单播发送
    Addr.endPoint = endPoint;                 //目的端口号
    Addr.addr.shortAddr = BUILD_UINT16(netAddrLo,netAddrHi);    //长度和内容为0表示空消息
    AF_DataRequest( &Addr, (endPointDesc_t *)&epDesc,
                    CID_C2A_GET_BASIC_VALUE, 0,                          
                    0, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  //否则表示未开启，提示查询错误
  else
  {
    successResponse(nodeType,nodeID,0);  //0表示失败                    
  }
}

/*********************************************************************
*********************************************************************/
