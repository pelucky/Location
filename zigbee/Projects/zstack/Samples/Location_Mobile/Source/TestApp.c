/*********************************************************************
  Filename:       Mobile.c
  Revised:        $Date: 2007-05-31 13:07:52 -0700 (Thu, 31 May 2007) $
  Revision:       $Revision: 14480 $

  Description: Blind Node for the Z-Stack Location Profile.
				
  Copyright (c) 2006 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reproduce, copy, prepare
  derivative works, modify, distribute, perform, display or sell this
  software and/or its documentation for any purpose is prohibited
  without the express written consent of Texas Instruments, Inc.
  修改成sink节点发送广播信息，移动节点接收到广播信息后就开始发送超声波信号，参考节点接收超声波信号
  要注意没有DS18B20的移动节点要关闭定时传输温度的功能，否则会卡住。
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "OSAL.h"
#include "OSAL_NV.h"
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
#include "UltraSend.h"
#include "DS18B20.h"
#include "math.h"         //pel+ pow(,)

#include "mac_mcu.h"      //调试测试发射超声波和电磁波的时间差

/*********************************************************************
 * CONSTANTS
 */

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

uint8 Mobile_TaskID;
static uint8 transId;
static unsigned char t3count = 0;
unsigned char mobile_temp_cycle  = 10;                     //单位为秒，在实际使用时，需要*1000，因为函数单位ms；
unsigned char mobile_id = 4;                               //移动节点的ID号，每个移动节点的ID都不能相同
unsigned char team_id = 2;                                 //移动节点队伍的ID号，要记得修改！默认时1和2号是队伍1,3和4是队伍2
unsigned char delay_time = 18;                             //延迟时间，实际延迟是2*delay_time ms，此处延迟是与参考节点延迟相同的时间
uint8 teamMobileData[4][2] = {{0,0},{0,0},{0,0},{0,0}};    //默认是一个队友，,最多有3个，第0行就为1号移动节点，第1行就为2号，以此类推。第一字节为网络地址高八位，第二字节位网络地址低八位
/*********************************************************************
 * LOCAL VARIABLES
 */

static const cId_t Mobile_InputClusterList[] =
{
  CID_S2MR_BROADCAST,       //接收sink节点的广播信号
  CID_C2A_GET_BASIC_VALUE,
  CID_C2A_SET_BASIC_VALUE,
  CID_C2M_RP_POSITION,
  CID_C2M_SET_JUDGE,
  CID_M2M_TEAM_DATA,
  CID_M2M_TEAM_CONTROL
};


static const cId_t Mobile_OutputClusterList[] =
{
  CID_M2M_TEAM_DATA,
  CID_M2M_TEAM_CONTROL,
  CID_M2C_TEMPERATURE,        //发送温度给协调器
  CID_M2C_REQ_POSITION,
  CID_A2C_RP_BASIC_VALUE,
  CID_A2C_SUCCESS_RESPONSE
};

static const SimpleDescriptionFormat_t Mobile_SimpleDesc =
{
  LOCATION_MOBILE_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_MOBILE_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,
  
  sizeof(Mobile_InputClusterList),
  (cId_t *)Mobile_InputClusterList,
  sizeof(Mobile_OutputClusterList),
  (cId_t *)Mobile_OutputClusterList
};

static const endPointDesc_t epDesc =
{
  LOCATION_MOBILE_ENDPOINT,
  &Mobile_TaskID,
  (SimpleDescriptionFormat_t *)&Mobile_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processMSGCmd( afIncomingMSGPacket_t *pckt );
void startSendUS();
void halSetTimer3Period(uint8 period);
void getBasicValue();
void setBasicValue(afIncomingMSGPacket_t *pkt);
void responsePosition(afIncomingMSGPacket_t *pkt);
void setJudge();
void successResponse(uint8 result);
void sendTeamData(afAddrMode_t AddrType, uint8 netAddr_HI, uint8 netAddr_LO);
/*********************************************************************
 * @fn      Mobile_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *
 * @param   task_id - the ID assigned by OSAL.
 *
 * @return  none
 */
void Mobile_Init( uint8 task_id )
{
  Mobile_TaskID = task_id;

  //state = eBnIdle;

  afRegister( (endPointDesc_t *)&epDesc );
  RegisterForKeys( Mobile_TaskID );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "Location-Blind", HAL_LCD_LINE_1 );
#endif

  //超声波设备初始化
  initP1();
  
  TIMER34_INIT(3);
  halSetTimer3Period(250); //表示延迟2ms 注意：实际写入T1CC0寄存器的值应小于255
  IEN1 |= (0x01 << 3);             // 使能Timer3的中断
  EA = 1;                      //开总使能
  
  HAL_TURN_OFF_LED1();         // 熄灭LED_G、LED_R、LED_Y 不使用绿灯，因为板子使用了P1_0口
  HAL_TURN_OFF_LED2();       
  HAL_TURN_OFF_LED3();
  
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
    if(t3count == delay_time*(mobile_id-1))     //2*12=24ms
    {
      t3count = 0;
      startSendUS();             //调用发送超声波事件
      
      //将定时器T1清0，为下次中断做准备
      TIMER3_RUN(FALSE);
      T3CTL |= 0x04;       //将计数值清0
    }    
  }
  
  TIMIF &= ~0x01;  
  //T1STAT &= ~0x21;  //清除定时器3的标志位
  //IRCON &= ~0x02;   //同上
  IEN1 |= 0x03;    //打开定时器3中断使能
}


/*********************************************************************
 * @fn      Mobile_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.
 *
 * @return  none
 */
uint16 Mobile_ProcessEvent( uint8 task_id, uint16 events )
{
   afIncomingMSGPacket_t *MSGpkt;
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Mobile_TaskID );

    while( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
      case AF_INCOMING_MSG_CMD:
        processMSGCmd( MSGpkt );
        break;

      case ZDO_STATE_CHANGE:
      {
        //初始化时将基本数据发送给协调器          
        getBasicValue();
        //初始化广播该节点的基本队伍信息给其他节点
        sendTeamData((afAddrMode_t)AddrBroadcast,0xFF,0xFF);
        
        //进入网络获取温度，并将其传到协调器
        //sendTemperature();
        //osal_start_timerEx( Mobile_TaskID, MOBILE_TEMP_EVT, mobile_temp_cycle*1000 );  //设置周期发送温度信息定时    
        break;
      }

      default:
        break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Mobile_TaskID );
      
    }

    return ( events ^ SYS_EVENT_MSG );  // Return unprocessed events.
  }

  if ( events & MOBILE_TEMP_EVT )    
  {
    sendTemperature();             //调用发送温度
    osal_start_timerEx( Mobile_TaskID, MOBILE_TEMP_EVT, mobile_temp_cycle*1000 );
    
    return ( events ^ MOBILE_TEMP_EVT );
  }
  
  return 0;  // Discard unknown events.
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
    //接收sink节点的广播信号
    case CID_S2MR_BROADCAST:
    {
      //1号移动节点直接发射超声波，不需要等待
      if(mobile_id == 1)
      {
        startSendUS();
      }
      //其他节点开启定时器，等待(n-1)*24ms的时间，再发射超声波
      else
      {
        TIMER3_RUN(TRUE);
      }
    }
    break;

    //接收coor节点的查询配置信号    
    case CID_C2A_GET_BASIC_VALUE:
    {
      getBasicValue();
    }
    break;
    
    //接收coor节点的设置配置信号
    case CID_C2A_SET_BASIC_VALUE:
    {
      setBasicValue(pkt);
    }
    break;
    
    //接收coor节点的响应节点位置信号
    case CID_C2M_RP_POSITION:
    {
      responsePosition(pkt);
    }
    break;
    
    //接收coor节点的起始信号
    case CID_C2M_SET_JUDGE:
    {
      setJudge();
    }
    break;
    
    //接收到其他mobile节点的队伍信息
    case CID_M2M_TEAM_DATA:
    {
      //判断如果Team_ID相同则，将其网络地址记录下来，
      if(pkt->cmd.Data[M2M_TEAM_DATA_TEAM_ID] == team_id)
      {
        teamMobileData[(pkt->cmd.Data[M2M_TEAM_DATA_MOB_ID]-1)][0] = HI_UINT16(pkt->srcAddr.addr.shortAddr);
        teamMobileData[(pkt->cmd.Data[M2M_TEAM_DATA_MOB_ID]-1)][1] = LO_UINT16(pkt->srcAddr.addr.shortAddr);
        
        //如果判断接收到的是广播形式，则回送单播信号给发送的节点
        if(pkt->wasBroadcast == true)
        {
          sendTeamData((afAddrMode_t)Addr16Bit, teamMobileData[(pkt->cmd.Data[M2M_TEAM_DATA_MOB_ID]-1)][0],
                                                teamMobileData[(pkt->cmd.Data[M2M_TEAM_DATA_MOB_ID]-1)][1]);
        }
      }
    }
    break;
    
    //接收到本队mobile节点的控制信息
    case CID_M2M_TEAM_CONTROL:
    {  
      //将接收到的数据通过串口发送给robot
      //add here!
      
    }
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      startBlast
 *
 * @brief   Start a sequence of blasts and calculate position.
 *
 * @param   none
 *
 * @return  none
 */
void startSendUS()
{
 
  //先发射超声波，后发射电磁波，根据示波器，超声波之后大概2ms发射了电磁波
  if (RedLedState == 0)
  {
    HAL_TURN_ON_LED2();     //改变一次LED_Y的状态，表示正在发射超声波
    RedLedState = 1;
  }
  else 
  {
    HAL_TURN_OFF_LED2();
    RedLedState = 0;
  }
  
  EA = 0;                                       //关闭中断，避免中断引起烧管
  SendUltra(SquareWaveTimes);                   //发射超声波
  EA = 1;                                        //打开中断  
      
  //P1_1 = 1;  
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
  //某些板硬件可能有问题，故将移动节点的闪灯取消，或考虑之后换个颜色的灯
  //收到查询信息闪烁绿灯3次，每次300ms
  //HalLedBlink(HAL_LED_1, 3, 50, 300);
  
  //将基本信息返回给协调器
  unsigned char theMessageData[M2C_RP_BASIC_VALUE_LENGTH];     
  theMessageData[M2C_RP_BASIC_VALUE_NODE_TYPE] = NT_MOB_NODE;
  theMessageData[M2C_RP_BASIC_VALUE_MOB_ID] = mobile_id;
  theMessageData[M2C_RP_BASIC_VALUE_TEAM_ID] = team_id;
  theMessageData[M2C_RP_BASIC_VALUE_TEMP_CYC] = mobile_temp_cycle;
  theMessageData[M2C_RP_BASIC_VALUE_SEND_DELAY_TIME] = delay_time * 2;        //同refer节点，需要乘以2
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_A2C_RP_BASIC_VALUE, M2C_RP_BASIC_VALUE_LENGTH,
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
  //将协调器发送过来的配置写入mobile节点中
  mobile_id = pkt->cmd.Data[C2M_SET_BASIC_VALUE_MOB_ID];
  team_id = pkt->cmd.Data[C2M_SET_BASIC_VALUE_TEAM_ID];
  mobile_temp_cycle = pkt->cmd.Data[C2M_SET_BASIC_VALUE_TEMP_CYC];
  delay_time = pkt->cmd.Data[C2M_SET_BASIC_VALUE_SEND_DELAY_TIME] / 2;    //同refer节点，需要除以2

  //将接受成功的消息发回给协调器
  successResponse(1);
}

/*********************************************************************
 * @fn      responsePosition
 *
 * @brief   get the request Position from Coor and send to serial to robot
 *
 * @param   none
 *
 * @return  none
 */
void responsePosition(afIncomingMSGPacket_t *pkt)
{
  //通过串口发送给智能小车请求的位置信息
  //add here!
  
  //将接受成功的消息发回给协调器
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
  //某些板硬件可能有问题，故将移动节点的闪灯取消，或考虑之后换个颜色的灯
  //收到设置信息闪烁绿灯3次，每次300ms
  HalLedBlink(HAL_LED_1, 3, 50, 300);
  
  //将设置成功信息返回给协调器
  unsigned char theMessageData[A2C_SUCCESS_RESPONSE_LENGTH];     
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_TYPE] = NT_MOB_NODE;
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_ID] = mobile_id;
  theMessageData[A2C_SUCCESS_RESPONSE_RESULT] = result;        //1表示配置成功
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_A2C_SUCCESS_RESPONSE, A2C_SUCCESS_RESPONSE_LENGTH,
                           theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/*********************************************************************
 * @fn      setJudge
 *
 * @brief   Set the judge Command from Coor and send back success response
 *
 * @param   none
 *
 * @return  none
 */
void setJudge()
{
  //通过串口发送给智能小车
  //add here!
  
  
  //将设置成功信息返回给协调器
  successResponse(1);
}

/*********************************************************************
 * @fn      requestPosition
 *
 * @brief   get the request Position from serial and send to Coor
 *
 * @param   none
 *
 * @return  none
 */
void requestPosition()
{
  //从串口获取需要获悉位置的移动节点ID
  //add here!
  
  //将请求位置信息发送给协调器
  unsigned char theMessageData[M2C_REQ_POSITION_LENGTH];     
  theMessageData[M2C_REQ_POSITION_REQ_MOB_ID] = mobile_id;    //请求的移动节点ID
  theMessageData[M2C_REQ_POSITION_GET_MOB_ID] = mobile_id;    //此参数是需要获悉位置的移动节点ID，可以是其他移动节点
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_M2C_REQ_POSITION, M2C_REQ_POSITION_LENGTH,
                           theMessageData, &transId, AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}


/*********************************************************************
 * @fn      sendTeamData
 *
 * @brief   send the team information to the other mobile node
 *
 * @param   none
 *
 * @return  none
 */
void sendTeamData(afAddrMode_t AddrType, uint8 netAddr_HI, uint8 netAddr_LO)
{
  //发射电磁波
  unsigned char theMessageData[M2M_TEAM_DATA_LENGTH];    
  theMessageData[M2M_TEAM_DATA_TEAM_ID] = team_id;
  theMessageData[M2M_TEAM_DATA_MOB_ID] = mobile_id;
  
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)AddrType;
  dstAddr.endPoint = LOCATION_MOBILE_ENDPOINT;        
  dstAddr.addr.shortAddr = BUILD_UINT16(netAddr_LO,netAddr_HI);  
  AF_DataRequest( &dstAddr, (endPointDesc_t *)&epDesc,
                 CID_M2M_TEAM_DATA, M2M_TEAM_DATA_LENGTH,
                 theMessageData, &transId,
                 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS);  
}


/*********************************************************************
 * @fn      sendTeamControl
 *
 * @brief   send the team control information to the other mobile node
 *
 * @param   none
 *
 * @return  none
 */
void sendTeamControl(uint8 get_mob_id, uint8 action_HI, uint8 action_LO)
{
  //发射电磁波
  unsigned char theMessageData[M2M_TEAM_CONTROL_LENGTH];    
  theMessageData[M2M_TEAM_CONTROL_REQ_MOB_ID] = mobile_id;
  theMessageData[M2M_TEAM_CONTROL_GET_MOB_ID] = get_mob_id;
  theMessageData[M2M_TEAM_CONGROL_ACTION_HI] = action_HI;
  theMessageData[M2M_TEAM_CONGROL_ACTION_LO] = action_LO;
  
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  dstAddr.endPoint = LOCATION_MOBILE_ENDPOINT;        
  dstAddr.addr.shortAddr = BUILD_UINT16(teamMobileData[get_mob_id-1][0],
                                        teamMobileData[get_mob_id-1][0]);  
  AF_DataRequest( &dstAddr, (endPointDesc_t *)&epDesc,
                 CID_M2M_TEAM_CONTROL, M2M_TEAM_CONTROL_LENGTH,
                 theMessageData, &transId,
                 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS);  
}

































/*************************************************************************************************************
DS18B20的程序
*******************************************************************/
/*********************************************************************
 * 函数名称：initDS18B20
 * 功    能：初始化P1_0、P1_1端口为输出GPIO,因为查看所得，只有该两个端口输出电流可达20mA
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
void init_1820()
{
  SET_DQ_OUT;   //将DQ设置为输出
  DQ = 0;       //将DQ置为低电平至少480u
  halMcuWaitUs(550);
  DQ = 1;       //置高
  
  SET_DQ_IN;    //将DQ设置为输入
  
  halMcuWaitUs(40); //释放总线后等待15-60us
  while(DQ)     //等待回复,读取DQ的值变为1
  {

  }  
  
  halMcuWaitUs(200);
  SET_DQ_OUT;
  DQ = 1;       //回到初始DQ=1；
  
  halMcuWaitUs(1); //延迟1us
   
}


/*********************************************************************
 * 函数名称：sendTemperature
 * 功    能：获取温度，并将数据转换 
 * 入口参数：无
 * 出口参数：温度值
 * 返 回 值：无
 ********************************************************************/
void sendTemperature(void)
{
  static uint16 sensor_temperature_value;
  sensor_temperature_value = read_data(); // 获取温度值
  
  float temperature=0.0;
  int n;
  uint16 roll = 0x0800;
  for(n=7;n>=-4;n--)
  {
    if(sensor_temperature_value&roll)
    {
      temperature += pow(2,n);
    }
     roll = roll>>1;
  }
  
                
  uint16 tempData;
  tempData = (int)(temperature*100);
  unsigned char moblieNodeTemperature[2]; 
  if (temperature >0 && temperature < 40)  //错误的数据不进行发送
  {
    moblieNodeTemperature[0] = ((uint8 *)&tempData)[1];      //温度高位 
    moblieNodeTemperature[1] = ((uint8 *)&tempData)[0];      //温度低位
    //moblieNodeTemperature[0] = ((uint8 *)&sensor_temperature_value)[1];      //温度高位 
    //moblieNodeTemperature[1] = ((uint8 *)&sensor_temperature_value)[0];      //温度低位
   }
  
  //错误的数据将数据填充为0xFFFF
  else
  {
    moblieNodeTemperature[0] = 0xFF;      //温度高位 
    moblieNodeTemperature[1] = 0xFF;      //温度低位
  }
  
  afAddrType_t Coord_DstAddr;
  Coord_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  Coord_DstAddr.endPoint = LOCATION_DONGLE_ENDPOINT;
  Coord_DstAddr.addr.shortAddr = 0x0000;  //表示数据包发往网络中的所有节点广播，设置0xFFFC，路由器会收不到，不知何解 
  AF_DataRequest( &Coord_DstAddr, (endPointDesc_t *)&epDesc,
                 CID_M2C_TEMPERATURE, 2,
                 moblieNodeTemperature, &transId,
                 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ); 
}
          
/*********************************************************************
 * 函数名称：read_data
 * 功    能：获取温度，并将数据转换 
 * 入口参数：无
 * 出口参数：无
 * 返 回 值：无
 ********************************************************************/
uint16 read_data(void)
{
  uint8 temh,teml;
  uint16 temp;
  
  init_1820();          //复位18b20
  write_1820(0xcc);     // 发出转换命令 忽略ROM
  write_1820(0x44);     //启动转换
  halMcuWaitUs(500);  //I/O 线就必须至少保持 500ms 高电平
  
  init_1820();
  write_1820(0xcc);
  write_1820(0xbe);     //读取暂存器和 CRC 字节
  
  teml=read_1820(); //读数据
  temh=read_1820();
  
  //sensor_temperature_value[0]=teml;
  //sensor_temperature_value[1]=temh;
  
  temp= temh;//将两个字节整合到一个unsigned int中
  temp<<=8;
  temp |= teml;
  
  return temp;
}

/*********************************************************************
 * 函数名称：read_1820
 * 功    能：获取温度，并将数据转换 
 * 入口参数：无
 * 出口参数：温度值
 * 返 回 值：无
 ********************************************************************/
uint8 read_1820(void)
{
  uint8 temp,k,n;
  temp=0;
  for(n=0;n<8;n++)
  {   SET_DQ_OUT;
      DQ = 1;
      halMcuWaitUs(1);
      DQ = 0;
      halMcuWaitUs(1); //延时1us
  
      DQ = 1;            
      SET_DQ_IN;
      halMcuWaitUs(6);//至少延时6us
  
      k = DQ;         //读数据,从低位开始
      if(k)
      temp|=(1<<n);
      else
      temp&=~(1<<n);
      //Delay_nus(60); //60~120us
      halMcuWaitUs(50);
      SET_DQ_OUT;
  
  }
  return (temp);
}

/*********************************************************************
 * 函数名称：write_1820
 * 功    能：获取温度，并将数据转换 
 * 入口参数：无
 * 出口参数：温度值
 * 返 回 值：无
 ********************************************************************/
void write_1820(uint8 x)
{
  uint8 m;
  SET_DQ_OUT;            
  for(m=0;m<8;m++)
   {  
     DQ = 1;
     halMcuWaitUs(1);
     DQ = 0;          //拉低
     if(x&(1<<m))    //写数据，从低位开始
      DQ = 1;         //要在15us把数据放在写的数据（0或1）送到总线上
     else
      DQ = 0;
     halMcuWaitUs(40);//拉高15—60us

     DQ = 1;
     halMcuWaitUs(3);//延时3us,两个写时序间至少需要1us的恢复期
   }
  DQ = 1;
}



/*********************************************************************
 * 函数名称： halMcuWaitUs
 * 功    能：忙等待功能。等待指定的微秒数。不同的指令所需的时钟周期数
 *           量不同。一个周期的持续时间取决于MCLK。(TI中的lightSwitch中的函数)
 * 入口参数：usec  延时时长，单位Wie微秒
 * 出口参数：无
 * 返 回 值：无
 * 注    意：此功能高度依赖于MCU架构和编译器。
 ********************************************************************/
#pragma optimize = none
void halMcuWaitUs(unsigned int usec)
{
  usec >>= 1;
  while(usec--)
  {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
  }
}