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
#include "mac_mcu.h"

#include "DS18B20.h"
#include "math.h"         //pel+ pow(,)

/*********************************************************************
 * CONSTANTS
 */
#define SEQUENCE                200  //序列号，用于判断是否收到的是同一次数据

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte Sink_TaskID;
uint8 transId;
static byte transId;

unsigned char uc_sequence = 100;        //改为计数从100-200，和结尾符0x0a分开
unsigned int sink_broadcast_period = 500; //广播周期一次为180ms发射一次,单位为ms,4个移动节点的极限了吧
unsigned char sink_temp_cycle  = 10;                     //单位为秒，在实际使用时，需要*1000，因为函数单位ms；

/*********************************************************************
 * FUNCATION
*/
void initP1();
static void processMSGCmd( afIncomingMSGPacket_t *pkt );
void getBasicValue();
void setBasicValue(afIncomingMSGPacket_t *pkt);
void successResponse(uint8 result);
void startBroadcast();

/*********************************************************************/
 // LOCAL VARIABLES
 
static const cId_t Sink_InputClusterList[] =
{
  CID_C2A_GET_BASIC_VALUE,
  CID_C2A_SET_BASIC_VALUE
};

static const cId_t Sink_OutputClusterList[] =
{
  CID_S2MR_BROADCAST,
  CID_S2C_TEMPERATURE,        //发送温度给协调器
  CID_A2C_RP_BASIC_VALUE,
  CID_A2C_SUCCESS_RESPONSE
};

static const SimpleDescriptionFormat_t Sink_SimpleDesc =
{
  LOCATION_SINK_ENDPOINT,
  LOCATION_PROFID,
  LOCATION_SINK_DEVICE_ID,
  LOCATION_DEVICE_VERSION,
  LOCATION_FLAGS,
  sizeof(Sink_InputClusterList),
  (cId_t*)Sink_InputClusterList,
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
        case AF_INCOMING_MSG_CMD:
        {
          processMSGCmd( MSGpkt );
          break;
        }
  
        case ZDO_STATE_CHANGE:
        {
          //初始化时将基本数据发送给协调器   
          getBasicValue();
          osal_start_timerEx( Sink_TaskID, SINK_BROADCAST_EVT, sink_broadcast_period );  //设置广播定时                                      
  
          //进入网络获取温度，并将其传到协调器
          //sendTemperature();
          //osal_start_timerEx( Sink_TaskID, SINK_TEMP_EVT, sink_temp_cycle*1000 );  //设置周期发送温度信息定时       
          break;
        }
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
    osal_start_timerEx( Sink_TaskID, SINK_BROADCAST_EVT, sink_broadcast_period );
      
    // return unprocessed events
    return ( events ^ SINK_BROADCAST_EVT );
  }
    
  if ( events & SINK_TEMP_EVT )    
  {
    sendTemperature();             //调用发送温度
    osal_start_timerEx( Sink_TaskID, SINK_TEMP_EVT, sink_temp_cycle*1000 );
    
    return ( events ^ SINK_TEMP_EVT );
  }
  
  // Discard unknown events.
  return 0;  
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
  unsigned char theMessageData[S2C_RP_BASIC_VALUE_LENGTH];     
  theMessageData[S2C_RP_BASIC_VALUE_NODE_TYPE] = NT_SINK_NODE;
  theMessageData[S2C_RP_BASIC_VALUE_BROAD_CYC] = sink_broadcast_period;
  theMessageData[S2C_RP_BASIC_VALUE_TEMP_CYC] = sink_temp_cycle;
  
  afAddrType_t coorAddr;
  coorAddr.addrMode = (afAddrMode_t)Addr16Bit; //单播发送
  coorAddr.endPoint = LOCATION_DONGLE_ENDPOINT; //目的端口号
  coorAddr.addr.shortAddr = 0x0000;            //协调器网络地址
  AF_DataRequest( &coorAddr, (endPointDesc_t *)&epDesc,
                           CID_A2C_RP_BASIC_VALUE, S2C_RP_BASIC_VALUE_LENGTH,
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
  //将协调器发送过来的配置写入sink节点中
  sink_broadcast_period = pkt->cmd.Data[C2S_SET_BASIC_VALUE_BROAD_CYC];
  sink_temp_cycle = pkt->cmd.Data[C2S_SET_BASIC_VALUE_TEMP_CYC];
  
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
  //收到配置信息闪烁绿灯3次，每次300ms
  HalLedBlink(HAL_LED_1, 3, 50, 300);
  
  //将设置成功信息返回给协调器
  unsigned char theMessageData[A2C_SUCCESS_RESPONSE_LENGTH];     
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_TYPE] = NT_SINK_NODE;
  theMessageData[A2C_SUCCESS_RESPONSE_NODE_ID] = 1;             //sink节点只有1个，故节点为1
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
  if (uc_sequence < SEQUENCE)     //设置发射序列号
  {
    uc_sequence++;
  }
  else
  {
    uc_sequence = 100;            
  }
    
  //发射电磁波
  unsigned char theMessageData[1];     //发射内容为序列号 移动节点号
  theMessageData[0] = uc_sequence;
  afAddrType_t broadcast_DstAddr;
  broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  broadcast_DstAddr.endPoint = LOCATION_REFER_ENDPOINT;           //暂时未解决如何广播两种端口的问题，只能将不同类型的节点的ENDPOINT设置为一样的
  broadcast_DstAddr.addr.shortAddr = 0xFFFF;  //表示数据包发往网络中的所有节点广播，设置0xFFFC，路由器会收不到，不知何解 
  if( AF_DataRequest( &broadcast_DstAddr, (endPointDesc_t *)&epDesc,
                 CID_S2MR_BROADCAST, 1,
                 theMessageData, &transId,
                 0, 1        //AF_DISCV_ROUTE AF_DEFAULT_RADIUS    //设置为一跳,直接进行广播
                 ) == afStatus_SUCCESS)
  {
    P1_1 = ~P1_1;             //改变一次LED_R的状态，表示发射完广播信号
  }  
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
  unsigned char sinkNodeTemperature[2]; 
  if (temperature >0 && temperature < 40)  //错误的数据不进行发送
  {
    sinkNodeTemperature[0] = ((uint8 *)&tempData)[1];      //温度高位 
    sinkNodeTemperature[1] = ((uint8 *)&tempData)[0];      //温度低位
    //moblieNodeTemperature[0] = ((uint8 *)&sensor_temperature_value)[1];      //温度高位 
    //moblieNodeTemperature[1] = ((uint8 *)&sensor_temperature_value)[0];      //温度低位
   }
  
  //错误的数据将数据填充为0xFFFF
  else
  {
    sinkNodeTemperature[0] = 0xFF;      //温度高位 
    sinkNodeTemperature[1] = 0xFF;      //温度低位
  }
  
  afAddrType_t Coord_DstAddr;
  Coord_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  Coord_DstAddr.endPoint = LOCATION_DONGLE_ENDPOINT;
  Coord_DstAddr.addr.shortAddr = 0x0000;  //表示数据包发往网络中的所有节点广播，设置0xFFFC，路由器会收不到，不知何解 
  AF_DataRequest( &Coord_DstAddr, (endPointDesc_t *)&epDesc,
                 CID_S2C_TEMPERATURE, 2,
                 sinkNodeTemperature, &transId,
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