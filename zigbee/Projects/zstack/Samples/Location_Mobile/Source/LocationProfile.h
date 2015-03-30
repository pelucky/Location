#ifndef LOCATIONPROFILE_H
#define LOCATIONPROFILE_H

/*Endpoint */
#define LOCATION_MOBILE_ENDPOINT			210   //暂时设置为一样的，这样广播才能接收
#define LOCATION_REFER_ENDPOINT				210
#define LOCATION_DONGLE_ENDPOINT 			212
#define LOCATION_SINK_ENDPOINT 			        213
//#define LOCATION_MOBILE_ENDPOINT			211 

#define LOCATION_PROFID					0xC003
#define LOCATION_DEVICE_VERSION				0
#define LOCATION_FLAGS					0


/*Device ID*/
#define LOCATION_REFER_DEVICE_ID			0x0010        
#define	LOCATION_MOBILE_DEVICE_ID			0x0011
#define LOCATION_DONGLE_DEVICE_ID			0x0012
#define LOCATION_SINK_DEVICE_ID			        0x0013

/*Cluster IDs */
#define CID_A2C_RP_BASIC_VALUE                0x0010//其他节点向coor节点返回查询配置
#define CID_A2C_SUCCESS_RESPONSE              0x0011//其他节点向coor节点响应配置成功
#define CID_C2A_GET_BASIC_VALUE               0x0012//coor节点向其他节点发出查询配置
#define CID_C2A_SET_BASIC_VALUE               0x0013//coor节点向其他节点发出设置配置
#define CID_S2MR_BROADCAST                    0x0014//sink节点向mobile和refer节点广播信号
#define CID_M2M_TEAM_DATA                     0x0015//mobile节点向其他mobile节点发送队伍信号
#define CID_M2M_TEAM_CONTROL                  0x0016//mobile节点向其他mobile节点发送控制信号
#define CID_M2C_REQ_POSITION                  0x0017//mobile节点向coor节点发出位置查询请求
#define CID_C2M_RP_POSITION                   0x0018//coor节点向mobile发出位置查询响应
#define CID_C2M_SET_JUDGE                     0x0019//coor节点向mobile发出起始命令
#define CID_R2C_DIFF_TIME                     0x001A//refer节点发时间差给coor节点
#define CID_S2C_TEMPERATURE                   0x001B//sink节点向coor节点发送温度数据



/*
缩写解释：
  下面的PC表示上位机；C表示协调器；S表示sink节点；M表示移动节点；R表示参考节点;A表示sink或移动或参考节点
  C2PC即表示从协调器发给上位机
  MT表示Message Type;NT表示Node Type;CID表示Cluster ID
*/
/*Message Format*/
#define MT_C2PC_DIFF_TIME              1   //协调器发给上位机距离时间差数据帧
#define MT_C2PC_TEMPERATURE_DATA       2   //协调器发给上位机温度数据帧
#define MT_C2PC_SUCCESS_RESPONSE       3   //协调器发给上位机设置成功的响应帧
#define MT_C2PC_RP_BASIC_VALUE_C       4   //协调器发给上位机协调器的基本信息帧
#define MT_C2PC_RP_BASIC_VALUE_S       5   //协调器发给上位机sink节点的基本信息帧
#define MT_C2PC_RP_BASIC_VALUE_M       6   //协调器发给上位机mobile节点的基本信息帧
#define MT_C2PC_RP_BASIC_VALUE_R       7   //协调器发给上位机refer节点的基本信息帧
#define MT_C2PC_REQ_POSITION           8   //协调器发给上位机节点位置请求信息帧

#define MT_PC2C_GET_BASIC_VALUE       11   //上位机发给协调器请求查询节点的基本信息
#define MT_PC2C_SET_BASIC_VALUE_C     12   //上位机发给协调器设置协调器节点的基本信息
#define MT_PC2C_SET_BASIC_VALUE_S     13   //上位机发给协调器设置sink节点的基本信息
#define MT_PC2C_SET_BASIC_VALUE_M     14   //上位机发给协调器设置mobile节点的基本信息
#define MT_PC2C_SET_BASIC_VALUE_R     15   //上位机发给协调器设置refer节点的基本信息
#define MT_PC2C_RP_POSITION           16   //上位机发给协调器响应移动节点位置的请求信息帧
#define MT_PC2C_SET_JUDGE             17   //上位机发给协调器设置开始结束的信息帧

/*Node Type*/
#define NT_COOR_NODE   1         
#define NT_SINK_NODE   2
#define NT_MOB_NODE    3
#define NT_REF_NODE    4

/*C2PC*/
//距离时间差数据帧
/*
   3-5位为1号参考节点的时间差数据；6-8位为2号参考节点的时间差数据；
   9-11为3号参考节点的时间差数据；12-14位为4号参考节点的时间差数据
   由于采用数组循环的方法，故不需要列出具体的帧
*/
#define C2PC_DIFF_TIME_LENGTH        16 //帧长度
#define C2PC_DIFF_TIME_MSG_TYPE       0 //帧类型
#define C2PC_DIFF_TIME_SEQ            1 //序列号
#define C2PC_DIFF_TIME_MOBID          2 //移动节点ID
#define C2PC_DIFF_TIME_END           15 //终止符 \n

//温度数据帧
#define C2PC_TEMPERATURE_DATA_LENGTH    4 //帧长度
#define C2PC_TEMPERATURE_DATA_MSG_TYPE  0 //帧类型
#define C2PC_TEMPERATURE_DATA_HIGH      1 //温度数据帧高8位  需要除以100                      
#define C2PC_TEMPERATURE_DATA_LOW       2 //温度数据帧低8位
#define C2PC_TEMPERATURE_DATA_END       3 //终止符\n

//设置成功响应帧
#define C2PC_SUCCESS_RESPONSE_LENGTH    5 //帧长度
#define C2PC_SUCCESS_RESPONSE_MSG_TYPE  0 //帧类型
#define C2PC_SUCCESS_RESPONSE_NODE_TYPE 1 //成功响应的节点类型
#define C2PC_SUCCESS_RESPONSE_NODE_ID   2 //成功响应的节点ID
#define C2PC_SUCCESS_RESPONSE_RESULT    3 //成功响应的结果：1表示成功，0表示失败
#define C2PC_SUCCESS_RESPONSE_END       4 //终止符\n

//各节点的基本信息帧
//C的基本信息帧
#define C2PC_RP_BASIC_VALUE_LENGTH_C      3 //帧长度
#define C2PC_RP_BASIC_VALUE_MSG_TYPE_C    0 //帧类型
#define C2PC_RP_BASIC_VALUE_DELAY_TIME_C  1 //延迟发送给上位机的时间
#define C2PC_RP_BASIC_VALUE_END_C         2 //终止符\n
//S的基本信息帧
#define C2PC_RP_BASIC_VALUE_LENGTH_S      4 //帧长度
#define C2PC_RP_BASIC_VALUE_MSG_TYPE_S    0 //帧类型
#define C2PC_RP_BASIC_VALUE_BROAD_CYC_S   1 //Sink节点的广播周期
#define C2PC_RP_BASIC_VALUE_TEMP_CYC_S    2 //Sink节点发送温度周期
#define C2PC_RP_BASIC_VALUE_END_S         3 //终止符\n
//M的基本信息帧
#define C2PC_RP_BASIC_VALUE_LENGTH_M           5 //帧长度
#define C2PC_RP_BASIC_VALUE_MSG_TYPE_M         0 //帧类型
#define C2PC_RP_BASIC_VALUE_MOB_ID_M           1 //移动节点ID号
#define C2PC_RP_BASIC_VALUE_TEAM_ID_M          2 //移动节点队伍ID号
#define C2PC_RP_BASIC_VALUE_SEND_DELAY_TIME_M  3 //移动节点发送延迟周期
#define C2PC_RP_BASIC_VALUE_END_M              4 //终止符\n
//R的基本信息帧
#define C2PC_RP_BASIC_VALUE_LENGTH_R           5 //帧长度
#define C2PC_RP_BASIC_VALUE_MSG_TYPE_R         0 //帧类型
#define C2PC_RP_BASIC_VALUE_REF_ID_R           1 //参考节点ID号
#define C2PC_RP_BASIC_VALUE_RECV_TIME_OUT_R    2 //参考节点接收超时时间
#define C2PC_RP_BASIC_VALUE_RECV_DELAY_TIME_R  3 //参考节点接收延迟周期
#define C2PC_RP_BASIC_VALUE_END_R              4 //终止符\n

//移动节点位置请求信息帧
#define C2PC_REQ_POSITION_LENGTH           4 //帧长度
#define C2PC_REQ_POSITION_MSG_TYPE         0 //帧类型
#define C2PC_REQ_POSITION_REQ_MOB_ID       1 //请求的移动节点的ID
#define C2PC_REQ_POSITION_GET_MOB_ID       2 //需要获取的移动节点的ID
#define C2PC_REQ_POSITION_END              3 //终止符\n

/*PC2C*/
//请求查询节点的基本信息
#define PC2C_GET_BASIC_VALUE_LENGTH         3 //帧长度
#define PC2C_GET_BASIC_VALUE_MSG_TYPE       0 //帧类型
#define PC2C_GET_BASIC_VALUE_NODE_TYPE      1 //查询节点的节点类型
#define PC2C_GET_BASIC_VALUE_NODE_ID        2 //查询节点的节点ID号

//设置节点的基本信息
//设置C的基本信息帧
#define PC2C_SET_BASIC_VALUE_LENGTH_C      2 //帧长度
#define PC2C_SET_BASIC_VALUE_MSG_TYPE_C    0 //帧类型
#define PC2C_SET_BASIC_VALUE_DELAY_TIME_C  1 //延迟发送给上位机的时间
//设置S的基本信息帧
#define PC2C_SET_BASIC_VALUE_LENGTH_S      3 //帧长度
#define PC2C_SET_BASIC_VALUE_MSG_TYPE_S    0 //帧类型
#define PC2C_SET_BASIC_VALUE_BROAD_CYC_S   1 //Sink节点的广播周期
#define PC2C_SET_BASIC_VALUE_TEMP_CYC_S    2 //Sink节点发送温度周期
//设置M的基本信息帧
#define PC2C_SET_BASIC_VALUE_LENGTH_M           4 //帧长度
#define PC2C_SET_BASIC_VALUE_MSG_TYPE_M         0 //帧类型
#define PC2C_SET_BASIC_VALUE_MOB_ID_M           1 //移动节点ID号
#define PC2C_SET_BASIC_VALUE_TEAM_ID_M          2 //移动节点队伍ID号
#define PC2C_SET_BASIC_VALUE_SEND_DELAY_TIME_M  3 //移动节点发送延迟周期
//设置R的基本信息帧
#define PC2C_SET_BASIC_VALUE_LENGTH_R           4 //帧长度
#define PC2C_SET_BASIC_VALUE_MSG_TYPE_R         0 //帧类型
#define PC2C_SET_BASIC_VALUE_REF_ID_R           1 //参考节点ID号
#define PC2C_SET_BASIC_VALUE_RECV_TIME_OUT_R    2 //参考节点接收超时时间
#define PC2C_SET_BASIC_VALUE_RECV_DELAY_TIME_R  3 //参考节点接收延迟周期

//响应移动节点位置的请求信息帧
#define PC2C_RP_POSITION_LENGTH           7 //帧长度
#define PC2C_RP_POSITION_MSG_TYPE         0 //帧类型
#define PC2C_RP_POSITION_REQ_MOB_ID       1 //请求查询的移动节点ID号
#define PC2C_RP_POSITION_GET_MOB_ID       2 //被查询的移动节点ID号
#define PC2C_RP_POSITION_X_HI             3 //被查询的移动节点的坐标X值高八位
#define PC2C_RP_POSITION_X_LO             4 //被查询的移动节点的坐标X值低八位
#define PC2C_RP_POSITION_Y_HI             5 //被查询的移动节点的坐标Y值高八位
#define PC2C_RP_POSITION_Y_LO             6 //被查询的移动节点的坐标Y值低八位

//设置开始结束的信息帧
#define PC2C_SET_JUDGE_LENGTH             3 //帧长度
#define PC2C_SET_JUDGE_MSG_TYPE           0 //帧类型
#define PC2C_SET_JUDGE_MOB_ID             1 //需要设置的移动节点的ID号
#define PC2C_SET_JUDGE_ACTION             2 //具体措施

/*S2C*/
//响应基本配置信息
#define S2C_RP_BASIC_VALUE_LENGTH       3 //帧长度
#define S2C_RP_BASIC_VALUE_NODE_TYPE    0 //节点类型
#define S2C_RP_BASIC_VALUE_BROAD_CYC    1 //Sink节点的广播周期
#define S2C_RP_BASIC_VALUE_TEMP_CYC     2 //Sink节点发送温度周期

//发送温度信息
#define S2C_TEMPERATURE_DATA_LENGTH     2 //帧长度
#define S2C_TEMPERATURE_DATA_HIGH       0 //温度数据高8位
#define S2C_TEMPERATURE_DATA_LOW        1 //温度数据低8位

/*C2S*/
//设置基本信息帧
#define C2S_SET_BASIC_VALUE_LENGTH       2 //帧长度
#define C2S_SET_BASIC_VALUE_BROAD_CYC    0 //Sink节点的广播周期
#define C2S_SET_BASIC_VALUE_TEMP_CYC     1 //Sink节点发送温度周期

/*M2M*/
//接收or发送本队其他移动节点的队伍信息
#define M2M_TEAM_DATA_LENGTH            2 //帧长度
#define M2M_TEAM_DATA_TEAM_ID           0 //移动节点队伍ID号
#define M2M_TEAM_DATA_MOB_ID            1 //移动节点ID号

//接收or发送本队其他移动节点的控制信息  
#define M2M_TEAM_CONTROL_LENGTH         4 //帧长度
#define M2M_TEAM_CONTROL_REQ_MOB_ID     0 //请求控制的移动节点ID号（也就是自己的ID）
#define M2M_TEAM_CONTROL_GET_MOB_ID     1 //需要控制的移动节点ID号
#define M2M_TEAM_CONGROL_ACTION_HI      2 //控制操作高八位
#define M2M_TEAM_CONGROL_ACTION_LO      3 //控制操作低八位


/*M2C*/
//响应基本配置信息
#define M2C_RP_BASIC_VALUE_LENGTH           4 //帧长度
#define M2C_RP_BASIC_VALUE_NODE_TYPE        0 //节点类型
#define M2C_RP_BASIC_VALUE_MOB_ID           1 //移动节点ID号
#define M2C_RP_BASIC_VALUE_TEAM_ID          2 //移动节点队伍ID号
#define M2C_RP_BASIC_VALUE_SEND_DELAY_TIME  3 //移动节点发送延迟周期

//节点位置请求信息
#define M2C_REQ_POSITION_LENGTH         2 //帧长度
#define M2C_REQ_POSITION_REQ_MOB_ID     0 //请求的移动节点的ID
#define M2C_REQ_POSITION_GET_MOB_ID     1 //需要获取的移动节点的ID

/*C2M*/
//设置基本信息帧
#define C2M_SET_BASIC_VALUE_LENGTH            3 //帧长度
#define C2M_SET_BASIC_VALUE_MOB_ID            0 //移动节点ID号
#define C2M_SET_BASIC_VALUE_TEAM_ID           1 //移动节点队伍ID号
#define C2M_SET_BASIC_VALUE_SEND_DELAY_TIME   2 //移动节点发送延迟周期

//获取移动节点位置的请求信息帧
#define C2M_RP_POSITION_LENGTH           5 //帧长度
#define C2M_RP_POSITION_GET_MOB_ID       0 //被查询的移动节点ID号
#define C2M_RP_POSITION_X_HI             1 //被查询的移动节点的坐标X值高八位
#define C2M_RP_POSITION_X_LO             2 //被查询的移动节点的坐标X值低八位
#define C2M_RP_POSITION_Y_HI             3 //被查询的移动节点的坐标Y值高八位
#define C2M_RP_POSITION_Y_LO             4 //被查询的移动节点的坐标Y值低八位

//设置开始结束的信息帧
#define C2M_SET_JUDGE_LENGTH             1 //帧长度
#define C2M_SET_JUDGE_ACTION             0 //具体措施

/*R2C*/
//响应基本配置信息
#define R2C_RP_BASIC_VALUE_LENGTH                      4 //帧长度
#define R2C_RP_BASIC_VALUE_NODE_TYPE                   0 //节点类型
#define R2C_RP_BASIC_VALUE_REF_ID                      1 //参考节点ID号
#define R2C_RP_BASIC_VALUE_RECV_TIME_OUT               2 //参考节点接收超时时间
#define R2C_RP_BASIC_VALUE_RECV_DELAY_TIME             3 //参考节点接收延迟周期

//发送时间差信息
/*
   2-4位为1号移动节点的时间差数据；5-7位为2号移动节点的时间差数据；
   8-10为3号移动节点的时间差数据；11-13位为4号移动节点的时间差数据
   由于采用数组循环的方法，故不需要列出具体的帧
*/
#define R2C_DIFF_TIME_LENGTH            14 //帧长度
#define R2C_DIFF_TIME_SEQ                0 //序列号
#define R2C_DIFF_TIME_REFID              1 //参考节点ID

/*C2R*/
//设置基本信息帧
#define C2R_SET_BASIC_VALUE_LENGTH           3 //帧长度
#define C2R_SET_BASIC_VALUE_REF_ID           0 //参考节点ID号
#define C2R_SET_BASIC_VALUE_RECV_TIME_OUT    1 //参考节点接收超时时间
#define C2R_SET_BASIC_VALUE_RECV_DELAY_TIME  2 //参考节点接收延迟周期

/*A2C*/
//响应配置成功
#define A2C_SUCCESS_RESPONSE_LENGTH                    3 //帧长度
#define A2C_SUCCESS_RESPONSE_NODE_TYPE                 0 //节点类型
#define A2C_SUCCESS_RESPONSE_NODE_ID                   1 //节点ID
#define A2C_SUCCESS_RESPONSE_RESULT                    2 //设置的结果

/*C2A*/
//查询基本信息
//不需要帧内容，直接发送即可，可以在AF_DataRequest中将内容和长度设置为0即可。

#endif //#ifndef LOCATIONPROFILE_H