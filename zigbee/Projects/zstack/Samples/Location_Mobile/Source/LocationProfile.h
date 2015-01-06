#ifndef LOCATIONPROFILE_H
#define LOCATIONPROFILE_H

/*Endpoint */
#define LOCATION_REFER_ENDPOINT				210
#define LOCATION_MOBILE_ENDPOINT				210   //暂时设置为一样的，这样广播才能接收
//#define LOCATION_MOBILE_ENDPOINT				211  
#define LOCATION_DONGLE_ENDPOINT 			212
#define LOCATION_SINK_ENDPOINT 			213

#define LOCATION_PROFID						0xC003
#define LOCATION_DEVICE_VERSION				0
#define LOCATION_FLAGS						0


/*Device ID*/
#define LOCATION_REFER_DEVICE_ID				0x0010        
#define	LOCATION_MOBILE_DEVICE_ID			0x0011
#define LOCATION_DONGLE_DEVICE_ID			0x0012
#define LOCATION_SINK_DEVICE_ID			        0x0013

/*Cluster IDs */
#define LOCATION_ULTRA_BLORDCAST                   0x0010//mobile接收sink节点广播请求
#define LOCATION_COOR_TEMPURATE                    0x0011//mobile发送当前温度给coor节点
#define LOCATION_REFER_DISTANCE_RSP                0x0012//refer发回Ultra距离时间差
//#define LOCATION_COOR_ERROR                        0x0013//发给协调器错误码报告




/*Default values */


/*Message Format*/
#define   RP_BLOADCAST_TIME         1   //发回距离时间差数据帧
#define   RP_TEMPERATURE_DATA       2   //发回温度数据帧
#define   RP_ERROR_DATA             3   //发回错误帧的报告

// LOCATION_RSSI_BLAST
// 不带任何信息，广播

// LOCATION_RSSI_REQ
// 不带任何信息，广播

/*

*/

// LOCATION_MOBILE_FIND_REQ
// 不带任何信息，发送给某一特定的mobile


//LOCATION_REFFR_POSITION_RSP
#define LOCATION_REFER_POSITION_RSP_LENGTH 		14//信息长度
//#define LOCATION_REFER_POSITION_RSP_LENGTH 		6//信息长度
#define LOCATION_REFER_POSITION_RSP_SEQUENCE            0//序列号
#define LOCATION_REFER_POSITION_RSP_REFID		1//参考节点ID
#define LOCATION_REFER_POSITION_RSP_DSITANCE_HIGH		3//测量出来的距离
#define LOCATION_REFER_POSITION_RSP_DSITANCE_MIDD		4//测量出来的距离
#define LOCATION_REFER_POSITION_RSP_DSITANCE_LOW		5//测量出来的距离

//LOCATION_REFFR_POSITION_RSP
#define LOCATION_REFER_ERROR_POSITION_RSP_LENGTH 		3//信息长度
#define LOCATION_REFER_ERROR_POSITION_MSG_TYPE                  0//错误类型
#define LOCATION_REFER_ERROR_POSITION_RSP_FIXID		        1//固定ID
#define LOCATION_REFER_ERROR_POSITION_SEQ_NO		        2//测量出来的距离


//协调器 距离信息帧
#define	    TIMEDIFF_MSG_LENGTH		                  7
#define     TIMEDIFF_MSG_TYPE                             0
#define     TIMEDIFF_SEQUENCE                             1//序列号
#define     TIMEDIFF_FIXID       			  2//ID号
#define     TIMEDIFF_TIMEDIFF_HIGH   			  3//时间差高八位
#define     TIMEDIFF_TIMEDIFF_MIDD   			  4//时间差中八位
#define     TIMEDIFF_TIMEDIFF_LOW   			  5//时间差低八位
#define     TIMEDIFF_END       			          6//终止符\n

//协调器 温度帧
#define	    TEMP_MSG_LENGTH		          4
#define     TEMP_MSG_TYPE                         0
#define     TEMP_DATA_LOW                         1//需要除以100
#define     TEMP_DATA_HIGH                        2//
#define     TEMP_END       			  3//终止符\n

//错误数据交由上位机处理显示
/*
//错误数据 类型
#define     ERROR_NOT_GET_US                      1//未接收到超声波信号
#define     ERROR_US_DATA                         2//接收到的超声波信号是错误数据
#define     ERROR_TEMP_DATA                       3//接收到的温度数据是错误数据
#define     ERROR_LESS_DATA                       4//接收到的定位信息少于3个

//协调器 温度错误帧
#define	    ERROR_TEMP_MSG_LENGTH		  4
#define     ERROR_TEMP_MSG_TYPE                   0
#define     ERROR_TEMP_TYPE                       1//错误类型
#define     ERROR_TEMP_NODE_ID                    2//节点ID
#define     ERROR_TEMP_END       		  3//终止符\n


//协调器 定位错误帧
#define	    ERROR_POS_MSG_LENGTH		  5
#define     ERROR_POS_MSG_TYPE                    0
#define     ERROR_POS_TYPE                        1//错误类型
#define     ERROR_POS_NODE_ID                     2//节点ID
#define     ERROR_POS_SEQ_NO       		  3//终止符\n
#define     ERROR_POS_END       		  4//终止符\n

//协调器 定位数少于3的错误帧
#define	    ERROR_LESS_MSG_LENGTH		  3
#define     ERROR_LESS_MSG_TYPE                   0
#define     ERROR_LESS_TYPE                       1//错误类型
#define     ERROR_LESS_END       		  2//终止符\n
*/
#endif //#ifndef LOCATIONPROFILE_H