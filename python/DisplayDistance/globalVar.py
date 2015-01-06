# Filename: globalVar.py
# Discription: This's file is used for define the global Variable
# made by pel 2014.11.09
# the data type is : 'SequenceNo(1) NodeNo(1) Data(4)*4 \n(1)'


#define the const
TOTAL_DATA_LENGTH = int(7)    #define the total data length 4 + 3 = 7
EACH_DATA_LENGTH = int(3)       #define the eahc data length is 4 bytes.  xx xx xx
#define the position of the reffer node  [NodeNo,x,y,z]
REFER_NODE_POSITION = [[1,0,0,120],[2,140,0,120],[3,0,280,120],[4,140,280,120]]

#define the global variable
lastSequenceNo = 0
eachDistanceDict = {}
tempurateData = 22              #the default tempurature is 20

