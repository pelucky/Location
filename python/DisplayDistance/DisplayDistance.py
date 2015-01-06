# Filename: CaculateDistance.py
# Discription: This's file is used for caculating the distance by using the 2nd way(add sink node) ,realDist = 0.9817 * CalDist +11.869
# made by pel 2014.11.05
# the data type is : 'MsgTYPE(1) SequenceNo(1) MobID(1) NodeNo(1) Data(3) \n(1)'

import serial
import globalVar
import numpy as np
import datetime
import time



def getSerial():
    global lastTime
    global MobID
    lastTime = 0
    ser = serial.Serial(port='COM5', baudrate=19200, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)     #remember change this!
    print ser
    while True:                      #Always read data
        #get the serial data
        print '************************begin*****************************'
        line = ser.readline()                      #make sure the receive data is end with '\n' per time
        Time = datetime.datetime.today()
        print 'This received time is: ' + str(Time)
        thisTime = time.time()
        spendTimes = int(thisTime*1000) - int(lastTime*1000)
        lastTime = thisTime   
        print 'it totaly used ' + str(spendTimes) + 'ms'
    
        msgType = checkFitstData(line)                        #check the first data -- the message type
        if msgType == '01':
            if (len(line)-4)%globalVar.EACH_DATA_LENGTH!=0:                      #make sure has get rigtt length data
                leftDataLength = ser.inWaiting()             #get the left data cos the data may be 0x0a .the same with \n
                leftData = ser.read(leftDataLength)
                line = line + leftData
            #deal with the data
            sequenceNo = getNo(line,1)
            MobID = getNo(line,2)
            print str(sequenceNo) + ' , MobileID is ' + str(MobID)
            dataTimes = 4       #(len(line)-4)/globalVar.EACH_DATA_LENGTH                   #check get how many times datas per getin
            for times in range (0,dataTimes):
                dataLine = line[times*globalVar.EACH_DATA_LENGTH+3:(times+1)*globalVar.EACH_DATA_LENGTH+3]       #get one data per times
                referNo = times+1                                                           #getNo(dataLine,0)
                print " :The Node " + str(referNo) + " Distance is :",                     #Do not change the line
                hexShow(dataLine)
                storeResult = hex2int(dataLine)
                #print storeResult[i]
                eachDistance = calcDistance(storeResult)
                globalVar.eachDistanceDict[referNo] = eachDistance
            calcPosition(globalVar.eachDistanceDict)
            '''
            if (len(line)-4)%globalVar.EACH_DATA_LENGTH!=0:                      #make sure has get rigtt length data,the four bytes are mstType SeqNo NodeNO \n
                leftDataLength = ser.inWaiting()             #get the left data cos the data may be 0x0a .the same with \n
                leftData = ser.read(leftDataLength)
                line = line + leftData
            #deal with the data
            dataTimes = len(line)/globalVar.TOTAL_DATA_LENGTH                   #check whether get many times datas per getin
            for times in range (0,dataTimes):
                dataLine = line[times*globalVar.TOTAL_DATA_LENGTH:(times+1)*globalVar.TOTAL_DATA_LENGTH]       #get one data per times
                sequenceNo = getNo(dataLine,1)
                referNo = getNo(dataLine,2)
                print str(sequenceNo) + " :The Node " + str(referNo) + " Distance is :",                     #Do not change the line
                hexShow(dataLine)
                storeResult = hex2int(dataLine)
                #print storeResult[i]
                eachDistance = calcDistance(storeResult)
                if eachDistance != 0:
                    calcPosition(eachDistance,referNo,sequenceNo)
            '''
        if msgType == '02':
            if len(line)%4!=0:                      #make sure has get rigtt length data
                leftDataLength = ser.inWaiting()             #get the left data cos the data may be 0x0a .the same with \n
                leftData = ser.read(leftDataLength)
                line = line + leftData
            hexShow(line)
            globalVar.tempurateData = getTempurate(line)

        print '************************end*****************************'
        print
        print
        print
        
    ser.close()

def calcPosition(argvEachDistanceDict):
    #for delNode, delDistance in globalVar.eachDistanceDict.items():              
    #print 'The node %s next distance is %s cm.' % (delNode,delDistance)       
    useMatrix(argvEachDistanceDict)
    for delNode, delDistance in globalVar.eachDistanceDict.items():              #delete the old data
        globalVar.eachDistanceDict[delNode] = 0

def useMatrix(argv):
    #print 'useMatrix'
    referList = []
    for getNode,getDistance in globalVar.eachDistanceDict.items():
        if getDistance != 0:
            #cos the list is begin with 0 and the append must have only one para. the list type is: [distance,x,y]
            templist = [getDistance,globalVar.REFER_NODE_POSITION[getNode-1][1],globalVar.REFER_NODE_POSITION[getNode-1][2]]    
            referList.append(templist)
    #print referList        
    goodValueLength = len(referList)
    if goodValueLength < 3:
        print "Can't Location cos the useful data is less than 3!"
        print '---------------------------------------------------------'
        print
    else:
        #print 'The goodValueLength is :' + str(goodValueLength)
        listA=[]
        listb=[]
        #must take a look at the expression, use numpy
        for i in range(0,goodValueLength-1):
            listA.append([2*(referList[i][1]-referList[goodValueLength-1][1]),\
               2*(referList[i][2]-referList[goodValueLength-1][2])])
            listb.append([(referList[i][1])**2-(referList[goodValueLength-1][1])**2+\
               (referList[i][2])**2-(referList[goodValueLength-1][2])**2+\
               (referList[goodValueLength-1][0])**2-(referList[i][0])**2])
        A = np.array(listA)
        b = np.array(listb)
        '''
        print A
        print
        print b
        print
        print np.linalg.inv(np.dot(A.transpose(),A))
        print
        '''
        mobilePos = []
        mobilePos = np.dot(np.dot(np.linalg.inv(np.dot(A.transpose(),A)),A.transpose()),b)
        #print mobilePos
        print "                                                               The mobile's Position is X:" + str(round(mobilePos[0],2)) + ' Y:' + str(round(mobilePos[1],2))
        print '---------------------------------------------------------'
        print
        outputFileList(MobID,mobilePos, "nodePosition.txt")

       
def hexShow(argv):
    result = ''
    hLen = len(argv)
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex+' '
    print result

def checkFitstData(argv):
    hvol = ord(argv[0])
    hhex = '%02x'%hvol
    #print hhex
    return hhex

def getTempurate(argv):
    hvolHigh = ord(argv[2])
    hvolLow = ord(argv[1])
    hhexHigh = '%02x'%hvolHigh
    hhexLow = '%02x'%hvolLow
    hhex = hhexHigh + hhexLow
    
    if hhex == 'ffff':
        temperature = globalVar.tempurateData
        print 'The temperature is wrong! '
    else:
        temperature = (int('0x'+hhex,16 ))/100.0
        if temperature < 0 or temperature > 40:
            print 'The temperature is wrong! ' +str(temperature) + ' Cels. '
            temperature = globalVar.tempurateData
        else:
            print 'The temperature is : ' + str(temperature) + ' Cels. '
    return temperature


def hex2int(argv):
    result = ''
    for i in range(0,3):
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex
    #print result
    return int('0x'+result,16 )           #16 to 10

    
def getNo(argv,bit):
    hvol = ord(argv[bit])                 #get node nmuber -- the Third byte;  get seq Number -- the second byte
    NodeNo = '%02x'%hvol
    intNodeNo = int('0x'+NodeNo,16)       #16 to 10
    return intNodeNo

#the second way to caculate the distance -> Sink Node bloadcast the syn information  get the avaerge data from the 4 times
def calcDistance(argv):
    #the node did't get the US data
    if argv == 0:
        finalDistance = 0
        print 'It does not get US signal!'
    #0xFFFFFF ,it means the error data!
    elif argv == 16777215:        
        finalDistance = 0
        print 'It cacl the error data!'
    else:
        distanceData = round((argv * 31.25 / 10**7 )* (331.5 + 0.607 * 22) ,2)                  #globalVar.tempurateData caculte the average distace in the temputare of 5.0 C  the Unit is cm
        print
        print 'Origan Average Data: ' + str(distanceData) + 'cm.',
        finalDistance = round(distanceData * 1 - 10,2)
        #finalDistance = round(secAveData * 1.011 - 2.1866,2)
        print 'Modify Data:                                                                            ' + str(finalDistance) + 'cm.'
            
    print
    print
    return finalDistance



def outputFileList(ID,List, filename):
    fileHandler = file(filename, 'a+')
    if (ID==1):
        fileHandler.write('\n')
        fileHandler.write(str(datetime.datetime.today()))
    fileHandler.write('     '+str(ID)+': ')
    for item in List:
        fileHandler.write(str(round(item,2))+' ')
    fileHandler.close()
    

if __name__ == "__main__":
    print "Begin caculate the Distance by using the 2nd way"    
    
    getSerial()

    #never go here!
    print '-----------------------------------'         
    print 'hello world!...END... '

#the old way to caculate the Position
'''
def getTempurate(argv):
    hvolHigh = ord(argv[2])
    hvolLow = ord(argv[1])
    hhexHigh = '%02x'%hvolHigh
    hhexLow = '%02x'%hvolLow
    hhex = hhexHigh + hhexLow
    #print hhex
    integer = int(hhex[1:3],16)         #get the integer temperature
    #print integer
    temp = bin(int((hhex[-1]),16))                #get the last byte in bin
    temp1 = (temp[2:])[::-1]
    decimal = 0
    i = -4
    for item in temp1:
        decimal += int(item) * 2**(i)
        i = i + 1
    #print str(decimal)
    if ((integer + decimal) < 0) or ((integer + decimal) > 40):               #delet the wrong data below 0 and upper than 40 cels
        print 'The temperature is wrong!'
        temperature = globalVar.tempurateData     
    else:
        temperature = round((integer + decimal),2)
        print 'The temperature is : ' + str(temperature) + ' Cels. '
    return temperature


def hex2int(argv):
    allResult = []
    dataLength = len(argv) - 4             #delete the MsgType. Sequence NO. Node NO. '\n' 4 bytes
    dataTimes = dataLength / globalVar.EACH_DATA_LENGTH            #every data is 4 bytes
    for temp in range(0,dataTimes):       #get the data. find Data Frame
        result = ''
        for i in range(globalVar.EACH_DATA_LENGTH*temp+3,globalVar.EACH_DATA_LENGTH*temp+7):
            hvol = ord(argv[i])
            hhex = '%02x'%hvol
            result += hhex
        #print result
        allResult.append(int('0x'+result,16 ))           #16 to 10
    #print '0x'+result
    #intResult = int('0x'+result,16)              #16 to 10
    #print intResult
    #for item in allResult:
    #    print item
           
    return allResult
'''
'''
def calcPosition(argvEachDistance,argvReferNo,argvSequenceNo):
    if argvSequenceNo == globalVar.lastSequenceNo:
        globalVar.eachDistanceDict[argvReferNo] = argvEachDistance    #use dict key incoording to the value
    else:
        if globalVar.lastSequenceNo != 0:                 #the sequence:0 is the beginning

            #for delNode, delDistance in globalVar.eachDistanceDict.items():              
                #print 'The node %s next distance is %s cm.' % (delNode,delDistance)
                
            useMatrix(globalVar.eachDistanceDict)
            
            for delNode, delDistance in globalVar.eachDistanceDict.items():              #delete the old data
                globalVar.eachDistanceDict[delNode] = 0
                
        globalVar.lastSequenceNo = argvSequenceNo
        globalVar.eachDistanceDict[argvReferNo] = argvEachDistance
'''
'''
#the second way to caculate the distance -> Sink Node bloadcast the syn information  get the avaerge data from the 4 times
def calcDistance(argv):
    getDistance = []
    for item in argv:        
        getDistance.append(round((item * 31.25 / 10**7 )* (331.5 + 0.607 * globalVar.tempurateData) ,2))                  #caculte the average distace in the temputare of 5.0 C  the Unit is cm
    #print temp
    #for i in temp:
    #    print i
    goodDataLength = len(getDistance)
    #print 'The get data length is ' + str(goodDataLength)
    totalData = 0
    delList = []                                    #del the error data in list 
    for temp in getDistance:
        if temp > 0 and temp <= 1500:              #ingnore the wrong data of below 0mm or bigger 15m
            totalData += temp
            print str(temp) + ' ',
        else:
            #print ' Being one error data! ' + str(temp),                  
            goodDataLength = goodDataLength -1
            delList.append(temp)                                   #store the error data
    for n in delList:                                      
        getDistance.remove(n)                       #delete the wrong data        
    #print 'The totalData is ' + str(totalData)
    if goodDataLength == 0:
        print 'These datas are all error data!'
        finalDistance = 0
    else: 
        aveData = round(totalData / goodDataLength,2)
        #print aveData
        secTotalData = 0
        for i in getDistance:                       #delete the wrong data which have more than 10cm differences than the averages.
            if abs(i-aveData) <= 10:
                secTotalData += i 
            else:
                #print 'delete the wrong data!',
                goodDataLength = goodDataLength -1
        if goodDataLength == 0:
            print 'These data are not stable!'
            finalDistance = 0
        else:
            secAveData = round(secTotalData / goodDataLength,2)
            print
            print 'Origan Average Data: ' + str(secAveData) + 'cm.',
            finalDistance = round(secAveData * 1 - 10,2)
            #finalDistance = round(secAveData * 1.011 - 2.1866,2)
            print 'Modify Data: ' + str(finalDistance) + 'cm.'
            
    print
    print
    return finalDistance
'''

        
