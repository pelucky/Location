# Filename: CaculateDistance.py
# Discription: This's file is used for caculating the differnet distance of the time by serial
# made by pel 2014.10.31

import serial

def getSerial():
    storeResult = []
    ser = serial.Serial(port='COM4', baudrate=19200, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=0, rtscts=0)
    print ser
    for i in range(0,300):                      #cacultae 100 times
        line = ser.read(6)                      #make sure the receive data is 6bytes per time
        print str(i) + ' ',                     #Do not change the line
        hexShow(line)
        storeResult.append(hex2int(line))
        #print storeResult[i]
    ser.close()
    return storeResult
        
def hexShow(argv):
    result = ''
    hLen = len(argv)
    #print hLen
    for i in xrange(hLen):
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex+' '
    print result

def hex2int(argv):
    result = ''
    for i in range(3,6):
        hvol = ord(argv[i])
        hhex = '%02x'%hvol
        result += hhex
    #print '0x'+result
    intResult = int('0x'+result,16)       #16 to 10
    #print intResult
    return intResult

def calAverage(argv):
    totalDistance = 0
    for item in argv:
         totalDistance += float(item)
    if len(argv) == 0:
        averageDistance = 0
    else:
        averageDistance = round(totalDistance / len(argv), 2)
    print averageDistance    
    return averageDistance

def storeData(argv):
    distanceData = []
    for item in argv:
        #round(i,n) float point after n bit

        #the first way to caculate the distance. -> US arrival first, RF arrival second!
        #temp = round((240 + 17.6 - item * 31.25 / 10**6 )* 346.675 ,2)                  #caculte the average distace in the temputare of 25.0 C  

        #the second way to caculate the distance -> Sink Node bloadcast the syn information
        temp = round((item * 31.25 / 10**7 )* 346.675 ,2)                  #caculte the average distace in the temputare of 25.0 C  the Unit is cm
        #print temp
        if temp > 0 and temp <= 1000:              #ingnore the wrong data of below 0mm or bigger 10m
            if temp > int(realDistance) - 50  and temp < int(realDistance) + 50:                  #ingnore the diferrence bigger than real Distance of 50cm
                distanceData.append(str(temp))          
    return distanceData

def outputFile(string, filename):
    fileHandler = file(filename, 'a+')
    fileHandler.write(string+' ')
    fileHandler.close()

def outputFileList(List, filename):
    fileHandler = file(filename, 'a+')
    for item in List:
        fileHandler.write(item+' ')
    fileHandler.close()

if __name__ == "__main__":
    again = 0
    while again != 'q':
        print "Please input the distance of the sender and the receiver:(cm)"    #must be the real Distance ,cos i ignore the wrong data 
        realDistance = raw_input()
        outputFile(realDistance, "realDistance.txt")
        
        intData = getSerial()
        perDistanceData = storeData(intData)
        outputFileList(perDistanceData,realDistance+'.txt')
        
        calcDistance = str(calAverage(perDistanceData))
        
        #print 'realDistance: ' + str(realDistance)
        #print 'calcDistance: ' + str(calcDistance)

        outputFile(calcDistance, "calcDistance.txt")

        print 'Again?'
        print "Press q for exit, press any key to again."
        again = raw_input()
        print '-----------------------------------'
    
    print 'Hello world...END'
