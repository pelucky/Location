# Filename: CalculateDistance.py
# Description: This is file is used for calculating the different distance of the time by serial
# made by pel 2014.10.31 modify in 2015.3.18

import serial
import time
import datetime

global MOBILENUMBER
global REFERNUMBER
global CALCTIMES
global temperature_data


def get_serial():
    store_result = []
    ser = serial.Serial(port='COM5', baudrate=19200, bytesize=8, parity='N',
                        stopbits=1, timeout=None, xonxoff=0, rtscts=0)
    print ser
    get_data_times = 0
    last_sequence_no = 0
    while get_data_times < CALCTIMES:
        line = ser.readline()
        msg_type = str(get_no(line, 0))                   # check the first data -- the message type
        if msg_type == '1':
	    # make sure has get right length data change the methand to avoid '0A'in the data
            if len(line) % 16 != 0:                       
                # get the left data cos the data may be 0x0a .the same with \n
                left_data_length = ser.inWaiting()
                left_data = ser.read(left_data_length)
                line = line + left_data
            # deal with the data
            sequence_no = get_no(line, 1)
            if last_sequence_no != sequence_no:
                get_data_times += 1
                last_sequence_no = sequence_no
            mob_id = get_no(line, 2)
            # print str(sequence_no) + ' , MobileID is ' + str(mob_id)
            if mob_id == MOBILENUMBER:
                data_line = line[3+3*(REFERNUMBER-1):3+3*REFERNUMBER]
                data_result = hex2int(data_line)
                each_distance = calc_distance(data_result)
                store_result.append(each_distance)
                print "The distance of Mobile %s and Refer %s is : %s" % (MOBILENUMBER, REFERNUMBER, each_distance)

        if msg_type == '2':
            if len(line) % 4 != 0:                      # make sure has get right length data
                # get the left data cos the data may be 0x0a .the same with \n
                left_data_length = ser.inWaiting()            
                left_data = ser.read(left_data_length)
                line = line + left_data
            # hex_show(line)
            temperature_data = get_temperature(line)
    ser.close()
    return store_result


# the second way to calculate the distance -> Sink Node broadcast the syn information
# get the average data from the 4 times
def calc_distance(argv):
    # the node did not get the US data
    if argv == 0:
        final_distance = 0
        print 'It does not get US signal!'
    # 0xFFFFFF ,it means the error data!
    elif argv >= 16777215:
        final_distance = 0
        print 'It calculate the error data!'
    else:
        # globalVar.temperature_data calculate the average distance in the temperature of 5.0 C  the Unit is cm
        distance_data = round((argv * 31.25 / 10**7) * (331.5 + 0.607 * temperature_data), 2)
        final_distance = distance_data
    return final_distance


def get_temperature(argv):
    hvol_high = ord(argv[1])
    hvol_low = ord(argv[2])
    hhex_high = '%02x' % hvol_high
    hhex_low = '%02x' % hvol_low
    hhex = hhex_high + hhex_low

    if hhex == 'ffff':
        temperature = temperature_data
        print 'The temperature is wrong! '
    else:
        temperature = (int('0x'+hhex, 16))/100.0
        if temperature < 0 or temperature > 40:
            print 'The temperature is wrong! ' + str(temperature) + ' Cels. '
            temperature = temperature_data
        else:
            print 'The temperature is : ' + str(temperature) + ' Cels. '
    return temperature


def hex_show(argv):
    result = ''
    h_len = len(argv)
    # print h_len
    for i in xrange(h_len):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex+' '
    print result


def hex2int(argv):
    result = ''
    for i in range(0, 3):
        hvol = ord(argv[i])
        hhex = '%02x' % hvol
        result += hhex
    # print '0x'+result
    int_result = int('0x'+result, 16)       # 16 to 10
    # print int_result
    return int_result


def cal_average(argv, distance):
    total_distance = 0
    temp_list = []
    for item in argv:
        # Don't calculate the wrong data.
        if item != 0:
            if abs(item - float(distance)) <= 20:
                temp_list.append(item)
    for item in temp_list:
        total_distance += float(item)
    if len(temp_list) == 0:
        average_distance = 0
    else:
        average_distance = round(total_distance / len(temp_list), 2)
    print "There are %i useful values." % (len(temp_list))
    print "It's average is %s cm. " % (str(average_distance))
    return average_distance, temp_list


def get_no(argv, bit):
    hvol = ord(argv[bit])                 # get node number -- the Third byte;  get seq Number -- the second byte
    node_no = '%02x' % hvol
    int_node_no = int('0x'+node_no, 16)       # 16 to 10
    return int_node_no


def output_file(string, filename):
    file_handler = file(filename, 'a+')
    file_handler.write(string+' ')
    file_handler.close()


def output_file_list(temp_list, filename):
    file_handler = file(filename, 'a+')
    for item in temp_list:
        file_handler.write(str(item)+' ')
    file_handler.close()


if __name__ == "__main__":
    again = 0
    # define the global values
    MOBILENUMBER = 1
    REFERNUMBER = 1
    CALCTIMES = 3000
    temperature_data = 25
    while again != 'q':
        # must be the real Distance ,cos i ignore the wrong data that bigger than 20cm
        print "Please input the distance of the sender and the receiver:(cm)"
        real_distance = raw_input()
        this_time = time.time()
        print "It begins at %s." % (str(datetime.datetime.today()))
        output_file(real_distance, "real_distance.txt")

        intData = []
        intData = get_serial()
        finish_time = time.time()
        print "It finishes at %s." % (str(datetime.datetime.today()))
        spend_time = int(finish_time * 1000) - int(this_time * 1000)
        print "It total used %s ms." % (str(spend_time))

        (aveDistance, good_data) = cal_average(intData, real_distance)
        output_file_list(good_data, real_distance+'.txt')
        output_file(str(aveDistance), "calc_distance.txt")

        print 'Again?'
        print "Press q for exit, press any key to again."
        again = raw_input()
        print '-----------------------------------'
    
    print 'Hello world...END'
