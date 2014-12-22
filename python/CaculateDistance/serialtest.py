#coding=gb18030


import sys,threading,time;
import serial;
import binascii,encodings;
import re;
import socket;
from struct import *;

class ComThread:
    def __init__(self, Port=3):
        self.l_serial = None;
        self.alive = False;
        self.waitEnd = None;
        self.port = Port;


    def waiting(self):
        if not self.waitEnd is None:
            self.waitEnd.wait();


    def SetStopEvent(self):
        if not self.waitEnd is None:
            self.waitEnd.set();
        self.alive = False;
        self.stop();


    def start(self):
        self.l_serial = serial.Serial();
        self.l_serial.port = self.port;
        self.l_serial.baudrate = 19200;
        self.l_serial.timeout = 2;
        self.l_serial.open();

        if self.l_serial.isOpen():
            self.waitEnd = threading.Event();
            self.alive = True;
            self.thread_read = None;
            self.thread_read = threading.Thread(target=self.FirstReader);
            self.thread_read.setDaemon(1);
            self.thread_read.start();
            return True;
        else:
            return False;


    def FirstReader(self):
        while self.alive:
            # 接收间隔
            time.sleep(0.1);
            try:
                data = '';
                n = self.l_serial.inWaiting();
                if n:
                    data = data + self.l_serial.read(n);
                    for l in xrange(len(data)):
                        print '%02X' % ord(data[l]),


                    # 发送数据
                    #snddata = '';
                    #snddata += chr(97)
                    #tt = 0x12345678;
                    #snddata += pack('i', tt)
                    #snddata += chr(0x64)
                    #self.l_serial.write(snddata);

                # 判断结束
                #if len(data) > 0 and ord(data[len(data)-1])==0x45:
                    #pass;
                    #break;
               
            except Exception, ex:
                print str(ex);


        self.waitEnd.set();
        self.alive = False;


    def stop(self):
        self.alive = False;
        self.thread_read.join();
        if self.l_serial.isOpen():
            self.l_serial.close();



#测试用部分
if __name__ == '__main__':
    rt = ComThread();
    try:
        if rt.start():
            rt.waiting();
            rt.stop();
        else:
            pass;            
    except Exception,se:
        print str(se);

   

    if rt.alive:
        rt.stop();


    print '';
    print 'End OK .';
    del rt;
