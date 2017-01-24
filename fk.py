

import struct
import time
import serial

ser = serial.Serial(
      port='COM3',
      baudrate=115200,
      )

ser.isOpen()

commandFlag = True
while(commandFlag) :
   ba = raw_input("base angle: ")
   la = raw_input("lower angle: ")
   ua = raw_input("upper angle: ")

   if input == 'exit':
      ser.close()
      exit()
   else:
      print "Sending angles..\r\n"
      ser.write(struct.pack('f',float(ba)))
      ser.write(struct.pack('f',float(ua)))
      ser.write(struct.pack('f',float(la)))

      out = ''
      time.sleep(1)
      while ser.inWaiting() > 0 :
          out += ser.read(1)

      if out !='':
          print ">>"+out


   s = raw_input('quit?')
   if (s == 'y'):
     commandFlag = False



