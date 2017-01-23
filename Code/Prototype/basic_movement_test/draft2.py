# import serial, sys, os, time
# import packets


# port = '/dev/tty.usbmodem1421'

# try:
#     s = serial.Serial(port, baudrate=115200)
# except:
#     exit('! Unable to open serial port %s' % port)

# print('* Resetting the board')
# s.setRTS(True)
# s.setDTR(False)
# time.sleep(0.1)
# s.setRTS(False)
# s.write('CM9X')
# s.close()
# time.sleep(1.0);

# print('* Connecting...')
# s = serial.Serial(port, baudrate=115200)
# s.write('AT&LD')
# print('* Download signal transmitted, waiting...')

# p = packets.get_read_position_packet(1)
# print p

from pyxl320 import xl320
from pyxl320 import ServoSerial, Packet, utils

serial = ServoSerial('/dev/tty.usbmodem1421')  # tell it what port you want to use
serial.open()

pkt = Packet.makeServoPacket(1, 158.6)  # move servo 1 to 158.6 degrees

err_num, err_str = serial.sendPkt(pkt)  # send packet to servo


# if err_num:
#     raise Exception('servo error: {}'.format(err_str))

# pkt = packet.makeLEDPacket(1, pyxl320.XL320_LED_GREEN)
# serial.sendPkt(pkt)
