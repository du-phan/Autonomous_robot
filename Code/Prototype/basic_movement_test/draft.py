from pydynamixel import dynamixel
import serial, sys, os, time



serial_port = '/dev/tty.usbmodem1421'

servo_id = 1
target_position = 1023 # (range: 0 to 1023)

# If this is the first time the robot was powered on,
# you'll need to read and set the current position.
# (See the documentation above.)
first_move = True


try:

    ser = dynamixel.get_serial_for_url(serial_port)

    print ser

    if first_move == True:
        dynamixel.init(ser, servo_id)


    dynamixel.set_position(ser, servo_id, target_position)
    dynamixel.send_action_packet(ser)

    print('Success!')

except Exception as e:
    print('Unable to move to desired position.')
    print(e)


