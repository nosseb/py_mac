from py_mac.MAC50Motor import *

my_motor = MAC50Motor("/dev/ttyUSB0", 255);

print(f'current mode: {my_motor.get_mode()}')
print(f'current position: {my_motor.get_position()}')

my_motor.set_mode(OperatingMode.POSITION)
print(f'new mode: {my_motor.get_mode()}')
my_motor.set_target_position(10000)
print(f'new position: {my_motor.get_position()}')
