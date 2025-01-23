import uuid
import serial

baudrate = 19200
port = '/dev/ttyUSB0'
ser = serial.Serial(port, baudrate, timeout=5)
motor_id = 254 # Broadcast ID

#  Frame to get the motor firmware version
version_register = 0x01
frame = [0x50, 0x50, 0x50,                          # Read instruction
         motor_id, 0xff ^ motor_id,                 # Address
         version_register, 0xff ^ version_register, # Register
         0xaa, 0xaa]                                # End of frame

frame_bytes = bytes(frame)

dump = ser.read(ser.in_waiting)
ser.write(frame_bytes)

# the response is between 19 and 27 bytes long
response = ser.read(27)


# generate file name from time stamp AAAA-MM-DD_HH:MM:SS
file_name = str(uuid.uuid1()) + '.bin'

# save the response to a new file
with open(file_name, 'wb') as f:
    f.write(response)

print(f'File {file_name} saved')