import serial

class MAC50Motor:
    def __init__(self, serial_path: str, address: int):
        if address < 0 or address > 255:
            raise ValueError("Invalid address")

        self.serial_path = serial_path
        self.baud = 19200 # bits per second
        self.address = address # 0xff for broadcast, 0x00 for master

        try:
            self.serial = serial.Serial(self.serial_path, self.baud, timeout=0.1)
        except serial.SerialException:
            raise ValueError("Invalid serial path")

    def read(self, reg_num: int):
        if reg_num < 0 or reg_num > 255:
            raise ValueError("Invalid register number")

        message = [0x50, 0x50, 0x50, self.address, 0xff ^ self.address, reg_num, 0xff ^ reg_num, 0xaa, 0xaa]
        self.serial.write(bytes(message))
        response = self.serial.read(19)

        expected      = [0x52, 0x52, 0x52, 0x00, 0xff, reg_num, 0xff ^ reg_num, 0x04, 0xfb,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa]
        frame_mask    = [0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff]
        address_mask  = [0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        register_mask = [0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        if all([(r & fm) == (e & fm) for r, fm, e in zip(response, frame_mask, expected)]):
            raise ValueError("Invalid frame")
        if all([(r & am) == (e & am) for r, am, e in zip(response, address_mask, expected)]):
            # TODO: if invalid address, try to read again
            raise ValueError("Invalid address")
        if all([(r & rm) == (e & rm) for r, rm, e in zip(response, register_mask, expected)]):
            raise ValueError("Invalid register")

        # split the response between the data and the complement
        data_full = response[9:17]
        data = data_full[0::2]
        complement = data_full[1::2]
        if [0xff ^ b for b in data] != complement:
            # TODO: if invalid complement, try to read again
            raise ValueError("Invalid complement")

        return data

    def write(self, reg_num: int, data: bytes):
        if reg_num < 0 or reg_num > 255:
            raise ValueError("Invalid register number")
        if len(data) % 2 != 0:
            raise ValueError("Number of bytes must be even")
        if len(data) > 255:
            raise ValueError("Number of bytes must be less than 256")

        complement = [0xff ^ b for b in data]
        data_with_complement = [b for pair in zip(data, complement) for b in pair]
        message = [0x52, 0x52, 0x52, self.address, 0xff ^ self.address, reg_num, 0xff ^ reg_num, len(data), 0xff ^ len(data)] + data_with_complement + [0xaa, 0xaa]

        self.serial.write(bytes(message))
        response = self.serial.read(3)

        expected = [0x11, 0x11, 0x11]
        if response != expected:
            raise ValueError("Invalid response")

