import importlib.resources
import json
import serial

class MAC50Motor:
    def __init__(self, serial_path: str, address: int):
        """
        Create a new MAC50Motor object.

        :param serial_path: Path to the serial port
        :param address: Address of the motor
        """
        if address < 0 or address > 255:
            raise ValueError("Invalid address")

        self.serial_path = serial_path
        self.baud = 19200 # bits per second
        self.address = address # 0xff for broadcast, 0x00 for master

        self.operating_modes = {
            "passive": 0,
            "velocity": 1,
            "position": 2,
            "gear position": 3,
            "analogue torque": 4,
            "analogue velocity": 5,
            "analogue velocity gear": 6,
            "manual current": 7,
            "step response test": 8,
            "internal test": 9,
            "brake": 10,
            "stop": 11,
            "torque based zero search": 12,
            "forward/only zero search": 13,
            "forward+backward zero search": 14,
            "safe mode": 15,
            "analogue velocity with dead band": 16,
            "velocity limited analogue torque": 17,
            "analogue gear": 18,
            "coil": 19,
            "analogue bi-position": 20,
            "analogue to position": 21,
            "internal test 2": 22,
            "internal test 3": 23,
            "gear follow": 24,
            "IHOME": 25,
        }

        # Load the registers from the json file in resources
        with importlib.resources.open_text("py_mac", "registers.json") as file:
            self.registers = json.load(file)

        # Open the serial port
        try:
            self.serial = serial.Serial(self.serial_path, self.baud, timeout=0.1)
        except serial.SerialException:
            raise ValueError("Invalid serial path")

        # Update the object tp match the motor
        self.status = {
            "operating mode": self.get_mode_name(),
            "max velocity": int.from_bytes(self.read_register("V_SOLL"), byteorder="little"),
            "max acceleration": int.from_bytes(self.read_register("A_SOLL"), byteorder="little"),
            "max_torque": int.from_bytes(self.read_register("T_SOLL"), byteorder="little"),
            "gear ratio nomitor": int.from_bytes(self.read_register("GEARF1"), byteorder="little"),
            "gear ratio denominator": int.from_bytes(self.read_register("GEARF2"), byteorder="little"),
            "max winding energy": int.from_bytes(self.read_register("I2TLIM"), byteorder="little"),
            "max dumped energy": int.from_bytes(self.read_register("UITLIM"), byteorder="little"),
            "max position error": int.from_bytes(self.read_register("FLWERRMAX"), byteorder="little"),
            "max velocity error": int.from_bytes(self.read_register("FNCERRMAX"), byteorder="little"),
            "min position": int.from_bytes(self.read_register("MIN_P_IST"), byteorder="little", signed=True),
            "max position": int.from_bytes(self.read_register("MAX_P_IST"), byteorder="little", signed=True),
            "emergency deceleration": int.from_bytes(self.read_register("ACC_EMERG"), byteorder="little"),
            "starting mode": int.from_bytes(self.read_register("STARTMODE"), byteorder="little"),
            "home position": int.from_bytes(self.read_register("P_HOME"), byteorder="little", signed=True),
            "homing velocity": int.from_bytes(self.read_register("V_HOME"), byteorder="little"),
            "homing mode": self.read_register("HOME_MODE")[1],
            "min supply voltage": int.from_bytes(self.read_register("MIN_U_SUP"), byteorder="little"),
            "motor type": self.read_register("MOTORTYPE")[1],
            "serial number": int.from_bytes(self.read_register("SERIAL"), byteorder="little"),
            "address": int.from_bytes(self.read_register("MYADDR"), byteorder="little"),
            "hardware version": int.from_bytes(self.read_register("HWVERSION"), byteorder="little"),
        }

    def __del__(self):
        self.serial.close()

    def read(self, reg_num: int)->bytes:
        """
        Read data from the motor (low-level function).

        :param reg_num: number of the first register to be read
        :return: data returned by the motor, little-endian (least significant byte first), typically 8 bytes
        """
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

    def write(self, reg_num: int, data: bytes)->None:
        """
        Write data to a register on the motor (low-level function).
        :param reg_num: number of the register
        :param data: data to be written, little-endian (least significant byte first)
        """
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

    def read_register(self, register: str|int)->bytes:
        """
        Read a register from the motor (high-level function).
        :param register: Name or number of the register
        :return: data returned by the motor, little-endian (least significant byte first), cropped to the size of the register
        """
        if isinstance(register, str):
            if register not in self.registers:
                raise ValueError("Invalid register name")
        if isinstance(register, int):
            if register < 0 or register > 255:
                raise ValueError("Invalid register number")
            register = [k for k, v in self.registers.items() if v["nb"] == register][0]

        return self.read(self.registers[register]["nb"])[0:self.registers[register]["size"]]

    def write_register(self, register: str|int, data: bytes|int)->None:
        """
        Write a register to the motor (high-level function).
        :param register: Name or number of the register
        :param data: data to be written. If int, it will be converted to bytes to match the size of the register. If bytes, it must have the same size as the register.
        """
        if isinstance(register, str):
            if register not in self.registers:
                raise ValueError("Invalid register name")
            register = self.registers[register]["nb"]
        if register < 0 or register > 255:
            raise ValueError("Invalid register number")

        if isinstance(data, int):
            data = data.to_bytes(self.registers[register]["size"], byteorder="little")
        if len(data) != self.registers[register]["size"]:
            raise ValueError("Invalid data size")

        self.write(register, data)

    def get_mode_id(self)->int:
        """
        Get the operating mode of the motor.
        :return: id of the current operating mode
        """
        mode_id = int.from_bytes(self.read_register("MODE_REG"), byteorder="little")

        # update the object to match the motor
        self.status["operating mode"] = [k for k, v in self.operating_modes.items() if v == mode_id][0]

        return mode_id

    def get_mode_name(self)->str:
        """
        Get the name of the operating mode of the motor.
        :return: name of the current operating mode
        """
        self.get_mode_id()

        # status["operating mode"] is already updated, so we can return it
        return self.status["operating mode"]

    def set_mode(self, mode: str|bytes|int)->None:
        """
        Set the operating mode of the motor.
        :param mode: name, id or bytes of the mode
        """
        if isinstance(mode, str):
            mode = self.operating_modes[mode]
        if isinstance(mode, bytes):
            mode = int.from_bytes(mode, byteorder="little")
        if mode < 0 or mode > 255:
            raise ValueError("Invalid mode")

        self.write_register("MODE_REG", mode)

        # update the object to match the motor
        self.status["operating mode"] = [k for k, v in self.operating_modes.items() if v == mode][0]

    def get_position(self)->int:
        """
        Get the current position of the motor.
        :return: current position
        """
        return int.from_bytes(self.read_register("P_IST"), byteorder="little", signed=True)

    def set_target_position(self, position: int, ignore_mode:bool=False)->None:
        """
        Set the target position of the motor.
        :param position: target position
        :param ignore_mode: if True, the function will not check if the motor is in position mode
        """
        # check that the motor is in position mode
        if not ignore_mode and self.status["operating mode"] != "position":
            raise ValueError("Motor must be in position mode")

        self.write_register("P_SOLL", position)
