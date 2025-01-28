from enum import Enum
import importlib.resources
import json
import serial
from threading import Lock

class OperatingMode(Enum):
    PASSIVE = 0
    VELOCITY = 1
    POSITION = 2
    GEAR_POSITION = 3
    ANALOGUE_TORQUE = 4
    ANALOGUE_VELOCITY = 5
    ANALOGUE_VELOCITY_GEAR = 6
    MANUAL_CURRENT = 7
    STEP_RESPONSE_TEST = 8
    INTERNAL_TEST = 9
    BRAKE = 10
    STOP = 11
    TORQUE_BASED_ZERO_SEARCH = 12
    FORWARD_ONLY_ZERO_SEARCH = 13
    FORWARD_BACKWARD_ZERO_SEARCH = 14
    SAFE_MODE = 15
    ANALOGUE_VELOCITY_WITH_DEAD_BAND = 16
    VELOCITY_LIMITED_ANALOGUE_TORQUE = 17
    ANALOGUE_GEAR = 18
    COIL = 19
    ANALOGUE_BI_POSITION = 20
    ANALOGUE_TO_POSITION = 21
    INTERNAL_TEST_2 = 22
    INTERNAL_TEST_3 = 23
    GEAR_FOLLOW = 24
    IHOME = 25

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

        # Load the registers from the json file in resources
        with importlib.resources.open_text("py_mac", "registers.json") as file:
            self.registers = json.load(file)

        # Open the serial port
        try:
            self.serial = serial.Serial(self.serial_path, self.baud, timeout=0.1)
        except serial.SerialException:
            raise ValueError("Invalid serial path")

        self.config_lock = Lock()
        self.serial_lock = Lock()
        self.status_lock = Lock()

        # Update the object tp match the motor
        self.config = {}
        self.status = {}
        self.refresh_config()
        self.refresh_status()

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

        with self.serial_lock:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.serial.write(bytes(message))
            response = self.serial.read(19)

        expected      = [0x52, 0x52, 0x52, 0x00, 0xff, reg_num, 0xff ^ reg_num, 0x04, 0xfb,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xaa, 0xaa]
        frame_mask    = [0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff]
        address_mask  = [0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        register_mask = [0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        if not all([(r & fm) == (e & fm) for r, fm, e in zip(response, frame_mask, expected)]):
            raise ValueError("Invalid frame")
        if not all([(r & am) == (e & am) for r, am, e in zip(response, address_mask, expected)]):
            # TODO: if invalid address, try to read again
            raise ValueError("Invalid address")
        if not all([(r & rm) == (e & rm) for r, rm, e in zip(response, register_mask, expected)]):
            raise ValueError("Invalid register")

        # split the response between the data and the complement
        data_full = response[9:17]
        data = data_full[0::2]
        complement = data_full[1::2]
        complement_computed = bytes([0xff ^b for b in data])
        if complement != complement_computed:
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

        with self.serial_lock:
            self.serial.write(bytes(message))
            response = self.serial.read(3)

        expected = bytes([0x11, 0x11, 0x11])
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
        
        register_name = [k for k, v in self.registers.items() if v["nb"] == register][0]

        if isinstance(data, int):
            data = data.to_bytes(self.registers[register_name]["size"], byteorder="little")
        if len(data) != self.registers[register_name]["size"]:
            raise ValueError("Invalid data size")

        self.write(register, data)

    def get_mode(self)->OperatingMode:
        """
        Get the operating mode of the motor.
        :return: current operating mode
        """
        with self.status_lock:
            mode = OperatingMode(int.from_bytes(self.read_register("MODE_REG"), byteorder="little"))

            # update the object to match the motor
            self.status["operating mode"] = mode

        return mode

    def set_mode(self, mode: str|bytes|int|OperatingMode)->None:
        """
        Set the operating mode of the motor.
        :param mode: name, id, bytes or OperatingMode object
        """
        if isinstance(mode, str):
            mode = OperatingMode[mode.upper()]
        if isinstance(mode, bytes):
            mode = OperatingMode(int.from_bytes(mode, byteorder="little"))
        if isinstance(mode, int):
            mode = OperatingMode(mode)
        if not isinstance(mode, OperatingMode):
            raise ValueError("Invalid mode")

        with self.status_lock:
            if mode == self.status["operating mode"]:
                return

            with self.config_lock:
                if mode == OperatingMode.POSITION and (self.config["min position"] != 0 or self.config["max position"] != 0):
                    position = self.get_position()
                    if position < self.config["min position"] or position > self.config["max position"]:
                        raise ValueError("Position out of bounds")

                self.write_register("MODE_REG", mode.value)
                self.status["operating mode"] = mode

    def get_position(self)->int:
        """
        Get the current position of the motor.
        :return: current position
        """
        with self.status_lock:
            pos = int.from_bytes(self.read_register("P_IST"), byteorder="little", signed=True)
            self.status["actual position"] = pos
        return pos

    def set_target_position(self, position: int, ignore_mode:bool=False)->None:
        """
        Set the target position of the motor.
        :param position: target position
        :param ignore_mode: if True, the function will not check if the motor is in position mode
        """
        with self.config_lock:
            # check that the motor is in position mode
            if not ignore_mode and self.status["operating mode"] != OperatingMode.POSITION:
                raise ValueError("Motor must be in position mode")

            with self.status_lock:
                self.write_register("P_SOLL", position)
                self.status["target position"] = position

    def refresh_config(self)->None:
        """"
        Refresh the status of the motor.
        """
        with self.config_lock:
            self.config = {
                "max velocity": int.from_bytes(self.read_register("V_SOLL"), byteorder="little"),
                "max acceleration": int.from_bytes(self.read_register("A_SOLL"), byteorder="little"),
                "max_torque": int.from_bytes(self.read_register("T_SOLL"), byteorder="little"),
                "gear ratio nomitor": int.from_bytes(self.read_register("GEARF1"), byteorder="little"),
                "gear ratio denominator": int.from_bytes(self.read_register("GEARF2"), byteorder="little"),
                "max winding energy": int.from_bytes(self.read_register("I2TLIM"), byteorder="little"),
                "max dumped energy": int.from_bytes(self.read_register("UITLIM"), byteorder="little"),
                "max regulation error": int.from_bytes(self.read_register("FLWERRMAX"), byteorder="little"),
                "max movement error": int.from_bytes(self.read_register("FNCERRMAX"), byteorder="little"),
                "min position": int.from_bytes(self.read_register("MIN_P_IST"), byteorder="little", signed=True),
                "max position": int.from_bytes(self.read_register("MAX_P_IST"), byteorder="little", signed=True),
                "emergency deceleration": int.from_bytes(self.read_register("ACC_EMERG"), byteorder="little"),
                "starting mode": OperatingMode(int.from_bytes(self.read_register("STARTMODE"), byteorder="little")),
                "home position": int.from_bytes(self.read_register("P_HOME"), byteorder="little", signed=True),
                "homing velocity": int.from_bytes(self.read_register("V_HOME"), byteorder="little"),
                "homing mode": int.from_bytes(self.read_register("HOMEMODE"), byteorder="little"),
                "min supply voltage": int.from_bytes(self.read_register("MIN_U_SUP"), byteorder="little"),
                "motor type": int.from_bytes(self.read_register("MOTORTYPE"), byteorder="little"),
                "serial number": int.from_bytes(self.read_register("SERIALNUMBER"), byteorder="little"),
                "address": int.from_bytes(self.read_register("MYADDR"), byteorder="little"),
                "hardware version": int.from_bytes(self.read_register("HWVERSION"), byteorder="little"),
            }

    def refresh_status(self)->None:
        """"
        Refresh the status of the motor.
        """
        with self.status_lock:
            self.status = {
                "operating mode": OperatingMode(int.from_bytes(self.read_register("MODE_REG"), byteorder="little")),
                "target position": int.from_bytes(self.read_register("P_SOLL"), byteorder="little"),
                "actual position": int.from_bytes(self.read_register("P_IST"), byteorder="little", signed=True),
                "actual velocity": int.from_bytes(self.read_register("V_IST"), byteorder="little", signed=True),
                "load factor": int.from_bytes(self.read_register("KVOUT"), byteorder="little"),
                "winding energy": int.from_bytes(self.read_register("I2T"), byteorder="little"),
                "dumped energy": int.from_bytes(self.read_register("UIT"), byteorder="little"),
                "regulation error": int.from_bytes(self.read_register("FLWERR"), byteorder="little"),
                "movement error": int.from_bytes(self.read_register("FNCERR"), byteorder="little"),
                "error": int.from_bytes(self.read_register("ERR_STAT"), byteorder="little"),
                "control": int.from_bytes(self.read_register("CNTRL_BITS"), byteorder="little"),
                "supply voltage": int.from_bytes(self.read_register("U_SUPPLY"), byteorder="little"),
            }
