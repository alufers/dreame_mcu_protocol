import struct
from io import BytesIO
from .crc_util import CRC_GetModbus16
import typing
from bitstring import BitArray

PACKET_START = b"<"
PACKET_END = b">"
PACKET_ESCAPE = b"?"


def read_packet(stream: BytesIO):
    """
    Reads a binary packet enclosed in < and >, with ? escaping.
    """
    packet = b""
    raw_packet = b""
    packet_started = False
    while True:
        c = stream.read(1)
        raw_packet += c
        # print(c)
        if c == PACKET_START:
            packet = b""
            packet_started = True
        elif c == PACKET_END and packet_started:
            return packet
        elif c == PACKET_ESCAPE and packet_started:
            c = stream.read(1)
            raw_packet += c
            packet += c
        elif packet_started:
            # append byte
            packet += c


def parse_packet(data: bytes) -> typing.Tuple[int, bytes]:
    """
    Parse packet from bytes, returns (type, payload)
    """
    if len(data) < 4:
        raise Exception(
            "Packet is too short, must be at least 4 bytes, got {}".format(len(data))
        )
    length = data[0]
    type = data[1]

    payload = data[2:-2]
    # cut to length
    payload = payload[:length]
    crc_received = data[-1] + (data[-2] << 8)
    crc_calculated = CRC_GetModbus16(data[:-2])
    if crc_received != crc_calculated:
        raise Exception(
            "CRC mismatch, received {:04x}, calculated {:04x}, hex: {}".format(
                crc_received, crc_calculated, data.hex()
            )
        )

    return (type, payload)


def unpack_checked(fmt: str, data: bytes) -> typing.Tuple:
    """
    Unpack data with given format, raise a nice exception if length does not match
    """
    if len(data) != struct.calcsize(fmt):
        raise Exception(
            "Invalid data length, expected {}, got {}".format(
                struct.calcsize(fmt), len(data)
            )
        )
    return struct.unpack(fmt, data)


def reverse_bits(data: bytes) -> bytes:
    """
    Reverse bits in each byte
    """
    return bytes(
        [
            (x & 0b10000000) >> 7
            | (x & 0b01000000) >> 5
            | (x & 0b00100000) >> 3
            | (x & 0b00010000) >> 1
            | (x & 0b00001000) << 1
            | (x & 0b00000100) << 3
            | (x & 0b00000010) << 5
            | (x & 0b00000001) << 7
            for x in data
        ]
    )


class Status20ms:
    """
    From MCU, sent every 20ms
    """

    timestamp_us: int  # int32
    x: int  # int32
    y: int  # int32
    yaw: float  # int32
    yaw_integral: int  # int32
    leftVel: int  # int16 # left wheel velocity
    rightVel: int  # int16 # right wheel velocity 
    edgeDis: int  # int16
    roller_motor_current: int
    sidebrush_motor_current: int

    def __init__(self, data):
        fmt = "<Iiihhhhhhh"

        # self.RollCurrent, self.SideCurrent = (0, 0)
        (
            self.timestamp_us,
            self.x,
            self.y,
            self.yaw,
            self.yaw_integral,
            self.leftVel,
            self.rightVel,
            self.edgeDis,
            self.roller_motor_current,
            self.sidebrush_motor_current,
        ) = unpack_checked(fmt, data)
        self.yaw /= 100.0

    def __repr__(self):
        return f"timestamp={self.timestamp_us}, x={self.x/10.0}mm, y={self.y/10.0}mm, yaw={self.yaw}°, yawIntegral={self.yaw_integral}, leftVel={self.leftVel}, rightVel={self.rightVel}, edgeDis={self.edgeDis}mm, RollCurrent={self.roller_motor_current}, SideCurrent={self.sidebrush_motor_current}"


class Status10ms:
    timestamp_us: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    accel_x: int
    accel_y: int
    accel_z: int
    leftDis: int
    rightDis: int

    def __init__(self, data):
        (
            self.timestamp_us,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.leftDis,
            self.rightDis,
        ) = unpack_checked("<Ihhhhhhbb", data)
        self.gyro_x /= 100.0
        self.gyro_y /= 100.0
        self.gyro_z /= 100.0
        self.accel_x /= 1000.0
        self.accel_y /= 1000.0
        self.accel_z /= 1000.0

    def __repr__(self):
        return "timestamp={:d}, gyro_x={: 7.2f}°/s, gyro_y={: 7.2f}°/s, gyro_z={: 7.2f}°/s, accel_x={: 7.2f}g, accel_y={: 7.2f}g, accel_z={: 7.2f}g, leftDis={:d}mm, rightDis={:d}mm".format(
            self.timestamp_us,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.leftDis,
            self.rightDis,
        )


class Status100ms:
    pitch: float
    roll: float
    leftCurrent: int
    rightCurrent: int

    bit_flags: int

    dust_container_missing: int
    water_tank_installed: int
    hepa_state: int
    carpet_state: int
    # unknown: int

    def __init__(self, data):
        (
            self.pitch,
            self.roll,
            self.leftCurrent,
            self.rightCurrent,
            self.bit_flags,
            # self.unknown,
        ) = unpack_checked("<hhhhB", data)
        self.pitch /= 10.0
        self.roll /= 10.0
        self.dust_container_missing = (self.bit_flags >> 0) & 1
        self.water_tank_installed = (self.bit_flags >> 1) & 1
        self.hepa_state = (self.bit_flags >> 2) & 1
        self.carpet_state = (self.bit_flags >> 3) & 1

    def __repr__(self):
        return "pitch={: 7.2f}°, roll={: 7.2f}°, leftCurrent={:d}mA, rightCurrent={:d}mA, dustBoxSta={:d}, waterBoxSta={:d}, hepaSta={:d}, carpetSta={:d}".format(
            self.pitch,
            self.roll,
            self.leftCurrent,
            self.rightCurrent,
            self.dust_container_missing,
            self.water_tank_installed,
            self.hepa_state,
            self.carpet_state,
            # self.unknown,
        )


class Triggers:
    key1: bool
    key2: bool
    key3: bool
    key4: bool
    left_bumper: bool
    right_bumper: bool
    left_wheel_floating: bool
    right_wheel_floating: bool
    d_view_lf: bool
    d_view_lmf: bool
    d_view_rmf: bool
    d_view_rf: bool
    d_view_lb: bool
    d_view_rb: bool
    mag_signal_left: bool
    mag_signal_right: bool
    ir_dock_lf: int  # 3 bits
    ir_field_lf: bool
    ir_dock_lmf: int  # 3 bits
    ir_field_lmf: bool
    ir_dock_rmf: int  # 3 bits
    ir_field_rmf: bool
    ir_dock_rf: int  # 3 bits
    ir_field_rf: bool
    dock_sta: bool
    lds_button1: bool
    lds_button2: bool
    res1: int  # 2 bits (reserved?)
    side_error: bool
    roll_error: bool
    pump_error: bool
    side_overcurrent: bool
    roll_overcurrent: bool
    fan_overcurrent: bool
    pump_overcurrent: bool
    left_wheel_overcurrent: bool
    right_wheel_overcurrent: bool
    res2: int  # 2 bits (reserved?)
    lidar_error: bool
    fan_error: bool
    left_vel_error: bool
    right_vel_error: bool
    left_mag_error: bool
    right_mag_error: bool
    imu_error: bool
    charge_error: bool

    def __init__(self, data):
        if len(data) != 7:
            raise ValueError("Triggers must be 7 bytes long, got %d" % len(data))
        # reverse the bits in each byte
        data = reverse_bits(data)
        bitstr = BitArray(data)

        # Byte 0
        self.key1 = bitstr[0]
        self.key2 = bitstr[1]
        self.key3 = bitstr[2]
        self.key4 = bitstr[3]
        self.left_bumper = bitstr[4]
        self.right_bumper = bitstr[5]
        self.left_wheel_floating = bitstr[6]
        self.right_wheel_floating = bitstr[7]

        # Byte 1
        self.d_view_lf = bitstr[8]
        self.d_view_lmf = bitstr[9]
        self.d_view_rmf = bitstr[10]
        self.d_view_rf = bitstr[11]
        self.d_view_lb = bitstr[12]
        self.d_view_rb = bitstr[13]
        self.mag_signal_left = bitstr[14]
        self.mag_signal_right = bitstr[15]

        # Byte 2
        self.ir_dock_lf = bitstr[16:19].uint
        self.ir_field_lf = bitstr[19]
        self.ir_dock_lmf = bitstr[20:23].uint
        self.ir_field_lmf = bitstr[23]

        # Byte 3
        self.ir_dock_rmf = bitstr[24:27].uint
        self.ir_field_rmf = bitstr[27]
        self.ir_dock_rf = bitstr[28:31].uint
        self.ir_field_rf = bitstr[31]

        # Byte 4
        self.dock_sta = bitstr[32]
        self.lds_button1 = bitstr[33]
        self.lds_button2 = bitstr[34]
        self.res1 = bitstr[35:37].uint
        self.side_error = bitstr[37]
        self.roll_error = bitstr[38]
        self.pump_error = bitstr[39]

        # Byte 5
        self.side_overcurrent = bitstr[40]
        self.roll_overcurrent = bitstr[41]
        self.fan_overcurrent = bitstr[42]
        self.pump_overcurrent = bitstr[43]
        self.left_wheel_overcurrent = bitstr[44]
        self.right_wheel_overcurrent = bitstr[45]
        self.res2 = bitstr[46:48].uint

        # Byte 6
        self.lidar_error = bitstr[48]
        self.fan_error = bitstr[49]
        self.left_vel_error = bitstr[50]
        self.right_vel_error = bitstr[51]
        self.left_mag_error = bitstr[52]
        self.right_mag_error = bitstr[53]
        self.imu_error = bitstr[54]
        self.charge_error = bitstr[55]

    def __repr__(self):
        # loop through all the attributes and print the ones that are true or non-zero
        return "Triggers(%s)" % ", ".join(
            "%s=%s" % (k, v) for k, v in self.__dict__.items() if v
        )


class BatteryStatus:
    battery_voltage: float  # V
    battery_current: float  # mA
    battery_temperature: float  # C
    charge_voltage: float  # V
    state_of_charge: float  # %
    unknown: int

    def __init__(self, data):
        (
            battery_voltage,
            battery_current,
            battery_temperature,
            charge_voltage,
            state_of_charge,
            self.unknown,
        ) = unpack_checked("<HHhHhH", data)
        self.battery_voltage = battery_voltage / 1000
        self.battery_current = battery_current
        self.battery_temperature = battery_temperature / 10
        self.charge_voltage = charge_voltage / 1000
        self.state_of_charge = state_of_charge / 100

    def __repr__(self):
        return "BatteryStatus(battery_voltage = {:.2f} V, battery_current = {:.2f} mA, battery_temperature = {:.1f} C, charge_voltage = {:.2f}V, state_of_charge = {:.1f} %, unknown = {})".format(
            self.battery_voltage,
            self.battery_current,
            self.battery_temperature,
            self.charge_voltage,
            self.state_of_charge,
            self.unknown,
        )

class Status500ms:
    unk1: int # uint8
    sequence: int # uint8
    rtc_timestamp: int # uint32, unix timestamp
    def __init__(self, data):
        self.unk1, self.sequence, self.rtc_timestamp = unpack_checked("<BBI", data)
    def __repr__(self):
        return "Status500ms(unk1 = {}, sequence = {}, rtc_timestamp = {})".format(self.unk1, self.sequence, self.rtc_timestamp)

class Unk0x0F:
    # one uint32 is timestamp, second uint32 is some kind of time delta
    timestamp: int # uint32
    unk1_delta: int # uint32 # this variable is called comDelay in the mcu code
    def __init__(self, data):
        self.timestamp, self.unk1_delta = unpack_checked("<II", data)
    def __repr__(self):
        return "Unk0x0F(timestamp = {}, unk1_delta = {})".format(self.timestamp, self.unk1_delta)

TYPES_FROM_MCU = {
    0x00: Triggers,
    0x01: Status20ms,
    0x02: Status10ms,
    0x03: Status100ms,
    # 0x04 - factory test, length = 1
    
    0x05: Status500ms, # 0x05 - 500ms, length = 6, RTC data
    # 0x07 - length 16, appears to contain the version and git hash of the MCU firmware
    # 0x0b - len 1, 1 = start lidar calibrate, 2 = stop lidar calibrate, 0 unknown [instead of calibrate it might be spinup]
    # 0x0d - length = 2
   
    0x0f: Unk0x0F,  # 0x0f - length = 8, sent from Com Timer, 
    # 0x10 - length = 1, contains no useful data, sent in reply to pkt 19

    # 0x11, length = 2, some kind of elaborate bitmask, sent from various places, seems like an ack
    # 0x20, length = 7, linelaser status, { timestamp(uint32), 0 uint16, status uint16 }
    # 0x21, length = 2
    # 0x23, length = 5, something connected with the base
    # 0x24, length = 1, one bit, something connected with the battery temperature 
    # 0x25, length = 3
    # 0x26, length = 2, slowSensor
    # 0x27, length = 12, log/error information [ sent from log_data_to_memory]
    # 0x29, length = 5, reads from product ID register, so probably the MCU type
    # 0x28 ???
    0x2B: BatteryStatus,
}

TYPES_TO_MCU = {

    # NOTES FROM RE OF MCU FIRWARE:
    # Maximal packet type is 0x29

    # 0x00 - length should be 9
    #  Byte 1 - 0 or 1
    # 0x01 - length should be 5
    # 0x02 - length should be 1, the byte should be less than 53
    # 0x03 - INVALID
    # 0x04 - length should be 14
    # 0x05 - INVALID
    # 0x06 - INVALID
    # 0x07 - INVALID
    # 0x08 - INVALID
    # 0x09 - INVALID
    # 0x0A - length should be 1, the byte should be 0, 1 or 2, somthing connected with OTA, or rebooting the MCU in some way
    # 0x0B - length should be 2, first byte 0-6, second byte ==1
    # 0x0C - length should be 4
    # 0x0D - length should be 4, (similar to 0x0C)
    # 0x0E - length should be 4
    # 0x0F - length should be 4, one uint (something with time, sets the delta from 0x05)
    # 0x10 - length should be 12, sets LDS calibration - 3 float32 (x,y,angle)
    # 0x11 (17) - length should be 4 - Set RTC time (uint32, unix timestamp)
    # 0x12 (18) - length should be 1
    # 0x13 (19) - length should be 2, some kind of combo packet, has the option to request the version of the MCU firmware
    # 0x14 (20) - length should be less than 32
    # 0x15 (21) - length should be less than 0x14

    # 0x1c (28) - length should be 1 or 2, turns on/off the linelaser when len=1, when len2 does not seem to do anything
    # 0x1d (29) - length should be 1, 0/1, sets a global variable
    # 0x1e (30) - length should be 1, 0/1, 
    # 0x1f (31) - length should be 4, one uint32, sets a global variable
    # 0x20 (32) - INVALID
    # 0x21 (33) - INVALID
    


}
