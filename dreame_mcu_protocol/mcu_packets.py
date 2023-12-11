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
    
class McuLog:
    data: bytes
    def __init__(self, data):
        self.data = data
        assert len(data) == 12
    def __repr__(self):
        return "McuLog(data = {})".format(self.data.hex())

class PingMsg:
    """
        Sent from the MCU, the SoC must respond with Pong (0x0F).
        Used to coalculate latency between the MCU and the SoC.
    """
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
    0x02: Status10ms, # AvaImuMsg
    0x03: Status100ms, # Used on Z10, len=9 (can have different length on other models) Decoded by SignalTransPac3
    # 0x04 - factory test, length = 1 (not handled in node_signal.so)
    
    0x05: Status500ms, # 0x05 - 500ms, length = 6, RTC data
    # 0x07 - length 16, contains the version and git hash of the MCU firmware (interpreted by node_sys.so)
    # 0x0b - len 1, 1 = start lidar calibrate, 2 = stop lidar calibrate, 0 unknown [instead of calibrate it might be spinup]
    # 0x0d - length = 2
   
    0x0f: PingMsg,  # 0x0f - length = 8, sent from Com Timer, 
    # 0x10 - length = 1, contains no useful data, sent in reply to pkt 19 (involved in poweroff)

    # 0x11, length = 2, "fct_calibration" some kind of elaborate bitmask, sent from various places, seems like an ack
    # 0x12, length = 7. DoubleCameraTimestamp, or something with TOF (Not used on Z10)
    # 0x13, length = 1 AvaTOFCalibrationCmdMsg (Not used on Z10)
    # 0x20, length = 7, linelaser status, { timestamp(uint32), 0 uint16, status uint16 }
    # 0x21, length = 2
    # 0x23, length = 5, something connected with the base (max length = 6 on other models), on Z10 SignalTransPac6 is used to decode this
    # 0x24, length = 1, one bit, something connected with the battery temperature 
    # 0x25, length = 3 (ava_ai_calibr_check_msg)
    # 0x26, length = 2, sent from slowSensor, _CtrlMcuCMD
    0x27: McuLog, # length = 12, log/error information [ sent from log_data_to_memory], data is saved to "/data/log/mculog.bin" by AVA
    # 0x28 ???
    # 0x29, length = 5, reads from product ID register, gets MCU type and IMU type (interpreted by node_sys.so)
    0x2B: BatteryStatus, # ava_msg_battery
}

class ToMcu_MotorCtrl:
    """
            void avaplugsignal::AvaNodeSignal::MoveControlProcess(AvaNodeSignal *this,AvaMoveControlMsg *param_1)

            {
                    Move_msg_scratch.flag = SUB41(param_1->field0_0x0,0);
                    if (param_1->field0_0x0 != 1) {
                        if (param_1->field0_0x0 == 0) {
                        Move_msg_scratch.linear_velocity = param_1->field1_0x4;
                        Move_msg_scratch.rotational_velocity = param_1->field2_0x8;
                        }
                        CastComMsg_actually_send(this,0x0,&Move_msg_scratch.flag,9);
                        return;
                    }
                    Move_msg_scratch.rotational_velocity =
                        (int32_t)(((float)param_1->field2_0x8 / 18000.0) * 3.141593);
                    Move_msg_scratch.linear_velocity = (int32_t)(float)param_1->field1_0x4;
                    CastComMsg_actually_send(this,0x0,&Move_msg_scratch.flag,9);
                    return;
            }
    
    """
    flag: int # uint8, must be 0 or 1 , (switches between degrees and radians ???)
    linear_velocity_set = int # int32 
    rotational_velocity_set = int # int32
    def __init__(self, data):
        self.flag, self.linear_velocity_set, self.rotational_velocity_set = unpack_checked("<bii", data)
        assert self.flag in [0, 1]
    def __repr__(self):
        return "ToMcu_MotorCtrl(flag = {}, vl_set = {}, vr_set = {})".format(self.flag, self.linear_velocity_set, self.rotational_velocity_set)


class ToMcu_SetCleaning:
    """
        Controls which pumps/fans/etc are on.
    """
    f1: int # uint8
    f2: int # uint8
    f3: int # uint8
    f4: int # uint8
    f5: int # uint8
    def __init__(self, data):
        self.f1, self.f2, self.f3, self.f4, self.f5 = unpack_checked("<BBBBB", data)
    def __repr__(self):
        return "ToMcu_SetCleaning(f1 = {}, f2 = {}, f3 = {}, f4 = {}, f5 = {})".format(self.f1, self.f2, self.f3, self.f4, self.f5)


class ToMcu_SetButtonLEDState:
    """
        Controls the button LEDs (value of state should be less than 53)
        It also doubles as a heartbeat packet, the MCU has some timers for it.

        2 = on
        33 = off
    """
    state: int # uint8
    def __init__(self, data):
        self.state = unpack_checked("<B", data)
    def __repr__(self):
        return "ToMcu_SetButtonLEDState(state = {})".format(self.state)

class ToMcu_SetOdometer:
    """
    
    Length: 14
    """
    op: int # uint8
    f1: int # uint32
    f2: int # uint32
    f3: int # uint32
    f4: int # uint8
    def __init__(self, data):
        self.op, self.f1, self.f2, self.f3, self.f4 = unpack_checked("<BIIIb", data)
    def __repr__(self):
        return "ToMcu_SetOdometer(op = {}, f1 = {}, f2 = {}, f3 = {}, f4 = {})".format(self.op, self.f1, self.f2, self.f3, self.f4)

class ToMcu_LaserOrTofControl:
    """
        reset_trans:
            - 1 = ava_tof_reset_msg
            - 4 = ava_camera_stero_reset_msg
            - 6 = ava_msg_laser_rese
    """
    reset_trans: int # uint8
    value: int # uint8, 0 or 1
    def __init__(self, data):
        self.reset_trans, self.value = unpack_checked("<BB", data)
    def __repr__(self):
        return "ToMcu_LaserOrTofControl(reset_trans = {}, value = {}".format(self.reset_trans, self.value)

class ToMcu_SetLDSCalibration:
    x: float # float32
    y: float # float32
    angle: float # float32
    def __init__(self, data):
        self.x, self.y, self.angle = unpack_checked("<fff", data)
    def __repr__(self):
        return "ToMcu_SetLDSCalibration(x = {}, y = {}, angle = {})".format(self.x, self.y, self.angle)

TYPES_TO_MCU = {

    # NOTES FROM RE OF MCU FIRWARE:
    # Maximal packet type is 0x29

    0x00: ToMcu_MotorCtrl, # len 9
    0x01: ToMcu_SetCleaning, # len 5
    0x02: ToMcu_SetButtonLEDState, # len 1
    # 0x03 - INVALID
    0x04: ToMcu_SetOdometer, # len 14
    # 0x05 - INVALID
    # 0x06 - INVALID
    # 0x07 - INVALID
    # 0x08 - INVALID
    # 0x09 - INVALID
    # 0x0A - 
    # 0x0B -
    # 0x0C 
    # 0x0D -
    # 0x0E 
    # 0x0F - PongMessage, len 4. Used to measure latency
    # 0x10 - INVALID
    0x11: ToMcu_SetLDSCalibration, # - set LDS calibration AvaCalibrationResultMsg, len=12
    # 0x12 - Set RTC time on the mcu len 4
    # 0x13 -
    # 0x14 - _CtrlMcuCMD len 2
    # 0x15 -

    # 0x1c - 
    0x1d: ToMcu_LaserOrTofControl # - ava_tof_reset_msg, ava_msg_laser_reset # len 1 or 2
    # 0x1e - 
    # 0x1f - 
    # 0x20 - 
    # 0x21 - INVALID
    # 0x22 - INVALID [AvaTOFCalibrationResultMsg]

    # 0x22 - AvaTOFCalibrationResultMsg

    # 0x25 ava_msg_station_set - length: 2
    # 0x26 - INVALID [ava_clean_dock_msg] len = 8
    # 0x27 
    # 0x28 ava_msg_station_led_set - length: 1

    # ---- invalid

    # 0x29 Uvlight len 1
    # 0x2a AvaTOFCheckResult len=18
    


}
