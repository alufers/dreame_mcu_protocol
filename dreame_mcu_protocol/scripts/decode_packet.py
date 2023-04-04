# Dreame Z10 Pro MCU communication protocol analyzer
# Based on firmware version 1156, board type MR813
# This robot talks to the MCU at /dev/ttyS4, baud rate 115200.
#
# Dreame appears to have two variants of this protocol, one is called DOUBLE_BYTE and the other is called SINGLE_BYTE.
# The Dreame Z10 Pro uses the SINGLE_BYTE variant, and so does this script.
# See: squashfs-root/usr/bin/boardconfig
#
# Message structure:
# The messages are binary, but are wrapped in < and > characters (presumably to synchronize the start and end of the message).
# The ? character is an escape character, and is used to escape the <, >, and ? characters.
# Please note that for some reason the MCU sometimes sends corrupted messages, which don't start with <, and the crc of them
# is wrong. You must take that into account when parsing the messages.
#
# The message between the delimeters is a sequence of bytes, with the following structure:
#
# +----------------+----------------+---------------------------------+-----------------+
# | Length         | Message type   | Payload                         | CRC-16          |
# | 1 byte         | 1 byte         | Variable length                 | 2 bytes         |
# +----------------+----------------+---------------------------------+-----------------+
# The crc-16 is calculated over the message length, message type, and payload.
# For implementation see CRC_GetModbus16()
#


from io import BytesIO
import struct
from ..crc_util import CRC_GetModbus16
from ..mcu_packets import *


def analyze_message(msg):
    msg_io = BytesIO(msg)
    packet = read_packet(msg_io)

    # print packet in hex, space separated
    print(" ".join("{:02x}".format(c) for c in packet))
    first_byte = packet[0]
    str = "LL TT "
    for i in range(first_byte):
        str += "XX "
    print(str + "CR CR")

    # print ascii representation, if possible
    ascii = ""
    for c in packet:
        if c >= 32 and c <= 126:
            ascii += chr(c) + "  "
        else:
            ascii += ".  "
    print(ascii)

    length = packet[0]
    type = packet[1]
    # calculate crc16

    crc = CRC_GetModbus16(packet[0:-2])
    print("crc16 calculated: {:04x}".format(crc))
    print("crc16 in packet: {:04x}".format(packet[-1] + (packet[-2] << 8)))
    print("Length: {}".format(length))
    print("Type: {}".format(type))
    print()

    # use the length to get the data
    data = packet[2:]
    # cut to length
    data = data[:length]
    try:
        if type == 0x01:
            status = Status20ms(data)
            print(status)
        elif type == 0x02:
            status = Status10ms(data)
            print(status)
    except Exception as e:
        print(e)


def main():
    analyze_message(
        b"\x3c\x1a\x01\xcd\xa8\xfb\x01\xe1\x0b\xfd\xff\x6f\xb5\xff\xff\x28\xee\xff\xff\xaf\x28\x00\x00\x00\x00\x00\x00\x00\x00\xa0\xe3\x3e"
    )
    analyze_message(
        b"\x3c\x12\x02\xf3\x77\xd8\x44\x18\x00\xee\xff\x18\x00\x37\x00\x29\x00\x76\x40\x00\x00\xe8\x32\x3e"
    )
    analyze_message(
        b"\x3c\x12\x02\xad\xe8\xdf\xa3\x1e\x00\xee\xff\x1c\x00\x11\x08\x7b\x3f\x3f\x70\x00\x00\x00\x7e\xa9\x3e"
    )

    analyze_message(
        bytes.fromhex("3C011E0160E83E")
    )  # squashfs-root/ava/script/ota_dw_fw.sh

    analyze_message(
        bytes.fromhex("3C01020060213E")
    )  # squashfs-root/ava/script/mcu_monitor_arm.sh
    analyze_message(
        bytes.fromhex("3C010A81C0E63E")
    )  # squashfs-root/ava/script/ota_base_station.sh

    # analyze_message(
    #     bytes.fromhex("3c1a01a49ec0fb16150000d5d3ffff24cbffff113700000000000000006f3f3e")
    # )  # squashfs-root/ava/script/ota_base_station.sh

    analyze_message(
        bytes.fromhex(
            "3c09030f0002000100ff120228f0b6b81800ecff18002b00030076400000b7043e"
        )
    )  # packet that had a mismatched crc

    # battery status parsing
    payload = bytes.fromhex("4040a600fa00000017250000")

    bat_status = BatteryStatus(payload)
    print(bat_status)
