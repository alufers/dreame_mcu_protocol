import argparse
from ..ssh_capture import capture_ssh_output
from ..mcu_packets import read_packet, parse_packet, TYPES_FROM_MCU


def main():
    parser = argparse.ArgumentParser(
        description="Sniff packets between the MCU and SOC from the vacuum over SSH"
    )
    parser.add_argument(
        "--host",
        help="The host and username to connect to (e. g. root@<ip>)",
        type=str,
        required=True,
    )
    parser.add_argument(
        "--serial_path",
        help="The path to the serial device (e. g. /dev/ttyS4)",
        type=str,
        default="/dev/ttyS4",
    )
    parser.add_argument(
        "--strace_path",
        help="The path to the strace binary",
        type=str,
        default="/data/strace_arm64",
    )
    parser.add_argument(
        "--focus_msg",
        help="Focus on a specific message type, and display it live",
        type=str,
        default=None,
        choices=list([x.__name__ for x in TYPES_FROM_MCU.values()]) + [None, "Unknown", "DecodingError", "CrcError"],
    )
    args = parser.parse_args()

    (r, w) = capture_ssh_output(
        host=args.host,
        strace_path=args.strace_path,
        serial_path=args.serial_path,
    )
    while True:
        try:
            raw_packet = read_packet(r)
            (type, payload) = parse_packet(raw_packet)
            if type in TYPES_FROM_MCU:
                type_name = TYPES_FROM_MCU[type].__name__
                try:
                    decoded = TYPES_FROM_MCU[type](payload)
                    if args.focus_msg is not None:
                        if args.focus_msg == type_name:
                            print("\33[2K\r{}: {}".format(type_name, decoded), end="", flush=True)
                    else:
                        print("{}: {}".format(type_name, decoded))
                except Exception as e:
                    if args.focus_msg is None or args.focus_msg == "DecodingError":
                        print("{}: [failed to decode: {}] Raw hex: {}".format(type_name, e, payload.hex()))
                    
            else:
                if args.focus_msg is None or args.focus_msg == "Unknown":
                    print("Unknown type {} (len = {}):".format(type, len(payload)))
        except Exception as e:
            if args.focus_msg is not None or args.focus_msg == "CrcError":
                print("CRC error: {}".format(e))
            # print(e)
            # # prtin stack trace
            # import traceback
            # traceback.print_exc()
