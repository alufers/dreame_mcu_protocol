import argparse
from ..ssh_capture import capture_ssh_output
from ..mcu_packets import read_packet, parse_packet, TYPES_FROM_MCU, Status10ms
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time
def main():
    parser = argparse.ArgumentParser(
        description="Sniff packets between the MCU and SOC and show them in a GUI"
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
   
    args = parser.parse_args()

    (r, w) = capture_ssh_output(
        host=args.host,
        strace_path=args.strace_path,
        serial_path=args.serial_path,
    )
    data = {}
    data_maxes = {} # maximum values for each data type
    data_mins = {} # minimum values for each data type
    data_lock = threading.Lock()
    class Datapoint():
        def __init__(self, data):
            self.val = data
            self.time = time.time()
    def push_data(data, key, val):
        if key not in data:
            data[key] = []
            data_maxes[key] = val
            data_mins[key] = val
        if val > data_maxes[key]:
            data_maxes[key] = val
        if val < data_mins[key]:
            data_mins[key] = val
        
        data[key].append(Datapoint(val))
        if len(data[key]) > 500:
            data[key].pop(0)
    def plot_thread():
        def anim_func(i):
            with data_lock:
                plt.cla()
                total_min = 0
                total_max = 0
                for key in data:
                    x = []
                    y = []
                    curr_time = time.time()
                    for d in data[key]:
                        x.append(d.time - curr_time)
                        y.append(d.val)
                    plt.plot(x, y, label=key)
                    total_min = min(total_min, data_mins[key])
                    total_max = max(total_max, data_maxes[key])
               
                plt.ylim(total_min, total_max)
                plt.xlabel('Time')
                plt.title('Accel over time')
                plt.grid(True)
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ani = animation.FuncAnimation(fig, anim_func, interval=30)
        plt.show()
    t = threading.Thread(target=plot_thread)
    t.start()

    while True:
        try:
            raw_packet = read_packet(r)
            (type, payload) = parse_packet(raw_packet)
            if type in TYPES_FROM_MCU:
                type_name = TYPES_FROM_MCU[type].__name__
                
                try:
                    decoded = TYPES_FROM_MCU[type](payload)
                    if isinstance(decoded, Status10ms):
                        with data_lock:
                            push_data(data, "Accel x", decoded.accel_x)
                            push_data(data, "Accel y", decoded.accel_y)
                            push_data(data, "Accel z", decoded.accel_z)
                       
                except Exception as e:
                   pass
                    
            else:
                pass
        except Exception as e:
            pass
