import subprocess
import io
import typing
import threading
from .fifo import Fifo


def bytes_from_strace_line(line: bytes) -> bytes:
    """
    Decode a line like this to raw bytes from the quotes:
    write(21, "\x3c\x01\x28\x01\xc0\xff\x3e", 7) = 7
    """

    parts = line.split(b'"')
    if len(parts) != 3:
        raise Exception("Unexpected line: {}".format(line))

    remove = b"\\x"

    return bytes.fromhex(parts[1].replace(remove, b"").decode("utf-8"))


def capture_ssh_output(
    host: str, strace_path: str, serial_path: str = "/dev/ttyS4"
) -> typing.Tuple[io.BytesIO, io.BytesIO]:
    """
    Sniffs serial communication over SSH using strace.

    It returns a tuple of two streams, the first one is the stream of bytes read from the serial device,
    the second one is the stream of bytes written to the serial device.
    """
    # run "lsof | grep /dev/ttyS4"
    # get the PID

    pid_output = subprocess.check_output(
        ["ssh", host, "sh", "-c", "lsof | grep {}".format(serial_path)]
    )
    lines = pid_output.splitlines()
    if len(lines) == 0:
        raise Exception("No process found using {}".format(serial_path))
    pid = lines[0].split()[0].decode("utf-8")
    process_name = lines[0].split()[1].decode("utf-8")
    print("Found process {} ({}) using {}".format(pid, process_name, serial_path))

    # run "strace -p <pid> -xx -P {serial_path} -e write,read"
    # capture output

    read_stream = Fifo()
    write_stream = Fifo()

    # run in another thread
    def read_strace_output():

        cmd = "{} -xx -p {} -P {}".format(strace_path, pid, serial_path)

        strace_proc = subprocess.Popen(
            ["ssh", host, cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        try:
            while True:
                line = strace_proc.stderr.readline()
                if not line:
                    break

                if line.startswith(b"write("):

                    # write to serial
                    # print("write: {}".format(line))

                    write_stream.write(bytes_from_strace_line(line))
                elif line.startswith(b"read("):

                    # read from serial

                    read_stream.write(bytes_from_strace_line(line))
        finally:

            strace_proc.terminate()

    t = threading.Thread(target=read_strace_output)
    t.start()
    return read_stream, write_stream
