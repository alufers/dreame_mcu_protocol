# dreame_mcu_protocol 

Python package for reverse engineering the Dreame Z10 Pro vacuum cleaner MCU communication protocol.

Robot vacuum cleaners usually consist of a SoC running Linux and a separate MCU for controlling the motors and sensors.

The SoC usually runs a big Linux application that handles navigation, mapping and networking, while the MCU is responsible for controlling things which require real-time control, such as motors and sensors.

Here are my notes about the vacuum cleaner's internals [dreame_z10_notes.md](./dreame_z10_notes.md) and the protocol.

## Installation

Python 3 and [poetry](https://python-poetry.org/) are required.

```bash
poetry install
```

## Available scripts

- `poetry run decode_packet` - a low level script for decoding packets, it contains some hardcoded data that I have captured using `strace` from the running vacuum cleaner application. 
- `poetry run sniff_over_ssh --host root@10.123.1.167` - a script for sniffing the communication between the vacuum cleaner and the MCU over SSH. It requires a rooted vacuum cleaner and a way to connect to it over SSH. You also need to copy `strace_arm64` to `/data`.

## Manually capturing data from the vacuum cleaner

If you have a rooted Dreame Z10 Pro you can ssh into it copy the `strace` program to it and run it to capture the data being sent and received from the MCU.

I've found the best way to get Linux programs onto the vacuum is to download prebuilt binaries from the Arch Linux ARM project. For strace you can head to https://archlinuxarm.org/packages/aarch64/strace, download and unpack the package and then copy the `strace` binary to the vacuum.

To start capturing data, first you need to get the pid of the process that communicates with the MCU.

```
$ lsof | grep /dev/ttyS4
$ NNNN	/usr/bin/ava	/dev/ttyS4
```

Instead of `NNNN` you should see the pid of the process that communicates with the MCU.

Then you can start capturing the data:

```bash
$ ./strace_arm64  -x -p 1275 -P /dev/ttyS4
```

This should give you an output like this:

```
write(21, "\x3c\x01\x28\x01\xc0\xff\x3e", 7) = 7
write(21, "\x3c\x09\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\xe8\x25\x3e", 15) = 15
write(21, "\x3c\x01\x28\x01\xc0\xff\x3e", 7) = 7
read(21, "\x02\x26\x07\x01\xa7\x23\x3e\x3c\x08\x0f\x02\x48\x9c\x22\xfc\x1f\x00\x00\xcd\x1d\x3e\x3c\x06\x05\x5a\x17\xff\x2f\x24\x64\xee\xf6"..., 1024) = 37
write(21, "\x3c\x04\x0f\x02\x48\x9c\x22\x29\x3d\x3e", 10) = 10
read(21, "\x00\x07\x00\x00\x00\x00\x00\x99\x3e\x3c\x09\x03\x17\x00\x12\x00\x02\x00\x00\x00\x00\x46", 1024) = 22
read(21, "\x78\x3e\x3c\x12\x02\xde\x68\x9c\x22\x1a\x00\xec\xff\x1a\x00\x41\x00\x35\x00\x72\x40\x00\x00\x0f\x99\x3e", 1024) = 26
read(21, "\x3c\x12\x02\xfb\x8f\x9c\x22\x1a\x00\xec\xff\x1a\x00\x43\x00\x31\x00\x76\x40\x00\x00\xa2\xf8\x3e\x3c\x1a\x01\x31\x93\x9c\x22\x08", 1024) = 32
read(21, "\x00\x00\x00\x00\x00\x00\x00\xfb\xff\xff\xff\xd3\x1e\x00\x00\x00\x00\x00\x00\x00\x00\x20\x23\x3e\x3c\x02\x26\x07\x01\xa7\x23\x3e", 1024) = 32
```
