"""
Quick test script for sending serial dumps one by one to a serial port.
First run:  sudo socat PTY,link=/dev/ttyS10 PTY,link=/dev/ttyS11
to set up a pair of virtual serial ports and chmod 666 on the underlying
pseudo terminals in /dev/pts
"""

import time
import struct
import sys
from datetime import datetime
import argparse

import serial

import teensy_log_parser as tlp

DEFAULT_SEND_INTERVAL_MS = 1000
DEFAULT_SERIAL_PORT = '/dev/ttyS10'
DEFAULT_DUMP_FILE = 'log_data.txt'

def send_packets_from_dump(serial_port,
                           dump_file,
                           cmd='GET_VALUE',
                           sub_cmd=None,
                           send_interval_ms=DEFAULT_SEND_INTERVAL_MS):
    ser = serial.Serial(serial_port, baudrate=115200)
    packets = tlp.filter_log_file(dump_file, cmd=cmd, sub_cmd=sub_cmd, source='teensy')

    for packet in packets:
        ser.write(packet.binary_data)
        print "Sending %s bytes" % len(packet.binary_data)
        time.sleep(send_interval_ms * 1.0e-3)

    ser.close()

if __name__ == "__main__":
    # CL arguments parse
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", default=DEFAULT_SERIAL_PORT, type=str, help="Specify serial port device")
    parser.add_argument("-f", "--file", default=DEFAULT_DUMP_FILE, type=str, help="Specify dump file path")
    args = parser.parse_args()

    send_packets_from_dump(args.port, args.file)
