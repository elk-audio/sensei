import serial
import time
from datetime import datetime
"""
Quick test script for putting data in a serial port.
First run:  sudo socat PTY,link=/dev/ttyS10 PTY,link=/dev/ttyS11
to set up a pair of virtual serial ports and set your code
to listen to /dev/ttyS10
"""

def main():
	ser = serial.Serial('/dev/ttyS11', baudrate=9600)
	print(ser.name)
	i = 0  
	while (1):
		ser.write('hello ' +str(i) +  str(datetime.now()) + '\n')
		i += 1
		time.sleep(1)

	ser.close()


if __name__ == "__main__":
    main()