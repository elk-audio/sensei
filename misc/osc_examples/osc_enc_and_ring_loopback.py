# Program to loopback rotary encoder value to the led ring using OSC messages
# when running the example SENSEI config file in
# misc/example_configs/encoder_and_led_ring.json

import liblo
import time

value = 0.0
led_ring_sensor_id = 1
target = liblo.Address('127.0.0.1', 23024)
server = liblo.Server(23000)

def callback(path, args, types, src, data):
    new_value = args[0]
    value = new_value
    liblo.send(target, '/set_output', ('i', led_ring_sensor_id), ('f', new_value))

def main():
    server.add_method("/sensors/analog/rot_enc", 'f', callback, 0)

    while True:
        server.recv(30)

if __name__ == "__main__":
    main()
