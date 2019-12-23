import liblo
import time

values = [0.0,0.0,0.0,0.0,0.0]
sensor_ids = [5,6,7,8,9]
target = liblo.Address('127.0.0.1', 24024)
server = liblo.Server(23023)

def callback(path, args, types, src, data):
	new_value = args[0]
	values[data] = new_value
	liblo.send(target, '/set_output', ('i', sensor_ids[data]), ('f', new_value))

def main():
	server.add_method("/sensors/analog/rot_enc_gain", 'f', callback, 0)
	server.add_method("/sensors/analog/rot_enc_low", 'f', callback, 1)
	server.add_method("/sensors/analog/rot_enc_mid", 'f', callback, 2)
	server.add_method("/sensors/analog/rot_enc_high", 'f', callback, 3)
	server.add_method("/sensors/analog/rot_enc_level", 'f', callback, 4)

	while True:
	    server.recv(30)

if __name__ == "__main__":
    main()
