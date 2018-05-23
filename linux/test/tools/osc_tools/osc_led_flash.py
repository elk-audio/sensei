
import liblo
import time

FIRST_RING = 5
LAST_RING = 9

def main():
	target = liblo.Address('127.0.0.1', 24024)
	led = 0
	ring = FIRST_RING
	inc = 1
	while True:
		liblo.send(target, '/set_range_output', ('i', ring), ('i', led))
		led += 1
		if led > 15:
			led = 0
			ring += 1

			if ring > LAST_RING:
				ring = FIRST_RING
				for i in range(FIRST_RING, LAST_RING + 1):
				 	liblo.send(target, '/set_range_output', ('i', i), ('i', 0))

		time.sleep(0.05)

if __name__ == "__main__":
    main()
