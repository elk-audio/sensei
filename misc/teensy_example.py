"""
This is just an example of how to use sensei module from e.g. an interactive shell
or inside other scripts for real-time communication with the board.
"""

import sensei

# Create frontend and send system initialize message
frontend = sensei.SerialFrontend('/dev/cu.usbmodem1225061')

# Check return ACKs
acks = frontend.receive_messages()

# Create some commands
config_msgs = []
config_msgs.append(sensei.ConfigurePinMessage(sub_cmd='SET_PIN_DIGITAL_INPUT',
                                              pin_idx=0))
config_msgs.append(sensei.ConfigurePinMessage(sub_cmd='SET_PIN_ANALOG_INPUT',
                                              pin_idx=1,
                                              adc_bit_resolution=10,
                                              lowpass_cutoff=20.0))

# Send configuration
frontend.send_messages(config_msgs, write_interval_ms=100)

# Acquire data on pin 1 for 10 seconds
tt, values = frontend.acquire_signal(1, 10)

# Turn digital output 0 to HIGH
frontend.send_messages([sensei.SetDigitalMessage(sub_cmd='SET_PIN', pin_idx=0, value=1)])

