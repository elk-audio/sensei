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

# Acquire data on pin 1
N_ACQUISITION_MESSAGES = 1000
ACQUISITION_PIN = 1
values = []
n_msg_received = 0

frontend.enable_sending_packets(True)
while (n_msg_received < N_ACQUISITION_MESSAGES):
    msgs = frontend.receive_messages('VALUE', 1)
    for m in msgs:
        if not (m.pin_idx == ACQUISITION_PIN):
            continue
        values.append((m.packet.timestamp, m.value))
        n_msgs_received += 1
frontend.enable_sending_packets(False)

print values

# Turn digital output 0 to HIGH
frontend.send_messages([sensei.SetDigitalMessage(sub_cmd='SET_PIN', pin_idx=0, value=1)])

