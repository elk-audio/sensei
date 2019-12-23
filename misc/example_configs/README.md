# Example configurations for SENSEI

Simple example configurations to control gpio with SENSEI.


### Output OSC port
All these configuration files tell SENSEI to use the port 23000 for OSC output.

### led_button_pot.json
This configuration describes an led, a button and a potentiometer connected to the gpio pins as described below:

Physical Device                 | SENSEI id| Pin |
--------------------------------|----------|---------|------------------------------------------------------------------------------------------------------
led               | 1  | Digital Output Pin 0      |
button            | 2  | Digital Input Pin 0      |
potentiometer     | 3  | Analog Pin 0      |

### encoder_and_led_ring.json
This configuration describes a rotary encoder and a led ring made up of 8 individual LEDs connected to the gpio pins as described below:

Physical Device                 | SENSEI id| Pin |
--------------------------------|----------|---------|------------------------------------------------------------------------------------------------------
led_ring               | 1  | Digital Output Pins 0 - 8      |
rot_enc            | 2  | Digital Input Pin 0 for Pin A and 1 for Pin B      |


## Usage
To run the config file with SENSEI:

    sensei -f < config file >

To view input controller data, open another terminal and run

`oscdupump 23000 `

To set a value on an output controller use

    oscsend localhost 23024 /set_output if < id >  < value between 0 and 1>

For example, to set the led in `led_button_pot.json`:

    oscsend localhost 23024 /set_output if 1  1

### Using python to route OSC messages
You can also use the given python scripts in misc/osc_examples which implements
an OSC loopback between the input and output controllers.