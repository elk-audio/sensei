import struct
import numpy as np

##################################################
#  Maps for retrieving enum contents
#  as defined in common/sensei_serial_protocol.h
##################################################

CMD_MAP = {
    0 : 'INITIALIZE_SYSTEM',
    1 : 'HELLO',
    2 : 'SERIAL_DEBUG',
    3 : 'GET_SYSTEM_STATUS',
    10 : 'ENABLE_SENDING_PACKETS',
    100 : 'CONFIGURE_PIN',
    101 : 'GET_PINS_CONFIGURATION',
    102 : 'GET_VALUE',
    103 : 'GET_ALL_VALUES',
    104 : 'SET_VALUE',
    105 : 'SET_DIGITAL_PINS',
    200 : 'IMU_START',
    201 : 'IMU_STOP',
    202 : 'IMU_SET_SETTINGS',
    203 : 'IMU_GET_SETTINGS',
    204 : 'IMU_GYROSCOPE_CALIBRATION',
    205 : 'IMU_RESET_FILTER',
    206 : 'IMU_GET_DATA',
    207 : 'IMU_TARE_WITH_CURRENT_ORIENTATION',
    208 : 'IMU_RESET_TARE',
    250 : 'STOP_BOARD',
    255 : 'ACK'
}

EMPTY_SUB_CMD_MAP = {0 : 'EMPTY'}

SUB_CMD_MAP = {
    'INITIALIZE_SYSTEM' : EMPTY_SUB_CMD_MAP,
    'HELLO' : EMPTY_SUB_CMD_MAP,
    'SERIAL_DEBUG' : EMPTY_SUB_CMD_MAP,
    'GET_SYSTEM_STATUS' : EMPTY_SUB_CMD_MAP,
    'ENABLE_SENDING_PACKETS' : EMPTY_SUB_CMD_MAP,
    'CONFIGURE_PIN' : {
                          0 : 'SET_PIN_DISABLE',
                          1 : 'SET_PIN_DIGITAL_INPUT',
                          2 : 'SET_PIN_DIGITAL_OUTPUT',
                          3 : 'SET_PIN_ANALOG_INPUT'
                      },
    'GET_PINS_CONFIGURATION' : EMPTY_SUB_CMD_MAP,
    'GET_VALUE' : { 0 : 'GET_SINGLE_PIN' },
    'GET_ALL_VALUES' :  {
                            0 : 'PINS',
                            1 : 'DIGITAL_PIN'
                        },
    'SET_VALUE' : { 0 : 'SET_SINGLE_PIN' },
    'SET_DIGITAL_PINS' : {  0 : 'SET_PIN',
                            1 : 'SET_BANK'
                         },
    'IMU_START' : EMPTY_SUB_CMD_MAP,
    'IMU_STOP' : EMPTY_SUB_CMD_MAP,
    'IMU_SET_SETTINGS' : EMPTY_SUB_CMD_MAP,
    'IMU_GET_SETTINGS' : EMPTY_SUB_CMD_MAP,
    'IMU_GYROSCOPE_CALIBRATION' : EMPTY_SUB_CMD_MAP,
    'IMU_RESET_FILTER' :EMPTY_SUB_CMD_MAP,
    'IMU_GET_DATA' : {
                        0 : 'GET_ALL_DATA',
                        1 : 'GET_DATA_COMPONENT_SENSOR',
                        2 : 'GET_DATA_COMPONENT_SENSOR_NORMALIZED',
                        3 : 'GET_DATA_QUATERNION',
                        4 : 'GET_DATA_LINEARACCELERATION',
                        5 : 'GET_DATA_QUATERNION_LINEARACCELERATION'
                     },
    'IMU_TARE_WITH_CURRENT_ORIENTATION' :EMPTY_SUB_CMD_MAP,
    'IMU_RESET_TARE' : EMPTY_SUB_CMD_MAP,
    'STOP_BOARD' : EMPTY_SUB_CMD_MAP,
    'ACK' : EMPTY_SUB_CMD_MAP
}

PIN_TYPE_MAP = {
        0 : 'DISABLE',
        1 : 'DIGITAL_INPUT',
        2 : 'DIGITAL_OUTPUT',
        3 : 'ANALOG_INPUT'
}

SENDING_MODE_MAP = {
        0 : 'SENDING_MODE_ON_REQUEST',
        1 : 'SENDING_MODE_CONTINUOUS',
        2 : 'SENDING_MODE_ON_VALUE_CHANGED'
}

ADC_RESOLUTION_MAP = {
        0 : 12,
        1 : 11,
        2 : 10,
        3 : 9,
        4 : 8
}

EXPECTED_START_HEADER = (1, 2, 3)
EXPECTED_STOP_HEADER  = (4, 5, 6)

##################################
#  Classes for parsing log data  #
##################################

class SenseiPacket(object):
    """Parse dump of a 64-bytes Sensei message,
       checking start/stop header signatures and parsing
       the most relevant fields.
    """
    class PacketValues(object):
        pass

    def __init__(self, binary_data):
        self.binary_data = binary_data
        self.data = np.array(struct.unpack('64B', self.binary_data), dtype=np.uint8)
        self.values = self.PacketValues()
        self._parse_binary_data()

    def to_c_array(self, var_prefix='packet_dump'):
        """Return a string with C source code for an array declaration
           of this packet.
        """
        N_VAL_PER_LINES = 8
        assert((len(self.binary_data) % N_VAL_PER_LINES) == 0)
        out_lines = []
        out_lines.append('static uint8_t %s[] = {\n' % var_prefix)
        n_lines = len(self.binary_data) / N_VAL_PER_LINES
        for n in range(n_lines):
            low = n*N_VAL_PER_LINES
            high = (n+1)*N_VAL_PER_LINES
            line_bytes = [ struct.unpack('B', x)[0] for x in self.binary_data[low:high] ]
            line = '    ' + ', '.join(["0x%02x" % b for b in line_bytes]) + ',    // %02d .. %02d\n' % (low, high-1)
            out_lines.append(line)
        out_lines.append('};\n')
        return ''.join(out_lines)

    def _parse_binary_data(self):
        self._parse_values()
        if not (self.values.start_header == EXPECTED_START_HEADER):
            raise ValueError, "Wrong start header: %s" % self.values.start_header
        if not (self.values.stop_header == EXPECTED_STOP_HEADER):
            raise ValueError, "Wrong stop header: %s" % self.values.stop_header
        computed_crc = np.uint16(np.sum(self.data[3:59]))
        if (computed_crc != self.values.crc):
            raise ValueError, "Wrong CRC found in packet: %s" % self.values.crc

        self.cmd = CMD_MAP[self.values.cmd]
        self.sub_cmd = SUB_CMD_MAP[self.cmd][self.values.sub_cmd]
        self.continuation = self.values.continuation
        self.timestamp = self.values.timestamp

        # Parse payload for some cmd type
        self.pin_idx = None
        self.sensor_value = None
        self.sending_mode = None
        self.delta_ticks = None
        self.adc_bit_resolution = None
        self.low_pass_cutoff = None
        self.filter_order = None
        self.slider_mode_enabled = None
        self.slider_threshold = None
        if (self.cmd == 'GET_VALUE'):
            self.pin_idx = struct.unpack_from('<H', self.values.payload, 0)[0]
            self.sensor_value = struct.unpack_from('<H', self.values.payload, 2)[0]
        if (self.cmd =='CONFIGURE_PIN'):
            self.pin_idx = struct.unpack_from('<H', self.values.payload, 0)[0]
            sending_mode_byte = struct.unpack_from('<B', self.values.payload, 2)[0]
            self.sending_mode = SENDING_MODE_MAP[sending_mode_byte]
            self.delta_ticks = struct.unpack_from('<H', self.values.payload, 3)[0]
            if (self.sub_cmd == 'SET_PIN_ANALOG_INPUT'):
                adc_resolution_byte = struct.unpack_from('<B', self.values.payload, 5)[0]
                self.adc_bit_resolution = ADC_RESOLUTION_MAP[adc_resolution_byte]
                self.filter_order = struct.unpack_from('<B', self.values.payload, 6)[0]
                self.low_pass_cutoff = struct.unpack_from('<f', self.values.payload, 7)[0]
                self.slider_mode_enabled = bool(struct.unpack_from('<B',self.values.payload, 11)[0])
                self.slider_threshold = struct.unpack_from('<H',self.values.payload, 12)[0]

    def get_config_text(self):
        """If packet is a CONFIGURE_PIN cmd,
           return a string with human-readable configuration parameters.
        """
        if not (self.cmd == 'CONFIGURE_PIN'):
            return 'Not a configuration command packet'

        out_lines = []
        out_lines.append("Pin index: %s\n" % self.pin_idx)
        out_lines.append("Pin Type: %s\n" % self.sub_cmd)
        if (self.sub_cmd in ['SET_PIN_DIGITAL_INPUT', 'SET_PIN_ANALOG_INPUT']):
            out_lines.append("Sending mode: %s\n" % self.sending_mode)
            out_lines.append("Delta ticks sending: %s\n" % self.delta_ticks)
            if (self.sub_cmd == 'SET_PIN_ANALOG_INPUT'):
                out_lines.append("ADC bit resolution: %s\n" % self.adc_bit_resolution)
                out_lines.append("Lowpass cutoff: %s\n" % self.low_pass_cutoff)
                out_lines.append("Filter order: %s\n" % self.filter_order)
                out_lines.append("Slider mode enabled: %s\n" % self.slider_mode_enabled)
                out_lines.append("Slider threshold: %s\n" % self.slider_threshold)
        return ''.join(out_lines)

    def _parse_values(self):
        PACKET_LEN = 64
        HEADER_LEN = 3
        PAYLOAD_LEN = 49
        offset = 0
        self.values.start_header = struct.unpack_from('<3B', self.binary_data, offset)
        offset += HEADER_LEN
        self.values.cmd = struct.unpack_from('<B', self.binary_data, offset)[0]
        offset += 1
        self.values.sub_cmd = struct.unpack_from('<B', self.binary_data, offset)[0]
        offset += 1
        self.values.payload = self.binary_data[offset:offset+PAYLOAD_LEN]
        offset += PAYLOAD_LEN
        self.values.continuation = struct.unpack_from('<B', self.binary_data, offset)[0]
        offset += 1
        self.values.timestamp = struct.unpack_from('<L', self.binary_data, offset)[0]
        offset += 4
        self.values.crc = struct.unpack_from('<H', self.binary_data, offset)[0]
        offset += 2
        self.values.stop_header = struct.unpack_from('<3B', self.binary_data, offset)
        offset += HEADER_LEN
        assert(offset == PACKET_LEN)

class TeensyLogParser(object):
    """Parse a Teensy log in text format,
       represented as line of 65 values:
            - first byte represents packet source
            - next 64 bytes are packet content
    """
    PACKET_SOURCE_MAP = { 0 : 'teensy', 1 : 'linux' }
    def __init__(self, log_line):
        self.log_line = log_line
        self.log_bytes = [int(s) for s in log_line.split()]
        self.source = self.PACKET_SOURCE_MAP[self.log_bytes[0]]

    def source(self):
        return self.source

    def to_binary_dump(self):
        return struct.pack('64B', *self.log_bytes[1:])

####################################################################
#  Functions for manipulating log files with multiple packets      #
####################################################################

def filter_log_file(log_filename, cmd=None, sub_cmd=None, source='teensy'):
    """Return SenseiPacket objects from a log file with
       packets dumped in format parsable by TeensyLogParser,
       one dump for line.

        Params:
            log_filename : name of input file
            cmd : returns only packets the given cmd string
            sub_cmd : returns only packets that match the given sub_cmd string
            source : returns only packets from the given source in {'teensy', 'linux'}

        Returns:
            list of SenseiPacket objects matching the given criteria

        Note:
            cmd, sub_cmd are as written in enums inside common/sensei_serial_protocol.h,
                         e.g. CONFIGURE_PIN, SET_PIN_DIGITAL_INPUT
    """
    with open(log_filename) as infile:
        log_lines = infile.readlines()
    logs = [TeensyLogParser(l) for l in log_lines]
    packets = [SenseiPacket(l.to_binary_dump()) for l in logs if (l.source == source)]
    if cmd is not None:
        if cmd not in CMD_MAP.values():
            raise ValueError, "Invalid cmd code %s" % cmd
        packets = [p for p in packets if (p.cmd == cmd)]
    if sub_cmd is not None:
        if cmd is None:
            raise ValueError, "Can't specify sub_cmd without cmd field"
        if sub_cmd not in SUB_CMD_MAP[cmd].keys():
            raise ValueError, "Invalid cmd code %s" % cmd
        packets = [p for p in packets if (p.sub_cmd == sub_cmd)]

    return packets

