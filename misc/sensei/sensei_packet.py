"""sensei_packet.py: classes for parsing and creating serial packets"""
__copyright__   = "Copyright (C) 2016 MIND Music Labs"

import struct
import array as pyarray

import numpy as np

import sensei_protocol_defs as spd

#########################################################
#  Meta stuff for compact Class Messages definition     #
#########################################################

class PayloadDescriptor(object):
    """Describe a field in message payload.
    """
    def __init__(self,
                 name,
                 type_fmt,
                 default_val=0,
                 enum_map=None):
        """Inputs:
                name        : name of attribute to initialize in Message subclasses
                type_fmt    : format identifier as in struct module, e.g. 'B', 'H', 'i'
                default_val : default value for member variables
                enum_map    : use given dictionary to convert from byte values to e.g. strings
        """
        self.name = name
        self.type_fmt = type_fmt
        self.default_val = default_val
        self.enum_map = enum_map
        if self.enum_map is not None:
            self.inv_enum_map = { v:k for k,v in self.enum_map.items() }

    def byte_value(self, value):
        if self.enum_map is None:
            return value
        else:
            return self.inv_enum_map[value]

    def symbolic_value(self, value):
        if self.enum_map is None:
            return value
        else:
            return self.enum_map[value]

# This get auto-populated every time a Message class is defined (metaclass magic ;)
COMMAND_FACTORY_MAP = { }

class MessageInterface(type):
    """Metaclass for having a compact definition of Message sub-classes and auto
       registration in factory dictionary.
    """
    def __new__(cls, name, parents, dct):
        # Prepare some convenience dicts for faster access
        dct['payload_map'] = { p.name:p for p in dct['payload_descriptors'] }
        _payload_fmt = '<' + ''.join([p.type_fmt for p in dct['payload_descriptors']])
        fields_size = struct.calcsize(_payload_fmt)
        dct['zeropad_size'] = spd.PAYLOAD_SIZE - fields_size
        dct['payload_fmt'] = _payload_fmt + '%dB' % dct['zeropad_size']
        # Register class in factory map
        new_class = super(MessageInterface, cls).__new__(cls, name, parents, dct)
        COMMAND_FACTORY_MAP[dct['cmd']] = new_class
        return new_class

class BaseMessage(object):
    """Common base class for all Message classes.
    """
    __metaclass__ = MessageInterface
    cmd = None
    payload_descriptors = []

    def __init__(self,
                 packet=None,
                 sub_cmd='EMPTY',
                 **kwargs):
        """Initialize a Message instance in one of the two ways:
           If packet is not None, parse the payload from the given instance of SenseiPacket

           Otherwise, create an instance from the given sub_cmd and keyword args corresponding
                      to the field descriptors in the subclasses.
        """
        # Set up descriptor fields for direct access
        for p in self.payload_descriptors:
            self.__setattr__(p.name, p.default_val)
        for k, v in kwargs.items():
            if k in self.payload_map.keys():
                self.__setattr__(k, v)
            else:
                raise TypeError('Invalid keywoard argument for payload field %s' % k)

        self.packet = packet
        if (packet is not None):
            self.sub_cmd = self.packet.sub_cmd
            self._initialize_from_binary_payload(self.packet.payload)
        else:
            if not(sub_cmd in spd.SUB_CMD_MAP[self.cmd].values()):
                raise TypeError('Invalid sub_cmd %s for cmd %s' % (sub_cmd, self.cmd))
            self.sub_cmd = sub_cmd
            self._create_payload()

    def __repr__(self):
        repr_str = "\n%s\n" % self.__class__.__name__
        repr_str += 'sub_cmd : %s\n' % self.sub_cmd
        fields_str = [ '    %s : %s' % (p.name, self.__getattribute__(p.name)) for p in self.payload_descriptors ]
        repr_str += '\n'.join(fields_str) + '\n'
        return repr_str

    def _create_payload(self):
        init_values = [ pd.byte_value(self.__getattribute__(pd.name)) for pd in self.payload_descriptors ]
        init_values.extend(np.zeros(self.zeropad_size, dtype=np.uint8))
        self.payload = struct.pack(self.payload_fmt, *init_values)

    def _initialize_from_binary_payload(self, binary_payload):
        self.payload = binary_payload
        offset = 0
        for pd in self.payload_descriptors:
            byte_value = struct.unpack_from('<' + pd.type_fmt, self.payload, offset)[0]
            self.__setattr__(pd.name, pd.symbolic_value(byte_value))
            offset += struct.calcsize(pd.type_fmt)

#############################################
#  Command classes
#############################################

class AckMessage(BaseMessage):
    cmd = 'ACK'
    payload_descriptors = [ PayloadDescriptor('status', 'i', 'OK', spd.ERROR_CODE_MAP),
                            PayloadDescriptor('origin_timestamp', 'I'),
                            PayloadDescriptor('origin_cmd', 'B', 'HELLO', spd.CMD_MAP),
                            PayloadDescriptor('origin_sub_cmd', 'B')
                          ]

class ValueMessage(BaseMessage):
    cmd = 'VALUE'
    payload_descriptors = [ PayloadDescriptor('pin_idx', 'H'),
                            PayloadDescriptor('value', 'H'),
                            PayloadDescriptor('pin_type', 'B', 'ANALOG_INPUT', spd.PIN_TYPE_MAP)
                          ]

class SetDigitalMessage(BaseMessage):
    cmd = 'SET_DIGITAL_PINS'
    payload_descriptors = [ PayloadDescriptor('pin_idx', 'H'),
                            PayloadDescriptor('value', 'B')
                          ]

class SystemInitializationMessage(BaseMessage):
    cmd = 'INITIALIZE_SYSTEM'
    payload_descriptors = [ PayloadDescriptor('sampling_rate_divider', 'B', 1),
                            PayloadDescriptor('n_pins', 'H', 64),
                            PayloadDescriptor('n_digital_outputs', 'H', 64)
                          ]

class EnableSendingPacketsMessage(BaseMessage):
    cmd = 'ENABLE_SENDING_PACKETS'
    payload_descriptors = [ PayloadDescriptor('sending_enabled', 'B', 1) ]

class ConfigurePinMessage(BaseMessage):
    cmd = 'CONFIGURE_PIN'
    payload_descriptors = [ PayloadDescriptor('pin_idx', 'H'),
                            PayloadDescriptor('sending_mode', 'B', 'SENDING_MODE_ON_VALUE_CHANGED', spd.SENDING_MODE_MAP),
                            PayloadDescriptor('delta_ticks', 'H', 1),
                            PayloadDescriptor('adc_bit_resolution', 'B', 10, spd.ADC_RESOLUTION_MAP),
                            PayloadDescriptor('filter_order', 'B', 4),
                            PayloadDescriptor('lowpass_cutoff', 'f', 20.0),
                            PayloadDescriptor('slider_mode_enabled', 'B'),
                            PayloadDescriptor('slider_threshold', 'H')
                          ]

COMMAND_FACTORY_INV_MAP = {v:k for k,v in COMMAND_FACTORY_MAP.items()}

#################################
#  Generic Packet Parser Class  #
#################################

class SenseiPacket(object):
    """Parse dump of a 64-bytes Sensei message,
       checking start/stop header signatures and
       delegating payload parsing to one of the Message classes.

       Public members accessible after initialization:

           dump             : 64 bytes string with packet contents on the wire
           data             : 64 numpy array with data encoded as np.uint8
           message          : instance of one of the message classes
           start_header     : len-3 tuple with start header numbers
           stop_header      : len-3 tuple with stop header numbers
           cmd              : string with command name as in C enum
           sub_cmd          : string with sub-command name as in C enum
           payload          : 49 bytes string with binary payload
           continuation     : n. of continuation messages
           timestamp        : timestamp in microseconds
           crc              : packet crc
    """
    def __init__(self, binary_dump=None,
                       message=None,
                       timestamp=0):
        """Create a packet even from a given 64-bytes string dump,
           or from an instance of one of the Message classes
        """
        if (binary_dump is not None) and (message is not None):
            raise TypeError('Must provide only one of binary_dump or message')
        elif (binary_dump is not None):
            self.message = None
            self._initialize_from_binary_dump(binary_dump)
        elif (message is not None):
            self.timestamp = timestamp
            self._initialize_from_message(message)
        else:
            raise TypeError('At least one of binary_dump and message must be not None')

        self.data = np.array(struct.unpack('%dB' % spd.PACKET_SIZE, self.binary_dump), dtype=np.uint8)
        self._validate_dump()

    def __repr__(self):
        repr_str = '\n#########################################################\n'
        repr_str += "%s: \n" % self.__class__.__name__
        fields_str = ['cmd = %s' % self.cmd,
                      'sub_cmd = %s' % self.sub_cmd,
                      'timestamp = %s' % self.timestamp,
                      'crc = %s' % self.crc,
                      '---------------------------------------------------------',
                      'Message :',
                      '%s' % self.message]
        repr_str += '\n'.join(fields_str) + '\n'
        return repr_str

    def to_c_array(self, var_prefix='packet_dump'):
        """Return a string with C source code for an array declaration
           of this packet.
        """
        N_VAL_PER_LINES = 8
        assert((len(self.binary_dump) % N_VAL_PER_LINES) == 0)
        out_lines = []
        out_lines.append('static uint8_t %s[] = {\n' % var_prefix)
        n_lines = len(self.binary_dump) / N_VAL_PER_LINES
        for n in range(n_lines):
            low = n*N_VAL_PER_LINES
            high = (n+1)*N_VAL_PER_LINES
            line_bytes = [ struct.unpack('B', x)[0] for x in self.binary_dump[low:high] ]
            line = '    ' + ', '.join(["0x%02x" % b for b in line_bytes]) + ',    // %02d .. %02d\n' % (low, high-1)
            out_lines.append(line)
        out_lines.append('};\n')
        return ''.join(out_lines)

    def _initialize_from_binary_dump(self, binary_dump):
        self.binary_dump = binary_dump
        header_len = len(spd.EXPECTED_START_HEADER)
        # Parse generic packet info
        offset = 0
        self.start_header = struct.unpack_from('<%dB' % header_len, self.binary_dump, offset)
        offset += header_len
        self.cmd = spd.CMD_MAP[struct.unpack_from('<B', self.binary_dump, offset)[0]]
        offset += 1
        self.sub_cmd = spd.SUB_CMD_MAP[self.cmd][struct.unpack_from('<B', self.binary_dump, offset)[0]]
        offset += 1
        self.payload = self.binary_dump[offset:offset+spd.PAYLOAD_SIZE]
        offset += spd.PAYLOAD_SIZE
        self.continuation = struct.unpack_from('<B', self.binary_dump, offset)[0]
        offset += 1
        self.timestamp = struct.unpack_from('<L', self.binary_dump, offset)[0]
        offset += 4
        self.crc = struct.unpack_from('<H', self.binary_dump, offset)[0]
        offset += 2
        self.stop_header = struct.unpack_from('<%dB' % header_len, self.binary_dump, offset)
        offset += header_len
        assert(offset == spd.PACKET_SIZE)

    def _validate_dump(self):
        if not (self.start_header == spd.EXPECTED_START_HEADER):
            raise ValueError, "Wrong start header: %s" % self.start_header
        if not (self.stop_header == spd.EXPECTED_STOP_HEADER):
            raise ValueError, "Wrong stop header: %s" % self.stop_header
        computed_crc = np.uint16(np.sum(self.data[3:59]))
        if (computed_crc != self.crc):
            raise ValueError, "Wrong CRC found in packet: %s" % self.crc

        if (self.message is None):
            type_factory = COMMAND_FACTORY_MAP[self.cmd]
            self.message = type_factory(packet=self)

    def _initialize_from_message(self, message):
        self.message = message
        self.cmd = COMMAND_FACTORY_INV_MAP[type(message)]
        self.sub_cmd = self.message.sub_cmd
        self.payload = self.message.payload
        self.start_header = spd.EXPECTED_START_HEADER
        self.stop_header = spd.EXPECTED_STOP_HEADER
        # Use array to index intermediate data in an easier way than with struct
        temp_dump = pyarray.array('B', np.zeros(spd.PACKET_SIZE, dtype=np.uint8))
        temp_dump[:3] = pyarray.array('B', self.start_header)
        temp_dump[3] = spd.CMD_INV_MAP[self.cmd]
        subcmd_inv_map = {v:k for k,v in spd.SUB_CMD_MAP[self.cmd].items()}
        temp_dump[4] = subcmd_inv_map[self.sub_cmd]
        temp_dump[5:54] = pyarray.array('B', self.payload)
        # temp_dump[54] is continuation = 0
        temp_dump[55:59] = pyarray.array('B', struct.pack('<I', self.timestamp))
        self.crc = sum(temp_dump[3:59])
        temp_dump[59:61] = pyarray.array('B', struct.pack('<H', self.crc))
        temp_dump[61:64] = pyarray.array('B', self.stop_header)
        self.binary_dump = temp_dump.tostring()

