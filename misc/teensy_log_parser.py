import struct

import sensei
import sensei.sensei_protocol_defs as spd

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
    packets = [sensei.SenseiPacket(l.to_binary_dump()) for l in logs if (l.source == source)]
    if cmd is not None:
        if cmd not in spd.CMD_MAP.values():
            raise ValueError, "Invalid cmd code %s" % cmd
        packets = [p for p in packets if (p.cmd == cmd)]
    if sub_cmd is not None:
        if cmd is None:
            raise ValueError, "Can't specify sub_cmd without cmd field"
        if sub_cmd not in spd.SUB_CMD_MAP[cmd].keys():
            raise ValueError, "Invalid cmd code %s" % cmd
        packets = [p for p in packets if (p.sub_cmd == sub_cmd)]

    return packets

