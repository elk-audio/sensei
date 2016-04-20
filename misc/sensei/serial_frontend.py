import time

import serial

import sensei_packet as sp
import sensei_protocol_defs as spd

DEFAULT_WRITE_INTERVAL_MS = 20
DEFAULT_READ_INTERVAL_MS = 5
DEFAULT_RECEIVE_TIMEOUT_SECS = 5

class SerialFrontend(object):
    def __init__(self,
                 port_name,
                 sampling_rate_divider=1,
                 n_pins=64,
                 n_digital_outputs=32,
                 serial_timeout_ms=DEFAULT_READ_INTERVAL_MS):
        self.port_name = port_name
        self.n_packets_sent = 0
        self.n_packets_received = 0
        self.port = serial.Serial(self.port_name, timeout=serial_timeout_ms*1.0e-3)
        self.reset_buffers()
        init_msg = sp.SystemInitializationMessage(sampling_rate_divider=sampling_rate_divider,
                                                  n_pins=n_pins,
                                                  n_digital_outputs=n_digital_outputs)
        self.send_messages([init_msg])

    def close(self):
        self.reset_buffers()
        self.port.close()

    def reset_buffers(self):
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

    def send_messages(self,
                      messages,
                      timestamp=0,
                      write_interval_ms=DEFAULT_WRITE_INTERVAL_MS):
        """Create packets with the given messages and send them
           periodically with the given interval in milliseconds.
        """
        for msg in messages:
            packet = sp.SenseiPacket(message=msg, timestamp=timestamp)
            self._send_packet(packet)
            time.sleep(write_interval_ms * 1.0e-3)

    def enable_sending_packets(self, enabled=True):
        """Send packet to enable/disable data flow from Teensy."""
        packet = sp.SenseiPacket(message=sp.EnableSendingPacketsMessage(sending_enabled=enabled))
        self._send_packet(packet)

    def receive_messages(self,
                         packet_type=None,
                         max_n_packets=1,
                         timeout_secs=DEFAULT_RECEIVE_TIMEOUT_SECS):
        msgs_received = []
        cur_n_packets_received = 0
        start_time = time.time()
        time_elapsed = 0
        while ((cur_n_packets_received < max_n_packets) and (time_elapsed < timeout_secs)):
            time_elapsed = time.time() - start_time
            new_dump = self.port.read(spd.PACKET_SIZE)
            n_bytes_received = len(new_dump)
            if (n_bytes_received == 0):
                continue
            if (n_bytes_received < spd.PACKET_SIZE):
                raise ValueError, "Received only %s bytes" % n_bytes_received
            packet = sp.SenseiPacket(binary_dump=new_dump)
            if ((packet_type is None) or (packet.cmd == packet_type)):
                msgs_received.append(packet.message)
                cur_n_packets_received += 1

        self.n_packets_received += cur_n_packets_received
        return msgs_received

    def _send_packet(self, packet):
        n_bytes_sent = self.port.write(packet.binary_dump)
        if (n_bytes_sent != spd.PACKET_SIZE):
            raise IOError('Error writing to device, only sent %s bytes' % n_bytes_sent)
        self.n_packets_sent += 1
