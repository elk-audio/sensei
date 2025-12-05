#!/usr/bin/env python3
"""
Socket bridge between /tmp/raspa (server) and /tmp/sensei (client).
Listens for connections on /tmp/raspa and forwards messages to /tmp/sensei.
"""

import socket
import os
import time
import threading
import logging
import subprocess

import gpio_protocol
from pythonosc import dispatcher, osc_server

import grpc
import sensei_rpc_pb2
import sensei_rpc_pb2_grpc

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

RECONNECT_DELAY = 2  # seconds
BUFFER_SIZE = 4096


class OscReceiver:
    """Receives OSC messages from sensei."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server = None
        self.server_thread = None
        self.last_message = (None, 0)
        self.event = threading.Event()

    def _osc_handler(self, address, *args):
        """Handle incoming OSC messages."""
        logger.info(f"[OSC] {address} {args}")
        self.last_message = (address, args[0])
        self.event.set()

    def start(self):
        """Start the OSC server."""
        try:
            # Create dispatcher and register handlers
            disp = dispatcher.Dispatcher()
            # Catch all messages under /sensors
            disp.map("/sensors/*", self._osc_handler)
            # Also catch any other messages
            disp.set_default_handler(self._osc_handler)

            # Create and start server
            self.server = osc_server.ThreadingOSCUDPServer(
                (self.host, self.port), disp
            )

            logger.info(f"Starting OSC server on {self.host}:{self.port}")

            # Run server in background thread
            self.server_thread = threading.Thread(
                target=self.server.serve_forever,
                daemon=True
            )
            self.server_thread.start()

            logger.info("OSC server started")
            return True

        except Exception as e:
            logger.error(f"Failed to start OSC server: {e}")
            return False

    def stop(self):
        """Stop the OSC server."""
        if self.server:
            logger.info("Stopping OSC server...")
            self.server.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=2)
            logger.info("OSC server stopped")


class GrpcReceiver:
    """Receives gRPC messages from sensei."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.channel = None
        self.stub = None
        self.receiver_thread = None
        self.running = False
        self.last_message = (None, 0)
        self.event = threading.Event()

    def _grpc_event_handler(self, event):
        """Handle incoming gRPC events."""
        # Determine event type and extract data
        event_type = event.WhichOneof('event')

        if event_type == 'analog_ev':
            ev = event.analog_ev
            logger.info(f"[gRPC] AnalogEvent controller_id={event.controller_id} timestamp={event.timestamp} value={ev.value}")
            self.last_message = (f"analog_{event.controller_id}", ev.value)

        elif event_type == 'toggle_ev':
            ev = event.toggle_ev
            logger.info(f"[gRPC] ToggleEvent controller_id={event.controller_id} timestamp={event.timestamp} value={ev.value}")
            self.last_message = (f"toggle_{event.controller_id}", ev.value)

        elif event_type == 'relative_ev':
            ev = event.relative_ev
            logger.info(f"[gRPC] RelativeEvent controller_id={event.controller_id} timestamp={event.timestamp} value={ev.value}")
            self.last_message = (f"relative_{event.controller_id}", ev.value)

        elif event_type == 'range_ev':
            ev = event.range_ev
            logger.info(f"[gRPC] RangeEvent controller_id={event.controller_id} timestamp={event.timestamp} value={ev.value}")
            self.last_message = (f"range_{event.controller_id}", ev.value)

        self.event.set()

    def _receive_events(self):
        """Background thread to receive events from gRPC stream."""
        try:
            # Create subscribe request (no filters = receive all events)
            request = sensei_rpc_pb2.SubscribeRequest()

            logger.info(f"Subscribing to gRPC events on {self.host}:{self.port}")

            # Start streaming - this blocks until stream ends
            for event in self.stub.SubscribeToEvents(request):
                if not self.running:
                    break
                self._grpc_event_handler(event)

        except grpc.RpcError as e:
            if self.running:  # Only log if we didn't intentionally stop
                logger.error(f"gRPC error: {e.code()} - {e.details()}")
        except Exception as e:
            logger.error(f"Error receiving gRPC events: {e}")
        finally:
            logger.info("gRPC event receiver stopped")

    def start(self):
        """Start the gRPC client."""
        try:
            # Create channel and stub
            self.channel = grpc.insecure_channel(f'{self.host}:{self.port}')
            self.stub = sensei_rpc_pb2_grpc.PinProxyServiceStub(self.channel)

            logger.info(f"Connecting to gRPC server at {self.host}:{self.port}")

            # Start receiver thread
            self.running = True
            self.receiver_thread = threading.Thread(
                target=self._receive_events,
                daemon=True
            )
            self.receiver_thread.start()

            logger.info("gRPC receiver started")
            return True

        except Exception as e:
            logger.error(f"Failed to start gRPC receiver: {e}")
            return False

    def stop(self):
        """Stop the gRPC client."""
        logger.info("Stopping gRPC receiver...")
        self.running = False

        if self.channel:
            self.channel.close()

        if self.receiver_thread:
            self.receiver_thread.join(timeout=2)

        logger.info("gRPC receiver stopped")


class SenseiProcess:
    """Manages the sensei subprocess lifecycle."""

    def __init__(self, binary_path, config_path):
        self.binary_path = binary_path
        self.config_path = config_path
        self.process = None

    def start(self):
        """Start the sensei process."""
        try:
            logger.info(f"Starting sensei: {self.binary_path} -f {self.config_path}")
            self.process = subprocess.Popen(
                [self.binary_path, "-f", self.config_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )

            # Start threads to log output
            threading.Thread(
                target=self._log_output,
                args=(self.process.stdout, "SENSEI-OUT"),
                daemon=True
            ).start()
            threading.Thread(
                target=self._log_output,
                args=(self.process.stderr, "SENSEI-ERR"),
                daemon=True
            ).start()

            # Give sensei a moment to start up and create its socket
            time.sleep(0.5)

            # Check if process is still running
            if self.process.poll() is not None:
                logger.error(f"Sensei process exited immediately with code {self.process.returncode}")
                return False

            logger.info(f"Sensei process started (PID: {self.process.pid})")
            return True

        except FileNotFoundError:
            logger.error(f"Sensei binary not found at {self.binary_path}")
            return False
        except Exception as e:
            logger.error(f"Failed to start sensei: {e}")
            return False

    def _log_output(self, stream, prefix):
        """Log output from sensei process."""
        try:
            for line in stream:
                line = line.rstrip()
                if line:
                    logger.info(f"[{prefix}] {line}")
        except Exception as e:
            logger.error(f"Error reading {prefix}: {e}")

    def stop(self):
        """Stop the sensei process."""
        if self.process:
            logger.info("Stopping sensei process...")
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
                logger.info("Sensei process terminated")
            except subprocess.TimeoutExpired:
                logger.warning("Sensei didn't terminate, killing...")
                self.process.kill()
                self.process.wait()
            self.process = None

    def is_running(self):
        """Check if the sensei process is running."""
        return self.process is not None and self.process.poll() is None


class SenseiClient:
    """Manages connection to /tmp/sensei socket with automatic reconnection."""

    def __init__(self, socket_path):
        self.socket_path = socket_path
        self.sock = None
        self.connected = False
        self.lock = threading.Lock()
        self.connected_event = threading.Event()

    def connect(self):
        """Connect to sensei socket with retry logic."""
        while not self.connected:
            try:
                if not os.path.exists(self.socket_path):
                    logger.warning(f"Socket {self.socket_path} does not exist. Waiting...")
                    time.sleep(RECONNECT_DELAY)
                    continue

                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.sock.connect(self.socket_path)
                self.connected = True
                self.connected_event.set()
                logger.info(f"Connected to {self.socket_path}")
                return True

            except (ConnectionRefusedError, FileNotFoundError, OSError) as e:
                logger.warning(f"Failed to connect to {self.socket_path}: {e}. Retrying in {RECONNECT_DELAY}s...")
                if self.sock:
                    self.sock.close()
                    self.sock = None
                time.sleep(RECONNECT_DELAY)

        return False

    def send(self, data):
        """Send data to sensei socket."""
        with self.lock:
            if not self.connected:
                logger.error("Cannot send: not connected to sensei")
                return False

            try:
                self.sock.send(data) # type: ignore
                logger.debug(f"Sent {len(data)} bytes to sensei")
                return True
            except OSError as e:
                logger.error(f"Error sending to sensei: {e}")
                self.connected = False
                # Reconnect in background
                threading.Thread(target=self.connect, daemon=True).start()
                return False

    def close(self):
        """Close the connection."""
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None
            self.connected = False


class RaspaServer:
    """Server listening on /tmp/raspa socket."""

    def __init__(self, socket_path, sensei_client):
        self.socket_path = socket_path
        self.sensei_client = sensei_client
        self.server_sock = None
        self.running = False
        self.started = False

    def start(self):
        """Start the server socket."""
        # Remove existing socket file if it exists
        if os.path.exists(self.socket_path):
            logger.info(f"Removing existing socket at {self.socket_path}")
            os.remove(self.socket_path)

        # Create and bind server socket
        self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.server_sock.bind(self.socket_path)
        self.server_sock.settimeout(1.0)  # 1 second timeout
        self.running = True

        logger.info(f"Server listening on {self.socket_path}")

        # Receive datagrams
        while self.running and not self.started:
            self.listen_once()

    def listen_once(self):
        try:
            data = self.server_sock.recv(BUFFER_SIZE) # type: ignore
            if data:
                logger.info(f"Received {len(data)} bytes")

                # Parse received data into GpioPacket structure
                packet = gpio_protocol.GpioPacket.from_buffer_copy(data)
                logger.info(f"Packet: cmd={packet.command}, sub_cmd={packet.sub_command}, seq={packet.sequence_no}")

                ack = gpio_protocol.GpioPacket()
                ack.command = gpio_protocol.GPIO_ACK
                ack.payload.gpio_ack_data.returned_seq_no = packet.sequence_no

                # Send ack to sensei
                if self.sensei_client.connected:
                    # Convert ctypes structure to bytes
                    ack_bytes = bytes(ack)
                    self.sensei_client.send(ack_bytes)
                else:
                    logger.warning("Cannot ack message: sensei not connected")

                if packet.command == gpio_protocol.GPIO_CMD_SYSTEM_CONTROL and \
                    packet.sub_command == gpio_protocol.GPIO_SUB_CMD_START_SYSTEM:
                    logger.info("Received START_SYSTEM command")
                    self.started = True

        except socket.timeout:
            # Timeout is normal, just continue the loop
            logger.info("recv timeout...")
        except OSError as e:
            if self.running:
                logger.error(f"Error receiving datagram: {e}")

    def stop(self):
        """Stop the server."""
        self.running = False
        if self.server_sock:
            self.server_sock.close()
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)
        logger.info("Server stopped")
