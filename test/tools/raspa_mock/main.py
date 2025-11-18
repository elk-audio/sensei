import math
import sys
import threading
import time

import gpio_protocol
from raspa_mock import OscReceiver, GrpcReceiver, SenseiProcess, SenseiClient, RaspaServer, logger

RASPA_SOCKET = "/tmp/raspa"
SENSEI_SOCKET = "/tmp/sensei"
SENSEI_BINARY = "../../../build/sensei"
SENSEI_CONFIG = "sensei_config.json"
OSC_HOST = "127.0.0.1"
OSC_PORT = 23000
GRPC_HOST = "localhost"
GRPC_PORT = 50051

use_grpc = True

def main():
    """Main entry point."""
    logger.info("Starting raspa_mock...")

    # Start sensei process
    sensei_process = SenseiProcess(SENSEI_BINARY, SENSEI_CONFIG)
    if not sensei_process.start():
        logger.error("Failed to start sensei process. Exiting.")
        return 1

    # Create sensei client
    sensei_client = SenseiClient(SENSEI_SOCKET)

    # Start connection to sensei in background
    sensei_thread = threading.Thread(target=sensei_client.connect, daemon=True)
    sensei_thread.start()

    # wait for connection to sensei
    sensei_client.connected_event.wait()

    logger.info("connected, sleeping 10 seconds...")
    #time.sleep(10)

    # Start OSC receiver first
    osc_receiver = OscReceiver(OSC_HOST, OSC_PORT)
    if not osc_receiver.start():
        logger.error("Failed to start OSC receiver. Exiting.")
        sensei_process.stop()
        return 1

    # Start gRPC receiver
    grpc_receiver = GrpcReceiver(GRPC_HOST, GRPC_PORT)
    if not grpc_receiver.start():
        logger.error("Failed to start gRPC receiver. Exiting.")
        sensei_process.stop()
        osc_receiver.stop()
        return 1

    # Create and start raspa server
    raspa_server = RaspaServer(RASPA_SOCKET, sensei_client)

    try:
        # this runs until we receive a START_SYSTEM message from sensei
        raspa_server.start()

        def send_value(controller_id, value):
            packet = gpio_protocol.GpioPacket()
            packet.command = gpio_protocol.GPIO_CMD_GET_VALUE
            packet.payload.gpio_value_data.controller_id = controller_id

            # for the pot, this value seems to be divided by 255 to get a float
            packet.payload.gpio_value_data.controller_val = value

            # Get current time in microseconds (equivalent to C++ chrono)
            packet.timestamp = int(time.time() * 1_000_000)

            packet_bytes = bytes(packet)
            logger.info(f"Sending from raspa: ({controller_id}, {value})")
            raspa_server.sensei_client.send(packet_bytes)

        event = osc_receiver.event
        if use_grpc:
            event = grpc_receiver.event

        def event_value():
            if use_grpc:
                return grpc_receiver.last_message[1]
            else:
                return osc_receiver.last_message[1]

        # this comes from sensei_config.json
        controller_3_adc_resolution = 8
        controller_3_denominator = pow(2, controller_3_adc_resolution) - 1

        # should receive the value from the backend
        event.clear()
        send_value(3, 128)
        event.wait()
        assert(math.isclose(event_value(), 128/controller_3_denominator, abs_tol=1e-6))

        # also this value
        event.clear()
        send_value(2, 0)
        event.wait()
        assert(math.isclose(event_value(), 0, abs_tol=1e-6))

        # this is the same value we had previously, no event and wait should timeout
        event.clear()
        send_value(3, 128)
        assert(not event.wait(1.0))

        # the new value should generate an event
        event.clear()
        send_value(3, 10)
        event.wait()
        assert(math.isclose(event_value(), 10/controller_3_denominator, abs_tol=1e-6))

        while raspa_server.running:
            raspa_server.listen_once()

    except KeyboardInterrupt:
        logger.info("\nShutting down...")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        raspa_server.stop()
        sensei_client.close()
        sensei_process.stop()
        grpc_receiver.stop()
        osc_receiver.stop()
        logger.info("Shutdown complete")

    return 0

if __name__ == "__main__":
    sys.exit(main())
