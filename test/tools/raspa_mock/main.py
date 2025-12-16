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

        def check_value(controller_id, mcu_value, grpc_value):
            event.clear()
            send_value(controller_id, mcu_value)
            event.wait()
            assert(math.isclose(event_value(), grpc_value, abs_tol=1e-6))

        def check_no_value(controller_id, mcu_value):
            event.clear()
            send_value(controller_id, mcu_value)
            assert(not event.wait(1.0))

        def test_pots():
            # from sensei_config.json
            controller_id = 7
            adc_resolution = 8

            max_value = pow(2, adc_resolution) - 1

            # should receive the value from the backend
            check_value(controller_id, 128, 128/max_value)

            # this is the same value we had previously, no event and wait should timeout
            check_no_value(controller_id, 128)

            # the new value should generate an event
            check_value(controller_id, 10, 10/max_value)

        def test_buttons():
            # from sensei_config.json
            controller_id = 5

            check_value(controller_id, 0, 0)
            check_value(controller_id, 1, 1)
            check_value(controller_id, -1, 0)
            check_value(controller_id, 500, 1)

        def test_switches():
            # from sensei_config.json
            controller_id = 13

            # range is 1-4
            check_value(controller_id, 0, 1)
            check_no_value(controller_id, 1)
            check_value(controller_id, 2, 2)
            check_value(controller_id, 3, 3)
            check_value(controller_id, 4, 4)
            check_no_value(controller_id, 12)

        def test_encoders():
            # from sensei_config.json
            analog_controller_id = 14
            range_controller_id = 15

            # range is 0-15 for both
            max_value = 15

            # this encoder is specified to use an analog_input sensor
            check_value(analog_controller_id, 1, 1/max_value)
            check_value(analog_controller_id, 0, 0)
            check_value(analog_controller_id, 5, 5/max_value)
            check_value(analog_controller_id, 250, 1)

            # this encoder is specified to use a range_input sensor
            check_value(range_controller_id, 1, 1)
            check_value(range_controller_id, 0, 0)
            check_value(range_controller_id, 5, 5)
            check_value(range_controller_id, 250, 15)

        test_pots()
        test_buttons()
        test_switches()
        test_encoders()

        logger.info("------")
        logger.info("Success, press Ctrl-C to quit...")

        while raspa_server.running:
            raspa_server.listen_once()

    except KeyboardInterrupt:
        logger.info("\nShutting down...")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
    finally:
        sensei_client.close()
        grpc_receiver.stop()
        osc_receiver.stop()
        sensei_process.stop()
        raspa_server.stop()
        logger.info("Shutdown complete")

    return 0

if __name__ == "__main__":
    sys.exit(main())
