import math
import sys
import threading
import time

import gpio_protocol
from raspa_mock import OscReceiver, SenseiProcess, SenseiClient, RaspaServer, logger

RASPA_SOCKET = "/tmp/raspa"
SENSEI_SOCKET = "/tmp/sensei"
SENSEI_BINARY = "../../../build/sensei"
SENSEI_CONFIG = "sensei_config.json"
OSC_HOST = "127.0.0.1"
OSC_PORT = 23000

def main():
    """Main entry point."""
    logger.info("Starting raspa_mock...")

    # Start OSC receiver first
    osc_receiver = OscReceiver(OSC_HOST, OSC_PORT)
    if not osc_receiver.start():
        logger.error("Failed to start OSC receiver. Exiting.")
        return 1

    # Start sensei process
    sensei_process = SenseiProcess(SENSEI_BINARY, SENSEI_CONFIG)
    if not sensei_process.start():
        logger.error("Failed to start sensei process. Exiting.")
        osc_receiver.stop()
        return 1

    # Create sensei client
    sensei_client = SenseiClient(SENSEI_SOCKET)

    # Start connection to sensei in background
    sensei_thread = threading.Thread(target=sensei_client.connect, daemon=True)
    sensei_thread.start()

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

        # should receive the value over OSC
        osc_receiver.event.clear()
        send_value(3, 128)
        osc_receiver.event.wait()
        assert(math.isclose(osc_receiver.last_message[1], 128/255, abs_tol=1e-6))

        # also this value
        osc_receiver.event.clear()
        send_value(2, 0)
        osc_receiver.event.wait()
        assert(math.isclose(osc_receiver.last_message[1], 0, abs_tol=1e-6))

        # this is the same value we had previously, no OSC event and wait should timeout
        osc_receiver.event.clear()
        send_value(3, 128)
        assert(not osc_receiver.event.wait(1.0))

        # the new value should generate an event
        osc_receiver.event.clear()
        send_value(3, 10)
        osc_receiver.event.wait()
        assert(math.isclose(osc_receiver.last_message[1], 10/255, abs_tol=1e-6))

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
        osc_receiver.stop()
        logger.info("Shutdown complete")

    return 0

if __name__ == "__main__":
    sys.exit(main())
