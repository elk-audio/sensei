import math
import pytest
import threading
import time

import gpio_protocol
from raspa_mock import (
    GrpcReceiver,
    SenseiProcess,
    RaspaApp,
    logger,
)

RASPA_SOCKET = "/tmp/raspa"
SENSEI_SOCKET = "/tmp/sensei"
SENSEI_BINARY = "../../../build/sensei"
SENSEI_CONFIG = "sensei_config.json"
GRPC_HOST = "localhost"
GRPC_PORT = 50051

class SenseiFixture:
    def __init__(self):
        self.sensei_process = SenseiProcess(SENSEI_BINARY, SENSEI_CONFIG)
        self.raspa_app = RaspaApp(RASPA_SOCKET, SENSEI_SOCKET)
        self.grpc_receiver = GrpcReceiver(GRPC_HOST, GRPC_PORT)

    def setup(self):
        if not self.sensei_process.start():
            raise RuntimeError("Failed to start sensei process")
        if not self.raspa_app.start():
            raise RuntimeError("Failed to start raspa app")
        if not self.grpc_receiver.start():
            raise RuntimeError("Failed to start gRPC receiver")

    def teardown(self):
        self.grpc_receiver.stop()
        self.raspa_app.stop()
        self.sensei_process.stop()


@pytest.fixture
def sensei():
    fixture = SenseiFixture()
    fixture.setup()
    yield fixture
    fixture.teardown()

def test_raspa_start_stop(sensei):
    for _ in range(0, 3):
        sensei.raspa_app.stop()
        sensei.raspa_app.start()

def test_sensor_values(sensei):
    def send_value(controller_id, value):
        packet = gpio_protocol.GpioPacket()
        packet.command = gpio_protocol.GPIO_CMD_GET_VALUE
        packet.payload.gpio_value_data.controller_id = controller_id
        packet.payload.gpio_value_data.controller_val = value
        packet.timestamp = int(time.time() * 1_000_000)
        logger.info(f"Sending from raspa: ({controller_id}, {value})")
        sensei.raspa_app.send_to_sensei(bytes(packet))

    event = sensei.grpc_receiver.event

    def event_value():
        return sensei.grpc_receiver.last_message[1]

    def check_value(controller_id, mcu_value, expected_value):
        event.clear()
        send_value(controller_id, mcu_value)
        event.wait()
        assert math.isclose(event_value(), expected_value, abs_tol=1e-6)

    def check_no_value(controller_id, mcu_value):
        event.clear()
        send_value(controller_id, mcu_value)
        assert not event.wait(1.0)

    def test_pots():
        controller_id = 7
        adc_resolution = 8
        max_value = pow(2, adc_resolution) - 1

        check_value(controller_id, 128, 128 / max_value)
        check_no_value(controller_id, 128)
        check_value(controller_id, 10, 10 / max_value)

    def test_buttons():
        controller_id = 5

        check_value(controller_id, 0, 0)
        check_value(controller_id, 1, 1)
        check_value(controller_id, -1, 0)
        check_value(controller_id, 500, 1)

    def test_switches():
        controller_id = 13
        adc_resolution = 8
        max_value = pow(2, adc_resolution) - 1

        check_value(controller_id, 0, 0)
        check_no_value(controller_id, int(0.05 * max_value))
        check_value(controller_id, int(0.11 * max_value), 1)
        check_value(controller_id, int(0.6 * max_value), 2)

    def test_encoders():
        controller_id = 14

        check_value(controller_id, 1, 1)
        check_value(controller_id, 0, 1)
        check_value(controller_id, 5, 1)
        check_value(controller_id, -1, -1)
        check_value(controller_id, -200, -1)

    test_pots()
    test_buttons()
    test_switches()
    test_encoders()

    controllers = sensei.grpc_receiver.get_controller_map()
    assert len(controllers) == 14
