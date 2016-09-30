/**
 * @brief Base class for runtime user frontend
 * @copyright MIND Music Labs AB, Stockholm
 *
 * This module give run-time control from the user over some fast-changing configuration
 * parameters (e.g. sensors enabled/disabled) and access to digital output pins.
 */

#include "user_frontend.h"

using namespace sensei;
using namespace sensei::user_frontend;

CommandErrorCode UserFrontend::apply_command(const Command* /*cmd*/)
{
    auto status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
    return status;
}

void UserFrontend::set_pin_enabled(const int pin_index, const bool enabled)
{
    auto msg = _factory.make_set_enabled_command(pin_index, enabled);
    _queue->push(std::move(msg));
}

void UserFrontend::set_digital_output(const int out_pin_index, const bool value)
{
    auto msg = _factory.make_send_digital_value_command(out_pin_index, value);
    _queue->push(std::move(msg));
}

void UserFrontend::set_imu_calibration()
{
    auto msg = _factory.make_imu_calibrate_command();
    _queue->push(std::move(msg));
}

void UserFrontend::set_imu_factory_reset()
{
    auto msg = _factory.make_imu_factory_reset_command();
    _queue->push(std::move(msg));
}

void UserFrontend::set_imu_reboot()
{
    auto msg = _factory.make_imu_reboot_command();
    _queue->push(std::move(msg));
}