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

void UserFrontend::set_enabled(int sensor_index, bool enabled)
{
    auto msg = _factory.make_set_enabled_command(sensor_index, enabled);
    _queue->push(std::move(msg));
}

void UserFrontend::set_digital_output(int index, bool value)
{
    auto msg = _factory.make_integer_set_value(index, value? 1:0);
    _queue->push(std::move(msg));
}

void UserFrontend::set_continuous_output(int index, float value)
{
    auto msg = _factory.make_float_set_value(index, value);
    _queue->push(std::move(msg));
}

void UserFrontend::set_range_output(int index, int value)
{
    auto msg = _factory.make_integer_set_value(index, value);
    _queue->push(std::move(msg));
}

void UserFrontend::set_imu_calibration()
{
    auto cal_msg = _factory.make_imu_calibrate_command();
    auto save_msg = _factory.make_imu_commit_settings_command();
    _queue->push(std::move(cal_msg));
    _queue->push(std::move(save_msg));
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