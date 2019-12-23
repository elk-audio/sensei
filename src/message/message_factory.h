/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Factory class for controlled message instantiation.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * The factory defined here provides the suggested method to instantiate messages
 * of different types.
 */
#ifndef SENSEI_MESSAGE_FACTORY_H
#define SENSEI_MESSAGE_FACTORY_H

#include <memory>

#include "message/value_defs.h"
#include "message/command_defs.h"
#include "message/error_defs.h"

// TODO:
//      add constructor parameters for e.g. maximum number of sensors, maximum analog value, etc.


namespace sensei {

/**
 * @brief Message factory implemented with Singleton pattern.
 *
 * The provided methods always return unique pointers in the form of
 *      std::unique_ptr<BaseMessage>
 * nullptr is returned if creation fails.
 *
 * Safe for usage from different thread contexts.
 */
class MessageFactory {

public:

    MessageFactory()
    {
    }

    ~MessageFactory()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Values
    ////////////////////////////////////////////////////////////////////////////////

    std::unique_ptr<BaseMessage> make_analog_value(const int sensor_id,
                                                   const int value,
                                                   const uint32_t timestamp = 0)
    {
        auto msg = new AnalogValue(sensor_id, value, timestamp);
        return std::unique_ptr<AnalogValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_digital_value(const int sensor_id,
                                                    const bool value,
                                                    const uint32_t timestamp = 0)
    {
        auto msg = new DigitalValue(sensor_id, value, timestamp);
        return std::unique_ptr<DigitalValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_continuous_value(const int sensor_id,
                                                       const float value,
                                                       const uint32_t timestamp = 0)
    {
        auto msg = new ContinuousValue(sensor_id, value, timestamp);
        return std::unique_ptr<ContinuousValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_output_value(const int sensor_id,
                                                   const float value,
                                                   const uint32_t timestamp = 0)
    {
        auto msg = new OutputValue(sensor_id, value, timestamp);
        return std::unique_ptr<OutputValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_integer_set_value(const int sensor_id,
                                                        const int value,
                                                        const uint32_t timestamp = 0)
    {
        auto msg = new IntegerSetValue(sensor_id, value, timestamp);
        return std::unique_ptr<IntegerSetValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_float_set_value(const int sensor_id,
                                                      const float value,
                                                      const uint32_t timestamp = 0)
    {
        auto msg = new FloatSetValue(sensor_id, value, timestamp);
        return std::unique_ptr<FloatSetValue>(msg);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Commands
    ////////////////////////////////////////////////////////////////////////////////

    std::unique_ptr<BaseMessage> make_set_enabled_command(const int sensor_id,
                                                          const bool enabled,
                                                          const uint32_t timestamp = 0)
    {
        auto msg = new SetEnabledCommand(sensor_id, enabled, timestamp);
        return std::unique_ptr<SetEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sensor_type_command(const int sensor_id,
                                                              const SensorType pin_type,
                                                              const uint32_t timestamp = 0)
    {
        auto msg = new SetSensorTypeCommand(sensor_id, pin_type, timestamp);
        return std::unique_ptr<SetSensorTypeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sensor_hw_type_command(const int sensor_id,
                                                                 const SensorHwType hw_type,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetSensorHwTypeCommand(sensor_id, hw_type, timestamp);
        return std::unique_ptr<SetSensorHwTypeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_hw_pins_command(const int sensor_id,
                                                          std::vector<int> pins,
                                                          const uint32_t timestamp = 0)
    {
        auto msg = new SetHwPinsCommand(sensor_id, pins, timestamp);
        return std::unique_ptr<SetHwPinsCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sending_mode_command(const int sensor_id,
                                                               const SendingMode mode,
                                                               const uint32_t timestamp = 0)
    {
        auto msg = new SetSendingModeCommand(sensor_id, mode, timestamp);
        return std::unique_ptr<SetSendingModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sending_delta_ticks_command(const int sensor_id,
                                                                      const int delta_ticks,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetSendingDeltaTicksCommand(sensor_id, delta_ticks, timestamp);
        return std::unique_ptr<SetSendingDeltaTicksCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_adc_bit_resolution_command(const int sensor_id,
                                                                     const int bit_resolution,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new SetADCBitResolutionCommand(sensor_id, bit_resolution, timestamp);
        return std::unique_ptr<SetADCBitResolutionCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_analog_time_constant_command(const int sensor_id,
                                                                       const float time_constant,
                                                                       const uint32_t timestamp = 0)
    {
        auto msg = new SetADCFitlerTimeConstantCommand(sensor_id, time_constant, timestamp);
        return std::unique_ptr<SetADCFitlerTimeConstantCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_slider_threshold_command(const int sensor_id,
                                                                   const int threshold,
                                                                   const uint32_t timestamp = 0)
    {
        auto msg = new SetSliderThresholdCommand(sensor_id, threshold, timestamp);
        return std::unique_ptr<SetSliderThresholdCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_multiplexed_sensor_command(const int sensor_id,
                                                                     const int multiplexer_id,
                                                                     const int multiplexer_pin,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new SetMultiplexedSensorCommand(sensor_id, {multiplexer_id, multiplexer_pin}, timestamp);
        return std::unique_ptr<SetMultiplexedSensorCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sensor_hw_polarity_command(const int sensor_id,
                                                                     HwPolarity polarity,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new SetSensorHwPolarityCommand(sensor_id, polarity, timestamp);
        return std::unique_ptr<SetSensorHwPolarityCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_fast_mode_command(const int sensor_id,
                                                            const bool enabled,
                                                            const uint32_t timestamp = 0)
    {
        auto msg = new SetFastModeCommand(sensor_id, enabled, timestamp);
        return std::unique_ptr<SetFastModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_digital_output_command(const int sensor_id,
                                                                 const bool value,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetDigitalOutputValueCommand(sensor_id, value, timestamp);
        return std::unique_ptr<SetDigitalOutputValueCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_continuous_output_command(const int sensor_id,
                                                                    const float value,
                                                                    const uint32_t timestamp = 0)
    {
        auto msg = new SetContinuousOutputValueCommand(sensor_id, value, timestamp);
        return std::unique_ptr<SetContinuousOutputValueCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_range_output_command(const int sensor_id,
                                                               const int value,
                                                               const uint32_t timestamp = 0)
    {
        auto msg = new SetRangeOutputValueCommand(sensor_id, value, timestamp);
        return std::unique_ptr<SetRangeOutputValueCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_enable_sending_packets_command(const int index,
                                                                     const bool enabled,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new EnableSendingPacketsCommand(index, enabled, timestamp);
        return std::unique_ptr<EnableSendingPacketsCommand>(msg);
    }

    // Internal commands

    std::unique_ptr<BaseMessage> make_set_invert_enabled_command(const int sensor_id,
                                                                 const bool enabled,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetInvertEnabledCommand(sensor_id, enabled, timestamp);
        return std::unique_ptr<SetInvertEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_input_range_command(const int sensor_id,
                                                              const float min,
                                                              const float max,
                                                              const uint32_t timestamp = 0)
    {
        auto msg = new SetInputRangeCommand(sensor_id, {min, max}, timestamp);
        return std::unique_ptr<SetInputRangeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_send_timestamp_enabled(const int index,
                                                                 const bool enabled,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetSendTimestampEnabledCommand(index, enabled, timestamp);
        return std::unique_ptr<SetSendTimestampEnabledCommand>(msg);
    }

    // Output Backend commands

    std::unique_ptr<BaseMessage> make_set_backend_type_command(const int index,
                                                               const BackendType type,
                                                               const uint32_t timestamp = 0)
    {
        auto msg = new SetBackendTypeCommand(index, type, timestamp);
        return std::unique_ptr<SetBackendTypeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sensor_name_command(const int sensor_id,
                                                              const std::string name,
                                                              const uint32_t timestamp = 0)
    {
        auto msg = new SetPinNameCommand(sensor_id, name, timestamp);
        return std::unique_ptr<SetPinNameCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_send_output_enabled_command(const int index,
                                                                      const bool enabled,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetSendOutputEnabledCommand(index, enabled, timestamp);
        return std::unique_ptr<SetSendOutputEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_send_raw_input_enabled_command(const int index,
                                                                         const bool enabled,
                                                                         const uint32_t timestamp = 0)
    {
        auto msg = new SetSendRawInputEnabledCommand(index, enabled, timestamp);
        return std::unique_ptr<SetSendRawInputEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_osc_output_base_path_command(const int index,
                                                                       const std::string path,
                                                                       const uint32_t timestamp = 0)
    {
        auto msg = new SetOSCOutputBasePathCommand(index, path, timestamp);
        return std::unique_ptr<SetOSCOutputBasePathCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_osc_output_raw_path_command(const int index,
                                                                      const std::string path,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetOSCOutputRawPathCommand(index, path, timestamp);
        return std::unique_ptr<SetOSCOutputRawPathCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_osc_output_host_command(const int index,
                                                                  const std::string hostname,
                                                                  const uint32_t timestamp = 0)
    {
        auto msg = new SetOSCOutputHostCommand(index, hostname, timestamp);
        return std::unique_ptr<SetOSCOutputHostCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_osc_output_port_command(const int index,
                                                                  const int port,
                                                                  const uint32_t timestamp = 0)
    {
        auto msg = new SetOSCOutputPortCommand(index, port, timestamp);
        return std::unique_ptr<SetOSCOutputPortCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_osc_input_port_command(const int index,
                                                                 const int port,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetOSCInputPortCommand(index, port, timestamp);
        return std::unique_ptr<SetOSCInputPortCommand>(msg);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Errors
    ////////////////////////////////////////////////////////////////////////////////

    std::unique_ptr<BaseMessage> make_bad_crc_error(const int index,
                                                    const uint32_t timestamp = 0)
    {
        auto msg = new BadCrcError(index, timestamp);
        return std::unique_ptr<BadCrcError>(msg);
    }

    std::unique_ptr<BaseMessage> make_too_many_timeouts_error(const int index,
                                                              const uint32_t timestamp = 0)
    {
        auto msg = new TooManyTimeoutsError(index, timestamp);
        return std::unique_ptr<TooManyTimeoutsError>(msg);
    }

};

} // namespace sensei

#endif //SENSEI_MESSAGE_FACTORY_H