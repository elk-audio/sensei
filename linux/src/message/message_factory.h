/**
 * @brief Factory class for controlled message instantiation.
 * @copyright MIND Music Labs AB, Stockholm
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

    std::unique_ptr<BaseMessage> make_imu_value(const int sensor_id,
                                                const float value,
                                                const uint32_t timestamp = 0)
    {
        auto msg = new ImuValue(sensor_id, value, timestamp);
        return std::unique_ptr<ImuValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_output_value(const int sensor_id,
                                                   const float value,
                                                   const uint32_t timestamp = 0)
    {
        auto msg = new OutputValue(sensor_id, value, timestamp);
        return std::unique_ptr<OutputValue>(msg);
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

    std::unique_ptr<BaseMessage> make_set_virtual_pin_command(const int sensor_id,
                                                              const ImuIndex parameter,
                                                              const uint32_t timestamp = 0)
    {
        auto msg = new SetVirtualPinCommand(sensor_id, parameter, timestamp);
        return std::unique_ptr<SetVirtualPinCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_hw_pin_command(const int sensor_id,
                                                         const int pin_id,
                                                         const uint32_t timestamp = 0)
    {
        auto msg = new SetSingleHwPinCommand(sensor_id, pin_id, timestamp);
        return std::unique_ptr<SetSingleHwPinCommand>(msg);
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

    std::unique_ptr<BaseMessage> make_set_lowpass_filter_order_command(const int sensor_id,
                                                                       const int order,
                                                                       const uint32_t timestamp = 0)
    {
        auto msg = new SetLowpassFilterOrderCommand(sensor_id, order, timestamp);
        return std::unique_ptr<SetLowpassFilterOrderCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_lowpass_cutoff_command(const int sensor_id,
                                                                 const float cutoff,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetLowpassCutoffCommand(sensor_id, cutoff, timestamp);
        return std::unique_ptr<SetLowpassCutoffCommand>(msg);
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

    std::unique_ptr<BaseMessage> make_send_digital_value_command(const int sensor_id,
                                                                 const bool value,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SendDigitalPinValueCommand(sensor_id, value, timestamp);
        return std::unique_ptr<SendDigitalPinValueCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_enable_sending_packets_command(const int index,
                                                                     const bool enabled,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new EnableSendingPacketsCommand(index, enabled, timestamp);
        return std::unique_ptr<EnableSendingPacketsCommand>(msg);
    }

    // IMU commands

    std::unique_ptr<BaseMessage> make_enable_imu_command(const bool enabled,
                                                         const uint32_t timestamp = 0)
    {
        auto msg = new SetImuEnabledCommand(0, enabled, timestamp);
        return std::unique_ptr<SetImuEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_filter_mode_command(const int mode,
                                                                  const uint32_t timestamp = 0)
    {
        auto msg = new SetImuFilterModeCommand(0, mode, timestamp);
        return std::unique_ptr<SetImuFilterModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_acc_range_max_command(const int range,
                                                                    const uint32_t timestamp = 0)
    {
        auto msg = new SetImuAccelerometerRangeMaxCommand(0, range, timestamp);
        return std::unique_ptr<SetImuAccelerometerRangeMaxCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_gyro_range_max_command(const int range,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new SetImuGyroscopeRangeMaxCommand(0, range, timestamp);
        return std::unique_ptr<SetImuGyroscopeRangeMaxCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_compass_range_max_command(const float range,
                                                                        const uint32_t timestamp = 0)
    {
        auto msg = new SetImuCompassRangeMaxCommand(0, range, timestamp);
        return std::unique_ptr<SetImuCompassRangeMaxCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_enable_compass_command(const bool enabled,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetImuCompassEnabledCommand(0, enabled, timestamp);
        return std::unique_ptr<SetImuCompassEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_sending_mode_command(const SendingMode mode,
                                                                   const uint32_t timestamp = 0)
    {
        auto msg = new SetImuSendingModeCommand(0, mode, timestamp);
        return std::unique_ptr<SetImuSendingModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_sending_delta_ticks_command(const int ticks,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetImuSendingDeltaTicksCommand(0, ticks, timestamp);
        return std::unique_ptr<SetImuSendingDeltaTicksCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_set_data_mode_command(const int mode,
                                                                const uint32_t timestamp = 0)
    {
        auto msg = new SetImuDataModeCommand(0, mode, timestamp);
        return std::unique_ptr<SetImuDataModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_acc_threshold_command(const float threshold,
                                                                const uint32_t timestamp = 0)
    {
        auto msg = new SetImuAccThresholdCommand(0, threshold, timestamp);
        return std::unique_ptr<SetImuAccThresholdCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_calibrate_command(const uint32_t timestamp = 0)
    {
        auto msg = new ImuCalibrateCommand(0, 0, timestamp);
        return std::unique_ptr<ImuCalibrateCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_factory_reset_command(const uint32_t timestamp = 0)
    {
        auto msg = new ImuFactoryResetCommand(0, 0, timestamp);
        return std::unique_ptr<ImuFactoryResetCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_reboot_command(const uint32_t timestamp = 0)
    {
        auto msg = new ImuRebootCommand(0, 0, timestamp);
        return std::unique_ptr<ImuRebootCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_get_temperature_command(const uint32_t timestamp = 0)
    {
        auto msg = new ImuGetTemperatureCommand(0, 0, timestamp);
        return std::unique_ptr<ImuGetTemperatureCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_imu_commit_settings_command(const uint32_t timestamp = 0)
    {
        auto msg = new ImuCommitSettingsCommand(0, 0, timestamp);
        return std::unique_ptr<ImuCommitSettingsCommand>(msg);
    }

    // Internal commands

    std::unique_ptr<BaseMessage> make_set_invert_enabled_command(const int sensor_id,
                                                                 const bool enabled,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetInvertEnabledCommand(sensor_id, enabled, timestamp);
        return std::unique_ptr<SetInvertEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_input_scale_range_low_command(const int sensor_id,
                                                                        const float value,
                                                                        const uint32_t timestamp = 0)
    {
        auto msg = new SetInputScaleRangeLow(sensor_id, value, timestamp);
        return std::unique_ptr<SetInputScaleRangeLow>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_input_scale_range_high_command(const int sensor_id,
                                                                         const float value,
                                                                         const uint32_t timestamp = 0)
    {
        auto msg = new SetInputScaleRangeHigh(sensor_id, value, timestamp);
        return std::unique_ptr<SetInputScaleRangeHigh>(msg);
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

}; // namespace sensei

#endif //SENSEI_MESSAGE_FACTORY_H
