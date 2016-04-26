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

    std::unique_ptr<BaseMessage> make_analog_value(const int pin_index,
                                                   const int value,
                                                   const uint32_t timestamp = 0)
    {
        auto msg = new AnalogValue(pin_index, value, timestamp);
        return std::unique_ptr<AnalogValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_digital_value(const int pin_index,
                                                    const bool value,
                                                    const uint32_t timestamp = 0)
    {
        auto msg = new DigitalValue(pin_index, value, timestamp);
        return std::unique_ptr<DigitalValue>(msg);
    }

    std::unique_ptr<BaseMessage> make_output_value(const int pin_index,
                                                   const float value,
                                                   const uint32_t timestamp = 0)
    {
        auto msg = new OutputValue(pin_index, value, timestamp);
        return std::unique_ptr<OutputValue>(msg);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Commands
    ////////////////////////////////////////////////////////////////////////////////

    std::unique_ptr<BaseMessage> make_set_sampling_rate_command(const int pin_index,
                                                                const float sampling_rate,
                                                                const uint32_t timestamp = 0)
    {
        auto msg = new SetSamplingRateCommand(pin_index, sampling_rate, timestamp);
        return std::unique_ptr<SetSamplingRateCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_enabled_command(const int pin_index,
                                                          const bool enabled,
                                                          const uint32_t timestamp = 0)
    {
        auto msg = new SetEnabledCommand(pin_index, enabled, timestamp);
        return std::unique_ptr<SetEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_pin_type_command(const int pin_index,
                                                           const PinType pin_type,
                                                           const uint32_t timestamp = 0)
    {
        auto msg = new SetPinTypeCommand(pin_index, pin_type, timestamp);
        return std::unique_ptr<SetPinTypeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sending_mode_command(const int pin_index,
                                                               const SendingMode mode,
                                                               const uint32_t timestamp = 0)
    {
        auto msg = new SetSendingModeCommand(pin_index, mode, timestamp);
        return std::unique_ptr<SetSendingModeCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_sending_delta_ticks_command(const int pin_index,
                                                                      const int delta_ticks,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetSendingDeltaTicksCommand(pin_index, delta_ticks, timestamp);
        return std::unique_ptr<SetSendingDeltaTicksCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_adc_bit_resolution_command(const int pin_index,
                                                                     const int bit_resolution,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new SetADCBitResolutionCommand(pin_index, bit_resolution, timestamp);
        return std::unique_ptr<SetADCBitResolutionCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_lowpass_filter_order_command(const int pin_index,
                                                                       const int order,
                                                                       const uint32_t timestamp = 0)
    {
        auto msg = new SetLowpassFilterOrderCommand(pin_index, order, timestamp);
        return std::unique_ptr<SetLowpassFilterOrderCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_lowpass_cutoff_command(const int pin_index,
                                                                 const float cutoff,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetLowpassCutoffCommand(pin_index, cutoff, timestamp);
        return std::unique_ptr<SetLowpassCutoffCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_slider_mode_enabled_command(const int pin_index,
                                                                      const bool enabled,
                                                                      const uint32_t timestamp = 0)
    {
        auto msg = new SetSliderModeEnabledCommand(pin_index, enabled, timestamp);
        return std::unique_ptr<SetSliderModeEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_slider_threshold_command(const int pin_index,
                                                                   const int threshold,
                                                                   const uint32_t timestamp = 0)
    {
        auto msg = new SetSliderThresholdCommand(pin_index, threshold, timestamp);
        return std::unique_ptr<SetSliderThresholdCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_send_digital_value_command(const int pin_index,
                                                                 const bool value,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SendDigitalPinValueCommand(pin_index, value, timestamp);
        return std::unique_ptr<SendDigitalPinValueCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_enable_sending_packets_command(const int index,
                                                                     const bool enabled,
                                                                     const uint32_t timestamp = 0)
    {
        auto msg = new EnableSendingPacketsCommand(index, enabled, timestamp);
        return std::unique_ptr<EnableSendingPacketsCommand>(msg);
    }

    // Internal commands

    std::unique_ptr<BaseMessage> make_set_invert_enabled_command(const int pin_index,
                                                                 const bool enabled,
                                                                 const uint32_t timestamp = 0)
    {
        auto msg = new SetInvertEnabledCommand(pin_index, enabled, timestamp);
        return std::unique_ptr<SetInvertEnabledCommand>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_input_scale_range_low_command(const int pin_index,
                                                                        const int value,
                                                                        const uint32_t timestamp = 0)
    {
        auto msg = new SetInputScaleRangeLow(pin_index, value, timestamp);
        return std::unique_ptr<SetInputScaleRangeLow>(msg);
    }

    std::unique_ptr<BaseMessage> make_set_input_scale_range_high_command(const int pin_index,
                                                                         const int value,
                                                                         const uint32_t timestamp = 0)
    {
        auto msg = new SetInputScaleRangeHigh(pin_index, value, timestamp);
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

    std::unique_ptr<BaseMessage> make_set_pin_name_command(const int pin_index,
                                                           const std::string name,
                                                           const uint32_t timestamp = 0)
    {
        auto msg = new SetPinNameCommand(pin_index, name, timestamp);
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
