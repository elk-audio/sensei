/**
 * @brief Classes for remapping raw sensor data into output range.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * The classes here store all the configuration regarding a single sensor as well.
 * They are only used as components in MappingProcessor
 */
#ifndef SENSEI_SENSOR_MAPPERS_H
#define SENSEI_SENSOR_MAPPERS_H

#include "message/command_defs.h"

namespace sensei {

namespace mapping {

/**
 * @brief Base class for sensor mappers.
 *
 * Handles enabled status and sending mode.
 */
class BaseSensorMapper
{

public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(BaseSensorMapper)

    BaseSensorMapper(const PinType pin_type = PinType::ANALOG_INPUT,
                     const int sensor_index = 0);

    virtual ~BaseSensorMapper();

    /**
     * @brief Modify internal configuration according to the given command.
     *
     * @param cmd Configuration command.
     *
     * @return CommandErrorCode::OK if command was succesful.
     *
     * Other return codes can be errors specific to a parameter (e.g. CommandErrorCode::INVALID_RANGE),
     * or CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE
     * if command is not appropriate for configuring this sensor.
     */
    virtual CommandErrorCode apply_command(const Command *cmd);

    /**
     * @brief Fill the given container with a sequence of commands that match internal configuration.
     *
     * Invariant: if the output sequence is sent back to the object with apply_command(..), there
     * should be no changes in the internal state.
     *
     * @param iterator back_inserter operator to output container to be filled
     */
    virtual void put_config_commands_into(CommandIterator iterator);

protected:
    PinType _pin_type;
    int _sensor_index;
    bool _sensor_enabled;
    SendingMode _sending_mode;

    bool _invert_value;
};

class DigitalSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(DigitalSensorMapper)

    DigitalSensorMapper(const int sensor_index = 0);

    ~DigitalSensorMapper();

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator iterator) override;

private:

};

class AnalogSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(AnalogSensorMapper)

    AnalogSensorMapper(const int sensor_index = 0,
                       const float adc_sampling_rate = 1000.0f);

    ~AnalogSensorMapper();

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator iterator) override;

private:
    CommandErrorCode _set_adc_bit_resolution(const int resolution);

    CommandErrorCode _set_input_scale_range_low(const int value);

    CommandErrorCode _set_input_scale_range_high(const int value);

    CommandErrorCode _set_delta_ticks_sending(const int value);

    CommandErrorCode _set_lowpass_filter_order(const int value);

    CommandErrorCode _set_lowpass_cutoff(const float value);

    CommandErrorCode _set_slider_threshold(const int value);

    // External board config
    int _delta_ticks_sending;
    int _adc_bit_resolution;
    int _lowpass_filter_order;
    float _lowpass_cutoff;
    bool _slider_mode_enabled;
    int _slider_threshold;

    // Mapping parameters
    int _input_scale_range_low;
    int _input_scale_range_high;

    // Internal helper attributes
    int _max_allowed_input;
    float _adc_sampling_rate;
};


}; // namespace mapping
}; // namespace sensei

#endif //SENSEI_SENSOR_MAPPERS_H
