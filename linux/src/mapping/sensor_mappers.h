/**
 * @brief Classes for storing external and mapping configurations for sensors
 * @copyright MIND Music Labs AB, Stockholm
 *
 * The classes here store all the state regarding a sensor, /excluded/ the backend configuration.
 *
 */
#ifndef SENSEI_SENSOR_MAPPERS_H
#define SENSEI_SENSOR_MAPPERS_H

#include "message/command_defs.h"

namespace sensei {

namespace {

static const int MAX_ADC_BIT_RESOLUTION = 16;
static const int DEFAULT_ADC_BIT_RESOLUTION = 12;
static const float DEFAULT_LOWPASS_CUTOFF = 100.0f;

}; // Anonymous namespace

typedef std::vector<std::unique_ptr<BaseMessage>> CommandContainer;
typedef std::back_insert_iterator<CommandContainer> CommandIterator;

class BaseSensorMapper
{

public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(BaseSensorMapper)

    BaseSensorMapper(const PinType pin_type=PinType::ANALOG_INPUT,
                     const int sensor_index=0);

    virtual ~BaseSensorMapper();

    virtual CommandErrorCode apply_command(const Command *cmd);

    virtual void put_config_commands_into(CommandIterator iterator);

protected:
    PinType _pin_type;
    int     _sensor_index;
    bool    _sensor_enabled;
    SendingMode _sending_mode;

    bool _invert_value;
};

class DigitalSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(DigitalSensorMapper)

    DigitalSensorMapper(const int sensor_index=0);

    ~DigitalSensorMapper();

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator iterator) override;

private:

};

class AnalogSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(AnalogSensorMapper)

    AnalogSensorMapper(const int sensor_index=0);

    ~AnalogSensorMapper();

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator iterator) override;

private:
    CommandErrorCode _set_adc_bit_resolution(const int resolution);

    CommandErrorCode _set_input_scale_range_low(const int value);

    CommandErrorCode _set_input_scale_range_high(const int value);

    // External board config
    int _delta_ticks_sending;
    int _adc_bit_resolution;
    float _lowpass_cutoff;
    bool _slider_mode_enabled;
    int _slider_threshold;

    // Mapping parameters
    int _input_scale_range_low;
    int _input_scale_range_high;

    // Internal helper attributes
    int _max_allowed_input;
};

}; // namespace sensei

#endif //SENSEI_SENSOR_MAPPERS_H
