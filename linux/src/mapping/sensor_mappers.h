/**
 * @brief Classes for remapping raw sensor data into output range.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * The classes here store all the configuration regarding a single sensor as well.
 * They are only used as components in MappingProcessor
 */
#ifndef SENSEI_SENSOR_MAPPERS_H
#define SENSEI_SENSOR_MAPPERS_H

#include "message/base_value.h"
#include "message/value_defs.h"
#include "output_backend/output_backend.h"
#include "message/command_defs.h"
#include "message/message_factory.h"

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

    explicit BaseSensorMapper(SensorType pin_type = SensorType::ANALOG_INPUT, int index = 0);

    virtual ~BaseSensorMapper() = default;

    /**
     * @brief Modify internal configuration according to the given command.
     *
     * @param [in] cmd Configuration command.
     *
     * @return CommandErrorCode::OK if command was successful.
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
     * @param [out] out_iterator back_inserter operator to output container to be filled
     */
    virtual void put_config_commands_into(CommandIterator out_iterator);

    /**
     * @brief Process a given input value and generate output values for the backend consumer.
     *
     * @param [in] value Input value coming from the gpio hw frontend
     * @param [out] out_iterator Iterator to a collection to which output values will be added
     */
    virtual void process(Value *value, output_backend::OutputBackend *backend) = 0;

    /**
     * @brief Process a given value coming from a user frontend and generate a set command
     *        that will be passed on to the hw frontend
     * @param [in] value Input value from a user fronted
     * @return A set value command to be sent to a hw frontend
     */
    virtual std::unique_ptr<Command> process_set_value(Value *value) = 0;

protected:
    MessageFactory      _factory;
    SensorType          _sensor_type;
    SensorHwType        _hw_type;
    int                 _sensor_index;
    std::vector<int>    _hw_pins;
    bool                _enabled;
    bool                _multiplexed;
    MultiplexerData     _multiplexer_data;
    SendingMode         _sending_mode;
    int                 _delta_ticks_sending;

    float               _previous_value;
    bool                _invert_value;
    bool                _send_timestamp;
    bool                _fast_mode;
};

/**
 * @brief Mapper for sensors that produce a digital (on/off) output
 */
class DigitalSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(DigitalSensorMapper)

    explicit DigitalSensorMapper(int index = 0);

    ~DigitalSensorMapper() = default;

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator out_iterator) override;

    void process(Value *value, output_backend::OutputBackend *backend) override;

    virtual std::unique_ptr<Command> process_set_value(Value *value) override;

private:

};

/**
 * @brief Mappers for sensors that produce analog value sampled with a given
 *        adc resolution, values are sent to mapper as an integer value
 */
class AnalogSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(AnalogSensorMapper)

    explicit AnalogSensorMapper(int index = 0, float adc_sampling_rate = 1000.0f);

    ~AnalogSensorMapper() = default;

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator out_iterator) override;

    void process(Value *value, output_backend::OutputBackend *backend) override;

    virtual std::unique_ptr<Command> process_set_value(Value *value) override;

private:
    CommandErrorCode _set_sensor_hw_type(SensorHwType hw_type);
    CommandErrorCode _set_adc_bit_resolution(int resolution);
    CommandErrorCode _set_input_scale_range(int low, int high);
    CommandErrorCode _set_lowpass_filter_order(int value);
    CommandErrorCode _set_lowpass_cutoff(float value);
    CommandErrorCode _set_slider_threshold(int value);

    // External board config
    int _delta_ticks_sending;
    int _adc_bit_resolution;
    int _lowpass_filter_order;
    float _lowpass_cutoff;
    int _slider_threshold;

    // Mapping parameters
    int _input_scale_range_low;
    int _input_scale_range_high;

    // Internal helper attributes
    int _max_allowed_input;
    float _adc_sampling_rate;
};

/**
 * @brief Mapper for sensors that produce discrete integer values such as
 *        multi position switches
 */
class RangeSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(RangeSensorMapper)

    explicit RangeSensorMapper(int index = 0);

    ~RangeSensorMapper() = default;

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator out_iterator) override;

    void process(Value *value, output_backend::OutputBackend *backend) override;

    virtual std::unique_ptr<Command> process_set_value(Value *value) override;

private:
    CommandErrorCode _set_sensor_hw_type(SensorHwType hw_type);
    CommandErrorCode _set_input_scale_range(int low, int high);

    // Mapping parameters
    int _input_scale_range_low;
    int _input_scale_range_high;

    // Internal helper attributes
    int _previous_int_value;
};

/**
 * @brief Mapper for sensors that produce a continuous value represented as a float
 */
class ContinuousSensorMapper : public BaseSensorMapper
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(ContinuousSensorMapper)

    explicit ContinuousSensorMapper(int index = 0);

    ~ContinuousSensorMapper() = default;

    CommandErrorCode apply_command(const Command *cmd) override;

    void put_config_commands_into(CommandIterator out_iterator) override;

    void process(Value *value, output_backend::OutputBackend *backend) override;

    virtual std::unique_ptr<Command> process_set_value(Value *value) override;

private:
    CommandErrorCode _set_input_scale_range(float low, float high);

    // Mapping parameters
    float _input_scale_range_low;
    float _input_scale_range_high;
};

}; // namespace mapping
}; // namespace sensei

#endif //SENSEI_SENSOR_MAPPERS_H
