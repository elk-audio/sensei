#include <vector>
#include <memory>
#include <iterator>

#include "gtest/gtest.h"

#include "mapping/sensor_mappers.cpp"

#include "message/command_defs.h"
#include "output_backend_mockup.h"
#include "../test_utils.h"

using namespace sensei;
using namespace sensei::mapping;


class TestDigitalSensorMapper : public ::testing::Test
{
protected:
    TestDigitalSensorMapper()
    {
    }

    ~TestDigitalSensorMapper()
    {
    }

    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_send_timestamp_enabled(_sensor_idx, _timestamp))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_fast_mode_command(_sensor_idx, _fast_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }
    }

    void TearDown()
    {
    }

protected:
    int              _sensor_idx{2};
    bool             _enabled{true};
    std::vector<int> _hw_pin{5};
    SensorHwType     _hw_type{SensorHwType::DIGITAL_INPUT_PIN};
    SendingMode      _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool             _inverted{true};
    bool             _timestamp{false};
    bool             _fast_mode{true};
    int              _delta_ticks{5};

    OutputBackendMockup _backend;
    DigitalSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestDigitalSensorMapper, test_config)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    auto cmd_fast_mode = extract_cmd_from<SetFastModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_FAST_MODE, cmd_fast_mode->type());
    ASSERT_EQ(_fast_mode, cmd_fast_mode->data());

    auto cmd_timestamped = extract_cmd_from<SetSendTimestampEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SEND_TIMESTAMP_ENABLED, cmd_timestamped->type());
    ASSERT_EQ(_timestamp, cmd_timestamped->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());
}


TEST_F(TestDigitalSensorMapper, test_config_fail)
{
    // Verify that some wrong commands return unhandled error
    MessageFactory factory;

    auto ret = _mapper.apply_command(CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 12)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_analog_time_constant_command(_sensor_idx, 0.020f)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);
}

TEST_F(TestDigitalSensorMapper, test_process)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    auto input_msg = factory.make_digital_value(_sensor_idx, false);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(0.0f, _backend._last_output_value);

    input_msg = factory.make_digital_value(_sensor_idx, true);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestDigitalSensorMapper, test_invert)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    auto input_msg = factory.make_digital_value(_sensor_idx, false);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestDigitalSensorMapper, test_disabled_process_dont_send_values)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Put some weird value out-of-range and verify that is not touched by process
    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    auto input_msg = factory.make_digital_value(_sensor_idx, false);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

TEST_F(TestDigitalSensorMapper, test_raw_input_send)
{
    MessageFactory factory;
    bool           sensor_value = true;
    auto           input_msg = factory.make_digital_value(_sensor_idx, sensor_value);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(sensor_value, _backend._last_raw_digital_input);
}

class TestAnalogSensorMapper : public ::testing::Test
{
protected:
    TestAnalogSensorMapper()
    {
    }

    ~TestAnalogSensorMapper()
    {
    }

    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_send_timestamp_enabled(_sensor_idx, _timestamp))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_fast_mode_command(_sensor_idx, _fast_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_multiplexed_sensor_command(_sensor_idx,
                                                                                             _multiplexer_data.id,
                                                                                             _multiplexer_data.pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, _adc_bit_resolution))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_analog_time_constant_command(_sensor_idx, _filter_time_constant))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_analog_hysteresis_command(_sensor_idx, _hysteresis))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_analog_stabilization_period_command(_sensor_idx, _stabilization_period))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_analog_filter_type_command(_sensor_idx, _filter_type))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_slider_threshold_command(_sensor_idx, _slider_threshold))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_input_range_command(_sensor_idx,
                                                                                      _input_scale_low,
                                                                                      _input_scale_high))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }
    }

    void TearDown()
    {
    }

protected:
    int              _sensor_idx{2};
    bool             _enabled{true};
    SensorHwType     _hw_type{SensorHwType::ANALOG_INPUT_PIN};
    std::vector<int> _hw_pin{3};
    SendingMode      _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool             _timestamp{true};
    bool             _fast_mode{true};
    bool             _inverted{true};
    int              _delta_ticks{5};
    MultiplexerData  _multiplexer_data{3, 8};
    float            _filter_time_constant{0.20f};
    int              _hysteresis{2};
    float            _stabilization_period{0.30f};
    AnalogFilterType _filter_type{AnalogFilterType::IIR};
    int              _adc_bit_resolution{12};
    int              _slider_threshold{9};
    int              _input_scale_low{22};
    int              _input_scale_high{2322};

    OutputBackendMockup _backend;
    AnalogSensorMapper  _mapper{_sensor_idx};
};

TEST_F(TestAnalogSensorMapper, test_config)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    auto cmd_range = extract_cmd_from<SetInputRangeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INPUT_RANGE, cmd_range->type());
    Range expected = {static_cast<float>(_input_scale_low), static_cast<float>(_input_scale_high)};
    ASSERT_EQ(expected, cmd_range->data());

    auto cmd_slider_thr = extract_cmd_from<SetSliderThresholdCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SLIDER_THRESHOLD, cmd_slider_thr->type());
    ASSERT_EQ(_slider_threshold, cmd_slider_thr->data());

    auto cmd_filter_type = extract_cmd_from<SetAnalogFilterTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ANALOG_FILTER_TYPE, cmd_filter_type->type());
    ASSERT_EQ(_filter_type, cmd_filter_type->data());

    auto cmd_stabilization = extract_cmd_from<SetAnalogStabilizationPeriodCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ANALOG_STABILIZATION_PERIOD, cmd_stabilization->type());
    ASSERT_EQ(_stabilization_period, cmd_stabilization->data());

    auto cmd_hysteresis = extract_cmd_from<SetAnalogHysteresisCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ANALOG_HYSTERESIS, cmd_hysteresis->type());
    ASSERT_EQ(_hysteresis, cmd_hysteresis->data());

    auto cmd_cutoff = extract_cmd_from<SetADCFilterTimeConstantCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ADC_FILTER_TIME_CONSTANT, cmd_cutoff->type());
    ASSERT_EQ(_filter_time_constant, cmd_cutoff->data());

    auto cmd_adc_res = extract_cmd_from<SetADCBitResolutionCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ADC_BIT_RESOLUTION, cmd_adc_res->type());
    ASSERT_EQ(_adc_bit_resolution, cmd_adc_res->data());

    auto cmd_multiplexed = extract_cmd_from<SetMultiplexedSensorCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_MULTIPLEXED, cmd_multiplexed->type());
    ASSERT_EQ(_multiplexer_data.id, cmd_multiplexed->data().id);
    ASSERT_EQ(_multiplexer_data.pin, cmd_multiplexed->data().pin);

    auto cmd_fast_mode = extract_cmd_from<SetFastModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_FAST_MODE, cmd_fast_mode->type());
    ASSERT_EQ(_fast_mode, cmd_fast_mode->data());

    auto cmd_timestamped = extract_cmd_from<SetSendTimestampEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SEND_TIMESTAMP_ENABLED, cmd_timestamped->type());
    ASSERT_EQ(_timestamp, cmd_timestamped->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());

    auto cmd_hw_pin = extract_cmd_from<SetHwPinsCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_HW_PINS, cmd_hw_pin->type());
    ASSERT_EQ(_hw_pin, cmd_hw_pin->data());

    auto _cmd_hw_type = extract_cmd_from<SetSensorHwTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_HW_TYPE, _cmd_hw_type->type());
    ASSERT_EQ(_hw_type, _cmd_hw_type->data());

    auto cmd_pintype = extract_cmd_from<SetSensorTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_TYPE, cmd_pintype->type());
    ASSERT_EQ(SensorType::ANALOG_INPUT, cmd_pintype->data());
}

TEST_F(TestAnalogSensorMapper, test_config_fail)
{
    // Verify that wrong commands return appropriate errors
    MessageFactory factory;

    auto ret = _mapper.apply_command(CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, -10)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);
    ret = _mapper.apply_command(CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 100)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, -23)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_analog_time_constant_command(_sensor_idx, -1)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx, -20, -10)));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx,
                                                                             _input_scale_high,
                                                                             _input_scale_high)));
    ASSERT_EQ(CommandErrorCode::CLIP_WARNING, ret);
}

TEST_F(TestAnalogSensorMapper, test_process)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Middle value should return 0.5

    // Force range sum to be even so that we can just check for 0.5
    if (((_input_scale_low + _input_scale_high) % 2) == 1)
    {
        _input_scale_low += 1;
        ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx,
                                                                                 _input_scale_low,
                                                                                 _input_scale_high)));
        ASSERT_EQ(CommandErrorCode::OK, ret);
    }
    auto input_msg = factory.make_analog_value(_sensor_idx,
                                               (_input_scale_low + _input_scale_high) / 2);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(0.5f, _backend._last_output_value);
}


TEST_F(TestAnalogSensorMapper, test_invert)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    int  sensor_input = static_cast<int>(0.25 * (_input_scale_low + _input_scale_high));
    auto input_msg = factory.make_analog_value(_sensor_idx, sensor_input);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    float out_val = _backend._last_output_value;

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    ASSERT_EQ(CommandErrorCode::OK, ret);
    _mapper.process(input_val, &_backend);
    float inverted_val = _backend._last_output_value;
    ASSERT_FLOAT_EQ(inverted_val, 1.0f - out_val);
}

TEST_F(TestAnalogSensorMapper, test_clip)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Under low range
    int  sensor_input = std::max(_input_scale_low - 10, 0);
    auto input_msg = factory.make_analog_value(_sensor_idx, sensor_input);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(0.0f, _backend._last_output_value);

    // Above high range
    sensor_input = std::min(_input_scale_high + 10, ((1 << _adc_bit_resolution) - 1));
    input_msg = factory.make_analog_value(_sensor_idx, sensor_input);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestAnalogSensorMapper, test_disabled_process_dont_send_values)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);
    auto input_msg = factory.make_analog_value(_sensor_idx, 100);
    auto input_val = static_cast<Value*>(input_msg.get());

    // Put some weird value out-of-range and verify that is not touched by process
    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

TEST_F(TestAnalogSensorMapper, test_output_timestamp_preserved)
{
    MessageFactory factory;
    uint32_t       ref_time = 123456;
    auto           input_msg = factory.make_analog_value(_sensor_idx, 100, ref_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(ref_time, _backend._last_timestamp);
}

TEST_F(TestAnalogSensorMapper, test_same_value_not_sent_twice)
{
    MessageFactory factory;
    uint32_t       first_message_time = 100;
    auto           input_msg = factory.make_analog_value(_sensor_idx, 100, first_message_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    uint32_t second_message_time = 999;
    input_msg = factory.make_analog_value(_sensor_idx, 100, second_message_time);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    ASSERT_EQ(first_message_time, _backend._last_timestamp);
}

TEST_F(TestAnalogSensorMapper, test_unchanged_value_sent_after_clear_previous)
{
    MessageFactory factory;
    const float    analog_value = 0.0f;
    const uint32_t first_message_time = 100;
    auto           input_msg = factory.make_analog_value(_sensor_idx, analog_value, first_message_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    _mapper.apply_command(CMD_PTR(factory.make_clear_previous_value_command(_sensor_idx)));

    const uint32_t second_message_time = 101;
    input_msg = factory.make_analog_value(_sensor_idx, analog_value, second_message_time);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    ASSERT_EQ(second_message_time, _backend._last_timestamp);
}

TEST_F(TestAnalogSensorMapper, test_raw_input_send)
{
    MessageFactory factory;
    int            sensor_value = 100;
    auto           input_msg = factory.make_analog_value(_sensor_idx, sensor_value);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(sensor_value, _backend._last_raw_analogue_input);
}


class TestRangeSensorMapper : public ::testing::Test
{
protected:
    TestRangeSensorMapper()
    {
    }

    ~TestRangeSensorMapper()
    {
    }

    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_send_timestamp_enabled(_sensor_idx, _timestamp))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_fast_mode_command(_sensor_idx, _fast_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_input_range_command(_sensor_idx,
                                                                                      _input_scale_low,
                                                                                      _input_scale_high))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }
    }

    void TearDown()
    {
    }

protected:
    int              _sensor_idx{2};
    bool             _enabled{true};
    SensorHwType     _hw_type{SensorHwType::N_WAY_SWITCH};
    std::vector<int> _hw_pin{3};
    SendingMode      _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool             _inverted{true};
    bool             _timestamp{true};
    bool             _fast_mode{true};
    int              _delta_ticks{5};
    int              _input_scale_low{2};
    int              _input_scale_high{15};

    OutputBackendMockup _backend;
    RangeSensorMapper   _mapper{_sensor_idx};
};

TEST_F(TestRangeSensorMapper, test_config)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    auto cmd_scale_range = extract_cmd_from<SetInputRangeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INPUT_RANGE, cmd_scale_range->type());
    Range expected = {static_cast<float>(_input_scale_low), static_cast<float>(_input_scale_high)};
    ASSERT_EQ(expected, cmd_scale_range->data());

    auto cmd_fast_mode = extract_cmd_from<SetFastModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_FAST_MODE, cmd_fast_mode->type());
    ASSERT_EQ(_fast_mode, cmd_fast_mode->data());

    auto cmd_timestamped = extract_cmd_from<SetSendTimestampEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SEND_TIMESTAMP_ENABLED, cmd_timestamped->type());
    ASSERT_EQ(_timestamp, cmd_timestamped->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());

    auto cmd_hw_pin = extract_cmd_from<SetHwPinsCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_HW_PINS, cmd_hw_pin->type());
    ASSERT_EQ(_hw_pin, cmd_hw_pin->data());

    auto _cmd_hw_type = extract_cmd_from<SetSensorHwTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_HW_TYPE, _cmd_hw_type->type());
    ASSERT_EQ(_hw_type, _cmd_hw_type->data());

    auto cmd_pintype = extract_cmd_from<SetSensorTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_TYPE, cmd_pintype->type());
    ASSERT_EQ(SensorType::ANALOG_INPUT, cmd_pintype->data());
}

TEST_F(TestRangeSensorMapper, test_config_fail)
{
    // Verify that wrong commands return appropriate errors
    MessageFactory factory;

    auto ret = _mapper.apply_command(CMD_PTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, -23)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx, 18, 18)));
    ASSERT_EQ(CommandErrorCode::CLIP_WARNING, ret);
}

TEST_F(TestRangeSensorMapper, test_process)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx,
                                                                             _input_scale_low,
                                                                             _input_scale_high)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    auto input_msg = factory.make_analog_value(_sensor_idx, 11);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(11.0f, _backend._last_output_value);
}


TEST_F(TestRangeSensorMapper, test_invert)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx,
                                                                             _input_scale_low,
                                                                             _input_scale_high)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    auto input_msg = factory.make_analog_value(_sensor_idx, 14);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(3.0f, _backend._last_output_value);
}

TEST_F(TestRangeSensorMapper, test_clip)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_input_range_command(_sensor_idx,
                                                                             _input_scale_low,
                                                                             _input_scale_high)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Under low range
    auto input_msg = factory.make_analog_value(_sensor_idx, _input_scale_low - 10);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(_input_scale_low, _backend._last_output_value);

    // Above high range
    input_msg = factory.make_analog_value(_sensor_idx, _input_scale_high + 10);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(_input_scale_high, _backend._last_output_value);
}

TEST_F(TestRangeSensorMapper, test_disabled_process_dont_send_values)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);
    auto input_msg = factory.make_analog_value(_sensor_idx, 5);
    auto input_val = static_cast<Value*>(input_msg.get());

    // Put some weird value out-of-range and verify that is not touched by process
    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

TEST_F(TestRangeSensorMapper, test_output_timestamp_preserved)
{
    MessageFactory factory;
    uint32_t       ref_time = 123456;
    auto           input_msg = factory.make_analog_value(_sensor_idx, 100, ref_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(ref_time, _backend._last_timestamp);
}

TEST_F(TestRangeSensorMapper, test_same_value_not_sent_twice)
{
    MessageFactory factory;
    uint32_t       first_message_time = 100;
    auto           input_msg = factory.make_analog_value(_sensor_idx, 10, first_message_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    uint32_t second_message_time = 999;
    input_msg = factory.make_analog_value(_sensor_idx, 10, second_message_time);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);

    ASSERT_EQ(first_message_time, _backend._last_timestamp);
}

TEST_F(TestRangeSensorMapper, test_raw_input_send)
{
    MessageFactory factory;
    int            sensor_value = 100;
    auto           input_msg = factory.make_analog_value(_sensor_idx, sensor_value);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(sensor_value, _backend._last_raw_analogue_input);
}

class TestContinuousSensorMapper : public ::testing::Test
{
protected:
    TestContinuousSensorMapper()
    {
    }

    ~TestContinuousSensorMapper()
    {
    }

    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_send_timestamp_enabled(_sensor_idx, _timestamp))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_fast_mode_command(_sensor_idx, _fast_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_input_range_command(_sensor_idx,
                                                                                      _input_scale_low,
                                                                                      _input_scale_high))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            EXPECT_EQ(CommandErrorCode::OK, status);
        }
    }

    void TearDown()
    {
    }

protected:
    int              _sensor_idx{2};
    bool             _enabled{true};
    SendingMode      _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool             _inverted{true};
    bool             _timestamp{false};
    bool             _fast_mode{true};
    int              _delta_ticks{5};
    SensorHwType     _hw_type{SensorHwType::ENCODER};
    std::vector<int> _hw_pin{0};

    float _input_scale_low{1};
    float _input_scale_high{3.14};

    OutputBackendMockup    _backend;
    ContinuousSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestContinuousSensorMapper, test_config)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;

    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));
    auto cmd_scale_range = extract_cmd_from<SetInputRangeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INPUT_RANGE, cmd_scale_range->type());
    Range expected = {_input_scale_low, _input_scale_high};
    EXPECT_EQ(expected, cmd_scale_range->data());

    auto cmd_fast_mode = extract_cmd_from<SetFastModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_FAST_MODE, cmd_fast_mode->type());
    ASSERT_EQ(_fast_mode, cmd_fast_mode->data());

    auto cmd_timestamped = extract_cmd_from<SetSendTimestampEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SEND_TIMESTAMP_ENABLED, cmd_timestamped->type());
    ASSERT_EQ(_timestamp, cmd_timestamped->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    EXPECT_EQ(_inverted, cmd_invert->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    EXPECT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    EXPECT_EQ(_enabled, cmd_enabled->data());

    auto cmd_hw_pin = extract_cmd_from<SetHwPinsCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_HW_PINS, cmd_hw_pin->type());
    ASSERT_EQ(_hw_pin, cmd_hw_pin->data());

    auto _cmd_hw_type = extract_cmd_from<SetSensorHwTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_HW_TYPE, _cmd_hw_type->type());
    ASSERT_EQ(_hw_type, _cmd_hw_type->data());

    auto cmd_sensortype = extract_cmd_from<SetSensorTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_TYPE, cmd_sensortype->type());
    ASSERT_EQ(SensorType::CONTINUOUS_INPUT, cmd_sensortype->data());
}


TEST_F(TestContinuousSensorMapper, test_process)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Middle value should return 0.5
    auto input_msg = factory.make_continuous_value(_sensor_idx, (_input_scale_low + _input_scale_high) / 2);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    EXPECT_FLOAT_EQ(0.5f, _backend._last_output_value);

    // 1/4 of the range should return 0.25
    input_msg = factory.make_continuous_value(_sensor_idx,
                                              ((_input_scale_high - _input_scale_low) / 4 + _input_scale_low));
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    EXPECT_FLOAT_EQ(0.25f, _backend._last_output_value);
}


TEST_F(TestContinuousSensorMapper, test_invert)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    int  sensor_input = 1;
    auto input_msg = factory.make_continuous_value(_sensor_idx, sensor_input);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    float out_val = _backend._last_output_value;

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    EXPECT_EQ(CommandErrorCode::OK, ret);
    _mapper.process(input_val, &_backend);
    float inverted_val = _backend._last_output_value;
    EXPECT_FLOAT_EQ(inverted_val, 1.0f - out_val);
}

TEST_F(TestContinuousSensorMapper, test_clip)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Under low range
    float sensor_input = -3.14f;
    auto  input_msg = factory.make_continuous_value(_sensor_idx, sensor_input);
    auto  input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    EXPECT_FLOAT_EQ(0.0f, _backend._last_output_value);

    // Above high range
    sensor_input = 5;
    input_msg = factory.make_continuous_value(_sensor_idx, sensor_input);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    EXPECT_FLOAT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestContinuousSensorMapper, test_disabled_process_dont_send_values)
{
    // Reset relevant configuration
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);
    auto input_msg = factory.make_analog_value(_sensor_idx, 100);
    auto input_val = static_cast<Value*>(input_msg.get());

    // Put some weird value out-of-range and verify that is not touched by process
    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    _mapper.process(input_val, &_backend);
    EXPECT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

////////////////////////////////////////////////////////////////////////////////
// TestRelativeSensorMapper
////////////////////////////////////////////////////////////////////////////////

class TestRelativeSensorMapper : public ::testing::Test
{
protected:
    TestRelativeSensorMapper()
    {
    }

    ~TestRelativeSensorMapper()
    {
    }

    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }
    }

    void TearDown()
    {
    }

protected:
    int              _sensor_idx{2};
    bool             _enabled{true};
    std::vector<int> _hw_pin{5};
    SensorHwType     _hw_type{SensorHwType::ENCODER};

    OutputBackendMockup  _backend;
    RelativeSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestRelativeSensorMapper, test_process)
{
    MessageFactory factory;

    auto input_msg = factory.make_analog_value(_sensor_idx, 1);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);

    input_msg = factory.make_analog_value(_sensor_idx, -1);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(-1.0f, _backend._last_output_value);
}

TEST_F(TestRelativeSensorMapper, test_every_event_forwarded)
{
    // Unlike analog/continuous mappers, relative input has no previous-value guard:
    // every event must be forwarded regardless of whether the value changed.
    MessageFactory factory;

    auto input_msg = factory.make_analog_value(_sensor_idx, 1);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);

    // Replace the captured value with a sentinel - if the next process() call
    // does NOT forward, the sentinel would survive.
    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    // Same value again - must still be forwarded
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestRelativeSensorMapper, test_invert)
{
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    auto input_msg = factory.make_analog_value(_sensor_idx, 1);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(-1.0f, _backend._last_output_value);

    input_msg = factory.make_analog_value(_sensor_idx, -1);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(1.0f, _backend._last_output_value);
}

TEST_F(TestRelativeSensorMapper, test_disabled_process_dont_send_values)
{
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    float fake_reference_value = -123456.789f;
    _backend._last_output_value = fake_reference_value;

    auto input_msg = factory.make_analog_value(_sensor_idx, 1);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

////////////////////////////////////////////////////////////////////////////////
// TestDiscreteSensorMapper
////////////////////////////////////////////////////////////////////////////////

class TestDiscreteSensorMapper : public ::testing::Test
{
protected:
    void SetUp()
    {
        MessageFactory                        factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_send_timestamp_enabled(_sensor_idx, _timestamp))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_fast_mode_command(_sensor_idx, _fast_mode))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_hw_pins_command(_sensor_idx, _hw_pin))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_hw_type_command(_sensor_idx, _hw_type))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, _adc_bit_resolution))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_discrete_ranges_command(_sensor_idx, _discrete_ranges))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _mapper.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }
    }

protected:
    int                _sensor_idx{2};
    bool               _enabled{true};
    SensorHwType       _hw_type{SensorHwType::ANALOG_INPUT_PIN};
    std::vector<int>   _hw_pin{3};
    SendingMode        _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool               _inverted{false};
    bool               _timestamp{true};
    bool               _fast_mode{true};
    int                _delta_ticks{5};
    int                _adc_bit_resolution{12};
    std::vector<Range> _discrete_ranges{{0.0f, 0.33f}, {0.33f, 0.66f}, {0.66f, 1.0f}};

    OutputBackendMockup  _backend;
    DiscreteSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestDiscreteSensorMapper, test_config)
{
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    // LIFO extraction order (last pushed = first extracted)
    auto cmd_ranges = extract_cmd_from<SetDiscreteRangesCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_DISCRETE_RANGES, cmd_ranges->type());
    ASSERT_EQ(_discrete_ranges, cmd_ranges->data());

    auto cmd_adc_res = extract_cmd_from<SetADCBitResolutionCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ADC_BIT_RESOLUTION, cmd_adc_res->type());
    ASSERT_EQ(_adc_bit_resolution, cmd_adc_res->data());

    auto cmd_fast_mode = extract_cmd_from<SetFastModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_FAST_MODE, cmd_fast_mode->type());
    ASSERT_EQ(_fast_mode, cmd_fast_mode->data());

    auto cmd_timestamped = extract_cmd_from<SetSendTimestampEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SEND_TIMESTAMP_ENABLED, cmd_timestamped->type());
    ASSERT_EQ(_timestamp, cmd_timestamped->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());

    auto cmd_hw_pin = extract_cmd_from<SetHwPinsCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_HW_PINS, cmd_hw_pin->type());
    ASSERT_EQ(_hw_pin, cmd_hw_pin->data());

    auto cmd_hw_type = extract_cmd_from<SetSensorHwTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_HW_TYPE, cmd_hw_type->type());
    ASSERT_EQ(_hw_type, cmd_hw_type->data());

    auto cmd_sensor_type = extract_cmd_from<SetSensorTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENSOR_TYPE, cmd_sensor_type->type());
    ASSERT_EQ(SensorType::DISCRETE_INPUT, cmd_sensor_type->data());
}

TEST_F(TestDiscreteSensorMapper, test_config_fail)
{
    MessageFactory factory;

    auto ret = _mapper.apply_command(CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 0)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 17)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_discrete_ranges_command(_sensor_idx, {})));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_discrete_ranges_command(_sensor_idx, {{0.5f, 0.3f}})));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_discrete_ranges_command(_sensor_idx, {{-0.1f, 0.5f}})));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_discrete_ranges_command(_sensor_idx, {{0.5f, 1.5f}})));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(CMD_PTR(factory.make_set_analog_time_constant_command(_sensor_idx, 0.02f)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);
}

TEST_F(TestDiscreteSensorMapper, test_process)
{
    MessageFactory factory;

    // Raw 500 → normalized ~0.12 → range 0
    auto input_msg = factory.make_analog_value(_sensor_idx, 500);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(0.0f, _backend._last_output_value);

    // Raw 2048 → normalized ~0.50 → range 1
    input_msg = factory.make_analog_value(_sensor_idx, 2048);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(1.0f, _backend._last_output_value);

    // Raw 3500 → normalized ~0.85 → range 2
    input_msg = factory.make_analog_value(_sensor_idx, 3500);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(2.0f, _backend._last_output_value);

    // Raw 4095 → normalized 1.0 → edge-case maps to last range (index 2)
    input_msg = factory.make_analog_value(_sensor_idx, 4095);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(2.0f, _backend._last_output_value);
}

TEST_F(TestDiscreteSensorMapper, test_invert)
{
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_invert_enabled_command(_sensor_idx, true)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    // Raw 0 → normalized 0.0 → inverted 1.0 → edge-case → last range (index 2)
    auto input_msg = factory.make_analog_value(_sensor_idx, 0);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(2.0f, _backend._last_output_value);

    // Raw 4095 → normalized 1.0 → inverted 0.0 → first range (index 0)
    input_msg = factory.make_analog_value(_sensor_idx, 4095);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(0.0f, _backend._last_output_value);
}

TEST_F(TestDiscreteSensorMapper, test_disabled_process_dont_send_values)
{
    MessageFactory factory;
    auto           ret = _mapper.apply_command(CMD_PTR(factory.make_set_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    float fake_reference_value = -1.0;
    _backend._last_output_value = fake_reference_value;

    auto input_msg = factory.make_analog_value(_sensor_idx, 2048);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}

TEST_F(TestDiscreteSensorMapper, test_same_discrete_index_not_sent_twice)
{
    MessageFactory factory;

    // First send for range 1 — goes through because _previous_discrete_index is initially empty
    uint32_t first_message_time = 100;
    auto     input_msg = factory.make_analog_value(_sensor_idx, 2048, first_message_time);
    auto     input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(first_message_time, _backend._last_timestamp);

    // Second send still in range 1 — should be suppressed
    uint32_t second_message_time = 999;
    input_msg = factory.make_analog_value(_sensor_idx, 2100, second_message_time);
    input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(first_message_time, _backend._last_timestamp);
}

TEST_F(TestDiscreteSensorMapper, test_output_timestamp_preserved)
{
    MessageFactory factory;
    uint32_t       ref_time = 123456;
    auto           input_msg = factory.make_analog_value(_sensor_idx, 500, ref_time);
    auto           input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_EQ(ref_time, _backend._last_timestamp);
}

TEST_F(TestDiscreteSensorMapper, test_value_outside_all_ranges)
{
    MessageFactory factory;

    // Configure ranges that don't cover 0
    auto ret = _mapper.apply_command(CMD_PTR(factory.make_set_discrete_ranges_command(_sensor_idx, {{0.5f, 1.0f}})));
    ASSERT_EQ(CommandErrorCode::OK, ret);

    float fake_reference_value = -1.0;
    _backend._last_output_value = fake_reference_value;

    // Raw 0 → normalized 0.0 → below all ranges → no value returned
    auto input_msg = factory.make_analog_value(_sensor_idx, 0);
    auto input_val = static_cast<Value*>(input_msg.get());
    _mapper.process(input_val, &_backend);
    ASSERT_FLOAT_EQ(fake_reference_value, _backend._last_output_value);
}