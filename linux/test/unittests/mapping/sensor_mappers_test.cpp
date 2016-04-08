#include <vector>
#include <memory>
#include <iterator>

#include "gtest/gtest.h"

#include "mapping/sensor_mappers.cpp"

using namespace sensei;

// Helper functions

template<typename Derived, typename Base>
std::unique_ptr<Derived>
static_unique_ptr_cast( std::unique_ptr<Base>&& p )
{
    auto d = static_cast<Derived *>(p.release());
    return std::unique_ptr<Derived>(d);
}

#define __CMD_UPTR(msg) static_unique_ptr_cast<Command, BaseMessage>(msg)
#define __CMD_PTR(msg) __CMD_UPTR(msg).get()

template<typename DerivedCommand>
std::unique_ptr<DerivedCommand> extract_cmd_from(std::vector<std::unique_ptr<BaseMessage>>& msg_queue)
{
    auto tmp_msg = static_unique_ptr_cast<DerivedCommand, BaseMessage>(std::move(msg_queue.back()));
    msg_queue.pop_back();

    return std::move(tmp_msg);
}


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
        MessageFactory factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));

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
    int _sensor_idx{2};
    bool _enabled{true};
    SendingMode _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool _inverted{true};

    DigitalSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestDigitalSensorMapper, test_digital_configuration)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    auto cmd_pintype = extract_cmd_from<SetPinTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_PIN_TYPE, cmd_pintype->type());
    ASSERT_EQ(PinType::DIGITAL_INPUT, cmd_pintype->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());
}


TEST_F(TestDigitalSensorMapper, test_digital_config_fail)
{
    // Verify that some wrong commands return unhandled error
    MessageFactory factory;

    auto ret = _mapper.apply_command(__CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 12)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_slider_mode_enabled_command(_sensor_idx, false)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_lowpass_cutoff_command(_sensor_idx, 1000.0f)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);
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
        MessageFactory factory;
        std::vector<std::unique_ptr<Command>> config_cmds;
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_enabled_command(_sensor_idx, _enabled))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_sending_mode_command(_sensor_idx, _sending_mode))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_invert_enabled_command(_sensor_idx, _inverted))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, _delta_ticks))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, _adc_bit_resolution))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_lowpass_filter_order_command(_sensor_idx, _lowpass_filter_order))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_lowpass_cutoff_command(_sensor_idx, _lowpass_cutoff))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_slider_mode_enabled_command(_sensor_idx, _slider_mode_enabled))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_slider_threshold_command(_sensor_idx, _slider_threshold))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_input_scale_range_low(_sensor_idx, _input_scale_low))));
        config_cmds.push_back(std::move(__CMD_UPTR(factory.make_set_input_scale_range_high(_sensor_idx, _input_scale_high))));

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
    int _sensor_idx{2};
    bool _enabled{true};
    SendingMode _sending_mode{SendingMode::ON_VALUE_CHANGED};
    bool _inverted{true};
    int _delta_ticks{5};
    int _lowpass_filter_order{4};
    float _lowpass_cutoff{199.123f};
    int _adc_bit_resolution{12};
    bool _slider_mode_enabled{true};
    int _slider_threshold{9};
    int _input_scale_low{23};
    int _input_scale_high{2322};

    AnalogSensorMapper _mapper{_sensor_idx};
};

TEST_F(TestAnalogSensorMapper, test_analog_configuration)
{
    // Get the config back inside a local container and check values in a LIFO manner
    std::vector<std::unique_ptr<BaseMessage>> stored_cmds;
    _mapper.put_config_commands_into(std::back_inserter(stored_cmds));

    auto cmd_scale_high = extract_cmd_from<SetInputScaleRangeHigh>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INPUT_SCALE_RANGE_HIGH, cmd_scale_high->type());
    ASSERT_EQ(_input_scale_high, cmd_scale_high->data());

    auto cmd_scale_low = extract_cmd_from<SetInputScaleRangeLow>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INPUT_SCALE_RANGE_LOW, cmd_scale_low->type());
    ASSERT_EQ(_input_scale_low, cmd_scale_low->data());

    auto cmd_slider_thr = extract_cmd_from<SetSliderThresholdCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SLIDER_THRESHOLD, cmd_slider_thr->type());
    ASSERT_EQ(_slider_threshold, cmd_slider_thr->data());

    auto cmd_slider_mode = extract_cmd_from<SetSliderModeEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SLIDER_MODE_ENABLED, cmd_slider_mode->type());
    ASSERT_EQ(_slider_mode_enabled, cmd_slider_mode->data());

    auto cmd_cutoff = extract_cmd_from<SetLowpassCutoffCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_LOWPASS_CUTOFF, cmd_cutoff->type());
    ASSERT_EQ(_lowpass_cutoff, cmd_cutoff->data());

    auto cmd_filt_ord = extract_cmd_from<SetLowpassFilterOrderCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_LOWPASS_FILTER_ORDER, cmd_filt_ord->type());
    ASSERT_EQ(_lowpass_filter_order, cmd_filt_ord->data());

    auto cmd_adc_res = extract_cmd_from<SetADCBitResolutionCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ADC_BIT_RESOLUTION, cmd_adc_res->type());
    ASSERT_EQ(_adc_bit_resolution, cmd_adc_res->data());

    auto cmd_delta_ticks = extract_cmd_from<SetSendingDeltaTicksCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, cmd_delta_ticks->type());
    ASSERT_EQ(_delta_ticks, cmd_delta_ticks->data());

    auto cmd_pintype = extract_cmd_from<SetPinTypeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_PIN_TYPE, cmd_pintype->type());
    ASSERT_EQ(PinType::ANALOG_INPUT, cmd_pintype->data());

    auto cmd_invert = extract_cmd_from<SetInvertEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_INVERT_ENABLED, cmd_invert->type());
    ASSERT_EQ(_inverted, cmd_invert->data());

    auto cmd_send_mode = extract_cmd_from<SetSendingModeCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_SENDING_MODE, cmd_send_mode->type());
    ASSERT_EQ(_sending_mode, cmd_send_mode->data());

    auto cmd_enabled = extract_cmd_from<SetEnabledCommand>(stored_cmds);
    ASSERT_EQ(CommandType::SET_ENABLED, cmd_enabled->type());
    ASSERT_EQ(_enabled, cmd_enabled->data());

}

TEST_F(TestAnalogSensorMapper, test_analog_config_fail)
{
    // Verify that wrong commands return appropriate errors
    MessageFactory factory;

    auto ret = _mapper.apply_command(__CMD_PTR(factory.make_set_sampling_rate_command(_sensor_idx, 1000.0f)));
    ASSERT_EQ(CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, -10)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);
    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_adc_bit_resolution_command(_sensor_idx, 100)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_sending_delta_ticks_command(_sensor_idx, -23)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_lowpass_filter_order_command(_sensor_idx, 0)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);
    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_lowpass_filter_order_command(_sensor_idx, 100)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_slider_threshold_command(_sensor_idx, -20)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);
    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_slider_threshold_command(_sensor_idx, 123456)));
    ASSERT_EQ(CommandErrorCode::INVALID_VALUE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_input_scale_range_low(_sensor_idx, -20)));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);
    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_input_scale_range_high(_sensor_idx, -20)));
    ASSERT_EQ(CommandErrorCode::INVALID_RANGE, ret);

    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_input_scale_range_low(_sensor_idx, _input_scale_high+1)));
    ASSERT_EQ(CommandErrorCode::CLIP_WARNING, ret);
    ret = _mapper.apply_command(__CMD_PTR(factory.make_set_input_scale_range_high(_sensor_idx, _input_scale_low-1)));
    ASSERT_EQ(CommandErrorCode::CLIP_WARNING, ret);
}

