
#include "gtest/gtest.h"

#define private public
#include "serial_frontend/serial_command_creator.cpp"


using namespace sensei;
using namespace serial_frontend;

static const uint32_t test_tstamp = test_tstamp;
static const int MAX_NUMBER_OFF_PINS = 64;

class TestSerialCommandCreator : public ::testing::Test
{
protected:
    TestSerialCommandCreator() :
            _module_under_test(MAX_NUMBER_OFF_PINS)
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    SerialCommandCreator _module_under_test;
};

TEST_F(TestSerialCommandCreator, test_initialization)
{
    pin_config p = _module_under_test._cfg_cache[MAX_NUMBER_OFF_PINS -1];
    ASSERT_EQ(0, p.pintype);
    ASSERT_EQ(0, p.cfg_data.idxPin);
    ASSERT_EQ(0, p.cfg_data.sendingMode);
    ASSERT_EQ(0, p.cfg_data.deltaTicksContinuousMode);
    ASSERT_EQ(0, p.cfg_data.ADCBitResolution);
    ASSERT_FLOAT_EQ(0, p.cfg_data.lowPassCutOffFilter);
    ASSERT_EQ(0, p.cfg_data.sliderMode);
    ASSERT_EQ(0, p.cfg_data.sliderThreshold);
}

TEST_F(TestSerialCommandCreator, test_initialize_common_data)
{
    sSenseiDataPacket packet;
    initialize_common_data(packet, test_tstamp, CONFIGURE_PIN);
    ASSERT_EQ(0, compare_packet_header(START_SIGNATURE, packet.start_header));
    ASSERT_EQ(0, compare_packet_header(STOP_SIGNATURE, packet.stop_header));
    ASSERT_EQ((uint32_t)test_tstamp, packet.timestamp);
    ASSERT_EQ(CONFIGURE_PIN, packet.cmd);
}


TEST_F(TestSerialCommandCreator, test_make_set_digital_pin_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_set_digital_pin_cmd(5, test_tstamp, 1);
    const teensy_set_value_cmd* data = reinterpret_cast<const teensy_set_value_cmd*>(&packet->payload);
    ASSERT_EQ(SET_PIN, packet->sub_cmd);
    ASSERT_EQ(SET_DIGITAL_PINS, packet->cmd);
    ASSERT_EQ(5, data->pin_idx);
    ASSERT_EQ(1, data->value);
    ASSERT_EQ(test_tstamp, packet->timestamp);
}

TEST_F(TestSerialCommandCreator, test_make_set_bank_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_set_bank_cmd(5, test_tstamp, 1);
    const teensy_set_value_cmd* data = reinterpret_cast<const teensy_set_value_cmd*>(&packet->payload);
    ASSERT_EQ(SET_BANK, packet->sub_cmd);
    ASSERT_EQ(SET_DIGITAL_PINS, packet->cmd);
    ASSERT_EQ(5, data->pin_idx);
    ASSERT_EQ(1, data->value);
    ASSERT_EQ(test_tstamp, packet->timestamp);
}

TEST_F(TestSerialCommandCreator, test_make_set_sampling_rate_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_set_sampling_rate_cmd(test_tstamp, 500);
    const teensy_set_samplerate_cmd* data = reinterpret_cast<const teensy_set_samplerate_cmd*>(&packet->payload);
    ASSERT_EQ(EMPTY, packet->sub_cmd);
    ASSERT_EQ(SET_SAMPLING_RATE, packet->cmd);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(2, data->sample_rate_divisor);

    packet = _module_under_test.make_set_sampling_rate_cmd(test_tstamp, 2);
    data = reinterpret_cast<const teensy_set_samplerate_cmd*>(&packet->payload);
    ASSERT_EQ(0, data->sample_rate_divisor);

    packet = _module_under_test.make_set_sampling_rate_cmd(test_tstamp, 333);
    data = reinterpret_cast<const teensy_set_samplerate_cmd*>(&packet->payload);
    ASSERT_EQ(3, data->sample_rate_divisor);
}

TEST_F(TestSerialCommandCreator, test_make_get_value_cmd)
{
    const sSenseiDataPacket *packet = _module_under_test.make_get_value_cmd(12, test_tstamp);
    const teensy_set_value_cmd* data = reinterpret_cast<const teensy_set_value_cmd*>(&packet->payload);
    ASSERT_EQ(EMPTY, packet->sub_cmd);
    ASSERT_EQ(GET_VALUE, packet->cmd);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(12, data->pin_idx);
}

TEST_F(TestSerialCommandCreator, test_make_config_pintype_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_pintype_cmd(3, test_tstamp, PIN_ANALOG_INPUT);
    ASSERT_EQ(PIN_ANALOG_INPUT, packet->sub_cmd);
    ASSERT_EQ(test_tstamp, packet->timestamp);
}

TEST_F(TestSerialCommandCreator, test_make_config_sendingmode_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_sendingmode_cmd(3, test_tstamp, PIN_SENDING_MODE_CONTINUOUS);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(PIN_SENDING_MODE_CONTINUOUS, pin_config->sendingMode);
}

TEST_F(TestSerialCommandCreator, test_make_config_delta_ticks_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_delta_ticks_cmd(3, test_tstamp, 10);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(10, pin_config->deltaTicksContinuousMode);
}

TEST_F(TestSerialCommandCreator, test_make_config_bitres_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_bitres_cmd(3, test_tstamp, PIN_ADC_RESOLUTION_10_BIT);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(PIN_ADC_RESOLUTION_10_BIT, pin_config->ADCBitResolution);
}

TEST_F(TestSerialCommandCreator, test_make_config_filter_order_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_filter_order_cmd(3, test_tstamp, 4);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(4, pin_config->filterOrder);
}

TEST_F(TestSerialCommandCreator, test_make_config_lowpass_cutoff_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_lowpass_cutoff_cmd(3, test_tstamp, 1.234);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_FLOAT_EQ(1.234, pin_config->lowPassCutOffFilter);
}

TEST_F(TestSerialCommandCreator, test_make_config_slidermode_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_slidermode_cmd(3, test_tstamp, 1);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(1, pin_config->sliderMode);
}

TEST_F(TestSerialCommandCreator, test_make_config_slider_threshold_cmd)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_slider_threshold_cmd(3, test_tstamp, 5);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(test_tstamp, packet->timestamp);
    ASSERT_EQ(5, pin_config->sliderThreshold);
}

TEST_F(TestSerialCommandCreator, test_cacheing)
{
    const sSenseiDataPacket* packet = _module_under_test.make_config_bitres_cmd(3, test_tstamp, PIN_ADC_RESOLUTION_10_BIT);
    packet = _module_under_test.make_config_pintype_cmd(3, test_tstamp, PIN_ANALOG_INPUT);
    packet = _module_under_test.make_config_sendingmode_cmd(3, 0x12341234, PIN_SENDING_MODE_CONTINUOUS);
    const sPinConfiguration* pin_config = reinterpret_cast<const sPinConfiguration*>(&packet->payload);
    ASSERT_EQ(static_cast<uint32_t>(0x12341234), packet->timestamp);
    ASSERT_EQ(PIN_ADC_RESOLUTION_10_BIT, pin_config->ADCBitResolution);
    ASSERT_EQ(PIN_ANALOG_INPUT, packet->sub_cmd);
    ASSERT_EQ(PIN_SENDING_MODE_CONTINUOUS, pin_config->sendingMode);
}