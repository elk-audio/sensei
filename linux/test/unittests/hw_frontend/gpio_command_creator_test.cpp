
#include "gtest/gtest.h"

#define private public
#include "hardware_frontend/gpio_command_creator.cpp"

using namespace sensei;
using namespace hw_frontend;

class TestGpioCommandCreator : public ::testing::Test
{
protected:
    TestGpioCommandCreator() :_module_under_test()
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    GpioCommandCreator _module_under_test;
};

TEST_F(TestGpioCommandCreator, test_initialize_common_data)
{
    GpioPacket packet = _module_under_test._prepare_packet();
    uint32_t seq_no = from_gpio_protocol_byteord(packet.sequence_no);
    EXPECT_EQ(0, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    for (unsigned int i = 0; i < sizeof(GpioPacket::payload.raw_data) ; ++i)
    {
        EXPECT_EQ(0, packet.payload.raw_data[i]);
    }
    /* Prepare a new packet and make sure the seq no is +1 */
    packet = _module_under_test._prepare_packet();
    uint32_t next_seq_no = from_gpio_protocol_byteord(packet.sequence_no);
    ASSERT_EQ(next_seq_no, seq_no + 1);
}

TEST_F(TestGpioCommandCreator, test_command_creation)
{
    GpioPacket packet = _module_under_test.make_reset_system_command();
    EXPECT_EQ(GPIO_CMD_SYSTEM_CONTROL, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_STOP_RESET_SYSTEM, packet.sub_command);

    packet = _module_under_test.make_start_system_command();
    EXPECT_EQ(GPIO_CMD_SYSTEM_CONTROL, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_START_SYSTEM, packet.sub_command);

    packet = _module_under_test.make_set_tick_rate_command(GPIO_SYSTEM_TICK_1000_HZ);
    EXPECT_EQ(GPIO_CMD_SYSTEM_CONTROL, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_SYSTEM_TICK_RATE, packet.sub_command);
    EXPECT_EQ(GPIO_SYSTEM_TICK_1000_HZ, packet.payload.system_tick_rate_data.gpio_system_tick_rate);

    packet = _module_under_test.make_get_board_info_command();
    EXPECT_EQ(GPIO_CMD_SYSTEM_CONTROL, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_GET_BOARD_INFO, packet.sub_command);

    packet = _module_under_test.make_reset_all_controllers_command();
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_RESET_ALL_CONTROLLERS, packet.sub_command);

    packet = _module_under_test.make_reset_controller_command(5);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_RESET_CONTROLLER, packet.sub_command);
    EXPECT_EQ(5, packet.payload.reset_controller_data.controller_id);

    packet = _module_under_test.make_add_controller_command(6, GPIO_ANALOG_INPUT);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_ADD_CONTROLLER, packet.sub_command);
    EXPECT_EQ(6, packet.payload.add_controller_data.controller_id);
    EXPECT_EQ(GPIO_ANALOG_INPUT, packet.payload.add_controller_data.gpio_hw_type);

    packet = _module_under_test.make_add_controller_to_mux_command(7, 8, 9);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX, packet.sub_command);
    EXPECT_EQ(7, packet.payload.controller_to_mux_data.controller_id);
    EXPECT_EQ(8, packet.payload.controller_to_mux_data.mux_controller_id);
    EXPECT_EQ(9, packet.payload.controller_to_mux_data.mux_controller_pin);

    packet = _module_under_test.make_set_polarity_command(6, GPIO_ACTIVE_HIGH);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_CONTROLLER_POLARITY, packet.sub_command);
    EXPECT_EQ(6, packet.payload.controller_polarity_data.controller_id);
    EXPECT_EQ(GPIO_ACTIVE_HIGH, packet.payload.controller_polarity_data.polarity);

    packet = _module_under_test.make_set_controller_tick_rate_command(10, 5);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE, packet.sub_command);
    EXPECT_EQ(10, packet.payload.controller_tick_rate.controller_id);
    EXPECT_EQ(5, packet.payload.controller_tick_rate.delta_tick_rate);

    packet = _module_under_test.make_set_notification_mode(11, GPIO_ON_VALUE_CHANGE);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE, packet.sub_command);
    EXPECT_EQ(11, packet.payload.controller_notif_data.controller_id);
    EXPECT_EQ(GPIO_ON_VALUE_CHANGE, packet.payload.controller_notif_data.notif_mode);

    Pinlist list = {2, {5,6}};
    packet = _module_under_test.make_add_pins_to_controller_command(12, list);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER, packet.sub_command);
    EXPECT_EQ(12, packet.payload.controller_pins_data.controller_id);
    EXPECT_EQ(2, packet.payload.controller_pins_data.num_pins);
    EXPECT_EQ(5, packet.payload.controller_pins_data.pins[0]);
    EXPECT_EQ(6, packet.payload.controller_pins_data.pins[1]);

    packet = _module_under_test.make_mute_controller_command(13, GPIO_CONTROLLER_UNMUTED);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER, packet.sub_command);
    EXPECT_EQ(13, packet.payload.controller_mute_data.controller_id);
    EXPECT_EQ(GPIO_CONTROLLER_UNMUTED, packet.payload.controller_mute_data.mute_status);

    packet = _module_under_test.make_set_range_command(14, 10, 25);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_CONTROLLER_RANGE, packet.sub_command);
    EXPECT_EQ(14, packet.payload.controller_range_data.controller_id);
    EXPECT_EQ(10u, packet.payload.controller_range_data.min_val);
    EXPECT_EQ(25u, packet.payload.controller_range_data.max_val);

    packet = _module_under_test.make_set_debounce_mode_command(15, GPIO_CONTROLLER_DEBOUNCE_ENABLED);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_CONTROLLER_DEBOUNCE_MODE, packet.sub_command);
    EXPECT_EQ(GPIO_CONTROLLER_DEBOUNCE_ENABLED, packet.payload.controller_debounce_data.controller_debounce_mode);
    EXPECT_EQ(15, packet.payload.controller_debounce_data.controller_id);

    packet = _module_under_test.make_set_analog_resolution_command(16, 8);
    EXPECT_EQ(GPIO_CMD_CONFIG_CONTROLLER, packet.command);
    EXPECT_EQ(GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES, packet.sub_command);
    EXPECT_EQ(16, packet.payload.analog_controller_res_data.controller_id);
    EXPECT_EQ(8u, from_gpio_protocol_byteord(packet.payload.analog_controller_res_data.res_in_bits));

    packet = _module_under_test.make_get_value_command(17);
    EXPECT_EQ(GPIO_CMD_GET_VALUE, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    EXPECT_EQ(17, packet.payload.gpio_value_request.controller_id);

    packet = _module_under_test.make_set_value_command(18, 2048);
    EXPECT_EQ(GPIO_CMD_SET_VALUE, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    EXPECT_EQ(18, packet.payload.gpio_value_data.controller_id);
    EXPECT_EQ(2048u, from_gpio_protocol_byteord(packet.payload.gpio_value_data.controller_val));
}

