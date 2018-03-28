
#include "gtest/gtest.h"

#define private public
#include "hardware_frontend/xmos_command_creator.cpp"

using namespace sensei;
using namespace hw_frontend;

class TestXmosCommandCreator : public ::testing::Test
{
protected:
    TestXmosCommandCreator() :_module_under_test()
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    XmosCommandCreator _module_under_test;
};

TEST_F(TestXmosCommandCreator, test_initialize_common_data)
{
    XmosGpioPacket packet = _module_under_test._prepare_packet();
    uint32_t seq_no = from_xmos_byteord(packet.sequence_no);
    EXPECT_EQ(0, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    for (unsigned int i = 0; i < sizeof(XmosGpioPacket::payload.raw_data) ; ++i)
    {
        EXPECT_EQ(0, packet.payload.raw_data[i]);
    }
    /* Prepare a new packet and make sure the seq no is +1 */
    packet = _module_under_test._prepare_packet();
    uint32_t next_seq_no = from_xmos_byteord(packet.sequence_no);
    ASSERT_EQ(next_seq_no, seq_no + 1);
}

TEST_F(TestXmosCommandCreator, test_command_creation)
{
    XmosGpioPacket packet = _module_under_test.make_reset_system_command();
    EXPECT_EQ(XMOS_CMD_SYSTEM_CNTRL, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_STOP_RESET_SYSTEM, packet.sub_command);

    packet = _module_under_test.make_start_system_command();
    EXPECT_EQ(XMOS_CMD_SYSTEM_CNTRL, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_START_SYSTEM, packet.sub_command);

    packet = _module_under_test.make_stop_system_command();
    EXPECT_EQ(XMOS_CMD_SYSTEM_CNTRL, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_STOP_SYSTEM, packet.sub_command);

    packet = _module_under_test.make_set_tick_rate_command(TICK_1000_HZ);
    EXPECT_EQ(XMOS_CMD_SYSTEM_CNTRL, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_SET_TICK_RATE, packet.sub_command);
    EXPECT_EQ(TICK_1000_HZ, packet.payload.tick_rate_data.system_tick_rate);

    packet = _module_under_test.make_get_board_info_command();
    EXPECT_EQ(XMOS_CMD_SYSTEM_CNTRL, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_GET_BOARD_INFO, packet.sub_command);

    packet = _module_under_test.make_reset_all_controllers_command();
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_RESET_ALL_CNTRLRS, packet.sub_command);

    packet = _module_under_test.make_reset_controller_command(5);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_RESET_CNTRLR, packet.sub_command);
    EXPECT_EQ(5, packet.payload.reset_cntrlr_data.controller_id);

    packet = _module_under_test.make_add_controller_command(6, HwType::ANALOG_INPUT);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_ADD_CNTRLR, packet.sub_command);
    EXPECT_EQ(6, packet.payload.cntrlr_data.controller_id);
    EXPECT_EQ(HwType::ANALOG_INPUT, packet.payload.cntrlr_data.hw_type);

    packet = _module_under_test.make_add_controller_to_mux_command(7, 8, 9);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX, packet.sub_command);
    EXPECT_EQ(7, packet.payload.cntrlr_to_mux_data.controller_id);
    EXPECT_EQ(8, packet.payload.cntrlr_to_mux_data.mux_controller_id);
    EXPECT_EQ(9, packet.payload.cntrlr_to_mux_data.mux_controller_pin);

    packet = _module_under_test.make_set_polarity_command(6, CntrlrPolarity::ACTIVE_HIGH);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_SET_CNTRLR_POLARITY, packet.sub_command);
    EXPECT_EQ(6, packet.payload.cntrlr_polarity_data.controller_id);
    EXPECT_EQ(CntrlrPolarity::ACTIVE_HIGH, packet.payload.cntrlr_polarity_data.polarity);

    packet = _module_under_test.make_set_controller_tick_rate_command(10, 5);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE, packet.sub_command);
    EXPECT_EQ(10, packet.payload.cntrlr_tick_rate.controller_id);
    EXPECT_EQ(5, packet.payload.cntrlr_tick_rate.delta_tick_rate);

    packet = _module_under_test.make_set_notification_mode(11, NotificationMode::ON_VALUE_CHANGE);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE, packet.sub_command);
    EXPECT_EQ(11, packet.payload.notif_mode_data.controller_id);
    EXPECT_EQ(NotificationMode::ON_VALUE_CHANGE, packet.payload.notif_mode_data.notif_mode);

    Pinlist list = {2, {5,6}};
    packet = _module_under_test.make_add_pins_to_controller_command(12, list);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR, packet.sub_command);
    EXPECT_EQ(12, packet.payload.pin_data.controller_id);
    EXPECT_EQ(2, packet.payload.pin_data.num_pins);
    EXPECT_EQ(5, packet.payload.pin_data.pins[0]);
    EXPECT_EQ(6, packet.payload.pin_data.pins[1]);

    packet = _module_under_test.make_mute_controller_command(13, MuteStatus::CNTRLR_UNMUTED);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR, packet.sub_command);
    EXPECT_EQ(13, packet.payload.mute_cmnd_data.controller_id);
    EXPECT_EQ(MuteStatus::CNTRLR_UNMUTED, packet.payload.mute_cmnd_data.mute_status);

    packet = _module_under_test.make_remove_controller_command(14);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_REMOVE_CNTRLR, packet.sub_command);
    EXPECT_EQ(14, packet.payload.remove_cntrlr_data.controller_id);

    packet = _module_under_test.make_set_controller_range_command(15, 1, 1045);
    EXPECT_EQ(XMOS_CMD_CONFIGURE_CNTRLR, packet.command);
    EXPECT_EQ(XMOS_SUB_CMD_SET_CNTRLR_RANGE, packet.sub_command);
    EXPECT_EQ(15, packet.payload.set_cntrlr_range_data.controller_id);
    EXPECT_EQ(1, from_xmos_byteord(packet.payload.set_cntrlr_range_data.min_value));
    EXPECT_EQ(1045, from_xmos_byteord(packet.payload.set_cntrlr_range_data.max_value));

    packet = _module_under_test.make_get_value_command(16);
    EXPECT_EQ(XMOS_CMD_GET_VALUE, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    EXPECT_EQ(16, packet.payload.value_request_data.controller_id);

    packet = _module_under_test.make_set_value_command(17, 2048);
    EXPECT_EQ(XMOS_CMD_SET_VALUE, packet.command);
    EXPECT_EQ(0, packet.sub_command);
    EXPECT_EQ(17, packet.payload.value_send_data.controller_id);
    EXPECT_EQ(2048, from_xmos_byteord(packet.payload.value_send_data.controller_val));
}

