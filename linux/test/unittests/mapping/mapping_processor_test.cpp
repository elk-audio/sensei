#include "gtest/gtest.h"

#include "mapping/mapping_processor.cpp"
#include "message/message_factory.h"
#include "output_backend_mockup.h"

#include "../test_utils.h"

using namespace sensei;
using namespace sensei::mapping;


class TestMappingProcessor : public ::testing::Test
{
protected:

    TestMappingProcessor()
    {
    }

    ~TestMappingProcessor()
    {
    }

    void SetUp()
    {
        MessageFactory factory;
        std::vector<std::unique_ptr<Command>> config_cmds;

        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_type_command(0, PinType::DIGITAL_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_type_command(1, PinType::ANALOG_INPUT))));

        for (auto const& c : config_cmds)
        {
            _processor.apply_command(c.get());
        }

    }

    void TearDown()
    {
    }

    PinType stored_pin_config(int sensor_index)
    {
        std::vector<std::unique_ptr<BaseMessage>> stored_cmds;

        // Test that digital pin has been created and configured
        _processor.put_config_commands_into(std::back_inserter(stored_cmds));

        PinType pin_type = PinType::UNDEFINED;
        for (auto const& msg : stored_cmds)
        {
            if (msg->index() != sensor_index)
            {
                continue;
            }
            auto cmd_msg = static_cast<Command *>(msg.get());

            CommandType cmd_type = cmd_msg->type();
            switch (cmd_type)
            {
            case CommandType::SET_PIN_TYPE:
            {
                auto typed_cmd = static_cast<SetPinTypeCommand*>(cmd_msg);
                pin_type = typed_cmd->data();
            };
                break;

            default:
                break;
            }
        }

        return pin_type;
    }

protected:
    int _max_n_sensors{64};

    MappingProcessor _processor{_max_n_sensors};
};

TEST_F(TestMappingProcessor, test_get_config)
{
    ASSERT_EQ(PinType::DIGITAL_INPUT, stored_pin_config(0));
    ASSERT_EQ(PinType::ANALOG_INPUT, stored_pin_config(1));
    ASSERT_EQ(PinType::UNDEFINED, stored_pin_config(2));
}

TEST_F(TestMappingProcessor, undefined_mappers_return_empty_process)
{
    MessageFactory factory;
    OutputBackendMockup backend;

    auto input_msg = factory.make_digital_value(2, false);
    auto input_val = static_cast<Value*>(input_msg.get());

    // Put some weird value out-of-range and verify that is not touched by process
    float fake_reference_value = -123456.789f;
    backend._last_output_value = fake_reference_value;
    _processor.process(input_val, &backend);
    ASSERT_FLOAT_EQ(fake_reference_value, backend._last_output_value);
}

