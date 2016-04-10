#include "gtest/gtest.h"

#include "mapping/mapping_processor.cpp"
#include "message/message_factory.h"

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

        // Set pin 0 to digital
        auto cmd_msg = factory.make_set_pin_type_command(0, PinType::DIGITAL_INPUT);
        auto cmd = std::unique_ptr<Command>(static_cast<Command*>(cmd_msg.release()));
        config_cmds.push_back(std::move(cmd));

        // Set pin 1 to analog
        cmd_msg = factory.make_set_pin_type_command(1, PinType::ANALOG_INPUT);
        cmd = std::unique_ptr<Command>(static_cast<Command*>(cmd_msg.release()));
        config_cmds.push_back(std::move(cmd));

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
            if (msg->sensor_index() != sensor_index)
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
    OutputValueContainer out_values;

    auto input_msg = factory.make_digital_value(2, false);
    auto input_val = static_cast<Value*>(input_msg.get());

    _processor.process(input_val, std::back_inserter(out_values));
    ASSERT_TRUE(out_values.empty());
}

