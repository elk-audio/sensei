#include <json/value.h>
#include "gtest/gtest.h"

//#include "message/base_message.h"
//#include "message/base_command.h"

#define private public
#include "config_backend/json_configuration.cpp"

using namespace sensei;
using namespace config;

static const std::string test_file = "/home/gustav/workspace/sensei/linux/test/unittests/configuration/test_configuration.json";

TEST(JsonConfigurationTestInternal, test_read_configuration)
{
    Json::Value config = read_configuration(test_file);
    EXPECT_FALSE(config.isNull());
    EXPECT_TRUE(config.isObject());
}



class JsonConfigurationTest : public ::testing::Test
{
protected:
    JsonConfigurationTest() :
            _module_under_test(&_queue, test_file)
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    SynchronizedQueue<std::unique_ptr<BaseMessage>>  _queue;
    JsonConfiguration _module_under_test;
};

TEST_F(JsonConfigurationTest, test_apply_configuration)
{
    EXPECT_TRUE(_queue.empty());
    _module_under_test.apply_configuration(test_file);
    ASSERT_FALSE(_queue.empty());
    std::unique_ptr<BaseMessage> m = std::move(_queue.pop());
    Command* c = static_cast<Command*>(m.get());
    EXPECT_EQ(CommandType::SET_PIN_TYPE, c->type());
}
