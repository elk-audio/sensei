
#ifndef SENSEI_JSONCONFIGURATION_H
#define SENSEI_JSONCONFIGURATION_H

#include "message/message_factory.h"
#include "base_configuration.h"
#include <json/json.h>

namespace sensei {
namespace config {

class JsonConfiguration : public BaseConfiguration
{
public:
    JsonConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue, const std::string& file) :
            BaseConfiguration(queue, file)
    {}

    ~JsonConfiguration() = default;

    /*
     * Open file, parse json and put commands in queue
     */
    ConfigStatus read(HwFrontendConfig& hw_config) override;

private:
    ConfigStatus handle_hw_config(const Json::Value& frontend, HwFrontendConfig& config);
    ConfigStatus handle_sensor(const Json::Value& sensor, HwFrontendType hw_type);
    ConfigStatus handle_sensor_hw(const Json::Value& hardware, int sensor_id, bool pins_first);
    ConfigStatus handle_backend(const Json::Value& backend);
    ConfigStatus handle_osc_backend(const Json::Value& backend, int id);
    ConfigStatus handle_imu(const Json::Value& imu);
    ConfigStatus read_pins(const Json::Value& pins, int sensor_id);

    MessageFactory _message_factory;
};


} // end namespace config
} // end namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H
