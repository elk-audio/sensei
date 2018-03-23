
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
    {
    }

    ~JsonConfiguration()
    {

    }
    /*
     * Open file, parse json and put commands in queue
     */
    ConfigStatus read() override;

private:
    ConfigStatus handle_sensor(const Json::Value& sensor);
    ConfigStatus handle_sensor_hw(const Json::Value& hardware, int sensor_id);
    ConfigStatus handle_backend(const Json::Value& backend);
    ConfigStatus handle_osc_backend(const Json::Value& backend, int id);
    ConfigStatus handle_imu(const Json::Value& imu);

    MessageFactory _message_factory;
};


} // end namespace config
} // end namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H
