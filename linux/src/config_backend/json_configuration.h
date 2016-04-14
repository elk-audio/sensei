
#ifndef SENSEI_JSONCONFIGURATION_H
#define SENSEI_JSONCONFIGURATION_H

#include "message/message_factory.h"
#include "base_configuration.h"
//#include "message/base_message.h"

namespace sensei {
namespace config {

class JsonConfiguration : public base_configuration
{
public:
    JsonConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>> *queue) : base_configuration(queue)
    { }

    JsonConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue, const std::string& file);

    ~JsonConfiguration();
private:
    void apply_configuration(const std::string& file_name);
    int handle_sensor(const Json::Value& sensor);
    int handle_backend(const Json::Value& backend);

    MessageFactory _message_factory;

    std::string _file_name;
};


} // end namespace config
} // end namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H
