
#ifndef SENSEI_JSONCONFIGURATION_H
#define SENSEI_JSONCONFIGURATION_H

#include "message/message_factory.h"
#include "base_configuration.h"
//#include "message/base_message.h"

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
    void read() override;

private:
    int handle_pin(const Json::Value &pin);
    int handle_backend(const Json::Value& backend);
    int handle_osc_backend(const Json::Value& backend, int id);
    int handle_stdout_backend(const Json::Value& backend, int id);

    MessageFactory _message_factory;
};


} // end namespace config
} // end namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H
