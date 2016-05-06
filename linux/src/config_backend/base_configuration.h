/**
 * @brief Base class for configuration backends
 * @copyright MIND Music Labs AB, Stockholm
 *
 * This class should not be instanciated directly, it should only be used to derive
 * concrete configuration classes
 */

#ifndef SENSEI_BASECONFIGURATION_H
#define SENSEI_BASECONFIGURATION_H

#include "message/base_message.h"
#include "synchronized_queue.h"

namespace sensei {
namespace config {

enum class ConfigStatus {
    OK,
    IO_ERROR,
    PARSING_ERROR,
    PARAMETER_ERROR,
};



class BaseConfiguration
{

public:
    BaseConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue, const std::string& source) :
            _queue(queue),
            _source(source),
            _enabled(false)
    {
    }

    virtual ~BaseConfiguration()
    {
    }

    /**
    * @brief Set the update mode of this configuration backend.  If set to true,
    * updates will be converted into internal commands and put in the queue. If
    * not, they will be silently dropped.
    *
    * @param [in] enabled sets updates enabled/disabled
    */
    void updates(bool enabled)
    {
        _enabled = enabled;
    }

    /**
     * @brief Returns the update state of the configuration backend
     */
    bool is_enabled()
    {
        return _enabled;
    }

    /**
     * @brief Read configuration and construct messages from it
     */
    virtual ConfigStatus read()
    {
        return ConfigStatus::OK;
    }
    /**
     * @brief Receive configuration data for exporting to file/socket/etc
     */
    // TODO - Think about input variables here
    void export_data(int m);


protected:
    SynchronizedQueue<std::unique_ptr<BaseMessage>>* _queue;
    std::string _source;
    bool _enabled;

};







}  // end namespace config
}  // end namespace sensei



#endif //SENSEI_BASECONFIGURATION_H
