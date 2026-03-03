/*
 * Copyright 2017-2026 Elk Audio AB
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Base class for configuration backends
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 *
 * This class should not be instanciated directly, it should only be used to derive
 * concrete configuration classes
 */
#ifndef SENSEI_BASECONFIGURATION_H
#define SENSEI_BASECONFIGURATION_H

#include "message/base_message.h"
#include "message/command_defs.h"
#include "handler_interface.h"
#include "synchronized_queue.h"

namespace sensei {
namespace config {

enum class ConfigStatus
{
    OK,
    IO_ERROR,
    PARSING_ERROR,
    PARAMETER_ERROR,
};

struct HwFrontendConfig
{
    HwFrontendType type;
    std::string    port;
};

struct BackendConfig
{
    BackendType type;
};

struct Config
{
    HwFrontendConfig hw_config;
    BackendConfig    backend_config;
};

class BaseConfiguration
{

public:
    BaseConfiguration(MessageHandler* handler, const std::string& source,
                      ThreadingMode threading_mode = ThreadingMode::ASYNCHRONOUS)
        : _handler(handler),
          _source(source),
          _enabled(false),
          _threading_mode(threading_mode)
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
    virtual ConfigStatus read(Config& /*config*/)
    {
        return ConfigStatus::OK;
    }

    /**
     * @brief Receive configuration data for exporting to file/socket/etc
     */
    // TODO - Think about input variables here
    void export_data(int m);


protected:
    MessageHandler* _handler;
    std::string     _source;
    bool            _enabled;
    ThreadingMode   _threading_mode;
};

} // namespace config
} // namespace sensei

#endif //SENSEI_BASECONFIGURATION_H