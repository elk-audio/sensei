/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
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
 * @brief Singleton wrapper around spdlog and custom logging macros
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Currently all logs end up in log.txt in the same folder as the executable
 *
 * If -DDISABLE_LOGGING is passed as a compiler argument, all logging code
 * disappears without a trace. Useful for testing and outside releases.
 *
 * Usage:
 * Call SENSEI_GET_LOGGER before any code in each file.
 *
 * Write to the logger using the SENSEI_LOG_XXX macros with cppformat style
 * ie: SENSEI_LOG_INFO("Setting x to {} and y to {}", x, y);
 *
 * spdlog supports ostream style too, but that doesn't work with
 * -DDISABLE_MACROS unfortunately
 */
#include "logging.h"
#ifndef DISABLE_LOGGING

namespace sensei {

std::shared_ptr<spdlog::logger> Logger::get_logger()
{
    /*
     * Note: A static function variable avoids all initialization
     * order issues associated with a static member variable
     */
    static auto spdlog_instance = setup_logging();
    if (!spdlog_instance)
    {
        std::cerr << "Error, logger is not initialized properly!!! " << std::endl;
    }
    return spdlog_instance;
}

std::shared_ptr<spdlog::logger> setup_logging()
{
    /*
     * Note, configuration parameters are defined here to guarantee
     * that they are defined before calling get_logger()
     */
    const size_t LOGGER_QUEUE_SIZE  = 4096;                  // Should be power of 2
    const std::string LOGGER_FILE   = "log";
    const std::string LOGGER_NAME   = "Sensei_logger";
    const int  MAX_LOG_FILE_SIZE    = 10'000'000;            // In bytes
    const auto MIN_FLUSH_LEVEL      = spdlog::level::err;    // Min level for automatic flush
    const spdlog::level::level_enum MIN_LOG_LEVEL = spdlog::level::warn;

    spdlog::set_level(MIN_LOG_LEVEL);
    spdlog::set_pattern("[%Y-%m-%d %T.%e] [%l] %v");
    spdlog::set_async_mode(LOGGER_QUEUE_SIZE);
    auto async_file_logger = spdlog::rotating_logger_mt(LOGGER_NAME,
                                                        LOGGER_FILE,
                                                        MAX_LOG_FILE_SIZE,
                                                        1);

    async_file_logger->flush_on(MIN_FLUSH_LEVEL);
    async_file_logger->info("#############################");
    async_file_logger->info("   Started Sensei Logger!");
    async_file_logger->info("#############################");
    return async_file_logger;
}

} // namespace sensei

#endif // DISABLE_LOGGING