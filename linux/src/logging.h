/**
 * @brief Singleton wrapper around spdlog and custom logging macros
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

#ifndef SENSEI_LOGGING_H
#define SENSEI_LOGGING_H

#include <iostream>

/* Sensei log macros */
#ifndef DISABLE_LOGGING
#include "spdlog/spdlog.h"

/* Add file and line numbers to debug prints */
#define ENABLE_DEBUG_FILE_AND_LINE_NUM

/* Use this macro  at the top of every file to declare a local logger */
#define SENSEI_GET_LOGGER static auto spdlog_instance = sensei::Logger::get_logger()

#ifdef ENABLE_DEBUG_FILE_AND_LINE_NUM
    #define SENSEI_EXTENDED_LOG << " (" << __FILE__ << " @" << __LINE__ <<")"
#else
    #define SENSEI_EXTENDED_LOG
#endif

/*
 * Use these macros to log messages. Use cppformat style, ie:
 * SENSEI_LOG_INFO("Setting x to {} and y to {}", x, y);
 *
 * spdlog supports ostream style, but that doesn't work with
 * -DDISABLE_MACROS unfortunately
 */

#define SENSEI_LOG_DEBUG(...)    spdlog_instance->debug(__VA_ARGS__)  SENSEI_EXTENDED_LOG
#define SENSEI_LOG_INFO(...)     spdlog_instance->info(__VA_ARGS__)
#define SENSEI_LOG_WARNING(...)  spdlog_instance->warn(__VA_ARGS__)
#define SENSEI_LOG_ERROR(...)    spdlog_instance->error(__VA_ARGS__)
#define SENSEI_LOG_CRITICAL(...) spdlog_instance->crit(__VA_ARGS__)

namespace sensei {

class Logger
{
public:
    static std::shared_ptr<spdlog::logger> get_logger();
};

std::shared_ptr<spdlog::logger> setup_logging();

} // end namespace sensei

#else
/* Define empty macros */
#define SENSEI_GET_LOGGER
#define SENSEI_LOG_DEBUG(...)
#define SENSEI_LOG_INFO(...)
#define SENSEI_LOG_WARNING(...)
#define SENSEI_LOG_ERROR(...)
#define SENSEI_LOG_CRITICAL(...)
#endif

#endif //SENSEI_LOGGING_H