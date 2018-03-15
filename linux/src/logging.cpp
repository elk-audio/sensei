
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
    const int  MAX_LOG_FILE_SIZE    = 10'0000'000;              // In bytes
    const bool LOGGER_FORCE_FLUSH   = true;
    const spdlog::level::level_enum MIN_LOG_LEVEL = spdlog::level::debug;

    spdlog::set_level(MIN_LOG_LEVEL);
    spdlog::set_async_mode(LOGGER_QUEUE_SIZE);
    auto async_file_logger = spdlog::rotating_logger_mt(LOGGER_NAME,
                                                        LOGGER_FILE,
                                                        MAX_LOG_FILE_SIZE,
                                                        1,
                                                        LOGGER_FORCE_FLUSH);

    async_file_logger->info("#############################");
    async_file_logger->info("   Started Sensei Logger!");
    async_file_logger->info("#############################");
    return async_file_logger;
}

} // end namespace sensei

#endif // DISABLE_LOGGING
