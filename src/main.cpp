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
 * @brief Main entry point to SENSEI
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#include <vector>
#include <fstream>
#include <csignal>
#include <cassert>
#include <iostream>

#include "optionparser.h"

#include "event_handler.h"
#include "logging.h"
#include "generated/version.h"

////////////////////////////////////////////////////////////////////////////////
// Global constants
////////////////////////////////////////////////////////////////////////////////

#define SENSEI_DEFAULT_N_INPUT_PINS         64
#define SENSEI_DEFAULT_N_INPUT_PINS_STR     "64"
#define SENSEI_DEFAULT_N_OUTPUT_PINS        32
#define SENSEI_DEFAULT_N_OUTPUT_PINS_STR    "32"
#define SENSEI_DEFAULT_WAIT_PERIOD_MS      10
#define SENSEI_DEFAULT_SLEEP_PERIOD_MS_STR  "10"
#define SENSEI_DEFAULT_CONFIG_FILENAME      "../../../scratch/sensei_config.json"
#define SENSEI_DEFAULT_LOG_FILENAME         "/tmp/sushi.log"
#define SENSEI_DEFAULT_LOG_LEVEL            "info"

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

sensei::EventHandler event_handler;
static volatile sig_atomic_t main_loop_running = 1;
static volatile sig_atomic_t config_reload_pending = 0;

std::string log_level = std::string(SENSEI_DEFAULT_LOG_LEVEL);
std::string log_filename = std::string(SENSEI_DEFAULT_LOG_FILENAME);
std::chrono::seconds log_flush_interval = std::chrono::seconds(0);
bool enable_flush_interval = false;

void print_headline()
{
    std::cout << "SENSEI - Copyright 2017-2019 Elk, Stockholm" << std::endl;
    std::cout << "SENSEI is licensed under the Affero GPL 3.0. Source code is available at github.com/elk-audio" << std::endl;
}

void print_version_and_build_info()
{
    std::cout << "\nVersion "   << SENSEI__VERSION_MAJ << "."
              << SENSEI__VERSION_MIN << "."
              << SENSEI__VERSION_REV << std::endl;

    std::cout << "Git commit: " << SENSEI_GIT_COMMIT_HASH << std::endl;
    std::cout << "Built on: " << SENSEI_BUILD_TIMESTAMP << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Signal handlers
////////////////////////////////////////////////////////////////////////////////

static void kill_signal_handler(int sig_number)
{
    if ( (sig_number == SIGINT) || (sig_number == SIGTERM) )
    {
        main_loop_running = 0;
    }
}

static void user_signal_handler(int sig_number)
{
    if (sig_number == SIGUSR1)
    {
        config_reload_pending = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Command Line parsing helpers
////////////////////////////////////////////////////////////////////////////////

struct SenseiArg : public option::Arg
{

    static void print_error(const char* msg1, const option::Option& opt, const char* msg2)
    {
        fprintf(stderr, "%s", msg1);
        fwrite(opt.name, opt.namelen, 1, stderr);
        fprintf(stderr, "%s", msg2);
    }

    static option::ArgStatus Unknown(const option::Option& option, bool msg)
    {
        if (msg)
        {
            print_error("Unknown option '", option, "'\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus NonEmpty(const option::Option& option, bool msg)
    {
        if (option.arg != 0 && option.arg[0] != 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires a non-empty argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Numeric(const option::Option& option, bool msg)
    {
        char* endptr = 0;
        if (option.arg != 0 && strtol(option.arg, &endptr, 10))
        {};

        if (endptr != option.arg && *endptr == 0)
        {
          return option::ARG_OK;
        }
        if (msg)
        {
            print_error("Option '", option, "' requires a numeric argument\n");
        }
        return option::ARG_ILLEGAL;
    }
};

////////////////////////////////////////////////////////////////////////////////
// Command Line options descriptors
////////////////////////////////////////////////////////////////////////////////

enum OptionIndex
{ 
    UNKNOWN,
    HELP,
    VERSION,
    N_INPUT_PINS,
    N_OUTPUT_PINS,
    SLEEP_PERIOD,
    CONFIG_FILENAME,
    LOG_FILENAME,
    LOG_LEVEL,
    LOG_FLUSH_INTERVAL
};

const option::Descriptor usage[] =
{
    {
        UNKNOWN,
        0,
        "",
        "",
        SenseiArg::Unknown,
        "\nUSAGE: sensei [options]\n\nOptions:"
    },
    {
        HELP,
        0,
        "h?",
        "help",
        SenseiArg::None,
        "\t\t-h --help \tPrint usage and exit."
    },
    {
        VERSION,
        0,
        "v?",
        "version",
        SenseiArg::None,
        "\t\t-v --version \tPrint version and build info and exit."
    },
    {
        N_INPUT_PINS,
        0,
        "i",
        "input-pins",
        SenseiArg::Numeric,
        "\t\t-i <value>, --input-pins=<value> \tSpecify number of configurable pins [default=" SENSEI_DEFAULT_N_INPUT_PINS_STR "]."
    },
    {
        N_OUTPUT_PINS,
        0,
        "o",
        "output-pins",
        SenseiArg::Numeric,
        "\t\t-o <value>, --output-pins=<value> \tSpecify number of digital output pins [default=" SENSEI_DEFAULT_N_OUTPUT_PINS_STR "]."
    },
    {
        SLEEP_PERIOD,
        0,
        "s",
        "sleep",
        SenseiArg::Numeric,
        "\t\t-s <value>, --sleep=<value> \tSpecify event loop sleep period in ms [default=" SENSEI_DEFAULT_SLEEP_PERIOD_MS_STR "]."
    },
    {
        CONFIG_FILENAME,
        0,
        "f",
        "file",
        SenseiArg::NonEmpty,
        "\t\t-f <file>, --file=<file> \tSpecify JSON configuration file [default=" SENSEI_DEFAULT_CONFIG_FILENAME "]."
    },
    {
        LOG_FILENAME,
        0,
        "L",
        "log-file",
        SenseiArg::NonEmpty,
        "\t\t-L <filename>, --log-file=<filename> \tSpecify logging file destination [default=" SENSEI_DEFAULT_LOG_FILENAME "]."
    },
    {
        LOG_LEVEL,
        0,
        "l",
        "log-level",
        SenseiArg::NonEmpty,
        "\t\t-l <level>, --log-level=<level> \tSpecify minimum logging level, from ('debug', 'info', 'warning', 'error')"
    },
    {
        LOG_FLUSH_INTERVAL,
        0,
        "",
        "log-flush-interval",
        SenseiArg::Numeric,
        "\t\t--log-flush-interval=<seconds> \tEnable flushing the log periodically and specify the interval."
    },

    { 0, 0, 0, 0, 0, 0}
};

int main(int argc, char* argv[])
{
    ////////////////////////////////////////////////////////////////////////////////
    // Command Line arguments parsing
    ////////////////////////////////////////////////////////////////////////////////

    // option_parser accepts arguments excluding program name,
    // so skip it if it is present
    if (argc > 0)
    {
        argc--;
        argv++;
    }

    option::Stats cl_stats(usage, argc, argv);
    std::vector<option::Option> cl_options(cl_stats.options_max);
    std::vector<option::Option> cl_buffer(cl_stats.buffer_max);
    option::Parser cl_parser(usage, argc, argv, &cl_options[0], &cl_buffer[0]);

    if (cl_parser.error())
    {
        return 1;
    }
    if (cl_parser.nonOptionsCount() > 0)
    {
        fprintf(stderr, "Unexpected non-optional argument %s\n", cl_parser.nonOption(0));
        return 1;
    }
    if (cl_options[HELP])
    {
        print_headline();
        option::printUsage(fwrite, stdout, usage);
        return 0;
    }
    if (cl_options[VERSION])
    {
        print_headline();
        print_version_and_build_info();
        return 0;
    }

    int n_input_pins = SENSEI_DEFAULT_N_INPUT_PINS;
    int n_output_pins = SENSEI_DEFAULT_N_OUTPUT_PINS;
    std::chrono::milliseconds wait_period_ms{SENSEI_DEFAULT_WAIT_PERIOD_MS};
    std::string config_filename = std::string(SENSEI_DEFAULT_CONFIG_FILENAME);
    for (int i=0; i<cl_parser.optionsCount(); i++)
    {
        option::Option& opt = cl_buffer[i];
        switch(opt.index())
        {
        case HELP:
        case UNKNOWN:
        case VERSION:
            // should be handled before arriving here
            assert(false);
            break;

        case N_INPUT_PINS:
            {
                int parsed_int = atoi(opt.arg);
                // horrible, but that's how atoi works and std::stoi needs exceptions
                if (parsed_int == 0)
                {
                    SenseiArg::print_error("Option '", opt, "' invalid number\n");
                    return 1;
                }
                n_input_pins = parsed_int;
            }
            break;

        case N_OUTPUT_PINS:
            {
                int parsed_int = atoi(opt.arg);
                // horrible, but that's how atoi works and std::stoi needs exceptions
                if (parsed_int == 0)
                {
                    SenseiArg::print_error("Option '", opt, "' invalid number\n");
                    return 1;
                }
                n_output_pins = parsed_int;
            }
            break;

        case SLEEP_PERIOD:
            {
                int parsed_int = atoi(opt.arg);
                // horrible, but that's how atoi works and std::stoi needs exceptions
                if (parsed_int == 0)
                {
                    SenseiArg::print_error("Option '", opt, "' invalid number\n");
                    return 1;
                }
                wait_period_ms = std::chrono::milliseconds(parsed_int);
            }
            break;

        case CONFIG_FILENAME:
            config_filename.assign(opt.arg);
            break;

        case LOG_FILENAME:
            log_filename.assign(opt.arg);
            break;

        case LOG_LEVEL:
            log_level.assign(opt.arg);
            break;

        case LOG_FLUSH_INTERVAL:
            log_flush_interval = std::chrono::seconds(std::strtol(opt.arg, nullptr, 0));
            enable_flush_interval = true;
            break;

        default:
            SenseiArg::print_error("Unhandled option '", opt, "' \n");
            break;
        }
    }

    // Post-config checks
    std::ifstream config_file(config_filename.c_str());
    if (! config_file.good())
    {
        std::cerr << "Failed to open config file " << config_filename << std::endl;
        config_file.close();
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Initialization
    ////////////////////////////////////////////////////////////////////////////////

    signal(SIGINT, kill_signal_handler);
    signal(SIGTERM, kill_signal_handler);
    signal(SIGUSR1, user_signal_handler);

    if(!event_handler.init(n_input_pins, n_output_pins, config_filename))
    {
        std::cerr << "Failed to initialize, check logs for details. Exiting..."
                  << std::endl;
        event_handler.deinit();
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Logger configuration
    ////////////////////////////////////////////////////////////////////////////////
    auto ret_code = SENSEI_INITIALIZE_LOGGER(log_filename, "Logger", log_level, enable_flush_interval, log_flush_interval);
    if (ret_code != SENSEI_LOG_ERROR_CODE_OK)
    {
        std::cerr << SENSEI_LOG_GET_ERROR_MESSAGE(ret_code) << ", using default." << std::endl;
    }

    SENSEI_GET_LOGGER_WITH_MODULE_NAME("main");

    ////////////////////////////////////////////////////////////////////////////////
    // Main loop
    ////////////////////////////////////////////////////////////////////////////////

    SENSEI_LOG_INFO("Starting  main loop");
    while (main_loop_running)
    {
        event_handler.handle_events(wait_period_ms);
        if (config_reload_pending)
        {
            event_handler.reload_config();
            config_reload_pending = 0;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Deinit
    ////////////////////////////////////////////////////////////////////////////////

    event_handler.deinit();
    std::cout << "Sensei terminated." << std::endl;

    return 0;
}