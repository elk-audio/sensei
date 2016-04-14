#include <vector>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <cstdio>
#include <cassert>

#include "optionparser.h"

#include "event_handler.h"
#include "serial_frontend/serial_frontend.h"

////////////////////////////////////////////////////////////////////////////////
// Global constants
////////////////////////////////////////////////////////////////////////////////

#define SENSEI_DEFAULT_N_PINS               64
#define SENSEI_DEFAULT_N_PINS_STR           "64"
#define SENSEI_DEFAULT_SLEEP_PERIOD_MS      10
#define SENSEI_DEFAULT_SLEEP_PERIOD_MS_STR  "10"
#define SENSEI_DEFAULT_SERIAL_DEVICE        "/dev/ttyS01"

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////

sensei::EventHandler event_handler;
static volatile sig_atomic_t main_loop_running = 1;

////////////////////////////////////////////////////////////////////////////////
// Signal handlers
////////////////////////////////////////////////////////////////////////////////

static void system_signal_handler(int sig_number)
{
    if ( (sig_number == SIGINT) || (sig_number == SIGTERM) )
    {
        main_loop_running = 0;
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
    PORT,
    N_PINS,
    SLEEP_PERIOD
};

const option::Descriptor usage[] =
{
    {
        UNKNOWN,
        0,
        "",
        "",
        SenseiArg::Unknown,
        "SEnsus SEnsor Interface. Copyright 2016 MIND Music Labs\n\nUSAGE: sensei [options]\n\nOptions:"
    },
    {
        HELP,
        0,
        "h?",
        "help",
        SenseiArg::None,
        "\t-h --help \tPrint usage and exit."
    },
    {
        PORT,
        0,
        "p",
        "port",
        SenseiArg::NonEmpty,
        "\t-p <device>, --port=<device> \tSpecify serial port device [default=" SENSEI_DEFAULT_SERIAL_DEVICE "]."
    },
    {
        N_PINS,
        0,
        "n",
        "pins",
        SenseiArg::Numeric,
        "\t-n <value>, --pins=<value> \tSpecify number of configurable pins [default=" SENSEI_DEFAULT_N_PINS_STR "]."
    },
    {
        SLEEP_PERIOD,
        0,
        "s",
        "sleep",
        SenseiArg::Numeric,
        "\t-s <value>, --sleep=<value> \tSpecify event loop sleep period in ms [default=" SENSEI_DEFAULT_SLEEP_PERIOD_MS_STR "]."
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
        option::printUsage(fwrite, stdout, usage);
        return 0;
    }

    int n_pins = SENSEI_DEFAULT_N_PINS;
    std::chrono::milliseconds sleep_period_ms{SENSEI_DEFAULT_SLEEP_PERIOD_MS};
    std::string port_name = std::string(SENSEI_DEFAULT_SERIAL_DEVICE);
    for (int i=0; i<cl_parser.optionsCount(); i++)
    {
        option::Option& opt = cl_buffer[i];
        switch(opt.index())
        {
        case HELP:
        case UNKNOWN:
            // should be handled before arriving here
            assert(false);
            break;

        case N_PINS:
            {
                int parsed_int = atoi(opt.arg);
                // horrible, but that's how atoi works and std::stoi needs exceptions
                if (parsed_int == 0)
                {
                    SenseiArg::print_error("Option '", opt, "' invalid number\n");
                    return 1;
                }
                n_pins = parsed_int;
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
                sleep_period_ms = std::chrono::milliseconds(parsed_int);
            }
            break;

        case PORT:
            port_name.assign(opt.arg);
            break;

        default:
            SenseiArg::print_error("Unhandled option '", opt, "' \n");
            break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Initialization
    ////////////////////////////////////////////////////////////////////////////////

    signal(SIGINT, system_signal_handler);
    signal(SIGTERM, system_signal_handler);

    event_handler.init(port_name, n_pins);

    ////////////////////////////////////////////////////////////////////////////////
    // Main loop
    ////////////////////////////////////////////////////////////////////////////////

    while (main_loop_running)
    {
        event_handler.handle_events();
        std::this_thread::sleep_for(sleep_period_ms);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Deinit
    ////////////////////////////////////////////////////////////////////////////////

    event_handler.deinit();
    printf("I have deinitialized everything, ready to shut down.\n");

    return 0;
}

