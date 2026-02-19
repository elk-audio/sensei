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
 * @brief Command line options for raspa_mockup
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */

#ifndef RASPA_MOCKUP_OPTIONS_H
#define RASPA_MOCKUP_OPTIONS_H

#include <cstdio>
#include <iostream>
#include <optional>
#include <vector>

#include "../../../third-party/optionparser/optionparser.h"

////////////////////////////////////////////////////////////////////////////////
// Default values
////////////////////////////////////////////////////////////////////////////////

constexpr auto DEFAULT_SEND_INTERVAL = std::chrono::milliseconds(500);
constexpr int DEFAULT_SENSOR_ID = 5;

////////////////////////////////////////////////////////////////////////////////
// Command Line parsing helpers
////////////////////////////////////////////////////////////////////////////////

struct RaspaArg : public option::Arg
{
    static void print_error(const char* msg1, const option::Option& opt, const char* msg2)
    {
        std::cerr << msg1 << std::string(opt.name, opt.namelen) << msg2;
    }

    static option::ArgStatus Unknown(const option::Option& option, bool msg)
    {
        if (msg)
        {
            print_error("Unknown option '", option, "'\n");
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
    SEND_INTERVAL,
    SENSOR_ID
};

const option::Descriptor usage[] =
{
    {
        UNKNOWN,
        0,
        "",
        "",
        RaspaArg::Unknown,
        "\nUSAGE: mockup [options]\n\nOptions:"
    },
    {
        HELP,
        0,
        "h?",
        "help",
        option::Arg::None,
        "\t\t-h --help \tPrint usage and exit."
    },
    {
        SEND_INTERVAL,
        0,
        "i",
        "interval",
        RaspaArg::Numeric,
        "\t\t-i <value>, --interval=<value> \tSpecify send interval in ms [default=500]."
    },
    {
        SENSOR_ID,
        0,
        "s",
        "sensor-id",
        RaspaArg::Numeric,
        "\t\t-s <value>, --sensor-id=<value> \tSpecify sensor id used for sending a value [default=0]."
    },

    { 0, 0, 0, 0, 0, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Options struct
////////////////////////////////////////////////////////////////////////////////

struct RaspaOptions
{
    std::chrono::milliseconds send_interval = DEFAULT_SEND_INTERVAL;
    int sensor_id = DEFAULT_SENSOR_ID;
};

inline std::optional<RaspaOptions> parse_options(int argc, char* argv[])
{
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
        return std::nullopt;
    }
    if (cl_parser.nonOptionsCount() > 0)
    {
        std::cerr << "Unexpected non-optional argument " << cl_parser.nonOption(0) << "\n";
        return std::nullopt;
    }
    if (cl_options[HELP])
    {
        option::printUsage(fwrite, stdout, usage);
        return std::nullopt;
    }

    RaspaOptions options;
    for (int i = 0; i < cl_parser.optionsCount(); i++)
    {
        option::Option& opt = cl_buffer[i];
        switch (opt.index())
        {
        case HELP:
        case UNKNOWN:
            break;

        case SEND_INTERVAL:
            {
                int parsed_int = atoi(opt.arg);
                if (parsed_int == 0)
                {
                    RaspaArg::print_error("Option '", opt, "' invalid number\n");
                    return std::nullopt;
                }
                options.send_interval = std::chrono::milliseconds(parsed_int);
            }
            break;

        case SENSOR_ID:
            {
                int parsed_int = atoi(opt.arg);
                options.sensor_id = parsed_int;
            }
            break;

        default:
            RaspaArg::print_error("Unhandled option '", opt, "' \n");
            break;
        }
    }

    return options;
}

#endif // RASPA_MOCKUP_OPTIONS_H
