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
 * @brief Main class for remapping sensors data into output format
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_MAPPING_PROCESSOR_H
#define SENSEI_MAPPING_PROCESSOR_H

#include <vector>

#include "sensor_mappers.h"
#include "output_backend/output_backend.h"

namespace sensei {
namespace mapping {

class MappingProcessor
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(MappingProcessor);

    MappingProcessor(int max_no_sensors = 64);

    CommandErrorCode apply_command(const Command *cmd);

    void put_config_commands_into(CommandIterator out_iterator);

    void process(Value* value, output_backend::OutputBackend* backend);

    std::unique_ptr<Command> process_set(Value* value);

private:
    int _max_no_sensors;
    std::vector<std::unique_ptr<BaseSensorMapper>> _mappers;
};

} // namespace mapping
} // namespace sensei

#endif //SENSEI_MAPPING_PROCESSOR_H