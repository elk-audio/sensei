/**
 * @brief Main class for remapping sensors data into output format
 * @copyright MIND Music Labs AB, Stockholm
 *
 */
#ifndef SENSEI_MAPPING_PROCESSOR_H
#define SENSEI_MAPPING_PROCESSOR_H

#include <vector>

#include "sensor_mappers.h"

namespace sensei {
namespace mapping {

class MappingProcessor
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(MappingProcessor);

    MappingProcessor(const int max_n_sensors = 64);

    CommandErrorCode apply_command(const Command *cmd);

    void put_config_commands_into(CommandIterator out_iterator);

    void process(Value* value, OutputValueIterator out_iterator);

private:
    int _max_n_sensors;
    std::vector<std::unique_ptr<BaseSensorMapper>> _mappers;
};

}; // namespace mapping
}; // namespace sensei

#endif //SENSEI_MAPPING_PROCESSOR_H
