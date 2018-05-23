/**
 * @brief Main class for remapping sensors data into output format
 * @copyright MIND Music Labs AB, Stockholm
 *
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

private:
    int _max_no_sensors;
    std::vector<std::unique_ptr<BaseSensorMapper>> _mappers;
};

}; // namespace mapping
}; // namespace sensei

#endif //SENSEI_MAPPING_PROCESSOR_H
