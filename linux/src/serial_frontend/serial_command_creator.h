/**
 * @brief Creates serial commands from general set functions. Also caches values on
 * a per pin basis since all parameters have to be set at once.
 * @copyright MIND Music Labs AB, Stockholm
 *
 */

#ifndef SENSEI_SERIAL_COMMAND_CREATOR_H
#define SENSEI_SERIAL_COMMAND_CREATOR_H

#include <vector>

#include "../../common/sensei_serial_protocol.h"
#include "serial_frontend_internal.h"

namespace sensei {
namespace serial_frontend {

struct pin_config
{
    uint8_t           pintype;
    sPinConfiguration cfg_data;
} __attribute__((packed)) ;

class SerialCommandCreator {
public:
    SerialCommandCreator();
    ~SerialCommandCreator();

    const sSenseiDataPacket* make_set_digital_pin_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t value);
    const sSenseiDataPacket* make_set_bank_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t value);
    const sSenseiDataPacket* make_set_sampling_rate_cmd(uint32_t timestamp, float sampling_rate);
    const sSenseiDataPacket* make_get_value_cmd(uint16_t pin_id, uint32_t timestamp);
    const sSenseiDataPacket* make_config_pintype_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t pintype);
    const sSenseiDataPacket* make_config_sendingmode_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t sendingmode);
    const sSenseiDataPacket* make_config_delta_ticks_cmd(uint16_t pin_id, uint32_t timestamp, uint16_t ticks);
    const sSenseiDataPacket* make_config_bitres_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t bits);
    const sSenseiDataPacket* make_config_filter_order_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t order);
    const sSenseiDataPacket* make_config_lowpass_cutoff_cmd(uint16_t pin_id, uint32_t timestamp, float cutoff);
    const sSenseiDataPacket* make_config_slidermode_cmd(uint16_t pin_id, uint32_t timestamp, uint8_t mode);
    const sSenseiDataPacket* make_config_slider_threshold_cmd(uint16_t pin_id, uint32_t timestamp, uint16_t threshold);

private:
    sSenseiDataPacket _cmd_buffer;
    std::vector<pin_config> _cfg_cache;
};

void initialize_common_data(sSenseiDataPacket& packet, uint32_t timestamp, uint8_t command);
void fill_data(const pin_config& cached_cfg, sSenseiDataPacket& packet, uint32_t timestamp, uint8_t command);

} // end namespace serial_receiver
} // end namespace sensei


#endif //SENSEI_SERIAL_COMMAND_CREATOR_H
