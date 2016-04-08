#include "serial_command_creator.h"

#include <cstring>

namespace sensei {
namespace serial_frontend {

const int MAX_NUMBER_OFF_PINS = 64 + 16; // Perhaps make this configurable as a constructor argument


SerialCommandCreator::SerialCommandCreator() :
_cfg_cache(MAX_NUMBER_OFF_PINS)
{
    for (pin_config& i : _cfg_cache)
    {
        memset(&i, 0, sizeof(pin_config));
    }
}

SerialCommandCreator::~SerialCommandCreator()
{
}

/*
 * Settings for commands below are simple passed on to the teensy board
 */
const sSenseiDataPacket* SerialCommandCreator::make_set_digital_pin_cmd(int pin_id, uint32_t timestamp, bool value)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::SET_DIGITAL_PINS);
    _cmd_buffer.sub_cmd = SET_PIN;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    cmd->value = value;
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_set_bank_cmd(int pin_id, uint32_t timestamp, int value)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::SET_DIGITAL_PINS);
    _cmd_buffer.sub_cmd = SET_BANK;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    cmd->value = value;
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_set_sampling_rate_cmd(uint32_t timestamp, float sampling_rate)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::SET_SAMPLING_RATE);
    _cmd_buffer.sub_cmd = EMPTY;
    teensy_set_samplerate_cmd* cmd = reinterpret_cast<teensy_set_samplerate_cmd*>(&_cmd_buffer.payload);
    if (sampling_rate < 3.9)   // this is the lowest possible samplerate to set, lower values will be treated as 0
    {
        cmd->sample_rate_divisor = 0;
    }
    else if (sampling_rate > 1000)
    {
        cmd->sample_rate_divisor = 1;
    }
    else
    {
        cmd->sample_rate_divisor = static_cast<int>(1000 / sampling_rate);
    }
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_get_value_cmd(int pin_id, uint32_t timestamp)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::GET_VALUE);
    _cmd_buffer.sub_cmd = EMPTY;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    return &_cmd_buffer;
}


/*
 * Settings for commands below are cached for every pin.
 */
const sSenseiDataPacket* SerialCommandCreator::make_config_pintype_cmd(int pin_id, uint32_t timestamp, int pintype)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.pintype = pintype;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_sendingmode_cmd(int pin_id, uint32_t timestamp, int sendingmode)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.sendingMode = sendingmode;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_delta_ticks_cmd(int pin_id, uint32_t timestamp, int ticks)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.deltaTicksContinuousMode = ticks;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_bitres_cmd(int pin_id, uint32_t timestamp, int bits)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.ADCBitResolution = bits;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_filter_order_cmd(int pin_id, uint32_t timestamp, int order)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.filterOrder = order;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_lowpass_cutoff_cmd(int pin_id, uint32_t timestamp, float cutoff)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.lowPassCutOffFilter = cutoff;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_slidermode_cmd(int pin_id, uint32_t timestamp, int mode)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.sliderMode = mode;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_slider_threshold_cmd(int pin_id, uint32_t timestamp, int threshold)
{
    if (pin_id >= MAX_NUMBER_OFF_PINS)
    {
        return nullptr;
    }
    pin_config& cached_cfg = _cfg_cache[pin_id];
    cached_cfg.cfg_data.sliderThreshold = threshold;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

/*
 * Helper functions
 */
void initialize_common_data(sSenseiDataPacket& packet, uint32_t timestamp, uint8_t command)
{
    memset(&packet, 0, sizeof(sSenseiDataPacket));
    packet.start_header = START_SIGNATURE;
    packet.stop_header = STOP_SIGNATURE;
    packet.timestamp = timestamp;
    packet.cmd = command;
}

void fill_data(const pin_config& cached_cfg, sSenseiDataPacket& packet, uint32_t timestamp, uint8_t command)
{
    initialize_common_data(packet, timestamp, command);
    sPinConfiguration* pinconf = reinterpret_cast<sPinConfiguration*>(&packet.payload);
    *pinconf = cached_cfg.cfg_data;
    packet.sub_cmd = cached_cfg.pintype;
    packet.crc = calculate_crc(&packet);
}

} //end namespace serial_frontend
} //end namespace sensei