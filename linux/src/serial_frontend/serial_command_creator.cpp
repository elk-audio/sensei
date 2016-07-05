#include "serial_command_creator.h"

#include <cstring>

namespace sensei {
namespace serial_frontend {


SerialCommandCreator::SerialCommandCreator(int max_pins) :
        _max_pins(max_pins),
        _cached_cfgs(_max_pins),
        _cached_imu_cfgs({0, 0, 0, 0, 0, 0, 0})
{
    int id = 0;
    for (std::vector<pin_config>::iterator i = _cached_cfgs.begin(); i < _cached_cfgs.end(); ++i)
    {
        memset(i.base(), 0, sizeof(pin_config));
        i->cfg_data.idxPin = id++;
    }
    _cached_imu_cfgs.typeOfData = eImuGetData::IMU_GET_DATA_QUATERNION; // Hardcoding this datatype for now
}

SerialCommandCreator::~SerialCommandCreator()
{
}

/*
 * Settings for commands below are simple passed on to the teensy board
 */

const sSenseiDataPacket* SerialCommandCreator::make_initialize_system_cmd(uint32_t timestamp, int ticks_delay, int pins, int digital_pins)
{
    _max_pins = pins + digital_pins;
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::INITIALIZE_SYSTEM);
    sSystemInitialization* cmd = reinterpret_cast<sSystemInitialization*>(&_cmd_buffer.payload);
    cmd->ticksDelayRtTask = ticks_delay;
    cmd->nPins = pins;
    cmd->nDigitalPins = digital_pins;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_enable_sending_packets_cmd(uint32_t timestamp, bool enabled)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::ENABLE_SENDING_PACKETS);
    _cmd_buffer.sub_cmd = EMPTY;
    _cmd_buffer.payload[0] = enabled;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_set_digital_pin_cmd(int pin_id, uint32_t timestamp, bool value)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::SET_DIGITAL_PINS);
    _cmd_buffer.sub_cmd = SET_PIN;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    cmd->value = value;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_set_bank_cmd(int pin_id, uint32_t timestamp, int value)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::SET_DIGITAL_PINS);
    _cmd_buffer.sub_cmd = SET_BANK;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    cmd->value = value;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
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
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_get_value_cmd(int pin_id, uint32_t timestamp)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::GET_VALUE);
    _cmd_buffer.sub_cmd = EMPTY;
    teensy_set_value_cmd* cmd = reinterpret_cast<teensy_set_value_cmd*>(&_cmd_buffer.payload);
    cmd->pin_idx = pin_id;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_enabled_cmd(uint32_t timestamp, bool enabled)
{
    // TODO - implement this properly when there is support in the fw for this command
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::HELLO);
    _cmd_buffer.sub_cmd = EMPTY;
    _cmd_buffer.payload[0] = enabled;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}


/*
 * Settings for commands below are cached for every pin.
 */

const sSenseiDataPacket* SerialCommandCreator::make_config_pintype_cmd(int pin_id, uint32_t timestamp, PinType type)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    switch (type)
    {
        case PinType::DIGITAL_INPUT:
            cached_cfg.pintype = ePinType::PIN_DIGITAL_INPUT;
            break;
        case PinType::DIGITAL_OUTPUT:
            cached_cfg.pintype = ePinType::PIN_DIGITAL_OUTPUT;
            break;
        case PinType::ANALOG_INPUT:
            cached_cfg.pintype = ePinType::PIN_ANALOG_INPUT;
            break;
        default:
            cached_cfg.pintype = ePinType::PIN_DISABLE;
            break;
    }
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_sendingmode_cmd(int pin_id, uint32_t timestamp, SendingMode mode)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    switch (mode)
    {
        case SendingMode::CONTINUOUS:
            cached_cfg.cfg_data.sendingMode = eSendingMode::SENDING_MODE_CONTINUOUS;
            break;
        case SendingMode::ON_VALUE_CHANGED:
            cached_cfg.cfg_data.sendingMode = eSendingMode::SENDING_MODE_ON_VALUE_CHANGED;
            break;
        case SendingMode::ON_REQUEST:
            cached_cfg.cfg_data.sendingMode = eSendingMode::SENDING_MODE_ON_REQUEST;
            break;
        case SendingMode::OFF:
        case SendingMode::N_SENDING_MODES:
            cached_cfg.cfg_data.sendingMode = 0; // TODO - Add when the proper enum is in place
            break;
    }
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_delta_ticks_cmd(int pin_id, uint32_t timestamp, int ticks)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    cached_cfg.cfg_data.deltaTicksContinuousMode = ticks;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_adc_bitres_cmd(int pin_id, uint32_t timestamp, int bits)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    switch (bits)
    {
        case 8:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_8_BIT;
            break;
        case 9:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_9_BIT;
            break;
        case 10:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_10_BIT;
            break;
        case 11:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_11_BIT;
            break;
        case 12:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_12_BIT;
            break;
        default:
            cached_cfg.cfg_data.ADCBitResolution = ePinAdcBitResolution::PIN_ADC_RESOLUTION_8_BIT;
            break;
    }
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_filter_order_cmd(int pin_id, uint32_t timestamp, int order)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    cached_cfg.cfg_data.filterOrder = order;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_lowpass_cutoff_cmd(int pin_id, uint32_t timestamp, float cutoff)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    cached_cfg.cfg_data.lowPassCutOffFilter = cutoff;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_slidermode_cmd(int pin_id, uint32_t timestamp, int mode)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    cached_cfg.cfg_data.sliderMode = mode;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_config_slider_threshold_cmd(int pin_id, uint32_t timestamp, int threshold)
{
    if (pin_id >= _max_pins)
    {
        return nullptr;
    }
    pin_config &cached_cfg = _cached_cfgs[pin_id];
    cached_cfg.cfg_data.sliderThreshold = threshold;
    fill_data(cached_cfg, _cmd_buffer, timestamp, SENSEI_CMD::CONFIGURE_PIN);
    return &_cmd_buffer;
}

/*
 * IMU commands below
 */
const sSenseiDataPacket* SerialCommandCreator::make_imu_enable_cmd(uint32_t timestamp, bool enable)
{
    initialize_common_data(_cmd_buffer, timestamp, enable ? SENSEI_CMD::IMU_START : SENSEI_CMD::IMU_STOP);
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_filtermode_cmd(uint32_t timestamp, int mode)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    _cached_imu_cfgs.filterMode = mode;
    sImuSettings *settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_datamode_cmd(uint32_t timestamp, int mode)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    _cached_imu_cfgs.typeOfData = mode;
    sImuSettings *settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_accelerometer_range_cmd(uint32_t timestamp, int range)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    if (range <= 2)
    {
        _cached_imu_cfgs.accerelometerRange = eImuSensorAccelerometerRange::IMU_SENSOR_ACCELEROMETER_RANGE_2G;
    }
    else if (range <= 4)
    {
        _cached_imu_cfgs.accerelometerRange = eImuSensorAccelerometerRange::IMU_SENSOR_ACCELEROMETER_RANGE_4G;
    }
    else
    {
        _cached_imu_cfgs.accerelometerRange = eImuSensorAccelerometerRange::IMU_SENSOR_ACCELEROMETER_RANGE_8G;
    }
    sImuSettings* settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_gyroscope_range_cmd(uint32_t timestamp, int range)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    if (range <= 250)
    {
        _cached_imu_cfgs.gyroscopeRange = eImuSensorGyroscopeRange::IMU_SENSOR_GYROSCOPE_RANGE_250;
    }
    else if (range <= 500)
    {
        _cached_imu_cfgs.gyroscopeRange = eImuSensorGyroscopeRange::IMU_SENSOR_GYROSCOPE_RANGE_500;
    }
    else
    {
        _cached_imu_cfgs.gyroscopeRange = eImuSensorGyroscopeRange::IMU_SENSOR_GYROSCOPE_RANGE_2000;
    }
    sImuSettings* settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_compass_range_cmd(uint32_t timestamp, float range)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    if (range <= 0.88)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_0_88;
    }
    else if (range <= 1.3)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_1_30;
    }
    else if (range <= 1.9)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_1_90;
    }
    else if (range <= 2.5)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_2_50;
    }
    else if (range <= 4)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_4_00;
    }
    else if (range <= 5.6)
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_5_60;
    }
    else
    {
        _cached_imu_cfgs.compassRange = eImuSensorCompassRange::IMU_SENSOR_COMPASS_RANGE_8_10;
    }
    sImuSettings* settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_compass_enable_cmd(uint32_t timestamp, bool enabled)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    _cached_imu_cfgs.compassEnable = enabled;
    sImuSettings* settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

const sSenseiDataPacket* SerialCommandCreator::make_imu_set_delta_tics_cmd(uint32_t timestamp, int ticks_delay)
{
    initialize_common_data(_cmd_buffer, timestamp, SENSEI_CMD::IMU_SET_SETTINGS);
    _cached_imu_cfgs.deltaTicksContinuousMode = ticks_delay;
    sImuSettings* settings = reinterpret_cast<sImuSettings*>(_cmd_buffer.payload);
    *settings = _cached_imu_cfgs;
    _cmd_buffer.crc = calculate_crc(&_cmd_buffer);
    return &_cmd_buffer;
}

/*
 * Helper functions
 */
void initialize_common_data(sSenseiDataPacket& packet, uint32_t timestamp, uint8_t command)
{
    memset(&packet, 0, sizeof(sSenseiDataPacket));
    packet.start_header = START_HEADER;
    packet.stop_header = STOP_HEADER;
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