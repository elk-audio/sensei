/**
 * @brief Class handling all serial communication with the Teensy board.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Serial frontend implementation
 */


#include <iostream>
#include <cstring>
#include <cassert>
#include <unistd.h>

#include "serial_frontend.h"
#include "serial_frontend_internal.h"

#include "logging.h"


namespace sensei {
namespace hw_frontend {

constexpr char SENSEI_DEFAULT_SERIAL_DEVICE[] = "/dev/ttyS01";

static const int MAX_NUMBER_OFF_INPUT_PINS = 64;
static const int MAX_NUMBER_OFF_OUTPUT_PINS = 32;
static const int INITIAL_TICK_DIVISOR = 1;
static const auto ACK_TIMEOUT = std::chrono::milliseconds(1000);
static const int MAX_RESEND_ATTEMPTS = 3;

SENSEI_GET_LOGGER;

/*
 * Verify that a received message has not been corrupted
 */
bool verify_message(const sSenseiDataPacket *packet)
{
    if (compare_packet_header(packet->start_header, START_HEADER) != 0 ||
        compare_packet_header(packet->stop_header, STOP_HEADER) != 0)
    {
        return false;
    }
    if (calculate_crc(packet) != packet->crc)
    {
        return false;
    }
    return true;
}

/*
 * SerialFrontend member functions below:
 */
SerialFrontend::SerialFrontend(const std::string &port_name,
                               SynchronizedQueue<std::unique_ptr<Command>> *in_queue,
                               SynchronizedQueue<std::unique_ptr<BaseMessage>> *out_queue) :
        HwFrontend(in_queue, out_queue),
        _packet_factory(MAX_NUMBER_OFF_INPUT_PINS + MAX_NUMBER_OFF_OUTPUT_PINS),
        _message_tracker(ACK_TIMEOUT, MAX_RESEND_ATTEMPTS),
        _read_thread_state(running_state::STOPPED),
        _write_thread_state(running_state::STOPPED),
        _ready_to_send(true),
        _connected(false),
        _muted(false),
        _verify_acks(true)
{
#ifndef DEBUG
    /* Set the debug handler to NULL, this removes useless calls to getenv
     * and speeds up the program   */
    sp_set_debug_handler(nullptr);
#endif
    for (int& port : _virtual_pin_table)
    {
        port = -1;
    }
    _id_to_pin_table.fill(0);
    _pin_to_id_table.fill(0);

    if (setup_port(port_name.empty()? SENSEI_DEFAULT_SERIAL_DEVICE : port_name) == SP_OK)
    {
        SENSEI_LOG_INFO("Serial port opened ok, sending initialize packet");
        send_initialize_packet(INITIAL_TICK_DIVISOR, MAX_NUMBER_OFF_INPUT_PINS, MAX_NUMBER_OFF_OUTPUT_PINS, 0);
        _connected = true;
    }
    else
    {
        SENSEI_LOG_ERROR("Failed to open serial port");
    }
}


/**
* @brief SerialReceiver destructor
*/
SerialFrontend::~SerialFrontend()
{
    _ready_to_send = true;
    stop();
    SENSEI_LOG_INFO("Closing serial port");
    sp_close(_port);
    sp_free_port(_port);
    SENSEI_LOG_INFO("Serial frontend de-initialized");
}


bool SerialFrontend::connected()
{
    return _connected;
}


void SerialFrontend::run()
{
    SENSEI_LOG_INFO("Starting read and write threads");
    if (_read_thread_state != running_state::RUNNING && _write_thread_state != running_state::RUNNING)
    {
        change_state(running_state::RUNNING);
        _read_thread = std::thread(&SerialFrontend::read_loop, this);
        _write_thread = std::thread(&SerialFrontend::write_loop, this);
    }
    else
    {
        SENSEI_LOG_WARNING("Read and write threads not stopped");
    }
}


void SerialFrontend::stop()
{
    if (_read_thread_state != running_state::RUNNING || _write_thread_state != running_state::RUNNING)
    {
        return;
    }
    SENSEI_LOG_INFO("Stopping read and write threads");
    change_state(running_state::STOPPING);
    if (_read_thread.joinable())
    {
        _read_thread.join();
    }
    if (_write_thread.joinable())
    {
        _write_thread.join();
    }
    SENSEI_LOG_INFO("Threads stopped");
}


void SerialFrontend::mute(bool enabled)
{
    _muted = enabled;
}

void SerialFrontend::verify_acks(bool enabled)
{
    _verify_acks = enabled;
}

int SerialFrontend::setup_port(const std::string &name)
{
    sp_return ret;
    ret = sp_get_port_by_name(name.c_str(), &_port);

    if (ret != SP_OK)
    {
        return ret;
    }
    ret = sp_open(_port, SP_MODE_READ_WRITE);
    if (ret != SP_OK)
    {
        return ret;
    }
    /* Turn of flow control, otherwise some byte patterns can be interpreted as
     * control codes and data can be lost */
    ret = sp_set_flowcontrol(_port, SP_FLOWCONTROL_NONE);
    if (ret != SP_OK)
    {
        return ret;
    }
    return sp_flush(_port, SP_BUF_BOTH);
}

void SerialFrontend::change_state(running_state state)
{
    std::lock_guard<std::mutex> lock(_state_mutex);
    _read_thread_state = state;
    _write_thread_state = state;
}

/*
 * Listening loop for the serial port
 */
void SerialFrontend::read_loop()
{
    uint8_t buffer[SENSEI_LENGTH_DATA_PACKET];
    while (_read_thread_state == running_state::RUNNING)
    {
        memset(buffer, 0, sizeof(buffer));
        int ret = sp_blocking_read_next(_port, buffer, sizeof(buffer), READ_WRITE_TIMEOUT_MS);
        if (_muted == false && ret >= SENSEI_LENGTH_DATA_PACKET)
        {
            sSenseiDataPacket *packet = reinterpret_cast<sSenseiDataPacket*>(buffer);
            if (verify_message(packet) == false)
            {
                SENSEI_LOG_WARNING("Serial packet failed verification");
            }
            process_serial_packet(packet);
        }
        if (!_ready_to_send)
        {
            handle_timeouts(); /* It's more efficient to not check this every time */
        }
        if (ret < 0)
        {
            SENSEI_LOG_WARNING("Serialport returned {} on read", ret);
            sleep(1);
        }
    }
    std::lock_guard<std::mutex> lock(_state_mutex);
    _read_thread_state = running_state::STOPPED;
    /* Notify the write thread in case it is waiting since no more notifications will follow */
    _ready_to_send_notifier.notify_one();
}

/*
 * Listening loop for in_queue
 */
void SerialFrontend::write_loop()
{
    while (_write_thread_state == running_state::RUNNING)
    {
        _in_queue->wait_for_data(std::chrono::milliseconds(READ_WRITE_TIMEOUT_MS));
        if (_in_queue->empty())
        {
            continue;
        }

        std::unique_lock<std::mutex> lock(_send_mutex);
        if (_verify_acks && !_ready_to_send )
        {
            _ready_to_send_notifier.wait(lock);
        }
        std::unique_ptr<Command> message = next_message_to_send();
        const sSenseiDataPacket* packet = handle_command(message.get());
        if (packet)
        {
            int ret = sp_nonblocking_write(_port, packet, sizeof(sSenseiDataPacket));
            if (_verify_acks && ret > 0)
            {
                uint64_t uuid = extract_uuid(packet);
                SENSEI_LOG_DEBUG("Sent serial packet {}", uuid);
                _message_tracker.store(std::move(message), uuid);
                _ready_to_send = false;
            }
            else if (ret < 0)
            {
                SENSEI_LOG_WARNING("Sending serial packed failed with code {}", ret);
            }
        }
        else
        {
            // Failed to create teensy packet TODO - Log error here
        }
    }
    std::lock_guard<std::mutex> lock(_state_mutex);
    _write_thread_state = running_state::STOPPED;
}

/*
 * Create internal message representation from received teensy packet
 */
void SerialFrontend::process_serial_packet(const sSenseiDataPacket *packet)
{
    switch (packet->cmd)
    {
        case SENSEI_CMD::VALUE:
            process_value(packet);
            break;

        case SENSEI_CMD::VALUE_IMU:
            process_imu_data(packet);
            break;

        case SENSEI_CMD::ACK:
            process_ack(packet);
            break;

        default:
            SENSEI_LOG_WARNING("Unhandled command type: {}", packet->cmd);
    }
}

void SerialFrontend::process_value(const sSenseiDataPacket *packet)
{
    const teensy_value_msg *m = reinterpret_cast<const teensy_value_msg *>(&packet->payload);
    switch (m->pin_type)
    {
        case PIN_DIGITAL_INPUT:
            _out_queue->push(_message_factory.make_digital_value(m->pin_id, m->value, packet->timestamp));
            break;

        case PIN_ANALOG_INPUT:
            _out_queue->push(_message_factory.make_analog_value(m->pin_id, m->value, packet->timestamp));
            break;

        default:
            SENSEI_LOG_WARNING("Unhandled pin type: {} from pin {}", m->pin_type, m->pin_id);
    }

}

void SerialFrontend::process_imu_data(const sSenseiDataPacket *packet)
{
    const char* assembled_payload = _message_concatenator.add(packet);
    if (!assembled_payload)
    {
        return;
    }

    if (packet->sub_cmd & IMU_GET_QUATERNIONS)
    {
        const sImuQuaternion *data = reinterpret_cast<const sImuQuaternion*>(assembled_payload);
        EulerAngles angles = quat_to_euler(data->qw, data->qx, data->qy, data->qz);
        if (_virtual_pin_table[ImuIndex::YAW] >= 0)
        {
            auto msg = _message_factory.make_imu_value(_virtual_pin_table[ImuIndex::YAW],
                                                       angles.yaw,
                                                       packet->timestamp);
            _out_queue->push(std::move(msg));
        }
        if (_virtual_pin_table[ImuIndex::PITCH] >= 0)
        {
            auto msg = _message_factory.make_imu_value(_virtual_pin_table[ImuIndex::PITCH],
                                                       angles.pitch,
                                                       packet->timestamp);
            _out_queue->push(std::move(msg));
        }
        if (_virtual_pin_table[ImuIndex::ROLL] >= 0)
        {
            auto msg = _message_factory.make_imu_value(_virtual_pin_table[ImuIndex::ROLL],
                                                       angles.roll,
                                                       packet->timestamp);
            _out_queue->push(std::move(msg));
        }
    }
    else
    {
        SENSEI_LOG_WARNING("Unsupported IMU data mode {}", packet->sub_cmd);
    }
}

void SerialFrontend::process_ack(const sSenseiDataPacket *packet)
{
    const sSenseiACKPacket *ack = reinterpret_cast<const sSenseiACKPacket *>(packet->payload);
    uint64_t uuid = extract_uuid(ack);
    SENSEI_LOG_DEBUG("Got ack for packet: {}", uuid);
    if (_verify_acks)
    {
        std::unique_lock<std::mutex> lock(_send_mutex);
        if (_message_tracker.ack(uuid))
        {
            _ready_to_send = true;
            _ready_to_send_notifier.notify_one();
        }
    }
    if (ack->status != SENSEI_ERROR_CODE::OK)
    {
        SENSEI_LOG_WARNING("Received bad ack from cmd {}, status: {}, {}", ack->cmd, translate_teensy_status_code(ack->status), ack->status);
        switch (ack->status)
        {
            case SENSEI_ERROR_CODE::CRC_NOT_CORRECT:
                // TODO - count the number of crc errors and only report error when they are too many
                _out_queue->push(_message_factory.make_bad_crc_error(0, ack->timestamp));
        }
    }
}

/*
 * Sends a packet to initialize the board with number of pins and tick divisor
 */

void SerialFrontend::send_initialize_packet(int ticks, int pins, int digital_pins, uint32_t timestamp)
{
    const sSenseiDataPacket* packet = _packet_factory.make_initialize_system_cmd(timestamp, ticks, pins, digital_pins);
    sp_nonblocking_write(_port, packet, sizeof(sSenseiDataPacket));
}

/*
 * Return the next message to send to the teensy board. It is either the top message
 * from in_queue or a retry of a timed out message
 */
std::unique_ptr<Command> SerialFrontend::next_message_to_send()
{
    auto message = _message_tracker.get_cached_message();
    if (message)
    {
        return std::move(message);
    }
    return _in_queue->pop();
}

/*
 * Check if we should stop waiting for an ack and signal timeouts upwards
 */
void SerialFrontend::handle_timeouts()
{
    std::lock_guard<std::mutex> lock(_send_mutex);
    switch (_message_tracker.timed_out())
    {
        case timeout::TIMED_OUT_PERMANENTLY:
        {
            /* Resending timed out too many times, push an error message to main loop.
             * NOTE: No break as we want to signal ready to send to the write thread too.
             * Also note that m is destroyed when this scope exits. */
            auto m = _message_tracker.get_cached_message();
            auto error_message = _message_factory.make_too_many_timeouts_error(m->index(), 0);
            SENSEI_LOG_WARNING("Message {} timed out, sending next message.", m->representation());
            _out_queue->push(std::move(error_message));
        }
        case timeout::TIMED_OUT:
        {
            /* Resend logic is handled when next_message_to_send() is called. */
            _ready_to_send = true;
            _ready_to_send_notifier.notify_one();
            break;
        }
        case timeout::NO_MESSAGE:
        case timeout::WAITING:
            /* Keep waiting */
            break;
    }
}

/*
 * Create teensy command packet from a Command message.
 */
const sSenseiDataPacket* SerialFrontend::handle_command(Command* message)
{
    assert(message->base_type() == MessageType::COMMAND);
    SENSEI_LOG_DEBUG("Got command {} ", message->representation());
    switch (message->type())
    {
        case CommandType::SET_VIRTUAL_PIN:
        {
            /* This command is internal to the SerialFrontend and is not passed on the the Teensy */
            auto cmd = static_cast<SetVirtualPinCommand *>(message);
            _virtual_pin_table[cmd->data()] = cmd->index();
            return nullptr;
        }
        case CommandType::ENABLE_SENDING_PACKETS:
        {
            auto cmd = static_cast<EnableSendingPacketsCommand *>(message);
            return _packet_factory.make_enable_sending_packets_cmd(cmd->timestamp(),
                                                                   cmd->data());
        }
        case CommandType::SET_HW_PIN:
        {
            auto cmd = static_cast<SetSingleHwPinCommand*>(message);
            unsigned int pin_id = static_cast<unsigned int>(cmd->data());
            unsigned int sensor_id = static_cast<unsigned int>(cmd->index());
            if (pin_id >= _pin_to_id_table.size() || sensor_id >= _id_to_pin_table.size())
            {
                SENSEI_LOG_ERROR("Wrong pin or sensor id ({}, {}", pin_id, sensor_id);
            }
            _pin_to_id_table[pin_id] = sensor_id;
            _id_to_pin_table[sensor_id] = pin_id;
            return nullptr;
        }
        case CommandType::SET_SENSOR_HW_TYPE:
        {
            auto cmd = static_cast<SetSensorHwTypeCommand*>(message);
            return _packet_factory.make_config_pintype_cmd(_id_to_pin_table[cmd->index()],
                                                           cmd->timestamp(),
                                                           cmd->data());
        }
        case CommandType::SET_ENABLED:
        {
            /* If the sensor index is used for an imu axis then don't send anything to the teensy */
            for (int& i : _virtual_pin_table)
            {
                if (message->index() == i)
                    return nullptr;
            }
            auto cmd = static_cast<SetEnabledCommand *>(message);
            return _packet_factory.make_config_enabled_cmd(cmd->timestamp(), cmd->data());
        }
        case CommandType::SET_SENDING_MODE:
        {
            auto cmd = static_cast<SetSendingModeCommand *>(message);
            return _packet_factory.make_config_sendingmode_cmd(_id_to_pin_table[cmd->index()],
                                                               cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<SetSendingDeltaTicksCommand *>(message);
            return _packet_factory.make_config_delta_ticks_cmd(_id_to_pin_table[cmd->index()],
                                                               cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            auto cmd = static_cast<SetADCBitResolutionCommand *>(message);
            return _packet_factory.make_config_adc_bitres_cmd(_id_to_pin_table[cmd->index()],
                                                              cmd->timestamp(),
                                                              cmd->data());
        }
        case CommandType::SET_LOWPASS_FILTER_ORDER:
        {
            auto cmd = static_cast<SetLowpassFilterOrderCommand *>(message);
            return _packet_factory.make_config_filter_order_cmd(_id_to_pin_table[cmd->index()],
                                                                cmd->timestamp(),
                                                                cmd->data());
        }
        case CommandType::SET_LOWPASS_CUTOFF:
        {
            auto cmd = static_cast<SetLowpassCutoffCommand *>(message);
            return _packet_factory.make_config_lowpass_cutoff_cmd(_id_to_pin_table[cmd->index()],
                                                                  cmd->timestamp(),
                                                                  cmd->data());
        }
        case CommandType::SET_SLIDER_THRESHOLD:
        {
            auto cmd = static_cast<SetSliderThresholdCommand *>(message);
            return _packet_factory.make_config_slider_threshold_cmd(_id_to_pin_table[cmd->index()],
                                                                    cmd->timestamp(),
                                                                    cmd->data());
        }
        case CommandType::SET_DIGITAL_OUTPUT_VALUE:
        {
            auto cmd = static_cast<SetDigitalOutputValueCommand *>(message);
            return _packet_factory.make_set_digital_pin_cmd(_id_to_pin_table[cmd->index()],
                                                            cmd->timestamp(),
                                                            cmd->data());
        }
        case CommandType::SET_IMU_ENABLED:
        {
            auto cmd = static_cast<SetImuEnabledCommand*>(message);
            return _packet_factory.make_imu_enable_cmd(cmd->timestamp(),
                                                       cmd->data());
        }
        case CommandType::SET_IMU_FILTER_MODE:
        {
            auto cmd = static_cast<SetImuFilterModeCommand*>(message);
            return _packet_factory.make_imu_set_filtermode_cmd(cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_IMU_ACC_RANGE_MAX:
        {
            auto cmd = static_cast<SetImuAccelerometerRangeMaxCommand*>(message);
            return _packet_factory.make_imu_set_accelerometer_range_cmd(cmd->timestamp(),
                                                                        cmd->data());
        }
        case CommandType::SET_IMU_GYRO_RANGE_MAX:
        {
            auto cmd = static_cast<SetImuGyroscopeRangeMaxCommand*>(message);
            return _packet_factory.make_imu_set_gyroscope_range_cmd(cmd->timestamp(),
                                                                    cmd->data());
        }
        case CommandType::SET_IMU_COMPASS_RANGE_MAX:
        {
            auto cmd = static_cast<SetImuCompassRangeMaxCommand*>(message);
            return _packet_factory.make_imu_set_compass_range_cmd(cmd->timestamp(),
                                                                  cmd->data());
        }
        case CommandType::SET_IMU_COMPASS_ENABLED:
        {
            auto cmd = static_cast<SetImuCompassEnabledCommand*>(message);
            return _packet_factory.make_imu_set_compass_enable_cmd(cmd->timestamp(),
                                                                   cmd->data());
        }
        case CommandType::SET_IMU_SENDING_MODE:
        {
            auto cmd = static_cast<SetImuSendingModeCommand*>(message);
            return _packet_factory.make_imu_set_sending_mode_cmd(cmd->timestamp(),
                                                                 cmd->data());
        }
        case CommandType::SET_IMU_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<SetImuSendingDeltaTicksCommand*>(message);
            return _packet_factory.make_imu_set_delta_tics_cmd(cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_IMU_DATA_MODE:
        {
            auto cmd = static_cast<SetImuDataModeCommand*>(message);
            return _packet_factory.make_imu_set_datamode_cmd(cmd->timestamp(),
                                                             cmd->data());
        }
        case CommandType::SET_IMU_ACC_THRESHOLD:
        {
            auto cmd = static_cast<SetImuAccThresholdCommand*>(message);
            return _packet_factory.make_imu_set_acc_threshold_cmd(cmd->timestamp(),
                                                                  cmd->data());
        }
        case CommandType::IMU_CALIBRATE:
        {
            auto cmd = static_cast<ImuCalibrateCommand*>(message);
            return _packet_factory.make_calibrate_gyro_cmd(cmd->timestamp());
        }
        case CommandType::IMU_FACTORY_RESET:
        {
            auto cmd = static_cast<ImuFactoryResetCommand*>(message);
            return _packet_factory.make_imu_factory_reset_cmd(cmd->timestamp());
        }
        case CommandType::IMU_REBOOT:
        {
            auto cmd = static_cast<ImuRebootCommand*>(message);
            return _packet_factory.make_imu_reboot_cmd(cmd->timestamp());
        }
        case CommandType::IMU_COMMIT_SETTINGS:
        {
            auto cmd = static_cast<ImuRebootCommand*>(message);
            return _packet_factory.make_imu_reboot_cmd(cmd->timestamp());
        }
        default:
            SENSEI_LOG_WARNING("Unsupported command type {}", static_cast<int>(message->base_type()));
            return nullptr;
    }
}


}; // end namespace sensei
}; // end namespace hw_frontend
