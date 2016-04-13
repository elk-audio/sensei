/**
 * @brief Class handling all serial communication with the Teensy board.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Serial frontend implementation
 */


#include <iostream>
#include <cstring>
#include <cassert>

#include "serial_frontend.h"
#include "serial_frontend_internal.h"

namespace sensei {
namespace serial_frontend {

static const int MAX_NUMBER_OFF_PINS = 64 + 16;
static const auto ACK_TIMEOUT = std::chrono::milliseconds(1000);

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
        _packet_factory(MAX_NUMBER_OFF_PINS),
        _message_tracker(ACK_TIMEOUT),
        _in_queue(in_queue),
        _out_queue(out_queue),
        _read_thread_state(running_state::STOPPED),
        _write_thread_state(running_state::STOPPED),
        _connected(false),
        _muted(false),
        _verify_acks(true)
{
    setup_port(port_name);
}


/**
* @brief SerialReceiver destructor
*/
SerialFrontend::~SerialFrontend()
{
    stop();
    sp_free_port(_port);
}


bool SerialFrontend::connected()
{
    return _connected;
}


void SerialFrontend::run()
{
    if (_read_thread_state != running_state::RUNNING && _write_thread_state != running_state::RUNNING)
    {
        change_state(running_state::RUNNING);
        _read_thread = std::thread(&SerialFrontend::read_loop, this);
        _write_thread = std::thread(&SerialFrontend::write_loop, this);
    }
}


void SerialFrontend::stop()
{
    if (_read_thread_state != running_state::RUNNING || _write_thread_state != running_state::RUNNING)
    {
        return;
    }
    change_state(running_state::STOPPING);
    if (_read_thread.joinable())
    {
        _read_thread.join();
    }
    if (_write_thread.joinable())
    {
        _write_thread.join();
    }
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
    return SP_OK;
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
    uint8_t buffer[100];
    while (_read_thread_state == running_state::RUNNING)
    {
        memset(buffer, 0, sizeof(buffer));
        int ret = sp_blocking_read_next(_port, buffer, sizeof(buffer), READ_WRITE_TIMEOUT_MS);
        if (_muted == false && ret >= SENSEI_LENGTH_DATA_PACKET)
        {
            sSenseiDataPacket *packet = reinterpret_cast<sSenseiDataPacket *>(buffer);
            if (verify_message(packet) == false)
            {
                continue; // log an error message here when logging functionality is in place
            }
            std::unique_ptr<BaseMessage> m = process_serial_packet(packet);
            if (m != nullptr)
            {
                _out_queue->push(std::move(m));
            }
            else
            {
                // failed to create BaseMessage, log error
            }
            if (_verify_acks)
            {
                uint64_t id;
                while ((id = _message_tracker.timed_out()) > 0)
                {
                    // handle timed out message here
                }
            }
        }
    }
    std::lock_guard<std::mutex> lock(_state_mutex);
    _read_thread_state = running_state::STOPPED;
}

/*
 * Listening loop for in_queue
 */
void SerialFrontend::write_loop()
{
    std::unique_ptr<Command> message;
    while (_write_thread_state == running_state::RUNNING)
    {
        _in_queue->wait_for_data(std::chrono::milliseconds(READ_WRITE_TIMEOUT_MS));
        if (_in_queue->empty())
        {
            continue;
        }
        message = _in_queue->pop();
        const sSenseiDataPacket* packet = create_send_command(message.get());
        if (packet != nullptr)
        {
            sp_nonblocking_write(_port, packet, sizeof(sSenseiDataPacket));
            if (_verify_acks)
            {
                _message_tracker.store(message->uuid());
            }
        }
    }
    std::lock_guard<std::mutex> lock(_state_mutex);
    _write_thread_state = running_state::STOPPED;
}

/*
 * Create internal message representation from received teensy packet
 */
std::unique_ptr<BaseMessage> SerialFrontend::process_serial_packet(const sSenseiDataPacket *packet)
{
    switch (packet->cmd)
    {
        case SENSEI_CMD::GET_VALUE:  // for now, assume that incoming unsolicited responses will have any of these command codes
        case SENSEI_CMD::GET_ALL_VALUES:
        {
            const teensy_digital_value_msg *m = reinterpret_cast<const teensy_digital_value_msg *>(&packet->payload);
            switch (m->pin_type)
            {
                case PIN_DIGITAL_INPUT:
                    return _message_factory.make_digital_value(m->pin_id, m->value, packet->timestamp);
                    break;

                case PIN_ANALOG_INPUT:
                    const teensy_analog_value_msg* a = reinterpret_cast<const teensy_analog_value_msg *>(&packet->payload);
                    return _message_factory.make_analog_value(a->pin_id, a->value, packet->timestamp);
            }
            break;
        }
        case SENSEI_CMD::ACK:
        {
            const sSenseiACKPacket *ack = reinterpret_cast<const sSenseiACKPacket *>(packet->payload);
            uint64_t uuid = extract_uuid(ack);
            if (_verify_acks)
            {
                switch (_message_tracker.check_status(uuid))
                {
                    case ack_status::ACKED_OK:
                        // Everything's fine, do nothing more
                        break;
                    case ack_status::TIMED_OUT:
                        // Make error message and return it
                        break;
                    case ack_status::UNKNOWN_IDENTIFIER:
                        // Should not happen, log an error here
                        break;
                }
            }
            if (ack->status != SENSEI_ERROR_CODE::OK)
            {
                // return create_error_message(ack->status);
            }
            break;
        }
        default:
            break;
    }
    return nullptr;
}

/*
 * Create teensy command packet from a Command message.
 * Note that message goes out of scope and is destroyed when the function returns
 */
const sSenseiDataPacket* SerialFrontend::create_send_command(Command* message)
{
    assert(message->base_type() == MessageType::COMMAND);

    switch (message->type())
    {
        case CommandType::SET_SAMPLING_RATE:
        {
            auto cmd = static_cast<SetSamplingRateCommand *>(message);
            return _packet_factory.make_set_sampling_rate_cmd(cmd->timestamp(),
                                                              cmd->data());
        }
        case CommandType::SET_PIN_TYPE:
        {
            auto cmd = static_cast<SetPinTypeCommand *>(message);
            return _packet_factory.make_config_pintype_cmd(cmd->sensor_index(),
                                                           cmd->timestamp(),
                                                           static_cast<int>(cmd->data()));
        }
        case CommandType::SET_SENDING_MODE:
        {
            auto cmd = static_cast<SetSendingModeCommand *>(message);
            return _packet_factory.make_config_sendingmode_cmd(cmd->sensor_index(),
                                                               cmd->timestamp(),
                                                               static_cast<int>(cmd->data()));
        }
        case CommandType::SET_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<SetSendingDeltaTicksCommand *>(message);
            return _packet_factory.make_config_delta_ticks_cmd(cmd->sensor_index(),
                                                               cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            auto cmd = static_cast<SetADCBitResolutionCommand *>(message);
            return _packet_factory.make_config_bitres_cmd(cmd->sensor_index(),
                                                          cmd->timestamp(),
                                                          cmd->data());
        }
        case CommandType::SET_LOWPASS_FILTER_ORDER:
        {
            auto cmd = static_cast<SetLowpassFilterOrderCommand *>(message);
            return _packet_factory.make_config_filter_order_cmd(cmd->sensor_index(),
                                                                cmd->timestamp(),
                                                                cmd->data());
        }
        case CommandType::SET_LOWPASS_CUTOFF:
        {
            auto cmd = static_cast<SetLowpassCutoffCommand *>(message);
            return _packet_factory.make_config_lowpass_cutoff_cmd(cmd->sensor_index(),
                                                                  cmd->timestamp(),
                                                                  cmd->data());
        }
        case CommandType::SET_SLIDER_THRESHOLD:
        {
            auto cmd = static_cast<SetSliderThresholdCommand *>(message);
            return _packet_factory.make_config_slider_threshold_cmd(cmd->sensor_index(),
                                                                    cmd->timestamp(),
                                                                    cmd->data());
        }
        case CommandType::SEND_DIGITAL_PIN_VALUE:
        {
            auto cmd = static_cast<SendDigitalPinValueCommand *>(message);
            return _packet_factory.make_set_digital_pin_cmd(cmd->sensor_index(),
                                                            cmd->timestamp(),
                                                            cmd->data());
        }
        default:
            return nullptr;
    }
}


}; // end namespace sensei
}; // end namespace serial_frontend
