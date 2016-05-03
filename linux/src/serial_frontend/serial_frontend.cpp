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

#include "logging.h"


namespace sensei {
namespace serial_frontend {

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
        _packet_factory(MAX_NUMBER_OFF_INPUT_PINS + MAX_NUMBER_OFF_OUTPUT_PINS),
        _message_tracker(ACK_TIMEOUT, MAX_RESEND_ATTEMPTS),
        _in_queue(in_queue),
        _out_queue(out_queue),
        _read_thread_state(running_state::STOPPED),
        _write_thread_state(running_state::STOPPED),
        _ready_to_send(true),
        _connected(false),
        _muted(false),
        _verify_acks(true)
{
#ifndef NDEBUG
    /* Set the debug handler to NULL, this removes useless calls to getenv
     * and speeds up the program   */
    sp_set_debug_handler(nullptr);
#endif
    if (setup_port(port_name) == SP_OK)
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
    sp_close(_port);
    sp_free_port(_port);
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
            std::unique_ptr<BaseMessage> m = process_serial_packet(packet);
            if (m)
            {
                _out_queue->push(std::move(m));
            }
        }
        if (!_ready_to_send)
        {
            handle_timeouts(); /* It's more efficient to not check this every time */
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
        const sSenseiDataPacket* packet = create_send_command(message.get());
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
std::unique_ptr<BaseMessage> SerialFrontend::process_serial_packet(const sSenseiDataPacket *packet)
{
    switch (packet->cmd)
    {
        case SENSEI_CMD::VALUE:
        {
            const teensy_value_msg *m = reinterpret_cast<const teensy_value_msg *>(&packet->payload);
            switch (m->pin_type)
            {
                case PIN_DIGITAL_INPUT:
                    return _message_factory.make_digital_value(m->pin_id, m->value, packet->timestamp);

                case PIN_ANALOG_INPUT:
                    return _message_factory.make_analog_value(m->pin_id, m->value, packet->timestamp);

                default:
                    SENSEI_LOG_WARNING("Unhandled pin type: {}", packet->cmd);
                    return nullptr;
            }
        }
        case SENSEI_CMD::ACK:
        {
            return process_ack(packet);
        }
        default:
            SENSEI_LOG_WARNING("Unhandled command type: {}", packet->cmd);
            return nullptr;
    }
}


std::unique_ptr<BaseMessage> SerialFrontend::process_ack(const sSenseiDataPacket *packet)
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
        SENSEI_LOG_WARNING("Received bad ack, status: {}", translate_teensy_status_code(ack->status));
        switch (ack->status)
        {
            case SENSEI_ERROR_CODE::CRC_NOT_CORRECT:
                // TODO - count the number of crc errors and only report error when they are too many
                return _message_factory.make_bad_crc_error(0, ack->timestamp);
        }
    }
    return nullptr;
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
            /* Resending timed out too many times, signal push an error message to main loop.
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
const sSenseiDataPacket* SerialFrontend::create_send_command(Command* message)
{
    assert(message->base_type() == MessageType::COMMAND);
    switch (message->type())
    {
        case CommandType::ENABLE_SENDING_PACKETS:
        {
            auto cmd = static_cast<EnableSendingPacketsCommand *>(message);
            return _packet_factory.make_enable_sending_packets_cmd(cmd->timestamp(),
                                                                   cmd->data());
        }
        case CommandType::SET_SAMPLING_RATE:
        {
            auto cmd = static_cast<SetSamplingRateCommand *>(message);
            return _packet_factory.make_set_sampling_rate_cmd(cmd->timestamp(),
                                                              cmd->data());
        }
        case CommandType::SET_PIN_TYPE:
        {
            auto cmd = static_cast<SetPinTypeCommand *>(message);
            return _packet_factory.make_config_pintype_cmd(cmd->index(),
                                                           cmd->timestamp(),
                                                           cmd->data());
        }
        case CommandType::SET_ENABLED:
        {
            auto cmd = static_cast<SetEnabledCommand *>(message);
            return _packet_factory.make_config_enabled_cmd(cmd->timestamp(), cmd->data());
        }
        case CommandType::SET_SENDING_MODE:
        {
            auto cmd = static_cast<SetSendingModeCommand *>(message);
            return _packet_factory.make_config_sendingmode_cmd(cmd->index(),
                                                               cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<SetSendingDeltaTicksCommand *>(message);
            return _packet_factory.make_config_delta_ticks_cmd(cmd->index(),
                                                               cmd->timestamp(),
                                                               cmd->data());
        }
        case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            auto cmd = static_cast<SetADCBitResolutionCommand *>(message);
            return _packet_factory.make_config_adc_bitres_cmd(cmd->index(),
                                                              cmd->timestamp(),
                                                              cmd->data());
        }
        case CommandType::SET_LOWPASS_FILTER_ORDER:
        {
            auto cmd = static_cast<SetLowpassFilterOrderCommand *>(message);
            return _packet_factory.make_config_filter_order_cmd(cmd->index(),
                                                                cmd->timestamp(),
                                                                cmd->data());
        }
        case CommandType::SET_LOWPASS_CUTOFF:
        {
            auto cmd = static_cast<SetLowpassCutoffCommand *>(message);
            return _packet_factory.make_config_lowpass_cutoff_cmd(cmd->index(),
                                                                  cmd->timestamp(),
                                                                  cmd->data());
        }
        case CommandType::SET_SLIDER_MODE_ENABLED:
        {
            auto cmd = static_cast<SetSliderModeEnabledCommand *>(message);
            return _packet_factory.make_config_slidermode_cmd(cmd->index(),
                                                              cmd->timestamp(),
                                                              cmd->data());
        }
        case CommandType::SET_SLIDER_THRESHOLD:
        {
            auto cmd = static_cast<SetSliderThresholdCommand *>(message);
            return _packet_factory.make_config_slider_threshold_cmd(cmd->index(),
                                                                    cmd->timestamp(),
                                                                    cmd->data());
        }
        case CommandType::SEND_DIGITAL_PIN_VALUE:
        {
            auto cmd = static_cast<SendDigitalPinValueCommand *>(message);
            return _packet_factory.make_set_digital_pin_cmd(cmd->index(),
                                                            cmd->timestamp(),
                                                            cmd->data());
        }
        default:
            SENSEI_LOG_WARNING("Unsupported command type {}", static_cast<int>(message->base_type()));
            return nullptr;
    }
}


}; // end namespace sensei
}; // end namespace serial_frontend
