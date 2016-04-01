/**
 * @brief Class handling all serial communication with the Teensy board.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Serial frontend implementation
 */


#include <iostream>
#include <cstring>

#include "serial_frontend.h"
#include "serial_frontend_internal_definitions.h"

namespace sensei {
namespace serial_frontend
{

/*
 * Convenience function for comparing header signatures, same pattern as memcmp, strcmp
 */
int compare_packet_header(const PACKET_HEADER &lhv, const PACKET_HEADER &rhv)
{
    int diff = 0;
    for (unsigned int i = 0; i < sizeof(lhv.vByte); ++i)
    {
        diff += lhv.vByte[i] - rhv.vByte[i];
    }
    return diff;
}

/*
 * Calculate the checksum of a teensy packet
 */
uint16_t calculate_crc(const sSenseiDataPacket *packet)
{
    uint16_t sum = packet->cmd + packet->sub_cmd;
    for (unsigned int i = 0; i < SENSEI_PAYLOAD_LENGTH
                                 + sizeof(packet->continuation)
                                 + sizeof(packet->timestamp); ++i)
    {
        sum += *(packet->data + i);
    }
    return sum;
}

/*
 * Verify that a received message has not been corrupted
 */
bool verify_message(const sSenseiDataPacket *packet)
{
    if (compare_packet_header(packet->start_header, START_SIGNATURE) != 0 ||
        compare_packet_header(packet->stop_header, STOP_SIGNATURE) != 0)
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
 * Create teensy command packet from
 */
void create_send_command(uint8_t *buffer, std::unique_ptr<CommandMessage> message)
{
    memset(buffer, 0, SENSEI_LENGTH_DATA_PACKET);
    sSenseiDataPacket *command = reinterpret_cast<sSenseiDataPacket *>(buffer);
    //command->start_header =
    //command->end_header =
    if (!message->is_cmd())
    {
        return;
    };
    //set generic stuff, signatures, timestamp etc.
    int command_type = 1; // = message->command_type() or something
    switch (command_type)
    {
        case 1:
            break;
        default:
            break;
    }

    command->crc = calculate_crc(command);
}

/*
 * SerialFrontend member functions below:
 */

SerialFrontend::SerialFrontend(const std::string &port_name,
                               SynchronizedQueue<std::unique_ptr<CommandMessage>> *in_queue,
                               SynchronizedQueue<std::unique_ptr<BaseMessage>> *out_queue) :
        _in_queue(in_queue),
        _out_queue(out_queue),
        _read_thread_state(running_state::STOPPED),
        _write_thread_state(running_state::STOPPED)
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
        if (ret >= SENSEI_LENGTH_DATA_PACKET)
        {
            sSenseiDataPacket *packet = reinterpret_cast<sSenseiDataPacket *>(buffer);
            if (verify_message(packet) == false)
            {
                continue; // log an error message here when logging functionality is in place
            }
            std::unique_ptr<BaseMessage> m = create_internal_message(packet);
            if (m != nullptr)
            {
                _out_queue->push(std::move(m));
            }
            else
            {
                // failed to create BaseMessage, log error
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
    std::unique_ptr<CommandMessage> message;
    uint8_t buffer[SENSEI_LENGTH_DATA_PACKET];
    while (_write_thread_state == running_state::RUNNING)
    {
        _in_queue->wait_for_data(std::chrono::milliseconds(READ_WRITE_TIMEOUT_MS));
        if (_in_queue->empty())
        {
            continue;
        }
        message = _in_queue->pop();
        create_send_command(buffer, std::move(message));
        sp_nonblocking_write(_port, buffer, sizeof(buffer));
    }
    std::lock_guard<std::mutex> lock(_state_mutex);
    _write_thread_state = running_state::STOPPED;
}

/*
 * Create internal message representation from received teensy packet
 */
std::unique_ptr<BaseMessage> SerialFrontend::create_internal_message(const sSenseiDataPacket *packet)
{
    std::unique_ptr<BaseMessage> message(nullptr);
    switch (packet->cmd)
    {
        case SENSEI_CMD::GET_VALUE:  // for now, assume that incoming unsolicited responses will have any of these command codes
        case SENSEI_CMD::GET_ALL_VALUES:
        {
            const teensy_digital_value_msg *m = reinterpret_cast<const teensy_digital_value_msg *>(&packet->data);
            switch (m->pin_type)
            {
                case PIN_DIGITAL_INPUT:
                    message = _message_factory.make_digital_value(m->pin_id, m->value, packet->timestamp);
                    break;

                case PIN_ANALOG_INPUT:
                    const teensy_analog_value_msg* a = reinterpret_cast<const teensy_analog_value_msg *>(&packet->data);
                    message = _message_factory.make_analog_value(a->pin_id, a->value, packet->timestamp);
            }
            break;
        }
        case SENSEI_CMD::ACK:
            // handle acked messages
            break;
        default:
            break;
    }
    return message;
}
}; // end namespace sensei
}; // end namespace serial_frontend