

#include <iostream>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

#include "raspa_frontend.h"
#include "xmos_gpio_protocol.h"
#include "logging.h"

namespace sensei {
namespace hw_frontend {


constexpr char SENSEI_SOCKET[] = "/tmp/sensei";
constexpr char RASPA_SOCKET[] = "/tmp/raspa";
constexpr int  SOCKET_TIMEOUT_US = 500'000;
constexpr auto READ_WRITE_TIMEOUT = std::chrono::seconds(1);
constexpr auto ACK_TIMEOUT = std::chrono::milliseconds(1000);
constexpr int  MAX_RESEND_ATTEMPTS = 3;

SENSEI_GET_LOGGER;

RaspaFrontend::RaspaFrontend(SynchronizedQueue <std::unique_ptr<sensei::Command>>*in_queue,
                             SynchronizedQueue <std::unique_ptr<sensei::BaseMessage>>*out_queue)
              : HwFrontend(in_queue, out_queue),
                //_message_tracker(ACK_TIMEOUT, MAX_RESEND_ATTEMPTS),
                _state(ThreadState::STOPPED),
                _in_socket(-1),
                _out_socket(-1),
                _connected(false),
                _ready_to_send{true},
                _muted(false),
                _verify_acks(true)
{
    _in_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    _out_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    _send_list.push_back(_packet_factory.make_reset_system_command());
    _send_list.push_back(_packet_factory.make_get_board_info_command());
    if (_in_socket >= 0 && _out_socket >= 0)
    {
        /* Sensei binds one socket to SENSEI_SOCKET, then tries to connect
         * the other one to RASPA_SOCKET, if this fails, Sensei will retry
         * the connection to RASPA_SOCKET when it receives something on
         * SENSEI_SOCKET.
         * Raspalib does the opposite when it starts up, binds to its own
         * port and waits for a message. This way the processes can be
         * started in any order and synchronise  */

        sockaddr_un address;
        address.sun_family = AF_UNIX;
        strcpy(address.sun_path, SENSEI_SOCKET);
        // In case sensei didn't quit gracefully, clear the socket handle
        unlink(SENSEI_SOCKET);
        auto res = bind(_in_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
        if (res != 0)
        {
            SENSEI_LOG_ERROR("Failed to bind to socket: {}", strerror(errno));
        }
        else
        {
            timeval time;
            time.tv_sec = 0;
            time.tv_usec = SOCKET_TIMEOUT_US;
            auto res = setsockopt(_in_socket, SOL_SOCKET, SO_RCVTIMEO, &time, sizeof(time));
            if (res != 0 )
            {
                SENSEI_LOG_ERROR("Failed to set incoming socket timeout: {}", strerror(errno));
            }
        }
        _connected = _connect_to_raspa();
        if (!_connected)
        {
            SENSEI_LOG_INFO("Could not connect to raspa");
        }
    }
    else
    {
        SENSEI_LOG_ERROR("Failed to get sockets from system");
    }
}

RaspaFrontend::~RaspaFrontend()
{
    unlink(SENSEI_SOCKET);
}

bool RaspaFrontend::connected()
{
    return _connected;
}

void RaspaFrontend::run()
{
    SENSEI_LOG_INFO("Starting read and write threads");
    if (_connected || _state.load() == ThreadState::STOPPED)
    {
        _state.store(ThreadState::RUNNING);
        _read_thread = std::thread(&RaspaFrontend::read_loop, this);
        _write_thread = std::thread(&RaspaFrontend::write_loop, this);
    }
    else
    {
        SENSEI_LOG_ERROR("Cant start RaspaFrontend, {}", _connected? "Already running" : "Not connected");
    }
}

void RaspaFrontend::stop()
{
    if (_state.load() == ThreadState::RUNNING)
    {
        return;
    }
    SENSEI_LOG_INFO("Stopping RaspaFrontend");
    _state.store(ThreadState::STOPPING);
    if (_read_thread.joinable())
    {
        _read_thread.join();
    }
    if (_write_thread.joinable())
    {
        _write_thread.join();
    }
    _state.store(ThreadState::STOPPED);
    SENSEI_LOG_INFO("Threads stopped");
}

void RaspaFrontend::mute(bool enabled)
{
    _muted = enabled;
}

void RaspaFrontend::verify_acks(bool enabled)
{
    _verify_acks = enabled;
}

void RaspaFrontend::read_loop()
{
    XmosGpioPacket buffer = {0};
    while (_state.load() == ThreadState::RUNNING)
    {
        memset(&buffer, 0, sizeof(buffer));
        auto bytes = recv(_in_socket, &buffer, sizeof(buffer), 0);
        if (_muted == false && bytes >= static_cast<ssize_t>(sizeof(XmosGpioPacket)))
        {
            if (!_connected)
            {
                _connected = _connect_to_raspa();
            }
            _handle_raspa_packet(buffer);
            SENSEI_LOG_INFO("Received from raspa: {} bytes", bytes);
        }
        if (!_ready_to_send)
        {
            // TODO - will wait forever if ack is lost - fix!
            //_handle_timeouts(); /* It's more efficient to not check this every time */
        }
        if (bytes < 0)
        {
            SENSEI_LOG_WARNING("Returned {} on recv", strerror(errno));
        }
    }
    /* Notify the write thread in case it is waiting since no more notifications will follow */
    _ready_to_send_notifier.notify_one();
}

void RaspaFrontend::write_loop()
{
    while (_state.load() == ThreadState::RUNNING)
    {
        _in_queue->wait_for_data(std::chrono::milliseconds(READ_WRITE_TIMEOUT));
        while(!_in_queue->empty())
        {
            std::unique_ptr<Command> message = _in_queue->pop();
            _process_sensei_command(message.get());
        }

        while(!_send_list.empty())
        {
            SENSEI_LOG_INFO("Going through sendlist: {} packets", _send_list.size());
            auto packet = _send_list.front();
            std::unique_lock<std::mutex> lock(_send_mutex);
            if (_verify_acks && !_ready_to_send )
            {
                SENSEI_LOG_INFO("Waiting for ack");
                _ready_to_send_notifier.wait(lock);
            }
            auto ret = send(_out_socket, &packet, sizeof(XmosGpioPacket), 0);
            if (_verify_acks && ret > 0)
            {
                SENSEI_LOG_INFO("Sent raspa packet, cmd {}, id {}", static_cast<int>(packet.command),
                                                                    static_cast<int>(from_xmos_byteord(packet.sequence_no)));
                _pending_sequence_number = from_xmos_byteord(packet.sequence_no);
                _ready_to_send = false;
            } else
            {
                _send_list.pop_front();
            }
            if (ret < static_cast<ssize_t>(sizeof(XmosGpioPacket)))
            {
                SENSEI_LOG_WARNING("Sending packet on socket failed {}", ret);
            }
        }
    }
}

//void RaspaFrontend::_handle_timeouts()
//{
//    std::lock_guard<std::mutex> lock(_send_mutex);
//    switch (_message_tracker.timed_out())
//    {
//        case timeout::TIMED_OUT_PERMANENTLY:
//        {
//            /* Resending timed out too many times, push an error message to main loop.
//             * NOTE: No break as we want to signal ready to send to the write thread too.
//             * Also note that m is destroyed when this scope exits. */
//            auto m = _message_tracker.get_cached_message();
//            auto error_message = _message_factory.make_too_many_timeouts_error(m->index(), 0);
//            SENSEI_LOG_WARNING("Message {} timed out, sending next message.", m->representation());
//            _out_queue->push(std::move(error_message));
//        }
//        case timeout::TIMED_OUT:
//        {
//            /* Resend logic is handled when next_message_to_send() is called. */
//            _ready_to_send = true;
//            _ready_to_send_notifier.notify_one();
//            break;
//        }
//        case timeout::NO_MESSAGE:
//        case timeout::WAITING:
//            /* Keep waiting */
//            break;
//    }
//}
//
//std::unique_ptr<Command> RaspaFrontend::_next_message_to_send()
//{
//    auto message = _message_tracker.get_cached_message();
//    if (message)
//    {
//        return std::move(message);
//    }
//    return _in_queue->pop();
//}

bool RaspaFrontend::_connect_to_raspa()
{
    sockaddr_un address;
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, RASPA_SOCKET);

    auto res = connect(_out_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
    if (res != 0)
    {
        SENSEI_LOG_ERROR("Failed to connect to Raspa socket: {}", strerror(errno));
        return false;
    }
    timeval time;
    time.tv_sec = 0;
    time.tv_usec = SOCKET_TIMEOUT_US;
    res = setsockopt(_out_socket, SOL_SOCKET, SO_SNDTIMEO, &time, sizeof(time));
    if (res != 0 )
    {
        SENSEI_LOG_ERROR("Failed to set outgoing socket timeout: {}", strerror(errno));
        return false;
    }
    SENSEI_LOG_INFO("Connected to Raspa!");
    return true;
}

void RaspaFrontend::_process_sensei_command(const Command*message)
{
    switch (message->type())
    {
        case CommandType::SET_SENSOR_HW_TYPE:
        {
            auto cmd = static_cast<const SetSensorHwTypeCommand*>(message);
            uint8_t hw_type;
            switch (cmd->data())
            {
                case SensorHwType::DIGITAL_INPUT_PIN:
                    hw_type = HwType::BINARY_INPUT;
                    break;

                case SensorHwType::DIGITAL_OUTPUT_PIN:
                    hw_type = HwType::BINARY_OUTPUT;
                    break;

                case SensorHwType::ANALOG_INPUT_PIN:
                    hw_type = HwType::ANALOG_INPUT;
                    break;

                case SensorHwType::BUTTON:
                    hw_type = HwType::BINARY_INPUT;
                    break;

                default:
                    SENSEI_LOG_WARNING("Unsupported Sensor HW type: {}", static_cast<int>(cmd->data()));
                    return;
            }
            _send_list.push_back(_packet_factory.make_add_controller_command(cmd->index(), hw_type));
            break;
        }
        case CommandType::SET_HW_PIN:
        {
            auto cmd = static_cast<const SetSingleHwPinCommand*>(message);
            Pinlist list;
            list.pincount = 1;
            list.pins[0] = cmd->data();
            _send_list.push_back(_packet_factory.make_add_pins_to_controller_command(cmd->index(), list));
            break;
        }
        case CommandType::SET_HW_PINS:
        {
            auto cmd = static_cast<const SetHwPinsCommand*>(message);
            Pinlist list;
            auto setpins = cmd->data();
            int i = 0;
            int i_mod = 0;
            /* Split into more than 1 xmos packet if neccesary */
            while (i < setpins.size())
            {
                list.pins[i_mod++] = static_cast<uint8_t>(setpins[i++]);
                if (i_mod >= sizeof(list.pins) || i >= setpins.size())
                {
                    list.pincount = static_cast<uint8_t>(i_mod);
                    _send_list.push_back(_packet_factory.make_add_pins_to_controller_command(cmd->index(), list));
                    i_mod = 0;
                }
            }
            break;
        }
        case CommandType::SET_ENABLED:
        {
            auto cmd = static_cast<const SetEnabledCommand*>(message);
            uint8_t muted = cmd->data()? CNTRLR_UNMUTED : CNTRLR_MUTED;
            _send_list.push_back(_packet_factory.make_mute_controller_command(cmd->index(), muted));
            break;
        }
        case CommandType::SET_SENDING_MODE:
        {
            auto cmd = static_cast<const SetSendingModeCommand*>(message);
            uint8_t mode;
            switch (cmd->data())
            {
                case SendingMode::OFF:
                    // TODO - Maybe send a mute command here
                    return;

                case SendingMode::CONTINUOUS:
                    mode = NotificationMode::EVERY_CNTRLR_TICK;
                    break;

                case SendingMode::ON_VALUE_CHANGED:
                    mode = NotificationMode::ON_VALUE_CHANGE;
                    break;

                case SendingMode::TOGGLED:
                case SendingMode::ON_PRESS:
                case SendingMode::ON_RELEASE:
                    // TODO - currently handling these in the mapper
                    mode = NotificationMode::ON_VALUE_CHANGE;
                    break;

                default:
                SENSEI_LOG_WARNING("Unsupported Sending Mode: {}", static_cast<int>(cmd->data()));
            }
            break;
        }
        case CommandType::SET_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<const SetSendingDeltaTicksCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_controller_tick_rate_command(cmd->index(), cmd->data()));
            break;
        }
        case CommandType::SEND_DIGITAL_PIN_VALUE:
        {
            auto cmd = static_cast<const SendDigitalPinValueCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_value_command(cmd->index(), cmd->data()? 1 : 0));
            break;
        }
        case CommandType::ENABLE_SENDING_PACKETS:
        {
            auto cmd = static_cast<const EnableSendingPacketsCommand*>(message);
        }

        default:
            SENSEI_LOG_WARNING("Unsupported command: {}", message->representation());
    }
    return;
}

void RaspaFrontend::_handle_raspa_packet(const XmosGpioPacket& packet)
{
    switch (packet.command)
    {
        case XMOS_CMD_GET_VALUE:
            _handle_value(packet);
            break;

        case XMOS_ACK:
            _handle_ack(packet);
            break;

        default:
            SENSEI_LOG_WARNING("Unhandled command type: {}", (int)packet.command);
    }
}

void RaspaFrontend::_handle_ack(const XmosGpioPacket& ack)
{
    uint32_t seq_no = from_xmos_byteord(ack.sequence_no);
    SENSEI_LOG_INFO("Got ack for packet: {}, {}", ack.command, seq_no);
    if (_verify_acks)
    {
        std::unique_lock<std::mutex> lock(_send_mutex);
        //if (_message_tracker.ack(ack.sequence_no))
        if (seq_no == _pending_sequence_number)
        {
            if (_send_list.size() > 0)
                _send_list.pop_front();
            _ready_to_send = true;
            _ready_to_send_notifier.notify_one();
        }
        else
        {
            SENSEI_LOG_WARNING("Got unrecognised ack for packet: {}, waiting for {}", seq_no, _pending_sequence_number);
        }
    }
    char status = ack.payload.raw_data[2]; // TODO - we need an ack in the union
    if (status != 0)
    {
        SENSEI_LOG_WARNING("Received bad ack from cmd {}, status: {}", ack.sequence_no, static_cast<int>(status));
    }
}

void RaspaFrontend::_handle_value(const XmosGpioPacket& packet)
{
    auto& m = packet.payload.value_send_data;
    _out_queue->push(_message_factory.make_analog_value(m.controller_id, from_xmos_byteord(m.controller_val), 0));
    SENSEI_LOG_INFO("Got a value packet!");
}

}
}

