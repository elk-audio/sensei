

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

using namespace xmos;

constexpr char      SENSEI_SOCKET[] = "/tmp/sensei";
constexpr char      RASPA_SOCKET[] = "/tmp/raspa";
constexpr uint8_t   DEFAULT_TICK_RATE = SystemTickRate::TICK_5000_HZ;
constexpr int       SOCKET_TIMEOUT_US = 500'000;
constexpr auto      READ_WRITE_TIMEOUT = std::chrono::milliseconds(500);
constexpr auto      ACK_TIMEOUT = std::chrono::milliseconds(1000);
constexpr int       MAX_RESEND_ATTEMPTS = 3;

SENSEI_GET_LOGGER;

RaspaFrontend::RaspaFrontend(SynchronizedQueue <std::unique_ptr<sensei::Command>>*in_queue,
                             SynchronizedQueue <std::unique_ptr<sensei::BaseMessage>>*out_queue)
              : HwFrontend(in_queue, out_queue),
                _message_tracker(ACK_TIMEOUT, MAX_RESEND_ATTEMPTS),
                _state(ThreadState::STOPPED),
                _in_socket(-1),
                _out_socket(-1),
                _ready_to_send{true},
                _connected(false),
                _muted(false),
                _verify_acks(true)
{
    /* Prepare the setup and query hw commands to be the first to send */
    _send_list.push_back(_packet_factory.make_reset_system_command());
    _send_list.push_back(_packet_factory.make_get_board_info_command());
    _send_list.push_back(_packet_factory.make_set_tick_rate_command(DEFAULT_TICK_RATE));
    _in_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    _out_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
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
    if (_state.load() != ThreadState::RUNNING)
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
    XmosGpioPacket buffer;
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
        }
        if (!_ready_to_send)
        {
            _handle_timeouts(); /* It's more efficient to not check this every time */
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

        while(!_send_list.empty() && _state.load() == ThreadState::RUNNING)
        {
            std::unique_lock<std::mutex> lock(_send_mutex);
            SENSEI_LOG_DEBUG("Going through sendlist: {} packets", _send_list.size());
            if (_verify_acks && !_ready_to_send )
            {
                SENSEI_LOG_DEBUG("Waiting for ack");
                _ready_to_send_notifier.wait(lock);
                continue;
            }
            auto& packet = _send_list.front();
            auto ret = send(_out_socket, &packet, sizeof(XmosGpioPacket), 0);
            if (_verify_acks && ret > 0)
            {
                SENSEI_LOG_DEBUG("Sent raspa packet: {}, id: {}", xmos_packet_to_string(packet),
                                                                 static_cast<int>(from_xmos_byteord(packet.sequence_no)));
                _message_tracker.store(nullptr, from_xmos_byteord(packet.sequence_no));
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

void RaspaFrontend::_handle_timeouts()
{
    std::lock_guard<std::mutex> lock(_send_mutex);
    switch (_message_tracker.timed_out())
    {
        case timeout::TIMED_OUT_PERMANENTLY:
        {
            /* Resending timed out too many times, push an error message to main loop.
             * NOTE: Intentional fall through as we want to signal ready to send to the
             * write thread too. */
            SENSEI_LOG_WARNING("Message timed out too many times, sending next message.");
            if (_send_list.size() > 0)
            {
                _send_list.pop_front();
            }
            [[fallthrough]];
        }
        case timeout::TIMED_OUT:
        {
            SENSEI_LOG_WARNING("Message timed out, retrying.");
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
    SENSEI_LOG_DEBUG("Raspafrontend: got command: {}", message->representation());
    switch (message->type())
    {
        case CommandType::SET_SENSOR_HW_TYPE:
        {
            auto cmd = static_cast<const SetSensorHwTypeCommand*>(message);
            uint8_t hw_type;
            bool    valid;
            std::tie(hw_type, valid) = to_xmos_hw_type(cmd->data());
            if (valid)
            {
                _send_list.push_back(_packet_factory.make_add_controller_command(cmd->index(), hw_type));
            }
            break;
        }
        case CommandType::SET_HW_PINS:
        {
            auto cmd = static_cast<const SetHwPinsCommand*>(message);
            Pinlist list;
            auto setpins = cmd->data();
            unsigned int i = 0;
            unsigned int i_mod = 0;
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
            bool valid;
            std::tie(mode, valid) = to_xmos_sending_mode(cmd->data());
            if (valid)
            {
                _send_list.push_back(_packet_factory.make_set_notification_mode(cmd->index(), mode));
            }
            break;
        }
        case CommandType::SET_SENDING_DELTA_TICKS:
        {
            auto cmd = static_cast<const SetSendingDeltaTicksCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_controller_tick_rate_command(cmd->index(), cmd->data()));
            break;
        }
        case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            auto cmd = static_cast<const SetADCBitResolutionCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_analog_resolution_command(cmd->index(), cmd->data()));
            break;
        }
        case CommandType::SET_MULTIPLEXED:
        {
            auto cmd = static_cast<const SetMultiplexedSensorCommand*>(message);
            _send_list.push_back(_packet_factory.make_add_controller_to_mux_command(cmd->index(),
                                                                                    cmd->data().id,
                                                                                    cmd->data().pin));
            break;
        }
        case CommandType::SET_HW_POLARITY:
        {
            auto cmd = static_cast<const SetSensorHwPolarityCommand*>(message);
            uint8_t polarity;
            switch (cmd->data())
            {
                case HwPolarity::ACTIVE_HIGH:
                    polarity = CntrlrPolarity::ACTIVE_HIGH;
                    break;
                case HwPolarity::ACTIVE_LOW:
                    polarity = CntrlrPolarity::ACTIVE_LOW;
                    break;
            }
            _send_list.push_back(_packet_factory.make_set_polarity_command(cmd->index(), polarity));
            break;
        }
        case CommandType::SET_DIGITAL_OUTPUT_VALUE:
        {
            auto cmd = static_cast<const SetDigitalOutputValueCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_value_command(cmd->index(), cmd->data()? 1 : 0));
            break;
        }
        case CommandType::SET_CONTINUOUS_OUTPUT_VALUE:
        {
            auto cmd = static_cast<const SetContinuousOutputValueCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_value_command(cmd->index(),
                                                                        std::round(cmd->data() * _dac_output_max)));
            break;
        }
        case CommandType::SET_ANALOG_OUTPUT_VALUE:
        {
            auto cmd = static_cast<const SetRangeOutputValueCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_value_command(cmd->index(), cmd->data()));
            break;
        }
        case CommandType::ENABLE_SENDING_PACKETS:
        {
            auto cmd = static_cast<const EnableSendingPacketsCommand*>(message);
            if (cmd->data() == true)
            {
                _send_list.push_back(_packet_factory.make_start_system_command());
            }
            else
            {
                _send_list.push_back(_packet_factory.make_stop_system_command());
            }
            break;
        }

        default:
            SENSEI_LOG_WARNING("Unsupported command: {}", message->representation());
    }
    return;
}

void RaspaFrontend::_handle_raspa_packet(const XmosGpioPacket& packet)
{
    SENSEI_LOG_DEBUG("Received a packet: {}", xmos_packet_to_string(packet));
    switch (packet.command)
    {
        case XMOS_CMD_GET_VALUE:
            _handle_value(packet);
            break;

        case XMOS_ACK:
            _handle_ack(packet);
            break;

        case XMOS_CMD_CONFIGURE_CNTRLR:
            if (packet.sub_command == XMOS_SUB_CMD_GET_BOARD_INFO)
            _handle_board_info(packet);
            break;

        default:
            SENSEI_LOG_WARNING("Unhandled command type: {}", (int)packet.command);
    }
}

void RaspaFrontend::_handle_ack(const XmosGpioPacket& ack)
{
    uint32_t seq_no = from_xmos_byteord(ack.payload.ack_data.returned_seq_no);
    SENSEI_LOG_DEBUG("Got ack for packet: {}", seq_no);
    if (_verify_acks)
    {
        std::unique_lock<std::mutex> lock(_send_mutex);
        if (_message_tracker.ack(seq_no))
        {
            if (_send_list.size() > 0)
            {
                _send_list.pop_front();
            }
            _ready_to_send = true;
            _ready_to_send_notifier.notify_one();
        }
        else
        {
            SENSEI_LOG_WARNING("Got unrecognised ack for packet: {}", seq_no);
        }
    }
    char status = ack.payload.ack_data.status;
    if (status != xmos::XmosReturnStatus::OK)
    {
        SENSEI_LOG_WARNING("Received bad ack from cmd {}, status: {}", from_xmos_byteord(ack.payload.ack_data.returned_seq_no),
                                                                       xmos_status_to_string(status));
    }
}

void RaspaFrontend::_handle_value(const XmosGpioPacket& packet)
{
    auto& m = packet.payload.value_data;
    _out_queue->push(_message_factory.make_analog_value(m.controller_id, from_xmos_byteord(m.controller_val), 0));
    SENSEI_LOG_DEBUG("Got a value packet!");
}

void RaspaFrontend::_handle_board_info(const xmos::XmosGpioPacket& packet)
{
    _board_info = packet.payload.board_info_data;
    _dac_output_max = powf(2.0f, _board_info.adc_res_in_bits);
    SENSEI_LOG_INFO("Received board info: No of digital input pins: {}",_board_info.num_digital_input_pins);
    SENSEI_LOG_INFO("No of digital output pins: {}",_board_info.num_digital_output_pins);
    SENSEI_LOG_INFO("No of analog input pins: {}", _board_info.num_analog_input_pins);
    SENSEI_LOG_INFO("ADC bit resolution: {}", _board_info.adc_res_in_bits);
}

std::pair<uint8_t, bool> to_xmos_hw_type(SensorHwType type)
{
    uint8_t hw_type;
    switch (type)
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

        case SensorHwType::STEPPED_OUTPUT:
            hw_type = HwType::STEPPED_OUTPUT;
            break;

        case SensorHwType::MULTIPLEXER:
            hw_type = HwType::MUX_OUTPUT;
            break;

        case SensorHwType::N_WAY_SWITCH:
            hw_type = HwType::N_WAY_SWITCH;
            break;

        case SensorHwType::ENCODER:
            hw_type = HwType::ROTARY_ENCODER;
            break;

        case SensorHwType::BUTTON:
            hw_type = HwType::BINARY_INPUT;
            break;

        default:
            SENSEI_LOG_WARNING("Unsupported Sensor HW type: {}", static_cast<int>(type));
            return {0, false};
    }
    return {hw_type, true};
}

std::pair<uint8_t, bool> to_xmos_sending_mode(SendingMode mode)
{
    uint8_t xmos_mode;
    switch (mode)
    {
        case SendingMode::OFF:
            // TODO - Maybe send a mute command here
            return {0, false};

        case SendingMode::CONTINUOUS:
            xmos_mode = NotificationMode::EVERY_CNTRLR_TICK;
            break;

        case SendingMode::ON_VALUE_CHANGED:
            xmos_mode = NotificationMode::ON_VALUE_CHANGE;
            break;

        case SendingMode::TOGGLED:
        case SendingMode::ON_PRESS:
        case SendingMode::ON_RELEASE:
            // TODO - currently handling these in the mapper
            xmos_mode = NotificationMode::ON_VALUE_CHANGE;
            break;

        default:
            SENSEI_LOG_WARNING("Unsupported Sending Mode: {}", static_cast<int>(mode));
    }
    return {xmos_mode, true};
}
}
}

