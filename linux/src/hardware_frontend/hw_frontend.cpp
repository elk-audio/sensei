#include <iostream>
#include <cstring>

#include "hw_frontend.h"
#include "gpio_protocol/gpio_protocol.h"
#include "logging.h"

namespace sensei {
namespace hw_frontend {

using namespace gpio;

constexpr uint8_t   DEFAULT_TICK_RATE = GPIO_SYSTEM_TICK_1000_HZ;
constexpr auto      READ_WRITE_TIMEOUT = std::chrono::milliseconds(20);
constexpr auto      HW_BACKEND_CON_TIMEOUT = std::chrono::milliseconds(250);
constexpr auto      ACK_TIMEOUT = std::chrono::milliseconds(1000);
constexpr int       MAX_RESEND_ATTEMPTS = 3;

SENSEI_GET_LOGGER_WITH_MODULE_NAME("gpio_hw_frontend");

HwFrontend::HwFrontend(SynchronizedQueue <std::unique_ptr<sensei::Command>>*in_queue,
                       SynchronizedQueue <std::unique_ptr<sensei::BaseMessage>>*out_queue,
                       hw_backend::BaseHwBackend* hw_backend)
                : BaseHwFrontend(in_queue, out_queue),
                _message_tracker(ACK_TIMEOUT, MAX_RESEND_ATTEMPTS),
                _hw_backend(hw_backend),
                _state(ThreadState::STOPPED),
                _ready_to_send{true},
                _muted(false),
                _verify_acks(true)
{
    /* Prepare the setup and query hw commands to be the first to send */
    _send_list.push_back(_packet_factory.make_reset_system_command());
    _send_list.push_back(_packet_factory.make_get_board_info_command());
    _send_list.push_back(_packet_factory.make_set_tick_rate_command(DEFAULT_TICK_RATE));
}

void HwFrontend::run()
{
    SENSEI_LOG_INFO("Starting read and write threads");
    if (_state.load() == ThreadState::STOPPED)
    {
        _state.store(ThreadState::RUNNING);
        _read_thread = std::thread(&HwFrontend::read_loop, this);
        _write_thread = std::thread(&HwFrontend::write_loop, this);
    }
    else
    {
        SENSEI_LOG_WARNING("Hw frontend is already running");
    }
}

void HwFrontend::stop()
{
    if (_state.load() != ThreadState::RUNNING)
    {
        return;
    }
    SENSEI_LOG_INFO("Stopping HwFrontend");
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

void HwFrontend::mute(bool enabled)
{
    _muted = enabled;
}

void HwFrontend::verify_acks(bool enabled)
{
    _verify_acks = enabled;
}

void HwFrontend::read_loop()
{
    GpioPacket buffer;
    while (_state.load() == ThreadState::RUNNING)
    {
        while(_hw_backend->receive_gpio_packet(buffer))
        {
            if (!_muted)
            {
                _handle_gpio_packet(buffer);
            }

            if (!_ready_to_send)
            {
                _handle_timeouts(); /* It's more efficient to not check this every time */
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(READ_WRITE_TIMEOUT));
    }
    /* Notify the write thread in case it is waiting since no more notifications will follow */
    _ready_to_send_notifier.notify_one();
}

void HwFrontend::write_loop()
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

            /* attempt to send packets. It will retry until it is successful or
              when the thread state has changed.*/
            bool failed_to_send_packet = false;
            while(!_hw_backend->send_gpio_packet(packet) &&
                  _state.load() == ThreadState::RUNNING)
            {
                // prevent logging for each attempt.
                if(!failed_to_send_packet)
                {
                    SENSEI_LOG_WARNING("Failed sending packet to hw backend");
                    failed_to_send_packet = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(HW_BACKEND_CON_TIMEOUT));
            }

            /* If the loop above exits due to thread state change, then skip
             * the rest of the loop */
            if(_state.load() != ThreadState::RUNNING)
            {
                continue;
            }
            else if (_verify_acks)
            {
                SENSEI_LOG_DEBUG("Sent Gpio packet: {}, id: {}", gpio_packet_to_string(packet),
                                                                 static_cast<int>(from_gpio_protocol_byteord(packet.sequence_no)));
                _message_tracker.store(nullptr, from_gpio_protocol_byteord(packet.sequence_no));
                _ready_to_send = false;
            }
            else
            {
                _send_list.pop_front();
            }
        }
    }
}

void HwFrontend::_handle_timeouts()
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

void HwFrontend::_process_sensei_command(const Command*message)
{
    SENSEI_LOG_DEBUG("HwFrontend: got command: {}", message->representation());
    switch (message->type())
    {
        case CommandType::SET_SENSOR_HW_TYPE:
        {
            auto cmd = static_cast<const SetSensorHwTypeCommand*>(message);
            auto hw_type = to_gpio_hw_type(cmd->data());
            if (hw_type.has_value())
            {
                _send_list.push_back(_packet_factory.make_add_controller_command(cmd->index(), hw_type.value()));
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
            /* Split into more than 1 Gpio packet if neccesary */
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
            uint8_t muted = cmd->data()? GPIO_CONTROLLER_UNMUTED : GPIO_CONTROLLER_MUTED;
            _send_list.push_back(_packet_factory.make_mute_controller_command(cmd->index(), muted));
            break;
        }
        case CommandType::SET_SENDING_MODE:
        {
            auto cmd = static_cast<const SetSendingModeCommand*>(message);
            auto mode = to_gpio_sending_mode(cmd->data());
            if (mode.has_value())
            {
                _send_list.push_back(_packet_factory.make_set_notification_mode(cmd->index(), mode.value()));
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
        case CommandType::SET_ADC_FILTER_TIME_CONSTANT:
        {
            auto cmd = static_cast<const SetADCFitlerTimeConstantCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_analog_time_constant_command(cmd->index(), cmd->data()));
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
            uint8_t polarity = 0;
            switch (cmd->data())
            {
                case HwPolarity::ACTIVE_HIGH:
                    polarity = GPIO_ACTIVE_HIGH;
                    break;
                case HwPolarity::ACTIVE_LOW:
                    polarity = GPIO_ACTIVE_LOW;
                    break;
            }
            _send_list.push_back(_packet_factory.make_set_polarity_command(cmd->index(), polarity));
            break;
        }
        case CommandType::SET_FAST_MODE:
        {
            auto cmd = static_cast<const SetFastModeCommand*>(message);
            _send_list.push_back(_packet_factory.make_set_debounce_mode_command(cmd->index(),
                                                                                cmd->data()? GPIO_CONTROLLER_DEBOUNCE_ENABLED :
                                                                                             GPIO_CONTROLLER_DEBOUNCE_DISABLED));
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
                                                                        std::round(cmd->data())));
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
                _send_list.push_back(_packet_factory.make_reset_system_command());
            }
            break;
        }
        case CommandType::SET_INPUT_RANGE:
        {
            // TODO - maybe this should be reserved for encoders and led rings
            auto cmd = static_cast<const SetInputRangeCommand*>(message);
            auto range = cmd->data();
            _send_list.push_back(_packet_factory.make_set_range_command(cmd->index(),
                                                                        static_cast<uint32_t>(std::round(range.min)),
                                                                        static_cast<uint32_t>(std::round(range.max))));
            break;
        }

        default:
            SENSEI_LOG_WARNING("Unsupported command: {}", message->representation());
    }
    return;
}

void HwFrontend::_handle_gpio_packet(const GpioPacket& packet)
{
    SENSEI_LOG_DEBUG("Received a packet: {}", gpio_packet_to_string(packet));
    switch (packet.command)
    {
        case GPIO_CMD_GET_VALUE:
            _handle_value(packet);
            break;

        case GPIO_ACK:
            _handle_ack(packet);
            break;

        case GPIO_CMD_SYSTEM_CONTROL:
            if (packet.sub_command == GPIO_SUB_CMD_GET_BOARD_INFO)
            {
                _handle_board_info(packet);
            }
            break;

        default:
            SENSEI_LOG_WARNING("Unhandled command type: {}", (int)packet.command);
    }
}

void HwFrontend::_handle_ack(const GpioPacket& ack)
{
    uint32_t seq_no = from_gpio_protocol_byteord(ack.payload.gpio_ack_data.returned_seq_no);
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
    char status = ack.payload.gpio_ack_data.gpio_return_status;
    if (status != gpio::GpioReturnStatus::GPIO_OK)
    {
        SENSEI_LOG_WARNING("Received bad ack from cmd {}, status: {}", from_gpio_protocol_byteord(ack.payload.gpio_ack_data.returned_seq_no),
                                                                       gpio_status_to_string(status));
    }
}

void HwFrontend::_handle_value(const GpioPacket& packet)
{
    auto& m = packet.payload.gpio_value_data;
   _out_queue->push(_message_factory.make_analog_value(m.controller_id,
                                                       from_gpio_protocol_byteord(m.controller_val),
                                                       packet.timestamp));
    SENSEI_LOG_DEBUG("Got a value packet!");
}

void HwFrontend::_handle_board_info(const gpio::GpioPacket& packet)
{
    _board_info = packet.payload.gpio_board_info_data;
    SENSEI_LOG_INFO("Received board info: No of digital input pins: {}",_board_info.num_digital_input_pins);
    SENSEI_LOG_INFO("No of digital output pins: {}",_board_info.num_digital_output_pins);
    SENSEI_LOG_INFO("No of analog input pins: {}", _board_info.num_analog_input_pins);
    SENSEI_LOG_INFO("ADC bit resolution: {}", _board_info.adc_res_in_bits);
}

std::optional<uint8_t> to_gpio_hw_type(SensorHwType type)
{
    uint8_t hw_type;
    switch (type)
    {
        case SensorHwType::DIGITAL_INPUT_PIN:
            hw_type = GPIO_BINARY_INPUT;
            break;

        case SensorHwType::DIGITAL_OUTPUT_PIN:
            hw_type = GPIO_BINARY_OUTPUT;
            break;

        case SensorHwType::ANALOG_INPUT_PIN:
            hw_type = GPIO_ANALOG_INPUT;
            break;

        case SensorHwType::STEPPED_OUTPUT:
            hw_type = GPIO_STEPPED_OUTPUT;
            break;

        case SensorHwType::MULTIPLEXER:
            hw_type = GPIO_MUX_OUTPUT;
            break;

        case SensorHwType::N_WAY_SWITCH:
            hw_type = GPIO_N_WAY_SWITCH;
            break;

        case SensorHwType::ENCODER:
            hw_type = GPIO_ROTARY_ENCODER;
            break;

        case SensorHwType::BUTTON:
            hw_type = GPIO_BINARY_INPUT;
            break;

        default:
            SENSEI_LOG_WARNING("Unsupported Sensor HW type: {}", static_cast<int>(type));
            return std::nullopt;
    }
    return std::make_optional(hw_type);
}

std::optional<uint8_t> to_gpio_sending_mode(SendingMode mode)
{
    uint8_t gpio_mode;
    switch (mode)
    {
        case SendingMode::OFF:
            // TODO - Maybe send a mute command here
            return std::nullopt;

        case SendingMode::CONTINUOUS:
            gpio_mode = GPIO_EVERY_CONTROLLER_TICK;
            break;

        case SendingMode::ON_VALUE_CHANGED:
            gpio_mode = GPIO_ON_VALUE_CHANGE;
            break;

        case SendingMode::ON_PRESS:
        case SendingMode::ON_RELEASE:
            // TODO - currently handling these in the mapper
            gpio_mode = GPIO_ON_VALUE_CHANGE;
            break;

        default:
            SENSEI_LOG_WARNING("Unsupported Sending Mode: {}", static_cast<int>(mode));
            return std::nullopt;
    }
    return gpio_mode;
}
}
}
