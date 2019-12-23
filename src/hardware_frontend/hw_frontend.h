/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Class handling communication using gpio protocol using the hardware
 *        backend. This acts as the gpio protocol master.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_HW_FRONTEND_H
#define SENSEI_HW_FRONTEND_H

#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <cassert>
#include <utility>
#include <optional>

#include "base_hw_frontend.h"
#include "hardware_backend/base_hw_backend.h"
#include "message_tracker.h"
#include "message/base_message.h"
#include "message/base_command.h"
#include "message/message_factory.h"
#include "gpio_command_creator.h"

namespace sensei {
namespace hw_frontend {

class HwFrontend : public BaseHwFrontend
{
public:
    /**
    * @brief Class constructor
    *
    * @param [in] in_queue Output queue where decoded messages go
    * @param [in] out_queue Queue for messages to be sent to the board
    */
    HwFrontend(SynchronizedQueue<std::unique_ptr<Command>>*in_queue,
               SynchronizedQueue<std::unique_ptr<BaseMessage>>*out_queue,
               hw_backend::BaseHwBackend* hw_backend);

    ~HwFrontend()
    {}

    /**
    * @brief Start the frontend
    */
    void run() override;

    /**
    * @brief Stops the frontend if it is running
    */
    void stop() override;

    /**
     * @brief Stops the flow of messages. If enabled, incoming packets are silently dropped.
     *
     * @param [in] enabled Sets mute enabled/disabled
     */
    void mute(bool enabled) override;

    /**
     * @brief Enables tracking and verification of packets ent
     *
     * @param [in] enabled Sets ack verification enabled/disabled
     */
    void verify_acks(bool enabled) override;

private:
    enum class ThreadState : int
    {
        RUNNING,
        STOPPING,
        STOPPED,
    };

    void read_loop();
    void write_loop();

    void _handle_timeouts();
    void _handle_gpio_packet(const gpio::GpioPacket& packet);
    void _handle_ack(const gpio::GpioPacket& ack);
    void _handle_value(const gpio::GpioPacket& packet);
    void _handle_board_info(const gpio::GpioPacket& packet);
    void _process_sensei_command(const Command*message);

    MessageFactory   _message_factory;
    GpioCommandCreator _packet_factory;
    MessageTracker     _message_tracker;
    std::deque<gpio::GpioPacket>  _send_list;
    hw_backend::BaseHwBackend* _hw_backend;

    std::atomic<ThreadState> _state;
    std::thread     _read_thread;
    std::thread     _write_thread;

    std::mutex      _send_mutex;
    std::condition_variable _ready_to_send_notifier;

    bool            _ready_to_send;
    bool            _muted;
    bool            _verify_acks;
    gpio::GpioBoardInfoData _board_info;
};

std::optional<uint8_t> to_gpio_hw_type(SensorHwType type);
std::optional<uint8_t> to_gpio_sending_mode(SendingMode mode);

} // namespace hw_frontend
} // namespace sensei

#endif //SENSEI_HW_FRONTEND_H