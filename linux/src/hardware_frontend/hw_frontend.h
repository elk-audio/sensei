/**
 * @brief Class handling communication using gpio protocol using the hardware backend
 * @copyright MIND Music Labs AB, Stockholm
 *
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
    bool            _hw_backend_connected;
    bool            _muted;
    bool            _verify_acks;
    gpio::GpioBoardInfoData _board_info;
};

std::optional<uint8_t> to_gpio_hw_type(SensorHwType type);
std::optional<uint8_t> to_gpio_sending_mode(SendingMode mode);

} // end namespace hw_frontend
} // end namespace sensei

#endif //SENSEI_HW_FRONTEND_H