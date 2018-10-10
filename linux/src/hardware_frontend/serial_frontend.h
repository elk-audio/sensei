/**
 * @brief Class handling all serial communication with the Teensy board.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * All communication is handled
 *
 */

#ifndef SENSEI_SERIAL_RECEIVER_H
#define SENSEI_SERIAL_RECEIVER_H

#include <functional>
#include <string>
#include <thread>
#include <condition_variable>
#include <mutex>

#include "libserialport.h"
#include "synchronized_queue.h"
#include "hw_frontend.h"
#include "message/message_factory.h"
#include "serial_protocol/sensei_serial_protocol.h"
#include "serial_command_creator.h"
#include "message_tracker.h"



namespace sensei {
namespace hw_frontend {

enum class running_state {
    RUNNING,
    STOPPING,
    STOPPED,
};
constexpr size_t MAX_SENSORS = 256;
constexpr size_t MAX_SERIAL_PINS = 64;

class SerialFrontend : public HwFrontend
{
public:
    /**
    * @brief Class constructor
    *
    * @param [in] port_name Name of the /dev serial port to use for communication
    * @param [in] in_queue Output queue where decoded serial messages go
    * @param [in] out_queue Queue for messages to be sent to the teensy board
    */
    SerialFrontend(const std::string& port_name,
                   SynchronizedQueue<std::unique_ptr<Command>>* in_queue,
                   SynchronizedQueue<std::unique_ptr<BaseMessage>>* out_queue);

    ~SerialFrontend();

    /**
    * @brief Returns true if connection to the serial port is ok and ready to send
    * @return State of serial port connection
    */
    bool connected() override;

    /**
    * @brief Spawn new threads for reading continuously from the port and in_queue
    */
    void run() override ;

    /**
    * @brief Stops the read and write threads if they are running
    */
    void stop() override;

    /**
     * @brief Stops the flow of messages. If set to true, incoming serial packets
     * are silently dropped.
     *
     * @param [in] enabled Sets mute enabled/disabled
     */
    void mute(bool enabled) override;

    /**
     * @brief Enables tracking and verification of serial ack packets
     *
     * @param [in] enabled Sets ack verification enabled/disabled
     */
    void verify_acks(bool enabled) override ;

private:
    int setup_port(const std::string& name);
    void change_state(running_state state);
    void read_loop();
    void write_loop();

    void process_serial_packet(const sSenseiDataPacket *packet);
    void process_value(const sSenseiDataPacket *packet);
    void process_imu_data(const sSenseiDataPacket *packet);
    void process_ack(const sSenseiDataPacket *packet);
    void send_initialize_packet(int ticks, int pins, int digital_pins, uint32_t timestamp);
    std::unique_ptr<Command> next_message_to_send();
    void handle_timeouts();

    const sSenseiDataPacket* handle_command(Command* message);

    MessageFactory       _message_factory;
    SerialCommandCreator _packet_factory;
    MessageTracker       _message_tracker;
    MessageConcatenator  _message_concatenator;

    sp_port *_port;

    running_state   _read_thread_state;
    running_state   _write_thread_state;
    std::thread     _read_thread;
    std::thread     _write_thread;
    std::mutex      _state_mutex;

    std::mutex      _send_mutex;
    std::condition_variable _ready_to_send_notifier;

    bool _ready_to_send;
    bool _connected;
    bool _muted;
    bool _verify_acks;

    std::array<int, ImuIndex::N_IMU_INDEXES> _imu_sensor_index;
    std::array<int, MAX_SENSORS> _id_to_pin_table;
    std::array<int, MAX_SERIAL_PINS> _pin_to_id_table;
};

}; // end namespace sensei
}; // end namespace SerialReceiver  serial_receiver
#endif //SENSEI_SERIAL_RECEIVER_H
