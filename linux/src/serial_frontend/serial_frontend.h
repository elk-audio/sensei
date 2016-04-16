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

#include "message/message_factory.h"
#include "../../common/sensei_serial_protocol.h"
#include "serial_command_creator.h"
#include "message_tracker.h"

#include "libserialport.h"
#include "synchronized_queue.h"

namespace sensei {
namespace serial_frontend {

enum class running_state {
    RUNNING,
    STOPPING,
    STOPPED,
};


class SerialFrontend
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
    bool connected();

    /**
    * @brief Spawn new threads for reading continuously from the port and in_queue
    */
    void run();

    /**
    * @brief Stops the read and write threads if they are running
    */
    void stop();

    /**
     * @brief Sends a SYSTEM_INITIALIZE command to the teensy board
     * Note, this function is only temprary. PLease call before calling run()
     * as it is not threadsafe.
     */
    void initialize(int ticks, int pins, int digital_pins, uint32_t timestamp);

    /**
     * @brief Stops the flow of messages. If set to true, incoming serial packets
     * are silently dropped.
     *
     * @param [in] enabled Sets mute enabled/disabled
     */
    void mute(bool enabled);

    /**
     * @brief Enables tracking and verification of serial ack packets
     *
     * @param [in] enabled Sets ack verification enabled/disabled
     */
    void verify_acks(bool enabled);

private:
    int setup_port(const std::string& name);
    void change_state(running_state state);
    void read_loop();
    void write_loop();

    std::unique_ptr<BaseMessage> process_serial_packet(const sSenseiDataPacket *packet);
    std::unique_ptr<BaseMessage> process_value(const sSenseiDataPacket *packet);
    std::unique_ptr<BaseMessage> process_ack(const sSenseiDataPacket *packet);


    const sSenseiDataPacket* create_send_command(Command* message);

    MessageFactory       _message_factory;
    SerialCommandCreator _packet_factory;
    MessageTracker       _message_tracker;

    sp_port *_port;
    SynchronizedQueue<std::unique_ptr<Command>>* _in_queue;
    SynchronizedQueue<std::unique_ptr<BaseMessage>>* _out_queue;

    running_state   _read_thread_state;
    running_state   _write_thread_state;
    std::thread     _read_thread;
    std::thread     _write_thread;
    std::mutex      _state_mutex;

    bool _connected;
    bool _muted;
    bool _verify_acks;
};

}; // end namespace sensei
}; // end namespace SerialReceiver  serial_receiver
#endif //SENSEI_SERIAL_RECEIVER_H
