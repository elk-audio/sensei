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
 * @brief Contains the shiftreg hw backend for controlling shift register gpios.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * File contains definition for the shiftreg gpio hw backend, which runs the
 * GpioClient in a xenomai real time task to control the gpios.
 */

#ifndef SHIFTREG_GPIO_H_
#define SHIFTREG_GPIO_H_

#include "logging.h"

// Consider the following if WITH_GPIO_LOGIC is defined
#ifdef WITH_GPIO_LOGIC

#include <thread>
#include <pthread.h>

#include "fifo/circularfifo_memory_relaxed_aquire_release.h"

#include "hardware_backend/base_hw_backend.h"
#include "gpio_protocol/gpio_protocol.h"
#include "gpio_protocol_client/gpio_client.h"
#include "gpio_protocol_client/gpio_logger_interface.h"
#include "logging.h"
#include "boards/elk_pi_hat_defs.h"

namespace sensei {
namespace hw_backend {
namespace shiftregister_gpio {

using namespace memory_relaxed_aquire_release;

// Sizes of FIFO which sit between nrt and rt threads
constexpr int GPIO_PACKET_Q_SIZE = 150;
constexpr int GPIO_LOG_MSG_Q_SIZE = 50;

constexpr auto RECV_LOOP_SLEEP_PERIOD = std::chrono::milliseconds(5);

/**
 * @brief Enum to denote various stages of initialization of the real task.
 *        Used to denote what to cleanup when any of the stages fail and the
 *        order in which it is to be cleaned up.
 */
enum class ShiftregTaskState
{
    NOT_INITIALIZED,
    DEVICE_OPENED,
    PIN_DATA_MEM_ACQUIRED,
    LOGGER_THREAD_STARTED,
    RT_TASK_CREATED,
    RUNNING,
};

/**
 * @brief Shift register based gpio hwbackend class. Instantiation of this
 *        class will facilitate the creation and management of real time and non
 *        real time threads which provide an interface between sensei and the
 *        gpio client. This class also is responsible for interfacing with the
 *        real time driver running on the host to exchange gpio and adc data and
 *        processing them using the gpio client library.
 */
class ShiftregGpio : public BaseHwBackend
{
public:
    ShiftregGpio(std::chrono::milliseconds recv_packet_timeout) :
                BaseHwBackend(recv_packet_timeout),
                _is_logger_running(false),
                 _task_state(ShiftregTaskState::NOT_INITIALIZED),
                 _device_handle(0),
                _pin_data(nullptr)
    {
        _num_recv_retries = recv_packet_timeout/RECV_LOOP_SLEEP_PERIOD;
    }

    ~ShiftregGpio()
    {
        deinit();
    }

    /**
     * @brief Initializes all the threads and the gpio client. Function flow is:
     *        -> initializes xenomai
     *        -> Checks all driver params and make sure they match with those
     *           that have been when compiled.
     *        -> Opens the driver and acquires the memory where the pin data is
     *           stored from the driver
     *        -> Creates the non real time logger thread, which pipes log info
     *           from the gpio client lib running in real time context to
     *           the non rt sensei logger.
     *        -> If all the above steps are successful, then it instantiates
     *           the gpio client class and creates the rt xenoami thread.
     *        -> if any of the above stages fail, it is properly cleaned up
     *           using ShiftregTaskState which stores the initialization
     *           state.
     * @return True if successful, false if not.
     *
     */
    bool init() override;

    /**
     * @brief Interface function to stop all threads and delete any allocated
     *        resources in the order they were instantiated.
     */
    void deinit() override;

    /**
     * @brief Interface for sensei to send a gpio packet to the gpio client
     *        which runs in real time context. Sends the packed through
     *        the lock free fifo to the rt xenomai thread.
     *
     * @param tx_gpio_packet The gpio packed sent from sensei to the rt
     *                       gpio client.
     * @return true          The packet was successfully sent
     * @return false         The packet was unable to be sent.
     */
    bool send_gpio_packet(const gpio::GpioPacket &tx_gpio_packet) override;

    /**
     * @brief Interface for sensei to receive a gpio packet from the gpio client
     *        running in real time context. Receives the packet through the
     *        lock free fifo between the rt thread and the calling thread. This
     *        is a blocking call with a timeout
     *
     * @param rx_gpio_packet The packet to be received.
     * @return true          A packet was successfully received
     * @return false         No packet was received.
     */
    bool receive_gpio_packet(gpio::GpioPacket &rx_gpio_packet) override;

    /**
     * @brief The real time task which runs the gpio client and processes all
     *        the shiftregister gpio data. The function flow is:
     *        -> Get start time.
     *        -> Call io control on driver to get new input gpio data and adc
     *           data and send previous output gpio data.
     *        -> handles rx packets from received from sensei and calls the
     *           gpio client to process these packets.
     *        -> calls the gpio cient to process the the gpio data for this
     *           iteration
     *        -> Send any tx packets generated by the gpio client to sensei
     *        -> Send any log msgs generated by the gpio client to sensei
     *        -> get stop time
     *        -> sleep for the system tick period specified taking into account
     *           execution time
     */
    void rt_shiftreg_gpio_task();

    /**
     * @brief The non real time logger task. This receives log msgs from the
     *        real time task and pipes them to the SENSEI log depending on their
     *        log level.  In each iteration, it checks for log msgs from the
     *        lock free fifo sent by the rt thread.
     */
    void nrt_logger_task();

private:
    /**
     * @brief helper function to destroy objects and join threads. It performs
     *        the necessary actions depending on the current task state
     */
    void _cleanup();

    /**
     * @brief Opens the shiftregister driver device
     * @return true if driver opened successfully, false if otherwise
     */
    bool _init_driver();

    /**
     * Get the memory mapped location of pin data from the driver.
     * @return true if successful, false otherwise
     */
    bool _get_pin_data_mem_from_driver();

    /**
     * @brief initialize the rt xenomai thread. It sets a task priority to it
     *        and forces it to run on the last available core of the machine.
     *        After rt thread is created, the affinities of the nrt threads
     *        are restored back.
     * @return true on success, false otherwise.
     */
    bool _init_rt_task();

    /**
     * @brief Tells the driver to start the real time shiftreg and adc sampling
     * @return True on success, false otherwise.
     */
    bool _start_driver();

    /**
     * @brief Tells the driver to stop the real time shiftreg and adc sampling
     * @return True on success, false otherwise.
     */
    void _stop_driver();

    /* =======  Functions which run in RT context  ======= */

    /**
     * @brief Real time loop which runs before all configuration options have been
     * received from sensei
     */
    void _pre_config_rt_loop();

    /**
     * @brief Real time loop which runs after all configuration options have been
     * received from sensei
     */
    void _post_config_rt_loop();

    /**
     * @brief helper function called by the rt threads to rx packets from the
     *        hw frontend through the lock free fifo and call the gpio client
     *        to process them.
     */
    void _handle_rx_packets();

    /**
     * @brief helper function called by the rt thread to take new tx packets
     *        generated by the client during that iteration and send them to
     *        the hw frontend through the lock free fifo
     */
    void _handle_tx_packets();

    /**
     * @brief helper function called by the rt thread to get new log msgs
     *        generated by the client and pass them through to the non rt
     *        logger task, using the lock free fifo.
     */
    void _handle_log_msgs();

    bool _is_logger_running;

    pthread_t _processing_task;
    std::thread _logging_task;

    ShiftregTaskState _task_state;

    int _device_handle;

    CircularFifo<gpio::GpioPacket, GPIO_PACKET_Q_SIZE> _to_rt_thread_packet_fifo;
    CircularFifo<gpio::GpioPacket, GPIO_PACKET_Q_SIZE> _from_rt_thread_packet_fifo;
    CircularFifo<gpio::GpioLogMsg, GPIO_LOG_MSG_Q_SIZE> _from_rt_thread_log_msg_fifo;

    gpio::GpioClient<NUM_DIGITAL_INPUTS,
            NUM_DIGITAL_OUTPUTS,
            NUM_ANALOG_INPUTS,
            ADC_RES_IN_BITS> _gpio_client;

    uint32_t* _pin_data;

    int _num_recv_retries;
};

} // namespace shiftregister_gpio
} // namespace hw_backend
} // namespace sensei

// Dummy ShiftregGpio when WITH_GPIO_LOGIC is not defined
#else

namespace sensei {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("Shiftreg Gpio");

namespace hw_backend {
namespace shiftregister_gpio {

class ShiftregGpio : public BaseHwBackend
{
public:
    ShiftregGpio(std::chrono::milliseconds recv_packet_timeout) :
                                        BaseHwBackend(recv_packet_timeout)
    {}

    bool init() override
    {
        SENSEI_LOG_ERROR("Cannot Init Shiftregister Hw Backend. Its not enabled!");
        return false;
    }

    void deinit() override
    {}

    bool send_gpio_packet([[maybe_unused]] const gpio::GpioPacket& tx_gpio_packet) override
    {
        return false;
    }

    bool receive_gpio_packet([[maybe_unused]] gpio::GpioPacket& rx_gpio_packet) override
    {
        return false;
    }
};

} // namespace shiftregister_gpio
} // namespace hw_backend
} // namespace sensei

#endif // WITH_GPIO_LOGIC

#endif // SHIFTREG_GPIO_H_