/**
 * @brief Contains the Rpi shiftreg hw backend for controlling gpios on
 *        Elk Pi boards.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 *
 * File contains definition for the rpi shiftreg gpio hw backend, which runs the
 * GpioClient in a xenomai real time task to control the gpios.
 */

#ifndef RPI_SHIFTREG_GPIO_H_
#define RPI_SHIFTREG_GPIO_H_

#include "logging.h"

// Consider the following if WITH_ELK_PI_GPIO is defined
#ifdef WITH_ELK_PI_GPIO

#include <thread>
#include <pthread.h>

#include "fifo/circularfifo_memory_relaxed_aquire_release.h"

#include "hardware_backend/base_hw_backend.h"
#include "gpio_protocol/gpio_protocol.h"
#include "gpio_protocol_client/gpio_client.h"
#include "gpio_protocol_client/gpio_logger_interface.h"
#include "logging.h"

namespace sensei {
namespace hw_backend {
namespace rpi_gpio {

using namespace memory_relaxed_aquire_release;

// Board info
constexpr int NUM_DIGITAL_OUTPUTS = 32;
constexpr int NUM_DIGITAL_INPUTS = 32;
constexpr int NUM_ANALOG_INPUTS = 16;
constexpr int ADC_RES_IN_BITS = 10;

// Sizes of FIFO which sit between nrt and rt threads
constexpr int GPIO_PACKET_Q_SIZE = 150;
constexpr int GPIO_LOG_MSG_Q_SIZE = 50;

/**
 * @brief Enum to denote various stages of initialization of the real task.
 *        Used to denote what to cleanup when any of the stages fail.
 */
enum class RpiShiftregTaskState
{
    NOT_INITIALIZED,
    DEVICE_OPENED,
    PIN_DATA_MEM_ACQUIRED,
    LOGGER_THREAD_STARTED,
    RT_TASK_CREATED,
    RUNNING,
};

/**
 * @brief Rpi shift register based gpio hwbackend class. Instantiation of this
 *        class will facliltate the creation and management of real time and non
 *        real time threads which provide an interface between sensei and the
 *        gpio client. This class also is responsible for interfacing with the
 *        real time driver running on rpi to exchange gpio and adc data and
 *        processing them using the gpio client library.
 */
class RpiShiftregGpio : public BaseHwBackend
{
public:
    RpiShiftregGpio() : _is_logger_running(false),
                        _task_state(RpiShiftregTaskState::NOT_INITIALIZED),
                        _device_handle(0),
                        _pin_data(nullptr)
    {}

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
     *           using RpiShiftregTaskState which stores the initialization
     *           state.
     */
    void init() override;

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
     *        lock free fifo between the rt thread and the calling thread.
     *
     * @param rx_gpio_packet The packet to be received.
     * @return true          A packet was successfully received
     * @return false         No packet was received.
     */
    bool receive_gpio_packet(gpio::GpioPacket &rx_gpio_packet) override;

    /**
     * @brief Interface for sensei to query the status of the hw backend. If
     *        all tasks were initialized correctly, this will return true.
     *
     * @return true All tasks were initialized and the hw backend is up and
     *              running
     * @return false Something went wrong during initialization and the hwbackend
     *               was not initialized
     */
    bool get_status() override;

    /**
     * @brief Function to reconnect to hw backend. In this implentation of the
     *        hw backend, this has no meaning and is reduntant. Only exists
     *        as a placeholder to override the pure virtual base function.
     */
    void reconnect_to_gpio_hw() override
    {}

    /**
     * @brief The real time task which runs the gpio client and processes all
     *        the shiftregister gpio data. The function flow is:
     *        -> Get start time.
     *        -> Call io control on driver to get new input gpio data and adc
     *           data and send previous output gpio data.
     *        -> handles rx packets from received fromk sensei and calls the
     *           gpio client to process these packets.
     *        -> calls the gpio cient to process the the gpio data for this
     *           iteration
     *        -> Send any tx packets generated by the gpio client to sensei
     *        -> Send any log msgs generated by the gpio client to sensei
     *        -> get stop time
     *        -> sleep for the system tick period specified taking into acount
     *           execution time
     */
    void rt_shiftreg_gpio_task();

    /**
     * @brief The non real time logger task. This receives log msgs from the
     *        real time task and pipes them to the SENSEI log depending on their
     *        log level. This thread runs at a periodicity of 100ms. In each
     *        iteration, it checks for log msgs from the lock free fifo sent by
     *        the rt thread.
     */
    void nrt_logger_task();

private:
    /**
     * @brief helper function to destroy objects and join threads. It performs
     *        the necessary actions depending on the current task state
     */
    void _cleanup();

    /**
     * @brief Initializes xenomai. Maintains affinity of nrt threads to what
     *        they were before xenomai init.
     *
     * @return true if init is successful, false if otherwise.
     */
    bool _init_xenomai();

    /**
     * @brief checks the parameters of the driver to ensure that its compatible
     *        with the internal configuration.
     *
     * @return true if all params match with the internal configuration
     */
    bool _check_driver_params();

    /**
     * @brief helper function to read driver parameter as an integer. It looks
     *        for param in a predefined path where the driver populates the
     *        parameters as files.
     * @param param The driver parameter name to be read
     * @return The driver parameter if parameter exists, -1 otherwise
     */
    int _read_driver_param(char* param);

    /**
     * @brief Opens the shiftreg driver device
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

    void _pre_config_rt_loop();

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
     *        logger task, using a lock free fifo.
     */
    void _handle_log_msgs();

    bool _is_logger_running;

    pthread_t _processing_task;
    std::thread _logging_task;

    RpiShiftregTaskState _task_state;

    int _device_handle;

    CircularFifo<gpio::GpioPacket, GPIO_PACKET_Q_SIZE> _to_rt_thread_packet_fifo;
    CircularFifo<gpio::GpioPacket, GPIO_PACKET_Q_SIZE> _from_rt_thread_packet_fifo;
    CircularFifo<gpio::GpioLogMsg, GPIO_LOG_MSG_Q_SIZE> _from_rt_thread_log_msg_fifo;

    gpio::GpioClient<NUM_DIGITAL_OUTPUTS,
            NUM_DIGITAL_INPUTS,
            NUM_ANALOG_INPUTS,
            ADC_RES_IN_BITS> _gpio_client;

    uint32_t* _pin_data;
};

} // rpi_gpio
} // hw_backend
} // sensei

// Dummy RpiShiftregGpio when WITH_ELK_PI_GPIO is not defined
#else

namespace sensei {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("RpiGpio");

namespace hw_backend {
namespace rpi_gpio {

class RpiShiftregGpio : public BaseHwBackend
{
    void init() override
    {
        SENSEI_LOG_ERROR("Cannot Init Rpi Shiftreg Hw Backend. Its not enabled!");
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

    bool get_status() override
    {
        return false;
    }

    void reconnect_to_gpio_hw() override
    {}
};

} // rpi_gpio
} // hw_backend
} // sensei

#endif // WITH_ELK_PI_GPIO

#endif // RPI_SHIFTREG_GPIO_H_