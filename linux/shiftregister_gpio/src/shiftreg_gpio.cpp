/**
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 */

#include <string>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <sched.h>
#include <array>
#include <iostream>

#include "shiftreg_gpio/shiftreg_gpio.h"
#include "shiftreg_driver_conf.h"
#include "gpio_protocol_client/gpio_client.h"

// xenomai includes
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <xenomai/init.h>
#include <rtdm/rtdm.h>
#include <cobalt/time.h>
#include <cobalt/sys/ioctl.h>
#include <cobalt/pthread.h>

#pragma GCC diagnostic pop

extern int optind;

namespace sensei {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("Shiftreg Gpio");

namespace hw_backend {
namespace shiftregister_gpio {

// Should be lower than audio thread
constexpr int TASK_PRIORITY = 50;
constexpr int MAX_PACKETS_PER_TICK = 50;
constexpr int MAX_LOG_MSGS_SENT_PER_TICK = 5;
constexpr int MAX_LOG_MSGS_RX_PER_TICK = 20;
constexpr int LOGGER_THREAD_TASK_PERIOD_MS = 250;

// Compile time version check of gpio protocol
static_assert(GPIO_PROTOCOL_VERSION_MAJOR == 0,
              "Gpio protocol major version mismatch");
static_assert(GPIO_PROTOCOL_VERSION_MINOR == 2,
              "Gpio protocol minor version mismatch");

/**
 * @brief static function to act as an entry point for the xenomai rt thread.
 * @param args ptr to an instance of ShiftregGpio
 * @return nullptr
 */
static void* rt_task_entry(void* args)
{
    const auto shiftreg_gpio = static_cast<ShiftregGpio*>(args);
    shiftreg_gpio->rt_shiftreg_gpio_task();

    return nullptr;
}

bool ShiftregGpio::init()
{
    // initialize xenomai
    if (!_init_xenomai())
    {
        SENSEI_LOG_ERROR("Failed to init xenomai.");
        _cleanup();
        return false;
    }

    // initialize driver
    if (!_init_driver())
    {
        SENSEI_LOG_ERROR("Failed to init driver");
        _cleanup();
        return false;
    }
    _task_state = ShiftregTaskState::DEVICE_OPENED;

    // check size of input, output, analog pins and adc res matches driver
    if (!_check_driver_params())
    {
        SENSEI_LOG_ERROR("Failed to init, error in driver parameters");
        _cleanup();
        return false;
    }

    const int adc_chans_per_tick =
            _read_driver_param((char*) "adc_chans_per_tick");

    if (NUM_ANALOG_INPUTS > 0)
    {
        if (adc_chans_per_tick == 0 ||
            NUM_ANALOG_INPUTS % adc_chans_per_tick != 0 ||
            adc_chans_per_tick > NUM_ANALOG_INPUTS)
        {
            SENSEI_LOG_ERROR(
                    "Num analog pins {} is not a multiple of the num adc" \
                             " channels {} sampled per tick mentioned in the driver.",
                    adc_chans_per_tick);
            SENSEI_LOG_ERROR( "Reconfigure driver with a valid num of " \
                              "adc channels sampled per tick");

            _cleanup();
            return false;
        }
    }

    if (!_get_pin_data_mem_from_driver())
    {
        SENSEI_LOG_ERROR("Failed to get pin data memory from driver");
        _cleanup();
        return false;
    }
    _task_state = ShiftregTaskState::PIN_DATA_MEM_ACQUIRED;

    // start the logging thread
    _logging_task = std::thread(&ShiftregGpio::nrt_logger_task, this);
    _task_state = ShiftregTaskState::LOGGER_THREAD_STARTED;

    // create the client
    uint32_t* input_pin_data = _pin_data;
    uint32_t* output_pin_data = _pin_data + NUM_DIGITAL_INPUTS;
    uint32_t* analog_pin_data = output_pin_data + NUM_DIGITAL_OUTPUTS;

    if (!_gpio_client.init(input_pin_data, output_pin_data, analog_pin_data,
                           adc_chans_per_tick))
    {
        SENSEI_LOG_ERROR("Cannot init gpio client.");
        return false;
    }

    if (!_init_rt_task())
    {
        SENSEI_LOG_ERROR("Failed to start RT task");
        _cleanup();
        return false;
    }
    _task_state = ShiftregTaskState::RT_TASK_CREATED;

    if (!_start_driver())
    {
        SENSEI_LOG_ERROR("Failed to start driver");
        _cleanup();
        return false;
    }
    _task_state = ShiftregTaskState::RUNNING;
    return true;
}

void ShiftregGpio::deinit()
{
    _cleanup();
}

bool ShiftregGpio::send_gpio_packet(const gpio::GpioPacket &tx_gpio_packet)
{
    return _to_rt_thread_packet_fifo.push(tx_gpio_packet);
}

bool ShiftregGpio::receive_gpio_packet(gpio::GpioPacket &rx_gpio_packet)
{
    return _from_rt_thread_packet_fifo.pop(rx_gpio_packet);
}

void ShiftregGpio::rt_shiftreg_gpio_task()
{
    _gpio_client.reset();

    _pre_config_rt_loop();

    // set driver tick period after gpio client has been configured
    const auto system_tick_period_ns = _gpio_client.get_tick_period_ns();
    __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_SET_TICK_PERIOD,
            &system_tick_period_ns);

    _post_config_rt_loop();
}

void ShiftregGpio::nrt_logger_task()
{
    _is_logger_running = true;
    gpio::GpioLogMsg log_msg;
    int num_log_msgs_per_iter = 0;
    while (_is_logger_running)
    {
        while (_from_rt_thread_log_msg_fifo.pop(log_msg) &&
               num_log_msgs_per_iter < MAX_LOG_MSGS_RX_PER_TICK)
        {
            switch (log_msg.level)
            {
            case gpio::GpioLogLevel::GPIO_LOG_INFO:
                SENSEI_LOG_INFO("{}", log_msg.msg.data());
                break;

            case gpio::GpioLogLevel::GPIO_LOG_WARNING:
                SENSEI_LOG_WARNING("{}", log_msg.msg.data());
                break;

            case gpio::GpioLogLevel::GPIO_LOG_ERROR:
                SENSEI_LOG_ERROR("{}", log_msg.msg.data());
                break;
            }

            num_log_msgs_per_iter++;
        }

        num_log_msgs_per_iter = 0;
        std::this_thread::sleep_for(
                std::chrono::milliseconds(LOGGER_THREAD_TASK_PERIOD_MS));
    }
}

void ShiftregGpio::_cleanup()
{
    switch (_task_state)
    {
    case ShiftregTaskState::RUNNING:
        _stop_driver();
        __attribute__((fallthrough));

    case ShiftregTaskState::RT_TASK_CREATED:
        pthread_cancel(_processing_task);
        __cobalt_pthread_join(_processing_task, NULL);
        __attribute__((fallthrough));

    case ShiftregTaskState::LOGGER_THREAD_STARTED:
        _is_logger_running = false;
        if (_logging_task.joinable())
        {
            _logging_task.join();
        }
        __attribute__((fallthrough));

    case ShiftregTaskState::PIN_DATA_MEM_ACQUIRED:
        munmap(_pin_data, (size_t) getpagesize());
        __attribute__((fallthrough));

    case ShiftregTaskState::DEVICE_OPENED:
        __cobalt_close(_device_handle);
        break;

    default:
        break;
    }

    _task_state = ShiftregTaskState::NOT_INITIALIZED;
}

bool ShiftregGpio::_init_xenomai()
{
    // Init xenomai
    int argc = 2;
    char** argv = (char**) malloc((argc + 1) * sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        argv[i] = (char*) malloc(32 * sizeof(char));
    }
    argv[argc] = NULL;
    strcpy(argv[0], "sensei");
    strcpy(argv[1], "--cpu-affinity=0,1,2,3");
    optind = 1;

    xenomai_init(&argc, (char* const**) &argv);

    for (int i = 0; i < argc; i++)
    {
        free(argv[i]);
    }
    free(argv);

    auto res = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (res != 0)
    {
        SENSEI_LOG_ERROR("Failed to lock memory {}", res);
        return false;
    }

    SENSEI_LOG_INFO("Xenomai succesfully initialized");
    return true;
}

bool ShiftregGpio::_check_driver_params()
{
    if (_read_driver_param((char*) "num_input_pins") != NUM_DIGITAL_INPUTS)
    {
        SENSEI_LOG_ERROR("Num input pins mismatched with driver");
        return false;
    }

    if (_read_driver_param((char*) "num_output_pins") != NUM_DIGITAL_OUTPUTS)
    {
        SENSEI_LOG_ERROR("Num output pins mismatched with driver");
        return false;
    }

    if (_read_driver_param((char*) "num_analog_pins") != NUM_ANALOG_INPUTS)
    {
        SENSEI_LOG_ERROR("Num input pins mismatched with driver");
        return false;
    }

    if (_read_driver_param((char*) "adc_res") != ADC_RES_IN_BITS)
    {
        SENSEI_LOG_ERROR("ADC resolution mismatched with driver");
        return false;
    }

    SENSEI_LOG_INFO("Driver params matched");
    return true;
}

int ShiftregGpio::_read_driver_param(char* param)
{
    constexpr int PATH_LEN = 100;
    constexpr int VAL_STR_LEN = 25;
    char path[PATH_LEN];
    char value[VAL_STR_LEN];

    std::snprintf(path, PATH_LEN, SHIFTREG_MODULE_PARAMETERS_PATH"/%s", param);

    auto rtdm_file = open(path, O_RDONLY);

    if (rtdm_file < 0)
    {
        SENSEI_LOG_ERROR("Cannot read driver params, invalid param file");
        return -1;
    }

    if (read(rtdm_file, value, VAL_STR_LEN) == -1)
    {
        SENSEI_LOG_ERROR("Cannot read driver params, invalid parameter");
        return -1;
    }

    close(rtdm_file);

    return atoi(value);
}

bool ShiftregGpio::_init_driver()
{
    _device_handle = __cobalt_open(SHIFTREG_DEVICE_NAME, O_RDWR);
    if (_device_handle < 0)
    {
        SENSEI_LOG_ERROR("Failed to open driver. Error {}", _device_handle);
        return false;
    }

    SENSEI_LOG_INFO("Driver successfully opened");
    return true;
}

bool ShiftregGpio::_get_pin_data_mem_from_driver()
{
    // get memory of the pin data
    _pin_data = (uint32_t*) __cobalt_mmap(NULL,
                                          (size_t) getpagesize(),
                                          PROT_READ | PROT_WRITE,
                                          MAP_PRIVATE, _device_handle, 0);

    if (_pin_data == MAP_FAILED)
    {
        SENSEI_LOG_ERROR("Failed to get pin data memory from driver");
        return false;
    }

    SENSEI_LOG_INFO("Pin data memory received from driver");
    return true;
}

bool ShiftregGpio::_init_rt_task()
{
    // Create the RT thread
    struct sched_param rt_params = {.sched_priority = TASK_PRIORITY};
    pthread_attr_t task_attributes;

    __cobalt_pthread_attr_init(&task_attributes);
    pthread_attr_setdetachstate(&task_attributes, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&task_attributes, SCHED_FIFO);
    pthread_attr_setschedparam(&task_attributes, &rt_params);

    // Force affinity on the last thread.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(get_nprocs() - 1, &cpuset);
    auto res = pthread_attr_setaffinity_np(&task_attributes, sizeof(cpu_set_t),
                                           &cpuset);
    if (res < 0)
    {
        SENSEI_LOG_ERROR(
                "While starting rt task, cannot set affinity of the first thread, Error {}",
                res);
        return false;
    }

    res = __cobalt_pthread_create(&_processing_task, &task_attributes,
                                  &rt_task_entry, static_cast<void*>(this));
    if (res < 0)
    {
        SENSEI_LOG_ERROR("Failed to start RT thread, Error {}", res);
        return false;
    }

    /* After Xenomai init + RT thread creation, all non-RT threads have the
     * affinity restricted to one single core. This reverts back to the default
     * of using all cores */
    CPU_ZERO(&cpuset);
    for (int i = 0; i < get_nprocs(); i++)
    {
        CPU_SET(i, &cpuset);
    }
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    return true;
}

bool ShiftregGpio::_start_driver()
{
    auto res = __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_START_RT_TASK);
    if(res < 0)
    {
        SENSEI_LOG_ERROR("Cannot start rtdm driver. Error {}", res);
        return false;
    }

    return true;
}

void ShiftregGpio::_stop_driver()
{
    __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_STOP_RT_TASK);

    // wait for a few ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

inline void ShiftregGpio::_pre_config_rt_loop()
{
    while (!_gpio_client.is_running())
    {
        auto res = __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_WAIT_ON_RT_TASK, NULL);
        if (res != 0)
        {
            break;
        }

        _handle_rx_packets();
        _handle_tx_packets();
        _handle_log_msgs();
    }
}

inline void ShiftregGpio::_post_config_rt_loop()
{
    while (1)
    {
        auto res = __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_WAIT_ON_RT_TASK, NULL);
        if (res != 0)
        {
            break;
        }

        _handle_rx_packets();
        _gpio_client.process();
        _handle_tx_packets();
        _handle_log_msgs();
    }
}

inline void ShiftregGpio::_handle_rx_packets()
{
    int num_rx_packets = 0;
    gpio::GpioPacket rx_packet;

    while (num_rx_packets < MAX_PACKETS_PER_TICK)
    {
        if (_to_rt_thread_packet_fifo.pop(rx_packet))
        {
            _gpio_client.handle_rx_packet(rx_packet);
            _gpio_client.clear_packet(rx_packet);
            num_rx_packets++;
            continue;
        }

        return;
    }
}

inline void ShiftregGpio::_handle_tx_packets()
{
    int num_tx_packets = 0;
    while (_gpio_client.has_new_tx_packet() &&
           num_tx_packets < MAX_LOG_MSGS_SENT_PER_TICK)
    {
        auto packet = _gpio_client.get_next_tx_packet();
        _from_rt_thread_packet_fifo.push(*packet);
        num_tx_packets++;
    }
}

void ShiftregGpio::_handle_log_msgs()
{
    int num_log_msg = 0;
    while (_gpio_client.has_new_log_msg() &&
           num_log_msg < MAX_LOG_MSGS_SENT_PER_TICK)
    {
        auto log_msg = _gpio_client.get_log_msg();
        _from_rt_thread_log_msg_fifo.push(*log_msg);
        num_log_msg++;
    }
}

} // shiftregister_gpio
} // hw_bacend
} //sensei