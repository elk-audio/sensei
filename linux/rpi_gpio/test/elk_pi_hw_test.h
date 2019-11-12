/**
 * @brief Contains the definition of the ElkPiHwTest class for testing all the
 *        shiftregister gpio and adc on the ElkPi board.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 *
 * This should be included for any program which is meant to run a gpio test
 * on the ElkPi board with the gpio test board hat connected to it.
 */

#include <iostream>
#include <sys/mman.h>
#include <thread>

#include "shiftreg_driver_conf.h"

// xenomai includes
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <xenomai/init.h>
#include <rtdm/rtdm.h>
#include <cobalt/time.h>
#include <cobalt/sys/ioctl.h>
#include <cobalt/pthread.h>

#pragma GCC diagnostic pop

// Board defs
constexpr int NUM_DIGITAL_OUTPUTS = 32;
constexpr int NUM_DIGITAL_INPUTS = 32;
constexpr int NUM_ANALOG_INPUTS = 16;

constexpr int BUTTON_0_PIN = 0;
constexpr int BUTTON_1_PIN = 1;
constexpr int BUTTON_2_PIN = 2;
constexpr int BUTTON_3_PIN = 3;
constexpr int LED_0_PIN = 0;
constexpr int LED_1_PIN = 1;
constexpr int LED_2_PIN = 2;
constexpr int LED_3_PIN = 3;
constexpr int LOOPBACKED_OUTPUT_START_PIN = 4;
constexpr int LOOPBACKED_OUTPUT_END_PIN = 31;
constexpr int LOOPBACKED_INPUT_START_PIN = 4;
constexpr int LOOPBACKED_INPUT_END_PIN = 31;
constexpr int NUM_LOOPBACKED_INPUTS = LOOPBACKED_INPUT_END_PIN -
                                      LOOPBACKED_INPUT_START_PIN + 1;

/**
 * @brief Enum to denote various stages of initialization of the ElkPiHwTest
 */
enum class HwTestInitState
{
    NOT_INITIALIZED,
    DRIVER_INIT,
    PIN_DATA_MEM_RECVD,
    PRINT_SNAPSHOT_TASK_STARTED,
    RT_TASK_CREATED,
    DRIVER_STARTED,
};

/**
 * @brief static function to act as an entry point for the xenomai rt thread.
 * @param args ptr to an instance of ElkPiHwTest
 * @return nullptr
 */
static void* rt_task_entry(void* args);

/**
 * @brief Class which creates and manages the rt and non rt threads to perform
 *        shiftregister gpio test on the ElkPi board with the gpio board test hat
 *        connected on top of it. It will switch on output pins 0-3 depending
 *        on the state of input pins 0-3. The rest of the output pins are looped
 *        back on the gpio test board to the remaining input pins. Hence they
 *        are toggled every 1 sec and the input gpio values along with the adc
 *        values are printed. On initialization, start() should be called to
 *        start the thread and cleanup() should be called when stopping the test
 */
class ElkPiHwTest
{
public:
    ElkPiHwTest() : _pin_data(nullptr),
                    _task_state(HwTestInitState::NOT_INITIALIZED),
                    _data_read_flag(false),
                    _is_running_flag(false)
    {}

    /**
     * @brief Initializes the driver and starts the rt gpio test task and the nrt
     *        print task. If anything goes wrong, it calls cleanup to free up
     *        any resources which where allocated along the way
     * @return 0 on success, error code otherwise.
     */
    int start()
    {
        auto res = _init_xenomai();
        if (res != 0)
        {
            cleanup();
            return res;
        }

        res = _init_driver();
        if (res != 0)
        {
            cleanup();
            return res;
        }
        _task_state = HwTestInitState::DRIVER_INIT;

        res = _get_pin_data_mem_from_driver();
        if (res != 0)
        {
            cleanup();
            return res;
        }
        _task_state = HwTestInitState::PIN_DATA_MEM_RECVD;

        _print_snapshot_task = std::thread(&ElkPiHwTest::print_snapshot, this);
        _task_state = HwTestInitState::PRINT_SNAPSHOT_TASK_STARTED;

        res = _init_rt_task();
        if (res != 0)
        {
            cleanup();
            return res;
        }
        _task_state = HwTestInitState::RT_TASK_CREATED;

        res = _start_driver();
        if (res != 0)
        {
            cleanup();
            std::cout << res << std::endl;
            return res;
        }
        _task_state = HwTestInitState::DRIVER_STARTED;

        return 0;
    }

    /**
     * @brief RT gpio test loop. Output pins 0-3 are set on the value of input
     *        pins 0-3. All other output pins are toggled at a rate of 1sec.
     *        A snapshot of the remaining input pins and all the adc pins are
     *        taken and the nrt thread is signalled for it to print the snapshot.
     */
    void rt_gpio_test()
    {
        int interrupt_count = 0;
        int button_val = 0;
        int led_val = 1;
        uint32_t* input_pin_data;
        uint32_t* output_pin_data;
        uint32_t* analog_pin_data;

        input_pin_data = _pin_data;
        output_pin_data = input_pin_data + NUM_DIGITAL_INPUTS;
        analog_pin_data = output_pin_data + NUM_DIGITAL_OUTPUTS;

        _input_data_snapshot.fill(0);
        _adc_data_snapshot.fill(0);

        while (1)
        {
            auto res = __cobalt_ioctl(_device_handle,
                                      SHIFTREG_DRIVER_WAIT_ON_RT_TASK, NULL);
            if (res != 0)
            {
                break;
            }

            // loopback first four button's inverted val to first four leds
            output_pin_data[LED_0_PIN] = ~input_pin_data[BUTTON_0_PIN] & 0x1;
            output_pin_data[LED_1_PIN] = ~input_pin_data[BUTTON_1_PIN] & 0x1;
            output_pin_data[LED_2_PIN] = ~input_pin_data[BUTTON_2_PIN] & 0x1;
            output_pin_data[LED_3_PIN] = ~input_pin_data[BUTTON_3_PIN] & 0x1;

            interrupt_count++;
            if (interrupt_count != 1000)
            {
                continue;
            }

            interrupt_count = 0;

            // take snapshot of input pin data.
            int pin_num = LOOPBACKED_INPUT_START_PIN;
            for(auto& i : _input_data_snapshot)
            {
                i = input_pin_data[pin_num];
                pin_num++;
            }

            // take snapshot of adc pin data.
            pin_num = 0;
            for(auto& i : _adc_data_snapshot)
            {
                i = analog_pin_data[pin_num];
                pin_num++;
            }

            // signal nrt thread that it can print
            _data_read_flag = true;

            // toggle outputs
            for (int i = LOOPBACKED_OUTPUT_START_PIN;
                 i <= LOOPBACKED_OUTPUT_END_PIN; i++)
            {
                output_pin_data[i] = (output_pin_data[i] + 1) & 0x1;
            }
        }
    }

    /**
     * @brief Responsible for stopping of threads, deallocation of any memory
     *        and closing the driver. Can be called at any phase of initialization.
     */
    void cleanup()
    {
        switch (_task_state)
        {
        case HwTestInitState::DRIVER_STARTED:
            _stop_driver();
            _is_running_flag = false;
            __attribute__((fallthrough));

        case HwTestInitState::RT_TASK_CREATED:
            pthread_cancel(_processing_task);
            __cobalt_pthread_join(_processing_task, NULL);
            __attribute__((fallthrough));

        case HwTestInitState::PRINT_SNAPSHOT_TASK_STARTED:
            if (_print_snapshot_task.joinable())
            {
                _print_snapshot_task.join();
            }
            __attribute__((fallthrough));

        case HwTestInitState::PIN_DATA_MEM_RECVD:
            munmap(_pin_data, (size_t) getpagesize());
            __attribute__((fallthrough));

        case HwTestInitState::DRIVER_INIT:
            __cobalt_close(_device_handle);
            break;

        default:
            break;
        }

        _task_state = HwTestInitState::NOT_INITIALIZED;
    }

    /**
     * @brief NRT Thread responsible for printing the input and adc pin values.
     */
    void print_snapshot()
    {
        int pin_num = 0;
        _is_running_flag = true;

        while(_is_running_flag)
        {
            if(_data_read_flag)
            {
                pin_num = LOOPBACKED_INPUT_START_PIN;
                for (int i = 0; i < NUM_LOOPBACKED_INPUTS; i++)
                {
                    std::cout << "Input pin " << pin_num << "= " << _input_data_snapshot[i];
                    pin_num++;
                    std::cout << "\t\t";
                    if(i < NUM_ANALOG_INPUTS)
                    {
                        std::cout << "Analog Pin " << i << "= " << _adc_data_snapshot[i];
                    }
                    std::cout << std::endl;
                }
                std::cout << "\n\n\n";
                _data_read_flag = false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    /**
     * @brief Initializes xenomai
     * @return 0 on success, linux error code otherwise.
     */
    int _init_xenomai()
    {
        // Init xenomai
        int argc = 2;
        char** argv = (char**) malloc((argc + 1) * sizeof(char*));
        for (int i = 0; i < argc; i++)
        {
            argv[i] = (char*) malloc(32 * sizeof(char));
        }
        argv[argc] = NULL;
        strcpy(argv[0], "elk_pi_gpio_test");
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
            std::cout << "Failed to lock memory Error" << res << std::endl;
            return res;
        }

        std::cout << "Xenomai succesfully initialized" << std::endl;
        return 0;
    }

    /**
     * @brief Calls the open on the shiftregister rtdm driver and gets the
     *        device handle.
     * @return 0 on success, linux error code otherwise.
     */
    int _init_driver()
    {
        _device_handle = __cobalt_open(SHIFTREG_DEVICE_NAME, O_RDWR);
        if (_device_handle < 0)
        {
            std::cout << "Failed to open driver. Error " << _device_handle
                      << std::endl;
            return _device_handle;
        }

        std::cout << "Driver successfully opened" << std::endl;
        return 0;
    }

    /**
     * @brief gets the pin data from the driver using mmap ioctl
     * @return 0 on success, -1 if fail
     */
    int _get_pin_data_mem_from_driver()
    {
        // get memory of the pin data
        _pin_data = (uint32_t*) __cobalt_mmap(NULL,
                                              (size_t) getpagesize(),
                                              PROT_READ | PROT_WRITE,
                                              MAP_PRIVATE, _device_handle, 0);

        if (_pin_data == MAP_FAILED)
        {
            std::cout << "Failed to get pin data memory from driver"
                      << std::endl;
            return -1;
        }

        std::cout << "Pin data memory received from driver" << std::endl;
        return 0;
    }

    /**
     * @brief Creates the rt xenomai thread.
     * @return  0 on success, linux error code otherwise.
     */
    int _init_rt_task()
    {
        // Create the RT thread
        struct sched_param rt_params = {.sched_priority = 50};
        pthread_attr_t task_attributes;

        __cobalt_pthread_attr_init(&task_attributes);
        pthread_attr_setdetachstate(&task_attributes, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&task_attributes, SCHED_FIFO);
        pthread_attr_setschedparam(&task_attributes, &rt_params);


        auto res = __cobalt_pthread_create(&_processing_task, &task_attributes,
                                           &rt_task_entry,
                                           static_cast<void*>(this));
        if (res < 0)
        {
            std::cout << "Failed to start RT thread, Error " << res
                      << std::endl;
            return res;
        }

        return 0;
    }

    /**
     * @brief Helper function to tell the driver to start its kernel thread.
     * @return 0 on success, linux error code otherwise.
     */
    int _start_driver()
    {
        auto res = __cobalt_ioctl(_device_handle,
                                  SHIFTREG_DRIVER_START_RT_TASK);
        if (res < 0)
        {
            std::cout << "Cannot start rtdm driver. Error " << res << std::endl;
            return res;
        }

        return 0;
    }

    /**
     * @brief Helper function to tell the driver to stop its kernel thread.
     */
    void _stop_driver()
    {
        __cobalt_ioctl(_device_handle, SHIFTREG_DRIVER_STOP_RT_TASK);

        // wait for a few ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    uint32_t* _pin_data;
    std::array<uint32_t, NUM_ANALOG_INPUTS> _adc_data_snapshot;
    std::array<uint32_t, NUM_LOOPBACKED_INPUTS> _input_data_snapshot;

    int _device_handle;
    HwTestInitState _task_state;
    pthread_t _processing_task;
    bool _is_running_flag;
    bool _data_read_flag;
    std::thread _print_snapshot_task;
};

void* rt_task_entry(void* args)
{
    const auto elk_pi_hw_test = static_cast<ElkPiHwTest*>(args);
    elk_pi_hw_test->rt_gpio_test();

    return nullptr;
}