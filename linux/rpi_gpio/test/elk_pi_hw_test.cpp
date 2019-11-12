/**
 * @brief Program to test all the 32 inputs, 32 outputs and 16 analog pins of
 *        the elk pi board using the shiftregister rtdm driver.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 */

#include <iostream>
#include <signal.h>

#include "elk_pi_hw_test.h"

static volatile sig_atomic_t running = 1;

static void sigint_handler(int sig_number)
{
    running = 0;
}

int main()
{
    std::cout << "\n########################\n";
    std::cout << "Elk Pi Gpio Hw test\n";

    ElkPiHwTest elk_pi_hw_test;

    signal(SIGINT, sigint_handler);

    auto res = elk_pi_hw_test.start();
    if(res != 0)
    {
        exit(res);
    }

    while(running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nStopping Test....\n";
    elk_pi_hw_test.cleanup();
}