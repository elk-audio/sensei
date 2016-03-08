//
// Created by gustav on 3/7/16.
//
#include "libserialport.h"
#include "Serial.h"
#include <iostream>
#include <memory>


int main()
{
    std::cout << "Starting... setting up!" << std::endl;
    LockedQueue queue;
    Serial serial("/dev/ttyS10", &queue);
    std::cout << "Serial device is " << (serial.connected()? "open" : "closed") << std::endl;

    serial.run();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::unique_ptr<Message> m = queue.pop();
        if (m)
        {
            std::cout << "Message received: " << m->name << std::endl;
        }
    }
    return 0;
}