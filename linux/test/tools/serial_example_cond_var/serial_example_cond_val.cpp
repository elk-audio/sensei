//
// Created by gustav on 3/7/16.
//
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "libserialport.h"
#include "Serial.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <string>
#include <condition_variable>

void print_time(timeval& tv, std::string s)

{
    struct tm* ptm;
    char time_string[40];

    ptm = localtime (&tv.tv_sec);
    strftime (time_string, sizeof (time_string), "%Y-%m-%d %H:%M:%S", ptm);
    int send_usec = tv.tv_usec - std::stoi(s.substr(s.size() - 7, 6));
    if (send_usec < 0){
        send_usec += 10000000;
    }
    std::cout << "Round trip time: " <<  send_usec << " usec" << std::endl;
}

int main()
{
    std::cout << "Starting... setting up!" << std::endl;
    LockedQueue queue;
    std::condition_variable notifier;
    std::mutex notify_mutex;
    Serial serial("/dev/ttyS10", &queue, &notifier);
    std::cout << "Serial device is " << (serial.connected()? "open" : "closed") << std::endl;

    serial.run();

    while (true)
    {
        std::unique_lock<std::mutex> lock(notify_mutex);
        struct timeval tv;
        std::unique_ptr<Message> m = queue.pop();
        gettimeofday (&tv, NULL);
        if (m)
        {
            std::cout << "Message received: " << m->name;
            print_time(tv, m->name);
        }
        notifier.wait_for(lock, std::chrono::milliseconds(100));
    }
    return 0;
}