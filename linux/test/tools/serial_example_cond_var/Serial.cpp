//
// Created by gustav on 3/4/16.
//

#include "libserialport.h"
#include "Serial.h"
#include <iostream>
#include <chrono>


int setupPort(std::string const& name, sp_port** port)
{
    sp_return ret;
    ret = sp_get_port_by_name(name.c_str(), port);
    if (ret != SP_OK)
    {
        std::cout << "Get port returned " << ret << std::endl;
        return ret;
    }
    ret = sp_open(*port, SP_MODE_READ_WRITE );
    if (ret != SP_OK)
    {
        char* err = sp_last_error_message();
        std::cout << "Open port returned " << ret << ", " << err << std::endl;
        sp_free_error_message(err);
        return ret;
    }
    return SP_OK;
}

Serial::Serial(std::string const& port, LockedQueue* queue, std::condition_variable* notifier) :
    _connected(false),
    _queue(queue),
    _notifier(notifier),
    _running(false)
{
    if (setupPort(port, &_port) == SP_OK)
    {
        _connected = true;
    }
}


Serial::~Serial()
{
    _running = false;
    if (_thread.joinable())
    {
        _thread.join();
    }
    sp_free_port(_port);
}


int Serial::send(Message const& message)
{
    return 0;
}

void Serial::run()
{
    _running = true;
    _thread = std::thread(&Serial::readLoop, this);

}


bool Serial::connected()
{
    return _connected;
}

void Serial::readLoop()
{
    while(_running)
    {
        char buffer[100] = {0};
        int ret = sp_blocking_read_next(_port, buffer, sizeof(buffer), 1000);
        if (ret > 0)
        {
            Message m;
            m.name.append(buffer);
            _queue->push(m);
            _notifier->notify_one();
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}



