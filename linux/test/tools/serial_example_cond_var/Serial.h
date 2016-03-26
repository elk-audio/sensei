//
// Created by gustav on 3/4/16.
//

#ifndef SENSOR_READER_SERIAL_H
#define SENSOR_READER_SERIAL_H

#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "types.h"

class Serial
{
    public:
    Serial(std::string const& port, LockedQueue* queue, std::condition_variable* notifier);
    ~Serial();
    int send(Message const & message);
    bool connected();
    void run();

    private:
    void readLoop();
    sp_port* _port;
    bool _connected;
    LockedQueue* _queue;
    std::condition_variable* _notifier;
    std::mutex _notifier_lock;
    bool _running;
    std::thread _thread;
};


class MessageDispatcher
{
public:
    //virtual MessageDispatcher();
    //~virtual MessageDispatcher();

    virtual int send(Message const& message);
private:

};



#endif //SENSOR_READER_SERIAL_H
