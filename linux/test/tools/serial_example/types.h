//
// Created by gustav on 3/4/16.
//

#ifndef SENSOR_READER_TYPES_H
#define SENSOR_READER_TYPES_H

#include <cstdint>
#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>

class Message {
public:
    int identifier;
    uint32_t command;
    uint32_t subCommand;
    uint32_t timestamp;
    std::string name;
};

class LockedQueue
{
public:
    void push(const Message& message)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_front(message);
    }

    std::unique_ptr<Message> pop()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queue.empty())
        {
            return std::unique_ptr<Message>(nullptr);
        }
        std::unique_ptr<Message> message(new Message(_queue.back()));
        _queue.pop_back();
        return message;
    }
private:
    std::deque<Message> _queue;
    std::mutex _mutex;
};

#endif //SENSOR_READER_TYPES_H
