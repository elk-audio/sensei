

#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

#include "raspa_frontend.h"

#include "logging.h"

namespace sensei {
namespace hw_frontend {


constexpr char SENSEI_SOCKET[] = "/tmp/sensei";
constexpr char RASPA_SOCKET[] = "/tmp/raspa";
constexpr int  SOCKET_TIMEOUT_US = 250'000;
constexpr auto READ_WRITE_TIMEOUT = std::chrono::seconds(1);
SENSEI_GET_LOGGER;

RaspaFrontend::RaspaFrontend(SynchronizedQueue <std::unique_ptr<sensei::Command>>*in_queue,
                             SynchronizedQueue <std::unique_ptr<sensei::BaseMessage>>*out_queue)
              : HwFrontend(in_queue, out_queue),
                _state(ThreadState::STOPPED),
                _in_socket(-1),
                _out_socket(-1),
                _connected(false),
                _muted(false),
                _verify_acks(false)
{
    _in_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    _out_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (_in_socket >= 0 && _out_socket >= 0)
    {
        /* Sensei tries to bind one socket to SENSEI_SOCKET, then connects
         * the other one to RASPA_SOCKET, which should be present since
         * Raspalib should already be running.
         * Raspalib then does the opposite but waits with connecting until
         * Sensei reports that it is up and running */

        sockaddr_un address;
        address.sun_family = AF_UNIX;
        strcpy(address.sun_path, SENSEI_SOCKET);
        // In case sensei didn't quit gracefully, clear the socket handle
        unlink(SENSEI_SOCKET);
        auto res = bind(_in_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
        if (res != 0)
        {
            SENSEI_LOG_ERROR("Failed to bind to socket: {}", strerror(errno));
        }
        strcpy(address.sun_path, RASPA_SOCKET);
        res = connect(_out_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
        if (res != 0)
        {
            SENSEI_LOG_ERROR("Failed to connect to Raspa socket: {}", strerror(errno));
        }
        else
        {
            // TODO - Send a hello to raspa
            _connected = true;
            timeval time;
            time.tv_sec = 0;
            time.tv_usec = SOCKET_TIMEOUT_US;
            auto res = setsockopt(_in_socket, SOL_SOCKET, SO_RCVTIMEO, &time, sizeof(time));
            if (res != 0 )
            {
                SENSEI_LOG_ERROR("Failed to set incoming socket timeout: {}", strerror(errno));
            }
            res = setsockopt(_out_socket, SOL_SOCKET, SO_SNDTIMEO, &time, sizeof(time));
            if (res != 0 )
            {
                SENSEI_LOG_ERROR("Failed to set outgoing socket timeout: {}", strerror(errno));
            }
        }
    }
    else
    {
        SENSEI_LOG_ERROR("Failed to get sockets from system");
    }
}

RaspaFrontend::~RaspaFrontend()
{
    unlink(SENSEI_SOCKET);
}

bool RaspaFrontend::connected()
{
    return _connected;
}

void RaspaFrontend::run()
{
    SENSEI_LOG_INFO("Starting read and write threads");
    if (_connected || _state.load() == ThreadState::STOPPED)
    {
        _state.store(ThreadState::RUNNING);
        _read_thread = std::thread(&RaspaFrontend::read_loop, this);
        _write_thread = std::thread(&RaspaFrontend::write_loop, this);
    }
    else
    {
        SENSEI_LOG_ERROR("Cant start RaspaFrontend, {}", _connected? "Already running" : "Not connected");
    }
}

void RaspaFrontend::stop()
{
    if (_state.load() == ThreadState::RUNNING)
    {
        return;
    }
    SENSEI_LOG_INFO("Stopping RaspaFrontend");
    _state.store(ThreadState::STOPPING);
    if (_read_thread.joinable())
    {
        _read_thread.join();
    }
    if (_write_thread.joinable())
    {
        _write_thread.join();
    }
    _state.store(ThreadState::STOPPED);
    SENSEI_LOG_INFO("Threads stopped");
}

void RaspaFrontend::mute(bool enabled)
{
    _muted = enabled;
}

void RaspaFrontend::verify_acks(bool enabled)
{
    _verify_acks = enabled;
}

void RaspaFrontend::read_loop()
{
    RaspaPacket buffer = {0};
    while (_state.load() == ThreadState::RUNNING)
    {
        memset(&buffer, 0, sizeof(buffer));
        auto bytes = recv(_in_socket, &buffer, sizeof(buffer), 0);
        SENSEI_LOG_INFO("Received: {}", buffer.data);
        if (_muted == false && bytes >= static_cast<ssize_t>(sizeof(RaspaPacket)))
        {
            SENSEI_LOG_INFO("Received from raspa: {}", buffer.data);
        }
        if (bytes < 0)
        {
            SENSEI_LOG_WARNING("Returned {} on recv", strerror(errno));
        }
    }
}

void RaspaFrontend::write_loop()
{
    while (_state.load() == ThreadState::RUNNING)
    {
        _in_queue->wait_for_data(READ_WRITE_TIMEOUT);
        if (_in_queue->empty())
        {
            continue;
        }

        std::unique_ptr<Command> message = _in_queue->pop();
        // TODO - convert to Raspa protocol
        RaspaPacket packet;
        strcpy(packet.data, "Packet");
        auto ret = send(_out_socket, &packet, sizeof(RaspaPacket), 0);
        if (ret < static_cast<ssize_t>(sizeof(RaspaPacket)))
        {
            SENSEI_LOG_WARNING("Sending packet on socket failed {}", ret);
        }
    }
}


}
}