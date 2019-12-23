#include <chrono>
#include <iostream>
#include <cstring>

#include <sys/un.h>
#include <sys/socket.h>

#include "gpio_hw_socket.h"
#include "logging.h"

namespace sensei {
namespace hw_backend {

constexpr char SENSEI_SOCKET[] = "/tmp/sensei";
constexpr size_t GPIO_PACKET_SIZE = sizeof(gpio::GpioPacket);

SENSEI_GET_LOGGER_WITH_MODULE_NAME("gpio_hw_socket");

bool GpioHwSocket::init()
{
    _in_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
    _out_socket = socket(AF_UNIX, SOCK_DGRAM, 0);

    if (_in_socket >= 0 && _out_socket >= 0)
    {
        /* Sensei binds one socket to SENSEI_SOCKET, then tries to connect
         * the other one to gpio_hw_socket_name, if this fails, Sensei will retry
         * the connection to gpio_hw_socket_name when it receives something on
         * SENSEI_SOCKET.
         * gpio_hw_socket_name should do the opposite when it starts up, binds to
         * its own port and waits for a message. This way the processes can be
         * started in any order and synchronise  */

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
        else
        {
            timeval time;
            time.tv_sec = 0;
            time.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(_recv_packet_timeout).count();
            auto res = setsockopt(_in_socket, SOL_SOCKET, SO_RCVTIMEO, &time, sizeof(time));
            if (res != 0)
            {
                SENSEI_LOG_ERROR("Failed to set incoming socket timeout: {}", strerror(errno));
            }
        }

        // attempt connection with the gpio hw socket
        _connect_to_gpio_hw_socket();

        return true;
    }

    SENSEI_LOG_ERROR("Failed to get sockets from system");
    return false;
}

void GpioHwSocket::deinit()
{
    unlink(SENSEI_SOCKET);
}

bool GpioHwSocket::send_gpio_packet(const gpio::GpioPacket& tx_gpio_packet)
{
    auto bytes = send(_out_socket, &tx_gpio_packet, GPIO_PACKET_SIZE, 0);
    if(bytes < static_cast<ssize_t>(GPIO_PACKET_SIZE))
    {
        SENSEI_LOG_WARNING("Sending packet on socket failed {}, Attempting"
                           " to reconnect to socket..,", bytes);

        _connect_to_gpio_hw_socket();
        return false;
    }

    return true;
}

bool GpioHwSocket::receive_gpio_packet(gpio::GpioPacket& rx_gpio_packet)
{
    memset(&rx_gpio_packet, 0, GPIO_PACKET_SIZE);
    auto bytes = recv(_in_socket, &rx_gpio_packet, GPIO_PACKET_SIZE, 0);
    if(bytes < static_cast<ssize_t>(GPIO_PACKET_SIZE))
    {
        return false;
    }

    return true;
}

inline void GpioHwSocket::_connect_to_gpio_hw_socket()
{
    sockaddr_un address;
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, _gpio_hw_socket_name.c_str());
    auto res = connect(_out_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
    if (res != 0)
    {
        SENSEI_LOG_ERROR("Failed to connect to Gpio Hw socket: {}", strerror(errno));
        return;
    }

    timeval time;
    time.tv_sec = 0;
    time.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(_recv_packet_timeout).count();
    res = setsockopt(_out_socket, SOL_SOCKET, SO_SNDTIMEO, &time, sizeof(time));
    if (res != 0)
    {
        SENSEI_LOG_ERROR("Failed to set outgoing socket timeout: {}", strerror(errno));
        return;
    }

    SENSEI_LOG_INFO("Connected to Gpio Hw Socket!");
    return;
}

} // gpio_hw_socket
} // sensei