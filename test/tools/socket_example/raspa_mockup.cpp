#include <iostream>
#include <cstring>
#include <thread>
#include <deque>
#include <cassert>
#include <cerrno>
#include <unistd.h>
#include <csignal>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>

#ifdef __APPLE__
#include <machine/endian.h>
#define htole32(arg) ntohl(arg)
#define le32toh(arg) htonl(arg)
#endif

#include "gpio_protocol/gpio_protocol.h"
#include "options.h"

constexpr char SENSEI_SOCKET[] = "/tmp/sensei";
constexpr char RASPA_SOCKET[] = "/tmp/raspa";
constexpr int  SOCKET_TIMEOUT_S = 2;


constexpr int  SILENT_THRESHOLD = 5;

using namespace gpio;

/*
 * Dummy task that can act as a stand in for raspalib
 * It claims a socket, prints what it receives on it
 * and sends some data to sensei if available
 */

inline uint32_t to_gpio_protocol_byteord(uint32_t word)
{
    return htole32(word);
}

inline uint32_t from_gpio_protocol_byteord(uint32_t word)
{
    return le32toh(word);
}

void print_packet(const GpioPacket& packet)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    for (int i = 0; i < sizeof(GpioPacket); ++i)
    {
        std::cout << ", " << (int)data[i]; 
    }
    std::cout << std::endl;
}

class RaspaMockup
{
public:
    RaspaMockup(std::chrono::milliseconds send_interval_ms, int sensor_id) : _send_interval(send_interval_ms),
                                                                             _sensor_id(sensor_id)
    {
        _in_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
        _out_socket = socket(AF_UNIX, SOCK_DGRAM, 0);
        sockaddr_un address;
        address.sun_family = AF_UNIX;
        strcpy(address.sun_path, RASPA_SOCKET);
        /* In case we didn't quit gracefully, clear the socket handle */
        unlink(RASPA_SOCKET);
        auto res = bind(_in_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
        assert(res == 0);

        timeval time;
        time.tv_usec = 0;
        time.tv_sec = SOCKET_TIMEOUT_S;
        res = setsockopt(_in_socket, SOL_SOCKET, SO_RCVTIMEO, &time, sizeof(time));
        std::cout << "Set recv timeout: "  << strerror(errno) << std::endl;

        assert(res == 0);
        std::cout << "Started Raspa Mockup"  << std::endl;
        connect_to_sensei();
    }

    ~RaspaMockup()
    {
        stop();
        unlink(RASPA_SOCKET);
    }

    void run()
    {
        if (!_running)
        {
            _running = true;
            _read_thread = std::thread(&RaspaMockup::read_loop, this);
            _write_thread = std::thread(&RaspaMockup::write_loop, this);
        }
    }
    void stop() 
    {
        if(_running)
        {
            std::cout << "Stopping " << std::endl;
            _running = false;
            _read_thread.join();
            _write_thread.join();
        }
    }

private:
    void connect_to_sensei()
     {
        /* Try to connect to sensei, it should be up and running now */
        sockaddr_un address;
        address.sun_family = AF_UNIX;
        strcpy(address.sun_path, SENSEI_SOCKET);
        auto res = connect(_out_socket, reinterpret_cast<sockaddr*>(&address), sizeof(sockaddr_un));
        if (res == 0)
        {
            timeval time;
            time.tv_usec = 0;
            time.tv_sec = SOCKET_TIMEOUT_S;
            auto res = setsockopt(_out_socket, SOL_SOCKET, SO_SNDTIMEO, &time, sizeof(time));
            assert(res == 0);
            std::cout << "Connected to Sensei!" << std::endl;
            _connected = true;
        }
        else
        {
            std::cout << "Failed to connect to sensei" << std::endl;
            _connected = false;
        }
    }

    void read_loop()
    {
        GpioPacket buffer;
        while (_running)
        {
            memset(&buffer, 0, sizeof(buffer));
            auto bytes = recv(_in_socket, &buffer, sizeof(buffer), 0);
            if (bytes >= (int)sizeof(buffer))
            {
                std::cout << "Received command: " ; print_packet(buffer);
                if (!_connected)
                {
                    connect_to_sensei();
                }
                handle_incoming_packet(buffer);
                _silent_count = 0;
            }
            else
            {
                std::cout << "Timeout on read: " << std::endl;
                if (++_silent_count > SILENT_THRESHOLD)
                {
                    _silent_count = 0;
                    connect_to_sensei();
                }
            }
        }
    }

    void write_loop()
    {
        GpioPacket buffer;
        while (_running)
        {
            if (_connected)
            {
                /* If we have queued acks, send them */
                if (!_ack_list.empty())
                {
                    std::lock_guard lock(_ack_list_mutex);
                    for (auto ack = _ack_list.rbegin(); ack != _ack_list.rend(); ++ack)
                    {
                        auto ret = send(_out_socket, &(*ack), sizeof(GpioPacket), 0);
                        if (ret < sizeof(GpioPacket))
                        {
                            std::cout << "Failed to send ack with error: " << ret << std::endl;
                        }
                        else
                        {
                            std::cout << "Sent ack to msg: " << from_gpio_protocol_byteord(ack->payload.gpio_ack_data.returned_seq_no) << std::endl;
                        }
                    }
                    _ack_list.clear();
                }
                else if (_send_count++ > _send_interval.count() / 5)
                {
                    /* Send a random value on the configured sensor */
                    GpioPacket packet;
                    packet.command = GPIO_CMD_GET_VALUE;
                    packet.payload.gpio_value_data.controller_id = _sensor_id;
                    packet.payload.gpio_value_data.controller_val = to_gpio_protocol_byteord(rand() % 128);

                    auto t = std::chrono::system_clock::now().time_since_epoch();
                    packet.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(t).count();  

                    auto ret = send(_out_socket, &packet, sizeof(packet), 0);
                    if (ret < sizeof(GpioPacket))
                    {
                        std::cout << "Failed to send random value: " << ret << std::endl;
                    }
                    else
                    {
                        std::cout << "Sent value msg: "  << packet.payload.gpio_value_data.controller_val << "on sensor" << packet.payload
                        .gpio_value_data.controller_id << std::endl;
                    }
                    _send_count = 0;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void handle_incoming_packet(const GpioPacket& packet)
    {
        /* Just reply every packet with an ok ack */
        GpioPacket ack{0};
        ack.command = GPIO_ACK;
        ack.payload.gpio_ack_data.returned_seq_no = packet.sequence_no;
        /* Signal ok to send it */
        std::lock_guard lock(_ack_list_mutex);
        {
            _ack_list.push_back(ack);
        }
    }


    bool            _running{false};
    bool            _connected{false};

    std::deque<GpioPacket>  _ack_list;
    std::mutex              _ack_list_mutex;

    std::thread     _read_thread;
    std::thread     _write_thread;

    int             _in_socket{0};
    int             _out_socket{0}; 
    int             _msg_count{0};
    int             _silent_count{0};
    int             _send_count{0};

    int             _sensor_id{0};
    std::chrono::milliseconds _send_interval{500};
};

bool running(true);

static void signal_handler(int sig_number)
{
    running = false;
}

int main(int argc, char* argv[])
{
    auto options = parse_options(argc, argv);
    if (!options)
    {
        return 1;
    }

    signal(SIGINT, signal_handler);
    RaspaMockup instance(options->send_interval, options->sensor_id);
    instance.run();
    while(running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    instance.stop();
    return 0;
}