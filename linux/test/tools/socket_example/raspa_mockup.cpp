

#include <iostream>
#include <cstring>
#include <thread>
#include <cassert>
#include <cerrno>
#include <unistd.h>
#include <csignal>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>

#include "../../../xmos_protocol/xmos_gpio_protocol.h"

constexpr char SENSEI_SOCKET[] = "/tmp/sensei";
constexpr char RASPA_SOCKET[] = "/tmp/raspa";
constexpr int  SOCKET_TIMEOUT_S = 2;
constexpr auto PING_INTERVAL = std::chrono::milliseconds(250);

constexpr int  SILENT_THRESHOLD = 5;

using namespace xmos;

/* Dummy task that can act as a stand in for raspalib
 * It claims a socket, prints what it receives on it
 * and sends some data to sensei if available 
 * 
 * build cmd: 
 * g++ raspa_mockup.cpp -g -o mockup -lpthread
 */

inline uint32_t to_xmos_byteord(uint32_t word)
{
    return htole32(word);
}

inline uint32_t from_xmos_byteord(uint32_t word)
{
    return le32toh(word);
}

void print_packet(const XmosGpioPacket& packet)
{
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);
    for (int i = 0; i < sizeof(XmosGpioPacket); ++i)
    {
        std::cout << ", " << (int)data[i]; 
    }
    std::cout << std::endl;
}

class RaspaMockup
{
public:
    RaspaMockup() 
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
        XmosGpioPacket buffer;
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
            else //if (bytes <= 0)
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
        XmosGpioPacket buffer;
        while (_running)
        {
            if (_connected)
            {
                /* If we have a queued ack, send it  */
                if (_send_ack)
                {
                    auto ret = send(_out_socket, &_ack, sizeof(_ack), 0);
                    if (ret < sizeof(_ack))
                    {
                        std::cout << "Failed to send ack with error: " << ret << std::endl;
                    }
                    else
                    {
                        std::cout << "Sent ack to msg: " << from_xmos_byteord(_ack.payload.ack_data.returned_seq_no) << std::endl;
                    }
                    _send_ack = false;
                }
                else
                {
                    /* Send a random value on controller 5 */
                    XmosGpioPacket packet;
                    packet.command = XMOS_CMD_GET_VALUE;
                    packet.payload.value_data.controller_id = 5;
                    packet.payload.value_data.controller_val = to_xmos_byteord(rand() % 128);

                    auto t = std::chrono::system_clock::now().time_since_epoch();
                    packet.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(t).count();  

                    auto ret = send(_out_socket, &packet, sizeof(packet), 0);
                    if (ret < sizeof(_ack))
                    {
                        std::cout << "Failed to send random value: " << ret << std::endl;
                    }
                    else
                    {
                        std::cout << "Sent value msg: "  << std::endl;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                /*snprintf(buffer.data, sizeof(buffer), "Raspa, pkt: %x", _msg_count++);
                buffer.data[19] = 0;
                auto ret = send(_out_socket, &buffer, sizeof(RaspaPacket), 0);
                if (ret < sizeof(RaspaPacket))
                {
                    std::cout << "Failed to send with error: " << ret << std::endl;
                }
                else
                {
                    std::cout << "Sent " << ret << " bytes, pkt: " << _msg_count << std::endl;
                }*/
            }
            std::this_thread::sleep_for(PING_INTERVAL);
        }
    }

    void handle_incoming_packet(const XmosGpioPacket& packet)
    {
        /* Just reply every packet with an ok ack */
        XmosGpioPacket ack{0};
        ack.command = XMOS_ACK;
        ack.payload.ack_data.returned_seq_no = packet.sequence_no;
        /* Signal ok to send it */
        _ack = ack;
        _send_ack = true;
    }

    
    bool            _running{false};
    bool            _connected{false};
    bool            _send_ack{false};
    XmosGpioPacket  _ack;

    std::thread     _read_thread;
    std::thread     _write_thread;

    int             _in_socket{0};
    int             _out_socket{0}; 
    int             _msg_count{0};
    int             _silent_count{0};
};

bool running(false);

static void signal_handler(int sig_number)
{
    running = false;
}

int main()
{
    signal(SIGINT, signal_handler);
    RaspaMockup instance;
    instance.run();
    running = true;
    while(running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    instance.stop();
    return 0;
}