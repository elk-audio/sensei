#ifndef SENSEI_BASE_HW_BACKEND_H
#define SENSEI_BASE_HW_BACKEND_H

#include "gpio_protocol/gpio_protocol.h"

namespace sensei {
namespace hw_backend {

/**
 * @brief Base class which provides a common interface to gpio hardware devices
 *        for the hardware frontend
 */
class BaseHwBackend
{
public:
    BaseHwBackend()
    {}

    virtual ~BaseHwBackend() = default;

    /**
     * @brief Function to initialize the hardware backend
     *
     * @return true if initialization was successful
     * @return false false if not
     */
    virtual void init() = 0;

    /**
     * @brief Function to de initialize the hardware backend
     *
     */
    virtual void deinit() = 0;

    /**
     * @brief Send GPIO Packet to gpio hardware device
     *
     * @param tx_gpio_packet The gpio packet to be sent
     * @return true if packet was sent successfully
     * @return false if packet was not sent.
     */
    virtual bool send_gpio_packet(const gpio::GpioPacket& tx_gpio_packet) = 0;

    /**
     * @brief Receive a GPIO Packet from the gpio hardware device
     *
     * @param rx_gpio_packet The gpio packet to be received
     * @return true if packet was received successfully
     * @return false if no packet was received.
     */
    virtual bool receive_gpio_packet(gpio::GpioPacket& rx_gpio_packet) = 0;

    /**
     * @brief Check the status of the gpio hardware device
     *
     * @return true if stuats is ok, false if not. Should have adequeate
     *         log messages in case of error.
     */
    virtual bool get_status() = 0;

    /**
     * @brief Virtual function to restablish communication to a gpio hw device
     *
     */
    virtual void reconnect_to_gpio_hw() = 0;
};

} // hw_backend
} // sensei

#endif // SENSEI_BASE_HW_BACKEND_H