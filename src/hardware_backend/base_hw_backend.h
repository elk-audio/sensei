/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Interface between the hw frontend and the underlying gpio hardware
 *        which implements gpio logic according to the gpio protocol
 *        specifications
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
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
    /**
     * @brief Constructor for the base hw backend.
     * @param recv_packet_timeout The blocking timeout for receiving packets
     */
    BaseHwBackend(std::chrono::milliseconds recv_packet_timeout) :
            _recv_packet_timeout(recv_packet_timeout)
    {}

    virtual ~BaseHwBackend() = default;

    /**
     * @brief Function to initialize the hardware backend
     *
     * @return true if initialization was successful
     * @return false false if not
     */
    virtual bool init() = 0;

    /**
     * @brief Function to de initialize the hardware backend
     *
     */
    virtual void deinit() = 0;

    /**
     * @brief Send GPIO Packet to gpio hardware device. This is a non blocking
     *        call
     *
     * @param tx_gpio_packet The gpio packet to be sent
     * @return true if packet was sent successfully
     * @return false if packet was not sent.
     */
    virtual bool send_gpio_packet(const gpio::GpioPacket& tx_gpio_packet) = 0;

    /**
     * @brief Receive a GPIO Packet from the gpio hardware device. This is a
     *        blocking call with a timeout of _recv_packet_timeout
     *
     * @param rx_gpio_packet The gpio packet to be received
     * @return true if packet was received successfully
     * @return false if no packet was received.
     */
    virtual bool receive_gpio_packet(gpio::GpioPacket& rx_gpio_packet) = 0;

protected:
    std::chrono::milliseconds _recv_packet_timeout;
};

/**
 * @brief Dummy hw backend
 */
class NoOpHwBackend : public BaseHwBackend
{
public:
    NoOpHwBackend(std::chrono::milliseconds recv_packet_timeout) :
                                BaseHwBackend(recv_packet_timeout)
    {}

    bool init()
    {
        return  false;
    }

    void deinit() {}

    // To suppress warnings
    bool send_gpio_packet([[maybe_unused]] const gpio::GpioPacket& tx_gpio_packet)
    {
        return true;
    }

    // To suppress warnings
    bool receive_gpio_packet([[maybe_unused]] gpio::GpioPacket& rx_gpio_packet)
    {
        return true;
    }
};

} // namespace hw_backend
} // namespace sensei

#endif // SENSEI_BASE_HW_BACKEND_H