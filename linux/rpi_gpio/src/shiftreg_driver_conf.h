/**
 * @brief Provides an interface to the shift register rtdm driver.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
 *
 * Contains the IOCTL definitions and device paths to use the rtdm
 * shift register driver. Only used by rpi_shiftreg_gpio to initialize
 * and operate the rtdm driver.
 */

#ifndef RPI_SHIFTREG_DRIVER_CONF_H
#define RPI_SHIFTREG_DRIVER_CONF_H

#include <cstdint>

#define SHIFTREG_DRIVER_IOC_MAGIC     's'

// Run time IOCTLS

// IO control to tell the driver to start the RT shiftregister and adc sampling
#define SHIFTREG_DRIVER_START_RT_TASK     _IO(SHIFTREG_DRIVER_IOC_MAGIC, 1)

// IO control to tell the driver to stop the RT task
#define SHIFTREG_DRIVER_STOP_RT_TASK      _IO(SHIFTREG_DRIVER_IOC_MAGIC, 2)

// Blocking IO control call to wait on the drivers signal to proceed.
#define SHIFTREG_DRIVER_WAIT_ON_RT_TASK   _IO(SHIFTREG_DRIVER_IOC_MAGIC, 3)

#define SHIFTREG_DEVICE_NAME   "/dev/rtdm/shiftreg_rtdm"
#define SHIFTREG_MODULE_PARAMETERS_PATH    "/sys/module/shiftreg_rtdm/parameters"

#endif // RPI_SHIFTREG_DRIVER_CONF_H