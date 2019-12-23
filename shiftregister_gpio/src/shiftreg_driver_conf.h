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
 * @brief Provides an interface to the shift register rtdm driver.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Contains the IOCTL definitions and device paths to use the rtdm
 * shift register driver. Only used by shiftreg_gpio to initialize
 * and operate the rtdm driver.
 */
#ifndef SHIFTREG_DRIVER_CONF_H
#define SHIFTREG_DRIVER_CONF_H

#define SHIFTREG_DRIVER_IOC_MAGIC     's'

// Run time IOCTLS

// IO control to tell the driver to start the RT shiftregister and adc sampling
#define SHIFTREG_DRIVER_START_RT_TASK     _IO(SHIFTREG_DRIVER_IOC_MAGIC, 1)

// IO control to tell the driver to stop the RT task
#define SHIFTREG_DRIVER_STOP_RT_TASK      _IO(SHIFTREG_DRIVER_IOC_MAGIC, 2)

// Blocking IO control call to wait on the drivers signal to proceed.
#define SHIFTREG_DRIVER_WAIT_ON_RT_TASK   _IO(SHIFTREG_DRIVER_IOC_MAGIC, 3)

// IO control to set the driver's shiftregister sampling period
#define SHIFTREG_DRIVER_SET_TICK_PERIOD   _IOW(SHIFTREG_DRIVER_IOC_MAGIC, 4, int)

#define SHIFTREG_DEVICE_NAME   "/dev/rtdm/shiftreg_rtdm"
#define SHIFTREG_MODULE_PARAMETERS_PATH    "/sys/module/shiftreg_rtdm/parameters"

#endif // SHIFTREG_DRIVER_CONF_H