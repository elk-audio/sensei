/**
 * @brief Value messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Declaration of Command Message types using macro facilities in message/command_base.h
 *
 * Define in this file all concrete message types, possibly using the macro
 *
 * SENSEI_DECLARE_MESSAGE(ClassName, command_tag, InternalType, representation_prefix, destination)
 *
 * Classes defined here have to be instantiated with factory methods provide by message_factory.h
 *
 */
#ifndef SENSEI_COMMAND_DEFS_H
#define SENSEI_COMMAND_DEFS_H

#include <vector>

#include "base_command.h"

namespace sensei {

/**
 * @brief Tags used for RTTI emulation.
 */
enum class CommandType
{
    // External Commands
    SET_PIN_TYPE,
    SET_VIRTUAL_PIN,
    SET_ENABLED,
    SET_SENDING_MODE,
    SET_SENDING_DELTA_TICKS,
    SET_ADC_BIT_RESOLUTION,
    SET_LOWPASS_FILTER_ORDER,
    SET_LOWPASS_CUTOFF,
    SET_SLIDER_MODE_ENABLED,
    SET_SLIDER_THRESHOLD,
    SEND_DIGITAL_PIN_VALUE,
    ENABLE_SENDING_PACKETS,
    // Imu specific commands
    SET_IMU_ENABLED,
    SET_IMU_FILTER_MODE,
    SET_IMU_ACC_RANGE_MAX,
    SET_IMU_GYRO_RANGE_MAX,
    SET_IMU_COMPASS_RANGE_MAX,
    SET_IMU_COMPASS_ENABLED,
    SET_IMU_SENDING_MODE,
    SET_IMU_SENDING_DELTA_TICKS,
    SET_IMU_DATA_MODE,
    SET_IMU_ACC_THRESHOLD,
    IMU_CALIBRATE,
    IMU_FACTORY_RESET,
    IMU_REBOOT,
    IMU_GET_TEMPERATURE,
    // Internal Commands
    SET_INVERT_ENABLED,
    SET_INPUT_SCALE_RANGE_LOW,
    SET_INPUT_SCALE_RANGE_HIGH,
    // Output Backend Commands
    SET_BACKEND_TYPE,
    SET_PIN_NAME,
    SET_SEND_OUTPUT_ENABLED,
    SET_SEND_RAW_INPUT_ENABLED,
    SET_OSC_OUTPUT_BASE_PATH,
    SET_OSC_OUTPUT_RAW_PATH,
    SET_OSC_OUTPUT_HOST,
    SET_OSC_OUTPUT_PORT,
    SET_OSC_INPUT_PORT,
    N_COMMAND_TAGS
};

/**
 * @brief Pin type configuration
 */
enum class PinType
{
    DIGITAL_INPUT,
    DIGITAL_OUTPUT,
    ANALOG_INPUT,
    IMU_INPUT,
    UNDEFINED,
    N_PIN_TYPES
};

/**
 * @brief Index for pins to Imu parameters
 */
enum ImuIndex
{
    YAW,
    PITCH,
    ROLL,
    N_IMU_INDEXES
};

/**
 * @brief Backend Types
 */
enum class BackendType
{
    NONE,
    OSC,
    STD_STREAM,
    N_BACKEND_TYPES
};

/**
 * @brief Sending behaviours
 */
enum class SendingMode
{
    OFF,
    CONTINUOUS,
    ON_VALUE_CHANGED,
    ON_REQUEST,
    N_SENDING_MODES
};

enum class CommandErrorCode
{
    OK,
    UNHANDLED_COMMAND_FOR_SENSOR_TYPE,
    INVALID_VALUE,
    INVALID_RANGE,
    CLIP_WARNING,
    INVALID_PIN_INDEX,
    INVALID_URL,
    UNINITIALIZED_PIN,
    INVALID_PORT_NUMBER,
    N_COMMAND_ERROR_CODES
};

////////////////////////////////////////////////////////////////////////////////
// Concrete command subclasses definitions
////////////////////////////////////////////////////////////////////////////////

// External commands

SENSEI_DECLARE_COMMAND(SetEnabledCommand,
                       CommandType::SET_ENABLED,
                       bool,
                       "Set Enabled",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetPinTypeCommand,
                       CommandType::SET_PIN_TYPE,
                       PinType,
                       "Set Pin Type",
                       CommandDestination::SERIAL_FRONTEND
                           | CommandDestination::INTERNAL
                           | CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetVirtualPinCommand,
                       CommandType::SET_VIRTUAL_PIN,
                       ImuIndex,
                       "Set virtual pin",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetSendingModeCommand,
                       CommandType::SET_SENDING_MODE,
                       SendingMode,
                       "Set Sending Mode",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetSendingDeltaTicksCommand,
                       CommandType::SET_SENDING_DELTA_TICKS,
                       int,
                       "Set Sending Delta Ticks",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetADCBitResolutionCommand,
                       CommandType::SET_ADC_BIT_RESOLUTION,
                       int,
                       "Set ADC Bit Resolution",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetLowpassFilterOrderCommand,
                       CommandType::SET_LOWPASS_FILTER_ORDER,
                       int,
                       "Set Lowpass Filter order",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetLowpassCutoffCommand,
                       CommandType::SET_LOWPASS_CUTOFF,
                       float,
                       "Set Lowpass Cutoff",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetSliderModeEnabledCommand,
                       CommandType::SET_SLIDER_MODE_ENABLED,
                       bool,
                       "Set Slider Mode Enabled",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetSliderThresholdCommand,
                       CommandType::SET_SLIDER_THRESHOLD,
                       int,
                       "Set Slider Threshold",
                       CommandDestination::SERIAL_FRONTEND | CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SendDigitalPinValueCommand,
                       CommandType::SEND_DIGITAL_PIN_VALUE,
                       bool,
                       "Send Digital Pin Value",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(EnableSendingPacketsCommand,
                       CommandType::ENABLE_SENDING_PACKETS,
                       bool,
                       "Enable Sending Packets",
                       CommandDestination::SERIAL_FRONTEND);

// IMU specific commands

SENSEI_DECLARE_COMMAND(SetImuEnabledCommand,
                       CommandType::SET_IMU_ENABLED,
                       bool,
                       "Enable IMU",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuFilterModeCommand,
                       CommandType::SET_IMU_FILTER_MODE,
                       int,
                       "Set IMU filter mode",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuAccelerometerRangeMaxCommand,
                       CommandType::SET_IMU_ACC_RANGE_MAX,
                       int,
                       "Set IMU Accelerometer max range",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuGyroscopeRangeMaxCommand,
                       CommandType::SET_IMU_GYRO_RANGE_MAX,
                       int,
                       "Set IMU Gyroscope max range",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuCompassRangeMaxCommand,
                       CommandType::SET_IMU_COMPASS_RANGE_MAX,
                       float,
                       "Set IMU compass max range",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuCompassEnabledCommand,
                       CommandType::SET_IMU_COMPASS_ENABLED,
                       bool,
                       "Enable IMU compass",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuSendingModeCommand,
                       CommandType::SET_IMU_SENDING_MODE,
                       SendingMode,
                       "Set IMU Sending Mode",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuSendingDeltaTicksCommand,
                       CommandType::SET_IMU_SENDING_DELTA_TICKS,
                       int,
                       "Set IMU Sending Delta Ticks",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuDataModeCommand,
                       CommandType::SET_IMU_DATA_MODE,
                       int,
                       "Set IMU type of data sent",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(SetImuAccThresholdCommand,
                       CommandType::SET_IMU_ACC_THRESHOLD,
                       float,
                       "Set IMU acceleration threshold for sending data in on_value_changed mode",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(ImuCalibrateCommand,
                       CommandType::IMU_CALIBRATE,
                       int,
                       "Initiate self calibration of IMU gyroscope",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(ImuFactoryResetCommand,
                       CommandType::IMU_FACTORY_RESET,
                       int,
                       "Reset the IMU to it's factory settings",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(ImuRebootCommand,
                       CommandType::IMU_REBOOT,
                       int,
                       "Initiate a reboot of the IMU",
                       CommandDestination::SERIAL_FRONTEND);

SENSEI_DECLARE_COMMAND(ImuGetTemperatureCommand,
                       CommandType::IMU_GET_TEMPERATURE,
                       int,
                       "Get the IMU temperature",
                       CommandDestination::SERIAL_FRONTEND);

// Internal commands

SENSEI_DECLARE_COMMAND(SetInvertEnabledCommand,
                       CommandType::SET_INVERT_ENABLED,
                       bool,
                       "Set Invert Enabled",
                       CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetInputScaleRangeLow,
                       CommandType::SET_INPUT_SCALE_RANGE_LOW,
                       float,
                       "Set Input Scale Range Low",
                       CommandDestination::INTERNAL);

SENSEI_DECLARE_COMMAND(SetInputScaleRangeHigh,
                       CommandType::SET_INPUT_SCALE_RANGE_HIGH,
                       float,
                       "Set Input Scale Range Low",
                       CommandDestination::INTERNAL);

// Output Backend commands

SENSEI_DECLARE_COMMAND(SetBackendTypeCommand,
                       CommandType::SET_BACKEND_TYPE,
                       BackendType,
                       "Set Backend Type",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetPinNameCommand,
                       CommandType::SET_PIN_NAME,
                       std::string,
                       "Set Pin Name",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetSendOutputEnabledCommand,
                       CommandType::SET_SEND_OUTPUT_ENABLED,
                       bool,
                       "Set Output Enabled",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetSendRawInputEnabledCommand,
                       CommandType::SET_SEND_RAW_INPUT_ENABLED,
                       bool,
                       "Set Raw Output Enabled",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetOSCOutputBasePathCommand,
                       CommandType::SET_OSC_OUTPUT_BASE_PATH,
                       std::string,
                       "Set OSC output base path",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetOSCOutputRawPathCommand,
                       CommandType::SET_OSC_OUTPUT_RAW_PATH,
                       std::string,
                       "Set OSC output raw path",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetOSCOutputHostCommand,
                       CommandType::SET_OSC_OUTPUT_HOST,
                       std::string,
                       "Set OSC output host",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetOSCOutputPortCommand,
                       CommandType::SET_OSC_OUTPUT_PORT,
                       int,
                       "Set OSC output port",
                       CommandDestination::OUTPUT_BACKEND);

SENSEI_DECLARE_COMMAND(SetOSCInputPortCommand,
                       CommandType::SET_OSC_INPUT_PORT,
                       int,
                       "Set OSC input port",
                       CommandDestination::USER_FRONTEND);

////////////////////////////////////////////////////////////////////////////////
// Container specifications
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Preferred container of commands used for inter-module communications
 */
typedef std::vector<std::unique_ptr<BaseMessage>> CommandContainer;

/**
 * @brief Iterator used in function interfaces
 */
typedef std::back_insert_iterator<CommandContainer> CommandIterator;

}; // namespace sensei

#endif //SENSEI_COMMAND_DEFS_H
