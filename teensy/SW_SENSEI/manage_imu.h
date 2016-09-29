#ifndef MANAGEIMU_H
#define MANAGE_IMU_H

#include "common.h"

#define SPI_MAX_ITER_WAIT_CMD 4000
#define DELAY_SPI_YEI 2 //[us]
#define DELAY_SPI_YEI_CLEAR_BUFFER 75 //[us]

// Default settings
#define AXIS_DIRECTION_RIGHT_UP_FORWARD_LHANDED 0
#define AXIS_DIRECTION_RIGHT_FORWARD_UP_RHANDED 1
#define DECOMPOSITION_ZYX 3

typedef enum SPI_CMD
{
    START_DATA_TRANSFER = 0xF6,
    DATA_BYTE = 0xFF,
    CLEAR_BUFFER = 0x01,

    IDLE_STATE = 0x00,
    READY_STATE = 0x01,
    BUSY_STATE = 0x02,
    ACC_STATE = 0x04,
} SPI_CMD;

typedef enum CMD_IMU
{
    NO_SLOT = 255,

    READ_TARED_ORIENTATION_AS_QUATERNION = 0,
    READ_TARED_ORIENTATION_AS_EULER_ANGLES = 1,
    READ_TARED_ORIENTATION_AS_ROTATION_MATRIX = 2,
    READ_UNTARED_ORIENTATION_AS_QUATERNION = 6,
    READ_UNTARED_ORIENTATION_AS_EULER_ANGLES = 7,
    READ_NORMALIZED_ACCELEROMETER_VECTOR = 34,
    READ_CORRECTED_ACCELEROMETER_VECTOR = 39,
    READ_RAW_GYROSCOPE_VECTOR = 65,
    READ_RAW_ACCELEROMETER_VECTOR = 66,
    READ_RAW_COMPASS_VECTOR = 67,
    READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA = 37,
    READ_ALL_NORMALIZED_COMPONENT_SENSOR_DATA = 32,
    READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = 41,

    SET_AXIS_DIRECTION = 116,
    GET_AXIS_DIRECTION = 143,
    SET_DECOMPOSITION_ORDER = 16,
    GET_DECOMPOSITION_ORDER = 156,

    OFFSET_WITH_CURRENT_ORIENTATION = 19,
    SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION = 22,
    GET_INTERRUPT_STATUS = 31,
    GET_TARED_ORIENTATION_AS_ROTATION_MATRIX = 129,

    SET_STREAMING_SLOT = 80,
    SET_STREAMING_TIMING = 82,
    START_STREAMING = 85,
    STOP_STREAMING = 86,
    RESET_BASE_OFFSET = 20,
    TARE_WITH_CURRENT_ORIENTATION = 96,
    SET_COMPASS_ENABLE = 109,
    RESET_FILTER = 120,
    SET_ACCELEROMETER_RANGE = 121,
    SET_FILTER_MODE = 123,
    SET_GYROSCOPE_RANGE = 125,
    SET_COMPASS_RANGE = 126,
    SET_PIN_MODE = 29,

    GET_COMPASS_ENABLED_STATE = 142,
    GET_ACCELEROMETER_RANGE = 148,
    GET_FILTER_MODE = 152,
    GET_GYROSCOPE_RANGE = 154,
    GET_COMPASS_RANGE = 155,
    BEGIN_GYROSCOPE_AUTOCALIBRATION = 165,
    SET_CALIBRATION_MODE = 169,
    RESPONSE_HEADER_BITFIELD = 221,
    SET_UART_BAUD_RATE = 231,
    GET_UART_BAUD_RATE = 232,
    SET_REFERENCE_VECTOR_MODE = 105,
    GET_CALIBRATION_MODE = 170,
    GET_REFERENCE_VECTOR_MODE = 135,

    GET_TEMPERATURE = 43,

    RESTORE_DEFAULT_SETTINGS = 224,
    SOFTWARE_RESET = 226
} CMD_IMU;

typedef enum eImuCalibrationMode {
    IMU_CALIBRATION_MODE_BIAS=0,
    IMU_CALIBRATION_MODE_BIAS_SCALE,
    IMU_CALIBRATION_MODE_ORTHO,
    N_IMU_CALIBRATION_MODE
} eImuCalibrationMode;

typedef enum eImuReferenceVector {
    IMU_REFERENCE_VECTOR_SINGLE_STATIC_MODE = 0,
    IMU_EFERENCE_VECTOR_SINGLE_AUTO_MODE,
    IMU_REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE,
    N_IMU_REFERENCE_VECTOR
} eImuReferenceVector;

typedef struct sImuInternalSettings
{
    uint8_t decompositionOrder;
    uint8_t axisDirection;
    uint8_t calibrationMode;
    uint8_t referenceVectorMode;
} sImuInternalSettings;

class ManageIMU {
public:
    ManageIMU();
    ~ManageIMU();

    int32_t sendCommand(uint8_t cmd);
    int32_t sendCommand(uint8_t cmd, uint8_t value);
    int32_t sendCommand(uint8_t cmd, uint8_t* value, uint8_t nByte);
    int32_t sendCommandWithoutChecks(uint8_t cmd);

    int32_t getValue(uint8_t cmd, void* data);
    int32_t getValue(uint8_t cmd, float* data);
    int32_t getData(uint8_t cmd, void* data, uint16_t nByte);

    bool getInterruptStatus();
    int32_t setInterruptMode();

    int32_t getSensorComponents(uint8_t components, uint8_t* data_vector, uint16_t& packetSize);

    int32_t resetFilter();

    int32_t setSettings();
    int32_t setSettings(sImuSettings* _settings);

    int32_t getSettings(sImuSettings* _settings);

    int32_t gyroscopeCalibration();
    int32_t tareWithCurrentOrientation();

    float getMinLinearAccelerationSquareNorm();
    uint16_t getDeltaTicksContinuousMode();
    uint8_t getTypeOfData();
    uint8_t getSendingMode();

    int32_t resetToFactorySettings();
    int32_t reboot();
    void printDebugImuSettings();

    //int32_t resetTare();

private:
    int32_t _initialize();

    void _clearBuffer();
    void _startCommunication();
    void _sendCmd(uint8_t cmd);
    uint8_t _readByte();
    void _writeByte(uint8_t value);
    int32_t _waitStatus(uint8_t status);

    bool _isInitialized;
    uint8_t _retSpi;
    sImuSettings _settings;
    sImuInternalSettings _internalSettings;
};

#endif // MANAGE_IMU_H
