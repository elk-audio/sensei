#ifndef MANAGEIMU_H
#define MANAGE_IMU_H

#include "common.h"

//SPI
#define SPI_MAX_TRANSFERS_PER_CMD 50000

#define START_SPI_DATA_TRANSFER 0xF6
#define START_NO_RESP_HEADER 0xF7
#define START_RESP_HEADER 0xF9
#define SPI_CLEAR_BUFFER 0x01
#define SPI_IDLE_STATE 0x00
#define SPI_READY_STATE 0x01
#define SPI_BUSY_STATE 0x02
#define SPI_ACC_STATE 0x04

// Delays
#define DELAY_SPI_YEI 5 //[us]

// Commands
#define NO_SLOT	255
#define READ_TARED_ORIENTATION_AS_QUATERNION 0
#define READ_TARED_ORIENTATION_AS_EULER_ANGLES 1
#define READ_TARED_ORIENTATION_AS_ROTATION_MATRIX 2
#define READ_UNTARED_ORIENTATION_AS_QUATERNION 6
#define READ_UNTARED_ORIENTATION_AS_EULER_ANGLES 7
#define READ_NORMALIZED_ACCELEROMETER_VECTOR  34
#define READ_CORRECTED_ACCELEROMETER_VECTOR   39
#define READ_RAW_GYROSCOPE_VECTOR 65
#define READ_RAW_ACCELEROMETER_VECTOR 66
#define	READ_RAW_COMPASS_VECTOR 67
#define READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA 37
#define READ_ALL_NORMALIZED_COMPONENT_SENSOR_DATA 32
#define READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE 41

#define CMD_SET_AXIS_DIRECTION 116
#define CMD_GET_AXIS_DIRECTION 143
#define CMD_RESET_BASE_OFFSET 20
#define CMD_SET_DECOMPOSITION_ORDER 16
#define CMD_GET_DECOMPOSITION_ORDER 156

#define CMD_OFFSET_WITH_CURRENT_ORIENTATION 19
#define CMD_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION 22
#define CMD_GET_INTERRUPT_STATUS 31
#define CMD_GET_TARED_ORIENTATION_AS_ROTATION_MATRIX 129

#define CMD_SET_STREAMING_SLOT 80
#define CMD_SET_STREAMING_TIMING 82
#define CMD_START_STREAMING 85
#define CMD_STOP_STREAMING 86
#define CMD_TARE_WITH_CURRENT_ORIENTATION 96
#define CMD_SET_COMPASS_ENABLE 109
#define CMD_RESET_FILTER 120
#define CMD_SET_ACCELEROMETER_RANGE 121
#define CMD_SET_FILTER_MODE 123
#define CMD_SET_GYROSCOPE_RANGE 125
#define CMD_SET_COMPASS_RANGE 126
#define CMD_SET_PIN_MODE 29

#define CMD_GET_COMPASS_ENABLED_STATE 142
#define CMD_GET_ACCELEROMETER_RANGE 148
#define CMD_GET_FILTER_MODE 152
#define CMD_GET_GYROSCOPE_RANGE 154
#define CMD_GET_COMPASS_RANGE 155
#define CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION 165
#define CMD_SET_CALIBRATION_MODE 169
#define CMD_RESPONSE_HEADER_BITFIELD 221
#define CMD_SET_UART_BAUD_RATE 231
#define CMD_GET_UART_BAUD_RATE 232
#define CMD_SET_REFERENCE_VECTOR_MODE 105
#define GET_CALIBRATION_MODE 170 //(set=169)
#define GET_REFERENCE_VECTOR_MODE 135//(set=105)
#define CMD_SOFTWARE_RESET 226

//Settings
#define AXIS_DIRECTION_RIGHT_FORWARD_UP_RHANDED 1
#define DECOMPOSITION_ZYX 3

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
    void clearBuffer();
    int32_t getAllComponents(ImuComponents* _components);

    int32_t writeCommand(uint8_t cmd);
    int32_t writeCommand(uint8_t cmd, uint8_t value);
    int32_t writeCommand(uint8_t cmd, uint8_t* value, uint8_t nByte);

    int32_t getData(uint8_t cmd, void* data, uint16_t nByte);
    int32_t getValue(uint8_t cmd, void* data);
    int32_t resetFilter();

    int32_t setSettings();
    int32_t setSettings(sImuSettings* _settings);

    int32_t getSettings(sImuSettings* _settings);
    int32_t setInterruptMode();

private:
    int32_t _initialize();
    bool _isInitialized;
    sImuSettings _settings;
    sImuInternalSettings _internalSettings;
};

#endif // MANAGE_IMU_H
