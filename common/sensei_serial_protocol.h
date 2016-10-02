/**
 * @file sensei_serial_protocol.h
 * @brief Serial Protocol data types
 * @author Simone Minto
 * @copyright MIND Music Labs AB, Stockholm
 * @date 2016-03-10
 */

#ifndef SENSEI_SERIAL_PROTOCOL_H_
#define SENSEI_SERIAL_PROTOCOL_H_

#define PRINT_IMU_DEBUG
//#define RESET_IMU_FACTORY_SETTINGS_ON_INITIALIZATION

#include <cstdint>

namespace sensei {

    typedef float FilterType;

    const FilterType DEFAULT_RT_FREQUENCY = 1000.0; //[Hz]

    #ifdef PRINT_IMU_DEBUG
    const float FREQUENCY_DEBUG_IMU = 2; //[Hz]
    #endif

    const uint8_t MAX_FILTER_ORDER = 12;

    // Start conditions
    const bool COND_DEBUG_MODE = false;
    const bool COND_MULTIPLE_PACKETS = false;
    const bool COND_SENDING_PACKETS = false;
    const bool COND_IMU_ENABLED = false;

    typedef enum SENSEI_ERROR_CODE {

        NO_EXTERNAL_PROCESSING_NECESSARY = 1,
        //------------------------------------------
        OK = 0,
        //------------------------------------------
        START_HEADER_NOT_PRESENT = -1,
        STOP_HEADER_NOT_PRESENT = -2,
        CRC_NOT_CORRECT = -3,
        CMD_NOT_VALID = -4,
        SUB_CMD_NOT_VALID = -5,
        CMD_NOT_PROCESSED = -6,
        DIGITAL_OUTPUT_IDX_BANK_NOT_VALID = -7,
        DIGITAL_OUTPUT_IDX_PIN_NOT_VALID = -8,
        IDX_PIN_NOT_VALID=-9,
        PIN_TYPE_NOT_VALID=-10,
        CMD_NOT_ALLOWED=-11,
        INCORRECT_NUMBER_OF_PINS=-12,
        INCORRECT_NUMBER_OF_DIGITAL_PINS=-13,
        SYSTEM_NOT_INITIALIZED=-14,
        WRONG_FILTER_SETTINGS=-15,
        //------------------------------------------
        TIMEOUT_ON_RESPONSE = -100,
        INCORRECT_PAYLOAD_SIZE = -101,
        NO_AFFINITY_WITH_RESPONSE_PACKET = -102,
        CMD_NOT_EXPECTED=-103,
        INCORRECT_PARAMETERS_NUMBER=-104,
        INCORRECT_PARAMETER_TYPE = -105,
        INCOMPLETE_PARAMETERS=-106,
        WRONG_NUMBER_EXPECTED_RESPONSE_PACKETS=-107,
        //------------------------------------------
        IMU_GENERIC_ERROR = -3000,
        IMU_COMMUNICATION_ERROR = -3001,
        IMU_NOT_CONNECTED = -3002,
        IMU_CMD_NOT_EXECUTED = -3003,
        IMU_DISABLED = -3004,
        //------------------------------------------
        SERIAL_DEVICE_GENERIC_ERROR = -4000,
        SERIAL_DEVICE_PORT_NOT_OPEN = -4001,
		    SERIAL_DEVICE_ERROR_ON_WRITING = -4002,
        //------------------------------------------
        PINS_CONFIGURATION_PARSING_ERROR = -10000,
        //------------------------------------------

        GENERIC_ERROR=-5000

    } SENSEI_ERROR_CODE;

    const uint16_t SENSEI_LENGTH_DATA_PACKET = 64;

    typedef struct PACKET_HEADER {
        uint8_t vByte[3];
    } __attribute__((packed)) PACKET_HEADER;

    const uint16_t SENSEI_PAYLOAD_LENGTH = SENSEI_LENGTH_DATA_PACKET
        - 2 * sizeof(PACKET_HEADER)     // start/stop headers
        - 1                             // cmd
        - 1                             // sub_cmd
        - 1                             // continuation
        - 4                             // timestamp
        - 2                             // crc
        ;

    const PACKET_HEADER START_HEADER = { 1, 2, 3 };
    const PACKET_HEADER STOP_HEADER = { 4, 5, 6 };

    typedef struct sSenseiDataPacket
    {
        PACKET_HEADER start_header;
        uint8_t       cmd;
        uint8_t       sub_cmd;
        char          payload[SENSEI_PAYLOAD_LENGTH];
        uint8_t       continuation;
        uint32_t      timestamp;
        uint16_t      crc;
        PACKET_HEADER stop_header;
    } __attribute__((packed)) sSenseiDataPacket;

    const uint16_t SENSEI_START_IDX_CRC = sizeof(PACKET_HEADER); //start-header
    const uint16_t SENSEI_STOP_IDX_CRC = SENSEI_LENGTH_DATA_PACKET
        - sizeof(PACKET_HEADER) //stop-header
        - 2 //crc
        - 1  //index
        ;

    typedef struct sSenseiACKPacket
    {
        int32_t  status;
        uint32_t timestamp;
        uint8_t  cmd;
        uint8_t  sub_cmd;

    } __attribute__((packed)) sSenseiACKPacket;

    typedef struct sSenseiStatus
    {
        uint64_t taskCyclesRt;
        int32_t  controlLoopDelay; //[us]
        uint32_t msgQueueReceivedRtTask;
        uint16_t msgQueueSendErrorsRtTask;
        uint16_t nCyclesExpired;
        uint16_t nCyclesCloseToExpiration;
        uint16_t lastTocRtTask;

        uint64_t taskCyclesCom;
        uint32_t nPacketReceived;
        uint32_t msgQueueReceivedComTask;
        uint16_t msgQueueSendErrorsComTask;

    } __attribute__((packed)) sSenseiStatus;


    typedef enum SENSEI_CMD {
        //----------------------------------------------
        INITIALIZE_SYSTEM = 0,
        HELLO = 1,
        SERIAL_DEBUG = 2,
        GET_SYSTEM_STATUS=3,
		    //----------------------------------------------
		    ENABLE_SENDING_PACKETS=10,
        ENABLE_MULTIPLE_PACKETS=11,
        //----------------------------------------------
        CONFIGURE_PIN = 100,
        GET_PINS_CONFIGURATION=101,
        GET_VALUE = 102,
        GET_ALL_VALUES = 103,
        SET_VALUE = 104,
        SET_DIGITAL_PINS = 105,
        //----------------------------------------------
		    IMU_ENABLE=200,
        IMU_SET_SETTINGS = 201,
        IMU_GET_SETTINGS = 202,
        IMU_GYROSCOPE_CALIBRATION = 203,
        IMU_RESET_FILTER = 204,
        IMU_GET_DATA = 205,
        IMU_TARE_WITH_CURRENT_ORIENTATION = 206,
        IMU_RESET_TO_FACTORY_SETTINGS = 207,
        IMU_REBOOT = 208,
        IMU_GET_TEMPERATURE = 209,
        //----------------------------------------------
        STOP_BOARD = 250,
        //----------------------------------------------
        VALUE_IMU = 253,
        VALUE = 254,
        //----------------------------------------------
        ACK = 255
        //----------------------------------------------

    } SENSEI_CMD;

    typedef enum SENSEI_SUB_CMD {
        EMPTY = 0,
        //----------------------------------------------
        // SET_DIGITAL_PINS
        //----------------------------------------------
        SET_PIN = 0,
        SET_BANK = 1,

        //----------------------------------------------
        // CONFIGURE_PIN
        //----------------------------------------------
        SET_PIN_DISABLE=0,
        SET_PIN_DIGITAL_INPUT = 1,
        SET_PIN_DIGITAL_OUTPUT = 2,
        SET_PIN_ANALOG_INPUT = 3,

        //----------------------------------------------
        // GET_ALL_VALUES
        //----------------------------------------------
		PINS=0,
		DIGITAL_PIN=1,

        //----------------------------------------------
        // IMU_GET_DATA
        //----------------------------------------------
		GET_ALL_DATA=0,
		GET_DATA_COMPONENT_SENSOR=1,
		GET_DATA_COMPONENT_SENSOR_NORMALIZED=2,
		GET_DATA_QUATERNION=3,
		GET_DATA_LINEARACCELERATION=4,
		GET_DATA_QUATERNION_LINEARACCELERATION=5,

        //----------------------------------------------
        // SET_VALUE
        //----------------------------------------------
		SET_SINGLE_PIN=0,

        //----------------------------------------------
        // GET_VALUE
        //----------------------------------------------
        GET_SINGLE_PIN=0

    } SENSEI_SUB_CMD;

    typedef struct sSystemInitialization {
		uint8_t ticksDelayRtTask;
		uint16_t nPins;
		uint16_t nDigitalPins;
    } __attribute__((packed)) sSystemInitialization;

    typedef enum eSamplingRateTicks {
		SAMPLING_RATE_1000_HZ=1,
		SAMPLING_RATE_500_HZ=2,
		SAMPLING_RATE_333_HZ=3,
		SAMPLING_RATE_250_HZ=4,
		SAMPLING_RATE_200_HZ=5,
		SAMPLING_RATE_125_HZ=8,
		SAMPLING_RATE_100_HZ=10,
		SAMPLING_RATE_50_HZ=20
    } eSamplingRateTicks;

    typedef enum eImuSensorAccelerometerRange {
        IMU_SENSOR_ACCELEROMETER_RANGE_2G,
        IMU_SENSOR_ACCELEROMETER_RANGE_4G,
        IMU_SENSOR_ACCELEROMETER_RANGE_8G,
        N_IMU_SENSOR_ACCELEROMETER_RANGES
    } eImuSensorAccelerometerRange;

    typedef enum eImuSensorGyroscopeRange {
        IMU_SENSOR_GYROSCOPE_RANGE_250,
        IMU_SENSOR_GYROSCOPE_RANGE_500,
        IMU_SENSOR_GYROSCOPE_RANGE_2000,
        N_IMU_SENSOR_GYROSCOPE_RANGES
    } eImuSensorGyroscopeRange;

    typedef enum eImuSensorCompassRange {
        IMU_SENSOR_COMPASS_RANGE_0_88,
        IMU_SENSOR_COMPASS_RANGE_1_30,
        IMU_SENSOR_COMPASS_RANGE_1_90,
        IMU_SENSOR_COMPASS_RANGE_2_50,
        IMU_SENSOR_COMPASS_RANGE_4_00,
        IMU_SENSOR_COMPASS_RANGE_4_70,
        IMU_SENSOR_COMPASS_RANGE_5_60,
        IMU_SENSOR_COMPASS_RANGE_8_10,
        N_IMU_SENSOR_COMPASS_RANGES
    } eImuSensorCompassRange;

    typedef enum eImuFilterType {
        IMU_FILTER_IMU,
        IMU_FILTER_KALMAN,
        IMU_FILTER_Q_COMP,
        IMU_FILTER_Q_GRAD,
        N_IMU_FILTER_TYPES
    } eImuFilterType;

    typedef struct sImuSettings
    {
        uint8_t filterMode;
        uint8_t accerelometerRange;
        uint8_t gyroscopeRange;
        uint8_t compassRange;
        uint8_t compassEnable;
        uint8_t sendingMode;
        uint16_t deltaTicksContinuousMode; //0->REQUEST_MODE
		uint8_t typeOfData; // -> eImuGetData
        float minLinearAccelerationSquareNorm;
    } __attribute__((packed)) sImuSettings;

    //quaternion
    //linearAcceleration
    //component sensor
    //normalized component sensor
    const uint8_t IMU_GET_LINEAR_ACCELERATION = (0x01 << 0);
    const uint8_t IMU_CLEAR_BIT_LINEAR_ACCELERATION = 0xFE;
    const uint8_t IMU_GET_QUATERNIONS = (0x01 << 1);
    const uint8_t IMU_GET_SENSOR_COMPONENTS = (0x01 << 2);
    const uint8_t IMU_GET_NORMALIZED_SENSOR_COMPONENTS = (0x01 << 3);

    const uint8_t IMU_MAX_SUB_CMD = IMU_GET_LINEAR_ACCELERATION | IMU_GET_QUATERNIONS | IMU_GET_SENSOR_COMPONENTS | IMU_GET_NORMALIZED_SENSOR_COMPONENTS;
    const uint8_t IMU_MAX_COMPONENTS = 4;

    typedef struct sImuLinearAcceleration
    {
        // To respect the order in IMU data frame: integer and floating point values coming from the sensor are stored in big-endian format
        float laz;
        float lay;
        float lax;
    } __attribute__((packed)) sImuLinearAcceleration;

    typedef struct sImuQuaternion
    {
        // To respect the order in IMU data frame: integer and floating point values coming from the sensor are stored in big-endian format
        float qw;
        float qz;
        float qy;
        float qx;
    } __attribute__((packed)) sImuQuaternion;

    typedef struct sImuComponentSensor
    {
        // To respect the order in IMU data frame: integer and floating point values coming from the sensor are stored in big-endian format
        float mz;
        float my;
        float mx;

        float az;
        float ay;
        float ax;

        float gz;
        float gy;
        float gx;
    } __attribute__((packed)) sImuComponentSensor;

    typedef struct sImuNormalizedComponentSensor
    {
        // To respect the order in IMU data frame: integer and floating point values coming from the sensor are stored in big-endian format
        float nmz;
        float nmy;
        float nmx;

        float naz;
        float nay;
        float nax;

        float ngz;
        float ngy;
        float ngx;
    } __attribute__((packed)) sImuNormalizedComponentSensor;

    /*typedef enum eImuGetData
    {
        IMU_GET_ALL_DATA,
        IMU_GET_DATA_QUATERNION,
        IMU_GET_DATA_LINEARACCELERATION,
        IMU_GET_DATA_COMPONENT_SENSOR,
        N_IMU_DATA_TYPES
    } eImuGetData;*/


    typedef enum ePinType{
        PIN_DISABLE=0,
        PIN_DIGITAL_INPUT,
        PIN_DIGITAL_OUTPUT,
        PIN_ANALOG_INPUT,
        N_PIN_TYPES
    } ePinType;

    typedef enum eSendingMode{
        SENDING_MODE_ON_REQUEST,
        SENDING_MODE_CONTINUOUS,
        SENDING_MODE_ON_VALUE_CHANGED,
        N_PIN_SENDING_MODE
    } ePinSendingMode;

    typedef enum ePinAdcBitResolution{
        PIN_ADC_RESOLUTION_12_BIT = 0,
        PIN_ADC_RESOLUTION_11_BIT,
        PIN_ADC_RESOLUTION_10_BIT,
        PIN_ADC_RESOLUTION_9_BIT,
        PIN_ADC_RESOLUTION_8_BIT,
        N_ADC_RESOLUTION
    } ePinAdcBitResolution;

    typedef struct sPinConfiguration
    {
        uint16_t idxPin;
        uint8_t sendingMode;
        uint16_t deltaTicksContinuousMode;
        uint8_t ADCBitResolution;
        uint8_t filterOrder;
        float lowPassCutOffFilter;
        uint8_t sliderMode;
        uint16_t sliderThreshold;
    } __attribute__((packed))  sPinConfiguration;

}; // namespace sensei

#endif // SENSEI_SERIAL_PROTOCOL_H
