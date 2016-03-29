/**
 * @file sensei_serial_protocol.h
 * @brief Serial Protocol data types
 * @author Simone Minto
 * @copyright MIND Music Labs AB, Stockholm
 * @date 2016-03-10
 */

#ifndef SENSEI_SERIAL_PROTOCOL_H_
#define SENSEI_SERIAL_PROTOCOL_H_

namespace sensei {

typedef enum SENSEI_ERROR_CODE {
	OK = 0,
	START_HEADER_NOT_PRESENT = -1,
	STOP_HEADER_NOT_PRESENT = -2,
	CRC_NOT_CORRECT = -3,
    N_SENSEI_ERROR_CODES
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

typedef struct sSenseiDataPacket
{
	PACKET_HEADER start_header;
	uint8_t       cmd;
	uint8_t       sub_cmd;
	char          data[SENSEI_PAYLOAD_LENGTH];
	uint8_t       continuation;
	uint32_t      timestamp;
	uint16_t      crc;
	PACKET_HEADER stop_header;
} __attribute__((packed)) sDataPacket;


typedef enum SENSEI_CMD {
	//----------------------------------------------
	HELLO = 1,
	SERIAL_DEBUG = 2,
	//----------------------------------------------
	SET_SAMPLING_RATE = 100,
	CONFIGURE_PIN = 101,
	GET_VALUE = 102,
	GET_ALL_VALUES = 103,
	SET_VALUE = 104,
	SET_DIGITAL_PINS = 105,
	//----------------------------------------------
	IMU_GET_SETTINGS = 200,
	IMU_SET_SETTINGS = 201,
	IMU_GYROSCOPE_CALIBRATION = 202,
	IMU_RESET_FILTER = 203,
	CMD_IMU_GET_DATA = 204,
	//----------------------------------------------
	ACK = 255
	//----------------------------------------------

} SENSEI_CMD;

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
} __attribute__((packed)) sImuSettings;

typedef struct sImuComponentSensor
{
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

typedef struct sImuLinearAcceleration
{
	float laz;
	float lay;
	float lax;
} __attribute__((packed)) sImuLinearAcceleration;


typedef struct sImuQuaternion
{
	float qw;
	float qz;
	float qy;
	float qx;
} __attribute__((packed)) sImuQuaternion;

typedef enum eImuGetData
{
	IMU_GET_ALL_DATA,
	IMU_GET_DATA_COMPONENT_SENSOR,
	IMU_GET_DATA_COMPONENT_SENSOR_NORMALIZED,
	IMU_GET_DATA_QUATERNION,
	IMU_GET_DATA_LINEARACCELERATION,
	IMU_GET_DATA_QUATERNION_LINEARACCELERATION,
	N_IMU_DATA_TYPES
} eImuGetData;

typedef enum ePinType{
	PIN_UNUSED,
	PIN_ANALOG_INPUT,
	PIN_DIGITAL_INPUT,
	PIN_DIGITAL_OUTPUT,
	N_PIN_TYPES
} ePinType;

typedef enum ePinSendingMode{
	PIN_SENDING_MODE_OFF,
	PIN_SENDING_MODE_CONTINUOUS,
	PIN_SENDING_MODE_ON_VALUE_CHANGED,
	PIN_SENDING_MODE_ON_REQUEST,
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
	uint8_t pinType;
	uint16_t idxPin;
	ePinSendingMode sendingMode;
	uint16_t deltaTicksContinuousMode;
	float lowPassCutOffFilter;
	uint8_t sliderMode;
	uint16_t sliderThreshold;
} __attribute__((packed))  sPinConfiguration;

}; // namespace sensei

#endif // #ifndef SENSEI_SERIAL_PROTOCOL_H_ 
