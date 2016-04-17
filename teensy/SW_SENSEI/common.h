#ifndef COMMON_H
#define COMMON_H

#include "Arduino.h"
#include "FreeRTOS_ARM.h"
#include "utility/queue.h"
#include "sensei_serial_protocol.h"
#include "SPI.h"
#include "arm_functions.h"
#include <vector>

using namespace sensei;
using namespace std;

const uint8_t CHANNELS_PER_MULTIPLEXER=16;
const uint8_t CHANNELS_PER_SHIFT_REGISTER=8;
const uint16_t STACK_SIZE=256;

#undef F
#define F(str) str

#define SerialDebug Serial1

const uint8_t DEBUG=0;

typedef union uByte{
	struct {
		uint8_t bit0 : 1;
		uint8_t bit1 : 1;
		uint8_t bit2 : 1;
		uint8_t bit3 : 1;
		uint8_t bit4 : 1;
		uint8_t bit5 : 1;
		uint8_t bit6 : 1;
		uint8_t bit7 : 1;
	};
	uint8_t value;
} Byte;

// FreeRTOS
typedef enum RT_MSG_TYPE
{
	ACK=0,
	DATA=1,
	VALUE=2
} RT_MSG_TYPE;

typedef struct ImuComponents
{
	sImuQuaternion quaternion;
    sImuComponentSensor componentSensor;
    sImuLinearAcceleration linearAcceleration;
	//sImuNormalizedComponentSensor normalizedComponentSensor;
} __attribute__((packed)) ImuComponents;


typedef struct SetupPin
{
	uint16_t idxPin;
	FilterType FsFilter;
	//---------------------------
	uint8_t sendingMode;
	uint16_t deltaTicksContinuousMode;
	uint8_t ADCBitResolution;
	uint8_t filterOrder;
	FilterType* filterCoeff_a;
	FilterType* filterCoeff_b;
	uint8_t sliderMode;
	uint16_t sliderThreshold;
} __attribute__((packed)) SetupPin;


typedef struct GetSetPin
{
	uint16_t idx;
	uint16_t value;
	uint8_t type;
}   __attribute__((packed)) GetSetPin;

typedef  struct HardwareSettings
{
	uint8_t ticksDelay;
	uint16_t nPin;
	uint16_t nDigitalPin;
}   __attribute__((packed)) HardwareSettings;

typedef  struct SystemSettings
{
	uint16_t nPin;
	uint16_t nDigitalPin;
	bool debugMode;
	bool enableMultiplePackets;
	bool enableSendingPackets;
}   __attribute__((packed)) SystemSettings;

typedef struct TaskRtStatus
{
    uint64_t nCycles;
    int32_t  controlLoopDelay; //[us]
    uint32_t msgQueueReceived;
    uint16_t msgQueueSendErrors;
	uint16_t nCyclesExpired;
	uint16_t nCyclesCloseToExpiration;
	uint16_t lastTocTask;
} __attribute__((packed)) TaskRtStatus;

typedef struct TaskComStatus
{
    uint64_t nCycles;
    uint32_t msgQueueReceived;
    uint16_t msgQueueSendErrors;
	uint16_t nCyclesCloseToExpiration;
	uint16_t lastTocTask;
} __attribute__((packed)) TaskComStatus;

typedef union Data
{
	SystemSettings systemSettings;
	HardwareSettings hw;
	TaskRtStatus taskRtStatus;
	TaskComStatus taskComStatus;
	SetupPin setupPin;
	GetSetPin pin;
	uint8_t value;
} Data;

typedef struct MsgRTtoCOM_IMU
{
	uint8_t msgType;
	int32_t status;
	ImuComponents imu;
} __attribute__((packed)) MsgRTtoCOM_IMU;

typedef struct MsgRTtoCOM_PIN
{
	GetSetPin pin;
} __attribute__((packed)) MsgRTtoCOM_PIN;

typedef struct Msg_DATA
{
	uint8_t cmd;
	uint8_t sub_cmd;
	uint32_t timestamp;
	uint8_t msgType;
	int32_t status;
	Data data;
} __attribute__((packed)) Msg_DATA;


// FreeRTOS
//const uint16_t MSG_QUEUE_ITEM_SIZE = 20;
const uint16_t MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE = 0;
const uint16_t MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_RT_TO_COM = 0;
const uint16_t MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_COM_TO_RT = 0;

//Hardware
const uint8_t STATUS_LED = 13;

const uint8_t VERSOR_Z_PINS = -1;

const uint8_t Z1 = 23;
const uint8_t Z2 = 22;
const uint8_t Z3 = 21;
const uint8_t Z4 = 20;

const uint8_t S0 = 2;
const uint8_t S1 = 3;
const uint8_t S2 = 4;
const uint8_t S3 = 5;

const uint8_t DS = 6;
const uint8_t ST = 7;
const uint8_t SH = 8;

const uint8_t SPI_SS = 10;
const uint8_t SPI_SCK = 14;
const uint8_t SPI_MISO = 12;
const uint8_t SPI_MOSI = 11;
const uint8_t INT_FILTER = 9;

#endif // COMMON_H
