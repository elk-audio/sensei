"""sensei_protocol_defs.py: enums and constants generated from common/sensei_serial_protocol.h """
__copyright__   = "Copyright (C) 2016 MIND Music Labs"

######################
#  Packet Constants  #
######################

PACKET_SIZE = 64
PAYLOAD_SIZE = 49
CONFIGURE_PIN_PAYLOAD_SIZE = 14
VALUE_PAYLOAD_SIZE = 5
SYSTEM_INITIALIZATION_PAYLOAD_SIZE = 5

EXPECTED_START_HEADER = (1, 2, 3)
EXPECTED_STOP_HEADER  = (4, 5, 6)

##################################################
#  Maps for retrieving enum contents
##################################################

ERROR_CODE_MAP = {
    1 : 'NO_EXTERNAL_PROCESSING_NECESSARY',
    0 : 'OK',
    -1 : 'START_HEADER_NOT_PRESENT',
    -2 : 'STOP_HEADER_NOT_PRESENT',
    -3 : 'CRC_NOT_CORRECT',
    -4 : 'CMD_NOT_VALID',
    -5 : 'SUB_CMD_NOT_VALID',
    -6 : 'CMD_NOT_PROCESSED',
    -7 : 'DIGITAL_OUTPUT_IDX_BANK_NOT_VALID',
    -8 : 'DIGITAL_OUTPUT_IDX_PIN_NOT_VALID',
    -9 : 'IDX_PIN_NOT_VALID',
    -10 : 'PIN_TYPE_NOT_VALID',
    -11 : 'CMD_NOT_ALLOWED',
    -12 : 'INCORRECT_NUMBER_OF_PINS',
    -13 : 'INCORRECT_NUMBER_OF_DIGITAL_PINS',
    -14 : 'SYSTEM_NOT_INITIALIZED',
    -100 : 'TIMEOUT_ON_RESPONSE',
    -101 : 'INCORRECT_PAYLOAD_SIZE',
    -102 : 'NO_AFFINITY_WITH_RESPONSE_PACKET',
    -103 : 'CMD_NOT_EXPECTED',
    -104 : 'INCORRECT_PARAMETERS_NUMBER',
    -105 : 'INCORRECT_PARAMETER_TYPE',
    -106 : 'INCOMPLETE_PARAMETERS',
    -107 : 'WRONG_NUMBER_EXPECTED_RESPONSE_PACKETS',
    -3000 : 'IMU_COMMUNICATION_ERROR',
    -3001 : 'IMU_NOT_CONNECTED',
    -3002 : 'IMU_CMD_NOT_EXECUTED',
    -4000 : 'SERIAL_DEVICE_GENERIC_ERROR',
    -4001 : 'SERIAL_DEVICE_PORT_NOT_OPEN',
    -4002 : 'SERIAL_DEVICE_ERROR_ON_WRITING',
    -10000 : 'PINS_CONFIGURATION_PARSING_ERROR',
    -5000 : 'GENERIC_ERROR'
}

CMD_MAP = {
    0 : 'INITIALIZE_SYSTEM',
    1 : 'HELLO',
    2 : 'SERIAL_DEBUG',
    3 : 'GET_SYSTEM_STATUS',
    10 : 'ENABLE_SENDING_PACKETS',
    11 : 'ENABLE_MULTIPLE_PACKETS',
    100 : 'CONFIGURE_PIN',
    101 : 'GET_PINS_CONFIGURATION',
    102 : 'GET_VALUE',
    103 : 'GET_ALL_VALUES',
    104 : 'SET_VALUE',
    105 : 'SET_DIGITAL_PINS',
    200 : 'IMU_START',
    201 : 'IMU_STOP',
    202 : 'IMU_SET_SETTINGS',
    203 : 'IMU_GET_SETTINGS',
    204 : 'IMU_GYROSCOPE_CALIBRATION',
    205 : 'IMU_RESET_FILTER',
    206 : 'IMU_GET_DATA',
    207 : 'IMU_TARE_WITH_CURRENT_ORIENTATION',
    208 : 'IMU_RESET_TARE',
    250 : 'STOP_BOARD',
    254 : 'VALUE',
    255 : 'ACK'
}

CMD_INV_MAP = { v:k for k,v in CMD_MAP.items() }

EMPTY_SUB_CMD_MAP = {0 : 'EMPTY'}

SUB_CMD_MAP = {
    'INITIALIZE_SYSTEM' : EMPTY_SUB_CMD_MAP,
    'HELLO' : EMPTY_SUB_CMD_MAP,
    'SERIAL_DEBUG' : EMPTY_SUB_CMD_MAP,
    'GET_SYSTEM_STATUS' : EMPTY_SUB_CMD_MAP,
    'ENABLE_SENDING_PACKETS' : EMPTY_SUB_CMD_MAP,
    'ENABLE_MULTIPLE_PACKETS' : EMPTY_SUB_CMD_MAP,
    'CONFIGURE_PIN' : {
                          0 : 'SET_PIN_DISABLE',
                          1 : 'SET_PIN_DIGITAL_INPUT',
                          2 : 'SET_PIN_DIGITAL_OUTPUT',
                          3 : 'SET_PIN_ANALOG_INPUT'
                      },
    'GET_PINS_CONFIGURATION' : EMPTY_SUB_CMD_MAP,
    'GET_VALUE' : { 0 : 'GET_SINGLE_PIN' },
    'GET_ALL_VALUES' :  {
                            0 : 'PINS',
                            1 : 'DIGITAL_PIN'
                        },
    'SET_VALUE' : { 0 : 'SET_SINGLE_PIN' },
    'SET_DIGITAL_PINS' : {  0 : 'SET_PIN',
                            1 : 'SET_BANK'
                         },
    'IMU_START' : EMPTY_SUB_CMD_MAP,
    'IMU_STOP' : EMPTY_SUB_CMD_MAP,
    'IMU_SET_SETTINGS' : EMPTY_SUB_CMD_MAP,
    'IMU_GET_SETTINGS' : EMPTY_SUB_CMD_MAP,
    'IMU_GYROSCOPE_CALIBRATION' : EMPTY_SUB_CMD_MAP,
    'IMU_RESET_FILTER' :EMPTY_SUB_CMD_MAP,
    'IMU_GET_DATA' : {
                        0 : 'GET_ALL_DATA',
                        1 : 'GET_DATA_COMPONENT_SENSOR',
                        2 : 'GET_DATA_COMPONENT_SENSOR_NORMALIZED',
                        3 : 'GET_DATA_QUATERNION',
                        4 : 'GET_DATA_LINEARACCELERATION',
                        5 : 'GET_DATA_QUATERNION_LINEARACCELERATION'
                     },
    'IMU_TARE_WITH_CURRENT_ORIENTATION' :EMPTY_SUB_CMD_MAP,
    'IMU_RESET_TARE' : EMPTY_SUB_CMD_MAP,
    'STOP_BOARD' : EMPTY_SUB_CMD_MAP,
    'VALUE' : EMPTY_SUB_CMD_MAP,
    'ACK' : EMPTY_SUB_CMD_MAP
}

PIN_TYPE_MAP = {
        0 : 'DISABLE',
        1 : 'DIGITAL_INPUT',
        2 : 'DIGITAL_OUTPUT',
        3 : 'ANALOG_INPUT'
}

SENDING_MODE_MAP = {
        0 : 'SENDING_MODE_ON_REQUEST',
        1 : 'SENDING_MODE_CONTINUOUS',
        2 : 'SENDING_MODE_ON_VALUE_CHANGED'
}

ADC_RESOLUTION_MAP = {
        0 : 12,
        1 : 11,
        2 : 10,
        3 : 9,
        4 : 8
}

