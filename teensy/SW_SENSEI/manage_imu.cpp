#include "manage_imu.h"

ManageIMU::ManageIMU()
{
    _internalSettings.decompositionOrder = DECOMPOSITION_ZYX;
    _internalSettings.axisDirection = AXIS_DIRECTION_RIGHT_FORWARD_UP_RHANDED;
    _internalSettings.calibrationMode = IMU_CALIBRATION_MODE_BIAS;
    _internalSettings.referenceVectorMode = IMU_REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE;

    _settings.filterMode = IMU_FILTER_KALMAN; //TODO DEFAULT
    _settings.accerelometerRange = IMU_SENSOR_ACCELEROMETER_RANGE_2G;
    _settings.gyroscopeRange = IMU_SENSOR_GYROSCOPE_RANGE_2000;
    _settings.compassRange = IMU_SENSOR_COMPASS_RANGE_1_30;
    _settings.compassEnable = 0;
    _settings.deltaTicksContinuousMode = 0;
    _settings.typeOfData = IMU_GET_ALL_DATA;

    if (_initialize() == SENSEI_ERROR_CODE::OK)
    {
        _isInitialized = true;
        SerialDebug.println("-> IMU: OK");
    }
    else
    {
        _isInitialized = false;
        SerialDebug.println("-> IMU: KO");
    }
}

ManageIMU::~ManageIMU()
{

}

void ManageIMU::_clearBuffer()
{
    _writeByte(CLEAR_BUFFER);
    delayMicroseconds(DELAY_SPI_YEI_CLEAR_BUFFER);
}

void ManageIMU::_startCommunication()
{
    _writeByte(START_DATA_TRANSFER);
}

uint8_t ManageIMU::_readByte() //TODO inline
{
    uint8_t retSpi;

    digitalWrite(SPI_SS,LOW);
    delayMicroseconds(DELAY_SPI_YEI);
    retSpi = SPI.transfer(0xFF);
    digitalWrite(SPI_SS,HIGH);
    delayMicroseconds(DELAY_SPI_YEI);

    return retSpi;
}

void ManageIMU::_writeByte(uint8_t value) //TODO inline
{

    digitalWrite(SPI_SS,LOW);
    delayMicroseconds(DELAY_SPI_YEI);
    _retSpi = SPI.transfer(value);
    digitalWrite(SPI_SS,HIGH);
    delayMicroseconds(DELAY_SPI_YEI);
}

int32_t ManageIMU::_waitStatus(uint8_t status) //TODO inline
{
    uint16_t iter = 0;
    while ( (_retSpi != status) && (_retSpi != IDLE_STATE) && (iter < SPI_MAX_ITER_WAIT_CMD) )
    {
        _writeByte(DATA_BYTE);
        iter++;
        //SerialDebug.println("-> _waitStatus: " + String(_retSpi) ); //TODO
    }

    if (iter == SPI_MAX_ITER_WAIT_CMD)
    {
        SerialDebug.println("-> IMU ERROR(_waitStatus): " + String(_retSpi) ); //TODO
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;
    }
    else
    {
        return SENSEI_ERROR_CODE::OK;
    }
}

void ManageIMU::_sendCmd(uint8_t cmd)
{
    _writeByte(cmd);
    //SerialDebug.println("-> _sendCmd: " + String(_retSpi) ); //TODO
}

int32_t ManageIMU::sendCommand(uint8_t cmd, uint8_t* value, uint8_t nByte)
{

    _startCommunication();
    _sendCmd(cmd);

    if (_retSpi == DATA_BYTE)
    {
        return SENSEI_ERROR_CODE::IMU_NOT_CONNECTED;
    }

    if (_waitStatus(ACC_STATE) != SENSEI_ERROR_CODE::OK)
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    for (int i = 0; i<nByte; i++)
    {
        //SerialDebug.println("DATA=" + String(value[i]));
        _writeByte(value[i]);
    }

    return _waitStatus(IDLE_STATE);
}

int32_t ManageIMU::sendCommand(uint8_t cmd)
{
    return sendCommand(cmd, 0, 0);
}

int32_t ManageIMU::sendCommand(uint8_t cmd, uint8_t value)
{
    return sendCommand(cmd, &value, 1);
}

int32_t ManageIMU::getValue(uint8_t cmd, void* data)
{
    return getData(cmd,data,1);
}

int32_t ManageIMU::getData(uint8_t cmd, void* data, uint16_t nByte)
{
    uint8_t* pointer = (uint8_t*)data;

    _startCommunication();
    _sendCmd(cmd);

    if (_waitStatus(ACC_STATE) != OK)
    {
      return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;
    }

    if (_waitStatus(READY_STATE) != OK)
    {
      return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;
    }

    for (int i = nByte; i>0; i--)
    {
        pointer[i - 1] = _readByte();
        //SerialDebug.println("GET=" + String(pointer[i - 1]));
    }

    if (_waitStatus(IDLE_STATE) != OK)
    {
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;
    }

    return SENSEI_ERROR_CODE::OK;
}

bool ManageIMU::getInterruptStatus()
{
    uint8_t imuInterruptStatus=0;
    getValue(CMD_IMU::GET_INTERRUPT_STATUS, &imuInterruptStatus);
    return static_cast<bool>(imuInterruptStatus);
}

int32_t ManageIMU::resetFilter()
{
    int32_t ret;

    ret = sendCommand(CMD_IMU::RESET_FILTER);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setInterruptMode()
{
    uint8_t data[] = { 2, 0 };
    return sendCommand(CMD_IMU::SET_PIN_MODE, data, sizeof(data));
}

int32_t ManageIMU::_initialize()
{
    int32_t ret;

    _clearBuffer();

    ret = sendCommand(CMD_IMU::RESTORE_DEFAULT_SETTINGS);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_DECOMPOSITION_ORDER, _internalSettings.decompositionOrder);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_AXIS_DIRECTION, _internalSettings.axisDirection);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_CALIBRATION_MODE, _internalSettings.calibrationMode);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_REFERENCE_VECTOR_MODE, _internalSettings.referenceVectorMode);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = setSettings();
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = resetFilter();
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getSettings(sImuSettings* settings)
{
    uint8_t value;

    if (getValue(CMD_IMU::GET_FILTER_MODE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->filterMode = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (getValue(CMD_IMU::GET_ACCELEROMETER_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
         settings->accerelometerRange = value;
    }
    else
    {
         return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (getValue(CMD_IMU::GET_GYROSCOPE_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->gyroscopeRange = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (getValue(CMD_IMU::GET_COMPASS_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->compassRange = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (getValue(CMD_IMU::GET_COMPASS_ENABLED_STATE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->compassEnable = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    settings->deltaTicksContinuousMode = _settings.deltaTicksContinuousMode;
    settings->typeOfData = _settings.typeOfData;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getAllSensorComponents(ImuComponents* components)
{
    uint8_t* ptr = (uint8_t*)components;

    // Get quaternions
    if (getData(CMD_IMU::READ_TARED_ORIENTATION_AS_QUATERNION,
                ptr,
                sizeof(sImuQuaternion))
                != SENSEI_ERROR_CODE::OK)
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    // Get Linear Acceleration
    if (getData(CMD_IMU::READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE,
                ptr + sizeof(sImuQuaternion),
                sizeof(sImuLinearAcceleration))
                != SENSEI_ERROR_CODE::OK)
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    // Get accelerometer, gyroscope, magnetometer
    /*if (getData(CMD_IMU::READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA,
                  ptr + sizeof(sImuQuaternion) + sizeof(sImuLinearAcceleration),
                  sizeof(sImuComponentSensor))
                  != SENSEI_ERROR_CODE::OK)
    {
         return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }
    */
    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setSettings(sImuSettings* settings)
{
    if (settings->filterMode < N_IMU_FILTER_TYPES)
    {
        _settings.filterMode = settings->filterMode;
    }
    else
    {
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
    }

    //TODO
    _settings.filterMode = settings->filterMode; //N_IMU_FILTER_TYPES
    _settings.accerelometerRange = settings->accerelometerRange; //N_IMU_SENSOR_ACCELEROMETER_RANGES
    _settings.gyroscopeRange = settings->gyroscopeRange; //N_IMU_SENSOR_GYROSCOPE_RANGES
    _settings.compassRange = settings->compassRange; //N_IMU_SENSOR_COMPASS_RANGES
    _settings.compassEnable = settings->compassEnable;
    _settings.deltaTicksContinuousMode = settings->deltaTicksContinuousMode;
    _settings.typeOfData = settings->typeOfData; //N_IMU_DATA_TYPES

    return setSettings();
}

int32_t ManageIMU::setSettings()
{
    int32_t ret = 0;

    ret = sendCommand(CMD_IMU::SET_FILTER_MODE, _settings.filterMode);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_ACCELEROMETER_RANGE, _settings.accerelometerRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_GYROSCOPE_RANGE, _settings.gyroscopeRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_COMPASS_RANGE, _settings.compassRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = sendCommand(CMD_IMU::SET_COMPASS_ENABLE, _settings.compassEnable);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}
