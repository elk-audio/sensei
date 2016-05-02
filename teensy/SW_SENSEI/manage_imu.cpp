#include "manage_imu.h"

ManageIMU::ManageIMU()
{
    _internalSettings.decompositionOrder = DECOMPOSITION_ZYX;
    _internalSettings.axisDirection = AXIS_DIRECTION_RIGHT_FORWARD_UP_RHANDED;
    _internalSettings.calibrationMode = IMU_CALIBRATION_MODE_BIAS;
    _internalSettings.referenceVectorMode = IMU_REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE;

    _settings.filterMode = IMU_FILTER_KALMAN;
    _settings.accerelometerRange = IMU_SENSOR_ACCELEROMETER_RANGE_2G;
    _settings.gyroscopeRange = IMU_SENSOR_GYROSCOPE_RANGE_2000;
    _settings.compassRange = IMU_SENSOR_COMPASS_RANGE_1_30;
    _settings.compassEnable = 0;
    _settings.deltaTicksContinuousMode = 0;
    _settings.typeOfData = 0; // -> eImuGetData

    if (_initialize() == SENSEI_ERROR_CODE::OK)
        _isInitialized = true;
    else
        _isInitialized = false;
}

ManageIMU::~ManageIMU()
{

}

void ManageIMU::clearBuffer()
{
    // Clear the internal data buffer on the IMU
    SPI.transfer(SPI_CMD::CLEAR_BUFFER);
    delayMicroseconds(DELAY_SPI_YEI);
}

bool ManageIMU::getInterruptStatus()
{
    uint8_t imuInterruptStatus=0;
    getValue(CMD_IMU::GET_INTERRUPT_STATUS, &imuInterruptStatus);
    return static_cast<bool>(imuInterruptStatus);
}

int32_t ManageIMU::writeCommand(uint8_t cmd, uint8_t* value, uint8_t nByte)
{
    //SerialDebug.println("writeCommand");

    uint16_t iter = 0;
    uint8_t ret = SPI.transfer(SPI_CMD::START_DATA_TRANSFER);
    delayMicroseconds(DELAY_SPI_YEI);

    if (ret == 0xFF)
    {
        return SENSEI_ERROR_CODE::IMU_NOT_CONNECTED;
    }

    ret = SPI.transfer(cmd);
    if (ret != SPI_CMD::ACC_STATE)
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;

    for (uint8_t i = 0; i<nByte; i++)
    {
        delayMicroseconds(DELAY_SPI_YEI);
        ret = SPI.transfer(value[i]);
    }

    while (ret != SPI_CMD::READY_STATE && iter<SPI_MAX_TRANSFERS_PER_CMD)
    {
        delayMicroseconds(DELAY_SPI_YEI);
        ret = SPI.transfer(0xFF);
        iter++;
    }
    //SerialDebug.println(iter);

    if (iter == SPI_MAX_TRANSFERS_PER_CMD)
    {
        SerialDebug.println("SPI_MAX_TRANSFERS_PER_CMD");
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;
    }

    delayMicroseconds(DELAY_SPI_YEI);

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::writeCommand(uint8_t cmd)
{
    return writeCommand(cmd, 0, 0);
}

int32_t ManageIMU::writeCommand(uint8_t cmd, uint8_t value)
{
    return writeCommand(cmd, &value, 1);
}

int32_t ManageIMU::getValue(uint8_t cmd, void* data)
{
    return getData(cmd,data,1);
}

int32_t ManageIMU::getData(uint8_t cmd, void* data, uint16_t nByte)
{
    int32_t ret;

    uint8_t* pointer = (uint8_t*)data;
    ret = writeCommand(cmd);

    if (ret != SENSEI_ERROR_CODE::OK)
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;

    for (uint8_t i = nByte; i>0; i--)
    {
        delayMicroseconds(DELAY_SPI_YEI);
        pointer[i - 1] = SPI.transfer(0xFF);
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::resetFilter()
{
    int32_t ret;
    ret = writeCommand(CMD_IMU::RESET_BASE_OFFSET);
    if (ret != SENSEI_ERROR_CODE::OK) return ret;

    ret = writeCommand(CMD_IMU::RESET_FILTER);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setInterruptMode()
{
    /*int32_t ret;
    uint8_t data[] = { 2, 0 };
    ret =  writeCommand(CMD_SET_PIN_MODE, data, sizeof(data));
    return ret;*/

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::_initialize()
{
    int32_t ret;

    ret = writeCommand(CMD_IMU::SET_DECOMPOSITION_ORDER, _internalSettings.decompositionOrder);
    if (ret != SENSEI_ERROR_CODE::OK)
        return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_AXIS_DIRECTION, _internalSettings.axisDirection);
    if (ret != SENSEI_ERROR_CODE::OK)
        return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_CALIBRATION_MODE, _internalSettings.calibrationMode);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_REFERENCE_VECTOR_MODE, _internalSettings.referenceVectorMode);
    if (ret != SENSEI_ERROR_CODE::OK)
        return IMU_CMD_NOT_EXECUTED;

    ret = setSettings();
    if (ret != SENSEI_ERROR_CODE::OK)
        return IMU_CMD_NOT_EXECUTED;

    //setInterruptMode();
    ret = resetFilter();
    if (ret != SENSEI_ERROR_CODE::OK)
        return IMU_CMD_NOT_EXECUTED;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getSettings(sImuSettings* settings)
{
    uint8_t value;

    if (getValue(CMD_IMU::GET_FILTER_MODE, &value) == SENSEI_ERROR_CODE::OK)
        settings->filterMode = value;
    else
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_IMU::GET_ACCELEROMETER_RANGE, &value) == SENSEI_ERROR_CODE::OK)
        settings->accerelometerRange = value;
    else
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_IMU::GET_GYROSCOPE_RANGE, &value) == SENSEI_ERROR_CODE::OK)
        settings->gyroscopeRange = value;
    else
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_IMU::GET_COMPASS_RANGE, &value) == SENSEI_ERROR_CODE::OK)
        settings->compassRange = value;
    else
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_IMU::GET_COMPASS_ENABLED_STATE, &value) == SENSEI_ERROR_CODE::OK)
        settings->compassEnable = value;
    else
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    settings->deltaTicksContinuousMode = _settings.deltaTicksContinuousMode;
    settings->typeOfData = _settings.typeOfData;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getAllComponents(ImuComponents* components)
{
    //TODO if return
    uint8_t* ptr = (uint8_t*)components;

    getData(CMD_IMU::READ_TARED_ORIENTATION_AS_QUATERNION, ptr, sizeof(sImuQuaternion));
    getData(CMD_IMU::READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA, ptr+sizeof(sImuQuaternion), sizeof(sImuComponentSensor));
    getData(CMD_IMU::READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE, ptr+sizeof(sImuQuaternion)+sizeof(sImuComponentSensor), sizeof(sImuLinearAcceleration));

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setSettings(sImuSettings* settings)
{
    if (settings->filterMode < N_IMU_FILTER_TYPES)
        _settings.filterMode = settings->filterMode;
    else
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;

    //TODO
    _settings.filterMode = settings->filterMode;
    _settings.accerelometerRange = settings->accerelometerRange;
    _settings.gyroscopeRange = settings->gyroscopeRange;
    _settings.compassRange = settings->compassRange;
    _settings.compassEnable = settings->compassEnable;
    _settings.deltaTicksContinuousMode = settings->deltaTicksContinuousMode;
    _settings.typeOfData = settings->typeOfData;

    return setSettings();
}

int32_t ManageIMU::setSettings()
{
    int32_t ret = 0;

    ret = writeCommand(CMD_IMU::SET_FILTER_MODE, _settings.filterMode);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_ACCELEROMETER_RANGE, _settings.accerelometerRange);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_GYROSCOPE_RANGE, _settings.gyroscopeRange);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_COMPASS_RANGE, _settings.compassRange);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret = writeCommand(CMD_IMU::SET_COMPASS_ENABLE, _settings.compassEnable);
    if (ret != SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    return SENSEI_ERROR_CODE::OK;
}
