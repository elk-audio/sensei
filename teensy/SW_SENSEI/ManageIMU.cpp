#include "ManageIMU.h"

ManageIMU::ManageIMU()
{
    internalSettings.decompositionOrder = DECOMPOSITION_ZYX;
    internalSettings.axisDirection = AXIS_DIRECTION_RIGHT_FORWARD_UP_RHANDED;
    internalSettings.calibrationMode = IMU_CALIBRATION_MODE_BIAS;
    internalSettings.referenceVectorMode = IMU_REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE;


    settings.filterMode = IMU_FILTER_KALMAN;
    settings.accerelometerRange = IMU_SENSOR_ACCELEROMETER_RANGE_2G;
    settings.gyroscopeRange = IMU_SENSOR_GYROSCOPE_RANGE_2000;
    settings.compassRange = IMU_SENSOR_COMPASS_RANGE_1_30;
    settings.compassEnable = 0;
    settings.deltaTicksContinuousMode = 0;
    settings.typeOfData = 0; // -> eImuGetData

    if (initialize()==SENSEI_ERROR_CODE::OK)
    {
        isInitialized=true;
    }
    else
    {
        isInitialized=false;
    }
}


ManageIMU::~ManageIMU()
{

}


void ManageIMU::clearBuffer()
{
    // Clear the internal data buffer on the IMU
    SPI.transfer(SPI_CLEAR_BUFFER);
    delayMicroseconds(DELAY_SPI_YEI);
}


int32_t ManageIMU::writeCommand(uint8_t cmd, uint8_t* value, uint8_t nByte)
{
    //SerialDebug.println("writeCommand");

    uint16_t iter = 0;
    uint8_t ret = SPI.transfer(START_SPI_DATA_TRANSFER);
    delayMicroseconds(DELAY_SPI_YEI);

    if (ret==0xFF)
    {
        return SENSEI_ERROR_CODE::IMU_NOT_CONNECTED;
    }

    ret = SPI.transfer(cmd);
    if (ret != SPI_ACC_STATE)
        return SENSEI_ERROR_CODE::IMU_COMMUNICATION_ERROR;

    for (uint8_t i = 0; i<nByte; i++)
    {
        delayMicroseconds(DELAY_SPI_YEI);
        ret = SPI.transfer(value[i]);
    }

    while (ret != SPI_READY_STATE && iter<SPI_MAX_TRANSFERS_PER_CMD)
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
    ret=writeCommand(CMD_RESET_BASE_OFFSET);
    if (ret!=SENSEI_ERROR_CODE::OK) return ret;

    ret = writeCommand(CMD_RESET_FILTER);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setInterruptMode()
{
    /*int32_t ret;
    uint8_t data[] = { 2, 0 };
    ret=writeCommand(CMD_SET_PIN_MODE, data, sizeof(data));
    return ret;*/

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::initialize()
{
    int32_t ret;

    ret=writeCommand(CMD_SET_DECOMPOSITION_ORDER, internalSettings.decompositionOrder);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_AXIS_DIRECTION, internalSettings.axisDirection);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_CALIBRATION_MODE, internalSettings.calibrationMode);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_REFERENCE_VECTOR_MODE, internalSettings.referenceVectorMode);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=setSettings();
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    //setInterruptMode();
    ret=resetFilter();
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getSettings(sImuSettings* _settings)
{
    uint8_t value;

    if (getValue(CMD_GET_FILTER_MODE,&value)==SENSEI_ERROR_CODE::OK)
    _settings->filterMode=value;
    else return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_GET_ACCELEROMETER_RANGE,&value)==SENSEI_ERROR_CODE::OK)
    _settings->accerelometerRange=value;
    else return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_GET_GYROSCOPE_RANGE,&value)==SENSEI_ERROR_CODE::OK)
    _settings->gyroscopeRange=value;
    else return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_GET_COMPASS_RANGE,&value)==SENSEI_ERROR_CODE::OK)
    _settings->compassRange=value;
    else return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    if (getValue(CMD_GET_COMPASS_ENABLED_STATE,&value)==SENSEI_ERROR_CODE::OK)
    _settings->compassEnable=value;
    else return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;

    _settings->deltaTicksContinuousMode=settings.deltaTicksContinuousMode;
    _settings->typeOfData=settings.typeOfData;

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getAllComponents(ImuComponents* _components)
{
    //TODO if return
    uint8_t* ptr=(uint8_t*)_components;

    getData(READ_TARED_ORIENTATION_AS_QUATERNION, ptr, sizeof(sImuQuaternion));
    getData(READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA, ptr+sizeof(sImuQuaternion), sizeof(sImuComponentSensor));
    getData(READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE, ptr+sizeof(sImuQuaternion)+sizeof(sImuComponentSensor), sizeof(sImuLinearAcceleration));

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setSettings(sImuSettings* _settings)
{
    //int32_t ret;

    if (_settings->filterMode<N_IMU_FILTER_TYPES)
    settings.filterMode=_settings->filterMode;
    else return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;

    //TODO
    settings.filterMode=_settings->filterMode;
    settings.accerelometerRange=_settings->accerelometerRange;
    settings.gyroscopeRange=_settings->gyroscopeRange;
    settings.compassRange=_settings->compassRange;
    settings.compassEnable=_settings->compassEnable;
    settings.deltaTicksContinuousMode=_settings->deltaTicksContinuousMode;
    settings.typeOfData=_settings->typeOfData;

    return setSettings();
}


int32_t ManageIMU::setSettings()
{
    int32_t ret=0;

    ret=writeCommand(CMD_SET_FILTER_MODE, settings.filterMode);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_ACCELEROMETER_RANGE, settings.accerelometerRange);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_GYROSCOPE_RANGE, settings.gyroscopeRange);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_COMPASS_RANGE, settings.compassRange);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;

    ret=writeCommand(CMD_SET_COMPASS_ENABLE, settings.compassEnable);
    if (ret!=SENSEI_ERROR_CODE::OK) return IMU_CMD_NOT_EXECUTED;


    return SENSEI_ERROR_CODE::OK;
}
