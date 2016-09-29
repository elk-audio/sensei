#include "manage_imu.h"

ManageIMU::ManageIMU()
{
    _internalSettings.decompositionOrder = DECOMPOSITION_ZYX;
    _internalSettings.axisDirection = AXIS_DIRECTION_RIGHT_UP_FORWARD_LHANDED;
    _internalSettings.calibrationMode = IMU_CALIBRATION_MODE_BIAS;
    _internalSettings.referenceVectorMode = IMU_REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE;

    //Default IMU's settings
    _settings.filterMode = IMU_FILTER_KALMAN;
    _settings.accerelometerRange = IMU_SENSOR_ACCELEROMETER_RANGE_2G;
    _settings.gyroscopeRange = IMU_SENSOR_GYROSCOPE_RANGE_2000;
    _settings.compassRange = IMU_SENSOR_COMPASS_RANGE_1_30;
    _settings.compassEnable = 0;
    _settings.sendingMode = SENDING_MODE_ON_REQUEST;
    _settings.deltaTicksContinuousMode = 0;
    _settings.typeOfData = IMU_GET_LINEAR_ACCELERATION | IMU_GET_QUATERNIONS;
    _settings.minLinearAccelerationSquareNorm = IMU_MIN_LINEAR_ACCELERATION_NORM * IMU_MIN_LINEAR_ACCELERATION_NORM;


    if (_initialize() == SENSEI_ERROR_CODE::OK)
    {
        _isInitialized = true;
        SerialDebug.println("-> IMU: OK - MAX_SUB_CMD = " + String(IMU_MAX_SUB_CMD));

        #ifdef RESET_IMU_FACTORY_SETTINGS_ON_STARTUP
          SerialDebug.println("-> IMU: RESET_IMU_FACTORY_SETTINGS_ON_STARTUP");
        #endif

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

uint8_t ManageIMU::_readByte()
{
    uint8_t retSpi;

    digitalWrite(SPI_SS,LOW);
    delayMicroseconds(DELAY_SPI_YEI);
    retSpi = SPI.transfer(0xFF);
    digitalWrite(SPI_SS,HIGH);
    delayMicroseconds(DELAY_SPI_YEI);

    return retSpi;
}

void ManageIMU::_writeByte(uint8_t value)
{

    digitalWrite(SPI_SS,LOW);
    delayMicroseconds(DELAY_SPI_YEI);
    _retSpi = SPI.transfer(value);
    digitalWrite(SPI_SS,HIGH);
    delayMicroseconds(DELAY_SPI_YEI);
}

int32_t ManageIMU::_waitStatus(uint8_t status)
{
    uint32_t iter = 0;
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

int32_t ManageIMU::_sendCommandWithoutChecks(uint8_t cmd)
// For reboot
{

    _startCommunication();
    _sendCmd(cmd);

    if (_retSpi == DATA_BYTE)
    {
        return SENSEI_ERROR_CODE::IMU_NOT_CONNECTED;
    }

    return SENSEI_ERROR_CODE::OK;
}


int32_t ManageIMU::_sendCommand(uint8_t cmd, uint8_t* value, uint8_t nByte)
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

int32_t ManageIMU::_sendCommand(uint8_t cmd)
{
    return _sendCommand(cmd, 0, 0);
}

int32_t ManageIMU::_sendCommand(uint8_t cmd, uint8_t value)
{
    return _sendCommand(cmd, &value, sizeof(uint8_t));
}

int32_t ManageIMU::_getValue(uint8_t cmd, float* data)
{
    return _getData(cmd,data,sizeof(float));
}

int32_t ManageIMU::_getValue(uint8_t cmd, void* data)
{
    return _getData(cmd,data,sizeof(uint8_t));
}

int32_t ManageIMU::_getData(uint8_t cmd, void* data, uint16_t nByte)
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

uint8_t ManageIMU::getTypeOfData()
{
    return _settings.typeOfData;
}

bool ManageIMU::getInterruptStatus()
{
    uint8_t imuInterruptStatus=0;
    _getValue(CMD_IMU::GET_INTERRUPT_STATUS, &imuInterruptStatus);
    return static_cast<bool>(imuInterruptStatus);
}

int32_t ManageIMU::resetToFactorySettings()
{
    int32_t ret;

    ret = _sendCommand(CMD_IMU::RESTORE_DEFAULT_SETTINGS);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    SerialDebug.println("-> IMU: resetToFactorySettings()");
    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::reboot()
{
    int32_t ret;

    ret = _sendCommandWithoutChecks(CMD_IMU::SOFTWARE_RESET);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    SerialDebug.println("-> IMU: reboot()");
    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::resetFilter()
{
    int32_t ret;

    ret = _sendCommand(CMD_IMU::RESET_FILTER);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::setInterruptMode()
{
    uint8_t data[] = { 2, 0 };
    return _sendCommand(CMD_IMU::SET_PIN_MODE, data, sizeof(data));
}

int32_t ManageIMU::_initialize()
{
    int32_t ret;

    _clearBuffer();

    // Restore default settings
    #ifdef RESET_IMU_FACTORY_SETTINGS_ON_STARTUP
      ret = resetToFactorySettings();
      if (ret != SENSEI_ERROR_CODE::OK)
      {
          return IMU_CMD_NOT_EXECUTED;
      }
    #endif

    ret = _sendCommand(CMD_IMU::SET_DECOMPOSITION_ORDER, _internalSettings.decompositionOrder);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_AXIS_DIRECTION, _internalSettings.axisDirection);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_CALIBRATION_MODE, _internalSettings.calibrationMode);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_REFERENCE_VECTOR_MODE, _internalSettings.referenceVectorMode);
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

    if (_getValue(CMD_IMU::GET_FILTER_MODE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->filterMode = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (_getValue(CMD_IMU::GET_ACCELEROMETER_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
         settings->accerelometerRange = value;
    }
    else
    {
         return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (_getValue(CMD_IMU::GET_GYROSCOPE_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->gyroscopeRange = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (_getValue(CMD_IMU::GET_COMPASS_RANGE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->compassRange = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    if (_getValue(CMD_IMU::GET_COMPASS_ENABLED_STATE, &value) == SENSEI_ERROR_CODE::OK)
    {
        settings->compassEnable = value;
    }
    else
    {
        return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }

    settings->sendingMode = _settings.sendingMode;
    settings->deltaTicksContinuousMode = _settings.deltaTicksContinuousMode;
    settings->typeOfData = _settings.typeOfData;
    settings->minLinearAccelerationSquareNorm = _settings.minLinearAccelerationSquareNorm;

    return SENSEI_ERROR_CODE::OK;
}


int32_t ManageIMU::getSensorComponents(uint8_t components,uint8_t* data_vector,uint16_t& packetSize)
{
    //SerialDebug.println("getSensorComponents");

    uint8_t comp_cmd;
    uint8_t comp_size;
    uint8_t* ptr = (uint8_t*)data_vector;

    for(int idx_comp=0; idx_comp<IMU_MAX_COMPONENTS; idx_comp++)
    {
        if (components & (0x01 << idx_comp))
        {
            //SerialDebug.println("POST idx_comp=" + String(idx_comp));

            switch (idx_comp)
            {
                case 0x00:
                    //SerialDebug.println("READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE");
                    comp_cmd = READ_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE;
                    comp_size = sizeof(sImuLinearAcceleration);
                break;

                case 0x01:
                    //SerialDebug.println("READ_TARED_ORIENTATION_AS_QUATERNION");
                    comp_cmd = READ_TARED_ORIENTATION_AS_QUATERNION;
                    comp_size = sizeof(sImuQuaternion);
                break;

                case 0x02:
                    //SerialDebug.println("READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA");
                    comp_cmd = READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA;
                    comp_size = sizeof(sImuComponentSensor);
                break;

                case 0x03:
                    //SerialDebug.println("READ_ALL_NORMALIZED_COMPONENT_SENSOR_DATA");
                    comp_cmd = READ_ALL_NORMALIZED_COMPONENT_SENSOR_DATA;
                    comp_size = sizeof(sImuNormalizedComponentSensor);
                break;

                default:
                    comp_size = 0;
            }

            if (comp_size > 0)
            {
                if (_getData(comp_cmd,
                            ptr + packetSize,
                            comp_size)
                            != SENSEI_ERROR_CODE::OK)
                {
                    return SENSEI_ERROR_CODE::IMU_GENERIC_ERROR;
                }
                packetSize+=comp_size;
            }
        }
    }

    return SENSEI_ERROR_CODE::OK;
}

/*int32_t ManageIMU::getAllSensorComponents(ImuComponents* components)
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

    //Get accelerometer, gyroscope, magnetometer
    if (getData(CMD_IMU::READ_ALL_CORRECTED_COMPONENT_SENSOR_DATA,
                  ptr + sizeof(sImuQuaternion) + sizeof(sImuLinearAcceleration),
                  sizeof(sImuComponentSensor))
                  != SENSEI_ERROR_CODE::OK)
    {
         return SENSEI_ERROR_CODE::IMU_CMD_NOT_EXECUTED;
    }
    return SENSEI_ERROR_CODE::OK;
}*/

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

    if (settings->accerelometerRange < N_IMU_SENSOR_ACCELEROMETER_RANGES)
    {
        _settings.accerelometerRange = settings->accerelometerRange;
    }
    else
    {
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
    }

    if (settings->gyroscopeRange < N_IMU_SENSOR_GYROSCOPE_RANGES)
    {
        _settings.gyroscopeRange = settings->gyroscopeRange;
    }
    else
    {
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
    }

    if (settings->gyroscopeRange < N_IMU_SENSOR_COMPASS_RANGES)
    {
        _settings.compassRange = settings->compassRange;
    }
    else
    {
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
    }

    //if (settings->typeOfData > 0 && settings->typeOfData <= IMU_MAX_SUB_CMD)
    if (settings->typeOfData <= IMU_MAX_SUB_CMD)
    {
        _settings.typeOfData = settings->typeOfData;
    }
    else
    {
        return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
    }

    _settings.compassEnable = settings->compassEnable;
    _settings.sendingMode = settings->sendingMode;
    _settings.deltaTicksContinuousMode = settings->deltaTicksContinuousMode;
    _settings.typeOfData = settings->typeOfData;
    _settings.minLinearAccelerationSquareNorm = settings->minLinearAccelerationSquareNorm;

    return setSettings();
}

int32_t ManageIMU::setSettings()
{
    int32_t ret = 0;

    ret = _sendCommand(CMD_IMU::SET_FILTER_MODE, _settings.filterMode);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_ACCELEROMETER_RANGE, _settings.accerelometerRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_GYROSCOPE_RANGE, _settings.gyroscopeRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_COMPASS_RANGE, _settings.compassRange);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    ret = _sendCommand(CMD_IMU::SET_COMPASS_ENABLE, _settings.compassEnable);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::getTemperature(float* temp)
{
    int32_t ret =  _getValue(CMD_IMU::GET_TEMPERATURE,*(&temp));
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::gyroscopeCalibration()
{
    int32_t ret =  _sendCommand(BEGIN_GYROSCOPE_AUTOCALIBRATION);
    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIMU::tareWithCurrentOrientation()
{
    int32_t ret = _sendCommand(TARE_WITH_CURRENT_ORIENTATION);

    if (ret != SENSEI_ERROR_CODE::OK)
    {
        return IMU_CMD_NOT_EXECUTED;
    }

    return SENSEI_ERROR_CODE::OK;
}

//int32_t ManageIMU::resetTare()
//{
//    return sendCommand(RESET_BASE_OFFSET);
//}

float ManageIMU::getMinLinearAccelerationSquareNorm()
{
    return _settings.minLinearAccelerationSquareNorm;
}

uint8_t ManageIMU::getSendingMode()
{
    return _settings.sendingMode;
}

uint16_t ManageIMU::getDeltaTicksContinuousMode()
{
    return _settings.deltaTicksContinuousMode;
}

void ManageIMU::printDebugImuSettings()
{
    SerialDebug.println("IMU SETTINGS (RT):");
    SerialDebug.println("   filterMode = " + String(_settings.filterMode));
    SerialDebug.println("   accerelometerRange = " + String(_settings.accerelometerRange));
    SerialDebug.println("   gyroscopeRange = " + String(_settings.gyroscopeRange));
    SerialDebug.println("   compassRange = " + String(_settings.compassRange));
    SerialDebug.println("   compassEnable = " + String(_settings.compassEnable));
    SerialDebug.println("   sendingMode = " + String(_settings.sendingMode));
    //SerialDebug.println("   deltaTicksContinuousMode = " + String(_settings.deltaTicksContinuousMode));
    //SerialDebug.println("   typeOfData = " + String(_settings.typeOfData));
    //SerialDebug.println("   minLinearAccelerationNorm = " + String(sqrt(_settings.minLinearAccelerationSquareNorm)));
    SerialDebug.println("");
}
