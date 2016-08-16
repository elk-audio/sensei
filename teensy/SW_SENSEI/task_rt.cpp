#include "task_rt.h"

// Handles for queues
extern QueueHandle_t hQueueRTtoCOM_DATA;
extern QueueHandle_t hQueueRTtoCOM_PIN;
extern QueueHandle_t hQueueRTtoCOM_IMU;

extern QueueHandle_t hQueueCOMtoRT_DATA;

void vTaskRT(void *pvParameters)
{
    SerialDebug.println("-> TASK: RT");

    SystemSettings systemSettings;
    systemSettings.debugMode=COND_DEBUG_MODE;
    systemSettings.enabledMultiplePackets=COND_MULTIPLE_PACKETS;
    systemSettings.enabledSendingPackets=COND_SENDING_PACKETS;
    systemSettings.enabledImu=COND_IMU_ENABLED;

    uint32_t startTaskTimestamp, endTaskTimestamp;
    uint32_t precStartTaskTimestamp=micros();

    sImuSettings imuSettings;
    memset(&imuSettings,0,sizeof(sImuSettings));

    sImuLinearAcceleration* linearAcceleration;
    uint32_t lastTickImuSentData=0;

    TaskRtStatus taskStatus;
    memset(&taskStatus,0,sizeof(TaskRtStatus));

    TickType_t xLastWakeTime;
    TickType_t xFrequency = 1; // 1 ms -> 1000Hz

    xLastWakeTime = xTaskGetTickCount();
    //FilterType FsFilter = DEFAULT_RT_FREQUENCY; //[Hz]

    //Manage IO
    ManageIO manageIO;

    // Message Queue Structs
    MsgRTtoCOM_IMU msgImu;
    MsgRTtoCOM_PIN msgPin;
    Msg_DATA msgData;

    hQueueRTtoCOM_IMU = xQueueCreate(MSG_QUEUE_ITEM_SIZE, sizeof(MsgRTtoCOM_IMU));
    hQueueRTtoCOM_PIN = xQueueCreate(MSG_QUEUE_ITEM_SIZE, sizeof(MsgRTtoCOM_PIN));
    hQueueRTtoCOM_DATA = xQueueCreate(MSG_QUEUE_ITEM_SIZE, sizeof(Msg_DATA));


    for (;;)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        startTaskTimestamp = micros();

        //------------------------------------------------------------------------------------------- [IMU]
        if (systemSettings.enabledSendingPackets && systemSettings.enabledImu)
        {
            uint8_t imuTypeOfData = manageIO.imu.getTypeOfData();
            uint16_t imuPacketSize = 0;
            uint8_t imuSendingMode = manageIO.imu.getSendingMode();
            uint16_t imuTicksContinuousMode = manageIO.imu.getDeltaTicksContinuousMode();
            uint32_t retImu = SENSEI_ERROR_CODE::IMU_GENERIC_ERROR;

            if (manageIO.imu.getInterruptStatus())
            {
                //SENDING_MODE_CONTINUOUS
                if (
                    (imuSendingMode == SENDING_MODE_CONTINUOUS) &&
                    (taskStatus.nCycles % imuTicksContinuousMode == 0)
                   )
                {

                    retImu = manageIO.imu.getSensorComponents(imuTypeOfData,(uint8_t*)&msgImu.vectorDataImu,imuPacketSize);

                }
                //SENDING_MODE_ON_VALUE_CHANGED
                else if (
                         (imuSendingMode == SENDING_MODE_ON_VALUE_CHANGED) &&
                         (taskStatus.nCycles > (lastTickImuSentData + imuTicksContinuousMode))
                        )
                {
                    //SerialDebug.println("!!");
                    if (manageIO.imu.getSensorComponents(IMU_GET_LINEAR_ACCELERATION,(uint8_t*)&msgImu.vectorDataImu,imuPacketSize) == SENSEI_ERROR_CODE::OK)
                    {
                        linearAcceleration = (sImuLinearAcceleration*)msgImu.vectorDataImu;
                        float squareAccelerationNorm = linearAcceleration->lax * linearAcceleration->lax +
                                                       linearAcceleration->lay * linearAcceleration->lay +
                                                       linearAcceleration->laz * linearAcceleration->laz;

                        if (squareAccelerationNorm > manageIO.imu.getMinLinearAccelerationSquareNorm())
                        {
                            if ((imuTypeOfData & IMU_GET_LINEAR_ACCELERATION) == 0x00)
                            {
                                imuPacketSize = 0;
                            }

                            retImu = manageIO.imu.getSensorComponents(imuTypeOfData & IMU_CLEAR_BIT_LINEAR_ACCELERATION,
                                                                      (uint8_t*)msgImu.vectorDataImu,
                                                                      imuPacketSize);
                            //SerialDebug.println("ax=" + String(linearAcceleration->lax) + " ay=" + String(linearAcceleration->lay) + " az=" + String(linearAcceleration->laz));

                        }
                    }
                }

                if (retImu == SENSEI_ERROR_CODE::OK)
                {
                    // Send Message Queue
                    //SerialDebug.println("hQueueRTtoCOM_IMU [" + String((float)taskStatus.nCycles) + "]");
                    msgImu.typeOfData=imuTypeOfData;
                    msgImu.packetSize = imuPacketSize;
                    lastTickImuSentData = taskStatus.nCycles;

                    if (xQueueSend(hQueueRTtoCOM_IMU, (void*)&msgImu, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_RT_TO_COM) != pdPASS)
                    {
                       if (DEBUG)
                       {
                           SerialDebug.println("hQueueRTtoCOM_IMU: xQueueSend");
                       }
                       taskStatus.msgQueueSendErrors++;
                    }
                } // else: No Data Ready
            }
        }

        //------------------------------------------------------------------------------------------- [HW]
        manageIO.hardwareAcquisition();

        if (systemSettings.enabledSendingPackets)
        {
            for (int idxPin = 0; idxPin < manageIO.getNumberOfPins(); idxPin++)
            {
                if ( ((manageIO.getSendingMode(idxPin) == SENDING_MODE_CONTINUOUS) && ((taskStatus.nCycles % manageIO.getDeltaTicksContinuousMode(idxPin)) == 0) ) ||
                     ((manageIO.getSendingMode(idxPin) == SENDING_MODE_ON_VALUE_CHANGED) && (manageIO.isMomentToSendValue(idxPin))) )
                {
                    msgPin.pin.idx=idxPin;
                    msgPin.pin.value=manageIO.getPinValue(idxPin);
                    msgPin.pin.type=manageIO.getPinType(idxPin);

                    if (xQueueSend(hQueueRTtoCOM_PIN, (void*)&msgPin, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_RT_TO_COM) != pdPASS)
                    {
                        if (DEBUG)
                        {
                            SerialDebug.println("QueueRTtoCOM_PIN: xQueueSend");
                        }
                        taskStatus.msgQueueSendErrors++;
                    }
                }
            }
        }

        //------------------------------------------------------------------------------------------- [CMD_COM]
        // CMD FROM COM
        if ((hQueueCOMtoRT_DATA != 0) && (xQueueReceive(hQueueCOMtoRT_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE)))
        {
            taskStatus.msgQueueReceived++;

            msgData.status = SENSEI_ERROR_CODE::CMD_NOT_VALID;
            msgData.msgType = RT_MSG_TYPE::ACK;

            if (systemSettings.debugMode)
            {
                SerialDebug.println("QueueRTtoCOM_DATA: xQueueReceive");
            }
            //---------------------------------------------------------------------
            // START COMMANDS
            //---------------------------------------------------------------------
            switch (msgData.cmd)
            {
                //--------------------------------------------------------------------- [CMD INITIALIZE_SYSTEM]
                case SENSEI_CMD::INITIALIZE_SYSTEM:
                    systemSettings.enabledSendingPackets=false;
                    msgData.status = manageIO.setSystem(msgData.data.hw.nPin, msgData.data.hw.nDigitalPin);
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_SENDING_PACKETS]
                case SENSEI_CMD::ENABLE_SENDING_PACKETS:
                    systemSettings.enabledSendingPackets = static_cast<bool>(msgData.data.value);
                    if (DEBUG)
                    {
                        SerialDebug.println("enabledSendingPackets= " + String(systemSettings.enabledSendingPackets));
                    }
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_MULTIPLE_PACKETS]
                case SENSEI_CMD::ENABLE_MULTIPLE_PACKETS:
                    systemSettings.enabledMultiplePackets = static_cast<bool>(msgData.data.value);
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD CONFIGURE_PIN]
                case SENSEI_CMD::CONFIGURE_PIN:
                    PRINT_PIN_CONFIGURATION_RT
                    if (manageIO.isSystemInitialized())
                    {
                        switch (msgData.sub_cmd) //pinType
                        {
                            case SENSEI_SUB_CMD::SET_PIN_DISABLE:
                            case SENSEI_SUB_CMD::SET_PIN_DIGITAL_INPUT:
                            case SENSEI_SUB_CMD::SET_PIN_DIGITAL_OUTPUT:
                                msgData.status = manageIO.configurePin(static_cast<ePinType>(msgData.sub_cmd), &msgData.data.setupPin);
                            break;

                            case SENSEI_SUB_CMD::SET_PIN_ANALOG_INPUT:
                                msgData.status = manageIO.configurePin(ePinType::PIN_ANALOG_INPUT, &msgData.data.setupPin);
                                if (msgData.data.setupPin.filterOrder > 0)
                                {
                                    delete[] msgData.data.setupPin.filterCoeff_a;
                                    delete[] msgData.data.setupPin.filterCoeff_b;
                                }
                            break;

                            default:
                                msgData.status = SENSEI_ERROR_CODE::PIN_TYPE_NOT_VALID;
                        }
                    }
                    else
                    {
                        msgData.status = SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;
                    }
                break;

                //--------------------------------------------------------------------- [CMD SET_DIGITAL_PINS]
                case SENSEI_CMD::SET_DIGITAL_PINS:
                    switch (msgData.sub_cmd)
                    {
                        case SET_PIN:
                            if (systemSettings.debugMode)
                            {
                                SerialDebug.println("SET_DIGITAL_PINS: pin=" + String(msgData.data.pin.idx) +
                                                    " value=" + String(static_cast<bool>(msgData.data.pin.value)));
                            }
                            msgData.status = manageIO.setDigitalPin(msgData.data.pin.idx, static_cast<bool>(msgData.data.pin.value));
                        break;

                        case SET_BANK:
                            if (systemSettings.debugMode)
                            {
                                SerialDebug.println("SET_DIGITAL_PINS: idxBank=" + String(msgData.data.pin.idx) +
                                                    " value=" + String(static_cast<bool>(msgData.data.pin.value)));
                            }
                            msgData.status = manageIO.setDigitalBank(msgData.data.pin.idx, msgData.data.pin.value);
                        break;

                        default:
                            msgData.status = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                    }
                break;

                //--------------------------------------------------------------------- [CMD SET_VALUE]
                case SENSEI_CMD::SET_VALUE:
                    switch (msgData.sub_cmd)
                    {
                        case SENSEI_SUB_CMD::SET_SINGLE_PIN:
                            if (systemSettings.debugMode)
                            {
                                SerialDebug.println("SET_SINGLE_PIN: pin=" + String(msgData.data.pin.idx) +
                                                    " value=" + String(static_cast<bool>(msgData.data.pin.value)));
                            }
                            msgData.status = manageIO.setPinValue(msgData.data.pin.idx, msgData.data.pin.value);
                        break;

                        default:
                            msgData.status = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                    }
                    break;

                //--------------------------------------------------------------------- [CMD GET_VALUE]
                case SENSEI_CMD::GET_VALUE:
                    switch (msgData.sub_cmd)
                    {
                        case SENSEI_SUB_CMD::GET_SINGLE_PIN:
                            msgData.msgType = RT_MSG_TYPE::DATA;
                            msgData.data.pin.type = manageIO.getPinType(msgData.data.pin.idx);
                            uint16_t value;
                            msgData.status = manageIO.getPinValue(msgData.data.pin.idx,value);
                            msgData.data.pin.value = value;
                        break;

                        default:
                            msgData.status = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                    }

                break;

                //--------------------------------------------------------------------- [CMD GET_SYSTEM_STATUS]
                case SENSEI_CMD::GET_SYSTEM_STATUS:
                    msgData.msgType = RT_MSG_TYPE::DATA;
                    msgData.data.systemStatus.taskRtStatus.nCycles = taskStatus.nCycles;
                    msgData.data.systemStatus.taskRtStatus.msgQueueReceived = taskStatus.msgQueueReceived;
                    msgData.data.systemStatus.taskRtStatus.msgQueueSendErrors = taskStatus.msgQueueSendErrors;
                    msgData.data.systemStatus.taskRtStatus.nCyclesExpired = taskStatus.nCyclesExpired;
                    msgData.data.systemStatus.taskRtStatus.controlLoopDelay = taskStatus.controlLoopDelay;
                    msgData.data.systemStatus.taskRtStatus.nCyclesCloseToExpiration = taskStatus.nCyclesCloseToExpiration;
                    msgData.data.systemStatus.taskRtStatus.lastTocTask = taskStatus.lastTocTask;
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //-----------------
                // IMU COMMANDS
                //-----------------
                //--------------------------------------------------------------------- [CMD IMU_ENABLE]
                case SENSEI_CMD::IMU_ENABLE:
                    systemSettings.enabledImu = static_cast<bool>(msgData.data.value);
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD IMU_SET_SETTINGS]
                case SENSEI_CMD::IMU_SET_SETTINGS:
                    msgData.status = manageIO.imu.setSettings(&msgData.data.imuSettings);
                break;

                //--------------------------------------------------------------------- [CMD CMD_IMU_GET_SETTINGS]
                case SENSEI_CMD::IMU_GET_SETTINGS:
                    msgData.msgType = RT_MSG_TYPE::DATA;
                    msgData.status = manageIO.imu.getSettings(&msgData.data.imuSettings);
                    manageIO.imu.printDebugImuSettings();
                break;

                //--------------------------------------------------------------------- [CMD CMD_IMU_GYROSCOPE_CALIBRATION]
                case SENSEI_CMD::IMU_GYROSCOPE_CALIBRATION:
                    if (systemSettings.enabledImu)
                    {
                        msgData.status = manageIO.imu.gyroscopeCalibration();
                    }
                    else
                    {
                        msgData.status = SENSEI_ERROR_CODE::IMU_DISABLED;
                    }
                break;

                //--------------------------------------------------------------------- [CMD CMD_IMU_RESET_FILTER]
                case SENSEI_CMD::IMU_RESET_FILTER:
                    if (systemSettings.enabledImu)
                    {
                        msgData.status = manageIO.imu.resetFilter();
                    }
                    else
                    {
                        msgData.status = SENSEI_ERROR_CODE::IMU_DISABLED;
                    }
                break;

                //--------------------------------------------------------------------- [CMD CMD_IMU_GET_DATA]
                case SENSEI_CMD::IMU_GET_DATA:
                    msgData.msgType = RT_MSG_TYPE::DATA;
                    if (systemSettings.enabledImu)
                    {
                        uint16_t imuPacketSize = 0;
                        msgData.status = manageIO.imu.getSensorComponents(msgData.sub_cmd,(uint8_t*)&msgData.data.vectorDataImu,imuPacketSize);
                        msgData.packetSize = imuPacketSize;
                    }
                    else
                    {
                        msgData.status = SENSEI_ERROR_CODE::IMU_DISABLED;
                    }
                break;

                //--------------------------------------------------------------------- [CMD IMU_TARE_WITH_CURRENT_ORIENTATION]
                case SENSEI_CMD::IMU_TARE_WITH_CURRENT_ORIENTATION:
                    if (systemSettings.enabledImu)
                    {
                        msgData.status = manageIO.imu.tareWithCurrentOrientation();
                    }
                    else
                    {
                        msgData.status = SENSEI_ERROR_CODE::IMU_DISABLED;
                    }
                break;

                //---------------------------------------------------------------------
                // END COMMANDS
                //---------------------------------------------------------------------
            } //switch (msgCmd.cmd)



            //Message DATA to COM
            if ((hQueueRTtoCOM_DATA != 0) && (xQueueSend(hQueueRTtoCOM_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_COM_TO_RT) != pdPASS))
            {
                if (DEBUG)
                {
                    SerialDebug.println("QueueRTtoCOM_DATA: msgQueueSendErrors");
                }
                taskStatus.msgQueueSendErrors++;
            }


        } //hQueueCOMtoRT_CMD
        //------------------------------------------------------------------------------------------- [CMD_COM]

        endTaskTimestamp = micros();

        if (manageIO.isSystemInitialized())
        {
            taskStatus.lastTocTask = endTaskTimestamp - startTaskTimestamp;
            if (taskStatus.lastTocTask > 900) //to define TODO
            {
                taskStatus.nCyclesCloseToExpiration++;
            }
            if (taskStatus.lastTocTask > 1000) //to define
            {
               taskStatus.nCyclesExpired++;
            }

            taskStatus.controlLoopDelay += (static_cast<int32_t>(startTaskTimestamp - precStartTaskTimestamp) - 1000);
        }
        precStartTaskTimestamp = startTaskTimestamp;

        taskStatus.nCycles++;

    } //for(;;)
}
