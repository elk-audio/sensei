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
    systemSettings.enableMultiplePackets=COND_MULTIPLE_PACKETS;
    systemSettings.enableSendingPackets=COND_SENDING_PACKETS;

    uint32_t startTaskTimestamp, endTaskTimestamp;
    uint32_t precStartTaskTimestamp=micros();

    TaskRtStatus taskStatus;
    memset(&taskStatus,0,sizeof(TaskRtStatus));

    TickType_t xLastWakeTime;
    TickType_t xFrequency = 1; // 1 ms -> 1000Hz //TODO define

    xLastWakeTime = xTaskGetTickCount();
    //FilterType FsFilter = DEFAULT_RT_FREQUENCY; //[Hz]

    ManageIO manageIO;

    // Message Queue Structs
    MsgRTtoCOM_IMU msgImu;
    MsgRTtoCOM_PIN msgPin;
    Msg_DATA msgData;

    hQueueRTtoCOM_IMU = xQueueCreate(10, sizeof(MsgRTtoCOM_IMU)); //TODO define
    hQueueRTtoCOM_PIN = xQueueCreate(10, sizeof(MsgRTtoCOM_PIN));
    hQueueRTtoCOM_DATA = xQueueCreate(64, sizeof(Msg_DATA));

    for (;;)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        startTaskTimestamp = micros();

        //------------------------------------------------------------------------------------------- [IMU]
        if (manageIO.imu.getInterruptStatus())
        {
            SerialDebug.println("IMU Ready!");
            //retCode==manageIO.imu.getAllComponents(imuComponents);
            if (xQueueSend(hQueueRTtoCOM_IMU, (void*)&msgImu, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_RT_TO_COM) != pdPASS)
            {
                if (DEBUG) //TODO
                {
                    SerialDebug.println("hQueueRTtoCOM_IMU: xQueueSend");
                }
                taskStatus.msgQueueSendErrors++;
            }
        }
        else
        {
            SerialDebug.println("IMU busy!");
        }

        //------------------------------------------------------------------------------------------- [HW]
        manageIO.hardwareAcquisition();

        if (systemSettings.enableSendingPackets)
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
                        if (DEBUG) //TODO
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

            if (systemSettings.debugMode) //TODO
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
                    msgData.status = manageIO.setSystem(msgData.data.hw.nPin, msgData.data.hw.nDigitalPin);
                    systemSettings.enableSendingPackets=false;
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_SENDING_PACKETS]
                case SENSEI_CMD::ENABLE_SENDING_PACKETS:
                    systemSettings.enableSendingPackets = static_cast<bool>(msgData.data.value);
                    if (DEBUG)
                    {
                        SerialDebug.println("enableSendingPackets= " + String(systemSettings.enableSendingPackets));
                    }
                    msgData.status = SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_MULTIPLE_PACKETS]
                case SENSEI_CMD::ENABLE_MULTIPLE_PACKETS:
                    systemSettings.enableMultiplePackets = static_cast<bool>(msgData.data.value);
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
                            msgData.status = manageIO.setDigitalBank(msgData.data.pin.idx, static_cast<bool>(msgData.data.pin.value));
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
