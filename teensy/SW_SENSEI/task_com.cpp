#include "task_com.h"

using namespace sensei;

// Handles for queues
extern QueueHandle_t hQueueRTtoCOM_DATA;
extern QueueHandle_t hQueueRTtoCOM_PIN;
extern QueueHandle_t hQueueRTtoCOM_IMU;

extern QueueHandle_t hQueueCOMtoRT_DATA;

void vTaskCOM(void *pvParameters)
{
    SerialDebug.println("-> TASK: COM");
    delay(DELAY_START_TASK_COM);

    sImuSettings imuSettings;
    memset(&imuSettings,0,sizeof(sImuSettings));

    SystemSettings systemSettings;
    systemSettings.debugMode=COND_DEBUG_MODE;
    systemSettings.enabledMultiplePackets=COND_MULTIPLE_PACKETS;

    TaskComStatus taskStatus;
    memset(&taskStatus,0,sizeof(TaskComStatus));

    // Message Queue Stuff
    MsgRTtoCOM_IMU msgImu;
    MsgRTtoCOM_PIN msgPin;
    Msg_DATA msgData;

    hQueueCOMtoRT_DATA = xQueueCreate(MSG_QUEUE_ITEM_SIZE, sizeof(Msg_DATA));

    // Filtering
    Butterworth butterworth;
    FilterType FsFilter = DEFAULT_RT_FREQUENCY; //[Hz]

    ManageDataPacket manageDataPacket;

    uint8_t cmd;
    uint8_t sub_cmd;
    uint32_t timestamp;
    int32_t retCode;

    //SPI Chip Select
    digitalWrite(SPI_SS, LOW);

    for (;;)
    {
        if (Serial.available()>=SENSEI_LENGTH_DATA_PACKET)
        {
            manageDataPacket.receivePacket();
            retCode = manageDataPacket.checkPacket();
            if (retCode == SENSEI_ERROR_CODE::OK)
            {
                manageDataPacket.getPacketID(cmd, sub_cmd, timestamp);
                retCode = SENSEI_ERROR_CODE::CMD_NOT_VALID;

                if (systemSettings.debugMode)
                {
                    SerialDebug.println("cmd=" + String(cmd) +
                                        " sub_cmd=" + String(sub_cmd) +
                                        " timestamp=" + String(timestamp));
                }

                //---------------------------------------------------------------------
                // START COMMANDS
                //---------------------------------------------------------------------
                switch (cmd)
                {
                    //--------------------------------------------------------------------- [CMD HELLO]
                    case SENSEI_CMD::HELLO:
                        if (systemSettings.debugMode)
                        {
                            SerialDebug.println("COM: HELLO");
                        }
                        retCode = SENSEI_ERROR_CODE::NO_EXTERNAL_PROCESSING_NECESSARY;
                    break;

                    //--------------------------------------------------------------------- [CMD INITIALIZE_SYSTEM]
                    case SENSEI_CMD::INITIALIZE_SYSTEM:
                        if (systemSettings.debugMode)
                        {
                            SerialDebug.println("COM: INITIALIZE_SYSTEM");
                        }
                        systemSettings.enabledSendingPackets=false;
                        manageDataPacket.setPayloadToVariable(&msgData.data.hw, sizeof(HardwareSettings));
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD CONFIGURE_PIN]
                    case SENSEI_CMD::CONFIGURE_PIN:
                        sPinConfiguration pinConfiguration;
                        manageDataPacket.setPayloadToVariable(&pinConfiguration, sizeof(sPinConfiguration));
                        PRINT_PIN_CONFIGURATION_COM

                        memset(&msgData.data.setupPin,0,sizeof(SetupPin));
                        msgData.data.setupPin.idxPin = pinConfiguration.idxPin;

                        switch (sub_cmd) //pinType
                        {
                            case SENSEI_SUB_CMD::SET_PIN_DISABLE:
                            case SENSEI_SUB_CMD::SET_PIN_DIGITAL_OUTPUT:
                                retCode = SENSEI_ERROR_CODE::OK;
                            break;

                            case SENSEI_SUB_CMD::SET_PIN_DIGITAL_INPUT:
                                msgData.data.setupPin.sendingMode = pinConfiguration.sendingMode;
                                msgData.data.setupPin.deltaTicksContinuousMode = pinConfiguration.deltaTicksContinuousMode;
                                retCode = SENSEI_ERROR_CODE::OK;
                            break;

                            case SENSEI_SUB_CMD::SET_PIN_ANALOG_INPUT:
                                msgData.data.setupPin.sendingMode= pinConfiguration.sendingMode;
                                msgData.data.setupPin.deltaTicksContinuousMode = pinConfiguration.deltaTicksContinuousMode;
                                msgData.data.setupPin.ADCBitResolution = pinConfiguration.ADCBitResolution;
                                msgData.data.setupPin.filterOrder = pinConfiguration.filterOrder;
                                msgData.data.setupPin.sliderMode = pinConfiguration.sliderMode;
                                msgData.data.setupPin.sliderThreshold = pinConfiguration.sliderThreshold;

                                if ( (pinConfiguration.filterOrder > MAX_FILTER_ORDER) ||
                                     (pinConfiguration.lowPassCutOffFilter<0) ||
                                     (pinConfiguration.lowPassCutOffFilter>(DEFAULT_RT_FREQUENCY/2)) )
                                {
                                    retCode = SENSEI_ERROR_CODE::WRONG_FILTER_SETTINGS;
                                    break;
                                }

                                if (pinConfiguration.lowPassCutOffFilter==0.0)
                                {
                                    msgData.data.setupPin.filterOrder=0;
                                }

                                if (msgData.data.setupPin.filterOrder > 0)
                                {
                                    msgData.data.setupPin.filterCoeff_a = new FilterType[pinConfiguration.filterOrder + 1]; //TODO STRUCT
                                    msgData.data.setupPin.filterCoeff_b = new FilterType[pinConfiguration.filterOrder + 1];

                                    butterworth.createFilterCoefficients(msgData.data.setupPin.filterOrder,
                                                                         FsFilter,
                                                                         pinConfiguration.lowPassCutOffFilter,
                                                                         msgData.data.setupPin.filterCoeff_a,
                                                                         msgData.data.setupPin.filterCoeff_b);
                                }

                                retCode = SENSEI_ERROR_CODE::OK;
                            break;
                        } //pinType
                    break; //case SENSEI_CMD::CONFIGURE_PIN

                    //--------------------------------------------------------------------- [CMD SET_DIGITAL_PINS]
                    case SENSEI_CMD::SET_DIGITAL_PINS:
                    if (systemSettings.debugMode)
                        SerialDebug.println("COM: SET_DIGITAL_PINS");

                    switch (sub_cmd)
                    {
                        case SENSEI_SUB_CMD::SET_PIN:
                        case SENSEI_SUB_CMD::SET_BANK:
                            manageDataPacket.setPayloadToVariable(&msgData.data.pin, sizeof(GetSetPin));
                            retCode = SENSEI_ERROR_CODE::OK;
                        break;

                        default:
                            retCode = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                    }
                    break;

                    //--------------------------------------------------------------------- [CMD SET_VALUE]
                    case SENSEI_CMD::SET_VALUE:
                        switch (sub_cmd)
                        {
                            case SENSEI_SUB_CMD::SET_SINGLE_PIN:
                                manageDataPacket.setPayloadToVariable(&msgData.data.pin, sizeof(GetSetPin));
                                retCode = SENSEI_ERROR_CODE::OK;
                            break;

                            default:
                                retCode = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                            break;
                        }
                    break;

                    //--------------------------------------------------------------------- [CMD GET_VALUE]
                    case SENSEI_CMD::GET_VALUE:
                        switch (sub_cmd)
                        {
                            case SENSEI_SUB_CMD::GET_SINGLE_PIN:
                                manageDataPacket.setPayloadToVariable(&msgData.data.pin, sizeof(GetSetPin));
                                retCode = SENSEI_ERROR_CODE::OK;
                            break;

                            default:
                            retCode = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                        }
                    break;

                    //--------------------------------------------------------------------- [CMD ENABLE_SENDING_PACKETS]
                    case SENSEI_CMD::ENABLE_SENDING_PACKETS:
                        manageDataPacket.setPayloadToVariable(&msgData.data.value, sizeof(uint8_t));
                        systemSettings.enabledSendingPackets = msgData.data.value;
                        if (systemSettings.debugMode)
                        {
                            SerialDebug.println("enabledSendingPackets = " + String(systemSettings.enabledSendingPackets));
                        }
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD ENABLE_MULTIPLE_PACKETS]
                    case SENSEI_CMD::ENABLE_MULTIPLE_PACKETS:
                        manageDataPacket.setPayloadToVariable(&msgData.data.value, sizeof(uint8_t));
                        systemSettings.enabledMultiplePackets = msgData.data.value;
                        if (systemSettings.debugMode)
                        {
                            SerialDebug.println("enableMultiplePackets = " + String(systemSettings.enabledMultiplePackets));
                        }
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD GET_SYSTEM_STATUS]
                    case SENSEI_CMD::GET_SYSTEM_STATUS:
                        msgData.data.systemStatus.taskComStatus.msgQueueReceived = taskStatus.msgQueueReceived ;
                        msgData.data.systemStatus.taskComStatus.msgQueueSendErrors = taskStatus.msgQueueSendErrors;
                        msgData.data.systemStatus.taskComStatus.nCycles = taskStatus.nCycles;
                        msgData.data.systemStatus.taskComStatus.dataPacketReceived = manageDataPacket.getNpacketReceived();
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //-----------------
                    // IMU COMMANDS
                    //-----------------
                    //--------------------------------------------------------------------- [CMD IMU_ENABLE]
                    case SENSEI_CMD::IMU_ENABLE:
                        manageDataPacket.setPayloadToVariable(&msgData.data.value, sizeof(uint8_t));
                        SerialDebug.println("enabledImu = " + String(msgData.data.value));
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD IMU_SET_SETTINGS]
                    case SENSEI_CMD::IMU_SET_SETTINGS:

                        manageDataPacket.setPayloadToVariable(&msgData.data.imuSettings, sizeof(sImuSettings));
                        SerialDebug.println("IMU SETTINGS (COM):");
                        SerialDebug.println("   filterMode = " + String(msgData.data.imuSettings.filterMode));
                        SerialDebug.println("   accerelometerRange = " + String(msgData.data.imuSettings.accerelometerRange));
                        SerialDebug.println("   gyroscopeRange = " + String(msgData.data.imuSettings.gyroscopeRange));
                        SerialDebug.println("   compassRange = " + String(msgData.data.imuSettings.compassRange));
                        SerialDebug.println("   compassEnable = " + String(msgData.data.imuSettings.compassEnable));
                        SerialDebug.println("   sendingMode = " + String(msgData.data.imuSettings.sendingMode));
                        //SerialDebug.println("   deltaTicksContinuousMode = " + String(msgData.data.imuSettings.deltaTicksContinuousMode));
                        //SerialDebug.println("   typeOfData = " + String(msgData.data.imuSettings.typeOfData));
                        //SerialDebug.println("   minLinearAccelerationNorm = " + String(sqrt(msgData.data.imuSettings.minLinearAccelerationSquareNorm)));
                        SerialDebug.println("");
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD CMD_IMU_GET_SETTINGS]
                    case SENSEI_CMD::IMU_GET_SETTINGS:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD CMD_IMU_GYROSCOPE_CALIBRATION]
                    case SENSEI_CMD::IMU_GYROSCOPE_CALIBRATION:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD CMD_IMU_RESET_FILTER]
                    case SENSEI_CMD::IMU_RESET_FILTER:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD CMD_IMU_GET_DATA]
                    case SENSEI_CMD::IMU_GET_DATA:
                        if (sub_cmd > 0 && sub_cmd <= IMU_MAX_SUB_CMD)
                        {
                            retCode = SENSEI_ERROR_CODE::OK;
                        }
                        else
                        {
                            retCode = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                        }

                    break;

                    //--------------------------------------------------------------------- [CMD IMU_TARE_WITH_CURRENT_ORIENTATION]
                    case SENSEI_CMD::IMU_TARE_WITH_CURRENT_ORIENTATION:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD IMU_RESET_TO_FACTORY_SETTINGS]
                    case SENSEI_CMD::IMU_RESET_TO_FACTORY_SETTINGS:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD IMU_REBOOT]
                    case SENSEI_CMD::IMU_REBOOT:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD IMU_GET_TEMPERATURE]
                    case SENSEI_CMD::IMU_GET_TEMPERATURE:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //--------------------------------------------------------------------- [CMD IMU_COMMIT_SETTINGS]
                    case SENSEI_CMD::IMU_COMMIT_SETTINGS:
                        retCode = SENSEI_ERROR_CODE::OK;
                    break;

                    //---------------------------------------------------------------------
                    // STOP COMMANDS
                    //---------------------------------------------------------------------
                } //switch (cmd)

            } //if (retCode == SENSEI_ERROR_CODE::OK)
            else
            {
                //Unknow packet received
                cmd = 0;
                sub_cmd = 0;
                timestamp = 0;
            }

            //------------------------------------------------------------------------------------------- [ROUTING]
            // Command Routing
            //-------------------------------------------------------------------------------------------
            switch (retCode)
            {
                case SENSEI_ERROR_CODE::NO_EXTERNAL_PROCESSING_NECESSARY:
                    manageDataPacket.prepareACK(SENSEI_ERROR_CODE::OK, timestamp, cmd, sub_cmd);
                    manageDataPacket.send();
                break;

                case SENSEI_ERROR_CODE::OK:
                    msgData.cmd = cmd;
                    msgData.sub_cmd = sub_cmd;
                    msgData.timestamp = timestamp;

                    //Message to RT in order to process the command
                    if ((hQueueCOMtoRT_DATA != 0) && (xQueueSend(hQueueCOMtoRT_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_COM_TO_RT) != pdPASS))
                    {
                        if (DEBUG)
                        {
                            SerialDebug.println("QueueCOMtoRT_DATA: msgQueueSendErrors");
                        }
                        taskStatus.msgQueueSendErrors++;
                        retCode=SENSEI_ERROR_CODE::CMD_NOT_PROCESSED;
                    }
                break;

                default: //ERRORS->Send ACK
                    manageDataPacket.prepareACK(retCode, timestamp, cmd, sub_cmd);
                    manageDataPacket.send();
            } // switch (retCode)
            //------------------------------------------------------------------------------------------- [ROUTING]
        } //if (Serial.available())

        //------------------------------------------------------------------------------------------- [PROCESSING DATA FROM RT]
        // Processing MESSAGES
        //-------------------------------------------------------------------------------------------

        //Message DATA from RT to COM
        if ((hQueueRTtoCOM_DATA != 0) && (xQueueReceive(hQueueRTtoCOM_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE)))
        {
            taskStatus.msgQueueReceived++;

            if (systemSettings.debugMode)
            {
                SerialDebug.println("QueueRTtoCOM_DATA: xQueueReceive");
            }

            //Send ACK
            manageDataPacket.prepareACK(msgData.status, msgData.timestamp, msgData.cmd, msgData.sub_cmd);
            manageDataPacket.send();

            switch (msgData.msgType)
            {
                case RT_MSG_TYPE::MSG_DATA:

                  if (msgData.status == SENSEI_ERROR_CODE::OK)
                  {
                      uint16_t idxStart;
                      uint16_t idxStop;
                      uint16_t payloadSize;
                      uint16_t nPackets;
                      uint8_t* pAddress;

                      switch(msgData.cmd)
                      {
                          case SENSEI_CMD::GET_VALUE:
                              payloadSize=sizeof(GetSetPin);
                              pAddress=(uint8_t*)&msgData.data.pin;
                          break;

                          case SENSEI_CMD::GET_SYSTEM_STATUS:
                              payloadSize=sizeof(SystemStatus);
                              pAddress=(uint8_t*)&msgData.data.systemStatus;
                          break;

                          case SENSEI_CMD::IMU_GET_SETTINGS:
                              payloadSize=sizeof(sImuSettings);
                              pAddress=(uint8_t*)&msgData.data.imuSettings;
                          break;

                          case SENSEI_CMD::IMU_GET_DATA:
                              //SerialDebug.println("msgData.packetSize=" + String(msgData.packetSize));
                              payloadSize=msgData.packetSize;
                              pAddress=(uint8_t*)&msgData.data.vectorDataImu;
                          break;

                          case SENSEI_CMD::IMU_GET_TEMPERATURE:
                              payloadSize=sizeof(float);
                              pAddress=(uint8_t*)&msgData.data.fValue;
                          break;

                          default:
                              payloadSize=0;
                      }

                      // Send Packets
                      nPackets = static_cast<uint16_t>(ceilf(static_cast<float>(payloadSize) / static_cast<float>(SENSEI_PAYLOAD_LENGTH)));
                      for (uint16_t idxPacket = 0; idxPacket < nPackets; idxPacket++)
                      {
                          idxStart = idxPacket*SENSEI_PAYLOAD_LENGTH;
                          idxStop = (idxPacket + 1)*SENSEI_PAYLOAD_LENGTH - 1;

                          if (idxStop > payloadSize - 1)
                              idxStop = payloadSize - 1;

                          manageDataPacket.preparePacket(msgData.cmd, msgData.sub_cmd, nPackets - idxPacket - 1, pAddress + idxStart, idxStop - idxStart + 1);
                          manageDataPacket.send();
                      }
                  } // if (msgData.status == SENSEI_ERROR_CODE::OK)
                break;

                case RT_MSG_TYPE::MSG_ACK:
                    // Nothing
                break;

            }
        } // xQueueReceive

        //Message PIN from RT to COM
        if ((hQueueRTtoCOM_PIN != 0) && (xQueueReceive(hQueueRTtoCOM_PIN, &msgPin, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE)))
        {
            taskStatus.msgQueueReceived++;
            if (systemSettings.debugMode)
            {
                SerialDebug.println("QueueRTtoCOM_PIN: xQueueReceive");
            }
            // Send PIN VALUE
            manageDataPacket.preparePacket(SENSEI_CMD::VALUE,SENSEI_SUB_CMD::EMPTY, 0, (uint8_t*)&msgPin, sizeof(GetSetPin));
            manageDataPacket.send();
        }

        //Message IMU from RT to COM
        if ((hQueueRTtoCOM_IMU != 0) && (xQueueReceive(hQueueRTtoCOM_IMU, &msgImu, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE)))
        {
            taskStatus.msgQueueReceived++;
            if (systemSettings.debugMode)
            {
                SerialDebug.println("QueueRTtoCOM_IMU: xQueueReceive");
            }
            // Send IMU_VALUE
            manageDataPacket.preparePacket(SENSEI_CMD::VALUE_IMU,msgImu.typeOfData, 0, (uint8_t*)&msgImu.vectorDataImu, msgImu.packetSize);
            manageDataPacket.send();
        }

        //------------------------------------------------------------------------------------------- [PROCESSING DATA FROM RT]
        taskStatus.nCycles++;
    }  // for (;;)
}