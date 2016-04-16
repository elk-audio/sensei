#include "taskRT.h"

// handles for queue
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

    TaskRtStatus taskStatus;
    memset(&taskStatus,0,sizeof(TaskRtStatus));

    TickType_t xLastWakeTime;
    TickType_t xFrequency = 1; // 1 ms -> 1000Hz //TODO define

    xLastWakeTime = xTaskGetTickCount();
    //type_filter_var FsFilter = DEFAULT_RT_FREQUENCY; //[Hz]

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

        //------------------------------------------------------------------------------------------- [HW]
        // Hardware Polling
        if (manageIO.isSystemInitialized())
        {
            uint16_t idxPin;
            for (uint8_t ch = 0; ch < CHANNELS_PER_MULTIPLEXER; ch++)
            {
                Byte S_Address;
                S_Address.value = ch;

                digitalWrite(S0, S_Address.bit0);
                digitalWrite(S1, S_Address.bit1);
                digitalWrite(S2, S_Address.bit2);
                digitalWrite(S3, S_Address.bit3);


                for (uint8_t idxMul = 0; idxMul < manageIO.getNmultiplexer(); idxMul++)
                {
                    idxPin = ch + idxMul * CHANNELS_PER_MULTIPLEXER;

                    switch(manageIO.getPinType(idxPin))
                    {
                       case ePinType::PIN_DISABLE:
                       pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT);
                       break;

                       case ePinType::PIN_DIGITAL_INPUT:
                       delayMicroseconds(1); //to define
                       pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT_PULLUP);
                       manageIO.setPinValue(idxPin,digitalRead(Z1 + VERSOR_Z_PINS * idxMul));
                       break;

                       case ePinType::PIN_DIGITAL_OUTPUT:
                       pinMode(Z1 + VERSOR_Z_PINS * idxMul, OUTPUT);
                       delayMicroseconds(1);
                       digitalWrite(Z1 + VERSOR_Z_PINS * idxMul,static_cast<bool>(manageIO.getPinValue(idxPin)));
                       delayMicroseconds(1);
                       digitalWrite(Z1 + VERSOR_Z_PINS * idxMul,LOW);
                       pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT_PULLDOWN);
                       break;

                       case ePinType::PIN_ANALOG_INPUT:
                       pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT);
                       delayMicroseconds(1);
                       manageIO.setPinValue(idxPin,analogRead(Z1 + VERSOR_Z_PINS * idxMul));
                       break;
                    }

                    if ((systemSettings.enableSendingPackets) && (manageIO.isPinValueChanged(idxPin))) //(manageIO.getPinType(idxPin)!=ePinType::PIN_DISABLE)
                    {
                        msgPin.pin.idx=idxPin;
                        msgPin.pin.value=manageIO.getPinValue(idxPin);
                        msgPin.pin.type=manageIO.getPinType(idxPin);

                        if (xQueueSend(hQueueRTtoCOM_PIN,(void*)&msgPin,(TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_RT_TO_COM) != pdPASS)
                        {
                            if (DEBUG) SerialDebug.println("QueueRTtoCOM_PIN: xQueueSend");
                            taskStatus.msgQueueSendErrors++;
                        }
                    }
                } //Mul
            } //Ch
        } //manageIO.isSystemInitialized()
        //------------------------------------------------------------------------------------------- [HW]

        //------------------------------------------------------------------------------------------- [CMD_COM]
        // CMD FROM COM
        if ((hQueueCOMtoRT_DATA != 0) && (xQueueReceive(hQueueCOMtoRT_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_RECEIVE)))
        {
            taskStatus.msgQueueReceived++;
            msgData.msgType=RT_MSG_TYPE::ACK;

            if (systemSettings.debugMode) SerialDebug.println("QueueRTtoCOM_DATA: xQueueReceive");

            //---------------------------------------------------------------------
            // START COMMANDS
            //---------------------------------------------------------------------
            switch (msgData.cmd)
            {
                //--------------------------------------------------------------------- [CMD INITIALIZE_SYSTEM]
                case SENSEI_CMD::INITIALIZE_SYSTEM:
                msgData.status=manageIO.setSystem(msgData.data.hw.nPin,msgData.data.hw.nDigitalPin);
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_SENDING_PACKETS]
                case SENSEI_CMD::ENABLE_SENDING_PACKETS:
                systemSettings.enableSendingPackets=static_cast<bool>(msgData.data.value);
                msgData.status=SENSEI_ERROR_CODE::OK;
                if (1) SerialDebug.println("enableSendingPackets= " + String(systemSettings.enableSendingPackets));
                break;

                //--------------------------------------------------------------------- [CMD ENABLE_MULTIPLE_PACKETS]
                case SENSEI_CMD::ENABLE_MULTIPLE_PACKETS:
                systemSettings.enableMultiplePackets=static_cast<bool>(msgData.data.value);
                msgData.status=SENSEI_ERROR_CODE::OK;
                break;

                //--------------------------------------------------------------------- [CMD CONFIGURE_PIN]
                case SENSEI_CMD::CONFIGURE_PIN:
                if (systemSettings.debugMode)
                {
                    SerialDebug.println("------------------------------------");
                    SerialDebug.println("RT: CONFIGURE_PIN");
                    SerialDebug.println("------------------------------------");
                    SerialDebug.println("idxPin=" + String(msgData.data.setupPin.idxPin));
                    SerialDebug.println("pinType=" + String(msgData.sub_cmd));
                    SerialDebug.println("sendingMode=" + String(msgData.data.setupPin.sendingMode));
                    SerialDebug.println("deltaTicksContinuousMode=" + String(msgData.data.setupPin.deltaTicksContinuousMode));
                    SerialDebug.println("ADCBitResolution=" + String(msgData.data.setupPin.ADCBitResolution));
                    SerialDebug.println("filterOrder=" + String(msgData.data.setupPin.filterOrder));
                    SerialDebug.println("sliderMode=" + String(msgData.data.setupPin.sliderMode));
                    SerialDebug.println("sliderThreshold=" + String(msgData.data.setupPin.sliderThreshold));
                    SerialDebug.println("------------------------------------");
                    SerialDebug.println("");
                }
                if (manageIO.isSystemInitialized())
                {
                    switch (msgData.sub_cmd) //pinType
                    {
                        case SENSEI_SUB_CMD::SET_PIN_DISABLE:
                        case SENSEI_SUB_CMD::SET_PIN_DIGITAL_INPUT:
                        case SENSEI_SUB_CMD::SET_PIN_DIGITAL_OUTPUT:
                        msgData.status=manageIO.configurePin(static_cast<ePinType>(msgData.sub_cmd), &msgData.data.setupPin);
                        break;

                        case SENSEI_SUB_CMD::SET_PIN_ANALOG_INPUT:

                        msgData.status=manageIO.configurePin(ePinType::PIN_ANALOG_INPUT, &msgData.data.setupPin);

                        delete[] msgData.data.setupPin.filterCoeff_a;
                        delete[] msgData.data.setupPin.filterCoeff_b;

                        break;

                        default:
                        msgData.status = SENSEI_ERROR_CODE::PIN_TYPE_NOT_VALID;
                        break;
                    }
                }
                else
                {
                    msgData.status=SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;
                }
                break;

                //--------------------------------------------------------------------- [CMD SET_DIGITAL_PINS]
                case SENSEI_CMD::SET_DIGITAL_PINS:
                switch (msgData.sub_cmd)
                {
                    case SET_PIN:
                    msgData.status = manageIO.setDigitalPin(msgData.data.pin.idx, static_cast<bool>(msgData.data.pin.value));

                    if (systemSettings.debugMode) SerialDebug.println("SET_DIGITAL_PINS: pin=" + String(msgData.data.pin.idx) + " value=" + String(static_cast<bool>(msgData.data.pin.value)));
                    break;

                    case SET_BANK:
                    msgData.status = manageIO.setDigitalBank(msgData.data.pin.idx, static_cast<bool>(msgData.data.pin.value));

                    if (systemSettings.debugMode) SerialDebug.println("SET_DIGITAL_PINS: idxBank=" + String(msgData.data.pin.idx) + " value=" + String(static_cast<bool>(msgData.data.pin.value)));
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
                    if (systemSettings.debugMode) SerialDebug.println("SET_SINGLE_PIN: pin=" + String(msgData.data.pin.idx) + " value=" + String(static_cast<bool>(msgData.data.pin.value)));
                    msgData.status = manageIO.setPinValue(msgData.data.pin.idx,msgData.data.pin.value);
                    break;

                    default:
                    msgData.status = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                    break;
                }
                break;

                //--------------------------------------------------------------------- [CMD GET_VALUE]
                case SENSEI_CMD::GET_VALUE:

                switch (msgData.sub_cmd)
                {
                    case SENSEI_SUB_CMD::GET_SINGLE_PIN:
                    msgData.msgType = RT_MSG_TYPE::DATA;
                    msgData.data.pin.type=manageIO.getPinType(msgData.data.pin.idx);
                    uint16_t value;
                    msgData.status=manageIO.getPinValue(msgData.data.pin.idx,value);
                    msgData.data.pin.value=value;
                    break;

                    default:
                    msgData.status = SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID;
                }

                break;

                //---------------------------------------------------------------------
                // END COMMANDS
                //---------------------------------------------------------------------
                default:
                msgData.status=SENSEI_ERROR_CODE::CMD_NOT_VALID;
            } //switch (msgCmd.cmd)



            //Message DATA to COM
            if ((hQueueRTtoCOM_DATA != 0) && (xQueueSend(hQueueRTtoCOM_DATA, &msgData, (TickType_t)MSG_QUEUE_MAX_TICKS_WAIT_TO_SEND_COM_TO_RT) != pdPASS))
            {
                if (DEBUG) SerialDebug.println("QueueRTtoCOM_DATA: msgQueueSendErrors");
                taskStatus.msgQueueSendErrors++;
            }


        } //hQueueCOMtoRT_CMD
        //------------------------------------------------------------------------------------------- [CMD_COM]

        taskStatus.nCycles++;

    } //for(;;)
}
