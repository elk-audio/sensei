#ifndef MACRO_PRINT_H
#define MACRO_PRINT_H

//http://c.learncodethehardway.org/book/ex20.html

#define PRINT_PIN_CONFIGURATION_RT                                                                                      \
{                                                                                                                       \
if (systemSettings.debugMode)                                                                                           \
{                                                                                                                       \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("RT: CONFIGURE_PIN");                                                                           \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("idxPin=" + String(msgData.data.setupPin.idxPin));                                              \
    SerialDebug.println("pinType=" + String(msgData.sub_cmd));                                                          \
    SerialDebug.println("sendingMode=" + String(msgData.data.setupPin.sendingMode));                                    \
    SerialDebug.println("deltaTicksContinuousMode=" + String(msgData.data.setupPin.deltaTicksContinuousMode));          \
    SerialDebug.println("ADCBitResolution=" + String(msgData.data.setupPin.ADCBitResolution));                          \
    SerialDebug.println("filterOrder=" + String(msgData.data.setupPin.filterOrder));                                    \
    SerialDebug.println("sliderMode=" + String(msgData.data.setupPin.sliderMode));                                      \
    SerialDebug.println("sliderThreshold=" + String(msgData.data.setupPin.sliderThreshold));                            \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("");                                                                                            \
}                                                                                                                       \
}                                                                                                                       \

#define PRINT_PIN_CONFIGURATION_COM                                                                                     \
{                                                                                                                       \
if (systemSettings.debugMode)                                                                                           \
{                                                                                                                       \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("COM: CONFIGURE_PIN");                                                                          \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("pinType=" + String(sub_cmd));                                                                  \
    SerialDebug.println("idxPin=" + String(pinConfiguration.idxPin));                                                   \
    SerialDebug.println("sendingMode=" + String(pinConfiguration.sendingMode));                                         \
    SerialDebug.println("deltaTicksContinuousMode=" + String(pinConfiguration.deltaTicksContinuousMode));               \
    SerialDebug.println("ADCBitResolution=" + String(pinConfiguration.ADCBitResolution));                               \
    SerialDebug.println("filterOrder=" + String(pinConfiguration.filterOrder));                                         \
    SerialDebug.println("lowPassCutOffFilter=" + String(pinConfiguration.lowPassCutOffFilter));                         \
    SerialDebug.println("sliderMode=" + String(pinConfiguration.sliderMode));                                           \
    SerialDebug.println("sliderThreshold=" + String(pinConfiguration.sliderThreshold));                                 \
    SerialDebug.println("------------------------------------");                                                        \
    SerialDebug.println("");                                                                                            \
}                                                                                                                       \
}                                                                                                                       \

#endif // MACRO_H
