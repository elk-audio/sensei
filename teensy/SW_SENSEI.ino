// TODO
// - max filter order
// - INPUT - OUTPUT classes
// - filterCoeff, Butterworth class
// -
// -

//#pragma GCC optimize ("-Ofast")
//teensy31.menu.speed.96.build.flags.optimize = -Ofast
//teensy31.build.flags.cpu=-mthumb -mcpu=cortex-m4 -mfpu=neon -fsingle-precision-constant

#include <Arduino.h>
#include "common.h"

#include "taskRT.h"
#include "taskCOM.h"
//#include "taskLED.h"

namespace std {
    void __throw_bad_alloc() {
        SerialDebug.println("Unable to allocate memory");
        while (1);
    }
    void __throw_length_error(char const *e) {
        SerialDebug.print("Length Error :");
        SerialDebug.println(e);
        while (1);
    }
}

// handles for tasks
TaskHandle_t hTaskRT = 0;
TaskHandle_t hTaskCOM = 0;
TaskHandle_t hTaskLED = 0;

// handles for queue
QueueHandle_t hQueueRTtoCOM_DATA = 0;
QueueHandle_t hQueueCOMtoRT_DATA = 0;
QueueHandle_t hQueueRTtoCOM_PIN = 0;
QueueHandle_t hQueueRTtoCOM_IMU = 0;


void setup()
{
    //Hardware
    pinMode(DS, OUTPUT);
    pinMode(ST, OUTPUT);
    pinMode(SH, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(SPI_SS, OUTPUT);
    pinMode(Z1, INPUT);
    pinMode(Z2, INPUT);
    pinMode(Z3, INPUT);
    pinMode(Z4, INPUT);
    pinMode(INT_FILTER, INPUT);

    //Disable Shift Register
    digitalWrite(DS, 0);
    digitalWrite(SH, 0);
    digitalWrite(ST, 1);
    digitalWrite(SPI_SS, LOW);

    //Disable SPI Chip Select
    digitalWrite(SPI_SS, HIGH);

    // Reset Shift register
    digitalWrite(ST, 0);
    for (uint8_t i=0;i<16;i++)
    {
        shiftOut(DS, SH, LSBFIRST, 0x00);
    }
    digitalWrite(ST, 1);

    //Serial communication
    Serial.begin(115200);
    Serial.setTimeout(250);

    SerialDebug.begin(115200);
    SerialDebug.setTimeout(250);

    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setSCK(SPI_SCK);
    SPI.begin();

    //ADC
    analogReadResolution(12);
    analogReadAveraging(1);
    analogReference(EXTERNAL);
    digitalWrite(SPI_SS, HIGH);

    // Startup
    for (uint8_t idx = 0; idx < 10; idx++)
    {
        digitalWrite(STATUS_LED, 1);
        delay(100);
        digitalWrite(STATUS_LED, 0);
        delay(50);
    }

    //TODO [!]
    xTaskCreate(vTaskRT,"task_RT",STACK_SIZE,NULL,configMAX_PRIORITIES-2,&hTaskRT); //-1
    xTaskCreate(vTaskCOM,"task_COM",STACK_SIZE,NULL,configMAX_PRIORITIES-2,&hTaskCOM);
    xTaskCreate(vTaskLED,"task_LED",configMINIMAL_STACK_SIZE,NULL,configMAX_PRIORITIES-2,&hTaskLED);

    SerialDebug.println(" -----------------------------");
    SerialDebug.println("|         SW_SENSEI           |");
    SerialDebug.println(" -----------------------------");
    SerialDebug.println("CPU frequency: " + String(F_CPU / 1000000) + " MHz");
    SerialDebug.print(__DATE__);
    SerialDebug.print(" ");
    SerialDebug.println(__TIME__);
    SerialDebug.println("!!! FREERTOS:checkPriority !!!");
    SerialDebug.println(" -----------------------------");

    delay(100);

    vTaskStartScheduler();

    while (1);

}

void loop()
{

}