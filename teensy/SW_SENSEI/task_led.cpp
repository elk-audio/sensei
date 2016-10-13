#include "task_led.h"

using namespace sensei;

void vTaskLED(void *pvParameters)
{
    pinMode(STATUS_LED, OUTPUT);

    for (;;)
    {
        digitalWrite(STATUS_LED, HIGH);
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
        digitalWrite(STATUS_LED, LOW);
        vTaskDelay((999L * configTICK_RATE_HZ) / 1000L);
    }
}
