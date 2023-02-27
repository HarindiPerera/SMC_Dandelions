#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"

QueueHandle_t experimentQueue = NULL;

void app_main(void)
{
    printf("ello");
    printf("new comment added to understand what build ");
}
