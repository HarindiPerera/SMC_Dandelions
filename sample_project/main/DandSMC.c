
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"

/**
 *@brief Configures GPIO pins for input and output
 *This function initializes GPIO pins for input and output as specified by theioConfig parameter. The outputs are configured with no interrupts enabled,
set as outputs with a defined output bit mask, and with pulldown mode enabled
and pullup mode disabled.
*@return [gpio_config_t] Returns ESP_OK on success, otherwise an error code indicating
the cause of the failure.
 */
esp_err_t setupHW(void){

    gpio_config_t ioConfig = {};    // zero initialise the structure
    esp_err_t rtn;  

    // Set up the outputs
    ioConfig.intr_type = GPIO_INTR_DISABLE;     // disable interupts
    ioConfig.mode = GPIO_MODE_OUTPUT;           // Set as outputs
    ioConfig.pin_bit_mask = OUTPUT_BIT_MASK;    // Defined in DandSMC.c
    ioConfig.pull_down_en = 1;                  // Enable pulldown mode
    ioConfig.pull_up_en  = 0;                   // disable pullup mode. 
    rtn = gpio_config(&ioConfig);               // commit these output configuations.

    // check to make sure that all was fine. 
    if(rtn != ESP_OK ){
        printf("FUNC-> setupHW()-> ERROR: " );
        printf(esp_err_to_name(rtn));
        printf("\n");
        return rtn;
    }
    ioConfig.intr_type = GPIO_INTR_DISABLE;     // This might chage in the future
    ioConfig.pin_bit_mask = INPUT_BIT_MASK;     // Defined in DandSMC.h
    ioConfig.mode = GPIO_MODE_INPUT;            // Set to inputs
    ioConfig.pull_down_en = 1;                  // enable pull down
    ioConfig.pull_up_en = 0;                // disable pull up


    return gpio_config(&ioConfig);
}

/**
 *@brief prints a message to the console
 *Super basic print of "hey now" for debugging
*@return None
 */
void print_check(void)
{
    printf("hey now");
}



