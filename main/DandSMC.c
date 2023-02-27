#include "DandSMC.h"
#include "esp_err.h"
#include "driver/gpio.h"

// FUNCTIONS 

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