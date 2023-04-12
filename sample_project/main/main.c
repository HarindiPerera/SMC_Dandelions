/**
 * Implementation of the fundemental operations of the Dandelions SMC space mission
 * By Harindi Perera & Jake Sheath
 * 2023
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>


// This is the entry point of the scheduler
void app_main(void)
{
    xTaskCreatePinnedToCore(
        hwWDPulseTask,  
        "HWWATCHDOG",   /*Task Name*/
        1024,           /*stackdepth*/
        NULL,           /*pvParameters*/
        16,             /*Priority*/
        NULL,           /*ret handel*/
        0               /*core*/
    );                

    // initilise nvr partition
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    printf("\n");

    // initialise all the hardware. Restart if no good
    if(setupHW()!= ESP_OK){
        printf("SETUP ERROR: Restarting...\n");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        esp_restart();
    }

    //[Software Setup]
    print_check();
    ADC_Pwr(1);     //Power the lads
    I2C_Init();     //Init dem comms
    I2C_Scan();     //Scan for I2C devices on the bus

   /*Set up the CAN BUS
        // [J] dunnon  what needs to happen here
   */

    // Currently this is the way to navigate throught the 'STATES'
    printf("Run (r)\nQuit (q)\nReset NVS (e)\n");
    bool done = false;
    char c;
    while(!done){
        c = getchar();

        if(c =='r'){ 
            // run the experiment
            RunExperiment();   
            updateExperimentCount(0);   // parameter is true for reset false for increment
        }else if(c =='q') {
            printf("Restaring ESP...\n");
            esp_restart();
        }else if (c =='e'){
            updateExperimentCount(1);   // Erase the experiment count
        }

        
        vTaskDelay(1000/portTICK_PERIOD_MS); // do nothing in the main loop 
    }
}

