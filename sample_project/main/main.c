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
#include "state.h"
#include "esp_spiffs.h"
#include <stdarg.h>
#include <driver/spi_master.h>

/**
 * @brief Task to feed the hardware watchdog every 5 seconds.
 * 
 * This function sets the WDI pin high to feed the hardware watchdog, waits for a settling time,
 * and then sets the WDI pin low to complete the feed. It then gives control back to the scheduler 
 * for 5 seconds before repeating the process. If the function breaks from the loop for some reason,
 * it is deleted and an error message is printed.
 * 
 * @param pvParamemters A pointer to any parameters passed to the function.  
 * @return None.
 */
void hwWDPulseTask(void* pvParamemters){

    //Infinite Loop
    for(;;){

        //rprintf("WD.\n");
        gpio_set_level(WDI,1);                              // Set the HW_watchdog high
        vTaskDelay(WD_DELAY_MS/portTICK_PERIOD_MS);         // settling time
        gpio_set_level(WDI,0);                              // Set the HW_watchdog low
        vTaskDelay(5000/portTICK_PERIOD_MS);                // Control back to scheduler
    }

    vTaskDelete(NULL);                                                          // delete task if it breaks for some reason
    printf("CRITICAL ERROR: hwWDPulseTask broke from loop. Task Deleted\n");    // error msg
}


  

//___________________________________________APP__MAIN_______________________________________________

void app_main(void)
{
    xTaskCreatePinnedToCore(
        hwWDPulseTask,  
        "HWWATCHDOG",   /*Task Name*/
        1024,           /*stackdepth*/
        NULL,           /*pvParameters*/
        16,             /*Priority*/
        NULL,           /*ret handel*/
        1               /*core*/
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

    I2C_Init();


   // char c = '\n';
    printf("R = Run experiment 8min approx\n");
    int ticks = 0;
    int phase = 0;
    int experimentCount = 0;

    for(;;){  
        vTaskDelay(100/portTICK_PERIOD_MS); // do nothing in the main loop 
        
        // If r is pressed then run the experiment 
        if( getchar() == 'r') {

            getExerpimentPhaseTicks(&phase,&ticks);      // Check the previous phase and ticks 
            if (phase !=0 || ticks !=0){
                printf("main: Phase = %d, Tick = %d\n", phase,ticks);
                neutralise(&phase,&ticks);              // if non-zero then neutralise and dont run experiment
            }else{
                RunExperiment(&phase, &ticks);          // else run the experiment
            }
            setExperimentPhaseTicks(&phase,&ticks,0);       // set the phase and ticks. 
            
            
            EnMotor(0);         // Disable Motor After experiment regardless of how it exits
            ADC_Pwr(0);         // Diable ADC Power Just in case 

            updateExperimentCount(0,&experimentCount);

        }else if(getchar() == 't'){
            setExperimentPhaseTicks(&phase,&ticks,1);       // reset set the phase and tick in nvs to 0;
            updateExperimentCount(1,&experimentCount);      // reset the experiment count in nvs to 0
        }else if(getchar() == 'q'){
            esp_restart();
        }
    }
}

