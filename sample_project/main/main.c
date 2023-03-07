
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"

//QUEUE
QueueHandle_t experimentQueue = NULL;

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

    //Queue Info
    char c = 'p';

    //Infinite Loop
    for(;;){

        printf("Watchdog fed\n");

        gpio_set_level(WDI,1);                              // Set the HW_watchdog high
        vTaskDelay(WD_DELAY_MS/portTICK_PERIOD_MS);         // settling time
        gpio_set_level(WDI,0);                              // Set the HW_watchdog low
        vTaskDelay(5000/portTICK_PERIOD_MS);                // Control back to scheduler

        //Send to Queue
        xQueueSend(experimentQueue, &c, portMAX_DELAY);
    }
}


void experimentTask(void*pvParameters){

    BaseType_t xStatus;
    //Queue Info
    char c  = 'p'; 

    for(;;){
        printf("the Experiment task is blocked after the line\n");
        xStatus = xQueueReceive(experimentQueue, &c, portMAX_DELAY);        //Blocking function    
        if (xStatus == pdPASS){
            
            printf("RUN:    Data received from the queue is : %c\n",c);     //Successful data recieved from queue

            //Start of Experiment
            RunMotor(1,TICKS_PER_REV);  
            ADC_Read();
        }
        else{
            printf("RUN:    Data cannot be read through queue");
        }
        //Time Delay
        vTaskDelay(10/portTICK_PERIOD_MS); 
    }

}


void restartTask(void*pvParameters){
    char c = 'p';
    for (;;) {
        
        c = getchar();
        if (c == 'r') {
            printf("Restarting ESP32...\n");

            // Delay for a short period to allow any pending tasks to complete.
            vTaskDelay(100 / portTICK_PERIOD_MS);

            // Restart the ESP32 
            esp_restart();

            //Send to Queue
            xQueueSendToBack(experimentQueue, &c, portMAX_DELAY);
        }

        //Time Delay
        vTaskDelay(10/portTICK_PERIOD_MS); 
    }


}



//___________________________________________APP__MAIN_______________________________________________

void app_main(void)
{

    //[HW Setup check]
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

    experimentQueue = xQueueCreate( QUEUE_LENGTH, ITEM_SIZE );

    //[Task Creation]
    xTaskCreatePinnedToCore(hwWDPulseTask, /*Task Name*/ "HWWATCHDOG", /*stackdepth*/ 1024, /*pvParameters*/ NULL,  /*Priority*/ 1, /*ret handel*/NULL, /*core*/0);                
    xTaskCreatePinnedToCore(experimentTask,"Experiment",4048,NULL,2,NULL,0);
    xTaskCreatePinnedToCore(restartTask,"Restart",1024,NULL,1,NULL,0);


    for(;;){  
        vTaskDelay(1000/portTICK_PERIOD_MS); // do nothing in the main loop 
    }

}

