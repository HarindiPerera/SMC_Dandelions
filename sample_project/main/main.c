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

    vTaskDelete(NULL);                                                          // delete task if it breaks for some reason
    printf("CRITICAL ERROR: hwWDPulseTask broke from loop. Task Deleted\n");    // error msg
}

void experimentTask(void*pvParameters){

    BaseType_t xStatus;
    //Queue Info
    char c  = 'p'; 

    for(;;){
        //printf("the Experiment task is blocked after the line\n");
        xStatus = xQueueReceive(experimentQueue, &c, portMAX_DELAY);        //Blocking function    
        if (xStatus == pdPASS){
            
            //printf("RUN:    Data received from the queue is : %c\n",c);     //Successful data recieved from queue

            printf("Experiment task: Start of Experiment\n\n");
            // Read 2 ADC 
            ADC_Read(ADC_ADDR_1);
            ADC_Read(ADC_ADDR_2);

        }
        else{
            printf("RUN:    Data cannot be read through queue");
        }
        //Time Delay
        vTaskDelay(10/portTICK_PERIOD_MS); 
    }

}

/**
 * @brief Task to handle triggering of experiments based on input received from the user.
 * 
 * This task listens for input from the user via the serial console and performs
 * experiments based on the input received. The experiments include restarting the ESP32,
 * running the motor in the forward or backward direction, and reading two ADC values
 * @param pvParamemters A pointer to any parameters passed to the function.  
 * @return None.
 */
void triggerTask(void*pvParameters){

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
        if (c == 'f') {
            printf("Quick Experiment - Forward\n");

            // Delay for a short period to allow any pending tasks to complete.
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Run Motor
            
            int Mot_err = RunMotor(1,TICKS_PER_REV); 
            printf("Restart task ERR : %d\n" , Mot_err );
        

            // Read 2 ADC 
            ADC_Read(ADC_ADDR_1);
            ADC_Read(ADC_ADDR_2);               


            //Send to Queue
            xQueueSendToBack(experimentQueue, &c, portMAX_DELAY);
        }
        if (c == 'b') {
            printf("Quick Experiment - Backward\n");

            // Delay for a short period to allow any pending tasks to complete.
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Run Motor
            
            int Mot_err = RunMotor(0,TICKS_PER_REV); 
            printf("Restart task ERR : %d\n" , Mot_err );
        

            // Read 2 ADC 
            ADC_Read(ADC_ADDR_1);
            ADC_Read(ADC_ADDR_2);


            //Send to Queue
            xQueueSendToBack(experimentQueue, &c, portMAX_DELAY);
        }
        if (c == 'e') {
            printf("Quick Experiment - Read and Log Data\n");

            // Delay for a short period to allow any pending tasks to complete.
            vTaskDelay(100 / portTICK_PERIOD_MS);
            
            // Read 2 ADC 
            ADC_Read(ADC_ADDR_1);
            ADC_Read(ADC_ADDR_2);
            

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

    ////Hardware Takeover
    printf("Setting the gpios externally without the masks\n");

    //set pins correctly to inputs and outs 
    gpio_set_direction(MVEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MSLEEP, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTSTEP, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTDIR, GPIO_MODE_OUTPUT);

    //[Software Setup]
 
    ADC_Pwr(1);                 //Power the lads
    I2C_Init();                 //Init dem comms
    I2C_Scan();                 //Scan for I2C devices on the bus
    
    experimentQueue = xQueueCreate( QUEUE_LENGTH, ITEM_SIZE );

    //[Task Creation]
    xTaskCreatePinnedToCore(hwWDPulseTask, /*Task Name*/ "HWWATCHDOG", /*stackdepth*/ 1024, /*pvParameters*/ NULL,  /*Priority*/ 1, /*ret handel*/NULL, /*core*/0);                
    xTaskCreatePinnedToCore(experimentTask,"Experiment",8096,NULL,2,NULL,0);
    xTaskCreatePinnedToCore(triggerTask,"Trigger",8096,NULL,1, NULL ,0);

    for(;;){  
        vTaskDelay(1000/portTICK_PERIOD_MS); // do nothing in the main loop 
    }

    

}

