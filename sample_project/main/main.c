/**
 * Implementation of the fundemental operations of the Dandelions SMC space mission
 * By Harindi Perera & Jake Sheath
 * 2023
*/
#include <stdio.h>
#include "DandSMC.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include "state.h"
#include "esp_spiffs.h"
#include <stdarg.h>
#include <driver/spi_master.h>
#include "canfdspi/drv_canfdspi_api.h"
#include "spi/drv_spi.h"
#include "canfd/drv_can.h"
#include "msg/msg.h"
#include "md5/md5.h"
#include "isotp/iso_tp.h"

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


void Listen(void *pvParameters){
    spi_device_handle_t spi = *((TaskParams_t*)pvParameters)->spi;
    for (;;) 
    {
        // sleep(1);
        SMC_MESSAGE_HANDLER(&spi);
    }    
}

/*void checkListenState(TaskHandle_t ListenHandle ) {
  while(1) {
    eTaskState state = eTaskGetState(ListenHandle);
    if(state == eDeleted) {
      printf("Task deleted successfully\n");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}*/




//___________________________________________APP__MAIN_______________________________________________

void app_main(void)
{

    // Setup
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

    //SPI CONFIG
    esp_err_t ret;
    spi_device_handle_t spi_0;
    spi_device_handle_t spi_1;
    spi_bus_config_t buscfg_0={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .max_transfer_sz=4096,
        
    };
    
    //SPI DEVICE CONFIG
    spi_device_interface_config_t devcfg_1={
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
        .mode=3,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .command_bits=4,                        //Command Size
        .address_bits=12,                       //Address Size
        .queue_size=100,                        //Queue Size
    };

    

    // patrick main
    

    
    // SMC Test Code for Dandelions side
    //TaskParams_t params = {.spi = spi_1};
    //TaskHandle_t ListenHandle = NULL;
    //TaskHandle_t Task2Handle = NULL; 

    //xTaskCreate(Listen,"Listen",8192,&params,1,&ListenHandle);



    //////////////////////////

    // set the flowFlag to be nominal
    enum flowFlag ctrlFlowFlag = NOMINAL;

    // wait for greenlight 
    while(ctrlFlowFlag != GREENLIGHT){
        // in debug phase check the r key for greenlight 
        if(DEBUG){
            if(getchar()=='r'){
                ctrlFlowFlag = GREENLIGHT;
            }
        }
    
        vTaskDelay(100/portTICK_PERIOD_MS);
        // if no greenlight just chill here. 
    }
    if(ctrlFlowFlag == GREENLIGHT){
        logError("Greenlit For exerpiment\n");
        ctrlFlowFlag = NOMINAL;
        ADC_Pwr(1);
    }
    
    // initialise the i2cBuss
    I2C_Init();

    // initialise the phase, tick and experiment count instantce. 
    int ticks = 0;
    int phase = 0;
    int experimentCount = 0;

    if( ctrlFlowFlag == NOMINAL) {

        getExerpimentPhaseTicks(&phase,&ticks);      // Check the previous phase and ticks 
        if (phase !=0 || ticks !=0){
            printf("main: Phase = %d, Tick = %d\n", phase,ticks);
            neutralise(&phase,&ticks, &ctrlFlowFlag);        // if non-zero then neutralise and dont run experiment
        }else{
            RunExperiment(&phase, &ticks, &ctrlFlowFlag);    // else run the experiment
        }
        setExperimentPhaseTicks(&phase,&ticks,0);        // set the phase and ticks. 
        if ((phase!=0)|(ticks!=0)){
            ctrlFlowFlag = ESD;
            logError("Motor actualtion terminated unexpectely\n"); 
        }
        // This is a 'just in case' condition
        EnMotor(0);         
        ADC_Pwr(0);         

        updateExperimentCount(0,&experimentCount);
    }

    for(;;){  
        vTaskDelay(100/portTICK_PERIOD_MS); // do nothing in the main loop 
    }
}

