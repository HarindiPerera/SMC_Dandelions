/**
 * Implementation of the fundemental operations of the Dandelions SMC space mission
 * By Harindi Perera & Jake Sheath and Patrick Oppel
 * 2023
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <driver/spi_master.h>
#include <inttypes.h>
#include "DandSMC.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "state.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"
#include "canfdspi/drv_canfdspi_api.h"
#include "canfd/drv_can.h"
#include "msg/msg.h"
#include "md5/md5.h"
#include "isotp/iso_tp.h"

// SemaphoreHandle_t exclusiveAccessMutex = NULL;
TaskHandle_t ListenHandle = NULL;
IRAM_ATTR TaskParams_t* params;

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

// establish the listening task
void Listen(void *pvParameters){

    TaskParams_t* task = (TaskParams_t*)pvParameters; 
    spi_device_handle_t spi = task->spi;
    for(;;){
        vTaskDelay(10/portTICK_PERIOD_MS);
        // xSemaphoreTake(exclusiveAccessMutex, portMAX_DELAY);        
        SMC_MESSAGE_HANDLER(&spi,&(task->ctrlFlowFlag)); 
        // xSemaphoreGive(exclusiveAccessMutex);
    }    
}

void checkListenState(TaskHandle_t ListenHandle ) {
  while(1) {
    eTaskState state = eTaskGetState(ListenHandle);
    if(state == eDeleted) {
      printf("Task deleted successfully\n");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

//___________________________________________APP__MAIN_______________________________________________


void app_main(void)
{
    // Create hardware Watchdog Task
    xTaskCreatePinnedToCore(
        hwWDPulseTask,  
        "HWWATCHDOG",   /*Task Name*/
        1024,           /*stackdepth*/
        NULL,           /*pvParameters*/
        16,             /*Priority = HIGHEST*/
        NULL,           /*ret handel*/
        1               /*core*/
    ); 


    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    printf("\n");     
    
    // mutex protects the writing to CAN funcation 
    // exclusiveAccessMutex = xSemaphoreCreateMutex();
    // initialise all the hardware. Restart if no good
    
    if(setupHW()!= ESP_OK){
        printf("SETUP ERROR: Restarting...\n");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        esp_restart();
    }
    // I2C Init
    ADC_Pwr(1);       
    I2C_Init();
    ADC_Pwr(0);

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
        .clock_speed_hz=5*1000*1000,           //Clock out at 5 MHz
        .mode=0,                                //SPI mode 0,0
        .spics_io_num=PIN_NUM_CS,               //CS pin 15
        .command_bits=4,                        //Command Size
        .address_bits=12,                       //Address Size
        .queue_size=100,                        //Queue Size
    };

    // initialise the CAN and SPI buses 
    spi_dma_chan_t dma_chan = SPI_DMA_CH_AUTO;
    ESP_ERROR_CHECK(spi_bus_initialize(CANSPI_HOST, &buscfg_0, dma_chan));      //Initialize the HSPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(CANSPI_HOST, &devcfg_1, &spi_1));        //Attach the device to the SPI bus
    printf("\n");

    vTaskDelay(4000/portTICK_PERIOD_MS);    // Delay
    printf("OP Mode 0:%X\n",DRV_CANFDSPI_OperationModeGet(&spi_1));
    ESP_ERROR_CHECK(DRV_CANFDSPI_OperationModeSelect(&spi_1, CAN_CONFIGURATION_MODE));    // make sure MCP is in Configuration Mode at start-up

    vTaskDelay(4000/portTICK_PERIOD_MS);    // Delay

    DRV_CAN_INIT(&spi_1);
    SMC_FILTER_CONFIG(&spi_1);

    vTaskDelay(1000/portTICK_PERIOD_MS);
    printf("CAN Initialisation finished\n");

    // Enable ECC
    // ESP_ERROR_CHECK(DRV_CANFDSPI_EccEnable(spi_0));
    ESP_ERROR_CHECK(DRV_CANFDSPI_EccEnable(&spi_1));
    // Initialize RAM
    // ESP_ERROR_CHECK(DRV_CANFDSPI_RamInit(spi_0, 0xff));
    ESP_ERROR_CHECK(DRV_CANFDSPI_RamInit(&spi_1, 0xff));
    // Configuration Done: Select Normal Mode
    ESP_ERROR_CHECK(DRV_CANFDSPI_OperationModeSelect(&spi_1, CAN_NORMAL_MODE));
    // ESP_ERROR_CHECK(DRV_CANFDSPI_OperationModeSelect(&spi_1, CAN_EXTERNAL_LOOPBACK_MODE));
    
    vTaskDelay(1000/portTICK_PERIOD_MS);
    printf("OP Mode 0:%X\n",DRV_CANFDSPI_OperationModeGet(&spi_1));

    // Begin the listen task
    TaskParams_t params = {.spi = spi_1, .ctrlFlowFlag = NOMINAL};
    xTaskCreatePinnedToCore(
       Listen,  
       "LISTEN",       /*Task Name*/
       8192,           /*stackdepth*/
       &params,        /*pvParameters*/
       15,              /*Priority*/
       &ListenHandle,  /*ret handel*/
       1               /*core*/
    ); 

    // Wait for Greenlight 
    printf("Waiting for greenlight\n");
    while(params.ctrlFlowFlag != GREENLIGHT){
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

    int i = 'x';
    uint8_t nil =0;

    while (i=='x') {
       
        switch(i) {
            case 'x':
                if(DEBUG){printf("Send: BEGINNING\n");}
                vTaskDelay(100/portTICK_PERIOD_MS);
                DRV_CAN_WRITE(&spi_1, &nil, BEGINNING, CAN_DLC_64);
                params.ctrlFlowFlag = NOMINAL;
                i =0;
                break;
            default:
            break;
        }
    }

    // initialise the phase, tick and experiment count instantce. 
    int ticks = 0;
    int phase = 0;
    int experimentCount = 0;

    if( params.ctrlFlowFlag == NOMINAL) {
        // // printf("This is the experiment running\n");
        // getExerpimentPhaseTicks(&phase,&ticks);      // Check the previous phase and ticks 
        // if (phase !=0 || ticks !=0){
        //     printf("main: Phase = %d, Tick = %d\n", phase,ticks);
        //     neutralise(&phase,&ticks, &params.ctrlFlowFlag);        // if non-zero then neutralise and dont run experiment
        // }else{
        //     RunExperiment(&phase, &ticks, &params.ctrlFlowFlag);    // else run the experiment
        // }
        // setExperimentPhaseTicks(&phase,&ticks,0);        // set the phase and ticks. 
        // if ((phase!=0)|(ticks!=0)){
        //     params.ctrlFlowFlag = ESD;
        //     logError("Motor actualtion terminated unexpectely\n"); 
        // }
        // // This is a 'just in case' condition
        // EnMotor(0);         
        // ADC_Pwr(0);         
        // updateExperimentCount(0,&experimentCount);
    }


// End Condition loop

    printf("End Condition: \n");
    printf("1 = BEGIN\n");
    printf("2 = CEASE\n");
    printf("3 = POWDWN_ALL\n");
    printf("4 = POWDWN\n");
    printf("5 = QUERY\n");

    for(;;){  

       scanf("%d",&i);

        switch (i) {
            case 1:
                printf("BEGIN\n");
                DRV_CAN_WRITE(&spi_1, &nil, BEGIN, CAN_DLC_64);
                // wait for reply 
                break;
            case 2:
                printf("CEASE\n");
                DRV_CAN_WRITE(&spi_1, &nil, CEASE, CAN_DLC_64);
                
                // wait for reply 
                break;
            case 3:
                printf("POWDWN_ALL\n");
                DRV_CAN_WRITE(&spi_1, &nil, POWDWN_ALL, CAN_DLC_64);
                
                // wait for reply 
                break;
            case 4:
                printf("POWDWN\n");
                DRV_CAN_WRITE(&spi_1, &nil, POWDWN, CAN_DLC_64);
                // wait for reply 
                break;
            case 5:
                printf("QUERY\n");
                DRV_CAN_WRITE(&spi_1, &nil, QUERY, CAN_DLC_64);
                // wait for reply 
                break;
            default:
                //printf(".");
                break;
        }
        i = 'x';

        
        vTaskDelay(100/portTICK_PERIOD_MS); // do nothing in the main loop 
    }
}

