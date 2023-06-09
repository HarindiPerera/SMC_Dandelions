/**
* This program has been desisgned to act as the OBC of the SMC mission.
* Need to understand why this is not receiving any more 
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

// implement the listen task
void Listen(void *pvParameters){
    TaskParams_t* task = (TaskParams_t*)pvParameters; 
    spi_device_handle_t spi = task->spi;
    for(;;){
        vTaskDelay(10/portTICK_PERIOD_MS);
        SMC_MESSAGE_HANDLER(&spi,&(task->ctrlFlowFlag));
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
        .clock_speed_hz=5*1000*1000,           //Clock out at 5 MHz
        .mode=0,                                //SPI mode 0,0
        .spics_io_num=PIN_NUM_CS,               //CS pin 15
        .command_bits=4,                        //Command Size
        .address_bits=12,                       //Address Size
        .queue_size=100,                        //Queue Size
    };

    
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
    //ESP_ERROR_CHECK(DRV_CANFDSPI_OperationModeSelect(&spi_1, CAN_INTERNAL_LOOPBACK_MODE));
    
    TaskParams_t params = {.spi = spi_1, .ctrlFlowFlag = NOMINAL};
    vTaskDelay(1000/portTICK_PERIOD_MS);

        xTaskCreatePinnedToCore(
       Listen,  
       "LISTEN",       /*Task Name*/
       8192,           /*stackdepth*/
       &params,        /*pvParameters*/
       15,             /*Priority*/
       &ListenHandle,  /*ret handel*/
       1               /*core*/
    ); 
    printf("OP Mode 0:%X\n",DRV_CANFDSPI_OperationModeGet(&spi_1));
    printf("Nominal: HOST COMMANDS\n");
    printf("1 = X_BEG_OP\n");       // Begin Opperations
    printf("2 = X_QDATA\n");        // Query if data is ready for xmission
    printf("3 = X_STOP\n");         // Cease Opperation
    printf("4 = X_TX_DATA\n");      // Host is ready for data transmission 
    printf("5 = X_ALL_PDOWN\n");    // Power down all payloads
    printf("6 = X_PDOWN\n");        // Powerdown our payload
    printf("7 = X_TX_ACK\n");       
    printf("8 = X_ISOTP\n");
    
    uint8_t nil =0;
    int i;

// Main function loop. 
    for(;;){  
       
       scanf("%d",&i);

        switch (i) {
            case 1:
                printf("Sent: X_BEG_OP\n");
                DRV_CAN_WRITE(&spi_1, &nil, X_BEG_OP, CAN_DLC_64);
                // Payload should reply P_BEG_OP = Beginning opperations
                break;
            case 2:
                printf("Sent: X_QDATA\n");    // Do you have data?
                DRV_CAN_WRITE(&spi_1, &nil, X_QDATA, CAN_DLC_64);
                // Payload should respond P_QDATA_RSP indicating data is ready
                break;
            case 3:
                printf("Sent: X_STOP\n");
                DRV_CAN_WRITE(&spi_1, &nil, X_STOP, CAN_DLC_64);
                // Payload should reply P_STOP = I have ceased opperations
                break;
            case 4: 
                printf("Sent: X_TX_DATA");    // Send the data
                DRV_CAN_WRITE(&spi_1,&nil,X_TX_DATA,CAN_DLC_64);
                // Payload should send data over ISO TP 
                break;
            case 5:
                printf("Sent: X_ALL_PDOWN\n");
                DRV_CAN_WRITE(&spi_1, &nil, X_ALL_PDOWN, CAN_DLC_64);
                // no response required?
                break;
            case 6:
                printf("Sent: X_PDOWN\n");
                DRV_CAN_WRITE(&spi_1, &nil, X_PDOWN, CAN_DLC_64);
                // no response required?
                break;

            default:
                //printf(".");
                break;
        }
        i = 'x';
        
        vTaskDelay(100/portTICK_PERIOD_MS); // do nothing in the main loop 
    }
}
