
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
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
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include <inttypes.h>


/**
 *@brief prints a message to the console
 *Super basic print of "hey now" for debugging
*@return None
 */
void print_check(void){
    printf("Print check\n");
}

/**
 * @brief Looks for I2C devices over the bus and prints on console
 * @return None
 */
void I2C_Scan()
{
    uint8_t i2c_addresses[128];
    int num_devices = 0;
    for (int i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i << 1 | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            i2c_addresses[num_devices++] = i;
        }
    }
    printf("ADC:    Found %d I2C devices:\n", num_devices);
    for (int i = 0; i < num_devices; i++) {
        printf("- 0x%02X\n", i2c_addresses[i]);
    }

}

/**
 * @brief Reads ADC value over I2C.
 *This function sends a command over channel 0 to adc. Waits for 
 conversation and reads value over adc.
 * @return 16bit signed integer val
 */
int16_t ADC_Read()
{
    uint8_t buf[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    //sends the i2c address and a write flag - true shows the the master should send a stop condition after writing data
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, true);
    //command 0x80 starts convo and sets adc to read channel 0
    i2c_master_write_byte(cmd, 0x80, true);

    //this is stop condition to end i2c comms
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM ,cmd, 1000 / portTICK_PERIOD_MS );
    i2c_cmd_link_delete(cmd);

    //wait a bit//
    vTaskDelay(SAMPLE_DELAY_MS / portTICK_PERIOD_MS);

    //read
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd , &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd , &buf[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd , &buf[2], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM ,cmd, 1000 / portTICK_PERIOD_MS );
    i2c_cmd_link_delete(cmd);

    /*Convert raw uggy data to 16bit signed integer - untested*/
    int16_t value = ((buf[0] & 0x0F) << 8) | buf[1];
    if (value & 0x0800)
    {
        value |= 0xF000;
    }
    printf("ADC Value:   %hd\n" ,value);
    return value;
}

/**
 * @brief Initialise I2C Comms on ESP32
 *This function initialises the master with the specified config params
 * @return None
 */
void I2C_Init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                             //Set the I2C master mode
    conf.sda_io_num = I2C_SDA_PIN;                           //Assign the sda and scl pin numbers
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                 //enable pullups but may need removal since we hw implemented this
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;              //set clock speed to 100kHz

    i2c_param_config(I2C_MASTER_NUM , &conf);
    i2c_driver_install(I2C_MASTER_NUM,conf.mode, 0,0,0 );    //install the driver
    printf("ADC:    I2C Master Initialised\n");
}

/**
 * @brief Powers GPIO's for ADC
 * @param[in] en Boolean 1 = power on and 0 = power off
 * @return None
 */
void ADC_Pwr(bool en){


    //setting pins as output for ADC
    gpio_set_direction(ADCLPWR, GPIO_MODE_OUTPUT);
    gpio_set_direction(ADCRPWR, GPIO_MODE_OUTPUT);
    printf("ADC:    ADC's Pins set to outputs\n");

    //set ADC pins as high
    gpio_set_level(ADCLPWR, en);
    gpio_set_level(ADCRPWR, en);
    if(en) {    
        printf("ADC:    ADC's powered ON.\n");
    } 
    else{
        printf("ADC:    ADC's powered OFF.\n");
    }
    
}

/**
*@brief Hardware setup function
* This function is responsible for setting up the GPIOs of the ESP32
*@param pvParamemters void pointer to parameters (not used)
*@return esp_err_t (int) 
*/
esp_err_t setupHW(void){

    gpio_config_t ioConfig = {};    // zero initialise the structure
    esp_err_t rtn;  

    // Set up the outputs
    ioConfig.intr_type = GPIO_INTR_DISABLE;     // disable interupts
    ioConfig.mode = GPIO_MODE_INPUT_OUTPUT;           // Set as outputs
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
*@brief Hardware watchdog pulse task.
*This task is responsible for toggling the hardware watchdog pin at regular intervals.
*The watchdog pin is toggled high for a short settling time and then back low.
*This action prevents the hardware watchdog timer from resetting the system.
*@param pvParamemters void pointer to parameters (not used)
*@return None.
*/
void hwWDPulseTask(void* pvParamemters){
   
    for(;;){
        printf(".");
        gpio_set_level(WDI,1);                  // set the hardware watchdog high
        vTaskDelay(10/portTICK_PERIOD_MS);      // settling time
        gpio_set_level(WDI,0);                  // Set the HW_watchdog low
        vTaskDelay(5000/portTICK_PERIOD_MS);    // give controll back to the scheduler for 5 seconds
    }

    vTaskDelete(NULL);                                                          // delete task if it breaks for some reason
    printf("CRITICAL ERROR: hwWDPulseTask broke from loop. Task Deleted\n");    // error msg
}

/**
*@brief Updates the experiment count stored in Non-Volatile Storage (NVS) and returns the updated value.
*@param erase If set to true, the stored experiment count will be erased and reset to 0.
*@return Returns the updated experiment count on success, -1 on failure.
*/
int updateExperimentCount(bool erase){

    //printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return -1;
    } else {
        

        // Read
       // printf("Reading restart counter from NVS ... ");
        int32_t experiment_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "restart_counter", &experiment_counter);
        switch (err) {
            case ESP_OK:
            //    printf("Done\n");
                printf("Restart counter = %" PRIu32 "\n", experiment_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        //printf("Updating restart counter in NVS ... ");
        
        if(erase){
            experiment_counter = 0;
        }else{
            experiment_counter++;
        }
        
        err = nvs_set_i32(my_handle, "restart_counter", experiment_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        //printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);   
        return (int)experiment_counter;
    }
    
}

/**
* @brief Calls a rang of functions that:
* check the state of the GPIO's, 
* comunicates with the ADC's and the CAN controler
* checks the avalible memory
* @param NONE
* @return NOT IMPLEMENTED YET
*/
void systemHealthCheck(){
    printf("SYSTEM HEALTH CHECK\n");

    checkGPIOS();   // Check the GPIO's 
    checkADC();     // Check the ADC's
    checkCANctrl(); // Check Can Controller
    checkMem();     // Check the memory avaliable
}

/**
 * @brief used to check a large number of GPIO's to make sure they working as intended
 * @param none
 * @return  not yet implemented
*/
void checkGPIOS(){
    // List that is just for reference 

    //{MSLEEP,"MSLEEP"},      // should be 1  = wake
    //{MVEN, "MVEN"},         // should be 1 = enable fuse
    //{MOTEN,"MOTEN"},        // should be 0 
    //{FFAULT, "FFAULT"},     // should be ?
    //{MFAULT, "MFAULT"},     // should be ?
    //{ADCLPWR, "ADCLPWR"},   // should be 1
    //{ADCRPWR, "ADCRPWR"},   // should be 1

    motor(1);       // enable the motor

    uint16_t bitmask = 0b0000000001111011;  /* @todo should be defined as a constant elsewhere */
    uint16_t levels =  0b0000000000000000;

    for (int i = 0; i < sizeof(healthCheck) / sizeof(GPIO_Pins); i++) {
            uint32_t level = gpio_get_level(healthCheck[i].pin);
            levels |= (level<<i);
            printf("%s , %d\n",healthCheck[i].name,(int)level);
        }
    if (levels != bitmask){
        printf("GPIOs fail system health check \n");
    }
    motor(0);   // disable the motor

}

/**
 * @brief Checks the amount of heap that is avalible and prints it to the console
 * @param none
 * @return  not yet implemented 
*/
void checkMem(){
    uint32_t free_heap_size = esp_get_free_heap_size();
    printf("Free Heap = %d bytes.\n",(int)free_heap_size);
    // what would constitue a memory issue? 
}

void checkADC(){
    printf("checkADC function not implemented\n");
}

void checkCANctrl(){
    printf("checkCANctrl function not implemented\n");
}



/*_________________________________________________________________*/
/**
 * @brief Function logically goes through the process of sequentially conducting a single experimental cycle
 * @param NONE
 * @return Will return some type of fault if there is an issue with the the experiment NOT YET IMPLEMENTED
*/
int RunExperiment(){
    // when the experiment is running. create a task that allows for the polling of the Fault Indicator Pins. 
    TaskHandle_t FaultIndHandle;
    xTaskCreate(
        PollFaultIndicatorsTask,    // pointer to the function that implements the task
        "HWFault",                  // text name given to the task
        2048,                       // size of stack that should be created in words not bytes
        NULL,                       // Reference to xParameters used by the task
        1,                          // task priorety
        &FaultIndHandle);           // handle to the task being created. 

    // turn on the ADC's 
    gpio_set_level(ADCRPWR,1);  
    gpio_set_level(ADCLPWR,1);

    // Note where the System Health check takes place. Both ADC's should be powered.
    systemHealthCheck();    
    
    calibrate(); // [TODO]
    
    printf("____RUNNING EXPERIMENT____\n");
    // Run the motor
    RunMotor(1,200*(TICKS_PER_REV*0.5));

    // Take measurement [J] TODO

    // Run the motor
    RunMotor(0,200*TICKS_PER_REV);

    // Take Measurement [J] TODO

    //Run the motor
    RunMotor(1,200*TICKS_PER_REV*0.5);

    // Disable the ADC's 
    gpio_set_level(ADCRPWR,0);
    gpio_set_level(ADCLPWR,0);

    vTaskDelay(5000/portTICK_PERIOD_MS);    // delay to check if the polling is happening repeadatly
    vTaskDelete(FaultIndHandle);            // delete the FI polling task. 
    return 1;                               // [TODO]

}


/**
 * @brief Task to poll fault indicators and print their status.
 * 
 * This task polls the fault indicators, which are defined in a constant 
 * struct called faultIndicators in DandDMC.h, and prints their status (GOOD or BAD)
 * to the console every second. The task runs indefinitely until it is deleted.
 *
 * @param pvParamemters pointer to task parameters (not used in this task).
 * @return Task does not return anything but its method of interupting opperation is yet to be implemented
 */
void PollFaultIndicatorsTask(void* pvParamemters){
    printf("Polling Fault Indicators TASK active\n");
    for (;;) {
        
        for (int i = 0; i < sizeof(faultIndicators) / sizeof(GPIO_Pins); i++) {
            uint32_t level = gpio_get_level(faultIndicators[i].pin);
            //printf("FI Task: %s %s\n", faultIndicators[i].name, (level & 1) == 0 ? "BAD" : "GOOD");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);                                                          // delete task if it breaks for some reason
    printf("CRITICAL ERROR: hwWDPulseTask broke from loop. Task Deleted\n");    // error msg
}

/**
* @brief handels the enableing and disableing of the motor driver
* @param bool 'en' is the desired state of the driver. 1 = enable, 0 = disabled
* @return nothing yet
*/
void motor(bool en){
    // This could probably have some error checking inplemented
    gpio_set_level(MOTEN,!en);  // 1 = off, 0 = on
    gpio_set_level(MVEN,en);    // 1 = on,  0 = off
    gpio_set_level(MSLEEP,en);  // 1 = wake,0 = sleep
}

/**
*@brief Drives the motor in the specified direction for a given number of pulses.
*This function sets the direction of the motor and enables the motor, output of the power fuse, and the sleep mode. It then runs the motor in the specified direction for the given number of pulses. If an error occurs during this process, the function returns a hardware fault.
*@param dir A boolean indicating the direction to run the motor. true indicates a clockwise rotation, and false indicates a counterclockwise rotation.
*@param ticks An integer representing the number of pulses to run the motor for.
*@return An integer indicating the success of the operation. SUCCESS (0) is returned if the operation was successful. HW_FAULT (-1) is returned if a hardware fault occurred during the operation.
*/
int RunMotor(bool dir, int ticks){

    esp_err_t err = ESP_OK;   

    if(gpio_set_level(MOTDIR,dir)!=ESP_OK){
        printf("ERROR: MOTDIR: Error in setting the motor direction\n");
        return HW_FAULT;                        
    }
    motor(1); // enable the motor

    // run the motor
    for(int i =0; i<=ticks; i++){
        err = gpio_set_level(MOTSTEP,1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        err = gpio_set_level(MOTSTEP,0);
        vTaskDelay(5/portTICK_PERIOD_MS);

        if(err!=ESP_OK){
            printf("ERROR: Error in setting toggeling the MOTSTEP pin. Breaking from loop.\n");
            return HW_FAULT;        // return fault if unable to toggle MOTSTEP pin
        }  
    }

    motor(0);   // disable motor
    
    return SUCCESS;                 // ELSE return success after motor has been pulsed 'ticks' number of times. 
}

/**
 * @brief NOT YET IMPLEMENTED
*/
void calibrate(){
    printf("calibrate function not implemented\n");
}




