
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
#include <errno.h>
#include "esp_spiffs.h"
#include <stdarg.h>




/**
 *@brief prints a message to the console
 *Super basic print of "hey now" for debugging
*@return None
 */
void print_check(void)
{
    printf("Print check\n");
}


/**
 *@brief Configures GPIO pins for input and output
 *This function initializes GPIO pins for input and output as specified by theioConfig parameter. The outputs are configured with no interrupts enabled,
set as outputs with a defined output bit mask, and with pulldown mode enabled
and pullup mode disabled.
*@return gpio_config_t : 'ESP_OK' if all sucessfull, else, it returns 'ESP_NONCRITICAL'.
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
    if(rtn != ESP_OK){
        if(DEBUG){
            printf("setupHW: setup output Fail\n");
        }else{
            logError("setupHW: setup output Fail\n");
        }
        return ESP_NONCRITICAL;
    }
    // Set up the inputs 
    ioConfig.intr_type = GPIO_INTR_DISABLE;     // This might chage in the future
    ioConfig.pin_bit_mask = INPUT_BIT_MASK;     // Defined in DandSMC.h
    ioConfig.mode = GPIO_MODE_INPUT;            // Set to inputs
    ioConfig.pull_down_en = 1;                  // enable pull down
    ioConfig.pull_up_en = 0;                    // disable pull up
    rtn = gpio_config(&ioConfig);
    if(rtn != ESP_OK){
        if(DEBUG){
            printf("setupHW: setup input Fail\n");
        }else{
            logError("setupHW: setup input Fail\n");
        }
        return ESP_NONCRITICAL;
    }
    // Set up the inputs/Outputs
    ioConfig.intr_type = GPIO_INTR_DISABLE;     // This might chage in the future
    ioConfig.pin_bit_mask = INPUT_OUTPUT_BIT_MASK;     // Defined in DandSMC.h
    ioConfig.mode = GPIO_MODE_INPUT_OUTPUT;            // Set to inputs
    ioConfig.pull_down_en = 1;                  // enable pull down
    ioConfig.pull_up_en = 0;                    // disable pull up
    rtn = gpio_config(&ioConfig);
    if(rtn != ESP_OK){
        if(DEBUG){
            printf("setupHW: setup in/out Fail\n");
        }else{
            logError("setupHW: setup in/out Fail\n");
        }
        return ESP_NONCRITICAL;
    }
    return ESP_OK;
}


/**
 * @brief Powers GPIO's for ADC
 * @param[in] en Boolean 1 = power on and 0 = power off
 * @return None
 */
void ADC_Pwr(bool en){


    //set ADC pins as high
    gpio_set_level(ADCLPWR, en);
    gpio_set_level(ADCRPWR, en);
    if(en) {    
        printf("ADC:    ADC's powered ON.\n");
    } 
    else{
        printf("ADC:    ADC's powered OFF.\n");
    }
    

    //If 1 down , non crit , if 2 then fail
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
    i2c_param_config(I2C_MASTER_NUM , &conf);                // ESPOK - CRIT

    i2c_driver_install(I2C_MASTER_NUM,conf.mode, 0,0,0 );    //install the driver , ESPOK - CRIT
    printf("ADC:    I2C Master Initialised\n");
}

/**
 * @brief Looks for I2C devices over the bus and prints on console
 * @return None
 */
void I2C_Scan()
{
    uint8_t i2c_addresses[128];             //An array to hold the addresses
    int num_devices = 0;                    //Number of devices connected
    for (int i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i << 1 | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            i2c_addresses[num_devices++] = i;
        }
    }

    //NON CRIT IF ONLY 1 ADDRESS FOUND
    printf("ADC:    Found %d I2C devices:\n", num_devices);
    for (int i = 0; i < num_devices; i++) {
        printf("- 0x%02X\n", i2c_addresses[i]);
    }

}

void Print_buffer(uint8_t* buf){

    for (int i = 0; i < sizeof(buf); i++) {
        printf("buf[%d] = 0x%02X\n", i, buf[i]);  
    }
    
}

/**
 * @brief Converts a buffer of uint8_t values to an array of int16_t values.
 *
 * This function iterates over the buffer, interpreting every two consecutive
 * uint8_t values as a 16-bit integer value, and stores the resulting array of
 * int16_t values. If the most significant bit of the value is set, the value
 * is treated as a negative two's complement number.
 *
 * @param buffer Pointer to the buffer containing the uint8_t values to convert.
 * @param size The number of uint8_t values to convert, which is half the number
 * of int16_t values to store.
 *
 * @note The function assumes that the buffer contains valid uint8_t values in
 * little-endian byte order.
 */
/*void buf_to_int(uint8_t* buffer,  int size){

    for (int i = 0 ; i < size ; i++) {
        int16_t value = ((buffer[i*2] & 0x0F) << 8) | buffer[i*2+1];
        if (value & 0x0800) {
            value |= 0xF000;
        }
        printf("Channel %d, ADC Value: %hd\n" ,i , value);
    }
}*/

void buf_to_int(uint8_t* buffer, int size){
    int16_t value;
    for (int i = 0 ; i < size/2 ; i++) {
        value = ((buffer[i*2] & 0x0F) << 8) | buffer[i*2+1];
        if (value & 0x0800) {
            value |= 0xF000;
        }
        printf("Channel %d, ADC Value: %hd\n" ,i , value);
    }

    //SIZE < 1MILLI 
}

/*For the MCP3424 adc chip , create a device connection test function that can follow the following steps :

The user can test the presence of the MCP3422/3/4 on
the I2C bus line without performing an input data
conversion. This test can be achieved by checking an
acknowledge response from the MCP3422/3/4 after
sending a read or write command. Here is an example
using Figure 6-3:
a. Set the R/W bit “HIGH” in the address byte.
b. Check the ACK pulse after sending the address
byte.
If the device acknowledges (ACK = 0), then the
device is connected, otherwise it is not
connected.
c. Send STOP or START bit.*/


/**
 * @brief Reads ADC value over I2C.
 *This function sends a command over channel 0 to adc. Waits for 
 conversation and reads value over adc.
 Variables that can be set :

• Conversion bit resolution: 12 bits
• Input channel selection: CH1 , CH2 , CH3 , or CH4
• PGA Gain selection: x1, x2, x4, or x8
• Continuous or one-shot conversion 
 * @return 18bit signed integer values
 */
void ADC_Read(uint8_t address)
{
    //4 channel , 18bit , x8 gain , one shot config bytes
    //uint8_t ADC_CH[] = {0X8F , 0XAF , 0XCF , 0XEF};
    uint8_t configByte[]=  {0XCF , 0XDF , 0XEF , 0XFF};
    int ch_size = sizeof(configByte)/sizeof(configByte[0]);

    //BUFFER INIT
    uint8_t buf[3];
    int buf_size = sizeof(buf)/sizeof(buf[0]);

    for (int j = 0 ; j < ch_size ; j++){    
        
        //ZERO EM ALL
        memset(buf , 0 ,buf_size);                      //zeros all values in buffer

        //I2C WRITE
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, configByte[j], false); 
        i2c_master_stop(cmd);

        //Implement I2C Comms
        i2c_master_cmd_begin(I2C_MASTER_NUM ,cmd, 1000 / portTICK_PERIOD_MS );
        i2c_cmd_link_delete(cmd);
        vTaskDelay(SAMPLE_DELAY_MS / portTICK_PERIOD_MS);

        //I2C READ
        cmd = i2c_cmd_link_create(); 
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_READ, true);

        for (int i = 0; i < buf_size; i++) {
            uint8_t byte;
            i2c_master_read_byte(cmd, &byte, (i == buf_size - 1) ? I2C_MASTER_LAST_NACK : I2C_MASTER_ACK);
        }

        i2c_master_stop(cmd);
        //Implement I2C Comms
        i2c_master_cmd_begin(I2C_MASTER_NUM ,cmd, 1000 / portTICK_PERIOD_MS );
        i2c_cmd_link_delete(cmd);
        
    }   

}



/**

@brief Enable or disable the motor.
This function sets the state of two GPIO pins to enable or disable the motor.
@param[in] en Whether to enable (true) or disable (false) the motor.
*/
void EnMotor(bool en){

    // This could probably have some error checking inplemented
    gpio_set_level(MOTEN,!en);              // 1 = off, 0 = on
    gpio_set_level(MVEN,en);                // 1 = on,  0 = off
    gpio_set_level(MSLEEP,en);              // 1 = wake,0 = sleep

    //CRIT 
    printf("EnMotor state : %d\n" , en);    //Print state
}

/**
 * @brief Runs a stepper motor for a specified number of steps in a specified direction.
 * 
 * The function enables the motor, sets the direction and moves a stepper motor a certain number of ticks,
 * returning SUCCESS if successful, or HW_FAULT if there are errors. 
 * 
 * @param dir A boolean value indicating the direction of the motor (`true` for clockwise, `false` for counter-clockwise).
 * @param ticks The number of steps to move the motor.
 * 
 * @return An integer value indicating the result of the operation (`SUCCESS` for success, `HW_FAULT` for a hardware fault).
 */ 
int RunMotor(bool dir, int ticks){

    //Enable the Motor
    EnMotor(1);

    //Returns fault condition when fails to set direction
    esp_err_t err = ESP_OK;                     
    if(gpio_set_level(MOTDIR,dir)!=ESP_OK){
        printf("ERROR: Error in setting the motor direction");
        return HW_FAULT;            //NON CRIT - TRY AGAIN HIGHER LEVEL                      
    }

    //Set motstep high and low for ticks amount
    for(int i =0; i<=ticks; i++){
        err = gpio_set_level(MOTSTEP,1);
        vTaskDelay(10/portTICK_PERIOD_MS);
        err = gpio_set_level(MOTSTEP,0);
        vTaskDelay(10/portTICK_PERIOD_MS);

        if (ticks == 6000){
            printf("I've done a rev");
        }
         // Returns fault if unable to toggle MOTSTEP pin
        if(err!=ESP_OK){
            printf("ERROR: Error in setting toggeling the MOTSTEP pin. Breaking from loop.\n");
            return HW_FAULT;        //NON CRIT - TRY AGAIN HIGHER LEVEL                  
        } 

         
    }

    //Disable the Motor
    EnMotor(0);

    return SUCCESS;                             // RET ESPOK
}

/**
*@brief Updates the experiment count stored in Non-Volatile Storage (NVS) and returns the updated value.
*@param erase If set to true, the stored experiment count will be erased and reset to 0.
*@return Returns the updated experiment count on success, -1 on failure.
*/
int updateExperimentCount(bool erase){
    //NON CRIT
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

void checkGPIOS(){
    // List that is just for reference 

    //{MSLEEP,"MSLEEP"},      // should be 1  = wake
    //{MVEN, "MVEN"},         // should be 1 = enable fuse
    //{MOTEN,"MOTEN"},        // should be 0 
    //{FFAULT, "FFAULT"},     // should be ?
    //{MFAULT, "MFAULT"},     // should be ?
    //{ADCLPWR, "ADCLPWR"},   // should be 1
    //{ADCRPWR, "ADCRPWR"},   // should be 1

    EnMotor(1);       // enable the motor

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
    EnMotor(0);   // disable the motor

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
 * @brief NOT YET IMPLEMENTED
*/
void calibrate(){
    printf("calibrate function not implemented\n");
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

    vTaskDelay(5000/portTICK_PERIOD_MS);    // delay to check if the polling is happening repeadatly
    vTaskDelete(FaultIndHandle);            // delete the FI polling task. 
    return 1;                               // [TODO]

}

void logError(const char* info)
{

    //Not Implemented
}

/**

@brief Logs the given information to a 2d array and prints the entire array.
@param info The information to be logged.
*/
/**

@brief Logs the given information to a 2d array and prints the entire array.
@param info The information to be logged.
*/
void logData(const char* info, ...){

    // create a 2D array with 100 rows and 100 columns each
    static char dataArray[100][100]; 

    //Last position tracking variable
    static int lastPosition = 0;
    
    //Variable arguments
    va_list args;
    va_start(args, info);
    vfprintf(stdout, info, args);
    va_end(args);
    
    //Find lenght of new string
    int len = strlen(info);

    //Move to next row if row is complete
    if (lastPosition + len + 1 > 100) { 
        lastPosition = 0; 
    }

    //Space as seperator
    dataArray[lastPosition][0] = ' ';

    //Append info and increment last position by size of info + space
    strcpy(&dataArray[lastPosition][1], info); 
    lastPosition += len + 1; 
    
    //print out array
    for (int i = 0; i <= lastPosition; i++) {
        printf("%s", dataArray[i]);
    }
}

/*void fileCreate() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    // Initialize SPIFFS
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        // handle error
        printf("config failed\n");
    }
}*/



















