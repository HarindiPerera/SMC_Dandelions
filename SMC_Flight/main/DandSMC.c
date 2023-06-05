
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
#include <stdarg.h>
#include "canfdspi/drv_canfdspi_api.h"
#include "canfd/drv_can.h"
#include "msg/msg.h"
#include "md5/md5.h"
#include "isotp/iso_tp.h"
#include "driver/spi_master.h"
#include "time.h"



static const GPIO_Pins faultIndicators[] = {
    {FFAULT, "FFAULT"},
    {MFAULT, "MFAULT"},
    {ADCLPWR, "ADCLPWR"},
    {ADCRPWR, "ADCRPWR"},
};

static const GPIO_Pins healthCheck[] = {
    {MSLEEP,"MSLEEP"},
    {MVEN, "MVEN"},
    {MOTEN,"MOTEN"},
    {FFAULT, "FFAULT"},
    {MFAULT, "MFAULT"},
    {ADCLPWR, "ADCLPWR"},
    {ADCRPWR, "ADCRPWR"},
};



/* Task functions*/

/*void SMC_Timestamp(spi_device_handle_t* spi){
    
    struct timeval tv;
    _gettimeofday_r(&tv, NULL);
    uint64_t time_ms = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    
    printf("Timestamp: %llu\n",time_ms);

    DRV_CAN_WRITE(spi,(uint8_t*) &time_ms,TIMESTAMP,CAN_DLC_64);
}*/

/*void SMC_Spacecraft_State(spi_device_handle_t* spi){
    
    Spacecraft_State state = {
        .Latitude = 0.0,
        .Longitude = 1.0,
        .Altitude = 2.0,
        .Roll = 3.0,
        .Pitch = 4.0,
        .Heading = 5.0,
        .Velocity_N = 6.0,
        .Velocity_E = 7.0,
        .Velocity_D = 8.0,
        .Angular_VX = 9.0,
        .Angular_VY = 10.0,
        .Angular_VZ = 11.0,
    };

    printf("%d",sizeof(Spacecraft_State));
    printf("Spacecraft State:\n"
        "lat:%f\n"
        "lon:%f\n"
        "alt:%f\n"
        "roll:%f\n"
        "pitch:%f\n"
        "heading:%f\n"
        "vel_n:%f\n"
        "vel_e:%f\n"
        "vel_d:%f\n"
        "ang_vel_x:%f\n"
        "ang_vel_y:%f\n"
        "ang_vel_z:%f\n\n",
        state.Latitude, state.Longitude, state.Altitude, state.Roll, state.Pitch, state.Heading, state.Velocity_N, state.Velocity_E, state.Velocity_D, state.Angular_VX, state.Angular_VY, state.Angular_VZ);

    uint8_t* s = malloc(sizeof(Spacecraft_State));
    memcpy(s,&state,sizeof(Spacecraft_State));

    DRV_CAN_WRITE(spi,s,SPACECRAFT_STATE,CAN_DLC_64);

    free(s);
}
*/

// void SMC_PowerDown(spi_device_handle_t* spi){
//     // printf("Powering down...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,POWDWN,CAN_DLC_64);
//     // DRV_CAN_WRITE(spi,NULL,POWDWN,CAN_DLC_64);
// }

// void SMC_BeginOperation(spi_device_handle_t* spi){
//     // printf("Ready to operate...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,BEGIN,CAN_DLC_64);
// }

// void SMC_CeaseOperation(spi_device_handle_t* spi){
//     // printf("Ending operation...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,CEASE,CAN_DLC_64);
//     // DRV_CAN_WRITE(spi,NULL,CEASE,CAN_DLC_64);
// }

// void SMC_PowerDownAll(spi_device_handle_t* spi){
//     // printf("Powering down all...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,POWDWN_ALL,CAN_DLC_64);
//     // DRV_CAN_WRITE(spi,NULL,POWDWN_ALL,CAN_DLC_64);
// }

// void SMC_Query(spi_device_handle_t* spi){
//     printf("Querying...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,QUERY,CAN_DLC_64);
//     // DRV_CAN_WRITE(spi,NULL,QUERY,CAN_DLC_64);
// }

// void SMC_Transmit(spi_device_handle_t* spi){
//     printf("Transmitting...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,TRANSMIT,CAN_DLC_64);
//     // DRV_CAN_WRITE(spi,NULL,TRANSMIT_CMD,CAN_DLC_64);
// }

// void SMC_Flow(spi_device_handle_t* spi){
//     // printf("Flowing...\n");
//     iso_tp_control_t control = {
//         .flow = CONTINUE,
//         .block_size = 1,
//         .sep_time = 0,
//         .num_segments = 0,
//         .index = 0,
//         .pci = 0,
//     };
//     DRV_CAN_WRITE(spi,(uint8_t*) &control,TRANSMIT_FLOW,CAN_DLC_64);
// }

// void SMC_Result(spi_device_handle_t* spi){
//     // printf("Resulting...\n");
//     uint8_t nil = 0;
//     DRV_CAN_WRITE(spi,&nil,TRANSMIT_RESULT,CAN_DLC_64);
// }

void print_bits2(uint8_t n) {
    for (int i = 7; i >= 0; i--) {
        printf("%d", (n >> i) & 1);
    }
    printf("\n");
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
            printf("setupHW(): setup output Fail\n");
        }else{
            logError("setupHW(): setup output Fail\n");
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
            printf("setupHW(): setup input Fail\n");
        }else{
            logError("setupHW(): setup input Fail\n");
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
            printf("setupHW(): setup in/out Fail\n");
        }else{
            logError("setupHW(): setup in/out Fail\n");
        }
        return ESP_NONCRITICAL;
    }

    // Nominal Exit
    if(DEBUG){
        printf("setupHW(): ESP_OK\n");
    }else{
        logError("setupHW(): ESP_OK\n");
    }
    return ESP_OK;
}

/**
 * @brief Powers GPIO's for ADC
 * @param[in] en Boolean 1 = power on and 0 = power off
 * @return esp_err_t ESP_FAIL if both ADC's fail. 
 * ESP_NONCRITICAL on single fail.
 * ESP_OK on sucess.
 */
esp_err_t ADC_Pwr(bool en){

    esp_err_t ADCRCheck; 
    esp_err_t ADCLCheck;

    //set ADC pins as high
    ADCLCheck = gpio_set_level(ADCLPWR, en);
    ADCRCheck = gpio_set_level(ADCRPWR, en);  

    if (ADCLCheck != ESP_OK && ADCRCheck != ESP_OK){
        if(DEBUG){
            printf("ADC_Pwr(): both gpio_set_level() fail - ESP_FAIL\n");
        }else{
            logError("ADC_Pwr(): both gpio_set_level() fail - ESP_FAIL\n");
        }
        return ESP_FAIL;        // Both are not ok = Critical 

    }else if (ADCLCheck != ESP_OK){
        if(DEBUG){
            printf("ADC_Pwr(): Left gpio_set_level() fail- ESP_NONCRITICAL\n");
        }else{
            logError("ADC_Pwr(): Left gpio_set_level() fail - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;    

    } else if (ADCRCheck != ESP_OK){
        if(DEBUG){
            printf("ADC_Pwr(): Right gpio_set_level() fail - ESP_NONCRITICAL\n");
        }else{
            logError("ADC_Pwr(): Right gpio_set_level() fail - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }
    // Nominal exit
    if(DEBUG){
        printf("ADC_Pwr(): Both success - ESP_OK\n");
    }else{
        logError("ADC_Pwr(): Both success - ESP_OK\n");
    }

    return ESP_OK;
}

/**
 * @brief Initialise I2C Comms on ESP32
 *This function initialises the master with the specified config params
 * @return esp_err_t 
 * ESP_FAIL if i2c_param_config or i2c_driver_install, else
 * ESP_NONCRITICAL if only one other devices is found
 *
 * ESP_OK.
 */
esp_err_t I2C_Init(void)
{
    
    esp_err_t rtn; 
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                             //Set the I2C master mode
    conf.sda_io_num = I2C_SDA_PIN;                           //Assign the sda and scl pin numbers
    conf.scl_io_num = I2C_SCL_PIN;
    //conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                 //enable pullups but may need removal since we hw implemented this
   // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;              //set clock speed to 100kHz

    rtn = i2c_param_config(I2C_MASTER_NUM , &conf);                
    if(rtn != ESP_OK){
        if(DEBUG){
            printf("I2C_Init(): i2c_param_config() - ESP_FAIL\n");
        }else{
            logError("I2C_Init(): i2c_param_config() - ESP_FAIL\n");
        }
        return ESP_FAIL;
    }

    rtn = i2c_driver_install(I2C_MASTER_NUM,conf.mode, 0,0,0 );    //install the driver , ESPOK - CRIT
    if(rtn != ESP_OK){
        if(DEBUG){
            printf("I2C_Init(): i2c_driver_install() - ESP_FAIL\n");
        }else{
            logError("I2C_Init(): i2c_driver_install() - ESP_FAIL\n");
        }
        return ESP_FAIL;
    }
   
    //printf("ADC:    I2C Master Initialised\n");
    if(DEBUG){
            printf("I2C_Init(): - ESP_OK\n");
        }else{
            logError("I2C_Init(): - ESP_OK\n");
        }
    return ESP_OK;
}

/**
 * @brief Looks for I2C devices over the bus and prints on console
 * @return esp_err_t
 * ESP_OK for nominal opperation
 * ESP_FAIL for no devices found
 * ESP_NONCRITICAL for one device found or issue with i2c write and read. 
 * ESP_ERR_INVALID_RESPONSE there was an issue reading or writing to the i2c bus
 */
esp_err_t I2C_Scan()
{
   

    esp_err_t rtn = ESP_OK;
    uint8_t i2c_addresses[128];             //An array to hold the addresses
    int num_devices = 0;                    //Number of devices connected
    for (int i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        rtn += i2c_master_start(cmd);
        rtn += i2c_master_write_byte(cmd, i << 1 | I2C_MASTER_WRITE, true);
        rtn += i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            i2c_addresses[num_devices++] = i;
        }
    }

   

    if (DEBUG){
        printf("I2C_Scan(): Found %d I2C devices:\n", num_devices);
         for (int i = 0; i < num_devices; i++) {
            printf("- 0x%02X\n", i2c_addresses[i]);
        }
    }else{
        char str[32]; 
        sprintf(str,"I2C_Scan(): Devices found : %d\n", num_devices);
        logError(str);           
    }
    
    if (rtn!= ESP_OK){
        if(DEBUG){
            printf("I2C_Scan(): Bad i2c RW - ESP_ERRINVALID_RESPONSE\n");
        }else{
            logError("I2C_Scan(): Bad i2c RW - ESP_ERRINVALID_RESPONSE\n");
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (num_devices == 1){
        if(DEBUG){
            printf("I2C_Scan(): Only self found - ESP_FAIL\n");
        }else{
            logError("I2C_Scan(): Only self found - ESP_FAIL\n");
        }
        return ESP_FAIL;  
    }

    if (num_devices == 2){
        if(DEBUG){
                printf("I2C_Scan(): Only 1 other device found - ESP_NONCRITICAL\n");
            }else{
                logError("I2C_Scan(): Only 1 other device found - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Nominal Exit
    if(DEBUG){
            printf("I2C_Scan(): All devices found - ESP_OK\n");
        }else{
            logError("I2C_Scan(): All devices found - ESP_OK\n");
        }
    return ESP_OK;
}

/**

@brief Enable or disable the motor.
This function sets the state of two GPIO pins to enable or disable the motor.
@param[in] en Whether to enable (true) or disable (false) the motor.
*/
esp_err_t EnMotor(bool en){
    
    esp_err_t rtn = ESP_OK;

    rtn |= gpio_set_level(MOTEN,!en);              // 1 = off, 0 = on
    rtn |= gpio_set_level(MVEN,en);                // 1 = on,  0 = off
    rtn |= gpio_set_level(MSLEEP,en);              // 1 = wake,0 = sleep

    // There was an issue with setting the GPIOS
    if (rtn != ESP_OK){
        if(DEBUG){
            printf("EnMotor(%d): gpio_set_level() ESP_FAIL\n",en);
        }else{
            logError("EnMotor(%d): gpio_set_level() ESP_FAIL\n",en);
        }
        return ESP_FAIL;
    }

    // Nominal Return 
    if(DEBUG){
            printf("EnMotor(%d): ESP_OK\n",en);
    }else{
        logError("EnMotor(): ESP_OK\n");
    }
    
    return rtn;
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
esp_err_t RunMotor(bool dir, int *ticks, enum flowFlag *flowFlagPtr){

    esp_err_t err = ESP_OK;  
;    if ((*ticks) <=0 ){
        if(DEBUG){
            printf("RunMotor():  %d <=0, invalid argument - ESP_FAIL\n",(*ticks));
        }else{
            logError("RunMotor(): ticks <0, invalid argument - ESP_FAIL\n");
        }
        return ESP_FAIL;
    }
    
    //Enable the Motor
    if (EnMotor(1)!=ESP_OK){
        if(DEBUG){
            printf("RunMotor(): EnMotor(1): - ESP_FAIL\n");
        }else{
            logError("RunMotor(): EnMotor(1): - ESP_FAIL\n");
        }
        return ESP_FAIL;  
    }

    //Set Motor Direction      
    if(gpio_set_level(MOTDIR,dir)!=ESP_OK){
        if(DEBUG){
            printf("RunMotor(): Motor Direction Set Fail - ESP_FAIL\n");
        }else{
            logError("RunMotor(): Motor Direction Set Fail - ESP_FAIL\n");
        }
        return ESP_FAIL;                                        //NON CRIT - TRY AGAIN HIGHER LEVEL                      
    }

    //Set motstep high and low for ticks amount
    // decrements the amount of ticks that are passed in as a pointer.
    // IF there is any issue anywhere then the amount of ticks will be altered in the parent functions. 

    
    for(esp_err_t mError =ESP_OK; (*ticks)>0 && mError==ESP_OK &&  *flowFlagPtr == NOMINAL; (*ticks)-=1){
        mError |= gpio_set_level(MOTSTEP,1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        mError |= gpio_set_level(MOTSTEP,0);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    
    if(*flowFlagPtr == ESD){
        // This is s test 
        printf("Emergency shut down\n");
    }

    // If loop ends because there has been an err!=0 condition
    if(err!=ESP_OK){
       if(DEBUG){
            printf("RunMotor(): Motor Stepping Loop Fail - ESP_FAIL\n");
        }else{
            logError("RunMotor(): Motor Stepping Loop Fail - ESP_FAIL\n");
        }
        err = ESP_FAIL;                     
    }  

    //Disable the Motor
    if (EnMotor(0)!=ESP_OK){
        if(DEBUG){
            printf("RunMotor(): EnMotor(0): - ESP_FAIL\n");
        }else{
            logError("RunMotor(): EnMotor(0): - ESP_FAIL\n");
        }
        return ESP_FAIL;  
    }

    // Nominal return 
    return ESP_OK;                             
}

/**
*@brief Updates the experiment count stored in Non-Volatile Storage (NVS) and returns the updated value.
*@param erase If set to true, the stored experiment count will be erased and reset to 0.
*@param count Pointer to an intergerger to be updated by this function.
*@return returns an esp_err_t error code. 
*/
esp_err_t updateExperimentCount(bool erase, int *count){
 
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK){
        if(DEBUG){
            printf("updateExperientCount(): nvs_open(): Open error - ESP_NONCRITICAL\n");
        }else {
            logError("updateExperientCount(): nvs_open(): open error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }


    // Read
    int32_t experiment_counter = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_i32(my_handle, "restart_counter", &experiment_counter);
    switch (err) {
        case ESP_OK:
        //    printf("Done\n");
            printf("Restart counter = %" PRIu32 "\n", experiment_counter);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            printf("Initialisiing memory \n");
            nvs_set_i32(my_handle,"restart_counter",experiment_counter);
            nvs_commit(my_handle);
            nvs_close(my_handle);
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    if(err != ESP_OK){
        if(DEBUG){
            printf("updateExperientCount(): nvs_get_i32(): Read error - ESP_NONCRITICAL\n");
        }else {
            logError("updateExperientCount(): nvs_get_i32(): Read error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }


    if(erase){
        experiment_counter = 0;
    }else{
        experiment_counter++;
    }
    
    err = nvs_set_i32(my_handle, "restart_counter", experiment_counter);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    if(err != ESP_OK){
        if(DEBUG){
            printf("updateExperientCount(): nvs_set_i32(): Write error - ESP_NONCRITICAL\n");
        }else {
            logError("updateExperientCount(): nvs_set_i32(): Write error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written to flash 
    err = nvs_commit(my_handle);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    if(err != ESP_OK){
        if(DEBUG){
            printf("updateExperientCount(): nvs_commit(): Commit error - ESP_NONCRITICAL\n");
        }else {
            logError("updateExperientCount(): nvs_commit(): Commit error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Close
    nvs_close(my_handle);   
    *count = (int)experiment_counter;

    // NOMINAL EXIT POINT
    return ESP_OK;
 
}

esp_err_t checkGPIOS(void){
    esp_err_t err = ESP_OK;
    // List for reference 
    // At the point where the motor should be ready to spin:

    //{MSLEEP,"MSLEEP"},      // should be 1  = wake
    //{MVEN, "MVEN"},         // should be 1 = enable fuse
    //{MOTEN,"MOTEN"},        // should be 0 = enable Motor
    //{FFAULT, "FFAULT"},     // should be 1 = Open drain active low with external PU resistor. 
    //{MFAULT, "MFAULT"},     // should be 1 = Open drain active low with external PU resistor
    //{ADCLPWR, "ADCLPWR"},   // should be 1 
    //{ADCRPWR, "ADCRPWR"},   // should be 1

    
    uint16_t bitmask = 0b0000000001111011;  /* @todo should be defined as a constant elsewhere */
    uint16_t levels =  0b0000000000000000;

    for (int i = 0; i < sizeof(healthCheck) / sizeof(GPIO_Pins); i++) {
            uint32_t level = gpio_get_level(healthCheck[i].pin);
            levels |= (level<<i);
            printf("%s , %d\n",healthCheck[i].name,(int)level);
        }
    if (levels != bitmask){
        err = ESP_FAIL;
    }
   
    return err;
}

/**
 * @brief Checks the amount of heap that is avalible and prints it to the console
 * @param none
 * @return  not yet implemented 
*/
esp_err_t checkMem(void){
    uint32_t free_heap_size = esp_get_free_heap_size();
    char str[32];
    sprintf(str,"Free Heap = %d bytes.\n",(int)free_heap_size);
    if(DEBUG){
        printf(str);
    }else{
        logError(str);
    }
    return ESP_OK;  // what would constitue a memory issue? 
    
}

/**
* @brief Calls a rang of functions that:
* check the state of the GPIO's, 
* comunicates with the ADC's and the CAN controler
* checks the avalible memory
* @param NONE
* @return NOT IMPLEMENTED YET
*/
esp_err_t systemHealthCheck(void){
    esp_err_t rtn = ESP_OK;
    esp_err_t i2cErr = ESP_OK;

   if(DEBUG){
    printf("SYSTEM HEALTH CHECK\n");
   }
    // It is the responsibility of the Calling function to make sure that everything is established 

    rtn |= checkGPIOS();   // Check the GPIO's
    rtn |= checkMem();     // Check the memory avaliable
    i2cErr = I2C_Scan();   // find both the adcs.

    if(i2cErr == ESP_FAIL){
        rtn |= i2cErr;
    }else if(i2cErr == ESP_NONCRITICAL){
        printf("systemHealthCheck(): One of the ADC's is down, But we must go on\n");
        rtn |= ESP_OK;      // Still opperate if one of the ADC's is down. 
    }else{
        rtn|= i2cErr;
    }

    if(rtn!=ESP_OK){
        if(DEBUG){
            printf("systemHealthCheck(): ESP_FAIL\n");
        }else{
            logError("systemHealthCheck(): ESP_FAIL\n");
        }
        return ESP_FAIL;
    }else{
        if(DEBUG){
            printf("systemHealthCheck(): ESP_OK\n");
        }else{
            logError("systemHealthCheck(): ESP_OK\n");
        }
    }
    // NOMINAL RETURN
    return ESP_OK;
}

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
esp_err_t ADC_Read(uint8_t address , int Channel)
{
    //uint8_t configByte[] = {0xCF, 0xDF, 0xEF, 0xFF};
    uint8_t configByte[] = {0X98 , 0XB8 , 0XD8 , 0XF8};
    uint8_t readBuffer[3];
    esp_err_t rtn = ESP_OK;
    //I2C WRITE CONFIG
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    rtn |=i2c_master_start(cmd);
    rtn |=i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);
    rtn |= i2c_master_write_byte(cmd, configByte[Channel], true);
    rtn |=i2c_master_stop(cmd);
    rtn |= i2c_master_cmd_begin(I2C_NUM_0, cmd, 500 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(SAMPLE_DELAY_MS / portTICK_PERIOD_MS);
    //I2C READ BYTE
    cmd = i2c_cmd_link_create();
    rtn |=i2c_master_start(cmd);
    rtn |=i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_READ, true);
    rtn |=i2c_master_read_byte(cmd, &readBuffer[0], I2C_MASTER_ACK);
    rtn |=i2c_master_read_byte(cmd, &readBuffer[1], I2C_MASTER_ACK);
    rtn |=i2c_master_read_byte(cmd, &readBuffer[2], I2C_MASTER_NACK);
    rtn |=i2c_master_stop(cmd);
    rtn |=i2c_master_cmd_begin(I2C_NUM_0, cmd, 500 / portTICK_PERIOD_MS);
    //Check Error
    if (rtn != ESP_OK)
    {
        printf("Error code: %s\n", esp_err_to_name(rtn));
        rtn = ESP_OK;
    }
    i2c_cmd_link_delete(cmd);
    //Convert to int16_t
    int16_t result = (int16_t)(((readBuffer[0] & 0x03) << 16) | (readBuffer[1] << 8) | readBuffer[2]);
    //LOG INTO FILE
    if(DEBUG){
        printf("Channel %d : %d\n" , Channel+1, result );
        vTaskDelay(100/portTICK_PERIOD_MS);
    }else{
        char str[64];
        printf(str, "Channel %d : %d\n" , Channel+1, result);
        logError(str);
    }
    return rtn;
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
esp_err_t RunExperiment(int *phase, int *ticks, enum flowFlag *flowFlagPtr){
    
  // NOTE that if you are here then the prvious action centered the cariage. 

    // when the experiment is running. create a task that allows for the polling of the Fault Indicator Pins. 
    TaskHandle_t FaultIndHandle;
    xTaskCreatePinnedToCore(
        PollFaultIndicatorsTask,    // pointer to the function that implements the task
        "HWFault",                  // text name given to the task
        2048,                       // size of stack that should be created in words not bytes
        NULL,                       // Reference to xParameters used by the task
        1,                          // task priorety
        &FaultIndHandle,            // handle to the task being created.
        1                           // Core that it is pinned to
        );   

    //Create Buffer for ADC adresses
    uint8_t ADC[] = {ADC_ADDR_1 ,ADC_ADDR_2 };

    EnMotor(1);

    if(systemHealthCheck()!= ESP_OK){
        // Here we assume that the lower level function does the logging
        // If system Health Check fails we return a fail
        return ESP_FAIL;
    }

    // CREATE THE AMOUNT OF TICKS TO MOVE
    (*ticks) = TICKS_PER_REV*2;        
    (*phase) = 1;              // This is the phase of the experiment 
    // Run motor in the first direction
    if(RunMotor(1,ticks,flowFlagPtr) !=ESP_OK){
        printf("There has been some motor issues\n");   // TODO -> make this an actuall thing 
    }
   // Add some condition to check the number of ticks. 
    if ((*ticks)!=0){
        if(DEBUG){
            printf("RunExperiment(): phase %d irregular, ticks %d\n",(*phase),(*ticks));
        }else{
            char str[64]; 
            sprintf(str, "RunExperiment(): irregular P %d, T %d\n", (*phase),(*ticks));
            logError(str);
        }
        return ESP_NONCRITICAL;
    }

    // Take measurement 
     for (int i = 0 ; i < 2 ; i++){
        if(DEBUG){
            printf("ADC Address 0x%02x\n" , ADC[i]);
            vTaskDelay(100/portTICK_PERIOD_MS);
        }else{
            char str[64];
            sprintf(str, "ADC Address 0x%02x\n" , ADC[i]);
            logError(str);
        }
        for (int channel = 0 ; channel < 4  ; channel ++ ){
            ADC_Read(ADC[i] , channel);
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
    }

    // Set the number of ticks for the second movement. 
    (*ticks) = 4*TICKS_PER_REV;     // set new tick amount 
    (*phase)+=1;                    // increment the phase
    
    // Run motor in the second direction
    if(RunMotor(0,ticks,flowFlagPtr) !=ESP_OK){
        printf("There has been some motor issues\n");
    }
    // Add some condition to check the number of ticks. 
    if ((*ticks)!=0){
        if(DEBUG){
            printf("RunExperiment(): phase %d irregular, ticks %d\n",(*phase),(*ticks));
        }else{
            char str[64]; 
            sprintf(str, "RunExperiment(): irregular P %d, T %d\n", (*phase),(*ticks));
            logError(str);
        }
        return ESP_NONCRITICAL;
    }
    
    // Take measurement 
     for (int i = 0 ; i < 2 ; i++){
        if(DEBUG){
            printf("ADC Address 0x%02x\n" , ADC[i]);
            vTaskDelay(100/portTICK_PERIOD_MS);
        }else{
            char str[64];
            sprintf(str, "ADC Address 0x%02x\n" , ADC[i]);
            logError(str);
        }
        for (int channel = 0 ; channel < 4  ; channel ++ ){
            ADC_Read(ADC[i] , channel);
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
    }


    // Set the number of ticks for the third movement. 
    (*ticks) = 2*TICKS_PER_REV;     // Set (*ticks) for the last phase
    (*phase)+=1;                    // increment the phase
    // Run motor in the second direction
    if(RunMotor(1,ticks,flowFlagPtr) !=ESP_OK){
        printf("There has been some motor issues\n");
    }
    // Add some condition to check the number of ticks. 
    if ((*ticks)!=0){
        if(DEBUG){
            printf("RunExperiment(): phase %d irregular, ticks %d\n",(*phase),(*ticks));
        }else{
            char str[64]; 
            sprintf(str, "RunExperiment(): irregular P %d, T %d\n", (*phase),(*ticks));
            logError(str);
        }
        return ESP_NONCRITICAL;
    }


    // for a nominal experiment the phase is set to zero 
    // and the (*ticks) should be 0
    (*phase) = 0;

    logError("Experiment Complete\n");

    vTaskDelete(FaultIndHandle);            // delete the FI polling task. 
    return ESP_OK;                        

}


// Sets the experiment phase and tick in memory
esp_err_t setExperimentPhaseTicks(int *phase, int *ticks, bool reset){

    if(reset){
        *phase = 0;
        *ticks = 0;
    }
    if(DEBUG){
        printf("setting phase %d and ticks %d in NVS\n", (*phase),(*ticks));
    }
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK){
        if(DEBUG){
            printf("setExperimentPhaseTick(): nvs_open(): Open error - ESP_NONCRITICAL\n");
        }else {
            logError("setExperimentPhaseTick(): nvs_open(): open error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // set Phase
    err = nvs_set_i32(my_handle, "phase", (int32_t)(*phase));
    printf("set phase NVS = ");
    printf((err != ESP_OK) ? "Failed!\n" : " Done\n");
    if(err != ESP_OK){
        if(DEBUG){
            printf("setExperimentPhaseTick(): nvs_set_i32(): phase Write error - ESP_NONCRITICAL\n");
        }else {
            logError("setExperimentPhaseTick(): nvs_set_i32(): phase Write error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }
    // Set Ticks
    err = nvs_set_i32(my_handle, "ticks", (int32_t)(*ticks));
    printf("set ticks NVS = ");
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    if(err != ESP_OK){
        if(DEBUG){
            printf("setExperimentPhaseTick(): nvs_set_i32(): tick error - ESP_NONCRITICAL\n");
        }else {
            logError("setExperimentPhaseTick(): nvs_set_i32(): tick error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written to flash 
    err = nvs_commit(my_handle);
    printf("commit phase & ticks NVS = ");
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    if(err != ESP_OK){
        if(DEBUG){
            printf("setExperimentPhaseTick(): nvs_commit(): Commit error - ESP_NONCRITICAL\n");
        }else {
            logError("setExperimentPhaseTick(): nvs_commit(): Commit error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Close
    nvs_close(my_handle);   

    // NOMINAL EXIT POINT
    return ESP_OK;


}

// gets the previous phase and tick from memory 
esp_err_t getExerpimentPhaseTicks(int *phase, int *ticks){
    if(DEBUG){
        printf("Getting previously stored phase and ticks from NVS\n");
    }
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK){
        if(DEBUG){
            printf("getExperimentPhaseTick(): nvs_open(): Open error - ESP_NONCRITICAL\n");
        }else {
            logError("getExperimentPhaseTick(): nvs_open(): Open error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Read phase
    err = nvs_get_i32(my_handle, "phase", (int32_t*)phase);
    switch (err) {
        case ESP_OK:
        //    printf("Done\n");
            printf("phase = %d\n", (*phase));
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    if(err != ESP_OK){
        if(DEBUG){
            printf("getExperimentPhaseTick(): nvs_get_i32(): phase Read error - ESP_NONCRITICAL\n");
        }else {
            logError("getExperimentPhaseTick(): nvs_get_i32(): phase Read error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }

    // Read ticks
    err = nvs_get_i32(my_handle, "ticks", (int32_t*)ticks);
    switch (err) {
        case ESP_OK:
        //    printf("Done\n");
            printf("ticks = %d\n", (*ticks));
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(err));
    }

    if(err != ESP_OK){
        if(DEBUG){
            printf("getExperimentPhaseTick(): nvs_get_i32():ticks Read error - ESP_NONCRITICAL\n");
        }else {
            logError("getExperimentPhaseTick(): nvs_get_i32(): ticks Read error - ESP_NONCRITICAL\n");
        }
        return ESP_NONCRITICAL;
    }


    return ESP_OK;
}

// takes the non-zero phase and tick and re-centers the cariage. 
esp_err_t neutralise(int *phase, int *ticks, enum flowFlag *flowFlagPtr){


    printf("neutralise(): phase = %d, ticks = %d\n",(*phase),(*ticks));
    if((*phase) ==1 ){
        // move backward by MaxTicks - ticks
        *ticks = 2*TICKS_PER_REV - *ticks;
        EnMotor(1); 
        RunMotor(0, ticks,flowFlagPtr);
        EnMotor(0);
        if (*ticks ==0 ){
            // neutralisation has been completed
            *phase = 0;
        }else{
            *phase = 1;    // neutralisation was interupted
        }
        
    }else if ((*phase) == 2){
       if ((*ticks)>2*TICKS_PER_REV){
            // Move backward 2*TICKS_PER_REV - (4*TICKS_PER_REV - ticks)
            *ticks = 2*TICKS_PER_REV - (4*TICKS_PER_REV - *ticks);
            EnMotor(1);
            RunMotor(0,ticks,flowFlagPtr);
            EnMotor(0);
       }else if((*ticks<2*TICKS_PER_REV)){
            // Move forwards 2*TICKS_PER_REV - ticks
            *ticks = 2*TICKS_PER_REV - *ticks;
            EnMotor(1);
            RunMotor(1,ticks,flowFlagPtr);
            EnMotor(0);
       }else if((*ticks) == 2*TICKS_PER_REV){
            printf("Already neutralised\n");
       }
       if (*ticks == 0 ){
            // neutralisation has been completed
            *phase = 0;
        }else{
            *phase = 2;    // neutralisation was interupted
        }
    }else if ((*phase) ==3){
        // Complete the remaining ticks to get to neutral
        EnMotor(1);         // enable motor
        RunMotor(1, ticks, flowFlagPtr);  
        EnMotor(0);
         if (*ticks == 0 ){
            // neutralisation has been completed
            *phase = 0;
        }else{
            *phase = 3;    // neutralisation was interupted
        }
    }else{
        printf("neutralise(): incompatible phase\n");
    }
    if(*phase != 0){
        logError("Neutralisation was interupted\n ");
    }else{
        logError("neutralisation was successful\n");
    }
    return ESP_OK;  
}

/**
@brief Logs the given information to a 2d array and prints the entire array.
@param info The information to be logged.
*/
void logError(const char* info, ...){

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