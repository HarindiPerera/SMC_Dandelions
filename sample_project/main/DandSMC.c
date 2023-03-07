
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"

/**
 *@brief Configures GPIO pins for input and output
 *This function initializes GPIO pins for input and output as specified by theioConfig parameter. The outputs are configured with no interrupts enabled,
set as outputs with a defined output bit mask, and with pulldown mode enabled
and pullup mode disabled.
*@return [gpio_config_t] Returns ESP_OK on success, otherwise an error code indicating
the cause of the failure.
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
 *@brief prints a message to the console
 *Super basic print of "hey now" for debugging
*@return None
 */
void print_check(void)
{
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
 * @brief Runs a stepper motor for a specified number of steps in a specified direction.
 * 
 * The function sets the direction and moves a stepper motor a certain number of steps,
 * returning SUCCESS if successful, or HW_FAULT if there are errors.
 * 
 * @param dir A boolean value indicating the direction of the motor (`true` for clockwise, `false` for counter-clockwise).
 * @param ticks The number of steps to move the motor.
 * 
 * @return An integer value indicating the result of the operation (`SUCCESS` for success, `HW_FAULT` for a hardware fault).
 */ 
int RunMotor(bool dir, int ticks){
    esp_err_t err = ESP_OK;                      // create an error variable to check the stuff. 
    if(gpio_set_level(MOTDIR,dir)!=ESP_OK){
        printf("ERROR: Error in setting the motor direction");
        return HW_FAULT;                        // return a HW fault condition if unable to set direction
    }
    for(int i =0; i<=ticks; i++){

        //Run Print
        err = gpio_set_level(MOTSTEP,1);
        vTaskDelay(5/portTICK_PERIOD_MS);
        err = gpio_set_level(MOTSTEP,0);
        vTaskDelay(5/portTICK_PERIOD_MS);

        if(err!=ESP_OK){
            printf("ERROR: Error in setting toggeling the MOTSTEP pin. Breaking from loop.\n");
            return HW_FAULT;        // return fault if unable to toggle MOTSTEP pin
        }  
    }
    printf("RUN: Actuation complete\n");
    return SUCCESS;                 // ELSE return success after motor has been pulsed 'ticks' number of times. 
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
 * @brief Read ADC value in One-Shot Conversion mode
 *
 * @param[in] channel ADC channel to read (0-3)
 * @return ADC value as a signed 16-bit integer
 */
/*static int16_t read_adc_oneshot(uint8_t channel)
{
    uint8_t buf[3];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    // Send the I2C address and write flag - true shows the master should send a stop condition after writing data
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | I2C_MASTER_WRITE, true);
    
    // Set up the conversion command based on the selected channel and mode
    uint8_t conversion_cmd = 0x80 | ((channel & 0x03) << 4);

    // Send the conversion command
    i2c_master_write_byte(cmd, conversion_cmd, true);

    // End the I2C communication
    i2c_master_stop(cmd);

    // Send the command to the I2C bus
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Wait for the conversion to complete
    vTaskDelay(SAMPLE_DELAY_MS / portTICK_PERIOD_MS);

    // Read the result from the ADC
    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ESP_SLAVE_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[2], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Convert the raw data to a signed 16-bit integer
    int16_t value = ((buf[0] & 0x0F) << 8) | buf[1];
    if (value & 0x0800)
    {
        value |= 0xF000;
    }

    return value;
}*/