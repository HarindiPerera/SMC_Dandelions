#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_err.h"
#include "DandSMC.h"


/**
 * @brief Initialise I2C Comms on ESP32
 *This function initialises the master with the specified config params
 * @return None
 */
static void i2c_master_init()
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
}



/**
 * @brief Reads ADC value over I2C.
 *This function sends a command over channel 0 to adc. Waits for 
 conversation and reads value over adc.
 * @return 16bit signed integer val
 */
static int16_t read_adc_val()
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
    return value;
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




void app_main(void)
{

    printf("start of main\n");

    //Init dem comms
    i2c_master_init();

    
    //yeahbois
    while(true)
    {
        int16_t adc_val = read_adc_val();
        printf("ADC Value : %d/n" , adc_val);
    }

}

