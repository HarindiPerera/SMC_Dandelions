#ifndef DANDSMC_H
#define DANDSMC_H


#include "esp_err.h"

#define DEBUG true                        // change this to change where messages go to
// HARDWARE CONSTANTS
#define TICKS_PER_REV   6000              // The number of ticks per revolution of our steper motor
#define WD_DELAY_MS     10                // setteling time for external watchdog timer. 

// QUEUE  CONSTANTS
#define QUEUE_LENGTH    10                //changed from 10 to 20 
#define ITEM_SIZE       sizeof(uint32_t)

// HARDWARE PIN DEFINITIONS
#define MOTDIR 25
#define MOTSTEP 26
#define MOTEN 27
#define MVEN 32
#define MSLEEP 33
#define FFAULT 34
#define MFAULT 35
#define WDI 23
#define ADCLPWR 16                              //Left ADC power on gpio 16
#define ADCRPWR 17                              //Right ADC power on gpio 17
#define I2C_SDA_PIN 21                          //ESP32 SDA 21  
#define I2C_SCL_PIN 22                          //ESP32 SCL 22

// GPIO ASSIGNMENT
#define OUTPUT_BIT_MASK       0b000000000110100000000000000000000000
#define INPUT_BIT_MASK        0b110000000000000000000000000000000000
#define INPUT_OUTPUT_BIT_MASK 0b001100001000000000110000000000000000
#define PU_MASK               0b110000000000011000000000000000000000
#define PD_MASK               0b001100001110100000110000000000000000

// I2C CONSTANTS
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0                //*I2C bus number on ESP32
#define I2C_MASTER_TX_BUF_DISABLE  0            //i2c master does not need a buffer
#define I2C_MASTER_RX_BUF_DISABLE  0            //i2c master does not need a buffer

                    
#define ADC_ADDR_1 0x68                         //Device address 1101 | ADC1 : 100 |
#define ADC_ADDR_2 0x69                         //Device address 1101 | ADC2 : 010 | 

#define ACK_EN 0x1                              //Master checks Ack enabled
#define ACK_DIS 0x0                             //Master checks Ack disabled

#define DATA_LENGTH 512                         //Max Buffer Len
#define RW_LENGTH 129                           //Reg READ_WRITE len 

#define SAMPLE_DELAY_MS 1000                    //delay between ADC samples in milliseconds


// Structure for quick pin itteration (implemented in DandSMC.c)
typedef struct {
    int pin;
    const char* name;
}GPIO_Pins;



// DANDELIONS FUNCTION PROTOYPES-  
void print_check(void);                             //Prints a message to the console
esp_err_t setupHW(void);                            //Configures all the gpio/direction/pullmode/intr status

esp_err_t ADC_Pwr(bool en);
esp_err_t I2C_Init(void);
esp_err_t I2C_Scan();                                    //Looks for I2C devices over the bus and prints on console
void Print_buffer(uint8_t* buf);
esp_err_t buf_to_int(uint8_t* buffer,  int size);        //Converts a buffer of uint8_t values to an array of int16_t values.
esp_err_t ADC_Read(uint8_t address);

esp_err_t EnMotor(bool en);                              //Enables the motor
int RunMotor(bool dir, int ticks);                  //Runs a stepper motor for a specified number of steps in a specified direction.

int updateExperimentCount(bool erase);
void checkGPIOS(void);
void checkMem(void);
void checkADC(void);
void checkCANctrl(void);

void systemHealthCheck(void);
void calibrate(void);
void PollFaultIndicatorsTask(void* pvParamemters);
int RunExperiment(void);

void logError(char* str);         // not yet implemented 

#endif