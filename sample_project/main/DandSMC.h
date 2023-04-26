#ifndef DANDSMC_H
#define DANDSMC_H

#include "esp_err.h"

// HARDWARE CONSTANTS
#define TICKS_PER_REV 6000          // The number of ticks per revolution of our steper motor


#define QUEUE_LENGTH    10              //changed from 10 to 20 
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

// For these masks each bit coresponds to a gpio, bit 4 = gpio 4 ect
#define OUTPUT_BIT_MASK 0b001100001110100000110000000000100000
#define INPUT_BIT_MASK  0xC0E000000

// DANDELIONS ERROR CODES
#define SUCCESS 1
#define MOTOR_FAULT -1
#define FUSE_FAULT -2
#define FORCE_QUIT -3
#define HW_FAULT -4


// Structure for quick pin itteration
typedef struct {
    int pin;
    const char* name;
}GPIO_Pins;

// This is a constant structure of GPIO_Pins that relate specifically to the fault indicators
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


/*STRUCTS*/

// Structure for quick pin itteration
typedef struct {
    int pin;
    const char* name;
}GPIO_Pins;

// This is a constant structure of GPIO_Pins that relate specifically to the fault indicators
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



// DANDELIONS FUNCTION PROTOYPES-  
void print_check(void);                             //Prints a message to the console
esp_err_t setupHW(void);                            //Configures all the gpio/direction/pullmode/intr status

void ADC_Pwr(bool en);
void I2C_Init(void);
void I2C_Scan();                                    //Looks for I2C devices over the bus and prints on console
void Print_buffer(uint8_t* buf);
void buf_to_int(uint8_t* buffer,  int size);        //Converts a buffer of uint8_t values to an array of int16_t values.
void ADC_Read(uint8_t address);

void EnMotor(bool en);                              //Enables the motor
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

#endif