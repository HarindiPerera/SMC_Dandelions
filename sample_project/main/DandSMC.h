#ifndef DANDSMC_H
#define DANDSMC_H

#include "esp_err.h"
#include "driver/spi_master.h"


#define DEBUG true                        // change this to change where messages go to
// HARDWARE CONSTANTS
#define TICKS_PER_REV 100          // The number of ticks per revolution of our steper motor
#define WD_DELAY_MS 10              // settling time for external watchdog timer. 

// QUEUE  CONSTANTS
#define QUEUE_LENGTH    10              //changed from 10 to 20 
#define ITEM_SIZE       sizeof(uint32_t)

// HARDWARE PIN DEFINITIONS
#define MFAULT 35
#define FFAULT 34
#define MSLEEP 33
#define MVEN 32
#define MOTEN 27
#define MOTSTEP 26
#define MOTDIR 25
#define WDI 23
#define I2C_SCL_PIN 22                          //ESP32 SCL 22
#define I2C_SDA_PIN 21                          //ESP32 SDA 21  
#define CAN_INT 19

#define ADCRPWR 17                              //Right ADC power on gpio 17
#define ADCLPWR 16                              //Left ADC power on gpio 16
#define PIN_NUM_CS   15
#define PIN_NUM_CLK  14
#define PIN_NUM_MOSI 13
#define PIN_NUM_MISO 12

#define DBCON 4

// GPIO ASSIGNMENT
#define OUTPUT_BIT_MASK       0b000000000110110000001110000000000000
#define INPUT_BIT_MASK        0b110000000000000000000001000000000001
#define INPUT_OUTPUT_BIT_MASK 0b001100001000001000110000000000000000    //Set up spi pins 12 13 14 15
// Pull Up and Pull down masks
#define PU_MASK               0b110000000000000000001100000000000000
#define PD_MASK               0b001100001110000000110011000000000000


// I2C CONSTANTS
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0                //*I2C bus number on ESP32
#define I2C_MASTER_TX_BUF_DISABLE  0            //i2c master does not need a buffer
#define I2C_MASTER_RX_BUF_DISABLE  0            //i2c master does not need a buffer

// I2C Addresses          
#define ADC_ADDR_1 0x6A                         //Device address 1101 | ADC1 : 100 |
#define ADC_ADDR_2 0x6C                         //Device address 1101 | ADC2 : 010 |

//uint8_t ADC_CH[] = {0X98 , 0XB8 , 0XD8 , 0XF8};

#define ACK_EN 0x1                              //Master checks Ack enabled
#define ACK_DIS 0x0                             //Master checks Ack disabled

#define DATA_LENGTH 512                         //Max Buffer Len
#define RW_LENGTH 129                           //Reg READ_WRITE len 

#define SAMPLE_DELAY_MS 1000                    //delay between ADC samples in milliseconds

// Patrick Implementation Defines 

#define CANSPI_HOST  HSPI_HOST
#define CANSPI_HOST_1 VSPI_HOST

#define TASK1_PRIORITY 1    //should this be 16/15? priority goes backward ask patrick
#define TASK2_PRIORITY 2

#define TASK1_STACK_SIZE 8096
#define TASK2_STACK_SIZE 2048

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 1

// Structure for quick pin itteration (implemented in DandSMC.c)
typedef struct {
    int pin;
    const char* name;
}GPIO_Pins;

enum flowFlag {
    GREENLIGHT,
    NOMINAL,
    ESD,
    NO_DATA,
}; 

// Patrick Task Params struct
typedef struct {
    spi_device_handle_t spi;
    enum flowFlag ctrlFlowFlag;
} TaskParams_t;


 
// DANDELIONS FUNCTION PROTOYPES-


void SMC_Spacecraft_State(spi_device_handle_t* spi);
void SMC_PowerDown(spi_device_handle_t* spi);
void SMC_BeginOperation(spi_device_handle_t* spi);
void SMC_CeaseOperation(spi_device_handle_t* spi);
void SMC_PowerDownAll(spi_device_handle_t* spi);
void SMC_Query(spi_device_handle_t* spi);
void SMC_Transmit(spi_device_handle_t* spi); 
void SMC_Flow(spi_device_handle_t* spi); 
void SMC_Result(spi_device_handle_t* spi);
void print_bits2(uint8_t n);
esp_err_t setupHW(void);                                        //Configures all the gpio/direction/pullmode/intr status

esp_err_t ADC_Pwr(bool en);
esp_err_t I2C_Init(void);
esp_err_t I2C_Scan();                                           //Looks for I2C devices over the bus and prints on console

//esp_err_t buf_to_int(uint8_t* buffer,  int size);              //Converts a buffer of uint8_t values to an array of int16_t values.
esp_err_t ADC_Read(uint8_t address , int Channel);

esp_err_t EnMotor(bool en);                                      //Enables the motor
esp_err_t RunMotor(bool dir, int *ticks, enum flowFlag *flowFlagPtr);     //Runs a stepper motor for a specified number of steps in a specified direction.

esp_err_t checkGPIOS(void);
esp_err_t checkMem(void);
esp_err_t checkCANctrl(void);

esp_err_t systemHealthCheck(void);
esp_err_t calibrate(void);
void PollFaultIndicatorsTask(void* pvParamemters);
esp_err_t RunExperiment(int *phase, int* tick, enum flowFlag *flowFlagPtr);      // input arguments are pointers to variabels that are saved in NVS in case of Emergency shut down. 

    

esp_err_t updateExperimentCount(bool erase, int *count);                // going to edit this to take a pointer to be edited, so that the return type is not an int. 
esp_err_t neutralise(int *phase, int *tick, enum flowFlag *flowFlagPtr);
esp_err_t setExperimentPhaseTicks(int *phase, int *ticks, bool reset);
esp_err_t getExerpimentPhaseTicks(int *phase, int* ticks);

    
void logError(const char* info, ...); 


#endif