/*******************************************************************************
   ISO Transfer Protocol: API Defines Header File

  Company:
    Dandelions

  File Name:
    iso_tp.h

  Summary:
    This header file contains object declarations used in the API.

  Description:
    None.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************

//#include "spi/drv_spi.h"
#include <stdlib.h>
#include <stdint.h>
#include "driver/spi_master.h"

#define TYPE_SINGLE 0
#define TYPE_FIRST 1
#define TYPE_CONSECUTIVE 2
#define TYPE_FLOW 3

#define ISO_TP_MAX_SIZE 64
#define MAX_BLOCK_SIZE 2048

#define N_PCI_SF    0x00 // single frame
#define N_PCI_FF    0x10 // first frame
#define N_PCI_CF    0x20 // consecutive frame
#define N_PCI_FC    0x30 // flow control

typedef enum FLOW_CONTROL_FLAG {
    CONTINUE = 0,
    WAIT = 1,
    ABORT = 2,
} FLOW_CONTROL_FLAG;

typedef enum N_PCI_HEADER {
    PCI_12 = 2,
    PCI_32 = 6,
} N_PCI_HEADER;

typedef struct iso_tp_control_t {
    FLOW_CONTROL_FLAG flow;
    uint8_t block_size;
    uint8_t sep_time;
    uint32_t num_segments;
    uint32_t index;
    N_PCI_HEADER pci;
} iso_tp_control_t;

typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[ISO_TP_MAX_SIZE];
} iso_tp_message_t;

bool iso_tp_send(spi_device_handle_t* spi, uint8_t* data, uint32_t data_len);

uint8_t* iso_tp_receive(spi_device_handle_t* spi);

// void iso_tp_transfer(spi_device_handle_t* spi, uint8_t* txd);

void handle_transfer(spi_device_handle_t* spi, uint8_t* txd, uint8_t* next);

void transmit_consecutive_frame(spi_device_handle_t* spi, uint8_t* buf, uint8_t index);

void transmit_x_frames(spi_device_handle_t* spi, uint8_t* txd, uint8_t* next, uint8_t frames);

void transmit_all_frames(spi_device_handle_t* spi, uint8_t* txd, uint8_t* next);