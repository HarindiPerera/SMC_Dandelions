/*******************************************************************************
  CAN FD SPI Driver: Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_canfdspi_api.c

  Summary:
    API implementation.

  Description:
    .
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and
any derivatives exclusively with Microchip products. It is your responsibility
to comply with third party license terms applicable to your use of third party
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS,
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO
THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID
DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// #include "esp_system.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"
#include "drv_canfdspi_defines.h"
//#include "../spi/drv_spi.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include <string.h>
#include <msg/msg.h>

// *****************************************************************************
// *****************************************************************************
// Section: Defines

#define CRCBASE    0xFFFF
#define CRCUPPER   1
#define SPI_DEFAULT_BUFFER_LENGTH 48
#define Nop() asm("nop")


// *****************************************************************************
// *****************************************************************************
// Section: Variables

// //! SPI Transmit buffer
// uint8_t spiTransmitBuffer[SPI_DEFAULT_BUFFER_LENGTH];

// //! SPI Receive buffer
// uint8_t spiReceiveBuffer[SPI_DEFAULT_BUFFER_LENGTH];

//! Reverse order of bits in byte
const uint8_t BitReverseTable256[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

//! Look-up table for CRC calculation
const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


// *****************************************************************************
// *****************************************************************************
// Section: Reset

int8_t DRV_CANFDSPI_Reset(spi_device_handle_t* spi)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = cINSTRUCTION_RESET;
    t.addr = 0;
    t.length = 16;
    t.user = (void*)1;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t); //patrick

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: SPI Access Functions
// void print_bits(uint8_t n) {
//     for (int i = 7; i >= 0; i--) {
//         printf("%d", (n >> i) & 1);
//     }
//     printf("\n");
// }

int8_t DRV_CANFDSPI_ReadByte(spi_device_handle_t* spi, uint16_t address, uint8_t *rxd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = cINSTRUCTION_READ;
    t.addr = address;
    t.length = 16;
    t.rxlength = 8;
    t.rx_buffer = malloc(sizeof(uint8_t));
    // memcpy(t.rx_buffer, rxd, sizeof(uint8_t));
    t.user = (void*)1;
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    // print_bits(*(uint8_t *) t.rx_buffer);
    memcpy(rxd, t.rx_buffer, sizeof(uint8_t));
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteByte(spi_device_handle_t* spi, uint16_t address, uint8_t txd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = cINSTRUCTION_WRITE;
    t.addr = address;
    t.tx_buffer = malloc(sizeof(uint8_t));
    t.length = 16;
    t.user = (void*)1;

    memcpy(t.tx_buffer, &txd, sizeof(uint8_t));
    // printf("value: %d",txd);
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReadWord(spi_device_handle_t* spi, uint16_t address, uint32_t *rxd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_READ;
    t.addr = address;
    t.length = 48;
    t.rxlength = 32;
    t.rx_buffer = (uint32_t*) malloc(sizeof(uint32_t));
    t.user = (void*)1;
    
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    
    memcpy(rxd,t.rx_buffer,sizeof(uint32_t));
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteWord(spi_device_handle_t* spi, uint16_t address,
        uint32_t txd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE;
    t.addr = address;
    t.tx_buffer = malloc(sizeof(uint32_t));
    memcpy(t.tx_buffer, &txd, sizeof(uint32_t));
    
    t.length = 48;
    t.user = (void*)1;
    
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReadHalfWord(spi_device_handle_t* spi, uint16_t address, uint16_t *rxd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_READ;
    t.addr = address;
    t.length = 32;
    t.rxlength = 16;
    t.rx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*2);
    t.user = (void*)1;
    
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);

    // Update data
    uint8_t i;
    uint16_t x;
    *rxd = 0;
    for (i = 0; i < 2; i++) {
        x = ((uint16_t*) t.rx_buffer)[i];
        *rxd += x << ((i)*8);
    }
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }
    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteHalfWord(spi_device_handle_t* spi, uint16_t address,
        uint16_t txd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE;
    t.addr = address;
    t.tx_buffer = malloc(sizeof(uint16_t));
    memcpy(t.tx_buffer, &txd, sizeof(uint16_t));
    t.length = 32;
    t.user = (void*)1;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteByteSafe(spi_device_handle_t* spi, uint16_t address,
        uint8_t txd)
{
    uint16_t crcResult = 0;

    // Compose command
    uint8_t* crcBuf = malloc(sizeof(uint8_t)*3);
    uint16_t addr = ((uint16_t) cINSTRUCTION_WRITE_SAFE << 12) + address;
    memcpy(crcBuf, &addr, sizeof(uint16_t));
    memcpy(crcBuf+2, &txd, sizeof(uint8_t));
    // crcBuf[0] = (uint8_t) ((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
    // crcBuf[1] = (uint8_t) (address & 0xFF);
    // crcBuf[2] = txd;

    // Add CRC
    crcResult = DRV_CANFDSPI_CalculateCRC16(crcBuf, sizeof(crcBuf));

    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE_SAFE;
    t.addr = address;
    // t.tx_buffer = (uint8_t*)&txd;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*3);
    memcpy(t.tx_buffer, &txd, sizeof(uint8_t));
    memcpy(t.tx_buffer+1, &crcResult, sizeof(uint16_t));
    // txbuf[0] = txd;
    // txbuf[1] = ((crcResult >> 8) & 0xFF);
    // txbuf[2] = (crcResult & 0xFF);
    // t.tx_buffer = (uint8_t*)&txbuf;

    t.length = 40;
    t.user = (void*)1;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    if (crcBuf != NULL) {
        free(crcBuf);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteWordSafe(spi_device_handle_t* spi, uint16_t address,
        uint32_t txd)
{
    uint16_t crcResult = 0;

    // Compose command
    // uint8_t* crcBuf = malloc(sizeof(uint8_t)*6);
    // crcBuf[0] = (uint8_t) ((cINSTRUCTION_WRITE_SAFE << 4) + ((address >> 8) & 0xF));
    // crcBuf[1] = (uint8_t) (address & 0xFF);
    uint8_t* crcBuf = malloc(sizeof(uint8_t)*6);
    uint16_t addr = ((uint16_t) cINSTRUCTION_WRITE_SAFE << 12) + address;
    memcpy(crcBuf, &addr, sizeof(uint16_t));
    memcpy(crcBuf+2, (uint8_t*) txd, 4*sizeof(uint8_t));

    // Add CRC
    crcResult = DRV_CANFDSPI_CalculateCRC16(crcBuf, sizeof(crcBuf));

    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE_SAFE;
    t.addr = address;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*6);
    memcpy(t.tx_buffer, &txd, sizeof(uint32_t));
    memcpy(t.tx_buffer+4, &crcResult, sizeof(uint16_t));
    // uint8_t* txbuf = (uint8_t*) malloc(sizeof(uint8_t)*6);
    // memcpy(txbuf, (uint8_t*)&txd, 4);
    // txbuf[4] = ((crcResult >> 8) & 0xFF);
    // txbuf[5] = (crcResult & 0xFF);
    // t.tx_buffer = (uint8_t*)&txbuf;

    t.length = 72;
    t.user = (void*)1;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    if (crcBuf != NULL) {
        free(crcBuf);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReadByteArray(spi_device_handle_t* spi, uint16_t address,
        uint8_t *rxd, uint8_t nBytes)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_READ;
    t.addr = address;
    t.length = 16+8*nBytes;
    // t.length = 8*nBytes;
    t.rxlength = 8*nBytes;
    t.rx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*nBytes);
    t.user = (void*)1;
    
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);

    // for (int i = 0; i<nBytes; i++) {
    //     printf("%u ",t.rx_buffer[i]);
    // }
    // printf("\n");

    memcpy(rxd, t.rx_buffer, nBytes);
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }
    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReadByteArrayWithCRC(spi_device_handle_t* spi, uint16_t address,
        uint8_t *rxd, uint8_t nBytes, bool fromRam, bool* crcIsCorrect)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_READ_CRC;
    t.addr = address;
    t.length = 40+8*nBytes;
    t.rxlength = 8*nBytes;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t));
    if (fromRam) {
        uint8_t nil = 0;
        memcpy(t.tx_buffer, &nil, sizeof(uint8_t));
    } else {
        memcpy(t.tx_buffer, &nBytes, sizeof(uint8_t));
        // t.tx_buffer = &nBytes;
    }
    t.rx_buffer = (uint8_t*) malloc(nBytes);
    t.user = (void*)1;
    
    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);

    // Get CRC from controller
    uint16_t crcFromSpiSlave = (((uint16_t*) t.rx_buffer)[nBytes - 2] << 8) + ((uint16_t*) t.rx_buffer)[nBytes - 1];

    // Use the receive buffer to calculate CRC
    // First three bytes need to be command
     uint8_t* crcBuf = malloc(sizeof(uint8_t)*3);
    uint16_t addr = ((uint16_t) cINSTRUCTION_WRITE_SAFE << 12) + address;
    memcpy(crcBuf, &addr, sizeof(uint16_t));
    memcpy(crcBuf+2, &nBytes, sizeof(uint8_t));
    // crcBuffer[0] = ((uint8_t) t.cmd << 4) + ((uint8_t) (t.addr >> 8) & 0xF);
    // crcBuffer[1] = (uint8_t) t.addr & 0xFF;
    // crcBuffer[2] = (uint8_t) nBytes;
    // memcpy(crcBuffer + 3, t.rx_buffer, nBytes);
    uint16_t crcAtController = DRV_CANFDSPI_CalculateCRC16(crcBuf, nBytes + 3);

    // Compare CRC readings
    if (crcFromSpiSlave == crcAtController) {
        *crcIsCorrect = true;
    } else {
        *crcIsCorrect = false;
    }

    memcpy(rxd, t.rx_buffer, sizeof(nBytes));
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    if (crcBuf != NULL) {
        free(crcBuf);
    }
    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteByteArray(spi_device_handle_t* spi, uint16_t address,
        uint8_t *txd, uint8_t nBytes)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE;
    t.addr = address;
    // t.tx_buffer = (uint8_t*) txd;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*nBytes);
    memcpy(t.tx_buffer, txd, nBytes);
    t.length = 16+8*nBytes;
    t.user = (void*)1;
    t.flags = 0;
    
    // for (int i = 0; i<nBytes; i++) {
    //     printf("%d ",t.tx_buffer[i]);
    // }
    // printf("\n");

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteByteArrayWithCRC(spi_device_handle_t* spi, uint16_t address,
        uint8_t *txd, uint8_t nBytes, bool fromRam)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE_CRC;
    t.addr = address;
    t.length = 40+8*nBytes;
    t.user = (void*)1;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*(nBytes+3));

    uint8_t* crcBuffer = (uint8_t*) malloc(nBytes+3);
    uint16_t addr = ((uint16_t) cINSTRUCTION_WRITE_CRC << 12) + address;
    memcpy(crcBuffer, &addr, sizeof(uint16_t));
    // crcBuffer[0] = (uint8_t) (t.cmd << 4) + (uint8_t) ((t.addr >> 8) &0xF);
    // crcBuffer[1] = (uint8_t) t.addr & 0xFF;
    if (fromRam) {
        uint8_t nil = 0;
        memcpy(crcBuffer+2, &nil, sizeof(uint8_t));
        // crcBuffer[2] = 0;
        // t.tx_buffer = 0;
    } else {
        memcpy(crcBuffer+2, &nBytes, sizeof(uint8_t));
        // crcBuffer[2] = nBytes;
        // t.tx_buffer = &nBytes;
    }
    memcpy(crcBuffer+3, txd, nBytes);
    uint16_t crcResult = DRV_CANFDSPI_CalculateCRC16(crcBuffer, sizeof(crcBuffer));
        
    // txbuf[0] = nBytes;
    memcpy(t.tx_buffer, &nBytes, sizeof(uint8_t));
    memcpy(t.tx_buffer+1, (uint8_t*)&txd, nBytes);
    memcpy(t.tx_buffer+nBytes+1, (uint8_t*)&crcResult, sizeof(uint16_t));
    // txbuf[nBytes+1] = ((crcResult >> 8) & 0xFF);
    // txbuf[nBytes+2] = (crcResult & 0xFF);
    // t.tx_buffer = (uint8_t*)&txbuf;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (t.tx_buffer != NULL) {
        free(t.tx_buffer);
    }
    if (crcBuffer != NULL) {
        free(crcBuffer);
    }
    
    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReadWordArray(spi_device_handle_t* spi, uint16_t address,
        uint32_t *rxd, uint16_t nWords)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_READ;
    t.addr = address;
    t.length = 16+8*nWords*4;
    t.rxlength = 8*nWords*4;
    t.user = (void*)1;
    t.rx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*4*nWords);

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);
    if (spiTransferError) {
        return spiTransferError;
    }

    memcpy(rxd,t.rx_buffer,4*nWords);
    
    if (t.rx_buffer != NULL) {
        free(t.rx_buffer);
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_WriteWordArray(spi_device_handle_t* spi, uint16_t address,
        uint32_t *txd, uint16_t nWords)
{
    uint16_t i, j, n;
    REG_t w;

    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.cmd = cINSTRUCTION_WRITE;
    t.addr = address;
    t.length = 16+8*nWords*4;
    t.user = (void*)1;
    t.tx_buffer = (uint8_t*) malloc(sizeof(uint8_t)*4*nWords);
    memcpy(t.tx_buffer, txd, 4*nWords);

    // // Convert ByteArray to word array
    // uint8_t* x = (uint8_t*) malloc(sizeof(uint8_t)*4*nWords);
    // n = 0;
    // for (i = 0; i < nWords; i++) {
    //     w.word = txd[i];
    //     for (j = 0; j < 4; j++, n++) {
    //         x[n] = w.byte[j];
    //     }
    // }
    // t.tx_buffer = x;

    int8_t spiTransferError = spi_device_transmit_cs(spi,&t);

    if (t.tx_buffer!=NULL) {
        free(t.tx_buffer);
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Configuration

int8_t DRV_CANFDSPI_Configure(spi_device_handle_t* spi, CAN_CONFIG* config)
{
    REG_CiCON ciCon;
    int8_t spiTransferError = 0;

    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    ciCon.bF.DNetFilterCount = config->DNetFilterCount;
    ciCon.bF.IsoCrcEnable = config->IsoCrcEnable;
    ciCon.bF.ProtocolExceptionEventDisable = config->ProtocolExpectionEventDisable;
    ciCon.bF.WakeUpFilterEnable = config->WakeUpFilterEnable;
    ciCon.bF.WakeUpFilterTime = config->WakeUpFilterTime;
    ciCon.bF.BitRateSwitchDisable = config->BitRateSwitchDisable;
    ciCon.bF.RestrictReTxAttempts = config->RestrictReTxAttempts;
    ciCon.bF.EsiInGatewayMode = config->EsiInGatewayMode;
    ciCon.bF.SystemErrorToListenOnly = config->SystemErrorToListenOnly;
    ciCon.bF.StoreInTEF = config->StoreInTEF;
    ciCon.bF.TXQEnable = config->TXQEnable;
    ciCon.bF.TxBandWidthSharing = config->TxBandWidthSharing;

    spiTransferError = DRV_CANFDSPI_WriteWord(spi,  cREGADDR_CiCON, ciCon.word);
    if (spiTransferError) {
        return -1;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ConfigureObjectReset(CAN_CONFIG* config)
{
    REG_CiCON ciCon;
    ciCon.word = canControlResetValues[cREGADDR_CiCON / 4];

    config->DNetFilterCount = ciCon.bF.DNetFilterCount;
    config->IsoCrcEnable = ciCon.bF.IsoCrcEnable;
    config->ProtocolExpectionEventDisable = ciCon.bF.ProtocolExceptionEventDisable;
    config->WakeUpFilterEnable = ciCon.bF.WakeUpFilterEnable;
    config->WakeUpFilterTime = ciCon.bF.WakeUpFilterTime;
    config->BitRateSwitchDisable = ciCon.bF.BitRateSwitchDisable;
    config->RestrictReTxAttempts = ciCon.bF.RestrictReTxAttempts;
    config->EsiInGatewayMode = ciCon.bF.EsiInGatewayMode;
    config->SystemErrorToListenOnly = ciCon.bF.SystemErrorToListenOnly;
    config->StoreInTEF = ciCon.bF.StoreInTEF;
    config->TXQEnable = ciCon.bF.TXQEnable;
    config->TxBandWidthSharing = ciCon.bF.TxBandWidthSharing;

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: Operating mode

int8_t DRV_CANFDSPI_OperationModeSelect(spi_device_handle_t* spi,
        CAN_OPERATION_MODE opMode)
{
    uint8_t d = 0;
    int8_t spiTransferError = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiCON + 3, &d);
    if (spiTransferError) {
        return -1;
    }
    printf("%X\n",d);

    // Modify
    d &= ~0x07;
    d |= opMode;

    printf("%X\n",d);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_CiCON + 3, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

CAN_OPERATION_MODE DRV_CANFDSPI_OperationModeGet(spi_device_handle_t* spi)
{
    uint8_t d = 0;
    CAN_OPERATION_MODE mode = CAN_INVALID_MODE;
    int8_t spiTransferError = 0;

    // Read Opmode
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiCON + 2, &d);
    if (spiTransferError) {
        return CAN_INVALID_MODE;
    }
    // Get Opmode bits
    d = (d >> 5) & 0x7;

    // Decode Opmode
    switch (d) {
        case CAN_NORMAL_MODE:
            mode = CAN_NORMAL_MODE;
            break;
        case CAN_SLEEP_MODE:
            mode = CAN_SLEEP_MODE;
            break;
        case CAN_INTERNAL_LOOPBACK_MODE:
            mode = CAN_INTERNAL_LOOPBACK_MODE;
            break;
        case CAN_EXTERNAL_LOOPBACK_MODE:
            mode = CAN_EXTERNAL_LOOPBACK_MODE;
            break;
        case CAN_LISTEN_ONLY_MODE:
            mode = CAN_LISTEN_ONLY_MODE;
            break;
        case CAN_CONFIGURATION_MODE:
            mode = CAN_CONFIGURATION_MODE;
            break;
        case CAN_CLASSIC_MODE:
            mode = CAN_CLASSIC_MODE;
            break;
        case CAN_RESTRICTED_MODE:
            mode = CAN_RESTRICTED_MODE;
            break;
        default:
            mode = CAN_INVALID_MODE;
            break;
    }

    return mode;
}

// int8_t DRV_CANFDSPI_LowPowerModeEnable(spi_device_handle_t* spi)
// {
//     int8_t spiTransferError = 0;
//     uint8_t d = 0;

// #ifdef MCP2517FD
//     // LPM not implemented
//     spiTransferError = -100;
// #else
//     // Read
//     spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_OSC, &d);
//     if (spiTransferError) {
//         return -1;
//     }

//     // Modify
//     d |= 0x08;

//     // Write
//     spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_OSC, d);
//     if (spiTransferError) {
//         return -2;
//     }
// #endif
    
//     return spiTransferError;    
// }

// int8_t DRV_CANFDSPI_LowPowerModeDisable(spi_device_handle_t* spi)
// {
//     int8_t spiTransferError = 0;
//     uint8_t d = 0;

// #ifdef MCP2517FD
//     // LPM not implemented
//     spiTransferError = -100;
// #else
//     // Read
//     spiTransferError = DRV_CANFDSPI_ReadByte(cREGADDR_OSC, &d);
//     if (spiTransferError) {
//         return -1;
//     }

//     // Modify
//     d &= ~0x08;

//     // Write
//     spiTransferError = DRV_CANFDSPI_WriteByte(cREGADDR_OSC, d);
//     if (spiTransferError) {
//         return -2;
//     }
// #endif
    
//     return spiTransferError;    
// }


// *****************************************************************************
// *****************************************************************************
// Section: CAN Transmit

int8_t DRV_CANFDSPI_TransmitChannelConfigure(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG* config)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Setup FIFO
    REG_CiFIFOCON ciFifoCon, ciFifoConVal;
    ciFifoCon.word = canFifoResetValues[0];
    ciFifoConVal.word = canFifoResetValues[0];
    ciFifoCon.txBF.TxEnable = 1;
    ciFifoCon.txBF.FifoSize = config->FifoSize;
    ciFifoCon.txBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.txBF.TxAttempts = config->TxAttempts;
    ciFifoCon.txBF.TxPriority = config->TxPriority;
    ciFifoCon.txBF.RTREnable = config->RTREnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    //printf("%X\n",ciFifoCon.word);
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, a, ciFifoCon.word);
    
    sleep(1);
    spiTransferError |= DRV_CANFDSPI_ReadWord(spi, a, &ciFifoConVal.word);
    //printf("%X\n",ciFifoConVal.word);

    spiTransferError |= memcmp(&ciFifoCon,&ciFifoConVal,sizeof(REG_CiFIFOCON));
    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG* config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    config->RTREnable = ciFifoCon.txBF.RTREnable;
    config->TxPriority = ciFifoCon.txBF.TxPriority;
    config->TxAttempts = ciFifoCon.txBF.TxAttempts;
    config->FifoSize = ciFifoCon.txBF.FifoSize;
    config->PayLoadSize = ciFifoCon.txBF.PayLoadSize;

    return 0;
}

int8_t DRV_CANFDSPI_TransmitQueueConfigure(spi_device_handle_t* spi,
        CAN_TX_QUEUE_CONFIG* config)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Setup FIFO
    REG_CiTXQCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    ciFifoCon.txBF.TxEnable = 1;
    ciFifoCon.txBF.FifoSize = config->FifoSize;
    ciFifoCon.txBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.txBF.TxAttempts = config->TxAttempts;
    ciFifoCon.txBF.TxPriority = config->TxPriority;

    a = cREGADDR_CiTXQCON;
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, a, ciFifoCon.word);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitQueueConfigureObjectReset(CAN_TX_QUEUE_CONFIG* config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    config->TxPriority = ciFifoCon.txBF.TxPriority;
    config->TxAttempts = ciFifoCon.txBF.TxAttempts;
    config->FifoSize = ciFifoCon.txBF.FifoSize;
    config->PayLoadSize = ciFifoCon.txBF.PayLoadSize;

    return 0;
}

int8_t DRV_CANFDSPI_TransmitChannelLoad(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush)
{
    uint16_t a;
    uint32_t fifoReg[3];
    uint32_t dataBytesInObject;
    REG_CiFIFOCON ciFifoCon;
    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
    int8_t spiTransferError = 0;

    // Get FIFO registers
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadWordArray(spi, a, fifoReg, 3);
    if (spiTransferError) {
        return -1;
    }

    // Check that it is a transmit buffer
    ciFifoCon.word = fifoReg[0];
    // printf("FifoCon: %X\n",ciFifoCon.word);
    if (!ciFifoCon.txBF.TxEnable) {
        return -2;
    }

    // Check that DLC is big enough for data
    dataBytesInObject = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) txObj->bF.ctrl.DLC);
    if (dataBytesInObject < txdNumBytes) {
        return -3;
    }

    // Get status
    ciFifoSta.word = fifoReg[1];

    // Get address
    ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    a = 4 * ciFifoUa.bF.UserAddress;
#else
    a = ciFifoUa.bF.UserAddress;
#endif
    a += cRAMADDR_START;

    // printf("value:%d\n",a);

    uint8_t txBuffer[MAX_MSG_SIZE];

    txBuffer[0] = txObj->byte[0]; //not using 'for' to reduce no of instructions
    txBuffer[1] = txObj->byte[1];
    txBuffer[2] = txObj->byte[2];
    txBuffer[3] = txObj->byte[3];

    txBuffer[4] = txObj->byte[4];
    txBuffer[5] = txObj->byte[5];
    txBuffer[6] = txObj->byte[6];
    txBuffer[7] = txObj->byte[7];

    uint8_t i;
    for (i = 0; i < txdNumBytes; i++) {
        txBuffer[i+8] = txd[i];
    }

    // Make sure we write a multiple of 4 bytes to RAM
    uint16_t n = 0;
    uint8_t j = 0;

    if (txdNumBytes % 4) {
        // Need to add bytes
        n = 4 - (txdNumBytes % 4);
        i = txdNumBytes + 8;

        for (j = 0; j < n; j++) {
            txBuffer[i + 8 + j] = 0;
        }
    }

    spiTransferError = DRV_CANFDSPI_WriteByteArray(spi, a, txBuffer, txdNumBytes + 8 + n);
    if (spiTransferError) {
        return -4;
    }

    uint8_t rx_buf[(MAX_DATA_BYTES+8+1)] = {0};
    // uint8_t rx_buf[sizeof(uint8_t)*(txdNumBytes+8+n)] = {0};
    // DRV_CANFDSPI_ReadByteArray(spi,a+8,&rx_buf,txdNumBytes);
    DRV_CANFDSPI_ReadByteArray(spi,a,&rx_buf,txdNumBytes+8+n+1);
    // printf("test:");
    // for (uint8_t i=0;i<txdNumBytes+8+n;i++) {
    //     printf("%u ", rx_buf[i]);
    // }
    // printf("\n");

    // Set UINC and TXREQ
    spiTransferError = DRV_CANFDSPI_TransmitChannelUpdate(spi, channel, flush);
    if (spiTransferError) {
        return -5;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelFlush(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    uint8_t d = 0;
    uint16_t a = 0;
    int8_t spiTransferError = 0;

    // Address of TXREQ
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    a += 1;

    // Set TXREQ
    d = 0x02;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, d);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelStatusGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_STATUS* status)
{
    uint16_t a = 0;
    uint32_t sta = 0;
    uint32_t fifoReg[2];
    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;

    // Get FIFO registers
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadWordArray(spi, a, fifoReg, 2);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    ciFifoCon.word = fifoReg[0];
    ciFifoSta.word = fifoReg[1];

    // Update status
    sta = ciFifoSta.byte[0];

    if (ciFifoCon.txBF.TxRequest) {
        sta |= CAN_TX_FIFO_TRANSMITTING;
    }

    *status = (CAN_TX_FIFO_STATUS) (sta & CAN_TX_FIFO_STATUS_MASK);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelReset(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    return DRV_CANFDSPI_ReceiveChannelReset(spi, channel);
}

int8_t DRV_CANFDSPI_TransmitChannelUpdate(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, bool flush)
{
    uint16_t a;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;

    // Set UINC
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1; // Byte that contains FRESET
    ciFifoCon.word = 0;
    ciFifoCon.txBF.UINC = 1;

    // Set TXREQ
    if (flush) {
        ciFifoCon.txBF.TxRequest = 1;
    }

    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[1]);
    if (spiTransferError) {
        return -1;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitRequestSet(spi_device_handle_t* spi,
        CAN_TXREQ_CHANNEL txreq)
{
    int8_t spiTransferError = 0;

    // Write TXREQ register
    uint32_t w = txreq;

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTXREQ, w);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitRequestGet(spi_device_handle_t* spi,
        uint32_t* txreq)
{
    int8_t spiTransferError = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiTXREQ, txreq);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelAbort(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    uint16_t a;
    uint8_t d;
    int8_t spiTransferError = 0;

    // Address
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    a += 1; // byte address of TXREQ

    // Clear TXREQ
    d = 0x00;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, d);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitAbortAll(spi_device_handle_t* spi)
{
    uint8_t d;
    int8_t spiTransferError = 0;

    // Read CiCON byte 3
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, (cREGADDR_CiCON + 3), &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d |= 0x8;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, (cREGADDR_CiCON + 3), d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitBandWidthSharingSet(spi_device_handle_t* spi,
        CAN_TX_BANDWITH_SHARING txbws)
{
    uint8_t d = 0;
    int8_t spiTransferError = 0;

    // Read CiCON byte 3
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, (cREGADDR_CiCON + 3), &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= 0x0f;
    d |= (txbws << 4);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, (cREGADDR_CiCON + 3), d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: CAN Receive

int8_t DRV_CANFDSPI_FilterObjectConfigure(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_FILTEROBJ_ID* id)
{
    uint16_t a;
    REG_CiFLTOBJ fObj;
    int8_t spiTransferError = 0;

    // Setup
    fObj.word = 0;
    fObj.bF = *id;
    a = cREGADDR_CiFLTOBJ + (filter * CiFILTER_OFFSET);

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, a, fObj.word);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_FilterMaskConfigure(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_MASKOBJ_ID* mask)
{
    uint16_t a;
    REG_CiMASK mObj;
    int8_t spiTransferError = 0;

    // Setup
    mObj.word = 0;
    mObj.bF = *mask;
    a = cREGADDR_CiMASK + (filter * CiFILTER_OFFSET);

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, a, mObj.word);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_FilterToFifoLink(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_FIFO_CHANNEL channel, bool enable)
{
    uint16_t a;
    REG_CiFLTCON_BYTE fCtrl;
    int8_t spiTransferError = 0;

    // Enable
    if (enable) {
        fCtrl.bF.Enable = 1;
    } else {
        fCtrl.bF.Enable = 0;
    }

    // Link
    fCtrl.bF.BufferPointer = channel;
    a = cREGADDR_CiFLTCON + filter;

    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, fCtrl.byte);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_FilterEnable(spi_device_handle_t* spi, CAN_FILTER filter)
{
    uint16_t a;
    REG_CiFLTCON_BYTE fCtrl;
    int8_t spiTransferError = 0;

    // Read
    a = cREGADDR_CiFLTCON + filter;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &fCtrl.byte);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    fCtrl.bF.Enable = 1;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, fCtrl.byte);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_FilterDisable(spi_device_handle_t* spi, CAN_FILTER filter)
{
    uint16_t a;
    REG_CiFLTCON_BYTE fCtrl;
    int8_t spiTransferError = 0;

    // Read
    a = cREGADDR_CiFLTCON + filter;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &fCtrl.byte);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    fCtrl.bF.Enable = 0;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, fCtrl.byte);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_DeviceNetFilterCountSet(spi_device_handle_t* spi,
        CAN_DNET_FILTER_SIZE dnfc)
{
    uint8_t d = 0;
    int8_t spiTransferError = 0;

    // Read CiCON byte 0
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiCON, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= 0x1f;
    d |= dnfc;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_CiCON, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelConfigure(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG* config)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) {
        return -100;
    }

    // Setup FIFO
    REG_CiFIFOCON ciFifoCon, ciFifoConVal;
    ciFifoCon.word = canFifoResetValues[0];
    ciFifoConVal.word = canFifoResetValues[0];

    ciFifoCon.rxBF.TxEnable = 0;
    ciFifoCon.rxBF.FifoSize = config->FifoSize;
    ciFifoCon.rxBF.PayLoadSize = config->PayLoadSize;
    ciFifoCon.rxBF.RxTimeStampEnable = config->RxTimeStampEnable;

    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    //printf("%X\n",ciFifoCon.word);
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, a, ciFifoCon.word);
    
    sleep(1);
    spiTransferError |= DRV_CANFDSPI_ReadWord(spi, a, &ciFifoConVal.word);
    //printf("%X\n",ciFifoConVal.word);

    spiTransferError |= memcmp(&ciFifoCon,&ciFifoConVal,sizeof(REG_CiFIFOCON));

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(CAN_RX_FIFO_CONFIG* config)
{
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = canFifoResetValues[0];

    config->FifoSize = ciFifoCon.rxBF.FifoSize;
    config->PayLoadSize = ciFifoCon.rxBF.PayLoadSize;
    config->RxTimeStampEnable = ciFifoCon.rxBF.RxTimeStampEnable;

    return 0;
}

int8_t DRV_CANFDSPI_ReceiveChannelStatusGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_STATUS* status)
{
    uint16_t a;
    REG_CiFIFOSTA ciFifoSta;
    int8_t spiTransferError = 0;

    // Read
    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *status = (CAN_RX_FIFO_STATUS) (ciFifoSta.byte[0] & 0x0F);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveMessageGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_MSGOBJ* rxObj,
        uint8_t* rxd, uint8_t nBytes)
{
    uint8_t n = 0;
    uint8_t i = 0;
    uint16_t a;
    uint32_t fifoReg[3];
    REG_CiFIFOCON ciFifoCon;
    REG_CiFIFOSTA ciFifoSta;
    REG_CiFIFOUA ciFifoUa;
    int8_t spiTransferError = 0;

    // Get FIFO registers
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadWordArray(spi, a, fifoReg, 3);
    if (spiTransferError) {
        return -1;
    }

    // Check that it is a receive buffer
    ciFifoCon.word = fifoReg[0];
    if (ciFifoCon.txBF.TxEnable) {
        return -2;
    }

    // Get Status
    ciFifoSta.word = fifoReg[1];

    // Get address
    ciFifoUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    a = 4 * ciFifoUa.bF.UserAddress;
#else
    a = ciFifoUa.bF.UserAddress;
#endif
    a += cRAMADDR_START;

    // Number of bytes to read
    n = nBytes + 8; // Add 8 header bytes
    // n = nBytes;

    if (ciFifoCon.rxBF.RxTimeStampEnable) {
        n += 4; // Add 4 time stamp bytes
    }

    // Make sure we read a multiple of 4 bytes from RAM
    if (n % 4) {
        n = n + 4 - (n % 4);
    }

    // Read rxObj using one access
    // uint8_t ba[MAX_MSG_SIZE]={0};
    uint8_t* ba = malloc(sizeof(uint8_t)*MAX_MSG_SIZE);

    if (n > MAX_MSG_SIZE) {
        n = MAX_MSG_SIZE;
    }

    // printf("n:%d\n",n);

    spiTransferError = DRV_CANFDSPI_ReadByteArray(spi, a, ba, n);
    if (spiTransferError) {
        return -3;
    }
    // for (uint8_t i=0;i<n;i++) {
    //     printf("%u ",ba[i]);
    // }
    printf("\n");
    // Assign message header
    REG_t myReg;

    myReg.byte[0] = ba[0];
    myReg.byte[1] = ba[1];
    myReg.byte[2] = ba[2];
    myReg.byte[3] = ba[3];
    rxObj->word[0] = myReg.word;

    myReg.byte[0] = ba[4];
    myReg.byte[1] = ba[5];
    myReg.byte[2] = ba[6];
    myReg.byte[3] = ba[7];
    rxObj->word[1] = myReg.word;

    if (ciFifoCon.rxBF.RxTimeStampEnable) {
        myReg.byte[0] = ba[8];
        myReg.byte[1] = ba[9];
        myReg.byte[2] = ba[10];
        myReg.byte[3] = ba[11];
        rxObj->word[2] = myReg.word;

        // Assign message data
        uint8_t* data = ba + 12;
        memcpy(rxd,data,nBytes);
        // for (i = 0; i < nBytes; i++) {
        //     rxd[i] = *(uint8_t*) ba[i + 12];
        // }
    } else {
        rxObj->word[2] = 0;

        // Assign message data
        uint8_t* data = ba + 8;
        memcpy(rxd,data,nBytes);
        // for (i = 0; i < nBytes; i++) {
        //     rxd[i] = *(uint8_t*) ba[i + 8];
        // }
    }

    // UINC channel
    spiTransferError = DRV_CANFDSPI_ReceiveChannelUpdate(spi, channel);
    if (spiTransferError) {
        return -4;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelReset(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    uint16_t a = 0;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;

    // Address and data
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1; // Byte that contains FRESET
    ciFifoCon.word = 0;
    ciFifoCon.rxBF.FRESET = 1;

    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[1]);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelUpdate(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    uint16_t a = 0;
    REG_CiFIFOCON ciFifoCon;
    int8_t spiTransferError = 0;
    ciFifoCon.word = 0;

    // Set UINC
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET) + 1; // Byte that contains FRESET
    ciFifoCon.rxBF.UINC = 1;

    // Write byte
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[1]);

    return spiTransferError;
}

// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO

int8_t DRV_CANFDSPI_TefStatusGet(spi_device_handle_t* spi,
        CAN_TEF_FIFO_STATUS* status)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    a = cREGADDR_CiTEFSTA;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciTefSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *status = (CAN_TEF_FIFO_STATUS) (ciTefSta.byte[0] & CAN_TEF_FIFO_STATUS_MASK);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefMessageGet(spi_device_handle_t* spi,
        CAN_TEF_MSGOBJ* tefObj)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;
    uint32_t fifoReg[3];
    uint8_t n = 0;

    // Get FIFO registers
    a = cREGADDR_CiTEFCON;

    spiTransferError = DRV_CANFDSPI_ReadWordArray(spi, a, fifoReg, 3);
    if (spiTransferError) {
        return -1;
    }

    // Get control
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = fifoReg[0];

    // Get status
    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = fifoReg[1];

    // Get address
    REG_CiFIFOUA ciTefUa;
    ciTefUa.word = fifoReg[2];
#ifdef USERADDRESS_TIMES_FOUR
    a = 4 * ciTefUa.bF.UserAddress;
#else
    a = ciTefUa.bF.UserAddress;
#endif
    a += cRAMADDR_START;

    // Number of bytes to read
    n = 8; // 8 header bytes

    if (ciTefCon.bF.TimeStampEnable) {
        n += 4; // Add 4 time stamp bytes
    }

    // Read rxObj using one access
    uint8_t ba[12];

    spiTransferError = DRV_CANFDSPI_ReadByteArray(spi, a, ba, n);
    if (spiTransferError) {
        return -2;
    }

    // Assign message header
    REG_t myReg;

    myReg.byte[0] = ba[0];
    myReg.byte[1] = ba[1];
    myReg.byte[2] = ba[2];
    myReg.byte[3] = ba[3];
    tefObj->word[0] = myReg.word;

    myReg.byte[0] = ba[4];
    myReg.byte[1] = ba[5];
    myReg.byte[2] = ba[6];
    myReg.byte[3] = ba[7];
    tefObj->word[1] = myReg.word;

    if (ciTefCon.bF.TimeStampEnable) {
        myReg.byte[0] = ba[8];
        myReg.byte[1] = ba[9];
        myReg.byte[2] = ba[10];
        myReg.byte[3] = ba[11];
        tefObj->word[2] = myReg.word;
    } else {
        tefObj->word[2] = 0;
    }

    // Set UINC
    spiTransferError = DRV_CANFDSPI_TefUpdate(spi);
    if (spiTransferError) {
        return -3;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefReset(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Set FRESET
    a = cREGADDR_CiTEFCON + 1;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
    ciTefCon.bF.FRESET = 1;

    // Write byte
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciTefCon.byte[1]);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefUpdate(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Set UINC
    a = cREGADDR_CiTEFCON + 1;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;
    ciTefCon.bF.UINC = 1;

    // Write byte
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciTefCon.byte[1]);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefConfigure(spi_device_handle_t* spi, CAN_TEF_CONFIG* config)
{
    int8_t spiTransferError = 0;

    // Setup FIFO
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = canControlResetValues[cREGADDR_CiTEFCON / 4];

    ciTefCon.bF.FifoSize = config->FifoSize;
    ciTefCon.bF.TimeStampEnable = config->TimeStampEnable;

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTEFCON, ciTefCon.word);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefConfigureObjectReset(CAN_TEF_CONFIG* config)
{
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = canFifoResetValues[0];

    config->FifoSize = ciTefCon.bF.FifoSize;
    config->TimeStampEnable = ciTefCon.bF.TimeStampEnable;

    return 0;
}


// *****************************************************************************
// *****************************************************************************
// Section: Module Events

int8_t DRV_CANFDSPI_ModuleEventGet(spi_device_handle_t* spi,
        CAN_MODULE_EVENT* flags)
{
    int8_t spiTransferError = 0;

    // Read Interrupt flags
    REG_CiINTFLAG intFlags;
    intFlags.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadHalfWord(spi, cREGADDR_CiINTFLAG, &intFlags.word);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_MODULE_EVENT) (intFlags.word & CAN_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventEnable(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiINTENABLE;
    REG_CiINTENABLE intEnables;
    intEnables.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadHalfWord(spi, a, &intEnables.word);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    intEnables.word |= (flags & CAN_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteHalfWord(spi, a, intEnables.word);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventDisable(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiINTENABLE;
    REG_CiINTENABLE intEnables;
    intEnables.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadHalfWord(spi, a, &intEnables.word);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    intEnables.word &= ~(flags & CAN_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteHalfWord(spi, a, intEnables.word);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventClear(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt flags
    a = cREGADDR_CiINTFLAG;
    REG_CiINTFLAG intFlags;
    intFlags.word = 0;

    // Write 1 to all flags except the ones that we want to clear
    // Writing a 1 will not set the flag
    // Only writing a 0 will clear it
    // The flags are HS/C
    intFlags.word = CAN_ALL_EVENTS;
    intFlags.word &= ~flags;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteHalfWord(spi, a, intFlags.word);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventRxCodeGet(spi_device_handle_t* spi,
        CAN_RXCODE* rxCode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;
    uint8_t rxCodeByte = 0;

    // Read
    a = cREGADDR_CiVEC + 3;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &rxCodeByte);
    if (spiTransferError) {
        return -1;
    }

    // Decode data
    // 0x40 = "no interrupt" (CAN_FIFO_CIVEC_NOINTERRUPT)
    if ((rxCodeByte < CAN_RXCODE_TOTAL_CHANNELS) || (rxCodeByte == CAN_RXCODE_NO_INT)) {
        *rxCode = (CAN_RXCODE) rxCodeByte;
    } else {
        *rxCode = CAN_RXCODE_RESERVED; // shouldn't get here
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventTxCodeGet(spi_device_handle_t* spi,
        CAN_TXCODE* txCode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;
    uint8_t txCodeByte = 0;

    // Read
    a = cREGADDR_CiVEC + 2;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &txCodeByte);
    if (spiTransferError) {
        return -1;
    }

    // Decode data
    // 0x40 = "no interrupt" (CAN_FIFO_CIVEC_NOINTERRUPT)
    if ((txCodeByte < CAN_TXCODE_TOTAL_CHANNELS) || (txCodeByte == CAN_TXCODE_NO_INT)) {
        *txCode = (CAN_TXCODE) txCodeByte;
    } else {
        *txCode = CAN_TXCODE_RESERVED; // shouldn't get here
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventFilterHitGet(spi_device_handle_t* spi,
        CAN_FILTER* filterHit)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;
    uint8_t filterHitByte = 0;

    // Read
    a = cREGADDR_CiVEC + 1;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &filterHitByte);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *filterHit = (CAN_FILTER) filterHitByte;

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ModuleEventIcodeGet(spi_device_handle_t* spi,
        CAN_ICODE* icode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;
    uint8_t icodeByte = 0;

    // Read
    a = cREGADDR_CiVEC;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &icodeByte);
    if (spiTransferError) {
        return -1;
    }

    // Decode
    if ((icodeByte < CAN_ICODE_RESERVED) && ((icodeByte < CAN_ICODE_TOTAL_CHANNELS) || (icodeByte >= CAN_ICODE_NO_INT))) {
        *icode = (CAN_ICODE) icodeByte;
    } else {
        *icode = CAN_ICODE_RESERVED; // shouldn't get here
    }

    return spiTransferError;
}

// *****************************************************************************
// *****************************************************************************
// Section: Transmit FIFO Events

int8_t DRV_CANFDSPI_TransmitChannelEventGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt flags
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_TX_FIFO_EVENT) (ciFifoSta.byte[0] & CAN_TX_FIFO_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitEventGet(spi_device_handle_t* spi, uint32_t* txif)
{
    int8_t spiTransferError = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiTXIF, txif);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitEventAttemptGet(spi_device_handle_t* spi,
        uint32_t* txatif)
{
    int8_t spiTransferError = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiTXATIF, txatif);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* idx)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read index
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, a, &ciFifoSta.word);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *idx = ciFifoSta.txBF.FifoIndex;

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelEventEnable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoCon.byte[0] |= (flags & CAN_TX_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelEventDisable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoCon.byte[0] &= ~(flags & CAN_TX_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TransmitChannelEventAttemptClear(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoSta.byte[0] &= ~CAN_TX_FIFO_ATTEMPTS_EXHAUSTED_EVENT;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Receive FIFO Events

int8_t DRV_CANFDSPI_ReceiveChannelEventGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) return -100;

    // Read Interrupt flags
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_RX_FIFO_EVENT) (ciFifoSta.byte[0] & CAN_RX_FIFO_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveEventGet(spi_device_handle_t* spi, uint32_t* rxif)
{
    int8_t spiTransferError = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiRXIF, rxif);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveEventOverflowGet(spi_device_handle_t* spi,
        uint32_t* rxovif)
{
    int8_t spiTransferError = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiRXOVIF, rxovif);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* idx)
{
    return DRV_CANFDSPI_TransmitChannelIndexGet(spi, channel, idx);
}

int8_t DRV_CANFDSPI_ReceiveChannelEventEnable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) return -100;

    // Read Interrupt Enables
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoCon.byte[0] |= (flags & CAN_RX_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelEventDisable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) return -100;

    // Read Interrupt Enables
    a = cREGADDR_CiFIFOCON + (channel * CiFIFO_OFFSET);
    REG_CiFIFOCON ciFifoCon;
    ciFifoCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoCon.byte[0] &= ~(flags & CAN_RX_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ReceiveChannelEventOverflowClear(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    if (channel == CAN_TXQUEUE_CH0) return -100;

    // Read Interrupt Flags
    REG_CiFIFOSTA ciFifoSta;
    ciFifoSta.word = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciFifoSta.byte[0] &= ~(CAN_RX_FIFO_OVERFLOW_EVENT);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciFifoSta.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO Events

int8_t DRV_CANFDSPI_TefEventGet(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt flags
    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    a = cREGADDR_CiTEFSTA;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciTefSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_TEF_FIFO_EVENT) (ciTefSta.byte[0] & CAN_TEF_FIFO_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefEventEnable(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiTEFCON;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciTefCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciTefCon.byte[0] |= (flags & CAN_TEF_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciTefCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefEventDisable(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Enables
    a = cREGADDR_CiTEFCON;
    REG_CiTEFCON ciTefCon;
    ciTefCon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciTefCon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciTefCon.byte[0] &= ~(flags & CAN_TEF_FIFO_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciTefCon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TefEventOverflowClear(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt Flags
    REG_CiTEFSTA ciTefSta;
    ciTefSta.word = 0;
    a = cREGADDR_CiTEFSTA;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &ciTefSta.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    ciTefSta.byte[0] &= ~(CAN_TEF_FIFO_OVERFLOW_EVENT);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, ciTefSta.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Error Handling

int8_t DRV_CANFDSPI_ErrorCountTransmitGet(spi_device_handle_t* spi,
        uint8_t* tec)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Error count
    a = cREGADDR_CiTREC + 1;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, tec);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ErrorCountReceiveGet(spi_device_handle_t* spi,
        uint8_t* rec)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Error count
    a = cREGADDR_CiTREC;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, rec);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ErrorStateGet(spi_device_handle_t* spi,
        CAN_ERROR_STATE* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Error state
    a = cREGADDR_CiTREC + 2;
    uint8_t f = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &f);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_ERROR_STATE) (f & CAN_ERROR_ALL);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_ErrorCountStateGet(spi_device_handle_t* spi,
        uint8_t* tec, uint8_t* rec, CAN_ERROR_STATE* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Error
    a = cREGADDR_CiTREC;
    REG_CiTREC ciTrec;
    ciTrec.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadWord(spi, a, &ciTrec.word);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *tec = ciTrec.byte[1];
    *rec = ciTrec.byte[0];
    *flags = (CAN_ERROR_STATE) (ciTrec.byte[2] & CAN_ERROR_ALL);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BusDiagnosticsGet(spi_device_handle_t* spi,
        CAN_BUS_DIAGNOSTIC* bd)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read diagnostic registers all in one shot
    a = cREGADDR_CiBDIAG0;
    uint32_t w[2];

    spiTransferError = DRV_CANFDSPI_ReadWordArray(spi, a, w, 2);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    CAN_BUS_DIAGNOSTIC b;
    b.word[0] = w[0];
    b.word[1] = w[1] & 0x0000ffff;
    b.word[2] = (w[1] >> 16) & 0x0000ffff;
    *bd = b;

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BusDiagnosticsClear(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t a = 0;

    // Clear diagnostic registers all in one shot
    a = cREGADDR_CiBDIAG0;
    uint32_t w[2];
    w[0] = 0;
    w[1] = 0;

    spiTransferError = DRV_CANFDSPI_WriteWordArray(spi, a, w, 2);

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: ECC

int8_t DRV_CANFDSPI_EccEnable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_ECCCON, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d |= 0x01;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_ECCCON, d);
    if (spiTransferError) {
        return -2;
    }

    return 0;
}

int8_t DRV_CANFDSPI_EccDisable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_ECCCON, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= ~0x01;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_ECCCON, d);
    if (spiTransferError) {
        return -2;
    }

    return 0;
}

int8_t DRV_CANFDSPI_EccEventGet(spi_device_handle_t* spi,
        CAN_ECC_EVENT* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Interrupt flags
    uint8_t eccStatus = 0;
    a = cREGADDR_ECCSTA;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &eccStatus);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_ECC_EVENT) (eccStatus & CAN_ECC_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccParitySet(spi_device_handle_t* spi,
        uint8_t parity)
{
    int8_t spiTransferError = 0;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_ECCCON + 1, parity);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccParityGet(spi_device_handle_t* spi,
        uint8_t* parity)
{
    int8_t spiTransferError = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_ECCCON + 1, parity);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccErrorAddressGet(spi_device_handle_t* spi,
        uint16_t* a)
{
    int8_t spiTransferError = 0;
    REG_ECCSTA reg;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_ECCSTA, &reg.word);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *a = reg.bF.ErrorAddress;

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccEventEnable(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_ECCCON;
    uint8_t eccInterrupts = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &eccInterrupts);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    eccInterrupts |= (flags & CAN_ECC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, eccInterrupts);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccEventDisable(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_ECCCON;
    uint8_t eccInterrupts = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &eccInterrupts);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    eccInterrupts &= ~(flags & CAN_ECC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, eccInterrupts);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_EccEventClear(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_ECCSTA;
    uint8_t eccStat = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &eccStat);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    eccStat &= ~(flags & CAN_ECC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, eccStat);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: CRC

int8_t DRV_CANFDSPI_CrcEventEnable(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read interrupt control bits of CRC Register
    a = cREGADDR_CRC + 3;
    uint8_t crc;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &crc);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    crc |= (flags & CAN_CRC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, crc);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_CrcEventDisable(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read interrupt control bits of CRC Register
    a = cREGADDR_CRC + 3;
    uint8_t crc;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &crc);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    crc &= ~(flags & CAN_CRC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, crc);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_CrcEventClear(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read interrupt flags of CRC Register
    a = cREGADDR_CRC + 2;
    uint8_t crc;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &crc);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    crc &= ~(flags & CAN_CRC_ALL_EVENTS);

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, crc);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_CrcEventGet(spi_device_handle_t* spi, CAN_CRC_EVENT* flags)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read interrupt flags of CRC Register
    a = cREGADDR_CRC + 2;
    uint8_t crc;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &crc);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *flags = (CAN_CRC_EVENT) (crc & CAN_CRC_ALL_EVENTS);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_CrcValueGet(spi_device_handle_t* spi, uint16_t* crc)
{
    int8_t spiTransferError = 0;

    // Read CRC value from CRC Register
    spiTransferError = DRV_CANFDSPI_ReadHalfWord(spi, cREGADDR_CRC, crc);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_RamInit(spi_device_handle_t* spi, uint8_t d)
{
    uint8_t txd[SPI_DEFAULT_BUFFER_LENGTH];
    uint32_t k;
    int8_t spiTransferError = 0;

    // Prepare data
    for (k = 0; k < SPI_DEFAULT_BUFFER_LENGTH; k++) {
        txd[k] = d;
    }

    uint16_t a = cRAMADDR_START;

    for (k = 0; k < (cRAM_SIZE / SPI_DEFAULT_BUFFER_LENGTH); k++) {
        spiTransferError = DRV_CANFDSPI_WriteByteArray(spi, a, txd, SPI_DEFAULT_BUFFER_LENGTH);
        if (spiTransferError) {
            return -1;
        }
        a += SPI_DEFAULT_BUFFER_LENGTH;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Time Stamp

int8_t DRV_CANFDSPI_TimeStampEnable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiTSCON + 2, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d |= 0x01;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_CiTSCON + 2, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TimeStampDisable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiTSCON + 2, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= 0x06;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_CiTSCON + 2, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TimeStampGet(spi_device_handle_t* spi, uint32_t* ts)
{
    int8_t spiTransferError = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadWord(spi, cREGADDR_CiTBC, ts);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TimeStampSet(spi_device_handle_t* spi, uint32_t ts)
{
    int8_t spiTransferError = 0;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTBC, ts);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TimeStampModeConfigure(spi_device_handle_t* spi,
        CAN_TS_MODE mode)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_CiTSCON + 2, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= 0x01;
    d |= mode << 1;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_CiTSCON + 2, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_TimeStampPrescalerSet(spi_device_handle_t* spi,
        uint16_t ps)
{
    int8_t spiTransferError = 0;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteHalfWord(spi, cREGADDR_CiTSCON, ps);

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Oscillator and Bit Time

int8_t DRV_CANFDSPI_OscillatorEnable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint8_t d = 0;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_OSC, &d);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    d &= ~0x4;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_OSC, d);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_OscillatorControlSet(spi_device_handle_t* spi,
        CAN_OSC_CTRL ctrl)
{
    int8_t spiTransferError = 0;

    REG_OSC osc;
    osc.word = 0;

    osc.bF.PllEnable = ctrl.PllEnable;
    osc.bF.OscDisable = ctrl.OscDisable;
    osc.bF.SCLKDIV = ctrl.SclkDivide;
    osc.bF.CLKODIV = ctrl.ClkOutDivide;
#ifndef MCP2517FD
    osc.bF.LowPowerModeEnable = ctrl.LowPowerModeEnable;
#endif

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, cREGADDR_OSC, osc.byte[0]);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_OscillatorControlObjectReset(CAN_OSC_CTRL* ctrl)
{
    REG_OSC osc;
    osc.word = mcp25xxfdControlResetValues[0];

    ctrl->PllEnable = osc.bF.PllEnable;
    ctrl->OscDisable = osc.bF.OscDisable;
    ctrl->SclkDivide = osc.bF.SCLKDIV;
    ctrl->ClkOutDivide = osc.bF.CLKODIV;

    return 0;
}

int8_t DRV_CANFDSPI_OscillatorStatusGet(spi_device_handle_t* spi,
        CAN_OSC_STATUS* status)
{
    int8_t spiTransferError = 0;

    REG_OSC osc;
    osc.word = 0;
    CAN_OSC_STATUS stat;

    // Read
    spiTransferError = DRV_CANFDSPI_ReadByte(spi, cREGADDR_OSC + 1, &osc.byte[1]);
    if (spiTransferError) {
        return -1;
    }

    stat.PllReady = osc.bF.PllReady;
    stat.OscReady = osc.bF.OscReady;
    stat.SclkReady = osc.bF.SclkReady;

    *status = stat;

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigure(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode,
        CAN_SYSCLK_SPEED clk)
{
    int8_t spiTransferError = 0;

    // Decode clk
    switch (clk) {
        case CAN_SYSCLK_40M:
            spiTransferError = DRV_CANFDSPI_BitTimeConfigureNominal40MHz(spi, bitTime);
            if (spiTransferError) return spiTransferError;

            spiTransferError = DRV_CANFDSPI_BitTimeConfigureData40MHz(spi, bitTime, sspMode);
            break;
        case CAN_SYSCLK_20M:
            spiTransferError = DRV_CANFDSPI_BitTimeConfigureNominal20MHz(spi, bitTime);
            if (spiTransferError) return spiTransferError;

            spiTransferError = DRV_CANFDSPI_BitTimeConfigureData20MHz(spi, bitTime, sspMode);
            break;
        case CAN_SYSCLK_10M:
            spiTransferError = DRV_CANFDSPI_BitTimeConfigureNominal10MHz(spi, bitTime);
            if (spiTransferError) return spiTransferError;

            spiTransferError = DRV_CANFDSPI_BitTimeConfigureData10MHz(spi, bitTime, sspMode);
            break;
        default:
            spiTransferError = -1;
            break;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureNominal40MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime)
{
    int8_t spiTransferError = 0;
    REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    // Arbitration Bit rate
    switch (bitTime) {
            // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_3M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 62;
            ciNbtcfg.bF.TSEG2 = 15;
            ciNbtcfg.bF.SJW = 15;
            break;

            // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 126;
            ciNbtcfg.bF.TSEG2 = 31;
            ciNbtcfg.bF.SJW = 31;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 30;
            ciNbtcfg.bF.TSEG2 = 7;
            ciNbtcfg.bF.SJW = 7;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 254;
            ciNbtcfg.bF.TSEG2 = 63;
            ciNbtcfg.bF.SJW = 63;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiNBTCFG, ciNbtcfg.word);

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureData40MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode)
{
    int8_t spiTransferError = 0;
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
    //    sspMode;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;

    // Configure Bit time and sample point
    ciTdc.bF.TDCMode = CAN_SSP_MODE_AUTO;
    uint32_t tdcValue = 0;

    // Data Bit rate and SSP
    switch (bitTime) {
        case CAN_500K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_3M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 4;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 5;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 3;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 4;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_8M:
        case CAN_1000K_8M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = 1;
            break;
        case CAN_500K_10M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 1;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 2;
            ciTdc.bF.TDCValue = 0;
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bF.BRP = 1;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bF.BRP = 1;
            ciDbtcfg.bF.TSEG1 = 17;
            ciDbtcfg.bF.TSEG2 = 4;
            ciDbtcfg.bF.SJW = 4;
            // SSP
            ciTdc.bF.TDCOffset = 18;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 18;
            ciDbtcfg.bF.TSEG2 = 5;
            ciDbtcfg.bF.SJW = 5;
            // SSP
            ciTdc.bF.TDCOffset = 19;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiDBTCFG, ciDbtcfg.word);
    if (spiTransferError) {
        return -2;
    }

    // Write Transmitter Delay Compensation
#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTDC, ciTdc.word);
    if (spiTransferError) {
        return -3;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureNominal20MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime)
{
    int8_t spiTransferError = 0;
    REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    // Arbitration Bit rate
    switch (bitTime) {
            // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 30;
            ciNbtcfg.bF.TSEG2 = 7;
            ciNbtcfg.bF.SJW = 7;
            break;

            // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 62;
            ciNbtcfg.bF.TSEG2 = 15;
            ciNbtcfg.bF.SJW = 15;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 14;
            ciNbtcfg.bF.TSEG2 = 3;
            ciNbtcfg.bF.SJW = 3;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 126;
            ciNbtcfg.bF.TSEG2 = 31;
            ciNbtcfg.bF.SJW = 31;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiNBTCFG, ciNbtcfg.word);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureData20MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode)
{
    int8_t spiTransferError = 0;
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
    //    sspMode;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;

    // Configure Bit time and sample point
    ciTdc.bF.TDCMode = CAN_SSP_MODE_AUTO;
    uint32_t tdcValue = 0;

    // Data Bit rate and SSP
    switch (bitTime) {
        case CAN_500K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_1000K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_5M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 1;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 2;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
        case CAN_1000K_8M:
            //qDebug("Data Bitrate not feasible with this clock!");
            return -1;
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 30;
            ciDbtcfg.bF.TSEG2 = 7;
            ciDbtcfg.bF.SJW = 7;
            // SSP
            ciTdc.bF.TDCOffset = 31;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 17;
            ciDbtcfg.bF.TSEG2 = 4;
            ciDbtcfg.bF.SJW = 4;
            // SSP
            ciTdc.bF.TDCOffset = 18;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 8;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 9;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
            //qDebug("Data Bitrate not feasible with this clock!");
            return -1;
            break;
        case CAN_250K_4M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiDBTCFG, ciDbtcfg.word);
    if (spiTransferError) {
        return -2;
    }

    // Write Transmitter Delay Compensation
#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTDC, ciTdc.word);
    if (spiTransferError) {
        return -3;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureNominal10MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime)
{
    int8_t spiTransferError = 0;
    REG_CiNBTCFG ciNbtcfg;

    ciNbtcfg.word = canControlResetValues[cREGADDR_CiNBTCFG / 4];

    // Arbitration Bit rate
    switch (bitTime) {
            // All 500K
        case CAN_500K_1M:
        case CAN_500K_2M:
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 14;
            ciNbtcfg.bF.TSEG2 = 3;
            ciNbtcfg.bF.SJW = 3;
            break;

            // All 250K
        case CAN_250K_500K:
        case CAN_250K_833K:
        case CAN_250K_1M:
        case CAN_250K_1M5:
        case CAN_250K_2M:
        case CAN_250K_3M:
        case CAN_250K_4M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 30;
            ciNbtcfg.bF.TSEG2 = 7;
            ciNbtcfg.bF.SJW = 7;
            break;

        case CAN_1000K_4M:
        case CAN_1000K_8M:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 7;
            ciNbtcfg.bF.TSEG2 = 2;
            ciNbtcfg.bF.SJW = 2;
            break;

        case CAN_125K_500K:
            ciNbtcfg.bF.BRP = 0;
            ciNbtcfg.bF.TSEG1 = 62;
            ciNbtcfg.bF.TSEG2 = 15;
            ciNbtcfg.bF.SJW = 15;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiNBTCFG, ciNbtcfg.word);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_BitTimeConfigureData10MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode)
{
    int8_t spiTransferError = 0;
    REG_CiDBTCFG ciDbtcfg;
    REG_CiTDC ciTdc;
    //    sspMode;

    ciDbtcfg.word = canControlResetValues[cREGADDR_CiDBTCFG / 4];
    ciTdc.word = 0;

    // Configure Bit time and sample point
    ciTdc.bF.TDCMode = CAN_SSP_MODE_AUTO;
    uint32_t tdcValue = 0;

    // Data Bit rate and SSP
    switch (bitTime) {
        case CAN_500K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_2M:
            // Data BR
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_500K_4M:
        case CAN_500K_5M:
        case CAN_500K_6M7:
        case CAN_500K_8M:
        case CAN_500K_10M:
        case CAN_1000K_4M:
        case CAN_1000K_8M:
            //qDebug("Data Bitrate not feasible with this clock!");
            return -1;
            break;

        case CAN_250K_500K:
        case CAN_125K_500K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 14;
            ciDbtcfg.bF.TSEG2 = 3;
            ciDbtcfg.bF.SJW = 3;
            // SSP
            ciTdc.bF.TDCOffset = 15;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_833K:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 7;
            ciDbtcfg.bF.TSEG2 = 2;
            ciDbtcfg.bF.SJW = 2;
            // SSP
            ciTdc.bF.TDCOffset = 8;
            ciTdc.bF.TDCValue = tdcValue;
            ciTdc.bF.TDCMode = CAN_SSP_MODE_OFF;
            break;
        case CAN_250K_1M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 6;
            ciDbtcfg.bF.TSEG2 = 1;
            ciDbtcfg.bF.SJW = 1;
            // SSP
            ciTdc.bF.TDCOffset = 7;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_1M5:
            //qDebug("Data Bitrate not feasible with this clock!");
            return -1;
            break;
        case CAN_250K_2M:
            ciDbtcfg.bF.BRP = 0;
            ciDbtcfg.bF.TSEG1 = 2;
            ciDbtcfg.bF.TSEG2 = 0;
            ciDbtcfg.bF.SJW = 0;
            // SSP
            ciTdc.bF.TDCOffset = 3;
            ciTdc.bF.TDCValue = tdcValue;
            break;
        case CAN_250K_3M:
        case CAN_250K_4M:
            //qDebug("Data Bitrate not feasible with this clock!");
            return -1;
            break;

        default:
            return -1;
            break;
    }

    // Write Bit time registers
    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiDBTCFG, ciDbtcfg.word);
    if (spiTransferError) {
        return -2;
    }

    // Write Transmitter Delay Compensation
#ifdef REV_A
    ciTdc.bF.TDCOffset = 0;
    ciTdc.bF.TDCValue = 0;
#endif

    spiTransferError = DRV_CANFDSPI_WriteWord(spi, cREGADDR_CiTDC, ciTdc.word);
    if (spiTransferError) {
        return -3;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: GPIO

int8_t DRV_CANFDSPI_GpioModeConfigure(spi_device_handle_t* spi,
        GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 2;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[2]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.PinMode0 = gpio0;
    iocon.bF.PinMode1 = gpio1;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[2]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioDirectionConfigure(spi_device_handle_t* spi,
        GPIO_PIN_DIRECTION gpio0, GPIO_PIN_DIRECTION gpio1)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.TRIS0 = gpio0;
    iocon.bF.TRIS1 = gpio1;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioStandbyControlEnable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.XcrSTBYEnable = 1;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioStandbyControlDisable(spi_device_handle_t* spi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[0]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.XcrSTBYEnable = 0;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[0]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(spi_device_handle_t* spi,
        GPIO_OPEN_DRAIN_MODE mode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[3]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.INTPinOpenDrain = mode;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[3]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioTransmitPinOpenDrainConfigure(spi_device_handle_t* spi,
        GPIO_OPEN_DRAIN_MODE mode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[3]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.TXCANOpenDrain = mode;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[3]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioPinSet(spi_device_handle_t* spi,
        GPIO_PIN_POS pos, GPIO_PIN_STATE latch)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 1;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[1]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    switch (pos) {
        case GPIO_PIN_0:
            iocon.bF.LAT0 = latch;
            break;
        case GPIO_PIN_1:
            iocon.bF.LAT1 = latch;
            break;
        default:
            return -1;
            break;
    }

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[1]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioPinRead(spi_device_handle_t* spi,
        GPIO_PIN_POS pos, GPIO_PIN_STATE* state)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 2;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[2]);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    switch (pos) {
        case GPIO_PIN_0:
            *state = (GPIO_PIN_STATE) iocon.bF.GPIO0;
            break;
        case GPIO_PIN_1:
            *state = (GPIO_PIN_STATE) iocon.bF.GPIO1;
            break;
        default:
            return -1;
            break;
    }

    return spiTransferError;
}

int8_t DRV_CANFDSPI_GpioClockOutputConfigure(spi_device_handle_t* spi,
        GPIO_CLKO_MODE mode)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read
    a = cREGADDR_IOCON + 3;
    REG_IOCON iocon;
    iocon.word = 0;

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &iocon.byte[3]);
    if (spiTransferError) {
        return -1;
    }

    // Modify
    iocon.bF.SOFOutputEnable = mode;

    // Write
    spiTransferError = DRV_CANFDSPI_WriteByte(spi, a, iocon.byte[3]);
    if (spiTransferError) {
        return -2;
    }

    return spiTransferError;
}


// *****************************************************************************
// *****************************************************************************
// Section: Miscellaneous

uint32_t DRV_CANFDSPI_DlcToDataBytes(CAN_DLC dlc)
{
    uint32_t dataBytesInObject = 0;

    Nop();
    Nop();

    if (dlc < CAN_DLC_12) {
        dataBytesInObject = dlc;
    } else {
        switch (dlc) {
            case CAN_DLC_12:
                dataBytesInObject = 12;
                break;
            case CAN_DLC_16:
                dataBytesInObject = 16;
                break;
            case CAN_DLC_20:
                dataBytesInObject = 20;
                break;
            case CAN_DLC_24:
                dataBytesInObject = 24;
                break;
            case CAN_DLC_32:
                dataBytesInObject = 32;
                break;
            case CAN_DLC_48:
                dataBytesInObject = 48;
                break;
            case CAN_DLC_64:
                dataBytesInObject = 64;
                break;
            default:
                break;
        }
    }

    return dataBytesInObject;
}

int8_t DRV_CANFDSPI_FifoIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* mi)
{
    int8_t spiTransferError = 0;
    uint16_t a = 0;

    // Read Status register
    uint8_t b = 0;
    a = cREGADDR_CiFIFOSTA + (channel * CiFIFO_OFFSET);
    a += 1; // byte[1]

    spiTransferError = DRV_CANFDSPI_ReadByte(spi, a, &b);
    if (spiTransferError) {
        return -1;
    }

    // Update data
    *mi = b & 0x1f;

    return spiTransferError;
}

uint16_t DRV_CANFDSPI_CalculateCRC16(uint8_t* data, uint16_t size)
{
    uint16_t init = CRCBASE;
    uint8_t index;

    while (size-- != 0) {
        index = ((uint8_t*) & init)[CRCUPPER] ^ *data++;
        init = (init << 8) ^ crc16_table[index];
    }

    return init;
}

CAN_DLC DRV_CANFDSPI_DataBytesToDlc(uint8_t n)
{
    CAN_DLC dlc = CAN_DLC_0;

    if (n <= 4) {
        dlc = CAN_DLC_4;
    } else if (n <= 8) {
        dlc = CAN_DLC_8;
    } else if (n <= 12) {
        dlc = CAN_DLC_12;
    } else if (n <= 16) {
        dlc = CAN_DLC_16;
    } else if (n <= 20) {
        dlc = CAN_DLC_20;
    } else if (n <= 24) {
        dlc = CAN_DLC_24;
    } else if (n <= 32) {
        dlc = CAN_DLC_32;
    } else if (n <= 48) {
        dlc = CAN_DLC_48;
    } else if (n <= 64) {
        dlc = CAN_DLC_64;
    }

    return dlc;
}