/*******************************************************************************
  SMC Messages: API Header File

  Company:
    Dandelions

  File Name:
    smc.c

  Summary:
    API implementation.

  Description:
    .
 *******************************************************************************/

#ifndef MSG_H
#define MSG_H

#include "canfd/drv_can.h"
#include <stdlib.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "DandSMC.h"

#define MD5_DIGEST_LENGTH 16

// Broadcast commands
// Timestamp (or 0x400)
#define X_DTM 0x001
// SpaceCraft State (or 0x200)
#define X_STATE 0x002
// Power Down (or 0x600)
#define X_ALL_PDOWN 0x003

// Payload Specific Commands
// Begin Operation (or 0x428)
#define X_BEG_OP 0x0A1
// Cease Operation (or 0x228)
#define X_STOP 0x0A2
// Power Down (or 0x628)
#define X_PDOWN 0x0A3
// Query for data to transmit (or 0x128)
#define X_QDATA 0x0A4 
// Transmit data command over ISO-TP (or 0x528)
#define X_TX_DATA 0x0A5
// Transmit data flow control (or 0x328)
#define X_ISOTP 0x0A6
// Data transmission result (or 0x728)
#define X_TX_ACK 0x0A7

// Standard IDs for Sending
// Ready to operate (or 0x40A)
#define P_RDY_OP 0x501
// Beginning Operation (or 0x20A)
#define P_BEG_OP 0x502
// Ceased Operation (or 0x60A)
#define P_STOP 0x503
// Ready for power down (or 0x10A)
#define P_PDOWN 0x504
// Query response for data to transmit (or 0x50A)
#define P_QDATA_RSP 0x505
// Transmit data over ISO-TP (or 0x30A)
#define P_ISOTP 0x506

/*typedef struct {
    double Latitude;
    double Longitude;
    double Altitude;
    float Roll;
    float Pitch;
    float Heading;
    float Velocity_N;
    float Velocity_E;
    float Velocity_D;
    float Angular_VX;
    float Angular_VY;
    float Angular_VZ;
} Spacecraft_State;
*/
typedef struct {
    uint16_t md5;
    char* name;
} Iso_Tp_File;

void SMC_FILTER_CONFIG(spi_device_handle_t* spi);

void SMC_MESSAGE_HANDLER(spi_device_handle_t* spi, enum flowFlag *flowFlagPtr);

//void get_timestamp(uint8_t* rxd);

//void get_spacecraft_state(uint8_t* rxd);

void powerdown();

void begin(uint8_t* rxd);

void stop();

void handle_tx(spi_device_handle_t* spi);

void handle_rx(spi_device_handle_t* spi, uint8_t* rxd);

#endif  