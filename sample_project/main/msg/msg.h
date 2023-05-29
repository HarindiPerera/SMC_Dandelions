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
#define TIMESTAMP 0x001
// SpaceCraft State (or 0x200)
#define SPACECRAFT_STATE 0x002
// Power Down (or 0x600)
#define POWDWN_ALL 0x003

// Payload Specific Commands
// Begin Operation (or 0x428)
#define BEGIN 0x0A1
// Cease Operation (or 0x228)
#define CEASE 0x0A2
// Power Down (or 0x628)
#define POWDWN 0x0A3
// Query for data to transmit (or 0x128)
#define QUERY 0x0A4 
// Transmit data command over ISO-TP (or 0x528)
#define TRANSMIT_CMD 0x0A5
// Transmit data flow control (or 0x328)
#define TRANSMIT_FLOW 0x0A6
// Data transmission result (or 0x728)
#define TRANSMIT_RESULT 0x0A7
// Prepare to Receive File
#define RECEIVE_CMD 0x0A8
// Receive Frames over ISO-TP
#define RECEIVE_FRAME 0x0A9


// Standard IDs for Sending
// Ready to operate (or 0x40A)
#define READY 0x501
// Beginning Operation (or 0x20A)
#define BEGINNING 0x502
// Ceased Operation (or 0x60A)
#define CEASED 0x503
// Ready for power down (or 0x10A)
#define READY_FOR_POWDWN 0x504
// Query response for data to transmit (or 0x50A)
#define RESPONSE 0x505
// Transmit data over ISO-TP (or 0x30A)
#define TRANSMIT 0x506
// Ready to Receive File
#define RX_READY 0x507
// Receive Flow Control frames
#define RX_FLOW 0x508
// Receive Transmission Result
#define RX_RESULT 0x509

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