/*******************************************************************************
  SMC Messages: Implementation

  Company:
    Dandelions

  File Name:
    smc.c

  Summary:
    API implementation.

  Description:
    .
 *******************************************************************************/

#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <md5/global.h>
#include <md5/md5.h>
#include "msg.h"
#include <canfdspi/drv_canfdspi_api.h>
#include <canfdspi/drv_canfdspi_defines.h>
#include <canfdspi/drv_canfdspi_register.h>
#include <spi/drv_spi.h>
#include <isotp/iso_tp.h>

void SMC_FILTER_CONFIG(spi_device_handle_t* spi) {
    
    //Configure Filter Object 0
    // Timestamp message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER0);
    CAN_FILTEROBJ_ID fObj;
    fObj.SID = TIMESTAMP;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER0, &fObj);

    // SpaceCraft State message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER1);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = SPACECRAFT_STATE;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER1, &fObj);

    // Power Down message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER2);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = POWDWN_ALL;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER2, &fObj);

    // Begin Operation message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER3);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = BEGIN;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER3, &fObj);
    
    // Cease Operation message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER4);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = CEASE;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER4, &fObj);
    
    // Power Down message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER5);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = POWDWN;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER5, &fObj);
    
    // Query for data to transmit message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER6);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = QUERY;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER6, &fObj);
    
    // Transmit data command over ISO-TP message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER7);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = TRANSMIT_CMD;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER7, &fObj);
    
    // Transmit data flow control message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER8);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = TRANSMIT_FLOW;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER8, &fObj);
    
    // Data transmission result message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER9);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = TRANSMIT_RESULT;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER9, &fObj);

    // Ready to operate message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER10);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = READY;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER10, &fObj);
    
    // // Beginning Operation message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER11);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = BEGINNING;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER11, &fObj);
    
    // // Ceased Operation message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER12);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = CEASED;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER12, &fObj);
    
    // // Ready for power down message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER13);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = READY_FOR_POWDWN;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER13, &fObj);
    
    // // Query response for data to transmit message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER14);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = RESPONSE;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER14, &fObj);
    
    // // Transmit data over ISO-TP message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER15);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = TRANSMIT;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER15, &fObj);

    // Link Filter to RX FIFO and enable Filter
    bool filterEnable = true;
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER0, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER1, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER2, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER3, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER4, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER5, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER6, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER7, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER8, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER9, CAN_FIFO_CH2, filterEnable);
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER10, CAN_FIFO_CH2, filterEnable);
    // TX Messages not RX, so no Filters needed
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER11, CAN_FIFO_CH2, filterEnable);
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER12, CAN_FIFO_CH2, filterEnable);
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER13, CAN_FIFO_CH2, filterEnable);
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER14, CAN_FIFO_CH2, filterEnable);
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER15, CAN_FIFO_CH2, filterEnable);
}

// Message Handler
void SMC_MESSAGE_HANDLER(spi_device_handle_t* spi) {
    // Receive Message Object
    CAN_RX_MSGOBJ rxObj;
    uint8_t rxd[MAX_DATA_BYTES] = {0};

    if (!DRV_CAN_READ_OBJ(spi,rxd,&rxObj)){
        switch (rxObj.bF.id.SID) {
            case TIMESTAMP: get_timestamp(&rxd);
            case SPACECRAFT_STATE: get_spacecraft_state(&rxd);
            case POWDWN_ALL | POWDWN: powerdown();
            case BEGIN: begin(&rxd);
            case CEASE: stop();
            case QUERY: handle_tx(spi);
            case RECEIVE_CMD: handle_rx(spi,rxd);
            // These cases will only occur during data transmission -> handled in handle_quer();
            // case TRANSMIT_CMD: break;
            // case TRANSMIT_FLOW: break;
            // case TRANSMIT_RESULT: break;
            default: break;
        }
    }        
}

// void get_timestamp(uint8_t* rxd) {
//     // this can also be a pointer to the timestamp saved somewhere in the main run file?
//     uint64_t timestamp = 0;

//     memcpy(timestamp, rxd, sizeof(uint64_t));

//     // invoke interrupt to save new timestamp?
// }

// Alternative
void get_timestamp(uint8_t* rxd) {
    struct timeval tv;
    
    uint64_t usecs;
    memcpy(&usecs,rxd,sizeof(uint64_t));

    tv.tv_sec = usecs/1000;
    tv.tv_usec = (usecs%1000)*1000;

    settimeofday(&tv, NULL);
    printf("Time set to %ld.%06ld", tv.tv_sec, tv.tv_usec);
}

// Same as above, probably not needed
void get_spacecraft_state(uint8_t* rxd) {
    Spacecraft_State state = {0};

    memcpy(&state, rxd, sizeof(Spacecraft_State));
}

void powerdown() {
    // Invoke interrupt to start powerdown of spacecraft
}

void begin(uint8_t* rxd) {
    // Invoke interrupt to start operation
}

void stop() {
    // Invoke interrupt to cease operation
}

void handle_tx(spi_device_handle_t* spi) {
    // Check if data is available for transmit and get name of file
    uint8_t data = 0;
    uint32_t data_len = sizeof(data);

    char *name = "Test";
    uint8_t name_len = strlen(name) + 1; // add 1 for null terminator
    uint8_t reply_len = MD5_DIGEST_LENGTH + name_len; 
    
    // Reply with MD5 hash and UTF-8 filename of data
    // Creat MD5 hash digest
    uint8_t digest[MD5_DIGEST_LENGTH];
    MD5(digest, &data, sizeof(data));

    uint8_t* reply = (uint8_t*) malloc(sizeof(data)); // allocate memory for data

    memcpy(reply,digest,MD5_DIGEST_LENGTH);

    memcpy(reply+MD5_DIGEST_LENGTH,name,name_len);

    DRV_CAN_WRITE(spi,reply,RESPONSE,CAN_DLC_64);

    bool ack = false;
    bool recv = false;
    // Implement a wait loop for ack
    while (ack == false) {
        CAN_RX_MSGOBJ msgObj;
        uint8_t rxd[MAX_DATA_BYTES] = {0};
        DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);

        if (msgObj.bF.id.SID == TRANSMIT_CMD) {
            ack = true;
        }
    }

    if (iso_tp_send(spi,data,data_len)) {
        while (recv == false) {
            CAN_RX_MSGOBJ msgObj;
            uint8_t rxd[MAX_DATA_BYTES] = {0};
            DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);

            if (msgObj.bF.id.SID == TRANSMIT_RESULT) {
                // It doesn't matter here if it fails or succeeds, we don't have any way to initiate a retry
                recv = true;
            }
        }
    }          
}

void handle_rx(spi_device_handle_t* spi, uint8_t* read) {
    Iso_Tp_File file;
    file.md5 = ((uint16_t) read[0]<<8) + (uint16_t) read[1];
    file.name = (char *) (read+2);

    uint8_t txd = NULL;
    
    DRV_CAN_WRITE(spi, &txd, RX_READY, CAN_DLC_64);

    iso_tp_receive(spi);

    /* save rxd to a file*/
}