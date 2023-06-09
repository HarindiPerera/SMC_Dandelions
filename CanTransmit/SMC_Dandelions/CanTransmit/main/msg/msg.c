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
#include <stdint.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <md5/global.h>
#include <md5/md5.h>
#include "msg.h"
#include <canfdspi/drv_canfdspi_api.h>
#include <canfdspi/drv_canfdspi_defines.h>
#include <canfdspi/drv_canfdspi_register.h>
//#include <spi/drv_spi.h>
#include <isotp/iso_tp.h>
#include "DandSMC.h"

/* Task handles*/
TaskHandle_t TxHandle = NULL;

typedef struct {
    spi_device_handle_t* spi;
} TxParams_t;

/* Task functions*/
void Tx(void *pvParameters){
    spi_device_handle_t spi = ((TxParams_t*)pvParameters)->spi;
         
    handle_tx(spi);        
}
void SMC_FILTER_CONFIG(spi_device_handle_t* spi) {
    
    //Configure Filter Object 0
    // Timestamp message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER0);
    CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_DTM;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER0, &fObj);

    // SpaceCraft State message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER1);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_STATE;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER1, &fObj);

    // Power Down message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER2);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_ALL_PDOWN;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER2, &fObj);

    // Begin Operation message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER3);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_BEG_OP;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER3, &fObj);
    
    // Cease Operation message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER4);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_STOP;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER4, &fObj);
    
    // Power Down message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER5);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_PDOWN;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER5, &fObj);
    
    // Query for data to transmit message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER6);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_QDATA;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER6, &fObj);
    
    // Transmit data command over ISO-TP message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER7);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_TX_DATA;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER7, &fObj);
    
    // Transmit data flow control message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER8);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_ISOTP;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER8, &fObj);
    
    // Data transmission result message
    DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER9);
    // CAN_FILTEROBJ_ID fObj;
    fObj.SID = X_TX_ACK;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect standard Frames
    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER9, &fObj);

    // Ready to operate message
    // DRV_CANFDSPI_FilterDisable(spi, CAN_FILTER10);
    // // CAN_FILTEROBJ_ID fObj;
    // fObj.SID = READY;
    // fObj.SID11 = 0;
    // fObj.EID = 0;
    // fObj.EXIDE = 0; // only expect standard Frames
    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER10, &fObj);
    
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

void SMC_MESSAGE_HANDLER(spi_device_handle_t *spi, enum flowFlag *flowFlagPtr) {
    // Receive Message Object
    CAN_RX_MSGOBJ rxObj;
    uint8_t rxd[MAX_DATA_BYTES] = {0};

    //printf("msg.c: in the msg handler\n");    // We get to here. 

    if (DRV_CAN_READ_OBJ(spi,&rxd,&rxObj)==0){

        // the following prints out the entire frame.
        // printf("SMC_MSG_HANDLER Rx = ");
        // for (uint8_t i = 0; i<MAX_DATA_BYTES; i++) {
        //      printf("%X",rxd[i]);
        // }
        // printf("\n");

        //  printf("SID: %X\n",rxObj.bF.id.SID);

        switch (rxObj.bF.id.SID) {

            //case TIMESTAMP: get_timestamp(&rxd);
            //case SPACECRAFT_STATE: get_spacecraft_state(rxd);
            case P_RDY_OP:
                printf("Received:    P_RDY_OP (Payload is Ready for Opperation)\n");
                // DRV_CAN_WRITE(&spi,0,BEGIN,CAN_DLC_64);
                break;
            case P_BEG_OP:
                printf("Received:    P_BEG_OP (Payload Beginning Opperation)\n");
                // Payload is begining operation
                break;
            case P_STOP:
                printf("Received:    P_STOP (Payload has completed opperation)\n");
                // Payload has ceased opperation
                break;
            case P_PDOWN:
                printf("Received:    P_PDOWN (Payload is ready to be powered down)\n");
                // Payload is ready for powerdown
                break;
            case P_QDATA_RSP:
                printf("Received:   P_QDATA_RSP (Payload has data ready to send)\n");
                // Payload has data to transmit
                break;
            case P_ISOTP:
                printf("Received:   P_ISOTP\n");
                // Series of messages that conform to ISO TP to carry out data transmission
                break;
            case X_PDOWN:
                printf("Received:    X_PDOWN\n");
                *flowFlagPtr = ESD;
                //powerdown();
                break;
            case X_ALL_PDOWN:
                printf("Received:   X_ALL_PDOWN\n");
                *flowFlagPtr = ESD;
                //powerdown();
                break;
            case X_BEG_OP: 
                printf("Received:    X_BEG_OP\n");
                // *flowFlagPtr = GREENLIGHT;   // not needed in host 
                vTaskDelay(100/portTICK_PERIOD_MS);
                // *flowFlagPtr = NOMINAL;      // not needed in host 
                // begin(&rxd);
                break;
            case X_STOP:
                printf("Received:    X_STOP\n");
                *flowFlagPtr = ESD;
                //stop();
                break;
            case X_QDATA: 
                printf("Received:    X_QDATA\n");
                // Query Response                
                // handle_tx(spi);
                break;
            
            // case RECEIVE_CMD: 
            //     handle_rx(spi,rxd);
            //     // These cases will only occur during data transmission -> handled in handle_quer();
            //     // case TRANSMIT_CMD: break;
            //     // case TRANSMIT_FLOW: break;
            //     // case TRANSMIT_RESULT: break;
            //     printf("Listen(): CAN Received: RECEIVE_CMD\n");
            //     break;
            // case RESPONSE:
            //     printf("Listen(): CAN Received: RESPONSE\n");
            //     // send the Transmit data command 
                
            default: break;
        }
    }else{
            // Do nothing 
    }       
}


 //void get_timestamp(uint8_t* rxd) {
     // this can also be a pointer to the timestamp saved somewhere in the main run file?
    // uint64_t timestamp = 0;

   //  memcpy(timestamp, rxd, sizeof(uint64_t));

     // invoke interrupt to save new timestamp?
// }

// Alternative
/*
void get_timestamp(uint8_t* rxd) {
    struct timeval tv;
    
    uint64_t usecs;
    memcpy(&usecs,rxd,sizeof(uint64_t));

    tv.tv_sec = usecs/1000;
    tv.tv_usec = (usecs%1000)*1000;

    settimeofday(&tv, NULL);
    printf("Time set to %ld.%06ld\n\n", tv.tv_sec, (long int)tv.tv_usec);
}

// Same as above, probably not needed
void get_spacecraft_state(uint8_t rxd[MAX_DATA_BYTES]) {
    Spacecraft_State state = {0};

    memcpy(&state, rxd, sizeof(Spacecraft_State));
    for (int i = 0; i < sizeof(Spacecraft_State); i++)
    {
        printf("%02x ",rxd[i]);
    }
    printf("\n");

     printf("Spacecraft State:\n"
        "lat:%f\n"
        "lon:%f\n"
        "alt:%f\n"
        "roll:%f\n"
        "pitch:%f\n"
        "heading:%f\n"
        "vel_n:%f\n"
        "vel_e:%f\n"
        "vel_d:%f\n"
        "ang_vel_x:%f\n"
        "ang_vel_y:%f\n"
        "ang_vel_z:%f\n\n",
        state.Latitude, state.Longitude, state.Altitude, state.Roll, state.Pitch, state.Heading, state.Velocity_N, state.Velocity_E, state.Velocity_D, state.Angular_VX, state.Angular_VY, state.Angular_VZ);
        

}
*/
void powerdown() {
    // Invoke interrupt to start powerdown of spacecraft
    printf("Powering down spacecraft\n\n");
    vTaskDelete(NULL);
}

void begin(uint8_t* rxd) {
    // Invoke interrupt to start operation
    printf("Starting operation\n\n");
}

void stop() {
    // Invoke interrupt to cease operation
    printf("Stopping operation\n\n");
    vTaskDelete(TxHandle);
}

void handle_tx(spi_device_handle_t* spi) {

    // Check if data is available for transmit and get name of file

    uint8_t data = 0;
    uint32_t data_len = sizeof(data);

    char *name = "Dandelions.txt";
    uint8_t name_len = strlen(name) + 1; // add 1 for null terminator
    uint8_t reply_len = MD5_DIGEST_LENGTH + name_len; 
    
    // Reply with MD5 hash and UTF-8 filename of data
    // Creat MD5 hash digest
    uint8_t digest[MD5_DIGEST_LENGTH];
    MD5(digest, &data, sizeof(data));

    uint8_t* reply = (uint8_t*) malloc(sizeof(data_len)); // allocate memory for data

    memcpy(reply,digest,MD5_DIGEST_LENGTH);

   // memcpy(reply+MD5_DIGEST_LENGTH,name,name_len); //PATRICK

    DRV_CAN_WRITE(spi,reply,P_QDATA_RSP,CAN_DLC_64);

    bool ack = false;
    bool recv = false;
    // Implement a wait loop for ack
    while (ack == false) {
        CAN_RX_MSGOBJ msgObj;
        uint8_t rxd[MAX_DATA_BYTES] = {0};
        DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);

        if (msgObj.bF.id.SID == X_TX_DATA) {
            ack = true;
        }
    }

    if (iso_tp_send(spi,data,data_len)) {
        while (recv == false) {
            CAN_RX_MSGOBJ msgObj;
            uint8_t rxd[MAX_DATA_BYTES] = {0};
            DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);

            if (msgObj.bF.id.SID == X_TX_ACK) {
                // It doesn't matter here if it fails or succeeds, we don't have any way to initiate a retry
                recv = true;
            }
        }
    }          
}

// void handle_rx(spi_device_handle_t* spi, uint8_t* read) {
//     Iso_Tp_File file;
//     file.md5 = ((uint16_t) read[0]<<8) + (uint16_t) read[1];
//     file.name = (char *) (read+2);

//     uint8_t txd = NULL;
    
//     DRV_CAN_WRITE(spi, &txd, RX_READY, CAN_DLC_64);

//     iso_tp_receive(spi);

//     /* save rxd to a file*/
// }