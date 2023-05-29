/*******************************************************************************
  CAN Driver: Implementation

  Company:
    Dandelions

  File Name:
    drv_can.c

  Summary:
    API implementation.

  Description:
    .
 *******************************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "drv_can.h"
#include <canfdspi/drv_canfdspi_api.h>
#include <canfdspi/drv_canfdspi_defines.h>
#include <canfdspi/drv_canfdspi_register.h>
//#include <spi/drv_spi.h>
#include <msg/msg.h>

void DRV_CAN_FILTER_CONFIG(spi_device_handle_t* spi) {
    // Disable Filter 0
    DRV_CANFDSPI_FilterDisable(spi,CAN_FILTER0);

    // Configure Filter Object 0
    CAN_FILTEROBJ_ID fObj;
    fObj.SID = 0x000;
    fObj.SID11 = 0;
    fObj.EID = 0;
    fObj.EXIDE = 0; // only expect Standard frames

    DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER0, &fObj);

    // Configure Mask Object 0
    CAN_MASKOBJ_ID mObj;
    mObj.MSID = 0x000;
    mObj.MSID11 = 0;
    mObj.MEID = 0;
    mObj.MIDE = 0; // match IDE bit

    DRV_CANFDSPI_FilterMaskConfigure(spi, CAN_FILTER0, &mObj);

    // Link Filter to RX FIFO and enable Filter
    bool filterEnable = true;
    DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER0, CAN_FIFO_CH2, filterEnable);

    // // Disable Filter 1
    // DRV_CANFDSPI_FilterDisable(spi,CAN_FILTER1);

    // // Configure Filter Object 1
    // CAN_FILTEROBJ_ID fObj_1;
    // fObj_1.SID = 0x0A0;
    // fObj_1.SID11 = 0;
    // fObj_1.EID = 0;
    // fObj_1.EXIDE = 0; // only expect Standard frames

    // DRV_CANFDSPI_FilterObjectConfigure(spi, CAN_FILTER1, &fObj_1);

    // // Reuse mask object
    // DRV_CANFDSPI_FilterMaskConfigure(spi, CAN_FILTER1, &mObj);

    // // Link Filter to RX FIFO and enable Filter
    // // bool filterEnable = true;
    // DRV_CANFDSPI_FilterToFifoLink(spi, CAN_FILTER1, CAN_FIFO_CH2, filterEnable);
}

void DRV_CAN_INIT(spi_device_handle_t* spi) {
    bool v = false;
    while (!v) {
        uint8_t ret = 0;
        // Reset device
        DRV_CANFDSPI_Reset(spi);

        // Oscillator Configuration: divide by 10
        CAN_OSC_CTRL oscCtrl;
        DRV_CANFDSPI_OscillatorControlObjectReset(&oscCtrl);
        oscCtrl.ClkOutDivide = OSC_CLKO_DIV10;
        DRV_CANFDSPI_OscillatorControlSet(spi,oscCtrl);

        // Input/Output Configuration: use nINT0 and nINT1
        DRV_CANFDSPI_GpioModeConfigure(spi,GPIO_MODE_INT,GPIO_MODE_INT);
        DRV_CANFDSPI_GpioTransmitPinOpenDrainConfigure(spi,GPIO_PUSH_PULL);

        // CAN Configuration: ISO_CRC, enable TEF, enable TXQ
        CAN_CONFIG config;
        DRV_CANFDSPI_ConfigureObjectReset(&config);
        config.IsoCrcEnable = 1;
        config.StoreInTEF = 1;
        config.TXQEnable = 1;
        DRV_CANFDSPI_Configure(spi, &config);

        // Bit Time Configuration: 500k/2M, 80% sample point
        DRV_CANFDSPI_BitTimeConfigure(spi, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);

        // // TEF Configuration: 12 messages, time stamping enabled
        // CAN_TEF_CONFIG tefConfig;
        // tefConfig.FifoSize = 11;
        // tefConfig.TimeStampEnable = 1;
        // DRV_CANFDSPI_TefConfigure(spi, &tefConfig);

        // // TXQ Configuration: 8 messages, 32 byte maximum payload, low priority
        // CAN_TX_QUEUE_CONFIG txqConfig;
        // DRV_CANFDSPI_TransmitQueueConfigureObjectReset(&txqConfig);
        // txqConfig.TxPriority = 0;
        // txqConfig.FifoSize = 7;
        // txqConfig.PayLoadSize = CAN_PLSIZE_32;
        // DRV_CANFDSPI_TransmitQueueConfigure(spi, &txqConfig);

        // FIFO 1: Transmit FIFO; 16 messages, 64 byte maximum payload, high priority
        CAN_TX_FIFO_CONFIG txfConfig;
        DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txfConfig);
        txfConfig.FifoSize = 15;
        txfConfig.PayLoadSize = CAN_PLSIZE_64;
        txfConfig.TxPriority = 1;
        ret = DRV_CANFDSPI_TransmitChannelConfigure(spi, CAN_FIFO_CH1, &txfConfig);

        v = !ret;
    }
    while (!v) {
        uint8_t ret = 0;
        // FIFO 2: Receive FIFO; 16 messages, 64 byte maximum payload, time stamping enabled
        CAN_RX_FIFO_CONFIG rxfConfig;
        rxfConfig.FifoSize = 15;
        rxfConfig.PayLoadSize = CAN_PLSIZE_64;
        rxfConfig.RxTimeStampEnable = 0;
        ret |= DRV_CANFDSPI_ReceiveChannelConfigure(spi, CAN_FIFO_CH2, &rxfConfig);

        sleep(2);
        v = !ret;
    }

    DRV_CAN_FILTER_CONFIG(spi);
}

// This is only an example at the moment, what is the difference between this and the next function. 
bool DRV_CAN_READ(spi_device_handle_t* spi, uint8_t* rxd) {
    // Receive Message Object
    CAN_RX_MSGOBJ rxObj;
    // uint8_t rxd[MAX_DATA_BYTES];

    // Check that FIFO is not empty
    CAN_RX_FIFO_EVENT rxFlags;

    DRV_CANFDSPI_ReceiveChannelEventGet(spi, CAN_FIFO_CH2, &rxFlags);

    // printf("rxflag: %X\n",rxFlags);

    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
        // Read message and UINC
        printf("%X\n",DRV_CANFDSPI_ReceiveMessageGet(spi, CAN_FIFO_CH2, &rxObj, rxd, MAX_DATA_BYTES));

        return true;
        // // Process message
        // if (rxObj.bF.id.SID == 0x300 && rxObj.bF.ctrl.IDE == 0) {
        //     Nop(); Nop();
        // }
    } else {
        return false;
    }
}

int DRV_CAN_READ_OBJ(spi_device_handle_t* spi, uint8_t* rxd, CAN_RX_MSGOBJ* rxObj) {
    // Check that FIFO is not empty
    CAN_RX_FIFO_EVENT rxFlags;
    

    // how is it known that the CAN_FIFO_Ch2 is the one that we want?
    // This function actually returns something. Lets see what it is. 
    uint8_t rtn = DRV_CANFDSPI_ReceiveChannelEventGet(spi, CAN_FIFO_CH2, &rxFlags);

    //printf("FRV_CAN_READ_OBJ(): DRV_CANFDSPI_ReceiveChannelEventGet(): %d\n",rtn);
    //printf("rxflag: %X\n",rxFlags);

    // rxflag is always returning 0. which means that this function always exits with return != 0

    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
        // Read message and UINC
        printf("%X\n",DRV_CANFDSPI_ReceiveMessageGet(spi, CAN_FIFO_CH2, rxObj, rxd, MAX_DATA_BYTES));

        return 0;
    } else if (rxFlags & CAN_RX_FIFO_OVERFLOW_EVENT)
    {
        return -1;
    } else {
        return -2;
    }
}

// This is only an example at the moment
void DRV_CAN_WRITE(spi_device_handle_t* spi, uint8_t* txd, uint16_t id, CAN_DLC length) {
    // Assemble transmit message: CAN FD Base fram with BRS, 64 bytes
    CAN_TX_MSGOBJ txObj = {0};

    // Initialise ID and Control bits
    // txObj.bF.id.SID = TIMESTAMP; // Standard or Base ID
    txObj.bF.id.SID = id;
    txObj.bF.id.EID = 0;
    txObj.bF.ctrl.FDF = 1;      // CAN FD frame
    txObj.bF.ctrl.BRS = 0;      // Switch bit rate
    txObj.bF.ctrl.IDE = 0;      // Standard frame
    txObj.bF.ctrl.RTR = 0;      // Not a remote frame request
    txObj.bF.ctrl.DLC = length; // 64 data bytes
    // Sequence doesn't get transmitted but will be stored in TEF
    txObj.bF.ctrl.SEQ = 1;

    // for (uint8_t i = 0; i<MAX_DATA_BYTES; i++) {
    //   txObj.byte[i] = txd[i];
    // }

    // Initialise transmit data
    // uint8_t i;
    // for (i = 0; i<MAX_DATA_BYTES; i++) {
    //     txd[i] = i;
    // }

    // Check that FIFO is not full
    CAN_TX_FIFO_EVENT txFlags;
    bool flush = true;
    // bool flush = false;

    DRV_CANFDSPI_TransmitChannelFlush(spi,CAN_FIFO_CH1);
    DRV_CANFDSPI_TransmitChannelEventGet(spi, CAN_FIFO_CH1, &txFlags);

    printf("txflag: %X\n", txFlags);

    if (txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) {
        // Load message and transmit
        int8_t i = DRV_CANFDSPI_TransmitChannelLoad(spi, CAN_FIFO_CH1, &txObj, txd, 
                DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), flush);
        printf("drv_can.c : DRV_CANFDSPI_TransmitChannelLoad(): %d\n",i);
        //printf("DRV_CANFD_SPI_TRANSMITCHannelLoad: %X\n",DRV_CANFDSPI_TransmitChannelLoad(spi, CAN_FIFO_CH1, &txObj, txd, 
        //        DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), flush));
    }
}