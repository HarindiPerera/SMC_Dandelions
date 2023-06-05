/*******************************************************************************
  CAN FD SPI Driver: API Functions Header File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_canfdspi_api.h

  Summary:
    This header file provides the API function prototypes for the CAN FD SPI
    controller.

  Description:
    API function prototypes for the CAN FD SPI controller family:
	MCP2517FD, and MCP2518FD.
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

#ifndef _DRV_CANFDSPI_API_H
#define _DRV_CANFDSPI_API_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "drv_canfdspi_defines.h"
#include "drv_canfdspi_register.h"
#include "driver/spi_master.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END  

typedef uint8_t CANFDSPI_MODULE_ID;

// *****************************************************************************
// *****************************************************************************
//! Reset DUT

int8_t DRV_CANFDSPI_Reset(spi_device_handle_t* spi);


// *****************************************************************************
// *****************************************************************************
// Section: SPI Access Functions

// *****************************************************************************
//! SPI Read Byte

int8_t DRV_CANFDSPI_ReadByte(spi_device_handle_t* spi, uint16_t address,
        uint8_t *rxd);

// *****************************************************************************
//! SPI Write Byte

int8_t DRV_CANFDSPI_WriteByte(spi_device_handle_t* spi, uint16_t address,
        uint8_t txd);

// *****************************************************************************
//! SPI Read Word

int8_t DRV_CANFDSPI_ReadWord(spi_device_handle_t* spi, uint16_t address,
        uint32_t *rxd);

// *****************************************************************************
//! SPI Write Word

int8_t DRV_CANFDSPI_WriteWord(spi_device_handle_t* spi, uint16_t address,
        uint32_t txd);

/// *****************************************************************************
//! SPI Read Word

int8_t DRV_CANFDSPI_ReadHalfWord(spi_device_handle_t* spi, uint16_t address,
        uint16_t *rxd);

// *****************************************************************************
//! SPI Write Word

int8_t DRV_CANFDSPI_WriteHalfWord(spi_device_handle_t* spi, uint16_t address,
        uint16_t txd);

// *****************************************************************************
//! SPI Read Byte Array

int8_t DRV_CANFDSPI_ReadByteArray(spi_device_handle_t* spi, uint16_t address,
        uint8_t *rxd, uint8_t nBytes);

// *****************************************************************************
//! SPI Write Byte Array

int8_t DRV_CANFDSPI_WriteByteArray(spi_device_handle_t* spi, uint16_t address,
        uint8_t *txd, uint8_t nBytes);

// *****************************************************************************
//! SPI SFR Write Byte Safe
/*!
 * Writes Byte to SFR at address using SPI CRC. Byte gets only written if CRC matches.
 * 
 * Remark: The function doesn't check if the address is an SFR address.
 */

int8_t DRV_CANFDSPI_WriteByteSafe(spi_device_handle_t* spi, uint16_t address,
        uint8_t txd);

// *****************************************************************************
//! SPI RAM Write Word Safe
/*!
 * Writes Word to RAM at address using SPI CRC. Word gets only written if CRC matches.
 * 
 * Remark: The function doesn't check if the address is a RAM address.
 */

int8_t DRV_CANFDSPI_WriteWordSafe(spi_device_handle_t* spi, uint16_t address,
        uint32_t txd);

// *****************************************************************************
//! SPI Read Byte Array with CRC

int8_t DRV_CANFDSPI_ReadByteArrayWithCRC(spi_device_handle_t* spi, uint16_t address,
        uint8_t *rxd, uint8_t nBytes, bool fromRam, bool* crcIsCorrect);

// *****************************************************************************
//! SPI Write Byte Array with CRC

int8_t DRV_CANFDSPI_WriteByteArrayWithCRC(spi_device_handle_t* spi, uint16_t address,
        uint8_t *txd, uint8_t nBytes, bool fromRam);

// *****************************************************************************
//! SPI Read Word Array

int8_t DRV_CANFDSPI_ReadWordArray(spi_device_handle_t* spi, uint16_t address,
        uint32_t *rxd, uint16_t nWords);

// *****************************************************************************
//! SPI Write Word Array

int8_t DRV_CANFDSPI_WriteWordArray(spi_device_handle_t* spi, uint16_t address,
        uint32_t *txd, uint16_t nWords);


// *****************************************************************************
// *****************************************************************************
// Section: Configuration

// *****************************************************************************
//! CAN Control register configuration

int8_t DRV_CANFDSPI_Configure(spi_device_handle_t* spi, CAN_CONFIG* config);

// *****************************************************************************
//! Reset Configure object to reset values
int8_t DRV_CANFDSPI_ConfigureObjectReset(CAN_CONFIG* config);


// *****************************************************************************
// *****************************************************************************
// Section: Operating mode

// *****************************************************************************
//! Select Operation Mode

int8_t DRV_CANFDSPI_OperationModeSelect(spi_device_handle_t* spi,
        CAN_OPERATION_MODE opMode);

// *****************************************************************************
//! Get Operation Mode

CAN_OPERATION_MODE DRV_CANFDSPI_OperationModeGet(spi_device_handle_t* spi);

// // *****************************************************************************
// //! Enable Low Power Mode

// int8_t DRV_CANFDSPI_LowPowerModeEnable(spi_device_handle_t* spi);

// // *****************************************************************************
// //! Disable Low Power Mode

// int8_t DRV_CANFDSPI_LowPowerModeDisable(spi_device_handle_t* spi);


// *****************************************************************************
// *****************************************************************************
// Section: CAN Transmit

// *****************************************************************************
//! Configure Transmit FIFO

int8_t DRV_CANFDSPI_TransmitChannelConfigure(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_CONFIG* config);

// *****************************************************************************
//! Reset TransmitChannelConfigure object to reset values

int8_t DRV_CANFDSPI_TransmitChannelConfigureObjectReset(CAN_TX_FIFO_CONFIG* config);

// *****************************************************************************
//! Configure Transmit Queue

int8_t DRV_CANFDSPI_TransmitQueueConfigure(spi_device_handle_t* spi,
        CAN_TX_QUEUE_CONFIG* config);

// *****************************************************************************
//! Reset TransmitQueueConfigure object to reset values

int8_t DRV_CANFDSPI_TransmitQueueConfigureObjectReset(CAN_TX_QUEUE_CONFIG* config);

// *****************************************************************************
//! TX Channel Load
/*!
 * Loads data into Transmit channel
 * Requests transmission, if flush==true
 */

int8_t DRV_CANFDSPI_TransmitChannelLoad(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush);

// *****************************************************************************
//! TX Queue Load

/*!
 * Loads data into Transmit Queue
 * Requests transmission, if flush==true
 */

inline int8_t DRV_CANFDSPI_TransmitQueueLoad(spi_device_handle_t* spi,
        CAN_TX_MSGOBJ* txObj,
        uint8_t *txd, uint32_t txdNumBytes, bool flush)
{
    return DRV_CANFDSPI_TransmitChannelLoad(spi, CAN_TXQUEUE_CH0, txObj, txd, txdNumBytes, flush);
}

// *****************************************************************************
//! TX Channel Flush
/*!
 * Set TXREG of one channel
 */

int8_t DRV_CANFDSPI_TransmitChannelFlush(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Transmit Channel Status Get

int8_t DRV_CANFDSPI_TransmitChannelStatusGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_STATUS* status);

// *****************************************************************************
//! Transmit FIFO Reset

int8_t DRV_CANFDSPI_TransmitChannelReset(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Transmit FIFO Update
/*!
 * Sets UINC of the transmit channel.
 * Requests transmission, if flush==true
 */

int8_t DRV_CANFDSPI_TransmitChannelUpdate(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, bool flush);

// *****************************************************************************
//! TX Queue Flush

/*!
 * Set TXREG of one channel
 */

inline int8_t DRV_CANFDSPI_TransmitQueueFlush(spi_device_handle_t* spi)
{
    return DRV_CANFDSPI_TransmitChannelFlush(spi, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Transmit Queue Status Get

inline int8_t DRV_CANFDSPI_TransmitQueueStatusGet(spi_device_handle_t* spi,
        CAN_TX_FIFO_STATUS* status)
{
    return DRV_CANFDSPI_TransmitChannelStatusGet(spi, CAN_TXQUEUE_CH0, status);
}

// *****************************************************************************
//! Transmit Queue Reset

inline int8_t DRV_CANFDSPI_TransmitQueueReset(spi_device_handle_t* spi)
{
    return DRV_CANFDSPI_TransmitChannelReset(spi, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Transmit Queue Update

/*!
 * Sets UINC of the transmit channel.
 * Requests transmission, if flush==true
 */

inline int8_t DRV_CANFDSPI_TransmitQueueUpdate(spi_device_handle_t* spi, bool flush)
{
    return DRV_CANFDSPI_TransmitChannelUpdate(spi, CAN_TXQUEUE_CH0, flush);
}

// *****************************************************************************
//! Request transmissions using TXREQ register

int8_t DRV_CANFDSPI_TransmitRequestSet(spi_device_handle_t* spi,
        CAN_TXREQ_CHANNEL txreq);

// *****************************************************************************
//! Get TXREQ register

int8_t DRV_CANFDSPI_TransmitRequestGet(spi_device_handle_t* spi,
        uint32_t* txreq);

// *****************************************************************************
//! Abort transmission of single FIFO

int8_t DRV_CANFDSPI_TransmitChannelAbort(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Abort transmission of TXQ

inline int8_t DRV_CANFDSPI_TransmitQueueAbort(spi_device_handle_t* spi)
{
    return DRV_CANFDSPI_TransmitChannelAbort(spi, CAN_TXQUEUE_CH0);
}

// *****************************************************************************
//! Abort All transmissions

int8_t DRV_CANFDSPI_TransmitAbortAll(spi_device_handle_t* spi);

// *****************************************************************************
//! Set Transmit Bandwidth Sharing Delay
int8_t DRV_CANFDSPI_TransmitBandWidthSharingSet(spi_device_handle_t* spi,
        CAN_TX_BANDWITH_SHARING txbws);


// *****************************************************************************
// *****************************************************************************
// Section: CAN Receive

// *****************************************************************************
//! Filter Object Configuration
/*!
 * Configures ID of filter object
 */

int8_t DRV_CANFDSPI_FilterObjectConfigure(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_FILTEROBJ_ID* id);

// *****************************************************************************
//! Filter Mask Configuration
/*!
 * Configures Mask of filter object
 */

int8_t DRV_CANFDSPI_FilterMaskConfigure(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_MASKOBJ_ID* mask);

// *****************************************************************************
//! Link Filter to FIFO
/*!
 * Initializes the Pointer from Filter to FIFO
 * Enables or disables the Filter
 */

int8_t DRV_CANFDSPI_FilterToFifoLink(spi_device_handle_t* spi,
        CAN_FILTER filter, CAN_FIFO_CHANNEL channel, bool enable);

// *****************************************************************************
//! Filter Enable

int8_t DRV_CANFDSPI_FilterEnable(spi_device_handle_t* spi, CAN_FILTER filter);

// *****************************************************************************
//! Filter Disable

int8_t DRV_CANFDSPI_FilterDisable(spi_device_handle_t* spi, CAN_FILTER filter);

// *****************************************************************************
//! Set Device Net Filter Count
int8_t DRV_CANFDSPI_DeviceNetFilterCountSet(spi_device_handle_t* spi,
        CAN_DNET_FILTER_SIZE dnfc);

// *****************************************************************************
//! Configure Receive FIFO

int8_t DRV_CANFDSPI_ReceiveChannelConfigure(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_CONFIG* config);

// *****************************************************************************
//! Reset ReceiveChannelConfigure object to reset value

int8_t DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(CAN_RX_FIFO_CONFIG* config);

// *****************************************************************************
//! Receive Channel Status Get

int8_t DRV_CANFDSPI_ReceiveChannelStatusGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_STATUS* status);

// *****************************************************************************
//! Get Received Message
/*!
 * Reads Received message from channel
 */

int8_t DRV_CANFDSPI_ReceiveMessageGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_MSGOBJ* rxObj,
        uint8_t* rxd, uint8_t nBytes);

// *****************************************************************************
//! Receive FIFO Reset

int8_t DRV_CANFDSPI_ReceiveChannelReset(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);

// *****************************************************************************
//! Receive FIFO Update
/*!
 * Sets UINC of the receive channel.
 */

int8_t DRV_CANFDSPI_ReceiveChannelUpdate(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO

// *****************************************************************************
//! Transmit Event FIFO Status Get

int8_t DRV_CANFDSPI_TefStatusGet(spi_device_handle_t* spi,
        CAN_TEF_FIFO_STATUS* status);

// *****************************************************************************
//! Get Transmit Event FIFO Message
/*!
 * Reads Transmit Event FIFO message
 */

int8_t DRV_CANFDSPI_TefMessageGet(spi_device_handle_t* spi,
        CAN_TEF_MSGOBJ* tefObj);

// *****************************************************************************
//! Transmit Event FIFO Reset

int8_t DRV_CANFDSPI_TefReset(spi_device_handle_t* spi);

// *****************************************************************************
//! Transmit Event FIFO Update
/*!
 * Sets UINC of the TEF.
 */

int8_t DRV_CANFDSPI_TefUpdate(spi_device_handle_t* spi);

// *****************************************************************************
//! Configure Transmit Event FIFO

int8_t DRV_CANFDSPI_TefConfigure(spi_device_handle_t* spi, CAN_TEF_CONFIG* config);

// *****************************************************************************
//! Reset TefConfigure object to reset value

int8_t DRV_CANFDSPI_TefConfigureObjectReset(CAN_TEF_CONFIG* config);


// *****************************************************************************
// *****************************************************************************
// Section: Module Events

// *****************************************************************************
//! Module Event Get
/*!
 * Reads interrupt Flags
 */

int8_t DRV_CANFDSPI_ModuleEventGet(spi_device_handle_t* spi,
        CAN_MODULE_EVENT* flags);

// *****************************************************************************
//! Module Event Enable
/*!
 * Enables interrupts
 */

int8_t DRV_CANFDSPI_ModuleEventEnable(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Module Event Disable
/*!
 * Disables interrupts
 */

int8_t DRV_CANFDSPI_ModuleEventDisable(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Module Event Clear
/*!
 * Clears interrupt Flags
 */

int8_t DRV_CANFDSPI_ModuleEventClear(spi_device_handle_t* spi,
        CAN_MODULE_EVENT flags);

// *****************************************************************************
//! Get RX Code

int8_t DRV_CANFDSPI_ModuleEventRxCodeGet(spi_device_handle_t* spi,
        CAN_RXCODE* rxCode);

// *****************************************************************************
//! Get TX Code

int8_t DRV_CANFDSPI_ModuleEventTxCodeGet(spi_device_handle_t* spi,
        CAN_TXCODE* txCode);

// *****************************************************************************
//! Get Filter Hit

int8_t DRV_CANFDSPI_ModuleEventFilterHitGet(spi_device_handle_t* spi,
        CAN_FILTER* filterHit);

// *****************************************************************************
//! Get ICODE

int8_t DRV_CANFDSPI_ModuleEventIcodeGet(spi_device_handle_t* spi,
        CAN_ICODE* icode);

// *****************************************************************************
// *****************************************************************************
// Section: Transmit FIFO Events

// *****************************************************************************
//! Transmit FIFO Event Get
/*!
 * Reads Transmit FIFO interrupt Flags
 */

int8_t DRV_CANFDSPI_TransmitChannelEventGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT* flags);

// *****************************************************************************
//! Transmit Queue Event Get

/*!
 * Reads Transmit Queue interrupt Flags
 */

inline int8_t DRV_CANFDSPI_TransmitQueueEventGet(spi_device_handle_t* spi,
        CAN_TX_FIFO_EVENT* flags)
{
    return DRV_CANFDSPI_TransmitChannelEventGet(spi, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Get pending interrupts of all transmit FIFOs

int8_t DRV_CANFDSPI_TransmitEventGet(spi_device_handle_t* spi, uint32_t* txif);

// *****************************************************************************
//! Get pending TXATIF of all transmit FIFOs

int8_t DRV_CANFDSPI_TransmitEventAttemptGet(spi_device_handle_t* spi,
        uint32_t* txatif);

// *****************************************************************************
//! Transmit FIFO spi Get
/*!
 * Reads Transmit FIFO spi
 */

int8_t DRV_CANFDSPI_TransmitChannelIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* idx);

// *****************************************************************************
//! Transmit FIFO Event Enable
/*!
 * Enables Transmit FIFO interrupts
 */

int8_t DRV_CANFDSPI_TransmitChannelEventEnable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit FIFO Event Disable
/*!
 * Disables Transmit FIFO interrupts
 */

int8_t DRV_CANFDSPI_TransmitChannelEventDisable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_TX_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit FIFO Event Clear
/*!
 * Clears Transmit FIFO Attempts Exhausted interrupt Flag
 */

int8_t DRV_CANFDSPI_TransmitChannelEventAttemptClear(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
//! Transmit Queue spi Get

/*!
 * Reads Transmit Queue spi
 */

inline int8_t DRV_CANFDSPI_TransmitQueueIndexGet(spi_device_handle_t* spi, uint8_t* idx)
{
    return DRV_CANFDSPI_TransmitChannelIndexGet(spi, CAN_TXQUEUE_CH0, idx);
}

// *****************************************************************************
//! Transmit Queue Event Enable

/*!
 * Enables Transmit Queue interrupts
 */

inline int8_t DRV_CANFDSPI_TransmitQueueEventEnable(spi_device_handle_t* spi,
        CAN_TX_FIFO_EVENT flags)
{
    return DRV_CANFDSPI_TransmitChannelEventEnable(spi, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Transmit Queue Event Disable

/*!
 * Disables Transmit FIFO interrupts
 */

inline int8_t DRV_CANFDSPI_TransmitQueueEventDisable(spi_device_handle_t* spi,
        CAN_TX_FIFO_EVENT flags)
{
    return DRV_CANFDSPI_TransmitChannelEventDisable(spi, CAN_TXQUEUE_CH0, flags);
}

// *****************************************************************************
//! Transmit Queue Event Clear

/*!
 * Clears Transmit FIFO Attempts Exhausted interrupt Flag
 */

inline int8_t DRV_CANFDSPI_TransmitQueueEventAttemptClear(spi_device_handle_t* spi)
{
    return DRV_CANFDSPI_TransmitChannelEventAttemptClear(spi, CAN_TXQUEUE_CH0);
}


// *****************************************************************************
// *****************************************************************************
// Section: Receive FIFO Events

// *****************************************************************************
//! Receive FIFO Event Get
/*!
 * Reads Receive FIFO interrupt Flags
 */

int8_t DRV_CANFDSPI_ReceiveChannelEventGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT* flags);

// *****************************************************************************
//! Get pending interrupts of all receive FIFOs

int8_t DRV_CANFDSPI_ReceiveEventGet(spi_device_handle_t* spi, uint32_t* rxif);

// *****************************************************************************
//!Get pending RXOVIF of all receive FIFOs

int8_t DRV_CANFDSPI_ReceiveEventOverflowGet(spi_device_handle_t* spi, uint32_t* rxovif);

// *****************************************************************************
//! Receive FIFO spi Get
/*!
 * Reads Receive FIFO spi
 */

int8_t DRV_CANFDSPI_ReceiveChannelIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* idx);

// *****************************************************************************
//! Receive FIFO Event Enable
/*!
 * Enables Receive FIFO interrupts
 */

int8_t DRV_CANFDSPI_ReceiveChannelEventEnable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags);

// *****************************************************************************
//! Receive FIFO Event Disable
/*!
 * Disables Receive FIFO interrupts
 */

int8_t DRV_CANFDSPI_ReceiveChannelEventDisable(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, CAN_RX_FIFO_EVENT flags);

// *****************************************************************************
//! Receive FIFO Event Clear
/*!
 * Clears Receive FIFO Overflow interrupt Flag
 */

int8_t DRV_CANFDSPI_ReceiveChannelEventOverflowClear(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel);


// *****************************************************************************
// *****************************************************************************
// Section: Transmit Event FIFO Events

// *****************************************************************************
//! Transmit Event FIFO Event Get
/*!
 * Reads Transmit Event FIFO interrupt Flags
 */

int8_t DRV_CANFDSPI_TefEventGet(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT* flags);

// *****************************************************************************
//! Transmit Event FIFO Event Enable
/*!
 * Enables Transmit Event FIFO interrupts
 */

int8_t DRV_CANFDSPI_TefEventEnable(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit Event FIFO Event Disable
/*!
 * Disables Transmit Event FIFO interrupts
 */

int8_t DRV_CANFDSPI_TefEventDisable(spi_device_handle_t* spi,
        CAN_TEF_FIFO_EVENT flags);

// *****************************************************************************
//! Transmit Event FIFO Event Clear
/*!
 * Clears Transmit Event FIFO Overflow interrupt Flag
 */

int8_t DRV_CANFDSPI_TefEventOverflowClear(spi_device_handle_t* spi);


// *****************************************************************************
// *****************************************************************************
// Section: Error Handling

// *****************************************************************************
//! Transmit Error Count Get

int8_t DRV_CANFDSPI_ErrorCountTransmitGet(spi_device_handle_t* spi,
        uint8_t* tec);

// *****************************************************************************
//! Receive Error Count Get

int8_t DRV_CANFDSPI_ErrorCountReceiveGet(spi_device_handle_t* spi,
        uint8_t* rec);

// *****************************************************************************
//! Error State Get

int8_t DRV_CANFDSPI_ErrorStateGet(spi_device_handle_t* spi,
        CAN_ERROR_STATE* flags);

// *****************************************************************************
//! Error Counts and Error State Get
/*!
 * Returns content of complete CiTREC
 */

int8_t DRV_CANFDSPI_ErrorCountStateGet(spi_device_handle_t* spi,
        uint8_t* tec, uint8_t* rec, CAN_ERROR_STATE* flags);

// *****************************************************************************
//! Get Bus Diagnostic Registers: all data at once, since we want to keep them in synch

int8_t DRV_CANFDSPI_BusDiagnosticsGet(spi_device_handle_t* spi,
        CAN_BUS_DIAGNOSTIC* bd);

// *****************************************************************************
//! Clear Bus Diagnostic Registers

int8_t DRV_CANFDSPI_BusDiagnosticsClear(spi_device_handle_t* spi);


// *****************************************************************************
// *****************************************************************************
// Section: ECC

// *****************************************************************************
//! Enable ECC

int8_t DRV_CANFDSPI_EccEnable(spi_device_handle_t* spi);

// *****************************************************************************
//! Disable ECC

int8_t DRV_CANFDSPI_EccDisable(spi_device_handle_t* spi);

// *****************************************************************************
//! ECC Event Get

int8_t DRV_CANFDSPI_EccEventGet(spi_device_handle_t* spi,
        CAN_ECC_EVENT* flags);

// *****************************************************************************
//! Set ECC Parity

int8_t DRV_CANFDSPI_EccParitySet(spi_device_handle_t* spi,
        uint8_t parity);

// *****************************************************************************
//! Get ECC Parity

int8_t DRV_CANFDSPI_EccParityGet(spi_device_handle_t* spi,
        uint8_t* parity);

// *****************************************************************************
//! Get ECC Error Address

int8_t DRV_CANFDSPI_EccErrorAddressGet(spi_device_handle_t* spi,
        uint16_t* a);

// *****************************************************************************
//! ECC Event Enable

int8_t DRV_CANFDSPI_EccEventEnable(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! ECC Event Disable

int8_t DRV_CANFDSPI_EccEventDisable(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! ECC Event Clear

int8_t DRV_CANFDSPI_EccEventClear(spi_device_handle_t* spi,
        CAN_ECC_EVENT flags);

// *****************************************************************************
//! Initialize RAM

int8_t DRV_CANFDSPI_RamInit(spi_device_handle_t* spi, uint8_t d);


// *****************************************************************************
// *****************************************************************************
// Section: CRC

// *****************************************************************************
//! CRC Event Enable

int8_t DRV_CANFDSPI_CrcEventEnable(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Disable

int8_t DRV_CANFDSPI_CrcEventDisable(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Clear

int8_t DRV_CANFDSPI_CrcEventClear(spi_device_handle_t* spi,
        CAN_CRC_EVENT flags);

// *****************************************************************************
//! CRC Event Get

int8_t DRV_CANFDSPI_CrcEventGet(spi_device_handle_t* spi, CAN_CRC_EVENT* flags);

// *****************************************************************************
//! Get CRC Value from device

int8_t DRV_CANFDSPI_CrcValueGet(spi_device_handle_t* spi, uint16_t* crc);


// *****************************************************************************
// *****************************************************************************
// Section: Time Stamp

// *****************************************************************************
//! Time Stamp Enable

int8_t DRV_CANFDSPI_TimeStampEnable(spi_device_handle_t* spi);

// *****************************************************************************
//! Time Stamp Disable

int8_t DRV_CANFDSPI_TimeStampDisable(spi_device_handle_t* spi);

// *****************************************************************************
//! Time Stamp Get

int8_t DRV_CANFDSPI_TimeStampGet(spi_device_handle_t* spi, uint32_t* ts);

// *****************************************************************************
//! Time Stamp Set

int8_t DRV_CANFDSPI_TimeStampSet(spi_device_handle_t* spi, uint32_t ts);

// *****************************************************************************
//! Time Stamp Mode Configure

int8_t DRV_CANFDSPI_TimeStampModeConfigure(spi_device_handle_t* spi,
        CAN_TS_MODE mode);

// *****************************************************************************
//! Time Stamp Prescaler Set

int8_t DRV_CANFDSPI_TimeStampPrescalerSet(spi_device_handle_t* spi,
        uint16_t ps);


// *****************************************************************************
// *****************************************************************************
// Section: Oscillator and Bit Time

// *****************************************************************************
//! Enable oscillator to wake-up from sleep

int8_t DRV_CANFDSPI_OscillatorEnable(spi_device_handle_t* spi);

// *****************************************************************************
//! Set Oscillator Control

int8_t DRV_CANFDSPI_OscillatorControlSet(spi_device_handle_t* spi,
        CAN_OSC_CTRL ctrl);

int8_t DRV_CANFDSPI_OscillatorControlObjectReset(CAN_OSC_CTRL* ctrl);


// *****************************************************************************
//! Get Oscillator Status

int8_t DRV_CANFDSPI_OscillatorStatusGet(spi_device_handle_t* spi,
        CAN_OSC_STATUS* status);

// *****************************************************************************
//! Configure Bit Time registers (based on CAN clock speed)

int8_t DRV_CANFDSPI_BitTimeConfigure(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode,
        CAN_SYSCLK_SPEED clk);

// *****************************************************************************
//! Configure Nominal bit time for 40MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureNominal40MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 40MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureData40MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);

// *****************************************************************************
//! Configure Nominal bit time for 20MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureNominal20MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 20MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureData20MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);

// *****************************************************************************
//! Configure Nominal bit time for 10MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureNominal10MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime);

// *****************************************************************************
//! Configure Data bit time for 10MHz system clock

int8_t DRV_CANFDSPI_BitTimeConfigureData10MHz(spi_device_handle_t* spi,
        CAN_BITTIME_SETUP bitTime, CAN_SSP_MODE sspMode);


// *****************************************************************************
// *****************************************************************************
// Section: GPIO

// *****************************************************************************
//! Initialize GPIO Mode

int8_t DRV_CANFDSPI_GpioModeConfigure(spi_device_handle_t* spi,
        GPIO_PIN_MODE gpio0, GPIO_PIN_MODE gpio1);

// *****************************************************************************
//! Initialize GPIO Direction

int8_t DRV_CANFDSPI_GpioDirectionConfigure(spi_device_handle_t* spi,
        GPIO_PIN_DIRECTION gpio0, GPIO_PIN_DIRECTION gpio1);

// *****************************************************************************
//! Enable Transceiver Standby Control

int8_t DRV_CANFDSPI_GpioStandbyControlEnable(spi_device_handle_t* spi);

// *****************************************************************************
//! Disable Transceiver Standby Control

int8_t DRV_CANFDSPI_GpioStandbyControlDisable(spi_device_handle_t* spi);

// *****************************************************************************
//! Configure Open Drain Interrupts

int8_t DRV_CANFDSPI_GpioInterruptPinsOpenDrainConfigure(spi_device_handle_t* spi,
        GPIO_OPEN_DRAIN_MODE mode);

// *****************************************************************************
//! Configure Open Drain TXCAN

int8_t DRV_CANFDSPI_GpioTransmitPinOpenDrainConfigure(spi_device_handle_t* spi,
        GPIO_OPEN_DRAIN_MODE mode);

// *****************************************************************************
//! GPIO Output Pin Set

int8_t DRV_CANFDSPI_GpioPinSet(spi_device_handle_t* spi,
        GPIO_PIN_POS pos, GPIO_PIN_STATE latch);

// *****************************************************************************
//! GPIO Input Pin Read

int8_t DRV_CANFDSPI_GpioPinRead(spi_device_handle_t* spi,
        GPIO_PIN_POS pos, GPIO_PIN_STATE* state);

// *****************************************************************************
//! Configure CLKO Pin

int8_t DRV_CANFDSPI_GpioClockOutputConfigure(spi_device_handle_t* spi,
        GPIO_CLKO_MODE mode);


// *****************************************************************************
// *****************************************************************************
// Section: Miscellaneous

// *****************************************************************************
//! DLC to number of actual data bytes conversion

uint32_t DRV_CANFDSPI_DlcToDataBytes(CAN_DLC dlc);

// *****************************************************************************
//! FIFO spi Get

int8_t DRV_CANFDSPI_FifoIndexGet(spi_device_handle_t* spi,
        CAN_FIFO_CHANNEL channel, uint8_t* mi);

// *****************************************************************************
//! Calculate CRC16

uint16_t DRV_CANFDSPI_CalculateCRC16(uint8_t* data, uint16_t size);

// *****************************************************************************
//! Data bytes to DLC conversion

CAN_DLC DRV_CANFDSPI_DataBytesToDlc(uint8_t n);


#endif // _DRV_CANFDSPI_API_H