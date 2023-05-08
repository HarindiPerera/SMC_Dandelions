/*******************************************************************************
   CAN FD Driver: API Header File

  Company:
    Dandelions

  File Name:
    drv_can.h

  Summary:
    This header file contains object declarations used in the API.
    This also contains device specific defines.

  Description:
    None.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <canfdspi/drv_canfdspi_api.h>
#include <canfdspi/drv_canfdspi_defines.h>
#include <canfdspi/drv_canfdspi_register.h>
#include <spi/drv_spi.h>

void DRV_CAN_INIT(spi_device_handle_t* spi);

bool DRV_CAN_READ(spi_device_handle_t* spi, uint8_t* rxd);
int DRV_CAN_READ_OBJ(spi_device_handle_t* spi, uint8_t* rxd, CAN_RX_MSGOBJ* rxObj);

void DRV_CAN_WRITE(spi_device_handle_t* spi, uint8_t* txd, uint16_t id, CAN_DLC length);
