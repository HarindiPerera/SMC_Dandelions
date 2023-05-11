/*******************************************************************************
   ISO Transfer Protocol: API Implementation

  Company:
    Dandelions

  File Name:
    iso_tp.c

  Summary:
    This header file contains object declarations used in the API.

  Description:
    None.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************

#include <stdlib.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "iso_tp.h"
//#include "spi/drv_spi.h"
#include "canfd/drv_can.h"
#include <string.h>
#include "msg/msg.h"
#include <unistd.h>
#include <time.h>

void iso_tp_send_single_frame(spi_device_handle_t* spi, uint8_t* data, uint8_t data_len) {
    uint8_t msgdata[MAX_DATA_BYTES] = {0};
    
    memset(msgdata,N_PCI_SF | data_len,sizeof(uint8_t));
    memcpy(msgdata+1, data, data_len);  

    if (data_len < MAX_DATA_BYTES) {
        uint8_t padding_len = MAX_DATA_BYTES - data_len;
        uint8_t* padding = (uint8_t*) calloc(padding_len, sizeof(uint8_t));
        memcpy(msgdata+2+data_len,padding,padding_len);
    }
    
    DRV_CAN_WRITE(spi,msgdata,TRANSMIT,CAN_DLC_64);
}

void iso_tp_send_first_frame(spi_device_handle_t* spi, iso_tp_control_t* iso, uint8_t* data, uint32_t data_len) {
    uint8_t msgdata[MAX_DATA_BYTES] = {0};
    uint8_t nil = 0;

    if (data_len > 4095) {
        memset(msgdata,N_PCI_FF,sizeof(uint8_t));
        memset(msgdata+1,nil,sizeof(uint8_t));
        memcpy(msgdata+2,data_len,sizeof(uint32_t));
        memcpy(msgdata+6,data,MAX_DATA_BYTES-6);
        iso->pci = PCI_32;        
    } else {
        memset(msgdata,N_PCI_FF,sizeof(uint8_t));
        memcpy(msgdata+1,(uint8_t*) data_len, sizeof(uint8_t));
        memcpy(msgdata+2, data, MAX_DATA_BYTES-2);
        iso->pci = PCI_32;
    }
    iso->index++;

    DRV_CAN_WRITE(spi,msgdata,TRANSMIT,CAN_DLC_64);
}

void iso_tp_send_consecutive_frame(spi_device_handle_t* spi, iso_tp_control_t* iso, uint8_t* data, uint32_t data_len) {
    uint8_t msgdata[MAX_DATA_BYTES] = {0};
    uint8_t nil = 0;
    switch (iso->pci) {
        case PCI_12: {
            memset(&msgdata,N_PCI_CF,sizeof(uint8_t));
            memcpy(msgdata+1,&data_len, sizeof(uint8_t));
            memcpy(msgdata+2, data+iso->index*data_len, MAX_DATA_BYTES-2);
        }
        case PCI_32: {
            memset(msgdata,N_PCI_CF,sizeof(uint8_t));
            memset(msgdata+1,nil,sizeof(uint8_t));
            memcpy(msgdata+2,&data_len,sizeof(uint32_t));
            memcpy(msgdata+6,data+iso->index*data_len,MAX_DATA_BYTES-6);
        }
    }
    if (data_len < (MAX_DATA_BYTES - iso->pci)) {
        size_t padding_length = MAX_DATA_BYTES - data_len - iso->pci;
        uint8_t* padding_bytes = (uint8_t*) calloc(padding_length, sizeof(uint8_t));
        memcpy(msgdata+data_len, padding_bytes, padding_length);
    }
    iso->index++;
    DRV_CAN_WRITE(spi,msgdata,TRANSMIT,CAN_DLC_64);
}

bool iso_tp_recv_flow(spi_device_handle_t* spi, iso_tp_control_t* iso) {
    time_t start_time;
    time_t current_time;
    time(&start_time);
    bool recv = false;
    bool continue_loop = true;
    CAN_RX_MSGOBJ msgObj;
    uint8_t rxd[MAX_DATA_BYTES] = {0};
    while (!recv && continue_loop) {
        DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);
        if (msgObj.bF.id.SID == TRANSMIT_FLOW) {
            if (rxd[0] == N_PCI_FC) {
                iso->flow = (FLOW_CONTROL_FLAG) rxd[1] & 0xF0;
                iso->block_size = rxd[2];
                iso->sep_time = rxd[3];
                recv = true;
            } else {
                /* ERROR HANDLING */
            }
        }
        time(&current_time);
        if ((current_time-start_time)>5) {
            continue_loop = false;
        }
    }
    return continue_loop;
}

bool iso_tp_send(spi_device_handle_t* spi, uint8_t* data, uint32_t data_len) {

    iso_tp_control_t iso = {CONTINUE,1,0,0,0,PCI_12};

    if (data_len < MAX_DATA_BYTES - 2) {
        iso_tp_send_single_frame(spi,data,(uint8_t) data_len);
    } else {
        iso_tp_send_first_frame(spi,&iso,data,data_len);
        
        iso_tp_recv_flow(spi,&iso);

        while (iso.index < iso.num_segments) {        
            if (iso.flow == CONTINUE) {
                usleep(iso.sep_time);
                uint8_t blk_cnt = 0;
                if (blk_cnt < iso.block_size) {              
                    // Calculate segment length
                    uint32_t segment_length = data_len - iso.index * (MAX_DATA_BYTES-iso.pci);
                    if (segment_length > MAX_DATA_BYTES-iso.pci) {
                        segment_length = MAX_DATA_BYTES-iso.pci;
                    }
                    iso_tp_send_consecutive_frame(spi,&iso,data,segment_length);
                } else {
                    iso_tp_recv_flow(spi,&iso);
                }                              
            } else if (iso.flow == WAIT) {
                if (!iso_tp_recv_flow(spi,&iso)) {
                    return false;
                }
            }
            else if (iso.flow == ABORT) {               
                iso.index = iso.num_segments;
                return false;
            }  
        }  
    }                        
    return true;
}

void iso_tp_send_flow(spi_device_handle_t* spi, iso_tp_control_t* iso) {
    uint8_t msgdata[MAX_DATA_BYTES] = {0};
    memset(msgdata,N_PCI_FC,sizeof(uint8_t));
    memcpy(msgdata+1,&iso->flow,sizeof(uint8_t));
    memcpy(msgdata+2,&iso->block_size,sizeof(uint8_t));
    memcpy(msgdata+3,&iso->sep_time,sizeof(uint8_t));
    DRV_CAN_WRITE(spi,msgdata,TRANSMIT_FLOW,CAN_DLC_64);
}

uint8_t* iso_tp_receive(spi_device_handle_t* spi) {
    iso_tp_control_t iso = {CONTINUE,1,0,0,0,PCI_12};

    uint8_t* rxd = (uint8_t*) malloc(MAX_DATA_BYTES * sizeof(uint8_t));
    CAN_RX_MSGOBJ msgObj;
    DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);

    if (msgObj.bF.id.SID == RECEIVE_FRAME) {
        if (rxd[0] == N_PCI_SF) {
            uint8_t data_len = rxd[1];
            uint8_t* data = (uint8_t*) malloc(data_len * sizeof(uint8_t));
            memcpy(data,rxd+2,data_len);
            return data;
        } else if (rxd[0] == N_PCI_FF) {
            iso.pci = PCI_32;
            uint32_t data_len = 0;
            memcpy(&data_len,rxd+2,sizeof(uint32_t));
            uint8_t* data = (uint8_t*) malloc(data_len * sizeof(uint8_t));
            memcpy(data,rxd+6,MAX_DATA_BYTES-6);
            iso.index++;
            iso.num_segments = data_len / (MAX_DATA_BYTES-iso.pci);
            if (data_len % (MAX_DATA_BYTES-iso.pci) != 0) {
                iso.num_segments++;
            }
            iso_tp_send_flow(spi,&iso);
            while (iso.index < iso.num_segments) {
                int check_rx_register = DRV_CAN_READ_OBJ(spi,&rxd,&msgObj);
                if (!check_rx_register) {
                    if (msgObj.bF.id.SID == RECEIVE_FRAME) {
                        if (rxd[0] == N_PCI_CF) {
                            uint32_t segment_length = data_len - iso.index * (MAX_DATA_BYTES-iso.pci);
                            if (segment_length > MAX_DATA_BYTES-iso.pci) {
                                segment_length = MAX_DATA_BYTES-iso.pci;
                            }
                            memcpy(data+iso.index*(MAX_DATA_BYTES-iso.pci),rxd+2,segment_length);
                            iso.index++;                        
                        }
                    }
                } else if (check_rx_register == -1) {
                    iso.flow = WAIT;
                    iso_tp_send_flow(spi,&iso);
                    bool continue_rx = false;
                    while (!continue_rx) {
                        usleep(iso.sep_time);
                        CAN_RX_FIFO_EVENT rxFlags;
                        DRV_CANFDSPI_ReceiveChannelEventGet(spi, CAN_FIFO_CH2, &rxFlags);
                        if (~rxFlags & CAN_RX_FIFO_OVERFLOW_EVENT) {
                            iso.flow = CONTINUE;
                            iso_tp_send_flow(spi,&iso);
                            continue_rx = true;
                        }
                    }                    
                }
                {
                    iso.flow = ABORT;
                    iso_tp_send_flow(spi,&iso);
                    return NULL;
                }
                
            }
            return data;
        }
    }
    return NULL; 
}