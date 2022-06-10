/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
* @defgroup temperature_example_main main.c
* @{
* @ingroup temperature_example
* @brief Temperature Example Application main file.
* @details
* This file contains the source code for a sample application using the temperature sensor.
* This contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31. PAN 43 is not covered.
*  - PAN_028 rev2.0A anomaly 28 - TEMP: Negative measured values are not represented correctly
*  - PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register.
*  - PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs.
*  - PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module
*  - PAN_028 rev2.0A anomaly 43 - TEMP: Using PPI between DATARDY event and START task is not functional.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "bsp.h"
#include "slip.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_uart.h"
#include "nrf_serial_dfu.h"
#define MTU_SIZE 129
#define MAX_ACTUAL_PAYLOAD (MTU_SIZE/2-2)
#define MAX_OBJECT_SIZE 4096
uint32_t find_image_size(uint32_t start_address, uint32_t end_address);

/** @brief Function for main application entry.
 */
 static serial_dfu_t m_dfu;
 static void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
    uint32_t err_code;
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_TX_DONE:
    
            break;

        case NRF_DRV_UART_EVT_RX_DONE:
            //Todo: instead of using delay response code checking should be done here.
            err_code = nrf_drv_uart_rx(&m_dfu.uart_instance, &m_dfu.uart_buffer, 100);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Failed initializing rx");
            }
            break;

        case NRF_DRV_UART_EVT_ERROR:
    
            break;
    }
}

// Simple algorithm for finding the end of the firmware image in the flash
// Assumes that there is nothing stored in the flash after the image, otherwise it will fail
uint32_t find_image_size(uint32_t start_address, uint32_t end_address)
{
    uint32_t img_size = end_address - 1;
    while(img_size > start_address)
    {
        if(*((uint8_t*)img_size) != 0xFF) break;
        img_size -= 1;
    }
    img_size = img_size + 1 - start_address;   
    
    return img_size;    
}

static uint8_t encoded_slip_packet[ MTU_SIZE ] = {0};
static uint8_t payload[ MAX_ACTUAL_PAYLOAD ] = {0};
static void uart_encode_and_send(serial_dfu_t * p_dfu,uint8_t *data, uint32_t length)
{    uint32_t encoded_slip_packet_length;
    (void)slip_encode(encoded_slip_packet, data, length, &encoded_slip_packet_length);
    (void)nrf_drv_uart_tx(&p_dfu->uart_instance, encoded_slip_packet, encoded_slip_packet_length);
     while(nrf_drv_uart_tx_in_progress(&p_dfu->uart_instance));  
}
    
static void uart_send_init_packet(serial_dfu_t * p_dfu,uint8_t *init_data, uint32_t length)
{
    uint32_t i=length;
    uint32_t encoded_slip_packet_length;

    //Create Object
    payload[0]=0x01;
    payload[1]=0x01;//Command Object
    payload[2]=length;
    payload[3]=length>>8;
    payload[4]=length>>16;
    payload[5]=length>>24;
    uart_encode_and_send(p_dfu,payload,6);
    nrf_delay_ms(5);
    while(length>0)
    {   
        encoded_slip_packet[0]=0x08;//Write object opcode

        if (length>=MAX_ACTUAL_PAYLOAD)
        {
            (void)slip_encode(&encoded_slip_packet[1], init_data, MAX_ACTUAL_PAYLOAD, &encoded_slip_packet_length);
            length= length-MAX_ACTUAL_PAYLOAD;
            init_data = init_data+MAX_ACTUAL_PAYLOAD;
        }else
        {
            (void)slip_encode(&encoded_slip_packet[1], init_data, length, &encoded_slip_packet_length);
            length=0;
        }
        NRF_LOG_INFO("Data : %d %d %d", length,&init_data,MAX_ACTUAL_PAYLOAD);
        // send
        (void)nrf_drv_uart_tx(&p_dfu->uart_instance, encoded_slip_packet, encoded_slip_packet_length+1);
         while(nrf_drv_uart_tx_in_progress(&p_dfu->uart_instance));    
        
    }
    //Ask for CRC
     payload[0]=0x03;
     uart_encode_and_send(p_dfu,payload,1);
     nrf_delay_ms(5);
    //To-DO: check CRC
    //Execute init packet
     payload[0]=0x04;
     uart_encode_and_send(p_dfu,payload,1);
    
}
static void uart_send_application_image(serial_dfu_t * p_dfu,uint8_t *data, uint32_t length)
{
    uint32_t i=length;
    uint32_t encoded_slip_packet_length;

    //Create Object
    payload[0]=0x01;
    payload[1]=0x02;//Command Object
    payload[2]=length;
    payload[3]=length>>8;
    payload[4]=length>>16;
    payload[5]=length>>24;
    uart_encode_and_send(p_dfu,payload,6);
    nrf_delay_ms(5);
    while(length>0)
    {   
        encoded_slip_packet[0]=0x08;//Write object opcode

        if (length>=MAX_ACTUAL_PAYLOAD)
        {
            (void)slip_encode(&encoded_slip_packet[1], data, MAX_ACTUAL_PAYLOAD, &encoded_slip_packet_length);
            length= length-MAX_ACTUAL_PAYLOAD;
            data = data+MAX_ACTUAL_PAYLOAD;
        }else
        {
            (void)slip_encode(&encoded_slip_packet[1], data, length, &encoded_slip_packet_length);
            length=0;
        }
        NRF_LOG_INFO("Data : %d %d %d", length,&data,MAX_ACTUAL_PAYLOAD);
        // send
        (void)nrf_drv_uart_tx(&p_dfu->uart_instance, encoded_slip_packet, encoded_slip_packet_length+1);
         while(nrf_drv_uart_tx_in_progress(&p_dfu->uart_instance));    
        
    }
    //Ask for CRC
     payload[0]=0x03;
     uart_encode_and_send(p_dfu,payload,1);
     nrf_delay_ms(5);
    //To-DO: check CRC
    //Execute init packet
     payload[0]=0x04;
     uart_encode_and_send(p_dfu,payload,1);
     nrf_delay_ms(15);
}


static void uart_send_ping_packet(serial_dfu_t          * p_dfu        )
{
    uint32_t encoded_slip_packet_length;
    uint8_t init_data[4]={0x09,0x01};
    uint32_t length= 2;
    (void)slip_encode(encoded_slip_packet, init_data, length, &encoded_slip_packet_length);

  

    // send
    (void)nrf_drv_uart_tx(&p_dfu->uart_instance, encoded_slip_packet, encoded_slip_packet_length);
}


void start_DFU(void)
{
    uint32_t data_size;
    uint32_t application_image_address = 0x30000;
    nrf_gpio_pin_clear(LED_1);
    data_size = find_image_size(0x9000,0x10000);
    NRF_LOG_INFO("Init data size: %d", data_size);
    uint8_t *data;
    data= (uint8_t *) 0x9000;
    NRF_LOG_INFO("Data : %x %x %d", *data, *(data+1),MAX_ACTUAL_PAYLOAD);
    nrf_delay_ms(5);
    uart_send_init_packet(&m_dfu,(uint8_t *) 0x9000,data_size);
    nrf_delay_ms(500);
    
    data_size = find_image_size(application_image_address,application_image_address+0x40000);
    NRF_LOG_INFO("Image size: %d", data_size);
    //Max object size is 4096 bytes.
     while(data_size>0)
    {   
        if ( data_size>=MAX_OBJECT_SIZE)
        {
            
            uart_send_application_image(&m_dfu,(uint8_t *) application_image_address,MAX_OBJECT_SIZE);
            data_size = data_size- MAX_OBJECT_SIZE;
            application_image_address= application_image_address + MAX_OBJECT_SIZE;
        }else
        {
             uart_send_application_image(&m_dfu,(uint8_t *) application_image_address,data_size);
             data_size = 0;
        }
    }  
        
    nrf_delay_ms(1000);
    nrf_gpio_pin_set(LED_1);
}

uint32_t serial_dfu_transport_init(void)
{
    uint32_t err_code;

    //leds_init();

    m_dfu.slip.p_buffer      = m_dfu.recv_buffer;
    m_dfu.slip.current_index = 0;
    m_dfu.slip.buffer_len    = sizeof(m_dfu.recv_buffer);
    m_dfu.slip.state         = SLIP_STATE_DECODING;

    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;

    uart_config.pseltxd   = TX_PIN_NUMBER;
    uart_config.pselrxd   = RX_PIN_NUMBER;
    uart_config.pselcts   = CTS_PIN_NUMBER;
    uart_config.pselrts   = RTS_PIN_NUMBER;
    uart_config.hwfc      = NRF_UART_HWFC_ENABLED;
    uart_config.p_context = &m_dfu;


    nrf_drv_uart_t instance =  NRF_DRV_UART_INSTANCE(0);
    memcpy(&m_dfu.uart_instance, &instance, sizeof(instance));

    err_code =  nrf_drv_uart_init(&m_dfu.uart_instance, &uart_config, uart_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed initializing uart");
        return err_code;
    }

    nrf_drv_uart_rx_enable(&m_dfu.uart_instance);

    err_code = nrf_drv_uart_rx(&m_dfu.uart_instance, &m_dfu.uart_buffer, 100);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed initializing rx");
    }
    
    NRF_LOG_DEBUG("UART initialized");

    return err_code;
}


uint32_t serial_dfu_transport_close(void)
{
    nrf_drv_uart_uninit(&m_dfu.uart_instance);
    return NRF_SUCCESS;
}
int main(void)
{
    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t volatile temp;
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    serial_dfu_transport_init();
    nrf_delay_ms(500);
    uart_send_ping_packet(&m_dfu);
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_cfg_input(BUTTON_1,NRF_GPIO_PIN_PULLUP);
   while(1)
    {
        
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {
            start_DFU();
        }
    }
}


/** @} */
