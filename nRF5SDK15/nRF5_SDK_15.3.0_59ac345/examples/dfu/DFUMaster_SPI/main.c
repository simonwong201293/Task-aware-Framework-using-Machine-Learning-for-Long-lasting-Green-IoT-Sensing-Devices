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
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_serial_dfu.h"
#include "nrf_drv_spi.h"
#define MTU_SIZE 129 //as defined in serial bootloader
#define MAX_ACTUAL_PAYLOAD (MTU_SIZE-2) //(MTU_SIZE/2-2)
#define MAX_OBJECT_SIZE 4096 //as defined in serial bootloader
#define SPI_INSTANCE  0
#define APPLICATION 0
#define BOOTLOADER_SOFTDEVICE 1

//*********** Location of the images ***********
#define APPLICATION_INIT_PACKET_LOCATION 0x9000
#define APPLICATION_IMAGE_LOCATION 0x10000
#define APPLICATION_MAX_SIZE 0x20000
#define BOOTLOADER_SOFTDEVICE_INIT_PACKET_LOCATION 0x30000
#define BOOTLOADER_SOFDEVICE_IMAGE_LOCATION 0x31000
#define BOOTLOADER_SOFTDEVICE_MAX_SIZE 0x40000
//*******************************************
#define COMMAND_OBJECT              0x01 //For init packet 
#define DATA_OBJECT                 0x02 // For actual image packet

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

uint32_t find_image_size(uint32_t start_address, uint32_t end_address);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static uint8_t       m_rx_buf[200];    /**< RX buffer. */

static void spi_response_polling(uint8_t *data, uint8_t length, uint32_t timeout_ms);
/** @brief Function for main application entry.
 */
// Simple algorithm for finding the end of the firmware image in the flash
// Assumes that there is nothing stored in the flash after the image, otherwise it will fail
//Assuming the last bytes of the image is not 0xFF. 
//To make sure this always works, better to write the exact size of the image to flash
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

//static uint8_t encoded_slip_packet[ MTU_SIZE ] = {0};
static uint8_t payload[ MAX_ACTUAL_PAYLOAD ] = {0};

static void spi_send_object(uint8_t type, uint8_t *data, uint32_t length)
{
    //Create Object
    payload[0]=0x01;
    payload[1]=type;//Command or Data Object
    payload[2]=length;
    payload[3]=length>>8;
    payload[4]=length>>16;
    payload[5]=length>>24;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, payload, 6, NULL, 0));
    spi_response_polling(m_rx_buf,3,5);
    while(length>0)
    {   
        payload[0]=0x08;//Write object opcode
     
        if (length>=MAX_ACTUAL_PAYLOAD)
        {
            memcpy(&payload[1],data,MAX_ACTUAL_PAYLOAD);
            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, payload, MAX_ACTUAL_PAYLOAD+1, NULL, 0));
            length= length-MAX_ACTUAL_PAYLOAD;
            data = data+MAX_ACTUAL_PAYLOAD;
        }else
        {
            memcpy(&payload[1],data,length);
            APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, payload, length+1, NULL, 0));
            length=0;
        }
          nrf_delay_ms(3);
    }
    //Ask for CRC
     payload[0]=0x03;
     APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, payload, 1, NULL, 0));
     spi_response_polling(m_rx_buf,11,5);
    //To-DO: check CRC
    //Execute init packet
     payload[0]=0x04;
     APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, payload, 1, NULL, 0));
    //Wait for the packet be written in flash on peer.
     nrf_delay_ms(10);
     spi_response_polling(m_rx_buf,3,5000);
   
}

static void spi_send_ping_packet( void )
{
  
    static uint8_t init_data[10]={0x09,0x01};
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, init_data, 2, NULL, 0));
    spi_response_polling(m_rx_buf,4,10);
    //nrf_delay_ms(2);
    //Get response
   // APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf, 4));
}
/** @brief spi_response_polling
* Polling every 1ms until timeout_ms with the expected [length] of the response packet. 
* Break after receiving response packet (start with 0x60)
*/
static void spi_response_polling(uint8_t *data, uint8_t length, uint32_t timeout_ms)
{
    for (uint32_t i=0;i<timeout_ms;i++)
    {
        nrf_delay_ms(1);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, data, length));
        if (data[0]==0x60) break;
    }
    nrf_delay_ms(1);
}
        

void start_DFU(uint8_t image_type)
{
    uint32_t data_size;
    uint32_t image_size;
    uint32_t image_address ;
    uint32_t init_packet_address; 
    uint8_t *data;
    nrf_gpio_pin_clear(LED_1);
    
    
    if (image_type == APPLICATION)
    { 
        init_packet_address=APPLICATION_INIT_PACKET_LOCATION;
        image_address= APPLICATION_IMAGE_LOCATION;
        image_size = find_image_size(image_address,image_address+APPLICATION_MAX_SIZE);
        
    }else if (image_type == BOOTLOADER_SOFTDEVICE)
    {    
         init_packet_address=BOOTLOADER_SOFTDEVICE_INIT_PACKET_LOCATION;
         image_address = BOOTLOADER_SOFDEVICE_IMAGE_LOCATION;
         image_size = find_image_size(image_address,image_address+BOOTLOADER_SOFTDEVICE_MAX_SIZE);
    }
    
    //Find the init packet size
    data_size = find_image_size(init_packet_address,init_packet_address+0xFFF);
    NRF_LOG_INFO("Init data size: %d", data_size);
    data= (uint8_t *) init_packet_address;
    spi_send_object(COMMAND_OBJECT,data,data_size);
    NRF_LOG_INFO("Image size: %d", image_size);
    while(image_size>0)
    {  
        if ( image_size>=MAX_OBJECT_SIZE)
        {
             
            spi_send_object(DATA_OBJECT, (uint8_t *) image_address,MAX_OBJECT_SIZE);
            image_size = image_size- MAX_OBJECT_SIZE;
            image_address= image_address + MAX_OBJECT_SIZE;
        }else
        {
             spi_send_object(DATA_OBJECT,(uint8_t *) image_address,image_size);
             image_size = 0;
        }
    }  
    nrf_gpio_pin_set(LED_1);
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}
void spi_dfu_transport_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
}
void led_button_cfg(void)
{
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_clear(LED_2);
    nrf_gpio_cfg_input(BUTTON_1,NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(BUTTON_2,NRF_GPIO_PIN_PULLUP);
}

int main(void)
{
    // This function contains workaround for PAN_028 rev2.0A anomalies 28, 29,30 and 31.
    int32_t volatile temp;
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(" DFU MASTER STARTED");
    spi_dfu_transport_init();
    nrf_delay_ms(5);
    led_button_cfg();
    spi_send_ping_packet();
    while(1)
    {
        
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {   
            NRF_LOG_INFO(" APPLICATION DFU STARTING");
            start_DFU(APPLICATION);
            NRF_LOG_INFO(" APPLICATION DFU FINISHED");
        }else if (nrf_gpio_pin_read(BUTTON_2) == 0)
        {
            NRF_LOG_INFO(" BOOTLOADER/SOFTDEVICE DFU STARTING");
            start_DFU(BOOTLOADER_SOFTDEVICE);
            NRF_LOG_INFO(" BOOTLOADER/SOFTDEVICE DFU FINISHED");
        }
        
        
    }
}


/** @} */
