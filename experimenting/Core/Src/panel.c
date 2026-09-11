/*
 * Panel Interface Code
 *
 * Copyright 2024: Digital Mermaid
 * Written by: digimer
 *
 */
#include "panel.h"
#include "main.h"
#include "config.h"
#include "stm32f0xx_hal.h"
#include <inttypes.h>

// hardware handles
extern SPI_HandleTypeDef hspi1;
uint8_t panel_rx_buf[2];

// init the panel
void panel_init(void) {
    int i;
    for(i = 0; i < PANEL_SPI_BUFLEN; i ++) {
        panstate.spi_tx_buf[i] = 0x00;
        panstate.spi_rx_buf[i] = 0xff;
    }
}

// run the panel timer task
void panel_timer_task(void) {
    static int count = 0;
    
    panstate.spi_tx_buf[1] = count >> 8;
    //panstate.spi_tx_buf[0] = count >> 6;
    //panstate.spi_tx_buf[1] = count >> 6;
    //  tx_buf[1] = 0x44;
/*
    int i;
    for(i = 0; i < PANEL_SPI_BUFLEN; i ++) {
        panstate.spi_tx_buf[i] = 0x02;
        panstate.spi_rx_buf[i] = 0x00;
    }*/
    count ++;

    // XXX debug
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    // start the SPI transfer
    if(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        return;
    }
    HAL_GPIO_WritePin(PANEL_CS_GPIO_Port, PANEL_CS_Pin, 1);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)panstate.spi_tx_buf,
        (uint8_t *)panstate.spi_rx_buf, PANEL_SPI_BUFLEN);
//    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)panstate.spi_tx_buf,
//        (uint8_t *)panel_rx_buf, PANEL_SPI_BUFLEN);
}

//
// callbacks
//
// handle TX/RX transfer complete
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if(hspi == &hspi1) {
        HAL_GPIO_WritePin(PANEL_CS_GPIO_Port, PANEL_CS_Pin, 0);
    }
}