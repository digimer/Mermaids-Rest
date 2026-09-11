/*
 * Panel Interface Code
 *
 * Copyright 2024: Digital Mermaid
 * Written by: digimer
 *
 */
#ifndef PANEL_H
#define PANEL_H

#include <inttypes.h>

// settings
#define PANEL_SPI_BUFLEN 2

// init the panel
void panel_init(void);

// run the panel timer task
void panel_timer_task(void);

typedef struct panel_state {
    uint8_t spi_tx_buf[PANEL_SPI_BUFLEN];
    uint8_t spi_rx_buf[PANEL_SPI_BUFLEN];
};

// panel state
extern struct panel_state panstate;

#endif