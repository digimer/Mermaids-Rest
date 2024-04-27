/**
  ******************************************************************************
  * @file           : ioctl.h
  * @brief          : Header for ioctl.c file.
  *                   This file contains I/O stuff
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 The Digital Mermaid.
  * All rights reserved.
  *
  * This software is licensed under the GPL v3, the terms can be found in the 
  * LICENSE file in the root directory of this software component.
  *
  ******************************************************************************
  */

#include <inttypes.h>

enum {
  IOCTL_THROTTLE_STBD_POT = 0, 
  IOCTL_REGEN_POT, 
  IOCTL_THROTTLE_PORT_POT, 
  IOCTL_NUM_POTS
};

void ioctl_init(void);

// run the timer task
void ioctl_timer_task(void);

// Get the requested pot value
uint16_t ioctl_get_pot(uint8_t chan);
