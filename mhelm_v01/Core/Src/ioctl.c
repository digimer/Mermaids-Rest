/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ioctl.c
  * @brief          : I/O Control Setup
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

#include "main.h"
#include "ioctl.h"
#include "config.h"

extern ADC_HandleTypeDef hadc;

struct ioctl_state {
  uint16_t adc_raw_vals[ADC_NUM_CHANS];
  uint16_t pot_vals[ADC_NUM_CHANS];
};

struct ioctl_state ios;

void ioctl_init(void) {
  int i;
  for(i = 0; i < ADC_NUM_CHANS; i++) {
    ios.pot_vals[i] = 0;
  }
  HAL_ADC_Start_DMA(&hadc, (uint32_t *)ios.adc_raw_vals, ADC_NUM_CHANS);
}

// run the timer task
void ioctl_timer_task(void) {
    // restart ADC
    if(HAL_ADC_GetState(&hadc) == HAL_ADC_STATE_READY) {
        HAL_ADC_Start_DMA(&hadc, (uint32_t *)ios.adc_raw_vals, ADC_NUM_CHANS);
    }
}

// Get the requested pot value
uint16_t ioctl_get_pot(uint8_t chan) {
  if (chan >= IOCTL_NUM_POTS) {
    return 0;
  }
  return ios.pot_vals[chan];
}

//
// Callbacks
//
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  int i;
  for(i = 0; i < ADC_NUM_CHANS; i++) {
    ios.pot_vals[i] = ios.adc_raw_vals[i];
  }
}


