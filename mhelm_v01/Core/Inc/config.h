/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Header for config.c file.
  *                   This file contains global configs for mhelm
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

#ifndef CONFIG_H
#define CONFIG_H

#define MAJOR_VERSION 0
#define MINOR_VERSION 1
#define ADC_NUM_CHANS 2

// interrupt prios
#define INT_PRIO_SYSTICK 0
#define INT_PRIO_USB 4
#define INT_PRIO_SPI 5
#define INT_PRIO_RT_TASK 6
#define INT_PRIO_ADC 7

#endif
