// Takes a switch number, and 'on' or 'off', to control a light on the relay 
// board.
//
// Based on 'blink.c';
// Author: Mike McCauley (mikem@open.com.au)
// Copyright (C) 2011 Mike McCauley
//
// Requires https://www.airspayce.com/mikem/bcm2835/
// - Tested on the Raspberry Pi 4b
//
// Written by Madison Kelly, digital.mermaid@gmail.com
// Mar. 22, 2025
//
// v0.1

#include <stdio.h>
#include <string.h>
#include <bcm2835.h>

// Map switches to GPIO pins
#define SW1 RPI_V2_GPIO_P1_11
#define SW2 RPI_V2_GPIO_P1_13
#define SW3 RPI_V2_GPIO_P1_15
#define SW4 RPI_V2_GPIO_P1_12

// The relay's normally opened outputs are closed on LOW (and vice versa)
int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
    // Use for testing
    //bcm2835_set_debug(1);

    if (!bcm2835_init())
        return 1;
    
    // Make sure we've got a switch number.
    if (!argv[1] || !argv[2]) {
	printf("Usage: [%s <switch> <on|off>]. Switch is 1 ~ 4.\n", argv[0]);
	return 1;
    }
    char *endptr;
    long int sw = strtol(argv[1], &endptr, 10);
    if (endptr == argv[1]) {
        printf("The switch number was not specified or was not a digit.\n");
	printf("Usage: [%s <switch> <on|off>]. Switch is 1 ~ 4.\n", argv[0]);
	return 1;
    } else if (*endptr != '\0') {
        printf("The switch: [%c] must be an integer.\n", *endptr);
	printf("Usage: [%s <switch> <on|off>]. Switch is 1 ~ 4.\n", argv[0]);
	return 1;
    } else {
        printf("Working on switch: [%ld]\n", sw);
    }

    if (sw < 1 || sw > 4) {
	printf("Switch: [%ld] is invalid. Should be between 1 and 4.\n", sw);
	return 1;
    }

    // Make sure we've got a valid task.
    char *task   = argv[2];
    int  taskOn  = strcmp(task,"on");
    int  taskOff = strcmp(task,"off");
    //printf("Raw task: [%s], taskOn: [%d], taskOff: [%d]\n", task, taskOn, taskOff);
    if (taskOn == 0) {
	printf("Turning the light on.\n");
    } else if (taskOff == 0) {
	printf("Turning the light off.\n");
    } else {
        printf("The task: [%s] is invalid. It must be 'on' or 'off'.\n", argv[2]);
	printf("Usage: [%s <switch> <on|off>]. Switch is 1 ~ 4.\n", argv[0]);
	return 1;
    }

    // Set the pin to be an output
    if (sw == 1) {
        bcm2835_gpio_fsel(SW1, BCM2835_GPIO_FSEL_OUTP);
	if (taskOn == 0) {
	    bcm2835_gpio_write(SW1, LOW);
	} else {
	    bcm2835_gpio_write(SW1, HIGH);
	}
    } else if (sw == 2) {
        bcm2835_gpio_fsel(SW2, BCM2835_GPIO_FSEL_OUTP);
	if (taskOn == 0) {
	    bcm2835_gpio_write(SW2, LOW);
	} else {
	    bcm2835_gpio_write(SW2, HIGH);
	}
    } else if (sw == 3) {
        bcm2835_gpio_fsel(SW3, BCM2835_GPIO_FSEL_OUTP);
	if (taskOn == 0) {
	    bcm2835_gpio_write(SW3, LOW);
	} else {
	    bcm2835_gpio_write(SW3, HIGH);
	}
    } else if (sw == 4) {
        bcm2835_gpio_fsel(SW4, BCM2835_GPIO_FSEL_OUTP);
	if (taskOn == 0) {
	    bcm2835_gpio_write(SW4, LOW);
	} else {
	    bcm2835_gpio_write(SW4, HIGH);
	}
    }

    return 0;
}

