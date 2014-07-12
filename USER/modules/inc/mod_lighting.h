/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_LIGHTING_H
#define MOD_LIGHTING_H

#include "stm32f10x.h"

extern void vLighting_Configuration(void);
extern void vLighting_Console(void);
extern void vLighting_RF_Control(const uint8_t type, const uint8_t state);

#endif
