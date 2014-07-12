/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_DRIVE_H
#define MOD_DRIVE_H

#include "FreeRTOS.h"
#include "queue.h"

#include "stm32f10x.h"

typedef struct {
    uint8_t operation;
    uint8_t speed_0;
    uint8_t speed_1;
} driveControlData_t;


void vDrive_Console(void);
void vDrive_Control(void *pvArg);


extern xQueueHandle xQueueDriveControlCmd;
#endif
