/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_SENSORS_H
#define MOD_SENSORS_H

#include <stdint.h>
#include "stm32f10x.h"

typedef enum {
    SENSOR_NONE = 0,
    SENSOR_FRONT,
    SENSOR_REAR,
} SensorType_t;


extern void vSensorsServiceTask(void * pvArg);
extern void vCheckSupplyVoltage(FlagStatus PVDO);
extern uint8_t PWR_PVDLevelGet(void);

extern double distance[];
extern SensorType_t sensorInUse;

#endif
