/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_SENSORS_H
#define MOD_SENSORS_H


typedef enum {
    SENSOR_NONE = 0,
    SENSOR_FRONT,
    SENSOR_REAR,
} SensorType_t;


void vSensorsServiceTask(void * pvArg);

extern double distance[];
extern SensorType_t sensorInUse;

#endif
