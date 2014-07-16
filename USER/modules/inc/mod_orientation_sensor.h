/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_ORIENT_SENSOR_H
#define MOD_ORIENT_SENSOR_H

#include <stdint.h>

typedef struct {
    double x;
    double y;
    double z;
} Vector_t;

typedef struct {
    Vector_t vect;
    uint8_t event;
} AcclData_t;

typedef enum {
    ACCL_EVENT_NONE = 0,
    ACCL_EVENT_TAP,
} AcclEvent_t;

void vOrientation_sensor_configuration(void);
void gyro_get_data(Vector_t *data);
void accl_get_data(AcclData_t *data);
uint8_t ADXL_getAxesTapDetection(void);

double ADXL_getTapDuration(void);
double ADXL_getTapThreshold(void);

#endif
