/*
    FreeRTOS V7.2.0 - Copyright (C) 2012 Real Time Engineers Ltd.

 */


#ifndef MOD_RADIO_CONTROL_H
#define MOD_RADIO_CONTROL_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define RADIO_FRAME_SIZE              16U
#define RADIO_PAYLOAD_SIZE            10U

typedef union {
    uint8_t radioRxFrameBuffer[RADIO_FRAME_SIZE];
    struct {
        uint8_t frameType;
        uint8_t senderAddr;
        uint8_t radioFct;
        uint8_t radioTargetAddr;
        uint8_t payloadBytesNb;
        
        union {
            uint8_t function;
            struct {
                uint8_t data[RADIO_PAYLOAD_SIZE];
            } plainData;
            struct {
                uint8_t function;
                uint8_t operation;
                uint8_t speed_0;
                uint8_t speed_1;
            } driveData;
            struct {
                uint8_t function;
                uint8_t lightingType;
                uint8_t lightingState;
            } lightingData;
            struct {
                uint8_t function;
                uint8_t soundSignalStatus;
            } soundSignalData;
        } payload;
        
        uint8_t chaecksum;
    } operations;
} radioData_t;


void vRadio_Control(void *pvArg);

extern radioData_t radioData;
extern xSemaphoreHandle xSemaphRadioPacketReady;

#endif
