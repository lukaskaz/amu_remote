/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_radio_control.c
** Descriptions:            Interface for RF radio operations
**
**--------------------------------------------------------------------------------------------------------
** Created by:              
** Created date:            
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#include <stdio.h>

#include "mod_radio_control.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include "mod_drive.h"
#include "mod_lighting.h"
#include "mod_sound_signal.h"
#include "mod_lcd.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

typedef enum {
    RADIO_OP_NONE = 0,
    RADIO_OP_DRIVE,
    RADIO_OP_LIGHTING,
    RADIO_OP_SOUND_SIG,
    RADIO_OP_HEARTBEAT,
} RadioOperation_t;

void vRadio_Configuration(void);

xSemaphoreHandle xSemaphRadioPacketReady = NULL;
bool startRadioRefresh = false;
radioData_t radioData = {0};


void vRadio_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    //configure GPIO pins for USART1
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //configure USART
    USART_InitStructure.USART_BaudRate   = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_WakeUpConfig(USART1, USART_WakeUp_AddressMark);
    USART_SetAddress(USART1, 0x05);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    USART_ClearFlag(USART1,USART_FLAG_RXNE);
    USART_ClearFlag(USART1,USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
}

void vRadioRefreshCallback(xTimerHandle pxTimer)
{
    startRadioRefresh = true;
}

void vRadio_Control(void *pvArg)
{
    xTimerHandle xRadioRefreshTimer;
    uint8_t lastOperation = RADIO_OP_NONE;

    vRadio_Configuration();

    xRadioRefreshTimer = xTimerCreate((signed char *)"Radio refresh timer", 50, pdFALSE, (void *)3, vRadioRefreshCallback);
    while(1) {
        lcdControlData_t lcdData = {0};
    
        if(xSemaphoreTake(xSemaphRadioPacketReady, 10/portTICK_RATE_MS) == pdTRUE) {
            xTimerStart(xRadioRefreshTimer, 0);

            if(radioData.operations.payload.function == RADIO_OP_DRIVE) {
                driveControlData_t driveControlData = {0};

                driveControlData.operation = radioData.operations.payload.driveData.operation;
                driveControlData.speed_0   = radioData.operations.payload.driveData.speed_0;
                driveControlData.speed_1   = radioData.operations.payload.driveData.speed_1;

                //printf("Radio |drive| operation: %d/%d/%d\n\r",
                    //driveControlData.operation, driveControlData.speed_0, driveControlData.speed_1);
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
            }
            else if(radioData.operations.payload.function == RADIO_OP_LIGHTING) {
                vLighting_RF_Control(radioData.operations.payload.lightingData.lightingType, radioData.operations.payload.lightingData.lightingState);
            }
            else if(radioData.operations.payload.function == RADIO_OP_SOUND_SIG) {
                // new packet should be received within 40ms (set 10ms more for safety)
                // trigger timer to disable sound signal when 50ms time is depleted and on signal is not sustained
                if(lastOperation != RADIO_OP_SOUND_SIG) {
                    vSound_Signal_RF_Control(radioData.operations.payload.soundSignalData.soundSignalStatus);
                    lcdData.operation = LCD_OP_SOUND_SIG;
                    lcdData.state = LCD_SOUND_ON;
                }
            }
            else if(radioData.operations.payload.function == RADIO_OP_HEARTBEAT) {
                // heartbeat signal for diagnosis purpuses on controller side, do nothing
            }
            else {
                // unsupported case, do nothing
            }
            
            lastOperation = radioData.operations.payload.function;
            //printf("Sig: %d, %d\r\n", radioData.operations.payload.function, radioData.operations.payload.soundSignalData.soundSignalStatus);
        }
        else if(startRadioRefresh == true) {
            startRadioRefresh = false;

            if(lastOperation == RADIO_OP_SOUND_SIG) {
                // there is no sound signal package received so suppress horn
                vSound_Signal_RF_Control(SOUND_RF_NONE);
                lcdData.operation = LCD_OP_SOUND_SIG;
                lcdData.state = LCD_SOUND_OFF;
                lastOperation = RADIO_OP_NONE;
            }
        }
        else {
            // no action defined
        }
        
        if(lcdData.operation != LCD_OP_NONE) {
            xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
        }
    }
}



/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
