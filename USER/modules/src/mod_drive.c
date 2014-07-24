/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_drive.c
** Descriptions:            Interface for controling dc motors drive
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
#include <stdbool.h>

#include "mod_drive.h"
#include "FreeRTOS.h"
#include "task.h"


#define DRIVE_DATA_QUEUE_SIZE    4U
#define MAX_VELOCITY_VALUE       100U

typedef enum {
    DRIVE_OP_STOPPED = 0,
    DRIVE_OP_FORWARD,
    DRIVE_OP_BACKWARD,
    DRIVE_OP_LEFT,
    DRIVE_OP_RIGHT,
    DRIVE_OP_JOY_STOPPED,
    DRIVE_OP_JOY_FORWARD,
    DRIVE_OP_JOY_BACKWARD,
    DRIVE_OP_JOY_LEFT,
    DRIVE_OP_JOY_RIGHT,
} driveOperations_t;


xQueueHandle xQueueDriveControlCmd = NULL;

void vDrive_Configuration(void);

void vDrive_Configuration(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;
    RCC_ClocksTypeDef        RCC_Clocks;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TIM_TimeBaseStructure.TIM_Period        = 100U - 1U;
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/(100U*100U) - 1U;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* TIM_SIG_PWM_TIMER enable counter */ 
    TIM_Cmd(TIM3, ENABLE);
    TIM3->CCR3 = 0;
    TIM3->CCR1 = 0;
}

void vDrive_Console(void)
{
    FILE * pFile;
    char selection = 0;
    static uint32_t velocity = 0;
    driveControlData_t driveControlData = {0};

    printf("\n\r"
           "--------------------------\n\r"
           "|     MOVEMENT MENU      |\n\r"
           "--------------------------\n\r"
           "| 1. move forward        |\n\r"
           "| 2. move backward       |\n\r"
           "| 7. turn left           |\n\r"
           "| 8. move right          |\n\r"
           "| 9. set speed           |\n\r"
           "|                        |\n\r"
           "| 0. back to main menu   |\n\r"
           "--------------------------\n\r"
           " Selection/> ");

    do {
        selection = getc(pFile);
        printf("Selected %c!\n\r", selection);

        switch(selection) {
            case '1':
                driveControlData.operation = DRIVE_OP_FORWARD;
                driveControlData.speed_0 = velocity;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '2':
                driveControlData.operation = DRIVE_OP_BACKWARD;
                driveControlData.speed_0 = velocity;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '7':
                driveControlData.operation = DRIVE_OP_LEFT;
                driveControlData.speed_0 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '8':
                driveControlData.operation = DRIVE_OP_RIGHT;
                driveControlData.speed_1 = velocity;
                xQueueSend(xQueueDriveControlCmd, (void *)&driveControlData, 0);
                break;
            case '9': 
                printf("Speed: ");
                scanf("%d", (uint32_t *)&velocity);
                break;
            default:
                printf("Movement operation not supported!\n\r");
        }
    } while(selection != '0');
}

void vDrive_Control(void *pvArg)
{
    driveControlData_t drive = {0};

    vDrive_Configuration();
    xQueueDriveControlCmd = xQueueCreate(DRIVE_DATA_QUEUE_SIZE, sizeof(driveControlData_t));

    while(1) {
        // RF packets should be received between 30ms intervals (set 40ms wait for safety)
        if(xQueueReceive(xQueueDriveControlCmd, &drive, 40U) != pdTRUE)
        {
            if(drive.operation == DRIVE_OP_STOPPED) {
                xQueueReceive(xQueueDriveControlCmd, &drive, portMAX_DELAY);
            }
            else {
                drive.operation = DRIVE_OP_STOPPED;
            }
        }

        if(drive.speed_0 > MAX_VELOCITY_VALUE) {
            drive.speed_0 = MAX_VELOCITY_VALUE;
        }
        if(drive.speed_1 > MAX_VELOCITY_VALUE) {
            drive.speed_1 = MAX_VELOCITY_VALUE;
        }
                
        switch(drive.operation) {
            case DRIVE_OP_STOPPED:
            case DRIVE_OP_JOY_STOPPED:
                //printf("Motors stopped!\n\r");
                TIM3->CCR1 = 0;
                TIM3->CCR3 = 0;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_FORWARD:
            case DRIVE_OP_JOY_FORWARD:
                //printf("Move forward %d\n\r", driveOperation.speed );
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = drive.speed_1;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_BACKWARD:
            case DRIVE_OP_JOY_BACKWARD:
                //printf("Move backward %d\n\r", driveOperation.speed);
                TIM3->CCR1 = MAX_VELOCITY_VALUE - drive.speed_0;
                TIM3->CCR3 = MAX_VELOCITY_VALUE - drive.speed_1;
                GPIO_SetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_LEFT:
                //printf("Turn left %d\n\r", driveOperation.speed);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = MAX_VELOCITY_VALUE - drive.speed_0;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_SetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_RIGHT:
                //printf("Turn right %d\n\r", driveOperation.speed);
                TIM3->CCR1 = MAX_VELOCITY_VALUE - drive.speed_1;
                TIM3->CCR3 = drive.speed_1;
                GPIO_SetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            case DRIVE_OP_JOY_LEFT:
            case DRIVE_OP_JOY_RIGHT:
                //printf("Move joy TURN %d/%d\n\r", drive.speed_0, drive.speed_1);
                TIM3->CCR1 = drive.speed_0;
                TIM3->CCR3 = drive.speed_1;
                GPIO_ResetBits(GPIOA, GPIO_Pin_5);
                GPIO_ResetBits(GPIOA, GPIO_Pin_7);
                break;
            default:
                printf("Unsupported drive operation!\n\r");
        }
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
