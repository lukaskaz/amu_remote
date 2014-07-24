/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_lighting.c
** Descriptions:            Interface for lighting control
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

#include "mod_lighting.h"
#include "stm32f10x.h"

typedef enum {
    LIGHT_OP_NONE = '0',
    LIGHT_OP_LEFT,
    LIGHT_OP_RIGHT,
    LIGHT_OP_INNER,
    LIGHT_OP_OUTER,
} LightOperation_t;

typedef enum {
    LIGHT_RF_NONE = 0,
    LIGHT_RF_LEFT,
    LIGHT_RF_RIGHT,
    LIGHT_RF_INNER,
    LIGHT_RF_OUTER,
} LightRFOperation_t;

typedef enum {
    LIGHT_RF_DISABLE = 0,
    LIGHT_RF_ENABLE,
} LightState_t;


static void vLighting_Control(const uint8_t state);

void vLighting_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

    /* SWJ Disabled (JTAG-DP + SW-DP) and I/O Pins enabled instead */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
}

void vLighting_Console(void)
{
    FILE * pFile;
    char selection = 0;

    printf("\n\r"
           "--------------------------\n\r"
           "|    SOUND SIGNAL MENU   |\n\r"
           "--------------------------\n\r"
           "| 1. switch left led     |\n\r"
           "| 2. switch right led    |\n\r"
           "| 3. switch inner leds   |\n\r"
           "| 4. switch outer leds   |\n\r"
           "|                        |\n\r"
           "| 0. back to main menu   |\n\r"
           "--------------------------\n\r"
           " Selection/> ");

    do {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        selection = getc(pFile);

        vLighting_Control(selection);

    } while(selection != '0');
}

static void vLighting_Control(const uint8_t state)
{
    switch(state) {
        case LIGHT_OP_LEFT:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
            }
            break;
        case LIGHT_OP_RIGHT:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_5);
            }
            break;
        case LIGHT_OP_INNER:
            if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3) == Bit_SET) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
            }
            break;
        case LIGHT_OP_OUTER:
            if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) == Bit_SET) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            }
            else {
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
            }
            break;
        default:
            printf("Lighting operation not supported!\n\r");
    }
}

void vLighting_RF_Control(const uint8_t type, const uint8_t state)
{
    //printf("Lighting %d/%d/r/n", type, state);
    switch(type) {
        case LIGHT_RF_LEFT:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_4);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_4);
            }
            break;
        case LIGHT_RF_RIGHT:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_5);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_5);
            }
            break;
        case LIGHT_RF_INNER:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            }
            else {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
            }
            break;
        case LIGHT_RF_OUTER:
            if(state == LIGHT_RF_ENABLE) {
                GPIO_ResetBits(GPIOA, GPIO_Pin_15);
            }
            else {
                GPIO_SetBits(GPIOA, GPIO_Pin_15);
            }
            break;
        default:
            printf("Lighting operation not supported!\n\r");
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
