/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_sensors.c
** Descriptions:            Sensors service for various measurements
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

#include "mod_sensors.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "mod_orientation_sensor.h"
#include "mod_lcd.h"
#include "mod_sound_signal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


#define ADC1_DR_Address    ((uint32_t)0x4001244C)

void vAnalogSensors_configuration(void);
void vDistSensors_configuration(void);
void vVoltageDetector_configuration(void);


uint16_t ADC_ConvertedData[8] = {0};
double SensorsMeasurements[2] = {0};
double distance[2]            = {0};
SensorType_t sensorInUse      = SENSOR_NONE;

double get_internal_temp(void)
{
    uint8_t i = 0;
    uint16_t adcAvgVal = 0;
    double voltVal = 0, tempVal = 0;
    
    for(i=4; i<8; i++) {
        adcAvgVal += ADC_ConvertedData[i];
    }
    adcAvgVal >>= 1;
    
    voltVal = (3.3 * adcAvgVal) / 8191U;
    tempVal = 25 + (1.41 - voltVal)/0.0043;

    return tempVal;
}

double get_illumination(void)
{
    uint8_t i = 0;
    uint16_t adcAvgVal = 0;
    double voltVal = 0, lightVal = 0;
    
    for(i=0; i<4; i++) {
        adcAvgVal += ADC_ConvertedData[i];
    }
    adcAvgVal >>= 1;
    
    voltVal = (3.3 * (8191U - adcAvgVal)) / 8191U;
    lightVal = 34.5 * voltVal - 10U;

    return lightVal;
}

void trigger_front_sensor(void)
{
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);

    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    vTaskDelay(1);  
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    TIM_SetCounter(TIM4, 0);

    distMeasReady = false;
    sensorInUse = SENSOR_FRONT;
    vTaskDelay(50);
    if(distMeasReady == false) {
        distance[0] = 0;  // no obstacle detected, set to 0
    }
    sensorInUse = SENSOR_NONE;
}

void trigger_rear_sensor(void)
{
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);

    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    vTaskDelay(1);  
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    TIM_SetCounter(TIM4, 0);

    distMeasReady = false;  
    sensorInUse = SENSOR_REAR;
    vTaskDelay(20);
    if(distMeasReady == false) {
        distance[1] = 0;  // no obstacle detected, set to 0
    }
    sensorInUse = SENSOR_NONE;
}

bool sensorCollisionDisable = false;
void vSensorCollisionCallback(xTimerHandle pxTimer)
{
    sensorCollisionDisable = true;
}

/*******************************************************************************
* Function Name  : vSensorsServiceTask
* Description    : Sensors service routine
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vSensorsServiceTask(void * pvArg)
{
    xTimerHandle xSensorCollisionTimer;
    Vector_t gyroData = {0};
    AcclData_t acclData = {0};
    lcdControlData_t lcdData = {0};
    extern volatile uint32_t power_pvd;
    uint32_t timer = 0;

    vAnalogSensors_configuration();
    vDistSensors_configuration();
    vVoltageDetector_configuration();
    vOrientation_sensor_configuration();

    xSensorCollisionTimer = xTimerCreate((signed char *)"Sensor collision timer", 2000, pdFALSE, (void *)2, vSensorCollisionCallback);

    while(1)
    {
        gyro_get_data(&gyroData);
        accl_get_data(&acclData);

        if(acclData.event == ACCL_EVENT_TAP) {
            lcdData.operation = LCD_OP_COLLISION;
            lcdData.state = acclData.event;

            xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
            vSound_Signal_RF_Control(SOUND_RF_PLAIN);
            xTimerStart(xSensorCollisionTimer, 0);
        }
        else if(sensorCollisionDisable) {
            sensorCollisionDisable = false;
            lcdData.operation = LCD_OP_COLLISION;
            lcdData.state = acclData.event;
            vSound_Signal_RF_Control(SOUND_RF_NONE);

            xQueueSend(xQueueLcdControl, (void *)&lcdData, 0);
        }

        timer++;
        printf("Nb: %d, power PVD: %d\r\n", timer, power_pvd);
        //printf("Accl dev: %.2f, %.2f, %.2f, %d, %d, %f, %f\r\n", 
        //    acclData.vect.x, acclData.vect.y, acclData.vect.z, acclData.event, 
        //        ADXL_getAxesTapDetection(), ADXL_getTapThreshold(), ADXL_getTapDuration());
        //printf("Gyro dev: %.2f, %.2f, %.2f\r\n", gyroData.x, gyroData.y, gyroData.z);
        //printf("ADC1: %.2f, %.2f\n\r", get_internal_temp(), get_illumination());
        //printf("Dist: %.2fcm, %.2fcm*/\n\r", distance[0], distance[1]);
        //trigger_front_sensor();
        //trigger_rear_sensor();
   
        vTaskDelay(100);
    }
}

void vVoltageDetector_configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    
    EXTI_ClearITPendingBit(EXTI_Line16);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    PWR_PVDLevelConfig(PWR_PVDLevel_2V2);
    PWR_PVDCmd(ENABLE);
}

void vAnalogSensors_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_ClocksTypeDef  RCC_Clocks;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
    /* Configure PA.04 (ADC Channel_4) as analog input */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_GetClocksFreq(&RCC_Clocks); 
    /* Time base configuration */ 
    TIM_TimeBaseStructure.TIM_Period        = 65000U - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/1000000UL - 1;  //for 1MHz frequency
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    /* Mode configuration: Channel1 */ 
    TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_Timing; 
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Disable; 
    TIM_OCInitStructure.TIM_Pulse           = 50000;
    TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    
    TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 8;
    ADC_Init(ADC1, &ADC_InitStructure);  

    /* setup sequencer order and sampling time for specific tasks */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  2, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  3, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  4, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 5,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 6,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 7,  ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 8,  ADC_SampleTime_239Cycles5);
    /* activate ADC module and calibrate it */
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);
    
    /* Enable DMA controller for storing data in a defined memory space for consecutive measurememts */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable internal temperature sensor for measuring ambient temperature */
    ADC_TempSensorVrefintCmd(ENABLE);
    
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    /* Enable analog data processing through determined sequencer */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 8;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    //DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);  
    /* Enable DMA Channel1 for ADC */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    

}

void vDistSensors_configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
    //TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    RCC_ClocksTypeDef  RCC_Clocks;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* Sensor_1 */
    /* TRIGGER configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ECHO configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* Sensor_2 */
    /* TRIGGER configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ECHO configuration */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    RCC_GetClocksFreq(&RCC_Clocks); 
    /* Time base configuration */ 
    TIM_TimeBaseStructure.TIM_Period        = 65000U - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler     = RCC_Clocks.HCLK_Frequency/1000000UL - 1;  //for 1MHz frequency
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
//    TIM_ICInitStructure.TIM_Channel         = TIM_Channel_2;
//    TIM_ICInitStructure.TIM_ICPolarity      = TIM_ICPolarity_Rising;
//    TIM_ICInitStructure.TIM_ICSelection     = TIM_ICSelection_DirectTI;
//    TIM_ICInitStructure.TIM_ICPrescaler     = TIM_ICPSC_DIV1;
//    TIM_ICInitStructure.TIM_ICFilter        = 0x0; 
//    TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
  
    TIM_ICInitStructure.TIM_Channel         = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity      = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection     = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler     = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter        = 0x0;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Gated);
    //TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    
    TIM_Cmd(TIM4, ENABLE); 
    //TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
    //TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM4, TIM_IT_Trigger, ENABLE);   
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
