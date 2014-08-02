/****************************************Copyright (c)****************************************************
**                                      
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               mod_console.c
** Descriptions:            Interface for console operations
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

#include "mod_console.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usb_lib.h"
#include "hw_config.h"

#include "mod_drive.h"
#include "mod_sound_signal.h"
#include "mod_lighting.h"

typedef enum
{
    MENU_MAIN = '0',
    MENU_MOVEMENT,
    MENU_LIGHTING,
    MENU_SOUND,
} menuLevel_t;


/*******************************************************************************
* Function Name  : vConsoleInterfaceTask
* Description    : Console operations routine
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void vConsoleInterfaceTask(void * pvArg)
{
    menuLevel_t menuLevel = MENU_MAIN, menuLevelSel = MENU_MAIN;

    while(1)
    {
        if(menuLevel != menuLevelSel || getchar() == 0x1B )
        {
            menuLevel = menuLevelSel;
            switch(menuLevel) {
                case MENU_MAIN:
                    printf("\n\r"
                           "--------------------------\n\r"
                           "|          MENU          |\n\r"
                           "--------------------------\n\r"
                           "| 1. movement control    |\n\r"
                           "| 2. lighting control    |\n\r"
                           "| 3. sound control       |\n\r"
                           "| 4. distance measures   |\n\r"
                           "| 5. lighting measure    |\n\r"
                           "| 6. temperature measure |\n\r"
                           "--------------------------\n\r"
                           " Selection/> ");

                    menuLevelSel = (menuLevel_t)getchar();
                    break;
                case MENU_MOVEMENT:
                    vDrive_Console();

                    menuLevelSel = MENU_MAIN;
                    break;
                case MENU_LIGHTING: {
                    vLighting_Console();

                    menuLevelSel = MENU_MAIN;
                    break;
                }
                case MENU_SOUND:
                    vSound_Signal_Console();

                    menuLevelSel = MENU_MAIN;
                    break;
                default:
                    printf("selected menu not supported!\n\r");
                    menuLevelSel = MENU_MAIN;
            }
        }
    }
}


void vUSB_configuration(void)
{
    Set_System();
    Set_USBClock();
    //Interrupts NVIC settings consolidated in main file
    //USB_Interrupts_Config();
    USB_Init();
}


int fputc(int ch, FILE *f)
{
    USB_Tx_Buffer[USB_Tx_ptr_in] = (uint8_t)ch;
    USB_Tx_ptr_in++;

    if(USB_Tx_ptr_in >= USB_TX_DATA_SIZE)
    {
        USB_Tx_ptr_in = 0; 
    }

    return ch;
}


int fgetc(FILE *f) {
    static uint32_t chNb = 0;
    char ch = 0;

    while(USB_Rx_Counter == 0) {
        vTaskDelay(10);
    }
    
    ch = USB_Rx_Buffer[chNb++];

    if(chNb >= USB_Rx_Counter) {
        chNb = 0;
        USB_Rx_Counter = 0;
    }

    return ch;
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
