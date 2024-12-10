/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for TCPWM 6 channel to generate
*              complementary PWM - center aligned with dead time.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CPU. It configures Configure TCPWM 6 channel
* to generate complementary PWM - center aligned with dead time.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /*Initialize and start PWM_U*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(PWM_U_HW, PWM_U_NUM, &PWM_U_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM_U */
    Cy_TCPWM_PWM_Enable(PWM_U_HW, PWM_U_NUM);

    /*Initialize and start PWM_V*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(PWM_V_HW, PWM_V_NUM, &PWM_V_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM_V */
    Cy_TCPWM_PWM_Enable(PWM_V_HW, PWM_V_NUM);

    /*Initialize and start PWM_W*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(PWM_W_HW, PWM_W_NUM, &PWM_W_config))
    {
        CY_ASSERT(0);
    }

    /* Enable the initialized PWM_W */
    Cy_TCPWM_PWM_Enable(PWM_W_HW, PWM_W_NUM);

    /*Initialize the timer, Configure as Trigger signal*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(PWM_Trigger_HW, PWM_Trigger_NUM, &PWM_Trigger_config))
    {
        CY_ASSERT(0);
    }
    Cy_TCPWM_Counter_Enable(PWM_Trigger_HW, PWM_Trigger_NUM);

    /*SWAP event: constant 1*/
     Cy_TCPWM_InputTriggerSetup(PWM_U_HW,PWM_U_NUM,CY_TCPWM_INPUT_TR_INDEX_OR_SWAP ,CY_TCPWM_INPUT_LEVEL,CY_TCPWM_INPUT_1);
     Cy_TCPWM_InputTriggerSetup(PWM_V_HW,PWM_V_NUM,CY_TCPWM_INPUT_TR_INDEX_OR_SWAP ,CY_TCPWM_INPUT_LEVEL,CY_TCPWM_INPUT_1);
     Cy_TCPWM_InputTriggerSetup(PWM_W_HW,PWM_W_NUM,CY_TCPWM_INPUT_TR_INDEX_OR_SWAP ,CY_TCPWM_INPUT_LEVEL,CY_TCPWM_INPUT_1);

    /* Enable global interrupts */
    __enable_irq();

    /*Start the trigger timer*/
    Cy_TCPWM_TriggerStart_Single(PWM_Trigger_HW, PWM_Trigger_NUM);


    for (;;)
    {
    }
}

/* [] END OF FILE */
