/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for using the Imagimob generated
* model.c/h files for Machine Learning operations.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "model.h"
#include "shields.h"
#include "utils.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* IMU read timer is configured to sample at (50 Hz). */
/* sample_rate = TIMER_CLOCK_HZ / (TIMER_PERIOD + 1) */
#define IMU_READ_TIMER_CLOCK_HZ         (1000000u)
#define IMU_READ_TIMER_PERIOD           (19999u)

/* The data comes back as int16s with a scale of +/- 8g */
/* The model we are using expects floating point values where 1g = +/- 1.0 */
/* So divide raw value by 0x1000 to convert */
#define IMU_MODEL_CONVERSION_FACTOR     (0x1000u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Timer object used for blinking the LED */
static cyhal_timer_t imu_read_timer;

static mtb_imu_t motion_sensor;

static const char* LABELS[IMAI_DATA_OUT_COUNT] = IMAI_SYMBOL_MAP;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

static void imu_read_handler(void *callback_arg, cyhal_timer_event_t event);
static void timer_init(void);
static void clear_screen(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It sets up a timer to trigger a periodic interrupt
* that is used to read data from an Accelerometer (IMU) and store the sample in
* a queue. The main loop checks how much data is in the queue and when there is
* a large enough quantity will extract it and perform an inference operation.
* The user LED will be turned on while inferencing is happening. When the
* inference operation is complete, the confidence of each activity is printed
* via the UART.
*
* Parameters:
*   none
*
* Return:
*   Status of the operation. Should never return.
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    float classifications[IMAI_DATA_OUT_COUNT];

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    MTB_HALT_IF_ERROR(result, "Unable to initialize the BSP configuration.\n");

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    MTB_HALT_IF_ERROR(result, "Unable to initialize the retarget-io.\n");

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    MTB_HALT_IF_ERROR(result, "Unable to initialize the user LED.\n");

    /* Clear screen for clean printing */
    clear_screen();

    /* Initialize the IMU to get sensor data from */
    imu_init(&motion_sensor);

    /* Initialize timer to read from the IMU */
    timer_init();

    /* Initialize the Machine Learning model */
    IMAI_init();

    for (;;)
    {
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
        int imai_reslut = IMAI_dequeue(&classifications[0]);
        cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        switch (imai_reslut)
        {
            case IMAI_RET_SUCCESS:
                clear_screen();
                printf("Inference operation succeeded.\n");
                for (uint8_t i = 0; i < IMAI_DATA_OUT_COUNT; i++)
                {
                    printf("%-8s: %6.2f%%\n", LABELS[i], 100 * classifications[i]);
                }
                break;
            case IMAI_RET_NODATA:
                /* Sleep until more data is available */
                cyhal_syspm_sleep();
                break;
            case IMAI_RET_NOMEM:
                /* Something went wrong, stop the program */
                MTB_HALT("Unable to perform inference. Internal memory error.\n");
                break;
            default:
                /* Something went wrong, stop the program */
                MTB_HALT("Unable to perform inference. Unknown error occurred.\n");
                break;
        }
    }
    /* Should never get here */
    return 1;
}


/*******************************************************************************
* Function Name: imu_read_handler
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt. It is used to
* read the latest values from the IMU.
*
* Parameters:
*   callback_arg  Arguments passed to the interrupt callback
*   event         Timer/counter interrupt triggers
*******************************************************************************/
static void imu_read_handler(void *callback_arg, cyhal_timer_event_t event)
{
    CY_UNUSED_PARAMETER(callback_arg);
    CY_UNUSED_PARAMETER(event);

    mtb_imu_data_t imu_data;
    cy_rslt_t result = imu_read(&motion_sensor, &imu_data);
    MTB_HALT_IF_ERROR(result, "Unable to read from IMU sensor.\n");

    /* Convert raw sensor data into format expected by the ML model. */
    /* Note: The SENSE shield has the BMX-160 sensor in a different orientation */
    /* than the TFT and EPD shields; so adjust the data we store in the buffer */
    /* to make sure they look the same to the model. */
    float imu_buffer[IMAI_DATA_IN_COUNT] =
    {
#if defined(SENSE_SHIELDv1) || defined(SENSE_SHIELDv2)
        imu_data.accel.y / (float)IMU_MODEL_CONVERSION_FACTOR,
        imu_data.accel.x / (float)IMU_MODEL_CONVERSION_FACTOR,
        imu_data.accel.z / -(float)IMU_MODEL_CONVERSION_FACTOR,
#else
        imu_data.accel.x / (float)IMU_MODEL_CONVERSION_FACTOR,
        imu_data.accel.y / (float)IMU_MODEL_CONVERSION_FACTOR,
        imu_data.accel.z / (float)IMU_MODEL_CONVERSION_FACTOR,
#endif
    };

    int imai_reslut = IMAI_enqueue(imu_buffer);
    if (IMAI_RET_SUCCESS != imai_reslut)
    {
        MTB_HALT("Insufficient memory to enqueue sensor data. Inferencing is not keeping up.\n");
    }
}


/*******************************************************************************
* Function Name: imu_read_handler
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'imu_read_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 50 miliseconds.
*
* Parameters:
*   none
*******************************************************************************/
static void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t imu_read_timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = IMU_READ_TIMER_PERIOD,    /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&imu_read_timer, NC, NULL);
    MTB_HALT_IF_ERROR(result, "Unable to initialize the IMU read timer.\n");

    /* Apply the timer configuration defined above */
    result = cyhal_timer_configure(&imu_read_timer, &imu_read_timer_cfg);
    MTB_HALT_IF_ERROR(result, "Unable to configure the IMU read timer.\n");

    /* Set the frequency of timer's clock source */
    result = cyhal_timer_set_frequency(&imu_read_timer, IMU_READ_TIMER_CLOCK_HZ);
    MTB_HALT_IF_ERROR(result, "Unable to set the IMU read timer's frequency.\n");

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&imu_read_timer, imu_read_handler, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&imu_read_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                             CYHAL_ISR_PRIORITY_DEFAULT, true);

    /* Start the timer with the configured settings */
    result = cyhal_timer_start(&imu_read_timer);
    MTB_HALT_IF_ERROR(result, "Unable to start the IMU read timer.\n");
}


/*******************************************************************************
* Function Name: clear_screen
********************************************************************************
* Summary:
* Erase the console display contents so we can start a new message
*
* Parameters:
*   none
*******************************************************************************/
static void clear_screen(void)
{
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("******* Imagimob Machine Learning Deploy model.c Example *******\r\n\n");
}
/* [] END OF FILE */
