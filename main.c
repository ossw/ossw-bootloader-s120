/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
 * -# Receive start data packet. 
 * -# Based on start packet, prepare NVM area to store received data. 
 * -# Receive data packet. 
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include "dfu_transport.h"
#include "bootloader.h"
#include "bootloader_util.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "nrf51.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_error.h"
#include "bsp/bsp.h"
#include "softdevice_handler_appsh.h"
#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "nrf_delay.h"
#include "spi.h"
//#include "ext_flash.h"
#include "mlcd.h"


#if BUTTONS_NUMBER < 1
#error "Not enough buttons on board"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                                       /**< Include the service_changed characteristic. For DFU this should normally be the case. */

#define BOOTLOADER_BUTTON               BUTTON_SELECT                                            /**< Led used to indicate that DFU is active. */

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)                /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                                                      /**< Maximum number of events in the scheduler queue. */

uint32_t * p_spi0_base_address;
uint32_t * p_spi1_base_address;

static uint8_t last_progress;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer handler module (app_timer).
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);

}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 *
 * @param[in] init_softdevice  true if SoftDevice should be initialized. The SoftDevice must only 
 *                             be initialized if a chip reset has occured. Soft reset from 
 *                             application must not reinitialize the SoftDevice.
 */
static void ble_stack_init(bool init_softdevice)
{
    uint32_t         err_code;
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

    if (init_softdevice)
    {
        err_code = sd_mbr_command(&com);
        APP_ERROR_CHECK(err_code);
    }
    
    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
   
    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    ble_enable_params.gap_enable_params.role = BLE_GAP_ROLE_PERIPH;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

static void spi_init(void)
{
    p_spi0_base_address = spi_master_init(SPI0, SPI_MODE0, false);
    p_spi1_base_address = spi_master_init(SPI1, SPI_MODE0, false);
}

static uint8_t progress_bg_draw_func(uint8_t x, uint8_t y)
{
    uint32_t d = (72-x)*(72-x) + (84-y)*(84-y);
    if(d>2800 && d < 4100){
        return 1;
    } else if (y<=125 && y>77 && x>56 && x<88) {
        if (x>64 && x<80 && y<117) {
            return 0;
        } else {
            return 1;
        }
    } else if (x+y > 110 && x-y < 34 && y<85 && !(x+y > 121 && x-y < 23 && y<78)) {
        return 1;
    }
    return 0;
}

static uint8_t progress_draw_func(uint8_t x, uint8_t y)
{
    uint32_t d = (72-x)*(72-x) + (84-y)*(84-y);
    if(d>2800 && d < 4100){
        return 1;
    } else if (y<=125 && y>77 && x>56 && x<88) {
        return 1;
    } else if (x+y > 110 && x-y < 34 && y<85) {
        return 1;
    }
    return 0;
}

static void init_progress_notification() {
    last_progress = 0;
    mlcd_set_screen_with_func(progress_bg_draw_func);
  
    mlcd_display_on();
    mlcd_backlight_on();
}

void bootloader_dfu_progress_update(uint32_t m_data_received, uint32_t m_image_size) {
    uint8_t progress = m_data_received*67/m_image_size;
    if(progress > last_progress) {
        mlcd_set_lines_with_func(progress_draw_func, 117u-progress, progress-last_progress);
        last_progress = progress;
    }
}

/**@brief Function for bootloader main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     dfu_start = false;
    bool     app_reset = (NRF_POWER->GPREGRET == BOOTLOADER_DFU_START);

    if (app_reset)
    {
        NRF_POWER->GPREGRET = 0;
    }

    // This check ensures that the defined fields in the bootloader corresponds with actual
    // setting in the nRF51 chip.
    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize.
    timers_init();
    buttons_init();
    spi_init();
    mlcd_init();
    mlcd_power_on();
		
		// make seure lcd is working
		nrf_delay_ms(10);

    (void)bootloader_init();

    if (bootloader_dfu_sd_in_progress())
    {
        err_code = bootloader_dfu_sd_update_continue();
        APP_ERROR_CHECK(err_code);

        ble_stack_init(!app_reset);
        scheduler_init();

        err_code = bootloader_dfu_sd_update_finalize();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If stack is present then continue initialization of bootloader.
        ble_stack_init(!app_reset);
        scheduler_init();
    }

    dfu_start  = app_reset;
    dfu_start |= ((nrf_gpio_pin_read(BOOTLOADER_BUTTON) == 0) ? true: false);
    
    if (dfu_start || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
				init_progress_notification();
        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);
    }

    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START) && !bootloader_dfu_sd_in_progress())
    {
        // Select a bank region to use as application region.
        // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }
    
    NVIC_SystemReset();
}
