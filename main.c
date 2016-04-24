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
#include "nrf_delay.h"
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
#include "softdevice_handler_appsh.h"
#include "pstorage_platform.h"
#include "nrf_mbr.h"
#include "spi_slave.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                                       /**< Include the service_changed characteristic. For DFU this should normally be the case. */

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                                                      /**< Maximum number of events in the scheduler queue. */

#define TX_BUF_SIZE   256              
#define RX_BUF_SIZE   TX_BUF_SIZE       
#define DEF_CHARACTER 0xABu             
#define ORC_CHARACTER 0x0u        

// SPI0. 
#define SPIS0_SCK     3u
#define SPIS0_MOSI    30u
#define SPIS0_MISO    0u
#define SPIS0_SS      2u

#define MASTER_INT 		29u

static uint8_t m_tx_buf[32];        
static uint8_t m_rx_buf[64]; 
        
				
//static uint8_t command_buffer_append[30];
//static bool command_buffer_append_flag;
static uint8_t command_buffer[512];
static uint16_t command_buffer_append_ptr = 0;
static uint16_t command_buffer_offset_ptr = 0;
static uint8_t command_buffer_last_reserved = 0;

void turn_off_ble(void) {
		//sd_softdevice_disable();
		nrf_gpio_cfg_sense_input(SPIS0_SS, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
		sd_power_system_off();
}

void command_buffer_append(uint8_t* data, uint16_t data_size) {
	
		if (command_buffer_append_ptr + data_size > 512) {
				// overflow!
				return;
		}
		
		//sd_nvic_critical_region_enter(0);
		memcpy(&command_buffer[command_buffer_append_ptr], data, data_size);
		command_buffer_append_ptr+=data_size;
		//sd_nvic_critical_region_exit(0);
	
		if (data_size < 20) {
				// interrupt master
				nrf_gpio_pin_clear(MASTER_INT);
				//nrf_delay_us(1);
				nrf_gpio_pin_set(MASTER_INT);
		}
};

uint8_t command_buffer_reserve() {
	//	sd_nvic_critical_region_enter(0);
		uint16_t available = command_buffer_append_ptr - command_buffer_offset_ptr;
		command_buffer_last_reserved = available > 255 ? 255:available;
	//	sd_nvic_critical_region_exit(0);
		return command_buffer_last_reserved;
};

void command_buffer_consume(uint8_t** data_ptr, uint8_t* size_ptr) {
	
		if (command_buffer_last_reserved == 0) {
				return;
		}
	
		//sd_nvic_critical_region_enter(0);
		//memcpy(data, command_buffer, command_buffer_last_reserved);
		//memcpy(command_buffer, &command_buffer[command_buffer_ptr+command_buffer_last_reserved], command_buffer_last_reserved);
		//command_buffer_ptr-=command_buffer_last_reserved;
		(*data_ptr) = &command_buffer[command_buffer_offset_ptr];
		(*size_ptr) = command_buffer_last_reserved;
		command_buffer_offset_ptr+=command_buffer_last_reserved;
		command_buffer_last_reserved = 0;
		//sd_nvic_critical_region_exit(0);
		//return ret;
};


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
    
    // Below code line is needed for s130. For s110 is inrrelevant - but executable
    // can run with both s130 and s110.
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
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

static void spi_slave_event_handle(spi_slave_evt_t event)
{
    uint32_t err_code;
     
    if (event.evt_type == SPI_SLAVE_XFER_DONE)
    {   
				uint8_t tx_size = 0;
				uint8_t* tx_buf = m_tx_buf;
			
				if (event.rx_amount > 1) {
						switch(m_rx_buf[0]) {
								case 0xA:
										switch(m_rx_buf[1]) {
												case 0x8:
														tx_size = 0;
														command_buffer_append_ptr = 0;
														command_buffer_offset_ptr = 0;
														if (event.rx_amount > 2) {
																nus_data_send_response(&m_rx_buf[2], event.rx_amount-2);
														}
														break;
												case 0x9:
														// efm32 power off
														// disable bluetooth;
														turn_off_ble();
														break;
												case 0x16:
														// efm32 power on
														// do nothing
														break;
										}
										break;
								case 0xB:
										switch(m_rx_buf[1]) {
											case 0x5:
													tx_size = 2;
													m_tx_buf[0] = 0;
													m_tx_buf[1] = command_buffer_reserve();
													break;
											case 0x6:
													command_buffer_consume(&tx_buf, &tx_size);
													break;
										}
										break;
						}
				}
			
        err_code = spi_slave_buffers_set(tx_buf, m_rx_buf, tx_size, (uint8_t)sizeof(m_rx_buf));
        APP_ERROR_CHECK(err_code);          
    }
}
 
uint32_t nrf51_spi_slave_init(void)
{
    uint32_t err_code;
	
		err_code = spi_slave_evt_handler_register(spi_slave_event_handle);
		APP_ERROR_CHECK(err_code);    
   
		spi_slave_config_t spi_slave_config;
		spi_slave_config.pin_miso           = SPIS0_MISO;
		spi_slave_config.pin_mosi           = SPIS0_MOSI;
		spi_slave_config.pin_sck            = SPIS0_SCK;
		spi_slave_config.pin_csn            = SPIS0_SS;
		spi_slave_config.mode               = SPI_MODE_0;           // CPOL : 0  / CPHA : 1    From Cortex-M3
		spi_slave_config.bit_order          = SPIM_LSB_FIRST;            
		spi_slave_config.def_tx_character   = DEF_CHARACTER;
		spi_slave_config.orc_tx_character   = ORC_CHARACTER;
	
		err_code = spi_slave_init(&spi_slave_config);
		APP_ERROR_CHECK(err_code);
	
    err_code = spi_slave_buffers_set(m_tx_buf, m_rx_buf, 0, (uint8_t)sizeof(m_rx_buf));
    APP_ERROR_CHECK(err_code);            
    
    return NRF_SUCCESS;
}

/**@brief Function for bootloader main entry.
 */
int main(void)
{
	
		nrf51_spi_slave_init();
    nrf_gpio_cfg_output(MASTER_INT);
    nrf_gpio_pin_set(MASTER_INT);
	
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
//    dfu_start |= ((nrf_gpio_pin_read(BOOTLOADER_BUTTON) == 0) ? true: false);
    
    if (dfu_start || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
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
