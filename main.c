/**
   * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
   * 
   * All rights reserved.
   * 
   * Redistribution and use in source and binary forms, with or without modification,
   * are permitted provided that the following conditions are met:
   * 
   * 1. Redistributions of source code must retain the above copyright notice, this
   *    list of conditions and the following disclaimer.
   * 
   * 2. Redistributions in binary form, except as embedded into a Nordic
   *    Semiconductor ASA integrated circuit in a product or a software update for
   *    such product, must reproduce the above copyright notice, this list of
   *    conditions and the following disclaimer in the documentation and/or other
   *    materials provided with the distribution.
   * 
   * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
   *    contributors may be used to endorse or promote products derived from this
   *    software without specific prior written permission.
   * 
   * 4. This software, with or without modification, must only be used with a
   *    Nordic Semiconductor ASA integrated circuit.
   * 
   * 5. Any software provided in binary form under this license must not be reverse
   *    engineered, decompiled, modified and/or disassembled.
   * 
   * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
   * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
   * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
   * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
   * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
   * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   * 
   */
/**
   * @brief BLE LED Button Service central and client application main file.
   *
   * This file contains the source code for a sample client application using the LED Button service.
   */

#include "app_timer.h"
#include "app_util.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_hci.h"
#include "ble_lbs_c.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_delay.h" // Kinda cheesy, just threw in NRF delay for my little demo
#include "nrf_fstorage.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwm.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_soc.h"
#include "nrfx_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#define PERIOD 255
#define DELAY 2000
#define SMOOTH_DELAY 50
#define RESOLUTION 5

#define RED_PIN NRF_GPIO_PIN_MAP(0, 22)
#define GREEN_PIN NRF_GPIO_PIN_MAP(0, 20)
#define BLUE_PIN NRF_GPIO_PIN_MAP(0, 24)

#define CENTRAL_SCANNING_LED BSP_BOARD_LED_3  /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED BSP_BOARD_LED_1 /**< Connected LED will be on when the device is connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2         /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define SCAN_INTERVAL 0x0050 /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW 0x0055   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION 0x0000 /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)  /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY 0                                          /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)      /**< Determines supervision time-out in units of 10 milliseconds. */

#define LEDBUTTON_BUTTON_PIN BSP_BUTTON_0          /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50) /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define APP_BLE_CONN_CFG_TAG 1   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO 3  /**< Application's BLE observer priority. You shouldn't need to modify this value. */
BLE_LBS_C_DEF(m_ble_lbs_c);      /**< Main structure used by the LBS client module. */
NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< DB discovery module instance. */

APP_TIMER_DEF(m_singleshot_timer_id);

void Packet_Handle(char *Packet);
void clear_show();
void Pack_ID_init();

//FDS Stuff
#define FILE_ID 0x1111
#define REC_KEY 0x2222

#include "nrf_fstorage_sd.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)
#define APP_BLE_CONN_CFG_TAG 1

#define SHOW_SIZE 256
#define FLASH_ADDR_START 0x3e000
#define FLASH_ADDR_END 0x3e400

#define FUEL_GAUGE_PIN NRF_GPIO_PIN_MAP(1, 13)


uint8_t PACK_ID[3]; //if PACKID = D0590F then PACKID[0] = D0 etc.
char Show_Num[2] = {0xFF, 0xFF};

struct Set {
    unsigned char R;
    unsigned char G;
    unsigned char B;
} Set;

struct Show {
    struct Set sets[SHOW_SIZE];
    //uint32 pack_id;
    //unsigned char num_sets;
} Show;

char prev_message[7];

struct Show current_show;

static void my_fds_evt_handler(fds_evt_t const *const p_fds_evt) {
    switch (p_fds_evt->id) {
    case FDS_EVT_INIT:
        if (p_fds_evt->result != FDS_SUCCESS) {
            // Initialization failed.
            NRF_LOG_INFO("FDS Initialization Failed");
        }
        break;
    case FDS_EVT_WRITE:
        if (p_fds_evt->result == FDS_SUCCESS) {
            //write_flag=1;
            //                                                        load_show_from_flash();
        }
        break;
    default:
        break;
    }
}

static bool fds_is_existed_record(fds_record_desc_t *record_desc, uint16_t key) {
    fds_find_token_t ftok = {0};

    return (fds_record_find(FILE_ID, key, record_desc, &ftok) == FDS_SUCCESS);
}

static uint32_t masterStuff;
static int globalGO = 1;

static ret_code_t async_write_show_to_flash(const struct Show Wr_Show) {

    //static uint32_t const m_deadbeef[2] = {0xDEADBEEF,0xBAADF00D};
    fds_record_t record;
    fds_record_desc_t record_desc;

    // Set up record.
    record.file_id = FILE_ID;
    record.key = REC_KEY;
    record.data.p_data = &Wr_Show;
    record.data.length_words = sizeof(Wr_Show) / sizeof(uint32_t) + 1;
    NRF_LOG_INFO("Saving Structure (size: %d)", record.data.length_words);
    //NRF_LOG_INFO("Size of Show: %d",sizeof(Wr_Show)/4 + 1);

    ret_code_t rc;
    if (fds_is_existed_record(&record_desc, REC_KEY)) {
        rc = fds_record_update(&record_desc, &record);
        fds_gc();
    } else {
        rc = fds_record_write(&record_desc, &record);
        fds_gc();
    }

    if (rc != FDS_SUCCESS) {
        /* Handle error. */
        NRF_LOG_INFO("ERROR IN FDS WRITE: %d", rc);

        if (rc == FDS_ERR_NO_SPACE_IN_FLASH) {
            NRF_LOG_INFO("NO SPACE IN FLASH");
            fds_gc();

            if (fds_is_existed_record(&record_desc, REC_KEY)) {
                NRF_LOG_INFO("Update Flash");
                rc = fds_record_update(&record_desc, &record);
            } else {
                NRF_LOG_INFO("Write Flash");
                rc = fds_record_write(&record_desc, &record);
            }

            if (rc == FDS_SUCCESS) {
                NRF_LOG_INFO("FIXED!!!");
            } else {
                NRF_LOG_INFO("NOT FIXED!!!: %d", rc);
            }
        }
    }

    NRF_LOG_INFO("Writing Record ID = %d \r\n", record_desc.record_id);
    masterStuff = record_desc.record_id;
    NRF_LOG_INFO("masterStuff %d", masterStuff);
    globalGO = 1;
    return NRF_SUCCESS;
}

int load_show_from_flash() {
    //int * re_int;
    //int return_int;
    struct Show *Re_Show;
    int re_code = 0;
    //struct Show re_s;
    fds_flash_record_t flash_record;
    fds_record_desc_t record_desc;
    fds_find_token_t ftok;
    /* It is required to zero the token before first use. */
    memset(&ftok, 0x00, sizeof(fds_find_token_t));

    /* Loop until all records with the given key and file ID have been found. */
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS) {
        if (fds_record_open(&record_desc, &flash_record) != FDS_SUCCESS) {
            /* Handle error. */
            re_code = -1;
            NRF_LOG_INFO("ERROR HAPPENED. :(");
        } else {
            NRF_LOG_INFO("READING Record ID = %d \r\n", record_desc.record_id);

            Re_Show = (struct Show *)flash_record.p_data;
            current_show = *Re_Show;
            break;
        }

        if (fds_record_close(&record_desc) != FDS_SUCCESS) {
            /* Handle error. */
            re_code = -1;
        }
    }
    return re_code;
}

static ret_code_t fds_test_find_and_delete(void) {

    fds_record_desc_t record_desc;
    fds_find_token_t ftok;

    ftok.page = 0;
    ftok.p_addr = NULL;
    // Loop and find records with same ID and rec key and mark them as deleted.
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS) {
        fds_record_delete(&record_desc);
        NRF_LOG_INFO("Deleted record ID: %d \r\n", record_desc.record_id);
    }
    // call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
    ret_code_t ret = fds_gc();
    if (ret != FDS_SUCCESS) {
        return ret;
    }
    return NRF_SUCCESS;
}

static ret_code_t fds_test_init(void) {

    ret_code_t ret = fds_register(my_fds_evt_handler);
    if (ret != FDS_SUCCESS) {
        return ret;
    }
    ret = fds_init();
    if (ret != FDS_SUCCESS) {
        return ret;
    }

    return NRF_SUCCESS;
}

struct Show Retrieve_Stored_Show() {
    struct Show to_read;

    return to_read;
}

int Store_Show(struct Show to_store) {
}

/* Dummy data to write to flash. */
static uint32_t m_data = 0xBADC0FFE;
static char m_hello_world[] = "hello world";

static char const m_target_periph_name[] = "LMND"; /**< Name of the device we try to connect to. This name is searched in the scan report data*/

/**@brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
    {
        .active = 1,
        .interval = SCAN_INTERVAL,
        .window = SCAN_WINDOW,
        .channel_mask[4] = 0x60, // ONLY SCAN on Channel 39
        .timeout = SCAN_DURATION,
        .scan_phys = BLE_GAP_PHY_1MBPS,
        .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
    {
        m_scan_buffer_data,
        BLE_GAP_SCAN_BUFFER_MIN};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
    {
        (uint16_t)MIN_CONNECTION_INTERVAL,
        (uint16_t)MAX_CONNECTION_INTERVAL,
        (uint16_t)SLAVE_LATENCY,
        (uint16_t)SUPERVISION_TIMEOUT};

/**@brief Function to handle asserts in the SoftDevice.
   *
   * @details This function will be called in case of an assert in the SoftDevice.
   *
   * @warning This handler is an example only and does not fit a final product. You need to analyze
   *          how your product is supposed to react in case of Assert.
   * @warning On assert from the SoftDevice, the system can only recover on reset.
   *
   * @param[in] line_num     Line number of the failing ASSERT call.
   * @param[in] p_file_name  File name of the failing ASSERT call.
   */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
   *
   * @details Initializes all LEDs used by the application.
   */
static void leds_init(void) {
    bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function to start scanning.
   */
static void scan_start(void) {
    ret_code_t err_code;

    (void)sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(err_code);

    //bsp_board_led_off(CENTRAL_CONNECTED_LED);
    //bsp_board_led_on(CENTRAL_SCANNING_LED);
}

/**@brief Handles events coming from the LED Button central module.
   */
static void lbs_c_evt_handler(ble_lbs_c_t *p_lbs_c, ble_lbs_c_evt_t *p_lbs_c_evt) {
    switch (p_lbs_c_evt->evt_type) {
    case BLE_LBS_C_EVT_DISCOVERY_COMPLETE: {
        ret_code_t err_code;

        err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
            p_lbs_c_evt->conn_handle,
            &p_lbs_c_evt->params.peer_db);
        NRF_LOG_INFO("LED Button service discovered on conn_handle 0x%x.", p_lbs_c_evt->conn_handle);

        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);

        // LED Button service discovered. Enable notification of Button.
        err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
        APP_ERROR_CHECK(err_code);
    } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

    case BLE_LBS_C_EVT_BUTTON_NOTIFICATION: {
        NRF_LOG_INFO("Button state changed on peer to 0x%x.", p_lbs_c_evt->params.button.button_state);
        if (p_lbs_c_evt->params.button.button_state) {
            //bsp_board_led_on(LEDBUTTON_LED);
        } else {
            //bsp_board_led_off(LEDBUTTON_LED);
        }
    } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for handling the advertising report BLE event.
   *
   * @param[in] p_adv_report  Advertising report from the SoftDevice.
   */
static void on_adv_report(ble_gap_evt_adv_report_t const *p_adv_report) {
    ret_code_t err_code;

    //    NRF_LOG_INFO("trigger");
    if (ble_advdata_name_find(p_adv_report->data.p_data,
            p_adv_report->data.len,
            m_target_periph_name))
    //    if (ble_advdata_short_name_find(p_adv_report->data.p_data, p_adv_report->data.len, m_target_periph_name, 4))
    {
        //NRF_LOG_INFO("found LMND");
        char *message = ble_advdata_parse(p_adv_report->data.p_data,
                            p_adv_report->data.len, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA) +
                        2;
        //NRF_LOG_INFO("yay, value is: %s", message);
        

        //change_lmnd_color(message[0],message[1],message[2]);
        Packet_Handle(message);
//        err_code = app_timer_start(m_singleshot_timer_id, APP_TIMER_TICKS(5), NULL);
//        APP_ERROR_CHECK(err_code);
        scan_start();
        // Name is a match, initiate connection.
        //        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
        //                                      &m_scan_params,
        //                                      &m_connection_param,
        //                                      APP_BLE_CONN_CFG_TAG);
        //        APP_ERROR_CHECK(err_code);
    } else {
        //        NRF_LOG_INFO("quick scan");
        scan_start();
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling BLE events.
   *
   * @param[in]   p_ble_evt   Bluetooth stack event.
   * @param[in]   p_context   Unused.
   */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
    // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
    // discovery, update LEDs status and resume scanning if necessary. */
    case BLE_GAP_EVT_CONNECTED: {
        NRF_LOG_INFO("Connected.");
        err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c, p_gap_evt->conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
        APP_ERROR_CHECK(err_code);

        // Update LEDs status, and check if we should be looking for more
        // peripherals to connect to.
        //bsp_board_led_on(CENTRAL_CONNECTED_LED);
        //bsp_board_led_off(CENTRAL_SCANNING_LED);
    } break;

    // Upon disconnection, reset the connection handle of the peer which disconnected, update
    // the LEDs status and start scanning again.
    case BLE_GAP_EVT_DISCONNECTED: {
        NRF_LOG_INFO("Disconnected.");
        scan_start();
    } break;

    case BLE_GAP_EVT_ADV_REPORT: {
        on_adv_report(&p_gap_evt->params.adv_report);
    } break;

    case BLE_GAP_EVT_TIMEOUT: {
        // We have not specified a timeout for scanning, so only connection attemps can timeout.
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
            NRF_LOG_DEBUG("Connection request timed out.");
        }
    } break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
        // Accept parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
            &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT: {
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTS_EVT_TIMEOUT: {
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    } break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief LED Button client initialization.
   */
static void lbs_c_init(void) {
    ret_code_t err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    //err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
   *
   * @details Initializes the SoftDevice and the BLE event interrupts.
   */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the button handler module.
   *
   * @param[in] pin_no        The pin that the event applies to.
   * @param[in] button_action The button action (press/release).
   */
static void button_event_handler(uint8_t pin_no, uint8_t button_action) {
    ret_code_t err_code;

    switch (pin_no) {
    case LEDBUTTON_BUTTON_PIN:
        //err_code = ble_lbs_led_status_send(&m_ble_lbs_c, button_action);
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE) {
            APP_ERROR_CHECK(err_code);
        }
        if (err_code == NRF_SUCCESS) {
            NRF_LOG_INFO("LBS write LED state %d", button_action);
        }
        break;

    default:
        APP_ERROR_HANDLER(pin_no);
        break;
    }
}

/**@brief Function for initializing the button handler module.
   */
static void buttons_init(void) {
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
        {
            {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}};

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
        BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
   *
   * @details This function is callback function to handle events from the database discovery module.
   *          Depending on the UUIDs that are discovered, this function should forward the events
   *          to their respective services.
   *
   * @param[in] p_event  Pointer to the database discovery event.
   */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt) {
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
}

/**@brief Database discovery initialization.
   */
static void db_discovery_init(void) {
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the log.
   */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the timer.
   */
static void timer_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Power manager. */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
   */
static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
   *
   * @details Handle any pending log operation(s), then sleep until the next event occurs.
   */
static void idle_state_handle(void) {
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}

/**@brief Timeout handler for the repeated timer.
   */
static void singleshot_timer_handler(void *p_context) {
    scan_start();
}

/**@brief Create timers.
   */
static void create_timers() {
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_singleshot_timer_id,
        APP_TIMER_MODE_SINGLE_SHOT,
        singleshot_timer_handler);
    APP_ERROR_CHECK(err_code);
}

// @BLAKE just set up the PWM to work with the new NRFX stuff
// instead of the legacy nrf_drv stuff.  Woot
//static uint16_t const counter_top = 10000;

static nrfx_pwm_t RGB_PWM = NRFX_PWM_INSTANCE(0); // Use PWM 0
static nrfx_pwm_t FUEL_PWM = NRFX_PWM_INSTANCE(1);

static nrf_pwm_values_individual_t seq_values[] = {255, 255, 255, 255};

static nrf_pwm_sequence_t const m_RGB_seq =
    {
        .values.p_individual = seq_values,
        .length = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats = 0,
        .end_delay = 0};

static nrf_pwm_values_common_t seq_fuel_values[] = {0x4000};

static nrf_pwm_sequence_t const m_fuel_seq =
    {
        .values.p_common = seq_fuel_values,
        .length = NRF_PWM_VALUES_LENGTH(seq_fuel_values),
        .repeats = 0,
        .end_delay = 0};

static void pwm_init() {
    uint32_t ret_code;

    nrfx_pwm_config_t const config_RGB_pwm =
        {
            .output_pins =
                {
                    RED_PIN | NRFX_PWM_PIN_INVERTED,   // channel 0
                    GREEN_PIN | NRFX_PWM_PIN_INVERTED, // channel 1
                    BLUE_PIN | NRFX_PWM_PIN_INVERTED,  // channel 2
                    NRFX_PWM_PIN_NOT_USED              // channel 3
                },
            .irq_priority = APP_IRQ_PRIORITY_HIGH,
            .base_clock = NRF_PWM_CLK_2MHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = PERIOD,
            .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
            .step_mode = NRF_PWM_STEP_AUTO};

    nrfx_pwm_config_t const config_fuel_pwm =
        {
            .output_pins =
                {
                    FUEL_GAUGE_PIN | NRFX_PWM_PIN_INVERTED, // channel 0
                    NRFX_PWM_PIN_NOT_USED,                  // channel 1
                    NRFX_PWM_PIN_NOT_USED,                  // channel 2
                    NRFX_PWM_PIN_NOT_USED                   // channel 3
                },
            .irq_priority = APP_IRQ_PRIORITY_HIGH,
            .base_clock = NRF_PWM_CLK_125kHz,
            .count_mode = NRF_PWM_MODE_UP,
            .top_value = 0xFFFF,
            .load_mode = NRF_PWM_LOAD_COMMON,
            .step_mode = NRF_PWM_STEP_AUTO};

    ret_code = nrfx_pwm_init(&RGB_PWM, &config_RGB_pwm, NULL);
    APP_ERROR_CHECK(ret_code);
    ret_code = nrfx_pwm_init(&FUEL_PWM, &config_fuel_pwm, NULL);
    APP_ERROR_CHECK(ret_code);

    nrfx_pwm_simple_playback(&FUEL_PWM, &m_fuel_seq, 1, NRFX_PWM_FLAG_LOOP);
}

void change_lmnd_color(uint16_t R, uint16_t G, uint16_t B) {

    if (R > PERIOD) {
        seq_values->channel_0 = 0;

    } else {
        seq_values->channel_0 = PERIOD - R;
    }
    if (G > PERIOD) {
        seq_values->channel_1 = 0;
    } else {
        seq_values->channel_1 = PERIOD - G;
    }
    if (B > PERIOD) {
        seq_values->channel_2 = 0;
    } else {
        seq_values->channel_2 = PERIOD - B;
    }

    nrfx_pwm_simple_playback(&RGB_PWM, &m_RGB_seq, 1, NRFX_PWM_FLAG_LOOP);
}

void System_Init() {
    log_init();
    timer_init();
    create_timers();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    lbs_c_init();
    pwm_init();
    if (fds_test_init() != NRF_SUCCESS) {
        NRF_LOG_INFO("Failed to INIT");
    }
    Pack_ID_init();
}

int RGB_Bound_Error(char RGB) {
    return !(RGB >= 0 && RGB < 256);
}

//$CALEB use this for handling a packet message
void Packet_Handle(char *Packet) {

    //P - 5B{3B - PACK ID, 2B show #} Program packid for show number
    //I - 6B{2B - Show #, 1B set #, 3B RGB} program the
    //B - 1B{Set #}
    //S - Save Current Show

    //P - Program Packet
    //S - Show Display Packet
    //D - Done Programming Packet (i.e. Save current show to flash)

    if (!(prev_message[0] == Packet[0] && prev_message[1] == Packet[1] && prev_message[2] == Packet[2] && prev_message[3] == Packet[3] && prev_message[4] == Packet[4] && prev_message[5] == Packet[5] && prev_message[6] == Packet[6])) {
        prev_message[0] = Packet[0];
        prev_message[1] = Packet[1];
        prev_message[2] = Packet[2];
        prev_message[3] = Packet[3];
        prev_message[4] = Packet[4];
        prev_message[5] = Packet[5];
        prev_message[6] = Packet[6];
        NRF_LOG_INFO("Packet Receieved");
        NRF_LOG_INFO("Packet(%c | %d | %d | %d | %d | %d )", Packet[0], Packet[1], Packet[2], Packet[3], Packet[4], Packet[5]);
        NRF_LOG_INFO("Packet[6] = %d", Packet[6]);

        if (Packet[0] == 'P') {
            if (Packet[1] == PACK_ID[0] && Packet[2] == PACK_ID[1] && Packet[3] == PACK_ID[2]) {
                Show_Num[0] = Packet[4];
                Show_Num[1] = Packet[5];
                clear_show();
                //change_lmnd_color(255,0,0);
                NRF_LOG_INFO("P-Packet: PACKID Request : %x | %x | %x", Packet[1], Packet[2], Packet[3]);
                NRF_LOG_INFO("P-Packet(Show Num[0] = %x, Show Num[1] = %x)", Packet[4], Packet[5]);
            }
        } else if (Packet[0] == 'B') {
            if (!RGB_Bound_Error(Packet[1])) {
                if (globalGO == 1) {
                    clear_show();
                    load_show_from_flash();
                    globalGO = 0;
                }
                //              load_show_from_flash();
                NRF_LOG_INFO("B-Packet(Set %d: (R,G,B) == (%d,%d,%d))", Packet[1], current_show.sets[Packet[1]].R, current_show.sets[Packet[1]].G, current_show.sets[Packet[1]].B);
                change_lmnd_color(current_show.sets[Packet[1]].R, current_show.sets[Packet[1]].G, current_show.sets[Packet[1]].B);
            }
        } else if (Packet[0] == 'S') {
            if (Packet[1] == Show_Num[0] && Packet[2] == Show_Num[1])
            {

            //NRF_LOG_INFO("Packet(%c | %d | %d | %d | %d| %d | %d)",Packet[0],Packet[1],Packet[2],Packet[3],Packet[4],Packet[5]);
              async_write_show_to_flash(current_show);
              }
        } else if (Packet[0] == 'I') {
            if (Packet[1] == Show_Num[0] && Packet[2] == Show_Num[1]) {
                if (!RGB_Bound_Error(Packet[3]) && !RGB_Bound_Error(Packet[4]) && !RGB_Bound_Error(Packet[5]) && !RGB_Bound_Error(Packet[6])) {
                    current_show.sets[Packet[3]].R = Packet[4];
                    current_show.sets[Packet[3]].G = Packet[5];
                    current_show.sets[Packet[3]].B = Packet[6];
                }
            }
        }
    }
}

void clear_show() {
    int i = 0;
    for (i = 0; i < 256; i++) {
        current_show.sets[i].R = 0;
        current_show.sets[i].G = 0;
        current_show.sets[i].B = 0;
    }
}

uint32_t crc32_compute(uint8_t const *p_data, uint32_t size, uint32_t const *p_crc) {
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++) {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--) {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

void Pack_ID_init() {
    //Calculate PACK ID
    ble_gap_addr_t ble_addr;
    sd_ble_gap_addr_get(&ble_addr);
    //NRF_LOG_INFO("MAC: %02X:%02X:%02X:%02X:%02X:%02X",ble_addr.addr[0],ble_addr.addr[1],ble_addr.addr[2],ble_addr.addr[3],ble_addr.addr[4],ble_addr.addr[5]);

    uint32_t CRC1 = crc32_compute(ble_addr.addr, 3, NULL) & 0xFFFFFF;
    uint32_t CRC2 = crc32_compute(&ble_addr.addr[3], 3, NULL) & 0xFFFFFF;

    //NRF_LOG_INFO("HASH1:%x",CRC1);
    //NRF_LOG_INFO("HASH2:%x",CRC2);

    uint32_t CRC_C = CRC1 ^ CRC2;

    //NRF_LOG_INFO("PACKID: %x",CRC_C);
    PACK_ID[0] = (CRC_C & 0xFF);
    PACK_ID[1] = (CRC_C & 0xFF00) >> 8;
    PACK_ID[2] = (CRC_C & 0xFF0000) >> 16;

    NRF_LOG_INFO("PACKID: %02x | %02x | %02x", PACK_ID[0], PACK_ID[1], PACK_ID[2]);
}

int main(void) {
    int i;
    System_Init();
    // Start execution.

    //If fails to load the show, use a blank one.
    if (load_show_from_flash() == -1) {
        clear_show();
        change_lmnd_color(255, 0, 0);
        nrf_delay_ms(100);
        change_lmnd_color(0, 0, 0);
        NRF_LOG_INFO("Failed to retrieve show");
    } else {
        change_lmnd_color(0, 255, 0);
        nrf_delay_ms(100);
        change_lmnd_color(0, 0, 0);
        NRF_LOG_INFO("Retrieved show yay!");
    }

    scan_start();

    // Turn on the LED to signal scanning.
    //bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Enter main loop.
    for (i = 0;; i++) {
        idle_state_handle();
    }
}