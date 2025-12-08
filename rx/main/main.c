/**
 * ESP32-S3 BLE MIDI Receiver
 *
 * 通过BLE接收MIDI信号，作为USB MIDI设备输出到电脑
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

// BLE
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// TinyUSB MIDI
#include "tinyusb.h"
#include "class/midi/midi_device.h"

// LED
#include "led_strip.h"
#include "driver/rmt_tx.h"

static const char *TAG = "BLE_MIDI_RX";

// ==================== 配置 ====================
#define WS2812_GPIO         48      // WS2812 LED GPIO (根据实际情况修改)
#define BLE_SCAN_DURATION   0       // 无限扫描

// BLE MIDI Service UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
static uint8_t ble_midi_service_uuid[16] = {
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

// BLE MIDI Characteristic UUID: 7772E5DB-3868-4112-A1A9-F2669D106BF3
static uint8_t ble_midi_char_uuid[16] = {
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};

// ==================== LED 控制 ====================
static led_strip_handle_t led_strip;

typedef enum {
    LED_STATE_DISCONNECTED,     // 红色 - 未连接
    LED_STATE_SEARCHING,        // 蓝色闪烁 - 搜索中
    LED_STATE_CONNECTED,        // 绿色 - 已连接
    LED_STATE_DATA_RX           // 青色闪烁 - 数据接收
} led_state_t;

static led_state_t current_led_state = LED_STATE_DISCONNECTED;
static volatile bool midi_activity = false;

static void led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

static void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static void led_task(void *arg)
{
    static int blink_cnt = 0;

    while (1) {
        blink_cnt++;

        switch (current_led_state) {
            case LED_STATE_DISCONNECTED:
                led_set_color(50, 0, 0);
                break;

            case LED_STATE_SEARCHING:
                if (blink_cnt % 2 == 0) {
                    led_set_color(0, 0, 50);
                } else {
                    led_set_color(0, 0, 0);
                }
                break;

            case LED_STATE_CONNECTED:
                if (midi_activity) {
                    led_set_color(0, 50, 50);
                    midi_activity = false;
                } else {
                    led_set_color(0, 50, 0);
                }
                break;

            case LED_STATE_DATA_RX:
                led_set_color(0, 50, 50);
                current_led_state = LED_STATE_CONNECTED;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ==================== USB MIDI Device ====================
// MIDI队列
#define MIDI_QUEUE_SIZE 256
static QueueHandle_t midi_rx_queue;

typedef struct {
    uint8_t data[4];
    uint8_t len;
} midi_packet_t;

// TinyUSB描述符
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

static uint8_t const desc_fs_configuration[] = {
    // Config descriptor
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, TUSB_DESC_TOTAL_LEN, 0, 100),
    // MIDI descriptor
    TUD_MIDI_DESCRIPTOR(0, 0, 0x01, 0x81, 64),
};

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,         // Espressif VID
    .idProduct = 0x4002,        // Custom PID
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},     // 0: 语言ID
    "Wireless MIDI",                 // 1: 制造商
    "BLE MIDI Receiver",            // 2: 产品名
    "123456",                       // 3: 序列号
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_fs_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t _desc_str[32];

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        _desc_str[0] = (TUSB_DESC_STRING << 8) | 4;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        uint8_t len = strlen(str);
        if (len > 31) len = 31;

        for (uint8_t i = 0; i < len; i++) {
            _desc_str[1 + i] = str[i];
        }
        _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 + 2 * len);
    }

    return _desc_str;
}

// 获取MIDI的Code Index Number
static uint8_t get_midi_cin(uint8_t status_byte)
{
    uint8_t cin = 0;
    uint8_t msg_type = status_byte & 0xF0;

    switch (msg_type) {
        case 0x80: cin = 0x8; break;  // Note Off
        case 0x90: cin = 0x9; break;  // Note On
        case 0xA0: cin = 0xA; break;  // Poly Aftertouch
        case 0xB0: cin = 0xB; break;  // Control Change
        case 0xC0: cin = 0xC; break;  // Program Change
        case 0xD0: cin = 0xD; break;  // Channel Aftertouch
        case 0xE0: cin = 0xE; break;  // Pitch Bend
        case 0xF0:
            switch (status_byte) {
                case 0xF0: cin = 0x4; break;  // SysEx start
                case 0xF7: cin = 0x5; break;  // SysEx end (single)
                case 0xF1: case 0xF3: cin = 0x2; break;  // 2-byte system
                case 0xF2: cin = 0x3; break;  // 3-byte system
                default: cin = 0xF; break;    // 1-byte system
            }
            break;
        default:
            cin = 0;
    }

    return cin;
}

// 发送MIDI数据到USB
static void send_usb_midi(uint8_t *midi_data, uint8_t len)
{
    if (len == 0 || !tud_midi_mounted()) return;

    uint8_t cin = get_midi_cin(midi_data[0]);
    if (cin == 0) return;

    uint8_t usb_packet[4] = {0};
    usb_packet[0] = cin;  // Cable number 0 + CIN

    // 复制MIDI数据
    for (int i = 0; i < len && i < 3; i++) {
        usb_packet[i + 1] = midi_data[i];
    }

    tud_midi_stream_write(0, usb_packet + 1, len);
    midi_activity = true;
}

// USB MIDI任务
static void usb_midi_task(void *arg)
{
    midi_packet_t pkt;

    while (1) {
        if (xQueueReceive(midi_rx_queue, &pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
            send_usb_midi(pkt.data, pkt.len);
        }

        tud_task();
    }
}

// ==================== BLE GATT Client ====================
#define PROFILE_NUM     1
#define PROFILE_APP_ID  0
#define INVALID_HANDLE  0

static const char *target_device_name = "BLE-MIDI-TX";

static bool ble_connected = false;
static bool ble_scanning = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = NULL,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

// 解析BLE MIDI数据
static void parse_ble_midi_data(uint8_t *data, uint16_t len)
{
    if (len < 3) return;  // 最小: header + timestamp + 1 byte midi

    // data[0]: Header byte (bit7=1, bit6-0: timestamp高位)
    // data[1]: Timestamp byte (bit7=1, bit6-0: timestamp低位)
    // data[2...]: MIDI数据

    uint8_t idx = 1;  // 跳过header

    while (idx < len) {
        // 检查是否是timestamp byte
        if (data[idx] & 0x80) {
            idx++;  // 跳过timestamp
            if (idx >= len) break;
        }

        uint8_t status = data[idx];

        // 判断MIDI消息长度
        uint8_t midi_len = 0;
        midi_packet_t pkt = {0};

        if (status >= 0x80 && status <= 0xEF) {
            uint8_t msg_type = status & 0xF0;
            switch (msg_type) {
                case 0xC0: case 0xD0:  // Program Change, Channel Aftertouch
                    midi_len = 2;
                    break;
                default:  // Note, CC, Pitch Bend, etc.
                    midi_len = 3;
                    break;
            }
        } else if (status >= 0xF0) {
            switch (status) {
                case 0xF1: case 0xF3:  // MTC Quarter Frame, Song Select
                    midi_len = 2;
                    break;
                case 0xF2:  // Song Position
                    midi_len = 3;
                    break;
                case 0xF0:  // SysEx start - 需要特殊处理
                    // 简化处理：查找F7结束符
                    {
                        uint8_t sysex_start = idx;
                        while (idx < len && data[idx] != 0xF7) idx++;
                        if (idx < len) idx++;  // 包含F7
                        // SysEx暂不处理
                        continue;
                    }
                default:  // 其他实时消息
                    midi_len = 1;
                    break;
            }
        } else {
            // Running status或无效数据
            idx++;
            continue;
        }

        // 检查边界
        if (idx + midi_len > len) break;

        // 复制MIDI数据
        pkt.len = midi_len;
        for (int i = 0; i < midi_len; i++) {
            pkt.data[i] = data[idx + i];
        }

        // 放入队列
        xQueueSend(midi_rx_queue, &pkt, 0);

        idx += midi_len;
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                         esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = param;

    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(TAG, "GATTC register, status=%d", param->reg.status);
            esp_ble_gap_set_scan_params(&ble_scan_params);
            break;

        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(TAG, "Connected, conn_id=%d", p_data->connect.conn_id);
            gl_profile_tab[PROFILE_APP_ID].conn_id = p_data->connect.conn_id;
            memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda,
                   sizeof(esp_bd_addr_t));

            esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
            break;

        case ESP_GATTC_OPEN_EVT:
            if (param->open.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Open failed, status=%d", param->open.status);
                current_led_state = LED_STATE_SEARCHING;
                esp_ble_gap_start_scanning(BLE_SCAN_DURATION);
                break;
            }
            ESP_LOGI(TAG, "Open success");
            break;

        case ESP_GATTC_CFG_MTU_EVT:
            if (param->cfg_mtu.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Config MTU failed, status=%d", param->cfg_mtu.status);
            }
            ESP_LOGI(TAG, "MTU=%d", param->cfg_mtu.mtu);
            esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
            break;

        case ESP_GATTC_SEARCH_RES_EVT:
            if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
                if (memcmp(p_data->search_res.srvc_id.uuid.uuid.uuid128, ble_midi_service_uuid, 16) == 0) {
                    ESP_LOGI(TAG, "Found MIDI service");
                    gl_profile_tab[PROFILE_APP_ID].service_start_handle = p_data->search_res.start_handle;
                    gl_profile_tab[PROFILE_APP_ID].service_end_handle = p_data->search_res.end_handle;
                }
            }
            break;

        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (param->search_cmpl.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Search service failed, status=%d", param->search_cmpl.status);
                break;
            }

            if (gl_profile_tab[PROFILE_APP_ID].service_start_handle != 0) {
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                    gattc_if, param->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
                    gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_APP_ID].service_end_handle, INVALID_HANDLE, &count);

                if (status == ESP_GATT_OK && count > 0) {
                    char_elem_result = malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (char_elem_result) {
                        esp_ble_gattc_get_all_char(gattc_if, param->search_cmpl.conn_id,
                            gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                            gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                            char_elem_result, &count, 0);

                        for (int i = 0; i < count; i++) {
                            if (char_elem_result[i].uuid.len == ESP_UUID_LEN_128) {
                                if (memcmp(char_elem_result[i].uuid.uuid.uuid128, ble_midi_char_uuid, 16) == 0) {
                                    ESP_LOGI(TAG, "Found MIDI characteristic, handle=%d",
                                             char_elem_result[i].char_handle);
                                    gl_profile_tab[PROFILE_APP_ID].char_handle = char_elem_result[i].char_handle;

                                    // 注册通知
                                    esp_ble_gattc_register_for_notify(gattc_if,
                                        gl_profile_tab[PROFILE_APP_ID].remote_bda,
                                        char_elem_result[i].char_handle);
                                    break;
                                }
                            }
                        }
                        free(char_elem_result);
                        char_elem_result = NULL;
                    }
                }
            }
            break;

        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            if (param->reg_for_notify.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Register notify failed, status=%d", param->reg_for_notify.status);
                break;
            }

            ESP_LOGI(TAG, "Register notify success");

            // 写入CCC使能通知
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                gattc_if, gl_profile_tab[PROFILE_APP_ID].conn_id,
                ESP_GATT_DB_DESCRIPTOR,
                gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                gl_profile_tab[PROFILE_APP_ID].char_handle, &count);

            if (status == ESP_GATT_OK && count > 0) {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (descr_elem_result) {
                    esp_ble_gattc_get_all_descr(gattc_if,
                        gl_profile_tab[PROFILE_APP_ID].conn_id,
                        gl_profile_tab[PROFILE_APP_ID].char_handle,
                        descr_elem_result, &count, 0);

                    for (int i = 0; i < count; i++) {
                        if (descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 &&
                            descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {

                            uint16_t notify_en = 0x0001;
                            esp_ble_gattc_write_char_descr(gattc_if,
                                gl_profile_tab[PROFILE_APP_ID].conn_id,
                                descr_elem_result[i].handle,
                                sizeof(notify_en), (uint8_t *)&notify_en,
                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
                            break;
                        }
                    }
                    free(descr_elem_result);
                    descr_elem_result = NULL;
                }
            }
            break;

        case ESP_GATTC_WRITE_DESCR_EVT:
            if (param->write.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Write CCC failed, status=%d", param->write.status);
            } else {
                ESP_LOGI(TAG, "Notification enabled");
                ble_connected = true;
                current_led_state = LED_STATE_CONNECTED;
            }
            break;

        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGD(TAG, "Notify received, len=%d", param->notify.value_len);
            parse_ble_midi_data(param->notify.value, param->notify.value_len);
            break;

        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Disconnected, reason=0x%x", param->disconnect.reason);
            ble_connected = false;
            current_led_state = LED_STATE_SEARCHING;
            gl_profile_tab[PROFILE_APP_ID].service_start_handle = 0;
            gl_profile_tab[PROFILE_APP_ID].service_end_handle = 0;
            gl_profile_tab[PROFILE_APP_ID].char_handle = 0;

            esp_ble_gap_start_scanning(BLE_SCAN_DURATION);
            break;

        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                          esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
    }

    for (int i = 0; i < PROFILE_NUM; i++) {
        if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[i].gattc_if) {
            if (gl_profile_tab[i].gattc_cb) {
                gl_profile_tab[i].gattc_cb(event, gattc_if, param);
            } else {
                gattc_profile_event_handler(event, gattc_if, param);
            }
        }
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan params set, starting scan...");
            current_led_state = LED_STATE_SEARCHING;
            esp_ble_gap_start_scanning(BLE_SCAN_DURATION);
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scan started");
                ble_scanning = true;
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // 检查设备名
                uint8_t adv_name_len = 0;
                uint8_t *adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                    ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

                if (adv_name != NULL && adv_name_len > 0) {
                    if (strncmp((char *)adv_name, target_device_name, adv_name_len) == 0) {
                        ESP_LOGI(TAG, "Found target device: %s", target_device_name);

                        esp_ble_gap_stop_scanning();
                        ble_scanning = false;

                        esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                            param->scan_rst.bda, param->scan_rst.ble_addr_type, true);
                    }
                }

                // 也检查服务UUID
                uint8_t adv_uuid_len = 0;
                uint8_t *adv_uuid = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                    ESP_BLE_AD_TYPE_128SRV_CMPL, &adv_uuid_len);

                if (adv_uuid == NULL) {
                    adv_uuid = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                        ESP_BLE_AD_TYPE_128SRV_PART, &adv_uuid_len);
                }

                if (adv_uuid != NULL && adv_uuid_len == 16) {
                    if (memcmp(adv_uuid, ble_midi_service_uuid, 16) == 0) {
                        ESP_LOGI(TAG, "Found device with MIDI service UUID");
                        esp_ble_gap_stop_scanning();
                        ble_scanning = false;

                        esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                            param->scan_rst.bda, param->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped");
            ble_scanning = false;
            break;

        default:
            break;
    }
}

static void ble_init(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(TAG, "GATTC register callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTC app register failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_ble_gatt_set_local_mtu(128);

    ESP_LOGI(TAG, "BLE GATTC initialized");
}

// ==================== USB初始化 ====================
static void usb_init(void)
{
    ESP_LOGI(TAG, "Initializing USB MIDI device");

    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_fs_configuration,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB MIDI device initialized");
}

// ==================== 主函数 ====================
void app_main(void)
{
    ESP_LOGI(TAG, "BLE MIDI Receiver starting...");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化LED
    led_init();
    led_set_color(50, 0, 0);

    // 创建MIDI队列
    midi_rx_queue = xQueueCreate(MIDI_QUEUE_SIZE, sizeof(midi_packet_t));

    // 初始化USB MIDI
    usb_init();

    // 初始化BLE
    ble_init();

    // 创建任务
    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
    xTaskCreate(usb_midi_task, "usb_midi", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "BLE MIDI Receiver ready");
}
