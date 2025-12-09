/**
 * ESP32-S3 BLE MIDI Transmitter
 *
 * USB Host接收电吹管MIDI信号，通过BLE MIDI发送给接收端
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

// USB Host
#include "usb/usb_host.h"

// BLE
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// LED
#include "led_strip.h"
#include "driver/rmt_tx.h"

static const char *TAG = "BLE_MIDI_TX";

// ==================== 配置 ====================
#define WS2812_GPIO         48      // WS2812 LED GPIO (根据实际情况修改)
#define USB_HOST_TASK_PRIORITY  5
#define BLE_MIDI_TASK_PRIORITY  4

// ==================== LED 控制 ====================
static led_strip_handle_t led_strip;

typedef enum {
    LED_STATE_DISCONNECTED,     // 红色 - 未连接
    LED_STATE_SEARCHING,        // 蓝色闪烁 - 搜索中
    LED_STATE_CONNECTED,        // 绿色 - 已连接
    LED_STATE_DATA_TX           // 青色闪烁 - 数据传输
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
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
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
                led_set_color(50, 0, 0);  // 红色
                break;

            case LED_STATE_SEARCHING:
                if (blink_cnt % 2 == 0) {
                    led_set_color(0, 0, 50);  // 蓝色
                } else {
                    led_set_color(0, 0, 0);
                }
                break;

            case LED_STATE_CONNECTED:
                if (midi_activity) {
                    led_set_color(0, 50, 50);  // 青色 - 有数据
                    midi_activity = false;
                } else {
                    led_set_color(0, 50, 0);  // 绿色
                }
                break;

            case LED_STATE_DATA_TX:
                led_set_color(0, 50, 50);  // 青色
                current_led_state = LED_STATE_CONNECTED;
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ==================== BLE MIDI ====================
// BLE MIDI Service UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
// BLE MIDI Characteristic UUID: 7772E5DB-3868-4112-A1A9-F2669D106BF3

static uint8_t ble_midi_service_uuid[16] = {
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

static uint8_t ble_midi_char_uuid[16] = {
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};

#define GATTS_SERVICE_UUID      0x00FF
#define GATTS_CHAR_UUID         0xFF01
#define GATTS_NUM_HANDLE        4
#define DEVICE_NAME             "BLE-MIDI-TX"

static uint16_t ble_midi_handle_table[GATTS_NUM_HANDLE];
static uint8_t adv_service_uuid128[32] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    // BLE MIDI Service UUID
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = ble_midi_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static bool ble_connected = false;
static uint16_t ble_conn_id = 0;
static esp_gatt_if_t ble_gatts_if = ESP_GATT_IF_NONE;
static uint16_t ble_midi_char_handle = 0;
static bool ble_notify_enabled = false;

// MIDI发送队列
#define MIDI_QUEUE_SIZE 128
static QueueHandle_t midi_tx_queue;

typedef struct {
    uint8_t data[4];
    uint8_t len;
} midi_packet_t;

// BLE MIDI时间戳
static uint16_t get_ble_midi_timestamp(void)
{
    uint32_t ms = esp_log_timestamp() & 0x1FFF;
    return 0x8000 | ms;
}

// 发送MIDI数据通过BLE
static void send_ble_midi(uint8_t *midi_data, uint8_t len)
{
    if (!ble_connected || !ble_notify_enabled || ble_midi_char_handle == 0) {
        return;
    }

    // BLE MIDI包格式: [Header, Timestamp, MIDI...]
    uint8_t ble_packet[20];
    uint16_t ts = get_ble_midi_timestamp();

    ble_packet[0] = (ts >> 7) | 0x80;  // Header byte
    ble_packet[1] = (ts & 0x7F) | 0x80; // Timestamp byte

    memcpy(&ble_packet[2], midi_data, len);

    esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, ble_midi_char_handle,
                                 len + 2, ble_packet, false);

    midi_activity = true;
}

// GATT Server回调
enum {
    IDX_SVC,
    IDX_CHAR,
    IDX_CHAR_VAL,
    IDX_CHAR_CFG,
};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ |
                                                    ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
                                                    ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t midi_char_value[128] = {0};
static uint8_t midi_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_db[GATTS_NUM_HANDLE] = {
    // Service Declaration
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         16, 16, ble_midi_service_uuid}
    },
    // Characteristic Declaration
    [IDX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         1, 1, (uint8_t *)&char_prop_read_write_notify}
    },
    // Characteristic Value
    [IDX_CHAR_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, ble_midi_char_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         128, 0, midi_char_value}
    },
    // Client Characteristic Configuration
    [IDX_CHAR_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         2, 2, midi_ccc}
    },
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server register");
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATTS_NUM_HANDLE, 0);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                memcpy(ble_midi_handle_table, param->add_attr_tab.handles,
                       sizeof(ble_midi_handle_table));
                ble_midi_char_handle = ble_midi_handle_table[IDX_CHAR_VAL];
                esp_ble_gatts_start_service(ble_midi_handle_table[IDX_SVC]);
                ESP_LOGI(TAG, "Create attribute table success, char handle: %d",
                         ble_midi_char_handle);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "BLE connected, conn_id: %d", param->connect.conn_id);
            ble_connected = true;
            ble_conn_id = param->connect.conn_id;
            ble_gatts_if = gatts_if;
            current_led_state = LED_STATE_CONNECTED;

            // 更新连接参数以获得更低延迟
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x0006;  // 7.5ms
            conn_params.min_int = 0x0006;  // 7.5ms
            conn_params.timeout = 400;     // 4s
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "BLE disconnected");
            ble_connected = false;
            ble_notify_enabled = false;
            current_led_state = LED_STATE_SEARCHING;
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == ble_midi_handle_table[IDX_CHAR_CFG]) {
                if (param->write.len == 2) {
                    uint16_t ccc_val = param->write.value[0] | (param->write.value[1] << 8);
                    ble_notify_enabled = (ccc_val == 0x0001);
                    ESP_LOGI(TAG, "Notify %s", ble_notify_enabled ? "enabled" : "disabled");
                }
            }
            break;

        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            current_led_state = LED_STATE_SEARCHING;
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started");
            }
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

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_ble_gatt_set_local_mtu(256);

    ESP_LOGI(TAG, "BLE MIDI initialized");
}

// ==================== USB Host MIDI ====================
#define USB_MIDI_SUBCLASS       0x03
#define USB_MIDI_PROTOCOL       0x00

typedef struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    uint8_t dev_addr;
    bool device_connected;
    uint8_t ep_in;
    uint8_t ep_out;
    uint16_t ep_in_mps;
    usb_transfer_t *in_xfer;
    uint8_t iface_num;  // 新增：MIDI接口号
} usb_midi_driver_t;

static usb_midi_driver_t usb_midi_driver;

// USB MIDI事件处理
static void usb_host_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    usb_midi_driver_t *driver = (usb_midi_driver_t *)arg;

    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device connected, addr: %d", event_msg->new_dev.address);
            driver->dev_addr = event_msg->new_dev.address;
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB device disconnected");
            driver->device_connected = false;
            driver->dev_hdl = NULL;
            current_led_state = LED_STATE_DISCONNECTED;
            break;
        default:
            break;
    }
}

// USB MIDI传输回调
static void usb_midi_transfer_cb(usb_transfer_t *transfer)
{
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        // 解析USB MIDI包 (每个包4字节)
        for (int i = 0; i < transfer->actual_num_bytes; i += 4) {
            uint8_t *pkt = &transfer->data_buffer[i];
            uint8_t cin = pkt[0] & 0x0F;  // Code Index Number

            // 跳过无效包
            if (cin == 0) continue;

            // 根据CIN确定MIDI消息长度
            uint8_t midi_len = 0;
            switch (cin) {
                case 0x5: case 0xF: // 单字节
                    midi_len = 1;
                    break;
                case 0x2: case 0x6: case 0xC: case 0xD: // 双字节
                    midi_len = 2;
                    break;
                case 0x3: case 0x4: case 0x7: case 0x8:
                case 0x9: case 0xA: case 0xB: case 0xE: // 三字节
                    midi_len = 3;
                    break;
                default:
                    continue;
            }

            // 发送MIDI数据
            if (midi_len > 0) {
                ESP_LOGI(TAG, "MIDI: %02X %02X %02X", pkt[1], pkt[2], pkt[3]);
                send_ble_midi(&pkt[1], midi_len);
            }
        }
    }

    // 重新提交传输请求
    if (usb_midi_driver.device_connected && usb_midi_driver.dev_hdl) {
        usb_host_transfer_submit(transfer);
    }
}

// 查找MIDI接口和端点 - 增强版处理大配置描述符
static esp_err_t find_midi_interface(const usb_config_desc_t *config_desc,
                                      uint8_t *ep_in, uint8_t *ep_out,
                                      uint16_t *ep_in_mps,uint8_t *iface_num)
{
    const uint8_t *p = (const uint8_t *)config_desc;
    const uint8_t *end = p + config_desc->wTotalLength;

    uint8_t current_iface_class = 0;
    uint8_t current_iface_subclass = 0;
    bool in_audio_streaming = false;

    *ep_in = 0;
    *ep_out = 0;

    while (p < end) {
        uint8_t desc_len = p[0];
        uint8_t desc_type = p[1];

        if (desc_len == 0) break;

        if (desc_type == USB_B_DESCRIPTOR_TYPE_INTERFACE && desc_len >= 9) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
            current_iface_class = intf->bInterfaceClass;
            current_iface_subclass = intf->bInterfaceSubClass;

            // Audio Class, MIDIStreaming subclass
            if (current_iface_class == USB_CLASS_AUDIO &&
                current_iface_subclass == USB_MIDI_SUBCLASS) {
                in_audio_streaming = true;
                *iface_num = intf->bInterfaceNumber;  // 保存接口号
                ESP_LOGI(TAG, "Found MIDI Streaming interface: %d", intf->bInterfaceNumber);
            } else {
                in_audio_streaming = false;
            }
        }
        else if (desc_type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && desc_len >= 7 && in_audio_streaming) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)p;

            if ((ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK ||
                (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_INT) {

                if (ep->bEndpointAddress & 0x80) {
                    *ep_in = ep->bEndpointAddress;
                    *ep_in_mps = ep->wMaxPacketSize;
                    ESP_LOGI(TAG, "Found MIDI IN EP: 0x%02X, MPS: %d", *ep_in, *ep_in_mps);
                } else {
                    *ep_out = ep->bEndpointAddress;
                    ESP_LOGI(TAG, "Found MIDI OUT EP: 0x%02X", *ep_out);
                }
            }
        }

        p += desc_len;
    }

    return (*ep_in != 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

// 打开MIDI设备
static esp_err_t open_midi_device(usb_midi_driver_t *driver)
{
    esp_err_t ret;

    // 打开设备
    ret = usb_host_device_open(driver->client_hdl, driver->dev_addr, &driver->dev_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取配置描述符
    const usb_config_desc_t *config_desc;
    ret = usb_host_get_active_config_descriptor(driver->dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(ret));
        usb_host_device_close(driver->client_hdl, driver->dev_hdl);
        return ret;
    }

    ESP_LOGI(TAG, "Config descriptor total length: %d bytes", config_desc->wTotalLength);

    // 查找MIDI端点
    ret = find_midi_interface(config_desc, &driver->ep_in, &driver->ep_out, &driver->ep_in_mps,&driver->iface_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No MIDI interface found");
        usb_host_device_close(driver->client_hdl, driver->dev_hdl);
        return ret;
    }

    // 声明MIDI接口
    ret = usb_host_interface_claim(driver->client_hdl, driver->dev_hdl,
                                    driver->iface_num, 0);  // 使用找到的接口号
    if (ret != ESP_OK) {
        // 尝试接口0
        ret = usb_host_interface_claim(driver->client_hdl, driver->dev_hdl, 0, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to claim interface %d: %s", driver->iface_num, esp_err_to_name(ret));
            usb_host_device_close(driver->client_hdl, driver->dev_hdl);
            return ret;
        }
    }

    // 分配IN传输
    ret = usb_host_transfer_alloc(driver->ep_in_mps, 0, &driver->in_xfer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(ret));
        return ret;
    }

    driver->in_xfer->device_handle = driver->dev_hdl;
    driver->in_xfer->bEndpointAddress = driver->ep_in;
    driver->in_xfer->callback = usb_midi_transfer_cb;
    driver->in_xfer->context = driver;
    driver->in_xfer->num_bytes = driver->ep_in_mps;

    // 提交IN传输
    ret = usb_host_transfer_submit(driver->in_xfer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit transfer: %s", esp_err_to_name(ret));
        usb_host_transfer_free(driver->in_xfer);
        return ret;
    }

    driver->device_connected = true;
    current_led_state = LED_STATE_CONNECTED;
    ESP_LOGI(TAG, "MIDI device opened successfully");

    return ESP_OK;
}

// USB Host任务
static void usb_host_task(void *arg)
{
    // 安装USB Host
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed");

    // 注册客户端
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_host_client_event_cb,
            .callback_arg = &usb_midi_driver,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &usb_midi_driver.client_hdl));

    current_led_state = LED_STATE_SEARCHING;

    while (1) {
        // 处理USB Host事件
        usb_host_lib_handle_events(0, NULL);
        usb_host_client_handle_events(usb_midi_driver.client_hdl, 0);

        // 如果有新设备但还未打开
        if (usb_midi_driver.dev_addr != 0 && !usb_midi_driver.device_connected) {
            esp_err_t ret = open_midi_device(&usb_midi_driver);
            if (ret != ESP_OK) {
                usb_midi_driver.dev_addr = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 主函数 ====================
void app_main(void)
{
    ESP_LOGI(TAG, "BLE MIDI Transmitter starting...");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化LED
    led_init();
    led_set_color(50, 0, 0);  // 红色 - 启动中

    // 创建MIDI队列
    midi_tx_queue = xQueueCreate(MIDI_QUEUE_SIZE, sizeof(midi_packet_t));

    // 初始化BLE
    ble_init();

    // 创建任务
    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
    xTaskCreate(usb_host_task, "usb_host", 4096, NULL, USB_HOST_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "BLE MIDI Transmitter ready");
}
