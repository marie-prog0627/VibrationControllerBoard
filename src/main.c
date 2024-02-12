#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "unistd.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "string.h"

#define TIMER_PERIOD pdMS_TO_TICKS(5000) // 5000ms = 5秒
#define SID 0x0121
#define GATTS_TAG "BLE_GATTS"
#define MY_DEVICE_NAME "VLE1"
#define GAT_NUM_HANDLE 4
#define GAT_UUID 0x0B1B

static uint16_t id_conn = 0xFFFF; // 接続IDの初期値（未接続を示す）
static esp_gatt_if_t if_gatt = ESP_GATT_IF_NONE; // インターフェースIDの初期値
static uint16_t char_handle = 0; // 特性ハンドルの初期値
static uint16_t gat_uuids = 0xFFFF;
static uint8_t char_value[512]; // 受信データ

esp_bt_uuid_t vib_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = GAT_UUID,
};

esp_gatt_srvc_id_t service_id = {
    .id = {
        .inst_id = 0x01,
            .uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = SID,},
            },
        },
    .is_primary = true,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false, // スキャン応答にこのアドバタイジングデータを使用しない
    .include_name = true, // アドバタイジングパケットにデバイス名を含める
    .include_txpower = false, // 送信電力レベルを含めない
    .manufacturer_len = 0, // メーカーデータは使用しない
    .p_manufacturer_data =  NULL, // メーカーデータポインタ
    .service_data_len = 0, // サービスデータは使用しない
    .p_service_data = NULL, // サービスデータポインタ
    .service_uuid_len = 0, // サービスUUIDは使用しない
    .p_service_uuid = NULL, // サービスUUIDポインタ
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

esp_attr_value_t gatt_char_src = {
    .attr_max_len = 512, // 最大値を適宜設定
    .attr_len     = 0,   // 初期値の長さ
    .attr_value   = NULL, // 初期値
};

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI("GAP", "Advertising.");
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, 
                                esp_gatt_if_t ifid, 
                                esp_ble_gatts_cb_param_t *param) {

    ESP_LOGI("GATT", "%d", event);

    switch (event) {
        case ESP_GATTS_REG_EVT:
            if_gatt = ifid;
            esp_ble_gatts_create_service(if_gatt, &service_id, GAT_NUM_HANDLE);
            ESP_LOGI(GATTS_TAG, "Registered.\n");
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "Created.\n");
            esp_ble_gatts_add_char(param->create.service_handle, &vib_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   &gatt_char_src, NULL);
            esp_ble_gatts_start_service(param->create.service_handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "Started.\n");
            break;
        case ESP_GATTS_CONNECT_EVT:
            id_conn = param->connect.conn_id;
            ESP_LOGI(GATTS_TAG, "Connected.\n");
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            char_handle = param->add_char.attr_handle;
            gat_uuids = param->add_char.char_uuid.uuid.uuid16;
            ESP_LOGI(GATTS_TAG, "Characteristic Added. id: %d, uuid %d \n", char_handle, gat_uuids);
            break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // 実際の書き込みデータを取得
                memcpy(char_value, param->write.value, param->write.len);
                // 受信したデータをシリアル出力にprint
                ESP_LOGI(GATTS_TAG, "Received data: %.*s", param->write.len, char_value);
            }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Disconnected.\n");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

void timer_callback(TimerHandle_t xTimer) {
    char message[] = "Hello from ESP32!";
    esp_ble_gatts_send_indicate(if_gatt, id_conn, char_handle,
                                sizeof(message), (uint8_t *)message, false);
}

void bt_initialize() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);

    esp_ble_gatts_app_register(SID);

    esp_ble_gap_set_device_name(MY_DEVICE_NAME);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gap_config_adv_data(&adv_data);
}

void app_main(void) {

    sleep(10);

    bt_initialize();

    // TimerHandle_t timer = xTimerCreate("msgTimer", TIMER_PERIOD, pdTRUE, (void *)0, timer_callback);
    // xTimerStart(timer, 0);
}