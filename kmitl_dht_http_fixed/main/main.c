#include "esp_rom_sys.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"   // สำหรับ critical section

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h"

/* ===================== User Config ===================== */
#define WIFI_SSID      "riize"
#define WIFI_PASS      "khing123"
#define BACKEND_URL    "https://kmitl-esp32-backend.onrender.com/api/readings"

/* DHT wiring (ของคุณเป็น DHT11 หัวฟ้า) */
#define DHT_GPIO       GPIO_NUM_4      // 👉 ถ้าย้ายไป IO4 ให้เปลี่ยนเป็น GPIO_NUM_4
#define DHT_IS_DHT22   0                // DHT11 = 0, DHT22 = 1
/* ======================================================= */

static const char *TAG = "KMITL_DHT_HTTP";

/* -------- helper: wait for GPIO level with timeout (us) -------- */
static bool dht_wait_level(int level, uint32_t timeout_us) {
    uint32_t t = 0;
    while (gpio_get_level(DHT_GPIO) != level) {
        if (++t >= timeout_us) return false;
        esp_rom_delay_us(1);
    }
    return true;
}

/* ---------------- Robust DHT reader ---------------- */
/* ปิด interrupt ระหว่างอ่านบิต + ขยาย timeout + ลด threshold */
#define DHT_MAX_RETRIES      7
#define DHT_ONE_THRESHOLD    45    // เดิม 50 → 45us ช่วยรับพัลส์ "1" ที่สั้นลง
#define DHT_WAIT_T_US        300   // เดิม 100–150 → 300us กัน jitter
#define DHT_START_LOW_US_11  22000 // start signal สำหรับ DHT11 ให้ยาวขึ้น

static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t dht_read_raw_once(uint8_t data[5]) {
    // idle high
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(1000);

    // start signal
    gpio_set_level(DHT_GPIO, 0);
#if DHT_IS_DHT22
    esp_rom_delay_us(1300);
#else
    esp_rom_delay_us(DHT_START_LOW_US_11);
#endif

    // release & input (pull-up)
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(40);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    // response (~80us low + ~80us high)
    if (!dht_wait_level(0, 250)) { ESP_LOGW(TAG, "DHT: no response (low)");  return ESP_FAIL; }
    if (!dht_wait_level(1, 250)) { ESP_LOGW(TAG, "DHT: no response (high)"); return ESP_FAIL; }

    // read 40 bits
    uint8_t bits[40] = {0};

    // ปิด interrupt ช่วงอ่าน (~4–5ms)
    taskENTER_CRITICAL(&dht_mux);
    for (int i = 0; i < 40; i++) {
        // low ~50us, แล้ว high ยาวสั้นแทนค่า 0/1
        if (!dht_wait_level(0, DHT_WAIT_T_US)) {
            taskEXIT_CRITICAL(&dht_mux);
            ESP_LOGW(TAG, "DHT: bit %d low timeout", i);
            return ESP_FAIL;
        }
        if (!dht_wait_level(1, DHT_WAIT_T_US)) {
            taskEXIT_CRITICAL(&dht_mux);
            ESP_LOGW(TAG, "DHT: bit %d high timeout", i);
            return ESP_FAIL;
        }

        // วัดความยาวช่วง HIGH
        uint32_t t = 0;
        while (gpio_get_level(DHT_GPIO) == 1) {
            if (++t > 220) break;   // safety (~220us)
            esp_rom_delay_us(1);
        }
        bits[i] = (t > DHT_ONE_THRESHOLD) ? 1 : 0;
    }
    taskEXIT_CRITICAL(&dht_mux);

    memset(data, 0, 5);
    for (int i = 0; i < 40; i++) {
        data[i/8] <<= 1;
        data[i/8] |= bits[i] & 0x01;
    }
    return ESP_OK;
}

static bool dht_frame_looks_empty(const uint8_t d[5]) {
    return (d[0] == 0 && d[1] == 0 && d[2] == 0 && d[3] == 0 && d[4] == 0);
}

static esp_err_t dht_read(float *humidity, float *temperature) {
    uint8_t d[5];
    for (int tryi = 0; tryi < DHT_MAX_RETRIES; tryi++) {
        // รีเซ็ตไลน์ก่อนลองใหม่เล็กน้อย
        gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(DHT_GPIO, 1);
        esp_rom_delay_us(1000);

        if (dht_read_raw_once(d) == ESP_OK) {
            if (dht_frame_looks_empty(d)) {
                ESP_LOGW(TAG, "DHT empty frame: 00 00 00 00 00 (try=%d)", tryi + 1);
                vTaskDelay(pdMS_TO_TICKS(40));
                continue;
            }
            uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
            if (sum != d[4]) {
                ESP_LOGW(TAG, "DHT checksum error try=%d: got=%u expected=%u", tryi+1, d[4], sum);
            } else {
#if DHT_IS_DHT22
                int16_t raw_h = ((int16_t)d[0] << 8) | d[1];
                int16_t raw_t = ((int16_t)d[2] << 8) | d[3];
                if (raw_t & 0x8000) raw_t = -(raw_t & 0x7FFF);
                *humidity = raw_h / 10.0f;
                *temperature = raw_t / 10.0f;
#else
                // DHT11: d[0]=RH int, d[2]=T int
                *humidity    = (float)d[0];
                *temperature = (float)d[2];
#endif
                ESP_LOGI(TAG, "raw bytes: %02x %02x %02x %02x %02x", d[0], d[1], d[2], d[3], d[4]);

                if ((*humidity == 0.0f && *temperature == 0.0f)) {
                    ESP_LOGW(TAG, "DHT got 0/0 reading; retrying...");
                    vTaskDelay(pdMS_TO_TICKS(40));
                    continue;
                }
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    return ESP_FAIL;
}

/* ---------------- Wi-Fi + HTTP ---------------- */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi start: connecting to SSID=%s", WIFI_SSID);
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "WiFi disconnected: reason=%d", e ? e->reason : -1);
        esp_wifi_connect(); // retry
        if (s_wifi_event_group) xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        if (s_wifi_event_group) xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // ตั้งประเทศเป็น TH เพื่อสแกน ch.1–13
    wifi_country_t country = {
        .cc = "TH", .schan = 1, .nchan = 13, .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    // กว้างสุดเพื่อความเข้ากันได้ (Open/WPA/WPA2/WPA3)
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t http_post_reading(float temperature, float humidity) {
    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);

    esp_http_client_config_t cfg = {
        .url = BACKEND_URL,
        .crt_bundle_attach = esp_crt_bundle_attach,  // ต้องเปิด CA bundle ใน menuconfig
        .timeout_ms = 8000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "User-Agent", "esp32-dht11/1.0");
    esp_http_client_set_post_field(client, payload, strlen(payload));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST -> status=%d, len=%d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        ESP_LOGW(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    return err;
}

/* ---------------- Main Task ---------------- */
static void sensor_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));           // ให้เซนเซอร์พร้อมหลังบูต
    const TickType_t interval = pdMS_TO_TICKS(3000);
    TickType_t last = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last, interval);

        float t = 0, h = 0;
        esp_err_t err = dht_read(&h, &t);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "DHT OK  T=%.1f°C  H=%.1f%%", t, h);

            // กรองค่าผิดปกติก่อนส่ง (กัน 0.0/ขยะ)
            if (h >= 1 && h <= 100 && t > -10 && t < 60) {
                EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
                if (bits & WIFI_CONNECTED_BIT) {
                    http_post_reading(t, h);
                } else {
                    ESP_LOGW(TAG, "Skip HTTP: not connected");
                }
            } else {
                ESP_LOGW(TAG, "Skip HTTP: invalid reading T=%.1f H=%.1f", t, h);
            }
        } else {
            ESP_LOGW(TAG, "DHT read error: %s", esp_err_to_name(err));
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    // ตั้งขา DHT เป็น input+pull-up (ฟังก์ชันอ่านจะสลับโหมดเอง)
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << DHT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    // รอ Wi-Fi สูงสุด 15 วินาที (จะ retry ต่อเบื้องหลัง)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi connect timeout. Will keep retrying in background.");
    }

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
