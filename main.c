#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <sys/param.h>
#include "rom/ets_sys.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#define PORT 6454
#define ART_POLL 0x2000
#define ART_DMX 0x5000
#define ART_DMX_BUF_MAXSZ 530
#define ART_DMX_HEADER_SZ 18
#define ART_PROTOCOL_VERSION_HI 0
#define ART_PROTOCOL_VERSION_LO 14
#define DMX_UNIVERSE1 0
#define UART_BUF_SZ 1024
#define UART_DMX_TX_PIN 17

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG_ART  = "ARTNET";
static const char *TAG_UART = "UART  ";
static const char *TAG_WIFI = "WIFI  ";
static const char *TAG_UDP  = "UDP   ";

static int s_retry_num = 0;

static uint8_t buf_udp_rx[1024];
static uint8_t buf_dmx_universe1[513];

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_WIFI,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }
}

static void udp_server_task(void *pvParameters)
{
    struct sockaddr_in dest_addr;

    while (1) {
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG_UDP, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG_UDP, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG_UDP, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG_UDP, "Socket bound, port %d", PORT);

        while (1) {
            //ESP_LOGI(TAG_UDP, "Waiting for data");
            int recv_len = recvfrom(sock, buf_udp_rx, sizeof(buf_udp_rx), 0, NULL, NULL);

            if (recv_len < 0) {
                ESP_LOGE(TAG_UDP, "recvfrom failed: errno %d", errno);
                break;
            }

            //ESP_LOGD(TAG_UDP, "Received %d", recv_len);

            if(recv_len > ART_DMX_BUF_MAXSZ || recv_len < ART_DMX_HEADER_SZ) {
                ESP_LOGW(TAG_ART, "Art-DMX-Packet has wrong size");
                continue;
            }

            uint16_t opcode = buf_udp_rx[8] | buf_udp_rx[9] << 8;
            uint8_t protocol_version_hi = buf_udp_rx[10];
            uint8_t protocol_version_lo = buf_udp_rx[11];
            //uint8_t sequence = buf_udp_rx[12];
            //uint8_t physical = buf_udp_rx[13];
            uint16_t universe = buf_udp_rx[14] | buf_udp_rx[15] << 8;
            uint16_t data_len = buf_udp_rx[16] << 8 | buf_udp_rx[17];
            uint8_t* data = buf_udp_rx + 18;

            if(
                opcode != ART_DMX ||
                protocol_version_hi != ART_PROTOCOL_VERSION_HI ||
                protocol_version_lo != ART_PROTOCOL_VERSION_LO ||
                data_len > 512
            ) {
                ESP_LOGW(TAG_ART, "Art-DMX-Packet has wrong format");
                continue;
            }

            if(universe != DMX_UNIVERSE1) {
                continue;
            }

            memcpy(buf_dmx_universe1 + 1, data, data_len);
        }

        if (sock != -1) {
            ESP_LOGE(TAG_UDP, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void dmx_sender_task(void *pvParameters) {

    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 250000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_DMX_TX_PIN, -1, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_FIFO_LEN*2, UART_BUF_SZ, 0, NULL, 0));

    
    BaseType_t xWasDelayed;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;

    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );

        if(xWasDelayed == pdFALSE) {
            ESP_LOGW(TAG_UART, "Missed dmx send deadline!");
        }

        uart_set_line_inverse(uart_num, UART_SIGNAL_TXD_INV);
        ets_delay_us(100);
        uart_set_line_inverse(uart_num, UART_SIGNAL_INV_DISABLE);
        ets_delay_us(10);

        buf_dmx_universe1[0] = 0;
        uart_write_bytes(uart_num, buf_dmx_universe1, sizeof(buf_dmx_universe1));
        ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, portMAX_DELAY));

        //ESP_LOGD(TAG_UART, "Ch1: %u, Ch2: %u", buf_dmx_universe1[1], buf_dmx_universe1[2]);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG_WIFI, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    xTaskCreate(dmx_sender_task, "dmx_sender", 4096, NULL, 6, NULL); //Keeping DMX-Timing is more important than receiving new data
}
