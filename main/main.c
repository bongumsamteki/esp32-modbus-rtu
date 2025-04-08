#include <math.h>
#include "modbus_rtu_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#include "iot_servo.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_camera.h"

#define SERVER_ADDRESS "192.168.13.232"
#define SERVER_PORT    8080

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define MAX_RETRY 10

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#define CONFIG_XCLK_FREQ 5000000 

#define SOCKET_TAG "Socket: "

struct sockaddr_in serv_addr;
int client_socket;

static EventGroupHandle_t wifi_events;
modbus_rtu_query_frame qframe_struct;

static int retry_cnt = 0;

uint8_t slave_id = 25, connected = 0;

static esp_err_t init_camera(void)
{
    camera_config_t camera_config = {
        .pin_pwdn  = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = CONFIG_XCLK_FREQ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 10,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };//CAMERA_GRAB_LATEST. Sets when buffers should be filled
    
    esp_err_t err = esp_camera_init(&camera_config);
    
    if (err != ESP_OK)
    {
        return err;
    }

    sensor_t *sens = esp_camera_sensor_get();
    
    sens->set_awb_gain(sens, 1);
    sens->set_wb_mode(sens, 1);
    
    return ESP_OK;
}
/*
static esp_err_t handle_http_event(esp_http_client_event_t *http_event)
{
    switch(http_event -> event_id)
    {
        case HTTP_EVENT_ON_DATA:
            const char *data = (const char *)http_event->data;
            int positions[4];

            char *token = strtok(data, ",");
            int idx = 0;

            while(token != NULL && idx < 4)
            {
                positions[idx] = atoi(token);
                token = strtok(NULL, ",");
                idx++;
            }

            float x0 = (float)positions[0] - 20;
            float angle = 90 + (atan(x0/480.00)*(180/M_PI));

            ESP_LOGI("POST response: ", "%f %f", x0, angle);

            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, angle);
            vTaskDelay(1000);
            break;
        case HTTP_EVENT_DISCONNECTED:
            break;
        case HTTP_EVENT_HEADERS_SENT:

        case HTTP_EVENT_ON_CONNECTED:
            break;
        default:
            break;    
    }

    return ESP_OK;
}
*/

/*
static void request_page(void *arg)
{
    esp_http_client_config_t config = {
        .method = HTTP_METHOD_POST,
        .url = "http://192.168.13.232:8080",
        .event_handler = handle_http_event
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    
    camera_fb_t *fb = NULL;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;

    fb = esp_camera_fb_get();

    if(fb)
    {
        if(fb->format != PIXFORMAT_JPEG){
            int jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE("Camera: ", "JPEG compression failed");
                esp_camera_fb_return(fb);
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }
        
        esp_http_client_set_post_field(client, (char *)_jpg_buf, _jpg_buf_len);

        if(esp_http_client_perform(client) != ESP_OK)
        {
            ESP_LOGE("HTTP: ", "http request failed!");
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}
*/

static void handle_wifi_connection(void *, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if(retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
            ESP_LOGI("WiFi: ", "wifi connect retry: %d", retry_cnt);
        }
        else
        {
            xEventGroupSetBits(wifi_events, WIFI_FAIL_BIT);
        }
    }
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        // ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WiFi: ", "Got IP");
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void init_wifi(void)
{ 
    wifi_events = xEventGroupCreate();

    /*
    memset(&qframe_struct, 0, sizeof(qframe_struct));

    qframe_struct.slave_id = slave_id;
    qframe_struct.function_code = READ_COILS;
    qframe_struct.start_address = COILS_BASE;
    qframe_struct.quantity = 80;

    uint8_t err_code;
    uint8_t ssid_buffer[5 + (int)ceil(qframe_struct.quantity/8)];
    uint8_t pass_buffer[5 + (int)ceil(qframe_struct.quantity/8)];

    while((err_code = modbus_rtu_read(&qframe_struct, ssid_buffer)) != NO_ERROR);
    
    uint8_t wifi_ssid[(int)ssid_buffer[2]];
    strncpy((char *)wifi_ssid, (char *)&ssid_buffer[3], (size_t)ssid_buffer[2]);

    qframe_struct.start_address = (COILS_BASE + 80);

    while((err_code = modbus_rtu_read(&qframe_struct, pass_buffer)) != NO_ERROR);

    uint8_t wifi_password[pass_buffer[2]];

    strncpy((char *)wifi_password, (char *)&pass_buffer[3], (size_t)pass_buffer[2]);
*/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "RossaneDut",
            .password = "DutRoss2023@",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            }
        }
    };
/*
    strncpy((char *)&wifi_config.sta.ssid, (char *)wifi_ssid, str_lenght(wifi_ssid));
    strncpy((char *)&wifi_config.sta.password, (char *)wifi_password, str_lenght(wifi_password));
*/
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &handle_wifi_connection, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &handle_wifi_connection, NULL);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if(bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("WiFi: ", "Connected");
    }
    else
    {
        // connection failed
        ESP_LOGI("WiFi: ", "Connection Failed!");
    }
}

static void init_servo(void)
{
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_1,
        .channels = {
            .servo_pin = {
                GPIO_NUM_13
            },
            .ch = {
                LEDC_CHANNEL_1
            }
        },
        .channel_number = 1
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

static void init_client_socket(void)
{
    serv_addr.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    serv_addr.sin_port = htons(SERVER_PORT);
    serv_addr.sin_family = AF_INET;

    client_socket = socket(AF_INET, SOCK_STREAM, 0);

    if(client_socket < -1) 
    {
        ESP_LOGE(SOCKET_TAG, "Creating socket failed!");
        vTaskDelay(1000/portTICK_PERIOD_MS);
        return;
    }
    else
    {
        ESP_LOGI(SOCKET_TAG, "Socket created");
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    if(nvs_flash_init() != ESP_OK)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();

    // modbus_uart_init();

    init_servo();
    init_camera();
    init_wifi();

    init_client_socket();

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 90);

    int client_conn = connect(client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    ESP_LOGI(SOCKET_TAG, "Connection: %d", client_conn);

    if(!client_conn)
    {
        ESP_LOGI(SOCKET_TAG, "Connected to server");

        while(1)
        {
            camera_fb_t *fb = NULL;

            uint8_t *_jpg_buf;
            size_t _jpg_buf_len;

            fb = esp_camera_fb_get();

            if(fb)
            {
                if(fb->format != PIXFORMAT_JPEG){
                    int jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);

                    if(!jpeg_converted)
                    {
                        ESP_LOGE("Camera: ", "JPEG compression failed");
                        esp_camera_fb_return(fb);
                    }
                } 
                else 
                {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }

                char buffer[25];
                char data_len[5];
                
                sprintf(buffer, "{\"imageSize\": %d}", _jpg_buf_len);
                
                int bytes_sent = send(client_socket, buffer, strlen(buffer), 0);
                ESP_LOGI(SOCKET_TAG, "Bytes sent: %d\n", bytes_sent);
                
                bytes_sent = send(client_socket, _jpg_buf, _jpg_buf_len, 0);

                int bytes_recieved = recv(client_socket, data_len, sizeof(data_len), 0);

                ESP_LOGI(SOCKET_TAG, "Bytes sent: %d\tBytes Recieved: %d\nBytes buffered: %d, Data: %s", bytes_sent, bytes_recieved, _jpg_buf_len, _jpg_buf);
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }
            else
            {
                ESP_LOGE("Camera: ", "Camera failed");
            }

            esp_camera_fb_return(fb);
        }
    }
    else
    {
        ESP_LOGE(SOCKET_TAG, "Failed to connect to server");
    }

    close(client_socket);
    
    /*
    while(1)
    {
        wifi_ap_record_t ap_records;
        if(esp_wifi_sta_get_ap_info(&ap_records) == ESP_OK)
        {
            xTaskCreate(&request_page, "http_req", 5*configMINIMAL_STACK_SIZE, NULL, 5, NULL);
            vTaskDelay(2000/portTICK_PERIOD_MS);
        }
    }
    */
}