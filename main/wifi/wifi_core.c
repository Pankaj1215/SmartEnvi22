/*
 *  Copyright 2020. Lucidtron Philippines. All Rights Reserved. Permission
 *  to use, copy, modify, and distribute the software and its documentation
 *  for education, research and non-for-profit purposes. without fee and without
 *  signed licensing agreement, is hereby granted, provided that the above
 *  copyright notice, this paragraph and the following two paragraphs appear
 *  in all copies, modification, and distributions. Contact The offfice of
 *  Lucidtron Philippines Inc. Unit D 2nd Floor GMV-Winsouth 2 104 
 *  East Science Avenue, Laguna Technopark, Biñan City, Laguna 4024.
 *  lucidtron@lucidtron.com (049) 302 7001 for commercial and licensing 
 *  opportunities.
 *
 *  IN NO EVENT SHALL LUCIDTRON BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 *  SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, 
 *  ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF 
 *  LUCIDTRON HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  LUCIDTRON SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO. THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
 *  PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
 *  PROVIDED HEREUNDER IS PROVIDED "AS IS". LUCIDTRON HAS NO OBLIGATION TO PROVIDE
 *  MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENS, OR MODIFICATIONS.
 *
 */

/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include "non_volatile_lib.h" 
#include "lucidtron_core.h"
#include "wifi_core.h"

#include "communication_server.h"
#include "app.h"

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

#ifdef P_TESTING

char uniqueDeviceID[12];
struct comm_wifi comm_wifi_dev; // New Added for mac adress testing

time_t keepAlive_ms = 0;
unsigned char keepAliveSendDataToAWS = 0;
#define KEEP_ALIVE_DATA__PACKET_DUR_MS 60000

// char replybuff[500];  //  newadded for ack
unsigned char maxTemperatureThresholdReachedWarning;
unsigned char minTemperatureThresholdReachedWarning;

char replybuff[150];  //  newadded for ack
int commandReceived_SendAck;  //  newadded for ack

int oneTimeRegistrationPacketToAWS = 0;
unsigned char keepAliveFlag;

unsigned char CommandAck;

#define TimeZoneAdded

#ifdef TimeZoneAdded
extern const int timezone_offset_list_min[];
#include "clock/clock.h"
void saveTimeZone(void);
#endif


#define subscribePublishTopic  // To activate the pub sub functionality code
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define TCP_Server_Code
#ifdef TCP_Server_Code
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"   // Need to add in the CMakeLists.txt
#include "nvs.h"

void initFlash();
void writeEEPROM();
int cmpString(char *a,char *b);
void saveDetails(char *buffer);
void readEEPROM();
void initSoftAP();

#endif


#ifdef subscribePublishTopic

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"

#include "common.h"  // new added for connected bit // p17Sept20
#include "heater.h"  // new Added fot Heater OnOff functions..

#define Payloading_main_Firmware
#ifdef Payloading_main_Firmware
#include "communication_msg_handler.h"  // New added for payload testing
#endif

unsigned char uchTopic_Set_temp_subscribe_status = 0;
unsigned char uchTopic_HeaterParameter_Publish_status = 1;  // By Default publish Heater Parameter
unsigned char uchTopic_HeaterDetails_Publish_status = 1;

unsigned char uchTopic_HeaterON_Publish_status = 0;
unsigned char uchTopic_HeaterOFF_Publish_status = 0;
static const char *TAG = "example";
// unsigned char uchHeaterOnOffStatus = 0;
static int s_retry_num = 0;
int HeaterOnOffStatus = 0;
int temperatureSetByCMD = 0;  // Added only for Testing
unsigned char HeaterOnByCMD = 0;// By Default Heater OFF
#endif
#endif



/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER              "192.168.254.127"
#define WEB_PORT                "3000"
#define WEB_URL                 "http://192.168.254.127/"
#define WEB_SERVER_PORT         80

const static char http_html_header[] =
    "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";
const static char http_html_reply[] =
    "<!DOCTYPE html>"
    "lucidtron no reply\r\n\r\n";

void http_web_server();

int (*wifi_msg_handler_callback)(char* msg, char* ret_reply) = NULL;
int (*web_server_msg_handler)(char* msg, char* response, int res_len) = NULL;
int (*wifi_conn_stat_callback)(int conn_stat) = NULL;
int esp32_wifi_status = ESP32_WIFI_UNKNOWN;
int web_server_status = WEB_SVR_STAT_UNKNOWN;

// char username[50],password[50],id[11],locID[20],name[20],timeZone[30];
char username[32],password[64],id[11],locID[50],name[20],timeZone[20]; // Time Zone size changed to 20 on 28Oct2020

wifi_config_t global_wifi_config;

static uint8_t ap_mac[6];
static uint8_t sta_mac[6];

static bool wifi_ap_en = false;

void run_msg_handler_callback(void* param)
{
    while(1)
    {
        printf("running msg handler thread\n");
        http_web_server();
    }
}

int esp32_reg_wifi_msg_callback(int (*wifi_callback)(char* msg, char* ret_reply))
{
    if(wifi_callback == NULL)
        return -1;

    wifi_msg_handler_callback = wifi_callback;
    return 0;
}

int esp32_reg_wifi_conn_callback(int (*wifi_conn_cb)(int conn_stat)) {
    if (wifi_conn_cb == NULL)
        return -1;

    wifi_conn_stat_callback = wifi_conn_cb;

    return 0;
}

#define P_HTTP_Testing


//TODO: improve label value checking, this may cause hangup if wrong command
void http_web_server()
{
	printf("http web server ...\n ");

    struct netconn *conn, *newconn;
    err_t err, err1;

    struct netbuf* inbuf;
    char* buff;
    char* msg_mark;
    u16_t buflen;

    char* marker;
    char* marker_last = NULL;
    char reply_buff[MAX_REPLY_BUF];

    if(web_server_status == WEB_SVR_STAT_RUNNING)
    {
#ifdef P_HTTP_Testing
    	printf(" web_server_status=  WEB_SVR_STAT_RUNNING.. \n");
#endif
    	return;
    }

    web_server_status = WEB_SVR_STAT_RUNNING;

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, WEB_SERVER_PORT);
    netconn_listen(conn);

    do{
#ifdef P_HTTP_Testing
    	printf(" inside do loop.. \n");
#endif

    	printf(" after inside do loop.. \n");
        printf("%s %d\n", __func__, __LINE__);

        printf(" after function line.. \n");
        err = netconn_accept(conn, &newconn);

        if(err == ERR_OK)
        {
#ifdef P_HTTP_Testing
    	printf(" err = ERR_OK. \n");
#endif
            printf("%s %d\n", __func__, __LINE__);
            err1 = netconn_recv(newconn, &inbuf);
            if(err1 == ERR_OK)
            {
#ifdef P_HTTP_Testing
    	printf(" err1 = ERR_OK. \n");
#endif
                printf("%s %d\n", __func__, __LINE__);
                //accept msg and apply 0 end to string
                netbuf_data(inbuf, (void**)&buff, &buflen);
                *(buff+buflen) = 0;
                printf("%s %d buff -->%s<-- %d \n", __func__, __LINE__, buff, buflen);

                if(strstr(buff, "POST") != NULL)
                {
                    msg_mark = strstr(buff, "\r\n\r\n");
                    if (msg_mark)
                        msg_mark += strlen("\r\n\r\n");
                }
                else //if(strstr(buff, "GET") != NULL)
                {
                    //orig
                    //msg_mark = strstr(buff, "\r\n\r\n");
                    //msg_mark += strlen("\r\n\r\n");

                    msg_mark = strstr(buff, "?");
                    if (msg_mark) {
                        msg_mark++;
                        printf("get msg [%s]\n", msg_mark);
                        marker = strstr(msg_mark, " ");
                        if(marker == NULL)
                            printf("marker null\n");
                        else
                            *marker = 0;
                    }
                }

                if(msg_mark != NULL)
                {
                    printf("%s %d\n", __func__, __LINE__);
                    printf("data received\n-->%s<--\n", msg_mark);
                    //msg parser
                    do
                    {
                        marker = msg_mark;
                        while(marker != NULL)
                        {
                            printf("%s %d\n", __func__, __LINE__);
                            printf("marker [%s]\n", marker);
                            marker_last = marker;
                            marker = strstr(marker, PARSER_KEY);
                            if(marker != NULL) marker++;
                        }
                        printf("%s %d\n", __func__, __LINE__);
                        printf("last marker [%s]\n", marker_last);
                        if(strlen(marker_last) > 0)
                        {
                            //this should be the message handler
                            //TODO: please check the lenght of reply
                            if(wifi_msg_handler_callback != NULL)
                            {
                                printf("%s %d\n", __func__, __LINE__);
                                memset(reply_buff, 0, MAX_REPLY_BUF);
                                wifi_msg_handler_callback(marker_last, reply_buff);
                            }

                            printf("%s %d\n", __func__, __LINE__);
                            if(msg_mark == marker_last) break;
                            marker_last--;
                            *marker_last = 0;
                        }
                    }while(1);
                    //msg parser
                }
                //reply to msg
                printf("%s %d\n", __func__, __LINE__);
                netconn_write(newconn, http_html_header,
                        sizeof(http_html_header)-1, NETCONN_NOCOPY);
                if(strlen(reply_buff) > 0)
                {
                    printf("%s %d\n", __func__, __LINE__);
                    netconn_write(newconn, reply_buff,
                            strlen(reply_buff), NETCONN_NOCOPY);
                }
                else
                {
                    printf("%s %d\n", __func__, __LINE__);
                    netconn_write(newconn, http_html_reply,
                            sizeof(http_html_reply)-1, NETCONN_NOCOPY);
                }
                printf("%s %d\n", __func__, __LINE__);
                netconn_close(newconn);
                netbuf_delete(inbuf);
            }
            printf("%s %d\n", __func__, __LINE__);
            netconn_delete(newconn);
        }
    }while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

int event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    switch(event_id) {
        //deprecated
        case SYSTEM_EVENT_WIFI_READY:
        case SYSTEM_EVENT_SCAN_DONE:
            break;
            //deprecated

            //WIFI_MODE_STA
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_STOP:
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            if (wifi_conn_stat_callback)
                wifi_conn_stat_callback(1);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // This is a workaround as ESP32 WiFi libs don't currently
             //  auto-reassociate.
        	//added by dilpreet for tcp to check wifi is connected or not
            printf("disconnected\n");
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            if (wifi_conn_stat_callback)
                wifi_conn_stat_callback(0);
            if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
				esp_wifi_connect();
				s_retry_num++;
				printf("retry to connect to the AP\n");
			} else {
				xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
			}
            printf("connect to the AP fail");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
        	//added by dilpreet for tcp
        	//event = (ip_event_got_ip_t*) event_data;
			//ESP_LOGI(TAG, "got ip:%s",ip4addr_ntoa(&event->ip_info.ip));
			s_retry_num = 0;
			xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
			printf("connected\n");
            break;
            //case SYSTEM_EVENT_AP_STA_GOT_IP:  //no macro like this TODO
            //break;

            //WIFI_MODE_AP
        case SYSTEM_EVENT_AP_START:
            wifi_ap_en = true;
            break;
        case SYSTEM_EVENT_AP_STOP:
            wifi_ap_en = false;
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            if (wifi_conn_stat_callback)
                wifi_conn_stat_callback(1);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            if (wifi_conn_stat_callback)
                wifi_conn_stat_callback(0);
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            break;

            //WIFI WPS
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            esp_wifi_wps_disable();
            // save SSID and password
            memset(&global_wifi_config, 0, sizeof(global_wifi_config));
            esp_wifi_get_config(WIFI_IF_STA, &global_wifi_config);
            set_string_to_storage(NVS_LUCIDTRON_SSID_KEY, (char *)(global_wifi_config.sta.ssid));
            set_string_to_storage(NVS_LUCIDTRON_PW_KEY, (char *)(global_wifi_config.sta.password));

            //changed location of eeprom and used location of flash which is used by tcp
            // strcpy(username,(char *)(global_wifi_config.sta.ssid));
            // strcpy(password,(char *)(global_wifi_config.sta.password));

            printf("I am in SYSTEM_EVENT_STA_WPS_ER_SUCCESS \n ");
           // writeEEPROM();
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            break;

        default:
            break;
    }
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		s_retry_num = 0;
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
	}
    return ESP_OK;
}
//added this event_handler and commented previous
//this event_handler checks if wifi is connected or not EXAMPLE_ESP_MAXIMUM_RETRY number of times
// if it founds wifi then WIFI_CONNECTED_BIT is set and if wifi fails to connect then WIFI_FAIL_BIT is set
/*
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            printf("retry to connect to the AP\n");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        printf("connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}*/
int esp32_initialise_wifi(void)
{
    //this init the high level protocol handler for the driver
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();

    //prepare the event callback
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    //this initialize the wifi driver 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    esp32_wifi_status = ESP32_WIFI_SLEEP;

    if(web_server_status == WEB_SVR_STAT_UNKNOWN)
    {
    	printf("in side xTaskCreate run_msg_handler_callback \n ");
        xTaskCreate(run_msg_handler_callback,
                "run_msg_handler_callback", 8192, NULL, 5, NULL);
    }

    return 0;
}

int esp32_wifi_config(int mode, char* ssid, char* password)
{
    //this config will save the config in ram, theres also an option to
    //save in nvs flash, that means even when restart, config are stored
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    //WIFI_MODE_NULL
    //WIFI_MODE_STA
    //WIFI_MODE_AP
    //WIFI_MODE_APSTA
    ESP_ERROR_CHECK( esp_wifi_set_mode(mode) );
    if(mode == WIFI_MODE_STA)
    {
        memset(&global_wifi_config, 0, sizeof(global_wifi_config));

        strncpy((char*)global_wifi_config.sta.ssid, ssid, 32);
        strncpy((char*)global_wifi_config.sta.password, password, 64);

        ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &global_wifi_config) );
    }
    else if(mode == WIFI_MODE_AP)
    {
        /*
        tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_AP);
        tcpip_adapter_ip_info_t ipInfo;
        IP4_ADDR(&ipInfo.ip, 192,168,100,99);
        IP4_ADDR(&ipInfo.gw, 192,168,100,1);
        IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
        tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo);
        */

        memset(&global_wifi_config, 0, sizeof(global_wifi_config));

        strncpy((char*)global_wifi_config.ap.ssid, ssid, 32);
        strncpy((char*)global_wifi_config.ap.password, password, 64);
        global_wifi_config.ap.channel        = 0;
        if (strlen(password) == 0) {
            global_wifi_config.ap.authmode   = WIFI_AUTH_OPEN;
        } else {
            global_wifi_config.ap.authmode   = WIFI_AUTH_WPA_WPA2_PSK;
        }
        global_wifi_config.ap.ssid_hidden    = 0;
        global_wifi_config.ap.max_connection = 1;
        global_wifi_config.ap.beacon_interval= 100;

        esp_wifi_set_mode(WIFI_MODE_AP);
            
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &global_wifi_config) );
    }

    //ESP_ERROR_CHECK( esp_wifi_start() );
    return 0;
}

int esp32_wake_up(void* param)
{

    return -1;
}
int esp32_sleep(void)
{

    return -1; 
}

int esp32_send_msg(char* destination, char* msgtosend, int lenght)
{

    return -1;
}
int esp32_receive_msg(char* toreceive, int lenght)
{

    return -1;
}
int esp32_wifi_scan(void* variable_args)
{
    return -1;
}
//wifi mode config
int esp32_wps_enable(void)
{
    esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);

    //if(esp32_wifi_status == ESP_IF_WIFI_AP) return -1;
    if(esp32_wifi_status != ESP32_WIFI_WPS)
    {
        esp32_wifi_status = ESP32_WIFI_WPS;
        esp_wifi_stop();
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();

        esp_wifi_wps_enable(&config);
        esp_wifi_wps_start(0);
    }

    return 0;
}
int esp32_wps_disable(void)
{
    esp_wifi_wps_disable();
    return 0;
}
int esp32_wifi_client_enable(char* ssid, char* pw)
{
    esp_wifi_stop();
    esp32_wifi_status = ESP32_WIFI_CLIENT;
    esp32_wifi_config(WIFI_MODE_STA, ssid, pw);
    esp_wifi_start();

    return 0;
}
int esp32_wifi_ap_enable(char* ssid_ap, char *pw)
{
    //do not call this, this will erase existing config
    //esp_wifi_stop();
    esp32_wifi_status = ESP32_WIFI_AP;
    esp32_wifi_config(WIFI_MODE_AP, ssid_ap, pw);
    esp_wifi_start();

    return 0;
}

int esp32_wifi_ap_disable(void) {
    if(esp32_wifi_status == ESP32_WIFI_AP) {
        //esp32_wifi_config(WIFI_MODE_NULL, NULL, NULL); 
        esp_wifi_stop();
        esp32_wifi_status = ESP32_WIFI_UNKNOWN;
    }
    return 0;
}

bool esp32_wifi_is_ap_enabled(void) {
    return wifi_ap_en;
}

uint8_t *esp32_wifi_ap_get_mac(void)
{
    esp_wifi_get_mac(ESP_IF_WIFI_AP, ap_mac);
    return ap_mac;
}

uint8_t *esp32_wifi_client_get_mac(void)
{
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    return sta_mac;
}

#if 0 //this is for REST API
static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];
    int data_received = 0;

    while(1) {
        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.
         * Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code 
         */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket\r\n");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}
#endif




// After this added added code for Testing

#ifdef subscribePublishTopic


 /* The examples use simple WiFi configuration that you can set via
    'make menuconfig'.
    If you'd rather not, just change the below entries to strings with
    the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
 */
 #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
 #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD


 /* FreeRTOS event group to signal when we are connected & ready to make a request */
//  static EventGroupHandle_t wifi_event_group;  // commented here for testing

 /* The event group allows multiple bits for each event,
    but we only care about one event - are we connected
    to the AP with an IP? */
// const int CONNECTED_BIT = BIT0;   // commented for merge code..

 /* CA Root certificate, device ("Thing") certificate and device
  * ("Thing") key.
    Example can be configured one of two ways:
    "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.
    "Filesystem Certs" are loaded from the filesystem (SD card, etc.)
    See example README for more details.
 */
 #if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)

 extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
 extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
 extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
 extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
 extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
 extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

 #elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)

 static const char * DEVICE_CERTIFICATE_PATH = CONFIG_EXAMPLE_CERTIFICATE_PATH;
 static const char * DEVICE_PRIVATE_KEY_PATH = CONFIG_EXAMPLE_PRIVATE_KEY_PATH;
 static const char * ROOT_CA_PATH = CONFIG_EXAMPLE_ROOT_CA_PATH;

 #else
 #error "Invalid method for loading certs"
 #endif

 /**
  * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
  */
 char HostAddress[255] = AWS_IOT_MQTT_HOST;   // Original
// char HostAddress[255] = "a2r3ux20tob9op-ats.iot.us-west-1.amazonaws.com";


 /**
  * @brief Default MQTT port is pulled from the aws_iot_config.h
  */
 // uint32_t port = AWS_IOT_MQTT_PORT;  // Testing
// uint32_t port = 8883;  // not working
// unsigned int  port = AWS_IOT_MQTT_PORT;   // not working
// #define port AWS_IOT_MQTT_PORT


 /*Function definition*/
 int  getSubString(char *source, char *target,int from, int to)
 {
 	int length=0;
 	int i=0,j=0;

 	//get length
 	while(source[i++]!='\0')
 		length++;

 	if(from<0 || from>length){
 		printf("Invalid \'from\' index\n");
 		return 1;
 	}
 	if(to>length){
 		printf("Invalid \'to\' index\n");
 		return 1;
 	}

 	for(i=from,j=0;i<=to;i++,j++){
 		target[j]=source[i];
 	}

 	//assign NULL at the end of string
 	target[j]='\0';

 	return 0;
 }




#ifdef OLD_AWS_IOT_LOGIC_SENDANDRECIEVE
 void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                     IoT_Publish_Message_Params *params, void *pData) {

	 char replybuffer[512];
	 // char replySubBuffer[30];
	 char replySubBuffer[6];  // Buffer for match of keyword eg Temp, HOON, HOFF
	 char replySubBuffer2[30];

	 char payLoadBuufer[55];

	// char label[6];

     ESP_LOGI(TAG, "Subscribe callback");
     ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

    // printf("\n params->payload) %s \n ", (char*) params-> payload);
     // if(){
	 memset(replybuffer,0,sizeof(replybuffer));
     memcpy(replybuffer,(char*) params-> payload,sizeof(replybuffer));

#define Payloading_main_Firmware
#ifdef Payloading_main_Firmware  // "{ \"cmd\": \"set\",\"set_target_temp\": \"30\" }"

     // getSubString(replybuffer,payLoadBuufer,0,49);  // "{ \"cmd\": \"set\",\"set_target_temp\": \"30\" }"  -> Value received in tne buffer
   //  getSubString(replybuffer,payLoadBuufer,0,37);  // "{"cmd":"set","set_target_temp":"30"}"
    // getSubString(replybuffer,payLoadBuufer,0,35);  //{"cmd":"set","set_target_temp":"30"}
     getSubString(replybuffer,payLoadBuufer,0,54);    //{"cmd":"set","set_target_temp":"30","target":"Heater1"}  // Working OK

     printf("\n payLoadBuufer: %s \n", payLoadBuufer);
     mainflux_msg_handler(payLoadBuufer, 0);
#endif
     // printf("\n replybuffer %s \n ", replybuffer);

     // getSubString(replybuffer,replySubBuffer,2, 5);  // eg {"Temp:30"};  // OK
    // getSubString(replybuffer,replySubBuffer,1, 4);  // eg  {Temp:30};    //OK // Last Tested AWS Working OK
     // getSubString(replybuffer,replySubBuffer,0, 3);  // eg  Temp:30;  // NotOk

     getSubString(replybuffer,replySubBuffer,13,16);  // eg  {"message": "Temp:30","topic": "console"}

     // printf ("\n\n replySubBuffer is: %s", replySubBuffer);

    if(strcmp(replySubBuffer,"Temp") == 0)
      {
		// getSubString(replybuffer,replySubBuffer2,7,8); // eg {"Temp:30"};  // ok
		// getSubString(replybuffer,replySubBuffer2,6,7); // eg {Temp:30};      // ok  // Last Tested AWS Working OK
		// getSubString(replybuffer,replySubBuffer2,5,6); // eg Temp:30;  // Not Ok

		 getSubString(replybuffer,replySubBuffer2,18,19); // eg  {"message": "Temp:30","topic": "console"}

		 //printf("String matched\n");
		 printf("Value of temp is: %s\n",replySubBuffer2);
		 temperatureSetByCMD = (atoi)(replySubBuffer2);
	     printf("Value of temp is: %d\n",temperatureSetByCMD);
		 memset(replySubBuffer2,0,sizeof(replySubBuffer2));
	     memset(replySubBuffer,0,sizeof(replySubBuffer));
	     uchTopic_Set_temp_subscribe_status = 1;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 0;

		  uchTopic_HeaterON_Publish_status = 0;
		  uchTopic_HeaterOFF_Publish_status = 0;
      }
    else if(strcmp(replySubBuffer,"HEON") == 0)
    {
    	  heater_on();  // Heater ON by command..
	     memset(replySubBuffer,0,sizeof(replySubBuffer));
	     memset(replybuffer,0,sizeof(replybuffer));

	     HeaterOnOffStatus = 1;

	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 0;
		  uchTopic_HeaterON_Publish_status = 1;
		  uchTopic_HeaterOFF_Publish_status = 0;

    }
    else if(strcmp(replySubBuffer,"HEOF") == 0)
     {

    	  heater_off();  // Heater OFF by command..
	       memset(replySubBuffer,0,sizeof(replySubBuffer));
	       memset(replybuffer,0,sizeof(replybuffer));

	       HeaterOnOffStatus = 0;

    	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
    	     uchTopic_HeaterParameter_Publish_status  = 0;
    		 uchTopic_HeaterON_Publish_status = 0;
    		 uchTopic_HeaterOFF_Publish_status = 1;
      }
    else
      {
	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 1;
		  uchTopic_HeaterON_Publish_status = 0;
		  uchTopic_HeaterOFF_Publish_status = 0;
      }
    // }// end of if
 }

#endif



#define Wifi_sub_pub

#ifdef Wifi_sub_pub
   void initialise_wifi(void);
  void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data);
#endif


 void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
     ESP_LOGW(TAG, "MQTT Disconnect");
     IoT_Error_t rc = FAILURE;

     if(NULL == pClient) {
         return;
     }

     if(aws_iot_is_autoreconnect_enabled(pClient)) {
         ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
     } else {
         ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
         rc = aws_iot_mqtt_attempt_reconnect(pClient);
         if(NETWORK_RECONNECTED == rc) {
             ESP_LOGW(TAG, "Manual Reconnect Successful");
         } else {
             ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
         }
     }
 }


// #define OLD_AWS_IOT_LOGIC_SENDANDRECIEVE
#ifdef OLD_AWS_IOT_LOGIC_SENDANDRECIEVE
#define HeaterParameterSendingToAWS

 void aws_iot_task(void *param) {

	 char cPayload[100];
     int32_t i = 0;

     IoT_Error_t rc = FAILURE;

     AWS_IoT_Client client;
     IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
     IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

//     IoT_Publish_Message_Params paramsQOS0;
//     IoT_Publish_Message_Params paramsQOS1;

#ifdef HeaterParameterSendingToAWS
     IoT_Publish_Message_Params HeaterParameter;
     IoT_Publish_Message_Params Set_Temp_Parameter;
     IoT_Publish_Message_Params HeaterOnOff;
#endif

     ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

     mqttInitParams.enableAutoReconnect = false; // We enable this later below
     mqttInitParams.pHostURL = HostAddress;
    // mqttInitParams.port = port;
      mqttInitParams.port = 8883;  // testing only

 #if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
     mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
     mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
     mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

 #elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
     mqttInitParams.pRootCALocation = ROOT_CA_PATH;
     mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
     mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
 #endif

     mqttInitParams.mqttCommandTimeout_ms = 20000;
     mqttInitParams.tlsHandshakeTimeout_ms = 5000;
     mqttInitParams.isSSLHostnameVerify = true;
     mqttInitParams.disconnectHandler = disconnectCallbackHandler;
     mqttInitParams.disconnectHandlerData = NULL;

 #ifdef CONFIG_EXAMPLE_SDCARD_CERTS
     ESP_LOGI(TAG, "Mounting SD card...");
     sdmmc_host_t host = SDMMC_HOST_DEFAULT();
     sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
         .format_if_mount_failed = false,
         .max_files = 3,
     };
     sdmmc_card_t* card;
     esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
         abort();
     }
 #endif

     rc = aws_iot_mqtt_init(&client, &mqttInitParams);
     if(SUCCESS != rc) {
         ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
         abort();
     }


     /* Wait for WiFI to show as connected */   // Commented fro testing only
//     xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                         false, true, portMAX_DELAY);


     connectParams.keepAliveIntervalInSec = 10;
     connectParams.isCleanSession = true;
     connectParams.MQTTVersion = MQTT_3_1_1;

     /* Client ID is set in the menuconfig of the example */
     connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
     connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
     connectParams.isWillMsgPresent = false;

     ESP_LOGI(TAG, "Connecting to AWS...");
     do {
         rc = aws_iot_mqtt_connect(&client, &connectParams);
         if(SUCCESS != rc) {
             ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
             vTaskDelay(1000 / portTICK_RATE_MS);
         }
     } while(SUCCESS != rc);

     /*
      * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
      *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
      *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
      */
     rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
     if(SUCCESS != rc) {
         ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
         abort();
     }

#ifdef HeaterParameterSendingToAWS

     //const char *TOPIC1 = "HeaterParameter";  // original
     const char *TOPIC1 = "topic1";  // testing for param key..
     const int TOPIC_LEN1 = strlen(TOPIC1);

         ESP_LOGI(TAG, "Subscribing...");
         rc = aws_iot_mqtt_subscribe(&client, TOPIC1, TOPIC_LEN1, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
         if(SUCCESS != rc) {
             ESP_LOGE(TAG, "Error subscribing : %d ", rc);
             abort();
         }

    	const char *TOPIC2 = "set_Temp";
    	const int TOPIC_LEN2 = strlen(TOPIC2);

		ESP_LOGI(TAG, "Subscribing...");
		rc = aws_iot_mqtt_subscribe(&client, TOPIC2, TOPIC_LEN2, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC2 = "set_Temp";
		if(SUCCESS != rc) {
		  ESP_LOGE(TAG, "Error subscribing : %d ", rc);
		  abort();
		}

		const char *TOPIC3_HeaterOnOff= "Heater_ON_OFF";
		const int TOPIC_LEN3_HeaterOnOff = strlen(TOPIC3_HeaterOnOff);

		ESP_LOGI(TAG, "Subscribing...");
		rc = aws_iot_mqtt_subscribe(&client, TOPIC3_HeaterOnOff, TOPIC_LEN3_HeaterOnOff, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC3 = "Heater_ON";
		if(SUCCESS != rc) {
		  ESP_LOGE(TAG, "Error subscribing : %d ", rc);
		  abort();
		}

		//TOPIC for sending wifi details and heater details, location and heater ID
		const char *TOPIC4_HeaterDetails= "HeaterDetails";
		const int TOPIC_LEN4_HeaterDetails = strlen(TOPIC4_HeaterDetails);

		ESP_LOGI(TAG, "Subscribing...");
		rc = aws_iot_mqtt_subscribe(&client, TOPIC4_HeaterDetails, TOPIC_LEN4_HeaterDetails, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC4 = "HeaterDetails";
		if(SUCCESS != rc) {
		  ESP_LOGE(TAG, "Error subscribing : %d ", rc);
		  abort();
		}

#endif  // end of #ifdef HeaterParameterSendingToAWS


#ifdef HeaterParameterSendingToAWS

     char cPayload1[300];
     HeaterParameter.qos = QOS1;
     HeaterParameter.payload = (void *) cPayload1;
     HeaterParameter.isRetained = 0;

	 char cPayload2[30];
	 Set_Temp_Parameter.qos = QOS1;
	 Set_Temp_Parameter.payload = (void *) cPayload2;
	 Set_Temp_Parameter.isRetained = 0;

	 char cPayload3_HeaterOnOff[20];
	 HeaterOnOff.qos = QOS1;
	 HeaterOnOff.payload = (void *) cPayload3_HeaterOnOff;
	 HeaterOnOff.isRetained = 0;

#endif

     while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

         //Max time the yield function will wait for read messages
         rc = aws_iot_mqtt_yield(&client, 100);
         if(NETWORK_ATTEMPTING_RECONNECT == rc) {
             // If the client is attempting to reconnect we will skip the rest of the loop.
             continue;
         }

         ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

          vTaskDelay(1000 / portTICK_RATE_MS);  // Original Testing
         // vTaskDelay(3000 / portTICK_RATE_MS);
#ifdef HeaterParameterSendingToAWS

         if(uchTopic_HeaterParameter_Publish_status == 1)
		  {
			  // printf("\n I am in Topic_HeaterParameter_Publish\n ");
				// uchTopic_Set_temp_subscribe_status = TRUE;
				memset(cPayload1,0,sizeof(cPayload1));
				// sprintf(cPayload1, "%s : %d  %s : %d %s : %d %s : %d %s : %d", "Ambient Temp", 40,"Set Temp", temperatureSetByCMD, "Heater Status", HeaterOnOffStatus, "Timer",1, "Schedule", 0);  // working one
				sprintf(cPayload1, "{\"%s\": \"%s\", \"%s\" : \"%d\",  \"%s\" : \"%d\", \"%s\" : \"%d\", \"%s\" : \"%d\", \"%s\" : \"%d\"}", "Device ID", "Heater1","Ambient Temp", 40,"Set Temp", temperatureSetByCMD, "Heater Status", HeaterOnOffStatus, "Timer",1, "Schedule", 0); // in JSON
				HeaterParameter.payloadLen = strlen(cPayload1);
				rc = aws_iot_mqtt_publish(&client, TOPIC1, TOPIC_LEN1, &HeaterParameter);
				// ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->cPayload1);
				// uchTopic_Set_temp_subscribe_status = 0;
		  }
         if(uchTopic_HeaterDetails_Publish_status == 1)
		  {
			  // printf("\n I am in Topic_HeaterParameter_Publish\n ");
				// uchTopic_Set_temp_subscribe_status = TRUE;
				memset(cPayload1,0,sizeof(cPayload1));
				// sprintf(cPayload1, "%s : %d  %s : %d %s : %d %s : %d %s : %d", "Ambient Temp", 40,"Set Temp", temperatureSetByCMD, "Heater Status", HeaterOnOffStatus, "Timer",1, "Schedule", 0);  // working one
				sprintf(cPayload1, "{\"%s\" : \"%s\", \"%s\" : \"%s\", \"%s\" : \"%s\", \"%s\" : \"%s\", \"%s\" : \"%s\", \"%s\" : \"%d\"}", "Device ID", "Heater1","SSID", username,"Password", password, "Heater ID", name, "Location ",locID, "Ambient Temp",45); // ONly for Testing
				HeaterParameter.payloadLen = strlen(cPayload1);
				rc = aws_iot_mqtt_publish(&client, TOPIC4_HeaterDetails, TOPIC_LEN4_HeaterDetails, &HeaterParameter);
				// ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->cPayload1);
				// uchTopic_Set_temp_subscribe_status = 0;
		  }
		 if(uchTopic_Set_temp_subscribe_status == 1)
			{
				 // printf("\n I am in Set Temp paramter publish \n\n ");
			     memset(cPayload2,0,sizeof(cPayload2));
				sprintf(cPayload2, "%s : %d", "Temperature is set to ",temperatureSetByCMD);
				Set_Temp_Parameter.payloadLen = strlen(cPayload2);
				  rc = aws_iot_mqtt_publish(&client, TOPIC2, TOPIC_LEN2, &Set_Temp_Parameter);
				  uchTopic_Set_temp_subscribe_status = 0;
				  uchTopic_HeaterParameter_Publish_status  = 1;
				  uchTopic_HeaterON_Publish_status = 0;
				  uchTopic_HeaterOFF_Publish_status = 0;
			}
		 if(uchTopic_HeaterON_Publish_status == 1)
		 {
		   printf("\n I am in uchTopic_HeaterOn_Publish_status \n\n ");
			memset(cPayload3_HeaterOnOff,0,sizeof(cPayload3_HeaterOnOff));

			//sprintf(cPayload3_HeaterOnOff, "%s  : %d", "Heater ON",1);
			 sprintf(cPayload3_HeaterOnOff, "%s : %d", "Heater ON",HeaterOnOffStatus);

			HeaterOnOff.payloadLen = strlen(cPayload3_HeaterOnOff);
			 rc = aws_iot_mqtt_publish(&client, TOPIC3_HeaterOnOff, TOPIC_LEN3_HeaterOnOff, &HeaterOnOff);
			  uchTopic_Set_temp_subscribe_status = 0;
			  uchTopic_HeaterParameter_Publish_status  = 1;
			  uchTopic_HeaterON_Publish_status = 0;
			  uchTopic_HeaterOFF_Publish_status = 0;
		  }
		 if(uchTopic_HeaterOFF_Publish_status == 1)
		 {
			 printf("\n I am in uchTopic_HeaterOFF_Publish_status \n\n ");
			 memset(cPayload3_HeaterOnOff,0,sizeof(cPayload3_HeaterOnOff));

			// sprintf(cPayload3_HeaterOnOff, "%s  : %d", "Heater OFF", 2);
			 sprintf(cPayload3_HeaterOnOff, "%s : %d", "Heater OFF", HeaterOnOffStatus);

			 HeaterOnOff.payloadLen = strlen(cPayload3_HeaterOnOff);
			 rc = aws_iot_mqtt_publish(&client, TOPIC3_HeaterOnOff, TOPIC_LEN3_HeaterOnOff, &HeaterOnOff);
			 uchTopic_Set_temp_subscribe_status = 0;
			 uchTopic_HeaterParameter_Publish_status  = 1;
			  uchTopic_HeaterON_Publish_status = 0;
			  uchTopic_HeaterOFF_Publish_status = 0;
		  }
#endif
         //  printf("After publish HeaterParameterSendingToAWS\n ");
         if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
             ESP_LOGW(TAG, "QOS1 publish ack not received.");
             rc = SUCCESS;
         }
     }
     ESP_LOGE(TAG, "An error occurred in the main loop.");
    // abort();  // Commented Abort for Tesing
 }
#endif // AWS_IOT_TASK


#define NEW_AWS_IOT_LOGIC_SENDANDRECIEVE
#ifdef NEW_AWS_IOT_LOGIC_SENDANDRECIEVE


 void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                     IoT_Publish_Message_Params *params, void *pData) {

	 // char replybuffer[512];  // W

	 char replybuffer[300];  // Json

	 // char replybuffer[900];   // Reply buffer size changed 900 as of payload size of that topic  // Stack OverFlow
	 // char replySubBuffer[30];
	 char replySubBuffer[6];  // Buffer for match of keyword eg Temp, HOON, HOFF
	 char replySubBuffer2[30];

     // char payLoadBuufer[55]; //w
     char payLoadBuufer[150];

	// char payLoadBuufer[500];  // Changed after resettig problem..
	// char label[6];

     ESP_LOGI(TAG, "Subscribe callback");
     ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

	 memset(replybuffer,0,sizeof(replybuffer));
     memcpy(replybuffer,(char*) params-> payload,sizeof(replybuffer));

#define PAYLOAD_INDEX_RECIEVE_BUFFER
#ifdef PAYLOAD_INDEX_RECIEVE_BUFFER
     const char Delimiter = '}';  // ch - Delimiter
     char *ret;
     int endIndex = 0;

     ret = strchr(replybuffer, Delimiter);

    // printf("String after |%c| is - |%s|\n", Delimiter, ret);

     endIndex = (ret - replybuffer);
     //printf("%d \n ", (ret -replybuffer));
     // printf("%d \n ", endIndex);

     getSubString(replybuffer,payLoadBuufer,0,endIndex);
     // printf("\n payLoadBuufer: %s \n", payLoadBuufer);
     mainflux_msg_handler(payLoadBuufer, 0);
#endif


// Tested for  // {"target":"Heater1","cmd":"set","set_target_temp":"30"}
//
// #define COMMENT_PAYLOAD_MAIN_FIRMWARE_DEFINE
#ifdef COMMENT_PAYLOAD_MAIN_FIRMWARE_DEFINE

//#define Payloading_main_Firmware
// #ifdef Payloading_main_Firmware
     // getSubString(replybuffer,payLoadBuufer,0,49);  // "{ \"cmd\": \"set\",\"set_target_temp\": \"30\" }"  -> Value received in tne buffer
     //  getSubString(replybuffer,payLoadBuufer,0,37);  // "{"cmd":"set","set_target_temp":"30"}"
     // getSubString(replybuffer,payLoadBuufer,0,35);  //{"cmd":"set","set_target_temp":"30"}
     getSubString(replybuffer,payLoadBuufer,0,54);    //{"cmd":"set","set_target_temp":"30","target":"Heater1"}  // Working OK

     printf("\n payLoadBuufer: %s \n", payLoadBuufer);
     mainflux_msg_handler(payLoadBuufer, 0);
// #endif
#endif


// #define INITIAL_REPLY_BUFFER_TESTING_IOT_HANDLER
#ifdef  INITIAL_REPLY_BUFFER_TESTING_IOT_HANDLER

     // printf("\n replybuffer %s \n ", replybuffer);
     // getSubString(replybuffer,replySubBuffer,2, 5);  // eg {"Temp:30"};  // OK
    // getSubString(replybuffer,replySubBuffer,1, 4);  // eg  {Temp:30};    //OK // Last Tested AWS Working OK
     // getSubString(replybuffer,replySubBuffer,0, 3);  // eg  Temp:30;  // NotOk

     getSubString(replybuffer,replySubBuffer,13,16);  // eg  {"message": "Temp:30","topic": "console"}

     // printf ("\n\n replySubBuffer is: %s", replySubBuffer);

    if(strcmp(replySubBuffer,"Temp") == 0)
      {
		// getSubString(replybuffer,replySubBuffer2,7,8); // eg {"Temp:30"};  // ok
		// getSubString(replybuffer,replySubBuffer2,6,7); // eg {Temp:30};      // ok  // Last Tested AWS Working OK
		// getSubString(replybuffer,replySubBuffer2,5,6); // eg Temp:30;  // Not Ok

		 getSubString(replybuffer,replySubBuffer2,18,19); // eg  {"message": "Temp:30","topic": "console"}

		 //printf("String matched\n");
		 printf("Value of temp is: %s\n",replySubBuffer2);
		 temperatureSetByCMD = (atoi)(replySubBuffer2);
	     printf("Value of temp is: %d\n",temperatureSetByCMD);
		 memset(replySubBuffer2,0,sizeof(replySubBuffer2));
	     memset(replySubBuffer,0,sizeof(replySubBuffer));
	     uchTopic_Set_temp_subscribe_status = 1;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 0;

		  uchTopic_HeaterON_Publish_status = 0;
		  uchTopic_HeaterOFF_Publish_status = 0;
      }
    else if(strcmp(replySubBuffer,"HEON") == 0)
    {
    	  heater_on();  // Heater ON by command..
	     memset(replySubBuffer,0,sizeof(replySubBuffer));
	     memset(replybuffer,0,sizeof(replybuffer));

	     HeaterOnOffStatus = 1;

	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 0;
		  uchTopic_HeaterON_Publish_status = 1;
		  uchTopic_HeaterOFF_Publish_status = 0;

    }
    else if(strcmp(replySubBuffer,"HEOF") == 0)
     {

    	  heater_off();  // Heater OFF by command..
	       memset(replySubBuffer,0,sizeof(replySubBuffer));
	       memset(replybuffer,0,sizeof(replybuffer));

	       HeaterOnOffStatus = 0;

    	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
    	     uchTopic_HeaterParameter_Publish_status  = 0;
    		 uchTopic_HeaterON_Publish_status = 0;
    		 uchTopic_HeaterOFF_Publish_status = 1;
      }
    else
      {
	     uchTopic_Set_temp_subscribe_status = 0;    // Set Temp is subscribed.
	     uchTopic_HeaterParameter_Publish_status  = 1;
		  uchTopic_HeaterON_Publish_status = 0;
		  uchTopic_HeaterOFF_Publish_status = 0;
      }
#endif // INITIAL_REPLY_BUFFER_TESTING_IOT_HANDLER
}


 // #define HeaterParameterSendingToAWS   // Old one logic for sending data to AWS .
 #define HeaterTopicData  // Latest for Testing Reply buffer

  void aws_iot_task(void *param) {

      IoT_Error_t rc = FAILURE;

      AWS_IoT_Client client;
      IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
      IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

 #ifdef HeaterTopicData
      IoT_Publish_Message_Params HeaterMeassage;
 #endif

      ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

      mqttInitParams.enableAutoReconnect = false; // We enable this later below
      mqttInitParams.pHostURL = HostAddress;
     // mqttInitParams.port = port;
       mqttInitParams.port = 8883;  // testing only

  #if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
      mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
      mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
      mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

  #elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
      mqttInitParams.pRootCALocation = ROOT_CA_PATH;
      mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
      mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
  #endif

      mqttInitParams.mqttCommandTimeout_ms = 20000;
      mqttInitParams.tlsHandshakeTimeout_ms = 5000;
      mqttInitParams.isSSLHostnameVerify = true;
      mqttInitParams.disconnectHandler = disconnectCallbackHandler;
      mqttInitParams.disconnectHandlerData = NULL;

  #ifdef CONFIG_EXAMPLE_SDCARD_CERTS
      ESP_LOGI(TAG, "Mounting SD card...");
      sdmmc_host_t host = SDMMC_HOST_DEFAULT();
      sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
      esp_vfs_fat_sdmmc_mount_config_t mount_config = {
          .format_if_mount_failed = false,
          .max_files = 3,
      };
      sdmmc_card_t* card;
      esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
      if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
          abort();
      }
  #endif

      rc = aws_iot_mqtt_init(&client, &mqttInitParams);
      if(SUCCESS != rc) {
          ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
         // abort(); // Commneted for testing
      }


//      /* Wait for WiFI to show as connected */   // Commented fro testing only
//      xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
//                          false, true, portMAX_DELAY);

      connectParams.keepAliveIntervalInSec = 10;
      connectParams.isCleanSession = true;
      connectParams.MQTTVersion = MQTT_3_1_1;

      /* Client ID is set in the menuconfig of the example */
      connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
      connectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
      connectParams.isWillMsgPresent = false;

      ESP_LOGI(TAG, "Connecting to AWS...");
      do {
          rc = aws_iot_mqtt_connect(&client, &connectParams);
          if(SUCCESS != rc) {
              ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
              printf("waiting for AWS SERVER\n ");
              vTaskDelay(1000 / portTICK_RATE_MS);
          }
      } while(SUCCESS != rc);

      /*
       * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
       *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
       *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
       */
      rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
      if(SUCCESS != rc) {
          ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
         // abort(); // Commneted for testing
      }


#ifdef HeaterTopicData
			const char *topicDevRegis = "dev_regis";  // testing for param key..
			const int topicDevRegis_Len = strlen(topicDevRegis);

			 ESP_LOGI(TAG, "Subscribing...");
			 rc = aws_iot_mqtt_subscribe(&client, topicDevRegis, topicDevRegis_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
			 if(SUCCESS != rc) {
				 ESP_LOGE(TAG, "Error subscribing : %d ", rc);
				// abort();  // Commneted for testing
			 }

			const char *topicSetTemp = "setTemp";  // testing for param key..
			const int topicSetTemp_Len = strlen(topicSetTemp);

			ESP_LOGI(TAG, "Subscribing...");
			rc = aws_iot_mqtt_subscribe(&client, topicSetTemp, topicSetTemp_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
			if(SUCCESS != rc) {
			ESP_LOGE(TAG, "Error subscribing : %d ", rc);
			// abort();  // Commneted for testing
			}
//
			const char *topicGetTemp = "getTemp";  // testing for param key..
			const int topicGetTemp_Len = strlen(topicGetTemp);

			ESP_LOGI(TAG, "Subscribing...");
			rc = aws_iot_mqtt_subscribe(&client, topicGetTemp, topicGetTemp_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
			if(SUCCESS != rc) {
			ESP_LOGE(TAG, "Error subscribing : %d ", rc);
			// abort();  // Commneted for testing
			}

			const char *topicKeepAlive = "keepAlive";  // testing for param key..
			const int topicKeepAlive_Len = strlen(topicKeepAlive);

			ESP_LOGI(TAG, "Subscribing...");
			rc = aws_iot_mqtt_subscribe(&client, topicKeepAlive, topicKeepAlive_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
			if(SUCCESS != rc) {
			 ESP_LOGE(TAG, "Error subscribing : %d ", rc);
			// abort();  // Commneted for testing
			}

//
//			const char *topicMaxThresTempReached = "maxThresTempReach";  //
//			const int topicMaxThresTempReached_Len = strlen(topicMaxThresTempReached);
//
//			ESP_LOGI(TAG, "Subscribing...");
//			rc = aws_iot_mqtt_subscribe(&client, topicMaxThresTempReached, topicMaxThresTempReached_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
//			if(SUCCESS != rc) {
//			 ESP_LOGE(TAG, "Error subscribing : %d ", rc);
//			// abort();  // Commneted for testing
//			}
//
//			 const char *topicMinThresholdReached = "minThresTempReach";  // testing for param key..
//			 const int topicMinThresholdReached_Len = strlen(topicKeepAlive);
//
//			ESP_LOGI(TAG, "Subscribing...");
//			rc = aws_iot_mqtt_subscribe(&client, topicMinThresholdReached, topicMinThresholdReached_Len, QOS0, iot_subscribe_callback_handler, NULL);  // TOPIC1 = "HeaterParameter";
//			if(SUCCESS != rc) {
//			 ESP_LOGE(TAG, "Error subscribing : %d ", rc);
//			// abort();  // Commneted for testing
//			}

 #endif

#ifdef HeaterTopicData
 	   //  char cPayload1[100];
 		 char cPayload1[300];
 		HeaterMeassage.qos = QOS1;
 		HeaterMeassage.payload = (void *) cPayload1;
 		HeaterMeassage.isRetained = 0;
 #endif

      while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

          //Max time the yield function will wait for read messages
          rc = aws_iot_mqtt_yield(&client, 100);
          if(NETWORK_ATTEMPTING_RECONNECT == rc) {
              // If the client is attempting to reconnect we will skip the rest of the loop.
              continue;
          }

          ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

           vTaskDelay(1000 / portTICK_RATE_MS);  // Original Testing
          // vTaskDelay(3000 / portTICK_RATE_MS);  // Testing


#define SEPERATE_TOPIC_LOGIC
#ifdef SEPERATE_TOPIC_LOGIC
			if(oneTimeRegistrationPacketToAWS==1)
			{
				memset(cPayload1,0,sizeof(cPayload1));

				printf("One Time Registration \n");
				//  sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", "Heater2","deviceName", username,"ssid", password, "accounId", name, "locationId ",locID); // ONly for Testing  // Getting Restart on this
				// sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", "Heater2","deviceName", "heater_name","ssid", "wifi_ssidf", "accounId", "user_account_id", "locationId ","location_id"); // working one..
				// sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", uniqueDeviceID,"deviceName",name ,"ssid", username , "accounId", id, "locationId ",locID ); // Testing for unique ID..// Working

				sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", uniqueDeviceID,"deviceName",name ,"ssid", username , "accounId", id, "locationId ",locID, "timeZone",timeZone); // Adding TimeZone..

				HeaterMeassage.payloadLen = strlen(cPayload1);
				rc = aws_iot_mqtt_publish(&client, topicDevRegis, topicDevRegis_Len, &HeaterMeassage);
				memset(replybuff,0,sizeof(replybuff));
				memset(cPayload1,0,sizeof(cPayload1));
			}

			 if(commandReceived_SendAck == 1)
			 {
				memset(cPayload1,0,sizeof(cPayload1));
				printf("commandReceived_SendAck \n");

				// sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",%s \n}", "deviceID", "Heater2",replybuff); //  Working one ..
				sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",%s \n}", "deviceID", uniqueDeviceID, replybuff); //  Testing Unique
				HeaterMeassage.payloadLen = strlen(cPayload1);

				switch(CommandAck)
				{
				    case SET_TEMP_ACK :
					                     rc = aws_iot_mqtt_publish(&client, topicSetTemp, topicSetTemp_Len, &HeaterMeassage);
					                     break;
				    case GET_TEMP_ACK :
				    			     	 rc = aws_iot_mqtt_publish(&client, topicGetTemp, topicGetTemp_Len, &HeaterMeassage);
				    			     	 break;
				    default:   break;
				}

				commandReceived_SendAck = 0;
				memset(replybuff,0,sizeof(replybuff));
				memset(cPayload1,0,sizeof(cPayload1));
		     }

			 if(keepAliveFlag==1)
			 {
				int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
				if ((cur_ms - keepAlive_ms) >= KEEP_ALIVE_DATA__PACKET_DUR_MS) {
					keepAlive_ms = cur_ms;
					keepAliveSendDataToAWS = 1;
				  }
			 }
			 if(keepAliveSendDataToAWS==1)
			 {
				printf("I am sendig Kepp Alive packet \n");
				memset(cPayload1,0,sizeof(cPayload1));
				//  sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", "Heater2","deviceName", username,"ssid", password, "accounId", name, "locationId ",locID); // ONly for Testing  // Getting Restart on this
				// sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", "Heater2","msg","EveryThingIsFine"); // WorkingOne..
				sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", uniqueDeviceID,"type","everythingFineHere"); // WorkingOne..

				HeaterMeassage.payloadLen = strlen(cPayload1);
				rc = aws_iot_mqtt_publish(&client, topicKeepAlive, topicKeepAlive_Len, &HeaterMeassage);
				memset(replybuff,0,sizeof(replybuff));
				memset(cPayload1,0,sizeof(cPayload1));
				keepAliveSendDataToAWS =0;
			  }

//
//			if(maxTemperatureThresholdReachedWarning==1){
//				memset(cPayload1,0,sizeof(cPayload1));
//				sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\"\n}", "deviceID", "Heater2","type", "alert","warning", "Max Temperature Threshold Range Reached - Heater OFF");
//				HeaterMeassage.payloadLen = strlen(cPayload1);
//				rc = aws_iot_mqtt_publish(&client, topicMaxThresTempReached, topicMaxThresTempReached_Len, &HeaterMeassage);
//				memset(replybuff,0,sizeof(replybuff));
//				memset(cPayload1,0,sizeof(cPayload1));
//				maxTemperatureThresholdReachedWarning= 0;
//			 }
//
//			if(minTemperatureThresholdReachedWarning==1){
//				memset(cPayload1,0,sizeof(cPayload1));
//				// sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\"}", "deviceID", "Heater2","deviceName", "heater_name","ssid", "wifi_ssidf", "accounId", "user_account_id", "locationId ","location_id"); // ONly for Testing
//				sprintf(cPayload1, "{\n\t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\", \n\t\"%s\" : \"%s\"\n}", "deviceID", "Heater2","type", "alert","warning", "Anti Freezing ON - Heater ON");
//				HeaterMeassage.payloadLen = strlen(cPayload1);
//				rc = aws_iot_mqtt_publish(&client, topicMinThresholdReached, topicMinThresholdReached_Len, &HeaterMeassage);
//				memset(replybuff,0,sizeof(replybuff));
//				memset(cPayload1,0,sizeof(cPayload1));
//				minTemperatureThresholdReachedWarning= 0;
//			 }

#endif
		//  printf("After publish HeaterParameterSendingToAWS\n ");
          if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
              ESP_LOGW(TAG, "QOS1 publish ack not received.");
              rc = SUCCESS;
          }
      }
      ESP_LOGE(TAG, "An error occurred in the main loop.");
     // abort();  // Commented Abort for Tesing
  }

#endif



//#define Addition_INITIALISE_WIFI_HANDLER
#ifdef Addition_INITIALISE_WIFI_HANDLER
  void initialise_wifi(void)
   {
  	printf("I am in initialise wifi \n ");

       tcpip_adapter_init();
       wifi_event_group = xEventGroupCreate();
       ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
       wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
       ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
       ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

       wifi_config_t wifi_config = {
           .sta = {
               .ssid = EXAMPLE_WIFI_SSID,
               .password = EXAMPLE_WIFI_PASS,
           },
       };

       ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
       ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
       ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
       ESP_ERROR_CHECK( esp_wifi_start() );
   }

#endif//  end of #ifdef Wifi_sub_pub

#endif  // end for subscribe and publish..





//For Adding TCP_Server Code for pairing Mobile to ESP..

#ifdef TCP_Server_Code
nvs_handle_t my_handle;

//#define PORT CONFIG_EXAMPLE_PORT  // original Lines


// Below lins added for testing only Start
#define CONFIG_EXAMPLE_IPV4 1
#define CONFIG_EXAMPLE_PORT 3333

#define PORT CONFIG_EXAMPLE_PORT
// Below lins added for testing only end


#define EXAMPLE_ESP_WIFI_SSID "WESP321"
#define EXAMPLE_ESP_WIFI_PASS "qwerty12345"

// static const char *TAG = "example";  // Original Lines
static const char *TCP_SERVER_TAG = "example";

static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[328];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TCP_SERVER_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_SERVER_TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_SERVER_TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_SERVER_TAG, "Socket bound, port %d", PORT);

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TCP_SERVER_TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_SERVER_TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TCP_SERVER_TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TCP_SERVER_TAG, "Socket accepted");
        vTaskDelay(2000/portTICK_PERIOD_MS);

        err = send(sock, "found\r\n", 7, 0);



        if (err < 0) {
			ESP_LOGE(TCP_SERVER_TAG, "Error occurred during sending: errno %d", errno);
			//break;
		}
        else
        	ESP_LOGI(TCP_SERVER_TAG, "found sent");
        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_SERVER_TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TCP_SERVER_TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TCP_SERVER_TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TCP_SERVER_TAG, "%s", rx_buffer);

                saveDetails(rx_buffer);  // Then networking if net is down.. then we can test the data from app payload for setting the parameter in the esp.  //

                // Test the message buffer for APP data to ESP when internet down.

                int err = send(sock, rx_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGE(TCP_SERVER_TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
        	shutdown(sock, 0);
        	close(sock);
        	abort();
            ESP_LOGE(TCP_SERVER_TAG, "Shutting down socket and restarting...");

        }
    }
    vTaskDelete(NULL);
}

void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, username);
    strcpy((char *)wifi_config.sta.password, password);

    printf( "username %s\n",username );
    printf( "password %s\n",password );
    printf( "(char *)wifi_config.sta.ssid %s\n",(char *)wifi_config.sta.ssid );
    printf( "(char *)wifi_config.sta.password %s\n",(char *)wifi_config.sta.password );

    /*wifi_config_t wifi_config = {
        .sta = {
            .ssid = username,
            .password = password,
        },
    };*/

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );


        /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
         * happened. */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
	 * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",username, password);
		xTaskCreate(&aws_iot_task, "aws_iot_task", 8192, NULL, 5, NULL);   // aws iot task .. initiation..

		oneTimeRegistrationPacketToAWS = 1; // New added to sending First packet to AWS

	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",username, password);
		initSoftAP();
		xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}

	//ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
	//ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
	//vEventGroupDelete(s_wifi_event_group);
}

void tcpServer_main()
{
	esp_err_t err = nvs_flash_init();
	//ESP_ERROR_CHECK(nvs_flash_erase());
	if (err != ESP_OK) printf("Error (%s) INITIALISING NVS!\n", esp_err_to_name(err));
	//ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    err=(esp_event_loop_create_default());
    if (err != ESP_OK) printf("Error (%s) INITIALISING LOOP!\n", esp_err_to_name(err));
    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

    // New added for for wifi - mac address.
    unsigned char ap_mac[6];
    esp_efuse_mac_get_default(ap_mac);
    printf("\nMAC ADRESS = %02x:%02x:%02x:%02x:%02x:%02x \n\n",ap_mac[0],ap_mac[1],ap_mac[2],ap_mac[3],ap_mac[4],ap_mac[5]);

    sprintf(comm_wifi_dev.wifi_ap_ssid, "%s-%02x%02x%02x", WIFI_AP_MODE_SSID_BASE, ap_mac[3], ap_mac[4], ap_mac[5]);

    printf("comm_wifi_dev.wifi_ap_ssid %s \n ",comm_wifi_dev.wifi_ap_ssid);
    strcpy(uniqueDeviceID, comm_wifi_dev.wifi_ap_ssid );  // New Added after mac address receiving //For changing the ssid
    printf("uniqueDeviceID in TCP_server_main   %s\n", uniqueDeviceID);

   // initFlash();  //
	readEEPROM();
	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
	initialise_wifi();
}



void initSoftAP()
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    wifi_config_t wifi_config = {
  		.ap = {
  			.password = EXAMPLE_ESP_WIFI_PASS,
  			.max_connection = 2,
  			.authmode = WIFI_AUTH_WPA_WPA2_PSK
  			},
  		};

    strcpy((char *)wifi_config.ap.ssid, comm_wifi_dev.wifi_ap_ssid );  // New Added after mac address receiving //For changing the ssid
    strcpy(uniqueDeviceID, comm_wifi_dev.wifi_ap_ssid );  // New Added after mac address receiving //For changing the ssid

    printf("(char *)wifi_config.ap.ssid:  %s\n", (char *)wifi_config.ap.ssid);
    printf("uniqueDeviceID in initSoftAP  %s\n", uniqueDeviceID);

    wifi_config.ap.ssid_len = strlen(comm_wifi_dev.wifi_ap_ssid);
    printf("(char *)wifi_config.ap.ssid_len:  %d\n",   wifi_config.ap.ssid_len);

    // Last working SSID
//    wifi_config_t wifi_config = {
//		.ap = {
//			.ssid = EXAMPLE_ESP_WIFI_SSID,
//			.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
//			.password = EXAMPLE_ESP_WIFI_PASS,
//			.max_connection = 2,
//			.authmode = WIFI_AUTH_WPA_WPA2_PSK
//			},
//		};

	if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}


void initFlash()
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );

	printf("Opening Non-Volatile Storage (NVS) handle... ");
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else
		printf("Done\n");
}



void saveDetails(char *eusart1RxBuffer)
{
    int startPos=0,endPos=0,j=0;
    char flag=0,saveTimezoneFlag = 0;

//    startPos=cmpString(eusart1RxBuffer, "U:<");
//    endPos=cmpString(eusart1RxBuffer, ">U");
    startPos=cmpString(eusart1RxBuffer, "S:<");
    endPos=cmpString(eusart1RxBuffer, ">S");
    for(int i=0;i<50;i++)
    {
        username[i]=0;
        password[i]=0;
    }
    for(int i=startPos;i<endPos-2;i++)
    {
    	username[j]=eusart1RxBuffer[i];
        j++;
        flag=1;
    }
    j=0;
    startPos=cmpString(eusart1RxBuffer, "P:<");
    endPos=cmpString(eusart1RxBuffer, ">P");
    for(int i=startPos;i<endPos-2;i++)
    {
        password[j]=eusart1RxBuffer[i];
        j++;
        flag=1;
    }
    j=0;
	startPos=cmpString(eusart1RxBuffer, "D:<");
	endPos=cmpString(eusart1RxBuffer, ">D");
	for(int i=startPos;i<endPos-2;i++)
	{
		id[j]=eusart1RxBuffer[i];
		j++;
		flag=1;
	}
	j=0;
	startPos=cmpString(eusart1RxBuffer, "L:<");
	endPos=cmpString(eusart1RxBuffer, ">L");
	for(int i=startPos;i<endPos-2;i++)
	{
		locID[j]=eusart1RxBuffer[i];
		j++;
		flag=1;
	}
	j=0;
	startPos=cmpString(eusart1RxBuffer, "H:<");
	endPos=cmpString(eusart1RxBuffer, ">H");
	for(int i=startPos;i<endPos-2;i++)
	{
		name[j]=eusart1RxBuffer[i];
		j++;
		flag=1;
	}

#ifdef TimeZoneAdded
	j=0;
		startPos=cmpString(eusart1RxBuffer, "T:<");
		endPos=cmpString(eusart1RxBuffer, ">T");
		for(int i=startPos;i<endPos-2;i++)
		{
			timeZone[j]=eusart1RxBuffer[i];
			j++;
			saveTimezoneFlag=1;
		}
		if(saveTimezoneFlag==1 && strlen(timeZone)>0)
			saveTimeZone();
#endif
    if(flag==1)
    	writeEEPROM();
}



#ifdef TimeZoneAdded
void saveTimeZone(void)
{
	int timezoneOffsetMinutes = atoi(timeZone);
	int timezone_offset_idx,i,len;
	//len = sizeof(timezone_offset_list_min)/sizeof(timezone_offset_list_min[0]);
	len=38;													//length of array timezone_offset_list_min
	for(i=0;i<len;i++)
	{
		if(timezone_offset_list_min[i]==timezoneOffsetMinutes)
			break;
	}
	if(i<len)
	{
		printf("Received valid timezone value %d from mobile app\n",timezoneOffsetMinutes);
		timezone_offset_idx=i;
		clock_set_timezone_offset(timezoneOffsetMinutes);
		tzset();
		set_integer_to_storage(STORAGE_KEY_TIMEZONE_OFFSET_INDEX, timezone_offset_idx);
		printf("Time Zone updated in memory\n");
	}

	set_string_to_storage(NVS_TIMEZONE, timeZone); // New added on 28Oct2020

	/*esp_err_t err;
	size_t size = strlen(timeZone);
	err = nvs_set_blob(my_handle, "timeZone", timeZone,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "timeZoneLength", strlen(timeZone));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	printf("Committing updates in NVS ... ");
	err = nvs_commit(my_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");*/
}

#endif


#define modifiedCode
#ifdef modifiedCode

void writeEEPROM()
{
	set_string_to_storage(NVS_LUCIDTRON_SSID_KEY, username);
	set_string_to_storage(NVS_LUCIDTRON_PW_KEY, password);
	set_string_to_storage(NVS_DEVICE_ID, id);
	set_string_to_storage(NVS_LOC_ID, locID);
	set_string_to_storage(NVS_DEVICE_NAME, name);
	esp_restart();
}

void readEEPROM()
{
	// username[0]='a';
	get_string_from_storage(NVS_LUCIDTRON_SSID_KEY, username); printf("Username 1st time  = %s",username);
	if(strlen(username)<=0)
	    strcpy(username,"asdf");

	get_string_from_storage(NVS_LUCIDTRON_PW_KEY, password); printf("Password = %s",password);
	get_string_from_storage(NVS_DEVICE_ID, id); printf("DeviceID = %s",id);
	get_string_from_storage(NVS_LOC_ID, locID); printf("LocID = %s",locID);
	get_string_from_storage(NVS_DEVICE_NAME, name); printf("DeviceName = %s",name);

	get_string_from_storage(NVS_TIMEZONE, timeZone); printf("TimeZone = %s",timeZone);  // New added for Time Zone
}
#else

void writeEEPROM()
{
	esp_err_t err;
	size_t size = strlen(username);
	err = nvs_set_blob(my_handle, "username", username,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "usernameLength", strlen(username));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");

	size = strlen(password);
	err = nvs_set_blob(my_handle, "password", password,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "passwordLength", strlen(password));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");

	size = strlen(id);
	err = nvs_set_blob(my_handle, "id", id,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "idLength", strlen(id));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");

	size = strlen(locID);
	err = nvs_set_blob(my_handle, "locID", locID,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "locIDLength", strlen(locID));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");

	size = strlen(name);
	err = nvs_set_blob(my_handle, "name", name,size);
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");
	err = nvs_set_u8(my_handle, "nameLength", strlen(name));
	printf((err != ESP_OK) ? "MEMORY WRITE Failed!\n" : "Done\n");

	printf("Committing updates in NVS ... ");
	err = nvs_commit(my_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

	//abort();
	esp_restart();
	//nvs_close(my_handle);
}
void readEEPROM()
{
	esp_err_t err;
	uint8_t length;
	size_t size;

	/*printf("Opening Non-Volatile Storage (NVS) handle... ");
	err = nvs_open("storage", NVS_READONLY, &my_handle);
	if (err != ESP_OK) {
		printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	} else
		printf("Done\n");*/

	printf("Reading WIFI USERNAME from NVS ... ");
	err = nvs_get_u8(my_handle, "usernameLength",&length);
	if(err==ESP_OK) printf("Username Length = %d",length);
	size=length;

	err = nvs_get_blob(my_handle, "username", username,&size);
	if(err==ESP_OK) printf("Username = %s",username);
	else if(err==ESP_ERR_NVS_NOT_FOUND){ printf("The value is not initialized yet!\n"); username[0]='a';}
	else printf("Error (%s) reading!\n", esp_err_to_name(err));

	printf("Reading WIFI PASSWORD from NVS ... ");
	err = nvs_get_u8(my_handle, "passwordLength",&length);
	printf("Password Length = %d",length);
	size=length;

	err = nvs_get_blob(my_handle, "password", password,&size);
	if(err==ESP_OK)
		printf("Password = %s",password);
	else if(err==ESP_ERR_NVS_NOT_FOUND)
		printf("The value is not initialized yet!\n");
	else
		printf("Error (%s) reading!\n", esp_err_to_name(err));

	printf("Reading ID from NVS ... ");
	err = nvs_get_u8(my_handle, "idLength",&length);
	printf("ID Length = %d",length);
	size=length;
	err = nvs_get_blob(my_handle, "id", id,&size);
	if(err==ESP_OK)
		printf("ID = %s",id);
	else if(err==ESP_ERR_NVS_NOT_FOUND)
		printf("The value is not initialized yet!\n");
	else
		printf("Error (%s) reading!\n", esp_err_to_name(err));

	printf("Reading Location ID from NVS ... ");
	err = nvs_get_u8(my_handle, "locIDLength",&length);
	printf("location ID Length = %d",length);
	size=length;
	err = nvs_get_blob(my_handle, "locID", locID,&size);
	if(err==ESP_OK)
		printf("Location ID = %s",locID);
	else if(err==ESP_ERR_NVS_NOT_FOUND)
		printf("The value is not initialized yet!\n");
	else
		printf("Error (%s) reading!\n", esp_err_to_name(err));

	printf("Reading heater name from NVS ... ");
	err = nvs_get_u8(my_handle, "nameLength",&length);
	printf("Heater name Length = %d",length);
	size=length;
	err = nvs_get_blob(my_handle, "name", name,&size);
	if(err==ESP_OK)
		printf("heater name = %s",name);
	else if(err==ESP_ERR_NVS_NOT_FOUND)
		printf("The value is not initialized yet!\n");
	else
		printf("Error (%s) reading!\n", esp_err_to_name(err));
}
#endif

int cmpString(char *a,char *b)
{
    int i,j,l,k;
    l=strlen(a);
    k=strlen(b);
    j=1;
    for(i=0;i<l;i++)
    {
        if(b[0]==a[i])
        {
            i++; k--;
            while(k!=0)
            {
                if(b[j]==a[i])
                {
                    j++; i++; k--;
                }
                else
                {
                    j=1;
                    k=strlen(b);
                    break;
                }
            }
            if(k==0)
                return i;
        }
    }
    return 0;
}


#endif
