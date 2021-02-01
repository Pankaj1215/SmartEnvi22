#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
//#include "esp_ota_ops.h"
#include "esp_https_ota.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "ota_update.h"
#include "esp_http_client.h"
#include "communication_msg_handler.h"
#include "version.h"
#include "wifi_core.h"
#include "clock/clock.h"
#include "app.h"

#define BUFFSIZE            1024
#define TEXT_BUFFSIZE       1024

char* g_server_ip;
char* g_filename;
int g_server_port;
int version_major=0,version_minor=0,version_revision=0;

// Original
//extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
//extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");

// Testing.
extern const uint8_t server_certificate_pem_crt_start[] asm("_binary_server_cert_pem_start");
extern const uint8_t server_certificate_pem_crt_end[] asm("_binary_server_cert_pem_end");

static const char *TAG = "ota";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = { 0 };
/* an image total length*/
static int binary_file_length = 0;
/*socket id*/
static int socket_id = -1;
static char http_request[64] = {0};
char rx_buffer[100]={0};

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data);
                strcpy(rx_buffer,evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");

            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}


void simple_ota_example_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA example");

    esp_http_client_config_t config = {
       // .url = "http://54.151.114.229:8080/api/fwdownload?fileName=SmartEnvi22.bin",    // original working one ..
       // .url = "http://54.151.114.229:8081/api/fwdownload?fileName=SmartEnvi22.bin",

		 .url = "https://eheatdev.com/api/fwdownload?fileName=SmartEnvi22.bin",  // Testing ..

		//.url = "http://192.168.43.81/SmartEnvi22.bin",
       // .cert_pem = (char *)certificate_pem_crt_start,  // Original Line..commented on 01Feb2021
		.cert_pem = (char *)server_certificate_pem_crt_start,

		//.skip_cert_common_name_check = true,   // Added for testing as Struct due to openssl cerificate needed..

        .event_handler = _http_event_handle,
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0) {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0';
        config.url = url_buf;
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong firmware upgrade image url");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void send_FW_request()
{
	int status;
	esp_http_client_config_t config = {
		 //  .url = "http://54.151.114.229:8080/api/fwversion?fileName=fw_version.txt",   // Original working one for ota update ..
		  // .url = "http://54.151.114.229:8081/api/fwversion?fileName=fw_version.txt",
		   .url = "https://eheatdev.com/api/fwversion?fileName=fw_version.txt",  // Testing ....

		   //.url = "http://192.168.43.81/fw_version.txt",
		   .event_handler = _http_event_handle,
		};
		esp_http_client_handle_t client = esp_http_client_init(&config);
		esp_err_t err = esp_http_client_perform(client);

		if (err == ESP_OK) {
		   ESP_LOGI(TAG, "Status = %d, content_length = %d",
		           status=esp_http_client_get_status_code(client),
		           esp_http_client_get_content_length(client));
		   	   	   if(status==200)
		   	   		   compare_FW_version(rx_buffer);
		}
		esp_http_client_cleanup(client);
}

void compare_FW_version(char *rx_buffer)
{
	char Delimiter = '}';  // ch - Delimiter
	char *ret;
	char payLoadBuffer[100]={0};
	int endIndex = 0;

	ret = strchr(rx_buffer, Delimiter);
	endIndex = (ret - rx_buffer);
	getSubString(rx_buffer,payLoadBuffer,0,endIndex);
	mainflux_msg_handler(payLoadBuffer, 0);
	printf("\nOTA FIRMWARE VERSION = %d.%d.%d",version_major,version_minor,version_revision);

	if(version_major>FW_VERSION_MAJOR || version_minor>FW_VERSION_MINOR || version_revision>FW_VERSION_REVISION)
		xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
}
void FW_version_task(void *pvParameter)
{
	char run_once_flag=0;
	send_FW_request();
	while(1)
	{
		printf("\nOTA FIRMWARE TASK");
		int yr,mnt,day,hr,min,sec;
		clock_get_date_and_time(&yr,&mnt,&day,&hr, &min, &sec);
		if(app_data->daylightSaving)
		{
			if(hr==23)
				hr=0;
			else
				hr++;
		}
		if(hr==20 && run_once_flag==0)
		{
			send_FW_request();
			run_once_flag=1;
		}
		if(hr!=20)
			run_once_flag=0;
		printf("\nTIME: yr=%d mnt=%d day=%d hr=%d min=%d\r\n",yr,mnt,day, hr, min);
		vTaskDelay(1800000 / portTICK_PERIOD_MS);
	}
}
//read buffer by byte still delim ,return read bytes counts
/*
static int read_until(char *buffer, char delim, int len)
{
//  TODO: delim check,buffer check,further: do an buffer length limited
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}*/

/* resolve a packet from http socket
 * return true if packet including \r\n\r\n that means http packet header finished,start to receive packet body
 * otherwise return false
 * */
/*
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle)
{
    //i means current position
    int i = 0, i_read_len = 0;
    while (text[i] != 0 && i < total_len) {
        i_read_len = read_until(&text[i], '\n', total_len);
        // if we resolve \r\n line,we think packet header is finished
        if (i_read_len == 2) {
            int i_write_len = total_len - (i + 2);
            memset(ota_write_data, 0, BUFFSIZE);
            //copy first http packet body to write buffer
            memcpy(ota_write_data, &(text[i + 2]), i_write_len);

            esp_err_t err = esp_ota_write( update_handle, (const void *)ota_write_data, i_write_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return false;
            } else {
                ESP_LOGI(TAG, "esp_ota_write header OK");
                binary_file_length += i_write_len;
            }
            return true;
        }
        i += i_read_len;
    }
    return false;
}*/

/*static bool connect_to_http_server()
{
    ESP_LOGI(TAG, "Server IP: %s Server Port:%d", 
        g_server_ip, g_server_port);
    sprintf(http_request, "GET %s HTTP/1.1\r\nHost: %s:%d \r\n\r\n",
        g_filename, g_server_ip, g_server_port);
    ESP_LOGI(TAG, "http_request [%s]\n", http_request);

    int  http_connect_flag = -1;
    struct sockaddr_in sock_info;

    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family = AF_INET;
    sock_info.sin_addr.s_addr = inet_addr(g_server_ip);
    sock_info.sin_port = htons(g_server_port);

    // connect to http server
    http_connect_flag = connect(socket_id, (struct sockaddr *)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    } else {
        ESP_LOGI(TAG, "Connected to server");
        return true;
    }
    return false;
}

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    close(socket_id);
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}*/

/*static void ota_update()
{
    esp_err_t err;
    // update handle : set by esp_ota_begin(), must be freed via esp_ota_end()
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA...");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();


    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    // Wait for the callback to set the CONNECTED_BIT in the
    //   event group.
    */
    //replace this with similar handler waiting for connectivity
    /*xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server....");
    */
	/*
    //TODO: THIS IS IMPORTANT
    //connect to http server
    if (connect_to_http_server()) {
        ESP_LOGI(TAG, "Connected to http server");
    } else {
        ESP_LOGE(TAG, "Connect to http server failed!");
        task_fatal_error();
    }

    int res = -1;
    //send GET request to http server
    res = send(socket_id, http_request, strlen(http_request), 0);
    if (res == -1) {
        ESP_LOGE(TAG, "Send GET request to server failed");
        task_fatal_error();
    } else {
        ESP_LOGI(TAG, "Send GET request to server succeeded");
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    bool resp_body_start = false, flag = true;
    //deal with all receive packet
    while (flag) {
        memset(text, 0, TEXT_BUFFSIZE);
        memset(ota_write_data, 0, BUFFSIZE);
        int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (buff_len < 0) { //receive error
            ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
            task_fatal_error();
        } else if (buff_len > 0 && !resp_body_start) { //deal with response header
            memcpy(ota_write_data, text, buff_len);
            resp_body_start = read_past_http_header(text, buff_len, update_handle);
        } else if (buff_len > 0 && resp_body_start) { //deal with response body
            memcpy(ota_write_data, text, buff_len);
            err = esp_ota_write( update_handle, (const void *)ota_write_data, buff_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                task_fatal_error();
            }
            binary_file_length += buff_len;
            ESP_LOGI(TAG, "Have written image length %d", binary_file_length);
        } else if (buff_len == 0) {  //packet over
            flag = false;
            ESP_LOGI(TAG, "Connection closed, all packets received");
            close(socket_id);
        } else {
            ESP_LOGE(TAG, "Unexpected recv result");
        }
    }

    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        task_fatal_error();
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return;
}*/

/*int ota_update_path_start(char* completepath)
{
    return 0;
}

int ota_update_start(char* serverip, int port, char* filename)
{
    g_server_ip = strdup(serverip);
    g_filename  = strdup(filename);
    g_server_port = port;  

    ota_update();
	xTaskCreate(&simple_ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);

    return 0;
}*/


