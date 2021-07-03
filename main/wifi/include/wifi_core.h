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

#ifndef __WIFI_CORE_H__
#define __WIFI_CORE_H__

// unsigned char pairON_blinkWifi = 0;

void WatchDogSOftReset_app_main(void);

void get_wifi_signal_Strength(void);
void getIP_address(void);

// Device Health macro
#define DEVICE_HEALTH_OK                                                             0
#define DEVICE_FREEZE_ALERT                                                          1
#define DEVICE_MALFUNCTION_ZERO_AMBIENT_TEMP_ON_DISPLAY                              2
#define DEVICE_MALFUNCTION_HEATER_STILL_OFF_AMBIENT_TEMP_REACHES_MIN_THRESHOLD       3
#define DEVICE_MALFUNCTION_HEATER_STILL_ON_AMBIENT_TEMP_REACHES_MAX_THRESHOLD        4

#define DEVICE_MALFUNCTION_AMBIENT_TEMP_NOT_CHANGING_AFTER_ONE_HOUR                  5
#define DEVICE_MALFUNCTION_AMBIENT_TEMP_NOT_CHANGING_FIVE_DEG_AFTER_ONE_HOUR         6

#define DEVICE_MALFUNCTION_AMBIENT_TEMP_MAX_THRESHOLD_REACHED                        7

#define DEVICE_HEATER_UNDER_REPAIR                                                   8

// These macro used for identify which command Ack
#define SET_TEMP_ACK    1
#define GET_TEMP_ACK    2

#define HEATER_ON_OFF_ACK   3
// #define HEATER_OFF_ACK  4
#define SET_RGB_ACK     5
#define EN_NIGHT_LIGHT_MODE_ACK     6
#define EN_ANTI_FREEZE_ACK     7

#define RGB_LED_STATE_ACK     8

#define DAY_LIGHT_TIME_STATE_ACK     9

#define SET_THRESHOLD_OFFSET_TIME_ACK     10

#define SET_TEMP_UNIT_ACK     11
#define HEATER_CONFIG_SYNCH_ACK     12


#define PING_DEVICE_ACK    13
#define AUTO_SCREEN_OFF_EN_ACK  14
#define DELETE_HEATER_ACK  15

#define AUTO_DIM_PILOT_EN_ACK  16
#define AUTO_DISPLAY_BRIGHTNESS_EN_ACK  17

#define MANUAL_CHANGE_DISPLAY_BRIGHTNESS_ACK  18
#define DELAY_AUTO_SCREEN_OFF_ACK 19

#define SET_TIMER_MODE_MIN_ACK  20

#define SECHEDULE_SET_FROM_APP_ACK  21
#define GET_DST_STATUS_ACK          22


// #define ACTIVATE_CHILD_LOCK_ACK     8

#define  MANUALLY_SET_TEMP_CHANGE                1
#define  MANUALLY_NIGHT_LIGHT_STATE_CHANGE       2
#define  MANUALLY_TEMP_UNIT_CHANGE               3
#define  MANUALLY_RESET_SSID_PASS_ENABLE         4
#define  MANUALLY_DAY_LIGHT_ON_OFF_CHANGE_ENABLE 5
#define  MANUALLY_PUT_HEATER_UNDER_REPAIR_ENABLE 6
#define  MANUALLY_RGB_MODE_CHANGED               7
#define  MANUALLY_DIM_PILOT_LIGHT_CHANGED        8
#define  MANUALLY_AUTO_SCREEN_BRIGHTNESS_CHANGED 9
#define  MANUALLY_AUTO_SCREEN_OFF_CHANGED        10
#define  MANUALLY_DELAY_AUTO_SCREEN_OFF_CHANGED  11
#define  MANUALLY_SET_BRIGHTNESS_CHANGED         12
#define  MANUALLY_TIMER_MODE_MIN_CHANGED         13

#define  HEATER_STATE_CHANGE_DATA_TO_AWS         14
#define  THERMOSTAT_STATE_CHANGE_DATA_TO_AWS     15
#define  AMBIENT_TEMP_CHANGE_DATA_TO_AWS         16







int  getSubString(char *source, char *target,int from, int to);
void heater_state_change_task(void *param);


#define ESP32_WIFI_UNKNOWN      0
#define ESP32_WIFI_CLIENT       1
#define ESP32_WIFI_AP           2
#define ESP32_WIFI_WPS          3
#define ESP32_WIFI_SLEEP        4
#define ESP32_ERROR             5

#define WEB_SVR_STAT_ERROR      -1
#define WEB_SVR_STAT_UNKNOWN    0
#define WEB_SVR_STAT_RUNNING    1
#define WEB_SVR_STAT_STOP       2

#define MAX_REPLY_BUF           1000
#define PARSER_KEY              "&"

#define P_TESTING     // Added only for Pankaj_07Oct2020

// unsigned char commandReceived_SendAck = 0;


/*!
 * \fn int wifi_version()
 * \brief returns the firmware wifi version
 * \return returns wifi version
 */
int wifi_version();

/*!
 * \fn int esp32_wifi_config(int mode, char* ssid, char* password)
 * \brief set the wifi configuration
 * \param mode wifi mode STA or AP
 * \param ssid name of the wifi to connect if STA, or ap name if AP
 * \param password password if STA, password of ap if mode is AP
 * \return returns success 
 */
int esp32_wifi_config(int mode, char* ssid, char* password);


/*!
 * \fn int esp32_initialise_wifi(void)
 * \brief initialize wifi device
 * \return always success
 */
int esp32_initialise_wifi(void);

int esp32_send_msg(char* destination, char* msgtosend, int lenght);
int esp32_receive_msg(char* toreceive, int lenght);

/*!
 * \fn int esp32_wps_enable(void)
 * \brief enable wps mode
 * \return returns success 
 */
int esp32_wps_enable(void);

/*!
 * \fn int esp32_wps_disable(void)
 * \brief disable wps mode
 * \return returns success 
 */
int esp32_wps_disable(void);

/*!
 * \fn int esp32_wifi_client_enable(char* ssid, char* pw)
 * \brief set wifi device to client mode
 * \param ssid wifi will connect to this ssid
 * \param mode pw will use this password to remote wifi ap
 * \return returns success 
 */
int esp32_wifi_client_enable(char* ssid, char* pw);

int esp32_wifi_client_enable_Testing_menu(char* ssid, char* pw);  // Added for Testing only

/*!
 * \fn int esp32_wifi_ap_enable(char* ssid_ap, char *pw)
 * \brief set wifi to ap mode
 * \param ssid_ap set the ap mode ssid
 * \param pw set the ap mode password
 * \return returns success 
 */
int esp32_wifi_ap_enable(char* ssid_ap, char *pw);

/*!
 * \fn int esp32_wifi_ap_disable(void)
 * \brief set wifi to ap mode to disable
 * \return returns success 
 */
int esp32_wifi_ap_disable(void);

/*!
 * \fn _Bool esp32_wifi_is_ap_enabled(void)
 * \brief check if wifi is in ap or not
 * \return return true if yes, falso if no
 */
_Bool esp32_wifi_is_ap_enabled(void);

/*!
 * \fn unsigned char *esp32_wifi_ap_get_mac(void)
 * \brief get the location of mac address string in ap mode
 * \return static address location of mac address
 */
unsigned char *esp32_wifi_ap_get_mac(void);

/*!
 * \fn unsigned char *esp32_wifi_client_get_mac(void)
 * \brief get the location of mac address string in client mode
 * \return static address location of mac address
 */
unsigned char *esp32_wifi_client_get_mac(void);

int esp32_wifi_scan(void* variable_args);
int esp32_wake_up(void* param);
int esp32_sleep(void);

/*!
 * \fn int esp32_reg_wifi_msg_callback(int (*msg_callbk)(char* msg, char* ret_reply))
 * \brief set a callback when wifi receive REST message
 * \param msg_callbk function pointer to be executed when wifi gets REST msg
 * \param ret_reply return string when message was handled
 * \return returns success 
 */
int esp32_reg_wifi_msg_callback(int (*msg_callbk)(char* msg, char* ret_reply));

/*!
 * \fn int esp32_reg_wifi_conn_callback(int (*wifi_conn_cb)(int conn_stat))
 * \brief sets the callback when wifi changed status
 * \param wifi_conn_cb function pointer to be executed when wifi stat changed
 * \param conn_stat wifi status
 * \return returns success 
 */
int esp32_reg_wifi_conn_callback(int (*wifi_conn_cb)(int conn_stat));


void get_NTP_Time(void); // new added for NTP testing_08Nov2020

// static void tcp_server_task(void *pvParameters); // Original
void tcp_server_task(void *pvParameters); // Testing on 13Dec2020


void Temp_MalfunctionTask(void *param);

#endif //__WIFI_CORE_H__
