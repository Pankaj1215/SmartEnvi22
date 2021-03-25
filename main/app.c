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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
#include "ping/ping.h"
#include "esp_ping.h"
#include "sdkconfig.h"

#include "button.h"
#include "display.h"
#include "led.h"
#include "tempsensor.h"
#include "lightsensor.h"
#include "heater.h"
#include "clock/clock.h"
#include "ntp.h"
#include "communication_server.h"
#include "esp_wifi.h"
#include "non_volatile_lib.h"
//#include "kaa_service.h"
#include "lucidtron_core.h"
#include "version.h"

#include "app.h"

// New Added _PS07Oct2020
#include "wifi_core.h"   // New Added for accessing macro #define P_TESTING which is placed in wifi_core.h

#define pilot_light_on() led1_on(); led2_on();
#define pilot_light_off() led1_off(); led2_off();
#define pilot_light_set_brightness(x) led1_set_brightness(x); led2_set_brightness(x);

// night light  // Original Line
//#define LED_R_POS 0
//#define LED_G_POS 8
//#define LED_B_POS 16
//
//#define LED_R_MASK 0x0000FF
//#define LED_G_MASK 0x00FF00
//#define LED_B_MASK 0xFF0000

// Only For tesing
// #define LED_R_POS 0
#define LED_R_POS 16
#define LED_G_POS 8
// #define LED_B_POS 16
#define LED_B_POS 0


// #define LED_R_MASK 0x0000FF
#define LED_R_MASK 0xFF0000

#define LED_G_MASK 0x00FF00

// #define LED_B_MASK 0xFF0000
#define LED_B_MASK 0x0000FF


#define GET_LED_R_VAL(BR) ((BR & LED_R_MASK) >> LED_R_POS)
#define GET_LED_G_VAL(BR) ((BR & LED_G_MASK) >> LED_G_POS)
#define GET_LED_B_VAL(BR) ((BR & LED_B_MASK) >> LED_B_POS)

#define night_light_set_br(r,g,b) {led_r_set_brightness(r);led_g_set_brightness(g);led_b_set_brightness(b);}
#define night_light_off() {led_r_off();led_g_off();led_b_off();}

// #define fahr_to_celsius(f) ((f - 32) * 5 / 9)
#define fahr_to_celsius(f) ((f - 32.0) * 5.0 / 9.0)

// #define celsius_to_fahr(c) (c * 9 / 5 + 32)
#define celsius_to_fahr(c) (c * 9.0 / 5.0 + 32.0)

static void app_task(void *param);
static void standby_mode_task(app_data_t *data);
static void manual_temperature_mode_task(app_data_t *data);
static void temperature_offset_set_mode_task(app_data_t *data);
static void timer_increment_mode_task(app_data_t *data);
static void auto_mode_task(app_data_t *data);
static void debug_mode_task(app_data_t *data);
static void menu_mode_task(app_data_t *data);
static void button_up_cb(int level);
static void button_down_cb(int level);
static void button_power_back_cb(int level);
static void button_timer_forward_cb(int level);
//static app_mode_t menu_energy(app_data_t *data);
static app_mode_t menu_calendar(app_data_t *data);
static app_mode_t menu_time_and_date(app_data_t *data);
static app_mode_t menu_communications(app_data_t *data);
static app_mode_t menu_settings(app_data_t *data);
static app_mode_t menu_display_settings(app_data_t *data);
static app_mode_t menu_update(app_data_t *data);
static void temp_sensor_task(void *param);
static void light_sensor_task(void *param);
static void pilot_light_task(void *param);
static void night_light_task(void *param);
static void ping_task(void *param);
static void display_brightness_task(void *param);
static int wifi_conn_stat(int stat);

// app_data_t *app_data = NULL;  // Original
//extern app_data_t *app_data; // TESTING // changed for wifi Icon
//static struct comm_wifi *comm_wifi_dev = NULL;  // Original  Commented only for Testing
extern struct comm_wifi *comm_wifi_dev = NULL; // Testing

static wifi_ap_record_t ap_info;

auto_mode_sched_t sched_weekday[AUTO_MODE_SCHED_NUM];
auto_mode_sched_t sched_weekend[AUTO_MODE_SCHED_NUM];

#ifdef P_TESTING
// void aws_iot_task(void *pvParameters);  // Commented on 14Dec2020 // ADDED FOR TESTING ..PS28aUG
// void initialise_wifi(void);  // Added for testing Wifi Testing _ P_16Sept2020
void tcpServer_main();

// extern unsigned char maxTemperatureThresholdReachedWarning;
// extern unsigned char minTemperatureThresholdReachedWarning;
//extern unsigned char setTempThresholdOffsetCrossed;
extern unsigned char rgb_led_state;
//extern unsigned char ambientTempChangeDataToAWS;


unsigned char oneTimeRegistrationPacketToAWS;
unsigned char pingDeviceEnablePilotLed = 0;
// extern unsigned char daylightSaving;   // New Added for Day light on Off
// bool daylightSaving;   // New Added for Day light on Off
// unsigned char daylightSaving;   // New Added for Day light on Off
// int daylightSaving;   // New Added for Day light on Off

#define NTP_Testing_dayLightSaving  // New testing Added ..on 15_51pm

extern unsigned char en_anti_freeze;
extern unsigned char heater_On_Off_state_by_command;
extern unsigned char manaully_Set_Temp_change;
extern unsigned char manaully_night_Light_State_change;
extern unsigned char manaully_child_Lock_State_change;
extern unsigned char manaully_Temp_unit_change;
extern unsigned char manaully_reset_ssid_pass_enable;
extern unsigned char manually_day_light_on_off_change_enable;

extern unsigned char device_health_status;

extern unsigned char FlashEraseEnableAPMode; // Added for direct AP mode enable After Flash erased.

#ifdef HeaterUnderReapir
unsigned char heater_underControl_status = 0;
extern unsigned char manually_put_heater_under_repair_enable;
extern unsigned char manually_put_heater_under_repair_status_for_malfunctionMonitor;

extern int nlight_br_TestingInSynchPacket;
extern bool PairDataRecievedFromAPP;

#endif

unsigned char heater_On_Off_state_by_command_ExistFromStandByMode = 0;
// Threshold_Offset 30Minute calculation ..

//#define MalfunctionTaskIncludedInTempTask
#ifdef MalfunctionTaskIncludedInTempTask
time_t TempChange_ms = 0;
int time_OneMinuteOver = 0;
int time_count = 0;
#endif

extern unsigned char TimerIntervalThresholdOffset;
// extern unsigned char Hysteris_Thresh_Off_Set_UnderWarning;
// extern unsigned char Hysteris_Thresh_Off_Set_OverWarning;
//extern unsigned char TimeInterval_Thresh_OffSet_UnderWarning;
//extern unsigned char TimeInterval_Thresh_OffSet_OverWarning;

extern bool pairON_blinkWifi;
unsigned char uchUpdateDoneOnce = 1;

#endif

// static int timezone_offset_list_min[] = { //  Original old Firmware..
const int timezone_offset_list_min[] = {
    -720,        //UTC−12:00
    -660,        //UTC−11:00
    -600,        //UTC−10:00
    -570,        //UTC−09:30
    -540,        //UTC−09:00
    -480,        //UTC−08:00
    -420,        //UTC−07:00
    -360,        //UTC−06:00
    -300,        //UTC−05:00
    -240,        //UTC−04:00
    -210,        //UTC−03:30
    -180,        //UTC−03:00
    -120,        //UTC−02:00
    -60,         //UTC−01:00
    0,           //UTC±00:00
    60,          //UTC+01:00
    120,         //UTC+02:00
    180,         //UTC+03:00
    210,         //UTC+03:30
    240,         //UTC+04:00
    270,         //UTC+04:30
    300,         //UTC+05:00
    330,         //UTC+05:30
    345,         //UTC+05:45
    360,         //UTC+06:00
    390,         //UTC+06:30
    420,         //UTC+07:00
    480,         //UTC+08:00
    525,         //UTC+08:45
    540,         //UTC+09:00
    570,         //UTC+09:30
    600,         //UTC+10:00
    630,         //UTC+10:30
    660,         //UTC+11:00
    720,         //UTC+12:00
    765,         //UTC+12:45
    780,         //UTC+13:00
    840,         //UTC+14:00
};
#define TIMEZONE_OFFSET_LIST_SIZE (sizeof(timezone_offset_list_min)/sizeof(int))

#define TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
#ifdef TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
    unsigned char menuModeKeypressedFlag =0;
#endif

    void pairOnPilotLedBlinking(void);
    unsigned char paringOnFlag =0;

    unsigned char AutoDisplayOffInParingOnFlag =0;

    time_t timePairingDur_ms= 0;
    int pairtime_OneMinuteOver = 0;
    int pair_TenMin_count = 0;

time_t ping_Dur_ms= 0;
int ping_TwentySecFirstIterationOver = 0;
int ping_TwentySecOver = 0;

 extern bool pingDeviceOnFlag;

void init_Variables(void);

void init_Variables(void){
en_anti_freeze = 1;  // It is used to enable anti freeze logic defualt ON.
rgb_led_state = 1;

manually_put_heater_under_repair_status_for_malfunctionMonitor = 1;

// app_data->mode
 heater_On_Off_state_by_command = app_data->lastHeaterState; //by default OFF

 heater_off();//initially heater will be off ..added on 02Dec2020

printf("init variables heater_On_Off_state_by_command %d app_data->lastHeaterState %d",heater_On_Off_state_by_command,app_data->lastHeaterState);
}

void testFunctionFoFToC(void);
#include "esp_flash_encrypt.h"
// #define TEST_F_C_Cabration
#ifdef TEST_F_C_Cabration
void testFunctionFoFToC(void){

	int temp_f,temp_c,count_f,count_c; float lf_temp =0; float lf_temp_roundOff =0; int new_intTemp =0;
	count_f = 30;
	while(count_f <= 100)
	{
		temp_f = count_f;
		// temp_c = fahr_to_celsius(temp_f);
		// temp_c = fahr_to_celsius(temp_f);
    	lf_temp = fahr_to_celsius(temp_f);
    	lf_temp_roundOff = round(lf_temp);
    	new_intTemp = lf_temp_roundOff;
    	temp_c = new_intTemp;

    	temp_c  = valueRoundOff(temp_f, CONVERT_F_TO_C);

		// printf("For F to C temp_f: %d , temp_c: %d \n", temp_f, temp_c );
    	printf("F to C temp_c: %d ,temp_f: %d, lf_temp %f,lf_temp_roundOff %f,new_intTemp %d \n", temp_c, temp_f,lf_temp, lf_temp_roundOff, new_intTemp );
    	count_f++;
	}
    printf("\n");

     lf_temp =0;  lf_temp_roundOff =0;  new_intTemp =0;
    count_c = 0; count_f = 0; temp_c = 0;	temp_f = 0;
	while(count_c <= 38 )
	{
    	temp_c = count_c;
	    temp_f = celsius_to_fahr(temp_c);
		// printf("For C to F  temp_c: %d , temp_f: %d \n", temp_c, temp_f );
    //	lf_temp = (float)celsius_to_fahr(count_c);
    	lf_temp = celsius_to_fahr(count_c);
    	lf_temp_roundOff = round(lf_temp);
    	new_intTemp = lf_temp_roundOff;

    	new_intTemp  = valueRoundOff(count_c, CONVERT_C_TO_F);

    	printf("CtoF temp_c: %d ,temp_f: %d, lf_temp %f,lf_temp_roundOff %f,new_intTemp %d \n", temp_c, temp_f,lf_temp, lf_temp_roundOff, new_intTemp );
		count_c++;
	}
	 printf("\n");
//
//	   if(esp_flash_encryption_enabled()){
//	      printf("Flash Encryption is Enabled ----->>>>>>>>\n");
//	   }
//	   else{
//	      printf("Flash Encryption is NOT Enabled ------<<<<<<<<<\n");
//	   }

}
#endif

// #define Test_Storage
static void print_fw_version(void)
{
	// testFunctionFoFToC();
    //  char fw_version[100];
    char fwVersion[8];
    sprintf(fwVersion,"%d.%d.%d",FW_VERSION_MAJOR,FW_VERSION_MINOR,FW_VERSION_REVISION);
   // get_version(fw_version);
   //   ESP_LOGI("firmware_version", "%s", fw_version);
    ESP_LOGI("firmware_version", "%s", fwVersion);
    // Added For testing only ..
   //  display_clear_screen();
   //   display_menu("Firm_ver", DISPLAY_COLOR, fw_version, DISPLAY_COLOR);
   // display_menu("Firm_ver", DISPLAY_COLOR, fwVersion, DISPLAY_COLOR);
  //   vTaskDelay(3000); //    // wait for at least Firmware version..
    printf("FIRMWARE VERSION: %s\n",fwVersion);
}

void test_Display_wifi_strenth(void)
{display_clear_screen();
// while(1){
	 //get_wifi_signal_Strength();
	 display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing
	 vTaskDelay(5000);
//	 display_clear_screen();
//	 display_wifi_level_1_icon(DISPLAY_COLOR);
//	vTaskDelay(2000);
//	display_clear_screen();
//	display_wifi_level_2_icon(DISPLAY_COLOR);
//	vTaskDelay(2000);
//	display_clear_screen();
//	display_wifi_level_3_icon(DISPLAY_COLOR);
//	vTaskDelay(2000);
//	display_clear_screen();
//	 display_wifi_level_4_icon(DISPLAY_COLOR);
//	// vTaskDelay(2000);
//	// display_clear_screen();
//	//display_wifi_level_5_icon(DISPLAY_COLOR);
//	 vTaskDelay(2000);
    //}
}

void Display_uniqueID_onbootup(void);
// Original One..
//void Display_uniqueID_onbootup(void)
//{
//	display_clear_screen();
//	 display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing
//	 vTaskDelay(10000);
//}

// Testing one
void Display_uniqueID_onbootup(void)
{
	if(FlashEraseEnableAPMode ==1)
	{   // FlashEraseEnableAPMode = 0;
		 esp32_wifi_ap_enable(uniqueDeviceID, ap_password); printf("In Display_uniqueID_onbootup  Enable AP Mode \n");
	     display_clear_screen();
//	     display_menu_pair_Heater("pair on", DISPLAY_COLOR, uniqueDeviceID, DISPLAY_COLOR);
	     display_menu_pair_Heater("Connect to", DISPLAY_COLOR, uniqueDeviceID, DISPLAY_COLOR);

	     vTaskDelay(10000);
	}
	else
	{
		 display_clear_screen();
		 display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing
		 vTaskDelay(10000);
	}
}


// void WatchDogSOftReset_app_main(void);
// #define Test_Storage
esp_err_t app_init(void) {
    print_fw_version();  // Comment FW_version...
   // test_Display_wifi_strenth();
   // pairOnPilotLedBlinking();

	// WatchDogSOftReset_app_main();  // Tesing soft_RESET

#ifdef Test_Storage
 printf("Before erase \n");
  erase_storage_all();
 printf("After erase \n");
#endif

   // while(1){}; // Testing only

    esp_err_t ret = ESP_OK;
    time_t t_start_ms = 0;

    if (app_data != NULL)
        return ESP_OK;
    if (app_data == NULL)
        app_data = malloc(sizeof(app_data_t));
    if (app_data == NULL)
        return ESP_FAIL;

    // welcome screen
    ret |= display_clear_screen();
    ret |= display_welcome_screen(DISPLAY_COLOR);

    // system time should be after manufacturing year
    int year;
    clock_get_date(&year, NULL, NULL);
    if (year < MANUFACTURING_YEAR)
        clock_set_year(MANUFACTURING_YEAR);

    t_start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // set default values
    app_data->mode = 0;
    app_data->is_connected = false;
    app_data->internet_conn = false;
    app_data->button_status = 0;
    app_data->ambient_temperature_celsius = -1;
    app_data->ambient_temperature_offset_celsius = TEMPERATURE_SENSOR_OFFSET_CELSIUS_DEF;

#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    app_data->manual_temperature_celsius = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_DEF;
    app_data->manual_temperature_fahrenheit = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_DEF;
    app_data->lastHeaterState = DEFAULT_LAST_HEATER_STATE;   // New Added for storing last status of Heater
#else
    app_data->manual_temperature_celsius = TEMPERATURE_CELSIUS_VAL_DEF;
    app_data->manual_temperature_fahrenheit = TEMPERATURE_FAHRENHEIT_VAL_DEF;
#endif

    app_data->current_timer_setting_min = 0;
    app_data->last_timer_setting_min = LAST_TIMER_SETTING_MIN_DEF;
    app_data->is_child_lock_active = false;
    app_data->is_auto_time_date_en = false;
    app_data->timezone_offset_idx = TIMEZONE_OFFSET_INDEX_DEF;
    app_data->ambient_light = 0;
    app_data->pilot_light_brightness = 0;
    app_data->night_light_cfg = NIGHT_LIGHT_VAL_DEF;
    app_data->settings.temperature_unit = TEMP_UNIT_FAHRENHEIT;

    app_data->settings.is_child_lock_en = true;
    app_data->settings.is_dim_pilot_light_en = false;
    app_data->settings.is_night_light_auto_brightness_en = false;
    app_data->settings.temperature_hysteresis_celsius = TEMPERATURE_HYSTERESIS_CELSIUS_DEFAULT;
    app_data->settings.temperature_hysteresis_fahrenheit = TEMPERATURE_HYSTERESIS_FAHRENHEIT_DEFAULT;
    app_data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_MAX;
    app_data->display_settings.is_auto_display_brightness_en = false;

     app_data->display_settings.is_auto_screen_off_en = true; // Original or old firmware
    // app_data->display_settings.is_auto_screen_off_en = false; //CR not yet Approved. asked by client to change to never dispaly mode by default.

    app_data->display_settings.auto_screen_off_delay_sec = AUTO_SCREEN_OFF_DELAY_SEC_DEFAULT;

    printf(" in begining app_data->display_settings.auto_screen_off_delay_sec: %d",app_data->display_settings.auto_screen_off_delay_sec);

    app_data->daylightSaving = 0; // Default variable

    memset(app_data->ap_ssid, 0, sizeof(app_data->ap_ssid));
    memset(app_data->ap_pw, 0, sizeof(app_data->ap_pw));
    memset(app_data->sta_ssid, 0, sizeof(app_data->sta_ssid));
    memset(app_data->sta_pw, 0, sizeof(app_data->sta_pw));

    // set initial values from the saved configuration from flash
    get_integer_from_storage(STORAGE_KEY_TEMP_SENSOR_OFFSET_CELSIUS, &(app_data->ambient_temperature_offset_celsius));
    get_integer_from_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, &(app_data->manual_temperature_celsius));
    get_integer_from_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, &(app_data->manual_temperature_fahrenheit));
    get_integer_from_storage(STORAGE_KEY_LAST_TIMER_SETTING, &(app_data->last_timer_setting_min));
    get_integer_from_storage(STORAGE_KEY_IS_AUTO_TIME_DATE_EN, (int *) &(app_data->is_auto_time_date_en));

    get_integer_from_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, &(app_data->daylightSaving));  // working..
   // get_integer_from_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int *) &(app_data->daylightSaving));
    // Ony for testing..
   if(app_data->daylightSaving  > 1 )
   // if(!((app_data->daylightSaving == 1 ) |(app_data->daylightSaving == 0)))
   {	app_data->daylightSaving = 0; printf("daylightSaving is greater that one \n");}

    printf("Initially app_data->daylightSaving %d \n",app_data->daylightSaving);

	//TimerIntervalThresholdOffset = 30;
	// set_integer_to_storage(STORAGE_KEY_THRESHOLD_OFFSET_TIME, (int)app_data-> TimerIntervalThresholdOffset);
    app_data-> TimerIntervalThresholdOffset = 15; /// Default Value
   // set_integer_to_storage(STORAGE_KEY_THRESHOLD_OFFSET_TIME, (int)app_data-> TimerIntervalThresholdOffset);

 //   get_integer_from_storage(STORAGE_KEY_THRESHOLD_OFFSET_TIME, &(app_data-> TimerIntervalThresholdOffset));
    get_integer_from_storage(STORAGE_KEY_THRESHOLD_OFFSET_TIME, (int *) &(app_data-> TimerIntervalThresholdOffset));

    TimerIntervalThresholdOffset = app_data-> TimerIntervalThresholdOffset;
    printf("TimerIntervalThresholdOffset %d \n",app_data-> TimerIntervalThresholdOffset);

#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    get_integer_from_storage(STORAGE_KEY_LAST_HEATER_STATE, (int *) &(app_data->lastHeaterState));
    printf("app_data->lastHeaterState %d \n",app_data->lastHeaterState);
#endif

    init_Variables();
    get_integer_from_storage(STORAGE_KEY_TIMEZONE_OFFSET_INDEX, &(app_data->timezone_offset_idx));
    get_integer_from_storage(STORAGE_KEY_NIGHT_LIGHT_CFG, &(app_data->night_light_cfg));

    app_data->night_light_cfg = 16711888; // Only For testing ..

    get_data_from_storage(STORAGE_KEY_SETTINGS, &(app_data->settings));
    printf("\n app_data->settings.is_child_lock_en %d \n",    app_data->settings.is_child_lock_en);

    get_data_from_storage(STORAGE_KEY_DISPLAY_SETTINGS, &(app_data->display_settings));
    printf("\n\n  app_data->settings.is_night_light_auto_brightness_en  %d \n",   app_data->settings.is_night_light_auto_brightness_en );

    printf("data->display_settings.auto_screen_off_delay_sec  %d\n ",app_data->display_settings.auto_screen_off_delay_sec );

//     if(app_data->daylightSaving == 1)
//        printf("daylightSaving  is One %d \n",app_data->daylightSaving);
//     else if(app_data->daylightSaving ==0)
//     	printf("daylightSaving zero %d \n",app_data->daylightSaving);
//     else
//     	printf("unvalid state\n ");

    clock_set_timezone_offset(timezone_offset_list_min[app_data->timezone_offset_idx]);

    // set default value of schedule
    for (int i = 0; i < AUTO_MODE_SCHED_NUM; i++) {
        sched_weekday[i].en = false;
        sched_weekday[i].hour = 0;
        sched_weekday[i].minute = 0;
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        sched_weekday[i].temp_c = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_DEF;
        sched_weekday[i].temp_f = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_DEF;
#else
        sched_weekday[i].temp_c = TEMPERATURE_CELSIUS_VAL_DEF;
        sched_weekday[i].temp_f = TEMPERATURE_FAHRENHEIT_VAL_DEF;
#endif
        sched_weekend[i].en = false;
        sched_weekend[i].hour = 0;
        sched_weekend[i].minute = 0;
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        sched_weekend[i].temp_c = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_DEF;
        sched_weekend[i].temp_f = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_DEF;
#else
        sched_weekend[i].temp_c = TEMPERATURE_CELSIUS_VAL_DEF;
        sched_weekend[i].temp_f = TEMPERATURE_FAHRENHEIT_VAL_DEF;
#endif
    }

    // load schedule from flash
    get_data_from_storage(STORAGE_KEY_SCHED_WEEKDAY, sched_weekday);
    get_data_from_storage(STORAGE_KEY_SCHED_WEEKEND, sched_weekend);

    ret |= button_up_set_cb(button_up_cb);
    ret |= button_down_set_cb(button_down_cb);
    ret |= button_power_back_set_cb(button_power_back_cb);
    ret |= button_timer_forward_set_cb(button_timer_forward_cb);

    int *stat = &(app_data->button_status);
    if (button_up_get_level())
        *stat |= 1 << BUTTON_UP_STAT;

    if (button_down_get_level())
        *stat |= 1 << BUTTON_DOWN_STAT;

    if (button_power_back_get_level())
        *stat |= 1 << BUTTON_POWER_BACK_STAT;

    if (button_timer_forward_get_level())
        *stat |= 1 << BUTTON_TIMER_FORWARD_STAT;

// set Wi-Fi status change callback
//   set_wifi_conn_status_change_cb(wifi_conn_stat);
////    // initialize and start communication service
//   initialize_communication_service();
//    comm_wifi_dev = get_wifi_dev();

//#ifdef P_TESTING   // Added for Testing  commented on 30Nov shifting it after other tasks get operational
//     tcpServer_main();
//#endif
    // wait for at least APP_WELCOME_SCREEN_DELAY_MS
    if ((xTaskGetTickCount() * portTICK_PERIOD_MS - t_start_ms) < APP_WELCOME_SCREEN_DELAY_MS)
        vTaskDelay(APP_WELCOME_SCREEN_DELAY_MS - (xTaskGetTickCount() * portTICK_PERIOD_MS - t_start_ms) / portTICK_RATE_MS);

    // start app task
    xTaskCreate(app_task, "app_task", 4096, (void *)app_data, 12, NULL);
    return ret;
}


void pairOnPilotLedBlinking(void)
{
// while(1){
	pilot_light_on();
	vTaskDelay(500);
	pilot_light_off();
	vTaskDelay(500);
// }
}


static void app_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    app_mode_t *mode = &(data->mode);

    // start at default mode
     *mode = APP_MODE_ON_STARTUP;   // original ...// commented only for testing...

    // start task that reads ambient temperature
     xTaskCreate(temp_sensor_task, "tsensor_task", 4096, (void *)app_data, 12, NULL);
    // start task that reads ambient light
     xTaskCreate(light_sensor_task, "lsensor_task", 4096, (void *)app_data, 12, NULL);
    // start task that controls pilot light

     xTaskCreate(pilot_light_task, "plight_task", 4096, (void *)app_data, 12, NULL);

    // start task that controls night light
     xTaskCreate(night_light_task, "nlight_task", 4096, (void *)app_data, 12, NULL);
    // start task that controls display brightness
     xTaskCreate(display_brightness_task, "dbright_task", 4096, (void *)app_data, 12, NULL);

#ifdef P_TESTING   // New Added here after other task  get operational..30Nov2020
     tcpServer_main();
#endif
     xTaskCreate(heater_state_change_task, "hscan_task", 4096, (void *)app_data, 12, NULL);
     xTaskCreate(Temp_MalfunctionTask, "tMalFunc_task", 4096, (void *)app_data, 12, NULL);

     Display_uniqueID_onbootup(); // Testing only ////

    while (1) {
        switch (*mode) {
        case APP_MODE_STANDBY:
        {   printf("APP_MODE_STANDBY\r\n"); standby_mode_task(data);
            break; }
        case APP_MODE_MANUAL_TEMPERATURE:
        {   printf("APP_MODE_MANUAL_TEMPERATURE\r\n");   manual_temperature_mode_task(data);
            break;   }
        case APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET:
        {    printf("APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET\r\n");  temperature_offset_set_mode_task(data);
            break;   }
        case APP_MODE_TIMER_INCREMENT:
        {    printf("APP_MODE_TIMER_INCREMENT\r\n");   timer_increment_mode_task((void *)data);
            break;   }
        case APP_MODE_AUTO:
        {   printf("APP_MODE_AUTO\r\n");   auto_mode_task((void *)data);
            break;   }
        case APP_MODE_MENU:
        {   printf("APP_MODE_MENU\r\n");   menu_mode_task((void *)data);
            break;   }
        case APP_MODE_DEBUG:
        {    printf("APP_MODE_DEBUG\r\n");  debug_mode_task((void *)data);
            break;   }
        }// end of switch
        vTaskDelay(1 / portTICK_RATE_MS);
    }// end of while
} //static void app_task(void *param) {

static void standby_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = 0;
    app_mode_t *mode = &(data->mode);
    time_t btn_up_press_ms = 0, btn_down_press_ms = 0, btn_timer_press_ms = 0;

    unsigned char uchPairDoneDisplayOnceFlag = 0;

    bool update_display = true;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // clear screen
    display_clear_screen();

    bool prev_pairON_blinkWifi = pairON_blinkWifi ;
	bool prev_oneTimeRegistrationPacketToAWS = oneTimeRegistrationPacketToAWS ;

    // Added for testing begin 03March2021 _Comment not approved CR for standby mode AP off.
//	if(FlashEraseEnableAPMode ==1)
//	{    // FlashEraseEnableAPMode = 0;
//		paringOnFlag =0; AutoDisplayOffInParingOnFlag =0;
//		esp32_wifi_client_enable(username,password);   printf("In standby mode  Disable  AP Mode \n");
//	}

    // cr not appproved , so comment this ..after testing..
    if((FlashEraseEnableAPMode == 1) ||(pairON_blinkWifi == 1))
	// if(FlashEraseEnableAPMode ==1)
	{       data->display_settings.auto_screen_off_delay_sec = 180;
	     //  data->display_settings.is_auto_screen_off_en = false;
	}

     // Test end


    // wait until power button is released
    while (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) vTaskDelay(1 / portTICK_RATE_MS);

    // this is necessary to disregard button toggle while waiting for power button released
    prev_btn = *btn;

   // if(heater_On_Off_state_by_command == 1){
  // if(heater_On_Off_state_by_command == 0){
    // turn off heater
    heater_off();
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    // app_data->lastHeaterState = false;
    // set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
     printf("in Stand by Mode initial status of  app_data->lastHeaterState %d \n",app_data->lastHeaterState);
#endif
   // } //end of if(heater_On_Off_state_by_command == 1){

//		if(FlashEraseEnableAPMode ==1)
//		{  esp32_wifi_ap_enable(uniqueDeviceID, ap_password); printf("In stand by mode  Enable AP Mode \n");}

    while(*mode == APP_MODE_STANDBY) {
        /* button
           - single press POWER button to enter Manual Temperature mode
           - long press UP and DOWN buttons to activate/deactivate child lock
           - long press DOWN and TIMER button to enter Debug mode
           - long press DOWN button to enter Menu mode
           - long press UP and TIMER buttons to enter Temperature sensor offset set mode
        //   - - long press UP and TIMER buttons to enter Temperature sensor offset set mode
        */
            // Testing begin Added on 28Feb2021
//    		if(FlashEraseEnableAPMode ==1)
//    		{  esp32_wifi_ap_enable(uniqueDeviceID, ap_password); printf("In stand by mode  Enable AP Mode \n");}
    	   //  Testing End


#ifdef TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
     menuModeKeypressedFlag = 0;
#endif

     // printf(" before out of stand by mode by Heater ON commmand \n");
     if(heater_On_Off_state_by_command_ExistFromStandByMode ==1  || heater_On_Off_state_by_command == 1 ){ // New Added for Coming out of stand by mode  //
    	 *mode = APP_MODE_MANUAL_TEMPERATURE; printf(" Out of stand by mode by Heater ON commmand \n"); heater_On_Off_state_by_command =0; heater_On_Off_state_by_command_ExistFromStandByMode =0;}
     //    printf(" After Out of stand by mode by Heater ON commmand \n");

        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up or down is pressed
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {

                           //  Original Lines commented..Begin
                            data->is_child_lock_active = !data->is_child_lock_active;
                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS
                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);
                           //  Original Ends

                            // This logic worked commented for check direct Ap mode afterFlash Erase.
//                        	// Testing On
//                            uchPairHeaterFlag = 1;
//                        	if(uchPairHeaterFlag == 1)
//                        	 {
//							   display_clear_screen();
//							   esp32_wifi_ap_enable(uniqueDeviceID, ap_password);   // Testing Line..
//							   while(uchPairHeaterFlag){
//								  // display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing
//								  display_menu("Pair on", DISPLAY_COLOR, "heater", DISPLAY_COLOR);
//
////								  if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up or down is pressed
////									       uchPairHeaterFlag = 0;
////										  break;} // end of if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up or down is pressed
//
//							   } // end of while(1)
//                        	 }// end of if(uchPairHeaterFlag = !uchPairHeaterFlag)
//							// Testing End

                            update_display = true;
                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                } else if ((!((*btn >> BUTTON_DOWN_STAT) & 0x01)) && (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01))) { // both down and timer buttons are pressed
                    if (((cur_ms - btn_down_press_ms) >= DEBUG_MODE_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_timer_press_ms) >= DEBUG_MODE_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            *mode = APP_MODE_DEBUG;
                        }
                        btn_down_press_ms = cur_ms;
                        btn_timer_press_ms = cur_ms;
                    }
                }

                // Testing on For displaying SSID short cut   // Working one verification pending..
// #define SHORT_CUT_DISPLAYING_SSID
#ifdef SHORT_CUT_DISPLAYING_SSID
                 else if ((!((*btn >> BUTTON_UP_STAT) & 0x01))&& (((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)))
                { // only up is pressed
                	if ((cur_ms - btn_up_press_ms) >= 2000) {
                		 if (!data->is_child_lock_active)
                		  {
                			display_clear_screen();
                		   display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing
                		   printf("Display SSID on short cut ");

                		 vTaskDelay(4000);}
                		 update_display = 1;
                         btn_up_press_ms = cur_ms;
                	}
                   }
#endif
                 //
                // Testing end

                else if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // only button down is pressed
		    if ((cur_ms - btn_down_press_ms) >= MENU_MODE_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MENU;
                            data->menu_mode_exit_mode = APP_MODE_STANDBY;
                        }
                        btn_down_press_ms = cur_ms;
		          }
                }
                else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01))) { // both up and timer buttons are pressed
                    if (((cur_ms - btn_up_press_ms) >= TEMP_OFFSET_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_timer_press_ms) >= TEMP_OFFSET_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                              *mode = APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET;  // Original Line..
                        }
                        btn_up_press_ms = cur_ms;
                        btn_timer_press_ms = cur_ms;
                    }
                }
            }
	} else { // any of the buttons was pressed
	    if (data->display_settings.is_auto_screen_off_en) {
		t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
	    }

	    if (screen_off) {
		// enable screen only if all buttons are unpressed
		if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
		    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
		    screen_off = false;
		    display_on();
		}
	    }

	    else
	    {
		      if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
		      if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
		      }
		      } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if (!((*btn >> BUTTON_UP_STAT) & 0x01)) { // pressed
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // pressed
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
                else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) { // pressed
                        btn_timer_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }// end of else//
            prev_btn = *btn;
        }

        // Original old Code
//        if (update_display) {
//            update_display = false;
//
//            display_clear_screen();
//            // display standby message
//            display_standby_message(DISPLAY_COLOR);
//            // display connection status indication if connected
//            if (data->is_connected)
//                display_wifi_icon(DISPLAY_COLOR);
//            // display lock icon if child lock is active
//            if (data->is_child_lock_active)
//                display_child_lock_icon(DISPLAY_COLOR);
//        }
//         vTaskDelay(1 / portTICK_RATE_MS);

        // Added for testing ..Begin

   	 // if(oneTimeRegistrationPacketToAWS == 1)
   	  if((oneTimeRegistrationPacketToAWS ==1) ||(pairON_blinkWifi ==1))
   	   {   update_display =1; // display_wifi_icon_pairing_blinking(DISPLAY_COLOR); printf(" in manual temperature  mode wifi icon blinking \n ");
   	   }


//   if(uchUpdateDoneOnce){
////   	if((oneTimeRegistrationPacketToAWS == 0)||(pairON_blinkWifi ==0))
//	   	if(oneTimeRegistrationPacketToAWS == 0)
//     	{	uchPairDoneDisplayOnceFlag++;
//
//   	   if(uchPairDoneDisplayOnceFlag == 2)
//   	   {  update_display = true;  printf("Update once \n "); uchUpdateDoneOnce = 0;}
//   	  }
//     }

   	if(PairDataRecievedFromAPP == 0){
        if (update_display) {
               update_display = false;

               display_clear_screen();

               if(pairON_blinkWifi ==1)
               {
            	   display_menu_pair_Heater("please wait..", DISPLAY_COLOR, "pairing heater", DISPLAY_COLOR);
            	 //  display_menu_pair_Heater("connecting....", DISPLAY_COLOR, "please wait !!", DISPLAY_COLOR);
               }
               else
               // display standby message
               { display_standby_message(DISPLAY_COLOR);}

               // display connection status indication if connected
//               if (data->is_connected)
//                   display_wifi_icon(DISPLAY_COLOR);

           	 //if(oneTimeRegistrationPacketToAWS == 1)
             if((oneTimeRegistrationPacketToAWS ==1)||(pairON_blinkWifi ==1))
              {
                  display_wifi_icon_pairing_blinking(DISPLAY_COLOR);
              }
              else {
              if (data->is_connected)
                  display_wifi_icon(DISPLAY_COLOR);
              }// end of else of  if(paringOnFlag == 1)


               // display lock icon if child lock is active
               if (data->is_child_lock_active)
                   display_child_lock_icon(DISPLAY_COLOR);
           }
   	   }

        if( prev_pairON_blinkWifi != pairON_blinkWifi )
        {  	 update_display = 1; prev_pairON_blinkWifi = pairON_blinkWifi;
        }

    	if (prev_oneTimeRegistrationPacketToAWS != oneTimeRegistrationPacketToAWS)
    	{update_display = 1; prev_oneTimeRegistrationPacketToAWS = oneTimeRegistrationPacketToAWS ;}


       // if(oneTimeRegistrationPacketToAWS == 1)
       if((oneTimeRegistrationPacketToAWS ==1)||(pairON_blinkWifi ==1))
        {   	vTaskDelay(1000);	}
        else
        { vTaskDelay(1 / portTICK_RATE_MS);}

         // Testing End..

    }
    // exit with display on
    if (screen_off)
       display_on();
}

static void manual_temperature_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = 0;
    app_mode_t *mode = &(data->mode);
    time_t btn_power_press_ms = 0, btn_timer_press_ms = 0, btn_up_press_ms = 0, btn_down_press_ms = 0;
    int *temp = NULL, temp_max = 0, temp_min = 0;
    int *target_temp_c = &(data->manual_temperature_celsius), prev_target_temp_c = -1;

   // New Line Added for Testing..
    int *target_temp_f = &(data->manual_temperature_fahrenheit), prev_target_temp_f = -1;

    int *ambient_temp_c = &(data->ambient_temperature_celsius), prev_ambient_temp_c = -1;
    bool is_heater_on = false;
    bool is_target_temp_changed = false;
    int *temp_hysteresis_c = &(data->settings.temperature_hysteresis_celsius);
    int *temp_hysteresis_f = &(data->settings.temperature_hysteresis_fahrenheit);
    int hysteresis_c = 0;
    temp_unit_t *temp_unit = &(data->settings.temperature_unit);

    bool prevTempUnit =0 ;

    bool update_display = true;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    bool display_target_temp = false;
    timer_t t_target_temp_changed_ms = 0;
  //  bool update_display = true;
    unsigned char uchPairDoneDisplayOnceFlag = 0;


    bool mprev_pairON_blinkWifi = pairON_blinkWifi ;
	bool mprev_oneTimeRegistrationPacketToAWS = oneTimeRegistrationPacketToAWS ;


    // clear screen
    display_clear_screen();

//    // New added
//	if(FlashEraseEnableAPMode ==1)
//	{    // FlashEraseEnableAPMode = 0;
//		paringOnFlag =1; AutoDisplayOffInParingOnFlag =1;
//		 esp32_wifi_ap_enable(uniqueDeviceID, ap_password); printf("In manual temperature loop  Enable AP Mode \n");
//	}

    // CR not yet  aprroved    // cr not appproved , so comment this ..after testing..
	 if((FlashEraseEnableAPMode ==1)||(pairON_blinkWifi ==1))
	// if(FlashEraseEnableAPMode ==1)
	{  data->display_settings.auto_screen_off_delay_sec = 180;
	  // data->display_settings.is_auto_screen_off_en = false;
	}

    // wait until timer button is released so that it will prevent entering AUTO mode if this mode is entered from Timer mode
    while (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) vTaskDelay(1 / portTICK_RATE_MS);
    // this is necessary to disregard button toggle while waiting for timer button released
    prev_btn = *btn;

#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    // set maximum and minimum temperatures and temp pointer based on unit
    if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
        temp_max = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX;
        temp_min = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN;
        temp = &(data->manual_temperature_celsius);
    } else {
        temp_max = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX;
        temp_min = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN;  //
        temp = &(data->manual_temperature_fahrenheit);
    }
#else
    // set maximum and minimum temperatures and temp pointer based on unit   // Original Last Firmware release logic
       if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
           temp_max = TEMPERATURE_CELSIUS_VAL_MAX;
           temp_min = TEMPERATURE_CELSIUS_VAL_MIN;
           temp = &(data->manual_temperature_celsius);
       } else {
           temp_max = TEMPERATURE_FAHRENHEIT_VAL_MAX;
           temp_min = TEMPERATURE_FAHRENHEIT_VAL_MIN;
           temp = &(data->manual_temperature_fahrenheit);
       }

#endif


       prevTempUnit = data->settings.temperature_unit ;
    while(*mode == APP_MODE_MANUAL_TEMPERATURE) {
       // Added for testing ctof
    	// data->settings.temperature_unit =1;
    	// data->settings.temperature_unit =0;

//    	if(paringOnFlag == 1){
//    	  // testing for three min interval for pairing..
//		  int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
//			if ((cur_ms - timePairingDur_ms) >= 60000) {
//				timePairingDur_ms= cur_ms;
//				pairtime_OneMinuteOver = 1;  printf("one min over.over in manual Temperature mode. \n");
//			  }
//
//			if(pairtime_OneMinuteOver == 1)
//			{
//			   pairtime_OneMinuteOver = 0;
//			   pair_TenMin_count++;
//			}
//
//			// Working for 3 minutes..
////			if(pair_TenMin_count > 3)
////			{pair_TenMin_count=0;pairtime_OneMinuteOver = 0; paringOnFlag =0;
////			printf("Three minutes over in manual Temperature mode \n ");
////			esp32_wifi_client_enable(username,password);   printf("In Three min over.over in manual Temperature mode   Disable  AP Mode \n");
////			}// end of if(pair_TenMin_count == 3)
//
//// Testing for 10 minutes. AP mode enable...
//            // Auto Screen Off after  minutes..
//			if(pair_TenMin_count > 5)
//			{  AutoDisplayOffInParingOnFlag = 0; }
//
//			// Enable the AP mode for 10 minutes..
//			if(pair_TenMin_count > 10)
//			{pair_TenMin_count=0;pairtime_OneMinuteOver = 0; paringOnFlag =0;
//			printf("Three minutes over in manual Temperature mode \n ");
//			esp32_wifi_client_enable(username,password);   printf("In Three min over.over in manual Temperature mode   Disable  AP Mode \n");
//			}// end of if(pair_TenMin_count == 3)
//
//
//    	} // end of pairingOnFlag

//    	if(FlashEraseEnableAPMode ==1)
//    	{    // FlashEraseEnableAPMode = 0;
//    		 // tcpServerTask= 1;
//    		 esp32_wifi_ap_enable(uniqueDeviceID, ap_password); printf("In manual temperature loop  Enable AP Mode \n");
//    	}


#ifdef TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
     menuModeKeypressedFlag =0;
#endif

    	if(prevTempUnit != data->settings.temperature_unit){
    	   prevTempUnit = data->settings.temperature_unit ;update_display = true;  }

    	   if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
    	        temp_max = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX;
    	        temp_min = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN;
    	        temp = &(data->manual_temperature_celsius);
    	    } else {
    	        temp_max = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX;
    	        temp_min = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN;  //
    	        temp = &(data->manual_temperature_fahrenheit);
    	    }


    	// turn off/on the heater based on temperature
        if (*ambient_temp_c < data->manual_temperature_celsius) {
            if (!is_heater_on) {

            	// if(heater_On_Off_state_by_command == 1){
            	heater_on();
                is_heater_on = true;

                #ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
                //    app_data->lastHeaterState = true;
                //    set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
               #endif
            	// }
               // printf("MANUAL: heater on ambient=%d taget=%d\r\n", *ambient_temp_c, data->manual_temperature_celsius);
            }
        } else {
            if (*temp_unit == TEMP_UNIT_CELSIUS)
                hysteresis_c = *temp_hysteresis_c;
            else
            { // hysteresis_c = fahr_to_celsius(*temp_hysteresis_f);  // original
               hysteresis_c  = valueRoundOff(*temp_hysteresis_f, CONVERT_F_TO_C);
            }

            if (*ambient_temp_c >= (data->manual_temperature_celsius + hysteresis_c)) {
                if (is_heater_on) {

                  // if(heater_On_Off_state_by_command == 1){
                	heater_off();
                    is_heater_on = false;

					#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
                    //	app_data->lastHeaterState = false;
                    //	 set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
                    //	printf("app_data->lastHeaterState %d \n",app_data->lastHeaterState);
					#endif
                // } // end of if(heater_On_Off_state_by_command == 1)
                  //  printf("MANUAL heater off ambient=%d taget=%d\r\n", *ambient_temp_c, data->manual_temperature_celsius);
                }
            }
        }

        if (*ambient_temp_c != prev_ambient_temp_c) {
            prev_ambient_temp_c = *ambient_temp_c;
            update_display = true;
        }

      //  printf(" Testing data->display_settings.is_auto_screen_off_en %d",data->display_settings.is_auto_screen_off_en);

        /* button
           - single press BACK button to go back to standby mode
           - single press UP or DOWN button to increase or decrease temperature
           - long press POWER button to go to standby mode
           - long press UP AND DOWN buttons to activate/deactivate child lock
           - long press DOWN button to go to MENU mode
           - long press TIMER button to enter AUTO mode
        */

       // printf("data->display_settings.auto_screen_off_delay_sec  %d\n ",data->display_settings.auto_screen_off_delay_sec );

        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {

            	// original
//            	if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
//                    display_off();
//                    screen_off = true;
//                }
                // Testing..
            	if(AutoDisplayOffInParingOnFlag == 0){
            	if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
            		display_off();
                    screen_off = true;
                  } // end of
                }// end of if(paringOnFlag == 0){
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        btn_power_press_ms = cur_ms;
                    }
                } else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up AND down are pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            data->is_child_lock_active = !data->is_child_lock_active;

                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS

                            update_display = true;
                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);

                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                } else if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // only button down is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_down_press_ms) >= MENU_MODE_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MENU;
                            data->menu_mode_exit_mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
                        btn_down_press_ms = cur_ms;
                    }
                } else if (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) { // pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_timer_press_ms) >= AUTO_MODE_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_AUTO;
                        }
                        btn_timer_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_STANDBY;
                            // new added for stand by mode by manually _30NOv2020
                            heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0;
                        }
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            if ((*temp + 1) <= temp_max) {
                                is_target_temp_changed = true;
                                *temp += 1;
                                manaully_Set_Temp_change = 1; // New Added for manual Set Temp change Notification
                            }
                        }
                    } else {
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            if ((*temp - 1) >= temp_min) {
                                is_target_temp_changed = true;
                                *temp -= 1;
                                manaully_Set_Temp_change = 1; // New Added for manual Set Temp change Notification
                            }
                        }
                    } else {
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_TIMER_INCREMENT;
                        }
                    } else {
                        btn_timer_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        if (is_target_temp_changed) {
            is_target_temp_changed = false;

            printf("\n new target temp=%d\r\n", *temp);

            // update the temperature of the other unit
            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
               // data->manual_temperature_fahrenheit = celsius_to_fahr(*temp);  // Orig
                data->manual_temperature_fahrenheit  = valueRoundOff(*temp, CONVERT_C_TO_F);
                printf("data->manual_temperature_fahrenheit  %d \n",data->manual_temperature_fahrenheit );

            } else {
               // data->manual_temperature_celsius = fahr_to_celsius(*temp);  // Orig
                data->manual_temperature_celsius  = valueRoundOff(*temp, CONVERT_F_TO_C);
                printf("data->manual_temperature_celsius  %d \n",data->manual_temperature_celsius );
            }

            // display target temperature
            t_target_temp_changed_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            display_target_temp = true;
            update_display = true;

            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, data->manual_temperature_fahrenheit);
        } else {
            if (display_target_temp) {
                if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_target_temp_changed_ms >= DISPLAY_TARGET_TEMP_DUR_MS) {
                    display_target_temp = false;
                    update_display = true;
                } else {
                    update_display = false;
                }
            }
        }

        // Original Lines..
//        // if target temp is remotely changed, update the display
//        if (*target_temp_c != prev_target_temp_c) {
//            prev_target_temp_c = *target_temp_c;
//            update_display = true;
//        }

        // Modified forTesting New Added for fahreneite change 26Dec2020
			if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
			{if (*target_temp_c != prev_target_temp_c) {
				prev_target_temp_c = *target_temp_c;
				update_display = true;
			}}
			else{if (*target_temp_f != prev_target_temp_f) {
				prev_target_temp_f = *target_temp_f;
				update_display = true;
			 } }//end of else


	 //  if(paringOnFlag == 1){
	 // if(oneTimeRegistrationPacketToAWS == 1)


//	  if(uchUpdateDoneOnce){
////	   	if((oneTimeRegistrationPacketToAWS == 0)||(pairON_blinkWifi ==0))
//	   	if(oneTimeRegistrationPacketToAWS == 0)
//		  {	uchPairDoneDisplayOnceFlag++;
//
//	   	   if(uchPairDoneDisplayOnceFlag == 2)
//	   	   {  update_display = true;  printf("Update once \n "); uchUpdateDoneOnce = 0;}
//	   	  }
//	     }

	        if( mprev_pairON_blinkWifi != pairON_blinkWifi )
	        {	update_display = 1; mprev_pairON_blinkWifi = pairON_blinkWifi;}

	    	if (mprev_oneTimeRegistrationPacketToAWS != oneTimeRegistrationPacketToAWS)
	    	{update_display = 1; mprev_oneTimeRegistrationPacketToAWS = oneTimeRegistrationPacketToAWS ;}


		if((oneTimeRegistrationPacketToAWS == 1) ||(pairON_blinkWifi ==1))
		   { update_display =1; // display_wifi_icon_pairing_blinking(DISPLAY_COLOR); printf(" in manual temperature  mode wifi icon blinking \n ");
		   }

		if(PairDataRecievedFromAPP == 0){
        if (update_display) {
            update_display = false;
            display_clear_screen();

            if(pairON_blinkWifi ==1)
            {
            	display_menu_pair_Heater("please wait..", DISPLAY_COLOR, "pairing heater", DISPLAY_COLOR);
            	// display_menu_pair_Heater("Please wait....", DISPLAY_COLOR, "connecting heater !!", DISPLAY_COLOR);
            //	display_menu_pair_Heater("connecting....", DISPLAY_COLOR, "please wait !!", DISPLAY_COLOR);
            }
            else{
					if (display_target_temp) {
						display_temperature(*temp, DISPLAY_COLOR);
					} else {

						// Original
					   // display_manual_temperature_normal(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : celsius_to_fahr(*ambient_temp_c), *temp, DISPLAY_COLOR);
		//				float lf_temp =0;         // Testing Line Begin_TEST
		//				float lf_temp_roundOff =0;
		//				int ambient_temp_f = 0;
		//				lf_temp = celsius_to_fahr(*ambient_temp_c);
		//				lf_temp_roundOff = round(lf_temp);
		//				ambient_temp_f = lf_temp_roundOff;
						int ambient_temp_f = 0;
						ambient_temp_f  = valueRoundOff(*ambient_temp_c, CONVERT_C_TO_F);
						display_manual_temperature_normal(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : ambient_temp_f, *temp, DISPLAY_COLOR);
					}
            }//end of else of  if(pairON_blinkWifi ==1)


            // display connected status icon if connected
            // Original ...
//            if (data->is_connected)
//                display_wifi_icon(DISPLAY_COLOR);

        	if((oneTimeRegistrationPacketToAWS == 1) ||(pairON_blinkWifi ==1))
           	 {
                display_wifi_icon_pairing_blinking(DISPLAY_COLOR);
            }
            else {
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);
            }// end of else of  if(paringOnFlag == 1)

            // display lock icon if child lock is active
            if (data->is_child_lock_active)
                display_child_lock_icon(DISPLAY_COLOR);
        }// end of if diplay update
		} // end if(PairDataRecievedFromAPP == 0){


        // Original ..
       // vTaskDelay(1 / portTICK_RATE_MS);

        // testing ..
      // if(oneTimeRegistrationPacketToAWS == 1)
      if((oneTimeRegistrationPacketToAWS == 1) ||(pairON_blinkWifi ==1))
        {
        	vTaskDelay(1000);	}
        else
        { vTaskDelay(1 / portTICK_RATE_MS);}

    }
    // exit with display on
    if (screen_off)
       display_on();
}


static void temperature_offset_set_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = 0;
    app_mode_t *mode = &(data->mode);
    time_t btn_power_press_ms = 0, btn_up_press_ms = 0, btn_down_press_ms = 0;
    int *ambient_temp_offset_c = &(data->ambient_temperature_offset_celsius);
    int *ambient_temp_c = &(data->ambient_temperature_celsius), prev_ambient_temp_c = -1;
    bool is_temp_offset_changed = false;

    bool update_display = true;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // clear screen
    display_clear_screen();

    // wait until up and timer buttons are released
    while ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)))
        vTaskDelay(1 / portTICK_RATE_MS);

    // this is necessary to disregard button toggle while waiting for up and timer buttons are released
    prev_btn = *btn;

    while(*mode == APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET) {
        if (*ambient_temp_c != prev_ambient_temp_c) {
            prev_ambient_temp_c = *ambient_temp_c;
            update_display = true;
        }

        /* button
           - single press BACK button to go back to standby mode
           - single press UP or DOWN button to increase or decrease temperature offset
           - long press POWER button to go to standby mode
           - long press UP and DOWN buttons to activate/deactivate child lock
         */
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        btn_power_press_ms = cur_ms;
                    }
                } else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up AND down are pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            data->is_child_lock_active = !data->is_child_lock_active;

                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS

                            update_display = true;
                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);

                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_STANDBY;
                            // new added for stand by mode by manually _30Nov2020
                           heater_On_Off_state_by_command = 0; 	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        }
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            if ((*ambient_temp_offset_c + 1) <= TEMPERATURE_SENSOR_OFFSET_CELSIUS_MAX) {
                                is_temp_offset_changed = true;
                                *ambient_temp_offset_c += 1;
                            }
                        }
                    } else {
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                       if (!data->is_child_lock_active) {
                            if ((*ambient_temp_offset_c - 1) >= TEMPERATURE_SENSOR_OFFSET_CELSIUS_MIN) {
                                is_temp_offset_changed = true;
                                *ambient_temp_offset_c -= 1;
                            }
                        }
                    } else {
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        if (is_temp_offset_changed) {
            is_temp_offset_changed = false;
            // save new offset
            set_integer_to_storage(STORAGE_KEY_TEMP_SENSOR_OFFSET_CELSIUS, *ambient_temp_offset_c);
            // update display
            update_display = true;
        }

        if (update_display) {
            update_display = false;
            display_clear_screen();

            display_temperature_offset_set_mode(*ambient_temp_c, *ambient_temp_offset_c, DISPLAY_COLOR);

            // display connected status icon if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            // display lock icon if child lock is active
            if (data->is_child_lock_active)
                display_child_lock_icon(DISPLAY_COLOR);
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();
}

static void debug_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = 0;
    app_mode_t *mode = &(data->mode);
    time_t btn_power_press_ms = 0, btn_up_press_ms = 0, btn_down_press_ms = 0;

    int *temp_c = &(data->ambient_temperature_celsius), prev_temp_c = -1;
    int *temp_offset_c = &(data->ambient_temperature_offset_celsius), prev_temp_offset_c = -1;
    int *ambient_light = &(data->ambient_light), prev_ambient_light = -1;
    int *screen_br = &(data->display_settings.display_brightness), prev_screen_br = -1;
    int *pilot_light_br = &(data->pilot_light_brightness), prev_pilot_light_br = -1;

    bool *conn_stat = &(data->is_connected), prev_conn_stat = -1;
    bool *internet_conn = &(data->internet_conn), prev_internet_conn = -1;
    //TODO: bool kaa_status = is_kaa_connected();
    //TODO: bool prev_kaa_status = ~kaa_status;
    bool ap_en = -1, prev_ap_en = -1;

  //  char *ap_ssid = comm_wifi_dev->wifi_ap_ssid;   // Commented for testing as it is unused variables, was giving warning

    bool update_display = true;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    int screen_num = 1;
    // start ping task
    xTaskCreate(ping_task, "ping_task", 4096, (void *)data, 12, NULL);
    // clear screen
    display_clear_screen();
    // wait until up and timer buttons are released
    while ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)))
        vTaskDelay(1 / portTICK_RATE_MS);

    // this is necessary to disregard button toggle while waiting for up and timer buttons are released
    prev_btn = *btn;

    while(*mode == APP_MODE_DEBUG) {
        /* button
           - single press BACK button to go back to standby mode
           - long press POWER button to go to standby mode
           - long press UP and DOWN buttons to activate/deactivate child lock
         */
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        btn_power_press_ms = cur_ms;
                    }
                } else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up AND down are pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            data->is_child_lock_active = !data->is_child_lock_active;

                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS

                            update_display = true;
                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);

                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_STANDBY;
                            // new added for stand by mode by manually _30Nov2020
                           heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        }
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if (screen_num < DEBUG_SCREEN_NUM_MAX) {
                            ++screen_num;
                            update_display = true;
                        }
                    } else {
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        if (screen_num > 1) {
                            --screen_num;
                            update_display = true;
                        }
                    } else {
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        if (screen_num == 1) {
            if (*temp_c != prev_temp_c) {
                prev_temp_c = *temp_c;
                update_display = true;
            }
            if (*temp_offset_c != prev_temp_offset_c) {
                prev_temp_offset_c = *temp_offset_c;
                update_display = true;
            }
            if (*ambient_light != prev_ambient_light) {
                prev_ambient_light = *ambient_light;
                update_display = true;
            }
            if (*screen_br != prev_screen_br) {
                prev_screen_br = *screen_br;
                update_display = true;
            }
            if (*pilot_light_br != prev_pilot_light_br) {
                prev_pilot_light_br = *pilot_light_br;
                update_display = true;
            }
        } else if (screen_num == 2) {
            if (*conn_stat != prev_conn_stat) {
                prev_conn_stat = *conn_stat;
                update_display = true;
            }
            if (*internet_conn != prev_internet_conn) {
                prev_internet_conn = *internet_conn;
                update_display = true;
            }
#if 0 //TODO:             
            kaa_status = is_kaa_connected();
            if (kaa_status != prev_kaa_status) {
                prev_kaa_status = kaa_status;
                update_display = true;
            }
#endif   
            ap_en = comm_wifi_dev->is_wifi_ap_enabled();
            if (ap_en != prev_ap_en) {
                prev_ap_en = ap_en;
                update_display = true;
            }
        }

        if (update_display) {
            update_display = false;
            display_clear_screen();

            if (screen_num == 1)
                display_debug_screen_1(*temp_c, *temp_offset_c, *ambient_light, *screen_br, *pilot_light_br, DISPLAY_COLOR);
            //TODO: else if (screen_num == 2)
                //display_debug_screen_2(*conn_stat, *internet_conn, kaa_status, ap_en, ap_ssid, DISPLAY_COLOR);

            // display lock icon if child lock is active
            if (data->is_child_lock_active)
                display_child_lock_icon(DISPLAY_COLOR);
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }
    // exit with display on
    if (screen_off)
       display_on();
}

static void timer_increment_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;
    app_mode_t *mode = &(data->mode);
    time_t btn_power_press_ms = 0, btn_up_press_ms = 0, btn_down_press_ms = 0, btn_timer_press_ms = 0;
    int *timer_min = &(data->current_timer_setting_min), prev_timer_min = -1;
    time_t t0_ms = 0, t1_ms = 0;
    int *target_temp_c = &(data->manual_temperature_celsius), *target_temp_f = &(data->manual_temperature_fahrenheit);

    int prev_target_temp_c = -1;
    int prev_target_temp_f = -1;   // New Added For Fahreneite.._26Dec2020

    int *ambient_temp_c = &(data->ambient_temperature_celsius), prev_ambient_temp_c = -1;
    bool is_heater_on = false;
    int *temp_hysteresis_c = &(data->settings.temperature_hysteresis_celsius);
    int *temp_hysteresis_f = &(data->settings.temperature_hysteresis_fahrenheit);
    int hysteresis_c = 0;
    temp_unit_t *temp_unit = &(data->settings.temperature_unit);
    int *temp = NULL, temp_max = 0, temp_min = 0;
    bool is_target_temp_changed = false;
    timer_t t_target_temp_changed_ms = 0;
    bool display_target_temp = false;

    bool update_display = true;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool is_timer_change_en = true; // whether timer can be increased or decreased
    bool is_timer_changed = true; // whether timer is increased or decreased
    time_t t_timer_changed_ms = 0;
    bool display_timer = false;

    time_t t_timer_expire_ms = 0;
    bool is_timer_expired = false;

    // set inital timer value
    *timer_min = data->last_timer_setting_min;
    t0_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    // set maximum and minimum temperatures and temp pointer based on unit
    if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
        temp_max = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX;
        temp_min = TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN;
        temp = &(data->manual_temperature_celsius);
    } else {
        temp_max = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX;
        temp_min = TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN;  //
        temp = &(data->manual_temperature_fahrenheit);
    }
#else
    // set maximum and minimum temperatures and temp pointer based on unit   // Original Last Firmware release logic
       if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
           temp_max = TEMPERATURE_CELSIUS_VAL_MAX;
           temp_min = TEMPERATURE_CELSIUS_VAL_MIN;
           temp = &(data->manual_temperature_celsius);
       } else {
           temp_max = TEMPERATURE_FAHRENHEIT_VAL_MAX;
           temp_min = TEMPERATURE_FAHRENHEIT_VAL_MIN;
           temp = &(data->manual_temperature_fahrenheit);
       }

#endif

    while(*mode == APP_MODE_TIMER_INCREMENT) {
        if (*timer_min == 0)
        {
				if (is_timer_expired) {
					// go to standby only if there is no timer increment within 5 seconds
					if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_timer_expire_ms) >= (TIMER_EXPIRE_WAIT_FOR_INCREMENT_MS)) {
						*mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
					}
				} else {
					is_timer_expired = true;

					// set time when the timer expired
					t_timer_expire_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
				}


				// turn off heater
				heater_off();
				is_heater_on = false;

				#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
				//	 app_data->lastHeaterState = false;
				//	 set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
				//	 printf("app_data->lastHeaterState %d \n",app_data->lastHeaterState);
				#endif

        }
        else
        {
            is_timer_expired = false;

            // update timer based on lapsed time
            t1_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((t1_ms - t0_ms) >= 60000) { // 1 minute has passed
                *timer_min -= 1;
                t0_ms = t1_ms;
            }

            // turn off/on the heater based on temperature
            if (*ambient_temp_c < *target_temp_c) {
                if (!is_heater_on) {

                	// if(heater_On_Off_state_by_command == 1){

                	heater_on();
                    is_heater_on = true;

					#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
					//	 app_data->lastHeaterState = true;
					//	 set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
					#endif
                	// }
                  //  printf("TIMER: heater on ambient=%d taget=%d\r\n", *ambient_temp_c, *target_temp_c);
                }
              }
             else
              {
					if (*temp_unit == TEMP_UNIT_CELSIUS)
						hysteresis_c = *temp_hysteresis_c;
					else
					{	// hysteresis_c = fahr_to_celsius(*temp_hysteresis_f);  // Original Line

//						float lf_temp =0;         // Testing Line Begin_TEST
//						float lf_temp_roundOff =0;
//						int l_hysteresis_c = 0;
//						lf_temp = fahr_to_celsius(*temp_hysteresis_f);
//						lf_temp_roundOff = round(lf_temp);
//						l_hysteresis_c = lf_temp_roundOff;
//						hysteresis_c = l_hysteresis_c;  // END

						hysteresis_c  = valueRoundOff(*temp_hysteresis_f, CONVERT_F_TO_C);
					}

					if (*ambient_temp_c >= (*target_temp_c + hysteresis_c)) {
						if (is_heater_on) {

							// if(heater_On_Off_state_by_command == 1){

							heater_off();
							is_heater_on = false;

							#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
								// app_data->lastHeaterState = false;
								// set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
								//  printf("app_data->lastHeaterState %d \n",app_data->lastHeaterState);
							#endif
							// }
							// printf("TIMER: heater off ambient=%d taget=%d\r\n", *ambient_temp_c, *target_temp_c);
						}
					}// end of if (*ambient_temp_c >= (*target_temp_c + hysteresis_c))
              } //end of else
        }// end of else

        /* button
           - single press TIMER button to enable increasing or increasing timer value
           - single press UP/DOWN button to increase/decrease target temperature if changing timer value is not enabled
           - single press UP/DOWN button to increase/decrease timer setting if changing timer setting is enabled
           - long press POWER button to go to standby mode
           - long press DOWN button to go to MENU mode
           - long press TIMER button to reset timer and go to manual temperature mode
        */
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        btn_power_press_ms = cur_ms;
                    }
                } else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up AND down are pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            data->is_child_lock_active = !data->is_child_lock_active;

                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS

                            update_display = true;
                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);

                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                } else if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // only button down is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_down_press_ms) >= MENU_MODE_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MENU;
                            data->menu_mode_exit_mode = APP_MODE_TIMER_INCREMENT;
                        }
                        btn_down_press_ms = cur_ms;
                    }
                } else if (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) { // pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_timer_press_ms) >= TIMER_MODE_RESET_TIMER_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *timer_min = 0;
                            *mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
                        btn_timer_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // pressed
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            if (is_timer_change_en) {
                                is_timer_changed = true;
                                int temp_timer_min = (*timer_min / TIMER_INCREMENT_MINUTES) * TIMER_INCREMENT_MINUTES + TIMER_INCREMENT_MINUTES;
                                if (temp_timer_min < TIMER_MAX_VALUE_MINUTES)
                                    *timer_min = temp_timer_min;
                                else
                                    *timer_min = TIMER_MAX_VALUE_MINUTES;
                            } else {
                                if ((*temp + 1) <= temp_max) {
                                    is_target_temp_changed = true;
                                    *temp += 1;
                                    manaully_Set_Temp_change = 1; // New Added for manual Set Temp change Notification

                                }
                            }
                        }
                    } else { // pressed
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            if (is_timer_change_en) {
                                if (*timer_min > TIMER_MIN_VALUE_MINUTES) {
                                    is_timer_changed = true;
                                    int temp_timer_min = (*timer_min % TIMER_INCREMENT_MINUTES  == 0) ? *timer_min - TIMER_INCREMENT_MINUTES
                                        : (*timer_min / TIMER_INCREMENT_MINUTES) * TIMER_INCREMENT_MINUTES;
                                    if (temp_timer_min > TIMER_MIN_VALUE_MINUTES)
                                        *timer_min = temp_timer_min;
                                    else
                                        *timer_min = TIMER_MIN_VALUE_MINUTES;
                                }
                            } else {
                                if ((*temp - 1) >= temp_min) {
                                    is_target_temp_changed = true;
                                    *temp -= 1;
                                    manaully_Set_Temp_change = 1; // New Added for manual Set Temp change Notification
                                }
                            }
                        }
                    } else { // pressed
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            is_timer_change_en = true;
                            is_timer_changed = true; // to update display only
                        }
                    } else { // pressed
                        btn_timer_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update display if ambient temperature changed
        // Original
        if (*ambient_temp_c != prev_ambient_temp_c) {
            prev_ambient_temp_c = *ambient_temp_c;
            update_display = true;
        }

        // update display if timer value changed
        if (*timer_min != prev_timer_min) {
            printf("timer_min=%d\r\n", *timer_min);
            prev_timer_min = *timer_min;
            update_display = true;
        }

        // normal timer mode display or timer changed display
        if (is_timer_changed) {
            is_timer_changed = false;

            printf("timer=%d\r\n", *timer_min);

            t_timer_changed_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            display_timer = true;
            update_display = true;

            // update saved timer in flash
            data->last_timer_setting_min = *timer_min;
            set_integer_to_storage(STORAGE_KEY_LAST_TIMER_SETTING, data->last_timer_setting_min);
        } else {
            if (display_timer) {
                if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_timer_changed_ms >= DISPLAY_TIMER_DUR_MS) {
                    display_timer = false;
                    update_display = true;
                    is_timer_change_en = false;
                } else {
                    update_display = false;
                }
            }
        }

        if (is_target_temp_changed) {
            is_target_temp_changed = false;

            printf("target temp=%d\r\n", *temp);

            // update the temperature of the other unit  // Original Lines
//            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
//                data->manual_temperature_fahrenheit = celsius_to_fahr(*temp);
//            } else {
//                data->manual_temperature_celsius = fahr_to_celsius(*temp);
//            }

            // New Added on 28Dec2020 for Sych purpose // Begin
            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {

//    			float lf_temp =0;         // Testing Line Begin_TEST
//    			float lf_temp_roundOff =0;
//    			int l_n_temp_f = 0;
//    			lf_temp = celsius_to_fahr(*temp);
//    			lf_temp_roundOff = round(lf_temp);
//    			l_n_temp_f = lf_temp_roundOff;
//                data->manual_temperature_fahrenheit = l_n_temp_f;

                data->manual_temperature_fahrenheit   = valueRoundOff(*temp, CONVERT_C_TO_F);

            } else {

//    			float lf_temp =0;         // Testing Line Begin_TEST
//    			float lf_temp_roundOff =0;
//    			int l_n_temp_c = 0;
//    			lf_temp = fahr_to_celsius(*temp);
//    			lf_temp_roundOff = round(lf_temp);
//    			l_n_temp_c = lf_temp_roundOff;
//                data->manual_temperature_celsius = l_n_temp_c;

                data->manual_temperature_celsius   = valueRoundOff(*temp, CONVERT_F_TO_C);

            }// END


            // display target temperature
            t_target_temp_changed_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            display_target_temp = true;
            update_display = true;

            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, data->manual_temperature_fahrenheit);
        } else {
            if (display_target_temp) {
                if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_target_temp_changed_ms >= DISPLAY_TARGET_TEMP_DUR_MS) {
                    display_target_temp = false;
                    update_display = true;
                } else {
                    update_display = false;
                }
            }
        }
// Original
        // if target temp is remotely changed, update the display
//        if (*target_temp_c != prev_target_temp_c) {
//            prev_target_temp_c = *target_temp_c;
//            if (display_timer == false) {
//                update_display = true;
//            }
//        }

	// New Added for fahrentneite _26Dec2020
		if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
		{if (*target_temp_c != prev_target_temp_c) {
			prev_target_temp_c = *target_temp_c;
			if (display_timer == false) {
				update_display = true;
			}
		}}
		else{if (*target_temp_f != prev_target_temp_f) {
			prev_target_temp_f = *target_temp_f;
            if (display_timer == false) {
                update_display = true;
            }
		 } }//end of else


        // update display
        if (update_display) {
            update_display = false;

            display_clear_screen();

            if (display_timer) {
                display_timer_mode_changed(*timer_min, DISPLAY_COLOR);
            } else if (display_target_temp) {
                display_temperature(*temp, DISPLAY_COLOR);
            } else {

            	// Original Line
                // display_timer_mode_normal(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : celsius_to_fahr(*ambient_temp_c), data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *target_temp_c : *target_temp_f, *timer_min, DISPLAY_COLOR);

//    			float lf_temp =0;         // Testing Line Begin_TEST
//    			float lf_temp_roundOff =0;
//    			int ambient_temp_f = 0;
//    			lf_temp = celsius_to_fahr(*ambient_temp_c);
//    			lf_temp_roundOff = round(lf_temp);
//    			ambient_temp_f = lf_temp_roundOff;

    			int ambient_temp_f = 0;
    			ambient_temp_f  = valueRoundOff(*ambient_temp_c, CONVERT_C_TO_F);

                display_timer_mode_normal(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : ambient_temp_f, data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *target_temp_c : *target_temp_f, *timer_min, DISPLAY_COLOR);
            }

            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            // display lock icon if child lock is active
            if (data->is_child_lock_active)
                display_child_lock_icon(DISPLAY_COLOR);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    // exit with display on
    if (screen_off)
       display_on();
}


static void auto_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;
    app_mode_t *mode = &(data->mode);
    time_t btn_power_press_ms = 0, btn_up_press_ms = 0, btn_down_press_ms = 0;
    bool is_heater_on = false;
    int *ambient_temp_c = &(data->ambient_temperature_celsius), prev_ambient_temp_c = -1;
    int *temp_hysteresis_c = &(data->settings.temperature_hysteresis_celsius);
    int *temp_hysteresis_f = &(data->settings.temperature_hysteresis_fahrenheit);
    int hysteresis_c = 0;
    temp_unit_t *temp_unit = &(data->settings.temperature_unit);

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool update_display = true;

    int auto_temp_c = -1, auto_temp_f = -1, auto_temp_c_prev= -1;
    int hr = -1, min = -1;
    int t_sched_minute = -1, t_now_minute = -1; // time in minute; hour + minute
    int wday = -1;
    int i = -1, j = -1;

    while(*mode == APP_MODE_AUTO) {
        int sched_past_idx = -1;
        int sched_next_idx = -1;

        wday = clock_get_day_of_week();

        auto_mode_sched_t *sched_week[7 * AUTO_MODE_SCHED_NUM];
        for (i = 0; i < 7 * AUTO_MODE_SCHED_NUM; i++) {
            if ((i == 0) || (i == 1) || (i == 2) || (i == 3) // Sunday
                || (i == 24) || (i == 25) || (i == 26) || (i == 27)) { // Saturday
                sched_week[i] = &sched_weekend[i % AUTO_MODE_SCHED_NUM];
            } else { // weekday
                sched_week[i] = &sched_weekday[i % AUTO_MODE_SCHED_NUM];
            }
        }

        // get time and convert it to minutes
        clock_get_time(&hr, &min, NULL);
//        printf("AUTO: hr=%d min=%d\r\n", hr, min);
        t_now_minute = hr * 60 + min;

        // get past sched
        sched_past_idx = -1;
        // check for the nearest schedule before t_now
        // start with the last sched of the day
        j = wday * AUTO_MODE_SCHED_NUM + AUTO_MODE_SCHED_NUM - 1;
        for (i = 0; i < AUTO_MODE_SCHED_NUM; i++, j--) {
            if (sched_week[j]->en) {
                t_sched_minute = sched_week[j]->hour * 60 + sched_week[j]->minute;
                if ((t_sched_minute == t_now_minute)
                    || (t_sched_minute < t_now_minute)) {
                    sched_past_idx = j;
                    break;
                }
            }
        }

        // if past sched is not found on the current day, search for the first enabled sched from the previous days
        if (sched_past_idx == -1) {
            for (i = 1; i < 7 * AUTO_MODE_SCHED_NUM; i++, j--) {
                if (j < 0) {
                   j = 7 * AUTO_MODE_SCHED_NUM - 1;
                }

                if (sched_week[j]->en) {
                    sched_past_idx = j;
                    break;
                }
            }
        }

//        printf("\r\nsched_past_idx %d wday %d\r\n", sched_past_idx, sched_past_idx / AUTO_MODE_SCHED_NUM);

        // set AUTO temperature
#if AUTO_MODE_INTERPOLATE == 0
        if (sched_past_idx != -1) {
            auto_temp_c = sched_week[sched_past_idx]->temp_c;
            auto_temp_f = sched_week[sched_past_idx]->temp_f;
#else
        // there should be past sched to get next sched
        if (sched_past_idx != -1) {
            // get next sched
            sched_next_idx = -1;
            // check for the nearest schedule after t_now
            // start with the first sched of the day
            j = wday * AUTO_MODE_SCHED_NUM;
            for (i = 0; i < AUTO_MODE_SCHED_NUM; i++, j++) {
                if (sched_week[j]->en) {
                    t_sched_minute = sched_week[j]->hour * 60 + sched_week[j]->minute;
                    if ((t_sched_minute == t_now_minute)
                        || (t_sched_minute > t_now_minute)) {
                        sched_next_idx = j;
                        break;
                    }
                }
            }

            // if next sched is not found on the current day, search for the first enabled sched from the next days
            if (sched_next_idx == -1) {
                for (i = 1; i < 7 * AUTO_MODE_SCHED_NUM; i++, j++) {
                    if (j == 7 * AUTO_MODE_SCHED_NUM) {
                       j = 0;
                    }

                    if (sched_week[j]->en) {
                        sched_next_idx = j;
                        break;
                    }
                }
            }
//            printf("\r\nsched_next_idx %d wday %d\r\n", sched_next_idx, sched_next_idx / AUTO_MODE_SCHED_NUM);
        }

        if ((sched_past_idx != -1) && (sched_next_idx != -1)) { // at least 1 schedule / period is set
            auto_mode_sched_t *sched_past = NULL;
            auto_mode_sched_t *sched_next = NULL;
            int wday_sched_past;
            int wday_sched_next;
            int t_past_minute;
            int temp_past_c;
            int temp_past_f;
            int t_next_minute;
            int temp_next_c;
            int temp_next_f;
            int32_t t_now_past_diff_minute;
            int32_t t_next_past_diff_minute;

            sched_past = sched_week[sched_past_idx];
            sched_next = sched_week[sched_next_idx];
            wday_sched_past = sched_past_idx / AUTO_MODE_SCHED_NUM;
            wday_sched_next = sched_next_idx / AUTO_MODE_SCHED_NUM;

            t_past_minute = sched_past->hour * 60 + sched_past->minute;
            temp_past_c = sched_past->temp_c;
            temp_past_f = sched_past->temp_f;
            t_next_minute = sched_next->hour * 60 + sched_next->minute;
            temp_next_c = sched_next->temp_c;
            temp_next_f = sched_next->temp_f;

//            printf("\r\nt_now_minute %d", t_now_minute);
//            printf("\r\ntemp_past_c %d t_past_minute %d", temp_past_c, t_past_minute);
//            printf("\r\ntemp_next_c %d t_next_minute %d", temp_next_c, t_next_minute);

            t_now_past_diff_minute = t_now_minute - t_past_minute;
            if (wday < wday_sched_past) {
                t_now_past_diff_minute += (wday + 6 - wday_sched_past) * 60 * 24;
            } else {
                t_now_past_diff_minute += (wday - wday_sched_past) * 60 * 24;
            }

            t_next_past_diff_minute = t_next_minute - t_past_minute;
            if (wday_sched_next < wday_sched_past) {
                t_next_past_diff_minute += (wday_sched_next + 6 - wday_sched_past) * 60 * 24;
            } else {
                t_next_past_diff_minute += (wday_sched_next - wday_sched_past) * 60 * 24;
            }

//            printf("\r\nt_now_past_diff_minute %d", t_now_past_diff_minute);
//            printf("\r\nt_next_past_diff_minute %d", t_next_past_diff_minute);

            if (t_next_minute == t_past_minute) {
                auto_temp_c = temp_past_c;
                auto_temp_f = temp_past_f;
            } else {
                if (t_next_past_diff_minute) {
                    auto_temp_c = temp_past_c + t_now_past_diff_minute * (temp_next_c - temp_past_c) / t_next_past_diff_minute;
                    auto_temp_f = temp_past_f + t_now_past_diff_minute * (temp_next_f - temp_past_f) / t_next_past_diff_minute;
                } else {
                    auto_temp_c = temp_past_c;
                    auto_temp_f = temp_past_f;
                }
            }
#endif
        } else { // no schedule / period is set, use set manual temperature
            auto_temp_c = data->manual_temperature_celsius;  
            auto_temp_f = data->manual_temperature_fahrenheit; 
        }

//        printf("\r\nAUTO: auto_temp_c=%d auto_temp_f=%d\r\n", auto_temp_c, auto_temp_f);

        if (auto_temp_c != auto_temp_c_prev) {
            auto_temp_c_prev = auto_temp_c;
            update_display = true;
        }

        // turn off/on the heater based on temperature
        if (*ambient_temp_c < auto_temp_c) {
            if (!is_heater_on) {

             //	if(heater_On_Off_state_by_command == 1){
                heater_on();
                is_heater_on = true;

				#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
				//	 app_data->lastHeaterState = true;
				//	 set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
				#endif
            	// }
              //  printf("AUTO: heater on ambient=%d target=%d\r\n", *ambient_temp_c, auto_temp_c);
            }
        } else {
            if (*temp_unit == TEMP_UNIT_CELSIUS)
                hysteresis_c = *temp_hysteresis_c;
            else
            {
            	// hysteresis_c = fahr_to_celsius(*temp_hysteresis_f);  // Original Line
//    			float lf_temp =0;         // Testing Line Begin_TEST
//    			float lf_temp_roundOff =0;
//    			int l_n_hysteresis_c = 0;
//    			lf_temp = fahr_to_celsius(*temp_hysteresis_f);
//    			lf_temp_roundOff = round(lf_temp);
//    			l_n_hysteresis_c = lf_temp_roundOff;
//    			hysteresis_c = l_n_hysteresis_c ;  // END

    			hysteresis_c  = valueRoundOff(*temp_hysteresis_f, CONVERT_F_TO_C);

            }

            if (*ambient_temp_c >= (auto_temp_c + hysteresis_c)) {
                if (is_heater_on) {

                	// if(heater_On_Off_state_by_command == 1){
                	heater_off();
                    is_heater_on = false;

					#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
                      //   printf("Heater Off \n ");
					//	 app_data->lastHeaterState = false;
					//	 set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
					//	 printf("app_data->lastHeaterState %d \n",app_data->lastHeaterState);
					#endif
                	// }// endof if(heater_On_Off_state_by_command == 1)
                  //  printf("AUTO: heater off ambient=%d taget=%d\r\n", *ambient_temp_c, auto_temp_c);
                }
            }
        }

        if (*ambient_temp_c != prev_ambient_temp_c) {
            prev_ambient_temp_c = *ambient_temp_c;
            update_display = true;
        }

        /* button
           - single press BACK/UP/DOWN button to go back to Manual Temperature mode
           - long press POWER button to go to standby mode
           - long press UP and DOWN buttons to activate/deactive child lock
           - long press DOWN button to go to MENU mode
        */
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        btn_power_press_ms = cur_ms;
                    }
                } else if ((!((*btn >> BUTTON_UP_STAT) & 0x01)) && (!((*btn >> BUTTON_DOWN_STAT) & 0x01))) { // both button up or down is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (((cur_ms - btn_up_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)
                        && ((cur_ms - btn_down_press_ms) >= CHILD_LOCK_LONG_PRESS_DUR_MS)) {
                        if (data->settings.is_child_lock_en) {
                            data->is_child_lock_active = !data->is_child_lock_active;

                            manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS

                            printf("is_child_lock_active=%d\r\n", data->is_child_lock_active);

                            // display / remove child lock logo
                            display_child_lock_icon(data->is_child_lock_active ? DISPLAY_COLOR : !DISPLAY_COLOR);
                            btn_up_press_ms = cur_ms;
                            btn_down_press_ms = cur_ms;
                        }
                    }
                } else if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // only button down is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_down_press_ms) >= MENU_MODE_LONG_PRESS_DUR_MS) {
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MENU;
                            data->menu_mode_exit_mode = APP_MODE_AUTO;
                        }
                        btn_down_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        if (!data->is_child_lock_active) {
                            *mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if(!data->is_child_lock_active) {
                            *mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
                    } else { // pressed
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        if(!data->is_child_lock_active) {
                            *mode = APP_MODE_MANUAL_TEMPERATURE;
                        }
                    } else { // pressed
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update display
        if (update_display) {
            update_display = false;
            display_clear_screen();


            // Original Line ..
//            display_auto_mode(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : celsius_to_fahr(*ambient_temp_c),
//                data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? auto_temp_c : auto_temp_f, DISPLAY_COLOR);

            // Testing Line
            // TEST_FTC
//			float lf_temp =0;         // Testing Line Begin
//			float lf_temp_roundOff =0;
//			int ambient_temp_f = 0;
//			lf_temp = celsius_to_fahr(*ambient_temp_c);
//			lf_temp_roundOff = round(lf_temp);
//			ambient_temp_f = lf_temp_roundOff;
//			// *temp_f = new_intTemp; // End

			int ambient_temp_f = 0;
			ambient_temp_f  = valueRoundOff(*ambient_temp_c, CONVERT_C_TO_F);

            display_auto_mode(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *ambient_temp_c : ambient_temp_f,
                data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? auto_temp_c : auto_temp_f, DISPLAY_COLOR);


            // display AUTO icon
            display_auto_icon(DISPLAY_COLOR);
            // display child lock icon if locked
            if (data->is_child_lock_active)
                display_child_lock_icon(DISPLAY_COLOR);
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();
}





static void menu_mode_task(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = -1;
    app_mode_t *mode = &(data->mode);
    bool update_display = true;
    uint8_t menu = 0;
    time_t btn_power_press_ms = 0;

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

#ifdef TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
     menuModeKeypressedFlag = 1;
#endif

    // display menu navigation screen
    display_clear_screen();
    display_menu_inst(DISPLAY_COLOR);


    // display for MENU_NAVIGATION_SCREEN_DISPLAY_DURATION_MS milliseconds
    vTaskDelay(MENU_NAVIGATION_SCREEN_DISPLAY_DURATION_MS / portTICK_RATE_MS);

    // set prev_btn to current button status
    prev_btn = *btn;

    // menu starts with schedule
    menu = MENU_CALENDAR;

    while(*mode == APP_MODE_MENU) {


        /* button
           - single press BACK button to go back to previous screen if not on the first level
           - single press BACK button to go back to previous mode if on the first level
           - single press UP or DOWN button to navigate the menu
           - single press FORWARD button to enter selected menu
           - long press POWER button to go to standby mode
        */
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        *mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        *mode = data->menu_mode_exit_mode;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        if (menu == MENU_LAST) {
                            menu = MENU_FIRST;
                        } else {
                            menu++;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        if (menu == MENU_FIRST) {
                            menu = MENU_LAST;
                        } else {
                            menu--;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (menu) {
/*
                        case MENU_ENERGY:
                            *mode = menu_energy(data);
                            break;
*/
//                        case MENU_CALENDAR:   // Earlier Working in last firmware..
//                            *mode = menu_calendar(data);
//                            break;
                        case MENU_TIME_DATE:
                            *mode = menu_time_and_date(data);
                            break;
                        case MENU_COMMUNICATIONS:
                            *mode = menu_communications(data);
                            break;
                        case MENU_SETTINGS:
                            *mode = menu_settings(data);
                            break;
                        case MENU_DISPLAY_SETTINGS:
                            *mode = menu_display_settings(data);
                            break;
                        case MENU_UPDATE:
                            *mode = menu_update(data);
                            break;
                        }

                        // reset time of screen on
                        t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update display
        if (update_display) {
            update_display = false;

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);
            switch (menu) {
/*
            case MENU_ENERGY:
                display_menu("Energy", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
*/
            case MENU_CALENDAR:
                display_menu("Auto", DISPLAY_COLOR, "Mode", DISPLAY_COLOR);
                break;
            case MENU_TIME_DATE:
                display_menu("Time &", DISPLAY_COLOR, "Date", DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS:
                display_menu("Wi-Fi", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_SETTINGS:
                display_menu("Settings", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS:
                display_menu("Display", DISPLAY_COLOR, "Settings", DISPLAY_COLOR);
                break;
            case MENU_UPDATE:
                display_menu("Update", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            }
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();
}

static void button_up_cb(int level) {
    uint8_t prev = (app_data->button_status >> BUTTON_UP_STAT) & 0x01;

    if (prev != level) {
       // printf("ui : UP\r\n");

        if (level)
            app_data->button_status |= 1 << BUTTON_UP_STAT;
        else
            app_data->button_status &= ~(1 << BUTTON_UP_STAT);
    }
}

static void button_down_cb(int level) {
    uint8_t prev = (app_data->button_status >> BUTTON_DOWN_STAT) & 0x01;

    if (prev != level) {
        // printf("ui : D0WN\r\n");

        if (level)
            app_data->button_status |= 1 << BUTTON_DOWN_STAT;
        else
            app_data->button_status &= ~(1 << BUTTON_DOWN_STAT);
    }
}

static void button_power_back_cb(int level) {
    uint8_t prev = (app_data->button_status >> BUTTON_POWER_BACK_STAT) & 0x01;

    if (prev != level) {
        //printf("ui : POWER_BACK\r\n");

        if (level)
            app_data->button_status |= 1 << BUTTON_POWER_BACK_STAT;
        else
            app_data->button_status &= ~(1 << BUTTON_POWER_BACK_STAT);
    }
}

static void button_timer_forward_cb(int level) {
    uint8_t prev = (app_data->button_status >> BUTTON_TIMER_FORWARD_STAT) & 0x01;

    if (prev != level) {
       // printf("ui : TIMER_FORWARD\r\n");

        if (level)
            app_data->button_status |= 1 << BUTTON_TIMER_FORWARD_STAT;
        else
            app_data->button_status &= ~(1 << BUTTON_TIMER_FORWARD_STAT);
    }
}

static app_mode_t menu_calendar(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

#if 1 

    uint8_t m_cal = MENU_DISPLAY_SETTINGS_BRIGHTNESS;
    bool update_display = true;
    time_t btn_power_press_ms = 0;

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    auto_mode_sched_t *sched = NULL;
    uint8_t sched_num = 0;
    char sched_num_str[11];
    bool weekend = false; // true if weekend, false if weekday
    bool is_sched_changed = false;

    uint8_t *temp = NULL, *temp_c = NULL, *temp_f = NULL;

      // Original Last firmware release
//    uint8_t temp_max = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_CELSIUS_VAL_MAX : TEMPERATURE_FAHRENHEIT_VAL_MAX;
//    uint8_t temp_min = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_CELSIUS_VAL_MIN : TEMPERATURE_FAHRENHEIT_VAL_MIN;

#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
    uint8_t temp_max = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX : TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX;
    uint8_t temp_min = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN : TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN;
#else
    uint8_t temp_max = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_CELSIUS_VAL_MAX : TEMPERATURE_FAHRENHEIT_VAL_MAX;
    uint8_t temp_min = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? TEMPERATURE_CELSIUS_VAL_MIN : TEMPERATURE_FAHRENHEIT_VAL_MIN;

#endif

    while (!exit) {
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else {
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_cal) {
                        case MENU_CALENDAR_WEEKTYPE:
                            exit = true;
                            break;
                        case MENU_CALENDAR_SCHED:
                            m_cal = MENU_CALENDAR_WEEKTYPE;
                            break;
                        case MENU_CALENDAR_SCHED_EN:
                        case MENU_CALENDAR_SCHED_TIME:
                        case MENU_CALENDAR_SCHED_TEMP:
                            m_cal = MENU_CALENDAR_SCHED;
                            break;
                        case MENU_CALENDAR_SCHED_EN_STAT:
                            m_cal = MENU_CALENDAR_SCHED_EN;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR:
                        case MENU_CALENDAR_SCHED_TIME_MINUTE:
                            m_cal = MENU_CALENDAR_SCHED_TIME;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP_CHANGE:
                            m_cal = MENU_CALENDAR_SCHED_TEMP;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE:
                            m_cal = MENU_CALENDAR_SCHED_TIME_HOUR;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE:
                            m_cal = MENU_CALENDAR_SCHED_TIME_MINUTE;
                            break;
                        }

                        update_display = true;
                    } else { // pressed
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        switch (m_cal) {
                        case MENU_CALENDAR_WEEKTYPE:
                            weekend = !weekend;
                            break;
                        case MENU_CALENDAR_SCHED:
                            if (sched_num < (AUTO_MODE_SCHED_NUM - 1))
                                ++sched_num;
                            else
                                sched_num = 0;
                            break;
                        case MENU_CALENDAR_SCHED_EN:
                            m_cal = MENU_CALENDAR_SCHED_TIME;
                            break;
                        case MENU_CALENDAR_SCHED_TIME:
                            m_cal = MENU_CALENDAR_SCHED_TEMP;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP:
                            m_cal = MENU_CALENDAR_SCHED_EN;
                            break;
                        case MENU_CALENDAR_SCHED_EN_STAT:
                            sched[sched_num].en = !sched[sched_num].en;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR:
                            m_cal = MENU_CALENDAR_SCHED_TIME_MINUTE;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE:
                            m_cal = MENU_CALENDAR_SCHED_TIME_HOUR;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP_CHANGE:
                            if (*temp < temp_max) {
                                is_sched_changed = true;
                                *temp += 1;

                                // update value of the other unit
                                if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
                                {    // *temp_f = celsius_to_fahr(*temp_c);  // Original _TEST_FTC
//									float lf_temp =0;         // Testing Line Begin
//									float lf_temp_roundOff =0;
//									int ambient_temp_f = 0;
//									lf_temp = celsius_to_fahr(*temp_c);
//									lf_temp_roundOff = round(lf_temp);
//									ambient_temp_f = lf_temp_roundOff;
//									*temp_f = ambient_temp_f; // End

									*temp_f  = valueRoundOff(*temp_c, CONVERT_C_TO_F);

                                }
                                else
                                {  //  *temp_c = fahr_to_celsius(*temp_f);  // Original

//									float lf_temp =0;         // Testing Line Begin
//									float lf_temp_roundOff =0;
//									int ambient_temp_c = 0;
//									lf_temp = fahr_to_celsius(*temp_f);
//									lf_temp_roundOff = round(lf_temp);
//									ambient_temp_c = lf_temp_roundOff;
//									*temp_c = ambient_temp_c; // End

									*temp_c  = valueRoundOff(*temp_f, CONVERT_F_TO_C);

                                }
                            }
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE:
                            if (sched[sched_num].hour < 23) {
                                is_sched_changed = true;
                                sched[sched_num].hour++;
                            }
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE:
                            if (sched[sched_num].minute < 59) {
                                is_sched_changed = true;
                                sched[sched_num].minute++;
                            }
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        switch (m_cal) {
                        case MENU_CALENDAR_WEEKTYPE:
                            weekend = !weekend;
                            break;
                        case MENU_CALENDAR_SCHED:
                            if (sched_num > 0)
                                --sched_num;
                            else
                                sched_num = AUTO_MODE_SCHED_NUM - 1;
                            break;
                        case MENU_CALENDAR_SCHED_EN:
                            m_cal = MENU_CALENDAR_SCHED_TEMP;
                            break;
                        case MENU_CALENDAR_SCHED_TIME:
                            m_cal = MENU_CALENDAR_SCHED_EN;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP:
                            m_cal = MENU_CALENDAR_SCHED_TIME;
                            break;
                        case MENU_CALENDAR_SCHED_EN_STAT:
                            sched[sched_num].en = !sched[sched_num].en;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR:
                            m_cal = MENU_CALENDAR_SCHED_TIME_MINUTE;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE:
                            m_cal = MENU_CALENDAR_SCHED_TIME_HOUR;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP_CHANGE:
                            if (*temp > temp_min) {
                                is_sched_changed = true;
                                *temp -= 1;

                                // update value of the other unit
                                if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
                                {
                                	// *temp_f = celsius_to_fahr(*temp_c);   // Original _TEST_FTC
//                             	    float lf_temp =0;         // Testing Line Begin
//                            		float lf_temp_roundOff =0;
//                            		int ambient_temp_f = 0;
//                            		lf_temp = celsius_to_fahr(*temp_c);
//                            		lf_temp_roundOff = round(lf_temp);
//                            		ambient_temp_f = lf_temp_roundOff;
//                            		*temp_f = ambient_temp_f; // End

                            		*temp_f  = valueRoundOff(*temp_c, CONVERT_C_TO_F);

                                }
                                else
                                {  // *temp_c = fahr_to_celsius(*temp_f);  // Original

//									float lf_temp =0;         // Testing Line Begin
//									float lf_temp_roundOff =0;
//									int ambient_temp_c = 0;
//									lf_temp = fahr_to_celsius(*temp_f);
//									lf_temp_roundOff = round(lf_temp);
//									ambient_temp_c = lf_temp_roundOff;
//									*temp_c = ambient_temp_c; // End

									*temp_c  = valueRoundOff(*temp_f, CONVERT_F_TO_C);

                                }
                            }
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE:
                            if (sched[sched_num].hour > 0) {
                                is_sched_changed = true;
                                sched[sched_num].hour--;
                            }
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE:
                            if (sched[sched_num].minute > 0) {
                                is_sched_changed = true;
                                sched[sched_num].minute--;
                            }
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_cal) {
                        case MENU_CALENDAR_WEEKTYPE:
                            sched_num = 0;

                            // set sched
                            if (weekend)
                                sched = sched_weekend;
                            else
                                sched = sched_weekday;

                            m_cal = MENU_CALENDAR_SCHED;
                            break;
                        case MENU_CALENDAR_SCHED:
                            // get target temp
                            temp_c = &(sched[sched_num].temp_c);
                            temp_f = &(sched[sched_num].temp_f);
                            temp = data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? temp_c : temp_f;
                            m_cal = MENU_CALENDAR_SCHED_EN;
                            break;
                        case MENU_CALENDAR_SCHED_EN:
                            m_cal = MENU_CALENDAR_SCHED_EN_STAT;
                            break;
                        case MENU_CALENDAR_SCHED_TIME:
                            m_cal = MENU_CALENDAR_SCHED_TIME_HOUR;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP:
                            m_cal = MENU_CALENDAR_SCHED_TEMP_CHANGE;
                            break;
                        case MENU_CALENDAR_SCHED_EN_STAT:
                            // do nothing
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR:
                            m_cal = MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE;
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE:
                            m_cal = MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE;
                            break;
                        case MENU_CALENDAR_SCHED_TEMP_CHANGE:
                            // do nothing
                            break;
                        case MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE:
                            // do nothing
                            break;
                        case MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE:
                            // do nothing
                            break;
                        }

                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update display
        if (update_display) {
            update_display = false;

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);
            switch (m_cal) {
            case MENU_CALENDAR_WEEKTYPE:
                display_menu(weekend ? "Weekend" : "Weekday", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED:
                if (sched_num == 0) {
                    sprintf(sched_num_str, "Wake");
                } else if (sched_num == 1) {
                    sprintf(sched_num_str, "Leave");
                } else if (sched_num == 2) {
                    sprintf(sched_num_str, "Return");
                } else if (sched_num == 3) {
                    sprintf(sched_num_str, "Sleep");
                }
//                sprintf(sched_num_str, "Sched %d", sched_num + 1);
                display_menu(sched_num_str, DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_EN:
                display_menu("Enable", DISPLAY_COLOR, sched_num_str, DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TIME:
                display_menu("Time", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TEMP:
                display_menu("Temp", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_EN_STAT:
                display_menu(sched[sched_num].en ? "Enabled" :  "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TIME_HOUR:
                display_menu("Hour", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TIME_MINUTE:
                display_menu("Minute", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TEMP_CHANGE:
                display_temperature(*temp, DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TIME_HOUR_CHANGE:
                display_hour(sched[sched_num].hour, DISPLAY_COLOR);
                break;
            case MENU_CALENDAR_SCHED_TIME_MINUTE_CHANGE:
                display_minute(sched[sched_num].minute, DISPLAY_COLOR);
                break;
            }

        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();

    // save schedule if changed
    if (is_sched_changed) {
        set_data_to_storage(STORAGE_KEY_SCHED_WEEKDAY, (void *) sched_weekday, sizeof(sched_weekday));
        set_data_to_storage(STORAGE_KEY_SCHED_WEEKEND, (void *) sched_weekend, sizeof(sched_weekend));
    }

#endif
    // return new mode
    return next_mode;
}


static app_mode_t menu_time_and_date(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

    uint8_t m_timedate = MENU_TIME_AND_DATE_AUTO;
    bool update_display = true;
    int year, month, day, hour, minute;
    time_t btn_power_press_ms = 0;

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    bool display_date_change_en = false;
    bool display_time_change_en = false;

    bool *is_auto_time_date_en = &(data->is_auto_time_date_en);
    bool prev_is_auto_time_date_en = *is_auto_time_date_en;

    int *timezone_offset_idx = &(data->timezone_offset_idx);
    int prev_timezone_offset_idx = *timezone_offset_idx;
    clock_get_date_and_time(&year, &month, &day, &hour, &minute, NULL);

//#define NTP_Testing_dayLightSaving
#ifdef NTP_Testing_dayLightSaving
    if(app_data->daylightSaving == 1)
	{
		if(hour==23)
			hour=0;
		else
			hour++;
	}
#endif

    while (!exit) {
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_timedate) {
                        case MENU_TIME_AND_DATE_AUTO:
                        case MENU_TIME_AND_DATE_MANUAL:
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET:
#ifdef	menuDateTime_DST
                        case MENU_TIME_AND_DATE_DST:
#endif
                            exit = true;
                            break;
#ifdef	menuDateTime_DST
                        case MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN:
                             m_timedate = MENU_TIME_AND_DATE_DST;
                             break;
#endif
                        case MENU_TIME_AND_DATE_AUTO_EN:
                            m_timedate = MENU_TIME_AND_DATE_AUTO;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE:
                        case MENU_TIME_AND_DATE_MANUAL_TIME:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET:
                            m_timedate = MENU_TIME_AND_DATE_TIMEZONE_OFFSET;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR:
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH:
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR:
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR_CHANGE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_YEAR;
                            display_date_change_en = true;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH_CHANGE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_MONTH;
                            display_date_change_en = true;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY_CHANGE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_DAY;
                            display_date_change_en = true;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR_CHANGE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_HOUR;
                            display_time_change_en = true;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE_CHANGE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE;
                            display_time_change_en = true;
                            break;
                        }

                        update_display = true;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        switch (m_timedate) {
                        case MENU_TIME_AND_DATE_AUTO:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL:
                            m_timedate = MENU_TIME_AND_DATE_TIMEZONE_OFFSET;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET:
#ifdef	menuDateTime_DST
                        	 m_timedate = MENU_TIME_AND_DATE_DST;
#else
                            m_timedate = MENU_TIME_AND_DATE_AUTO;

#endif
                            break;
#ifdef	menuDateTime_DST
                        case MENU_TIME_AND_DATE_DST:
                        	 m_timedate = MENU_TIME_AND_DATE_AUTO;
                        	 break;
                    	case MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN:
							// m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN;
							  data->daylightSaving = !data->daylightSaving;
							// Save pending //
							set_integer_to_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int)app_data->daylightSaving);
							manually_day_light_on_off_change_enable = 1;
							break;
#endif

                        case MENU_TIME_AND_DATE_AUTO_EN:
                            // toggle to enable <--> disable
                            *is_auto_time_date_en = !(*is_auto_time_date_en);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET:
			    if (*timezone_offset_idx < (TIMEZONE_OFFSET_LIST_SIZE - 1))
                                *timezone_offset_idx += 1;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_MONTH;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_DAY;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_YEAR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_HOUR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR_CHANGE:
                            // increment year
                            clock_get_date(&year, NULL, NULL);
                            clock_set_year(++year);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH_CHANGE:
                            // increment month
                            clock_get_date(NULL, &month, NULL);
                            clock_set_month(++month);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY_CHANGE:
                            // increment day
                            clock_get_date(NULL, NULL, &day);
                            clock_set_day(++day);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR_CHANGE:
                            // increment hour
                            clock_get_time(&hour, NULL, NULL);
                            clock_set_hour(++hour);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE_CHANGE:
                            // increment minute
                            clock_get_time(NULL, &minute, NULL);
                            clock_set_minute(++minute);
                            break;
                        }
                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        switch (m_timedate) {
                        case MENU_TIME_AND_DATE_AUTO:
#ifdef	menuDateTime_DST
                        	 m_timedate = MENU_TIME_AND_DATE_DST;
#else
                            m_timedate = MENU_TIME_AND_DATE_TIMEZONE_OFFSET;
#endif
                            break;
                        case MENU_TIME_AND_DATE_MANUAL:
                            m_timedate = MENU_TIME_AND_DATE_AUTO;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET:
                        	m_timedate = MENU_TIME_AND_DATE_MANUAL;
                            break;
#ifdef	menuDateTime_DST
                        case MENU_TIME_AND_DATE_DST:
                        	 m_timedate = MENU_TIME_AND_DATE_TIMEZONE_OFFSET;
                        	break;
                    	case MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN:
							// m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN;
							  data->daylightSaving = !data->daylightSaving;
							// Save pending //
							set_integer_to_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int)app_data->daylightSaving);
							manually_day_light_on_off_change_enable = 1;
							break;
#endif

                        case MENU_TIME_AND_DATE_AUTO_EN:
                            // toggle to enable <--> disable
                            *is_auto_time_date_en = !(*is_auto_time_date_en);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET:
                            if (*timezone_offset_idx > 0)
                                *timezone_offset_idx -= 1;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_DAY;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_YEAR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_MONTH;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_HOUR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR_CHANGE:
                            // decrement year
                            clock_get_date(&year, NULL, NULL);
                            if ((year - 1) >= MANUFACTURING_YEAR)
                                clock_set_year(--year);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH_CHANGE:
                            // decrement month
                            clock_get_date(NULL, &month, NULL);
                            clock_set_month(--month);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY_CHANGE:
                            // decrement day
                            clock_get_date(NULL, NULL, &day);
                            clock_set_day(--day);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR_CHANGE:
                            // decrement hour
                            clock_get_time(&hour, NULL, NULL);
                            clock_set_hour(--hour);
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE_CHANGE:
                            // decrement minute
                            clock_get_time(NULL, &minute, NULL);
                            clock_set_minute(--minute);
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_timedate) {
                        case MENU_TIME_AND_DATE_AUTO:
                            m_timedate = MENU_TIME_AND_DATE_AUTO_EN;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE;
                            break;
                        case MENU_TIME_AND_DATE_TIMEZONE_OFFSET:
                            m_timedate = MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET;
                            break;
#ifdef	menuDateTime_DST
    					case MENU_TIME_AND_DATE_DST:
    						m_timedate = MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN;
    						break;
#endif
                        case MENU_TIME_AND_DATE_MANUAL_DATE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_YEAR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_HOUR;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_YEAR_CHANGE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_MONTH_CHANGE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_DATE_DAY:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_DATE_DAY_CHANGE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_HOUR_CHANGE;
                            break;
                        case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE:
                            m_timedate = MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE_CHANGE;
                            break;
                        }
                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }

        if (*is_auto_time_date_en != prev_is_auto_time_date_en) {
            prev_is_auto_time_date_en = *is_auto_time_date_en;

            if (!ntp_is_enabled() && *is_auto_time_date_en)
                ntp_init(NTP_SERVER);
            else if (ntp_is_enabled() && *is_auto_time_date_en == false)
                ntp_deinit();

            // save new value to flash
            set_integer_to_storage(STORAGE_KEY_IS_AUTO_TIME_DATE_EN, (int) *is_auto_time_date_en);

            if (m_timedate == MENU_TIME_AND_DATE_AUTO_EN)
                update_display = true;
        }

	if (*timezone_offset_idx!= prev_timezone_offset_idx) {
            prev_timezone_offset_idx= *timezone_offset_idx;

	    // set timezone
	    clock_set_timezone_offset(timezone_offset_list_min[*timezone_offset_idx]);

            // save new value to flash
            set_integer_to_storage(STORAGE_KEY_TIMEZONE_OFFSET_INDEX, *timezone_offset_idx);

            if (m_timedate == MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET)
                update_display = true;
	}

        // update the display
        if (update_display) {
            printf("m_timedate=%d\r\n", m_timedate);
            update_display = false;

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            // display date / time screen if enabled
            if (display_date_change_en) {
                display_date_change_en = false;
                clock_get_date(&year, &month, &day);
                display_date(year, month, day, DISPLAY_COLOR);
                vTaskDelay(DISPLAY_DATE_AFTER_CHANGE_MS / portTICK_RATE_MS);
                display_date(year, month, day, !DISPLAY_COLOR);
            } else if (display_time_change_en) {
                display_time_change_en = false;
                clock_get_time(&hour, &minute, NULL);

				#ifdef NTP_Testing_dayLightSaving
					if(app_data->daylightSaving==1)  { if(hour==23)   hour=0;   else   hour++; }
				#endif

                display_time(hour, minute, DISPLAY_COLOR);
                vTaskDelay(DISPLAY_TIME_AFTER_CHANGE_MS / portTICK_RATE_MS);
                display_time(hour, minute, !DISPLAY_COLOR);
            }

            switch (m_timedate) {
            case MENU_TIME_AND_DATE_AUTO:
                display_menu("Auto", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL:
                display_menu("Manual", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_TIMEZONE_OFFSET:
                display_menu("Timezone", DISPLAY_COLOR, "Offset", DISPLAY_COLOR);
                break;

#ifdef  menuDateTime_DST
			case MENU_TIME_AND_DATE_DST:
				printf("MENU_TIME_AND_DATE_DST \r\n");
				// display_menu("Day Light", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
				display_menu("DST", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
				break;
			case MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN:
					printf("MENU_TIME_AND_DATE_DST_ON_OFF_CHANGE_EN\r\n");
					display_menu(data->daylightSaving ? "ON" : "OFF", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
					printf("data->daylightSaving  %d \n", data->daylightSaving);
					break;
#endif

            case MENU_TIME_AND_DATE_AUTO_EN:
                display_menu(*is_auto_time_date_en ? "Enabled" : "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE:
                display_menu("Date", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_TIME:
                display_menu("Time", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_TIMEZONE_OFFSET_SET:
                display_timezone(timezone_offset_list_min[*timezone_offset_idx], DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR:
                display_menu("Year", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH:
                display_menu("Month", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_DAY:
                display_menu("Day", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR:
                display_menu("Hour", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE:
                display_menu("Minute", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_YEAR_CHANGE:
                clock_get_date(&year, NULL, NULL);
                display_year(year, DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_MONTH_CHANGE:
                clock_get_date(NULL, &month, NULL);
                display_month(month, DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_DATE_DAY_CHANGE:
                clock_get_date(NULL, NULL, &day);
                display_day(day, DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_TIME_HOUR_CHANGE:
                clock_get_time(&hour, NULL, NULL);

				#ifdef NTP_Testing_dayLightSaving
					if(app_data->daylightSaving==1)  { if(hour==23)   hour=0;   else   hour++; }
				#endif

                display_hour(hour, DISPLAY_COLOR);
                break;
            case MENU_TIME_AND_DATE_MANUAL_TIME_MINUTE_CHANGE:
                clock_get_time(NULL, &minute, NULL);
                display_minute(minute, DISPLAY_COLOR);
                break;
            }
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();

    // return new mode
    return next_mode;
}


int valueRoundOff(int pvalue, int ptemp_Conversion_unit)
{
	float lf_temp =0;         // Testing Line Begin_TEST
	float lf_temp_roundOff =0;
	int lnTemp = 0;

	if (ptemp_Conversion_unit == CONVERT_C_TO_F)
	   lf_temp = celsius_to_fahr(pvalue);
	else
	   lf_temp = fahr_to_celsius(pvalue);

	lf_temp_roundOff = round(lf_temp);
	lnTemp = lf_temp_roundOff;
	return lnTemp;
}


static app_mode_t menu_communications(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

    uint8_t m_comms = MENU_COMMUNICATIONS_AP_MODE;
    bool update_display = true;
    time_t btn_power_press_ms = 0;
    time_t btn_forward_press_ms = 0;
    time_t btn_up_press_ms = 0;
    time_t btn_down_press_ms = 0;
    time_t t_last_char_blink_ms = 0;

    char buf[MENU_COMMS_BUF_MAX_LEN + 1];
    size_t input_len = 0;
    char *current_char = NULL;
    bool is_char_change = false;
    uint8_t char_fast_scroll_increment = 0;
    bool last_char_blink = false;

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!exit) {
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                } else if (!((*btn >> BUTTON_UP_STAT) & 0x01)) { // up button is pressed
//                    if ((cur_ms - btn_up_press_ms) >= QUICK_SCROLL_LONG_PRESS_DUR_MS) {
//                        if ((m_comms == MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE)
//                            || (m_comms == MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE)) {
//                            if (input_len == 0) {
//                                current_char = &(buf[0]);
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_FIRST_CHAR; // set the first index value to the first char
//                                input_len = 1;
//                            } else if (*current_char == 0) {
//                                *current_char = *(current_char - 1);
//                            } else {
//                                char_fast_scroll_increment += 2;
//                                if (char_fast_scroll_increment > 10) {
//                                    char_fast_scroll_increment = 10;
//                                }
//                                *current_char += char_fast_scroll_increment;
//                                if (*current_char > MENU_COMMS_INPUT_CHARACTER_MAX_VAL) {
//                                    *current_char = MENU_COMMS_INPUT_CHARACTER_MAX_VAL;
//                                }
//                            }
//                            is_char_change = true;
//                        }
//                        btn_up_press_ms = cur_ms;
//                    }
                } else if (!((*btn >> BUTTON_DOWN_STAT) & 0x01)) { // down button is pressed
//                    if ((cur_ms - btn_down_press_ms) >= QUICK_SCROLL_LONG_PRESS_DUR_MS) {
//                        if ((m_comms == MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE)
//                            || (m_comms == MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE)) {
//                            if (input_len == 0) {
//                                current_char = &(buf[0]);
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_FIRST_CHAR; // set the first index value to the first char
//                                input_len = 1;
//                            } else if (*current_char == 0) {
//                                *current_char = *(current_char - 1);
//                            } else {
//                                char_fast_scroll_increment += 2;
//                                if (char_fast_scroll_increment > 10) {
//                                    char_fast_scroll_increment = 10;
//                                }
//                                *current_char -= char_fast_scroll_increment;
//                                if (*current_char < MENU_COMMS_INPUT_CHARACTER_MIN_VAL) {
//                                    *current_char = MENU_COMMS_INPUT_CHARACTER_MIN_VAL;
//                                }
//                            }
//                            is_char_change = true;
//                        }
//                        btn_down_press_ms = cur_ms;
//                    }
                } else if (!((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) { // forward button is pressed
                    if ((cur_ms - btn_forward_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        if (m_comms == MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE) {
                            memcpy(data->ap_ssid, buf, strlen(buf));
                            set_string_to_storage(NVS_LUCIDTRON_SSID_KEY, buf);

                            m_comms = MENU_COMMUNICATIONS_WIFI_AP;
                        } else if (m_comms == MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE) {
                            memcpy(data->sta_pw, buf, strlen(buf));
                            set_string_to_storage(NVS_LUCIDTRON_PW_KEY, buf);
                            comm_wifi_dev->wifi_client_enable(data->ap_ssid, data->sta_pw);

                            m_comms = MENU_COMMUNICATIONS_WIFI_AP;
                        }
                        btn_forward_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_comms) {
                        case MENU_COMMUNICATIONS_AP_MODE:
                        case MENU_COMMUNICATIONS_WIFI_AP:
                        case MENU_COMMUNICATIONS_WPS:
#ifdef RESET_SSID_PASS
                        case MENU_COMMUNICATIONS_RESET_SSID_PASS:
#endif
                            exit = true;
                            break;
#ifdef RESET_SSID_PASS
                        case MENU_COMMUNICATIONS_RESET_ARE_YOU_SURE_CONFIRM:
                        	  m_comms = MENU_COMMUNICATIONS_RESET_SSID_PASS;
                        	break;
#endif

                        case MENU_COMMUNICATIONS_AP_MODE_EN:
                        case MENU_COMMUNICATIONS_AP_MODE_SSID:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE;
                            break;
                        case MENU_COMMUNICATIONS_WIFI_AP_SSID:
                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD:
                            m_comms = MENU_COMMUNICATIONS_WIFI_AP;
                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN:// commented for testing
//                            m_comms = MENU_COMMUNICATIONS_WPS;
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID_VAL:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_SSID;
                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
//                            if (input_len > 0) {
//                                // delete end character
//                                *current_char = 0x00;
//                                current_char--;
//                                --input_len;
//                            } else {
//                                m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID;
//                            }
//                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
//                            if (input_len > 0) {
//                                // delete end character
//                                *current_char = 0x00;
//                                current_char--;
//                                --input_len;
//                            } else {
//                                m_comms = MENU_COMMUNICATIONS_WIFI_AP_PASSWORD;
//                            }
//                            break;

											case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
//												if (input_len > 0) {
//													// delete end character
//													*current_char = 0x00;
//													current_char--;
//													--input_len;
//												} else {
													m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID;
												//}
												break;
											case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
//												if (input_len > 0) {
//													// delete end character
//													*current_char = 0x00;
//													current_char--;
//													--input_len;
//												} else {
													m_comms = MENU_COMMUNICATIONS_WIFI_AP_PASSWORD;
												//}
												break;

//                        case MENU_COMMUNICATIONS_WPS_EN_INST:// commented for testing
//                            // do nothing
//                            break;
                        }

                        update_display = true;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        switch (m_comms) {
                        case MENU_COMMUNICATIONS_AP_MODE:
                            m_comms = MENU_COMMUNICATIONS_WIFI_AP;
                            break;

#ifdef RESET_SSID_PASS
//						case MENU_COMMUNICATIONS_AP_MODE: // commented for testing
//							m_comms = MENU_COMMUNICATIONS_WIFI_AP;
//							break;
                        case MENU_COMMUNICATIONS_WIFI_AP:
                        	 m_comms = MENU_COMMUNICATIONS_RESET_SSID_PASS;
                        	break;
						case MENU_COMMUNICATIONS_RESET_SSID_PASS:
							m_comms = MENU_COMMUNICATIONS_AP_MODE;
							break;
#endif


//                        case MENU_COMMUNICATIONS_WIFI_AP:// commented for testing
//                            m_comms = MENU_COMMUNICATIONS_WPS;
//                            break;
//                        case MENU_COMMUNICATIONS_WPS:// commented for testing
//                            m_comms = MENU_COMMUNICATIONS_AP_MODE;
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_EN:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_SSID;
                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_EN;
                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID:// commented on 19Nov2020
//                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_PASSWORD;
                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD:  // commented on 19Nov2020
//                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID;
//                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN:  // commented for testing
//                            // do nothing
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID_VAL:
                            // do nothing
                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN_INST:  // commented for testing
//                            // do nothing
//                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
                            // reset multiplier
//                            char_fast_scroll_increment = 0;
//                            if (input_len == 0) {
//                                current_char = &(buf[0]);
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_FIRST_CHAR; // set the first index value to the first char
//                                input_len = 1;
//                            } else if (*current_char == 0) {
//                                *current_char = *(current_char - 1);
//                            } else if (*current_char == MENU_COMMS_INPUT_CHARACTER_MAX_VAL) {
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_MIN_VAL;
//                            } else {
//                                *current_char += 1;
//                            }
//                            is_char_change = true;
//                            break;
                        }

                        update_display = true;
                    } else {
                        btn_up_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        switch (m_comms) {
//                        case MENU_COMMUNICATIONS_AP_MODE: // commented for testing
//                            m_comms = MENU_COMMUNICATIONS_WPS;
//                            break;
#ifdef RESET_SSID_PASS
						case MENU_COMMUNICATIONS_AP_MODE: // commented for testing
							m_comms = MENU_COMMUNICATIONS_WIFI_AP;
							break;
//                        case MENU_COMMUNICATIONS_WIFI_AP:
//                        	 m_comms = MENU_COMMUNICATIONS_RESET_SSID_PASS;
//                        	break;
						case MENU_COMMUNICATIONS_RESET_SSID_PASS:
							m_comms = MENU_COMMUNICATIONS_AP_MODE;
							break;

                        case MENU_COMMUNICATIONS_WIFI_AP:
                            m_comms = MENU_COMMUNICATIONS_RESET_SSID_PASS;
                            break;
#endif
//                        case MENU_COMMUNICATIONS_WIFI_AP:  // Commented only for testing
//                            m_comms = MENU_COMMUNICATIONS_AP_MODE;
//                            break;
//                        case MENU_COMMUNICATIONS_WPS:   // Commented only for testing
//                            m_comms = MENU_COMMUNICATIONS_WIFI_AP;
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_EN:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_SSID;
                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_EN;
                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID:  // commented on 19Nov2020
//                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_PASSWORD;
//                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD:// commented on 19Nov2020
//                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID;
//                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN:// commented for testing
//                            // do nothing
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID_VAL:
                            // do nothing
                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN_INST:// commented for testing
//                            // disable WPS
//                            comm_wifi_dev->wps_disable();
//                            m_comms = MENU_COMMUNICATIONS_WPS;
//                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
//                            // input
//                            // reset multiplier
//                            char_fast_scroll_increment = 0;
//                            if (input_len == 0) {
//                                current_char = &(buf[0]);
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_FIRST_CHAR; // set the first index value to the first char
//                                input_len = 1;
//                            } else if (*current_char == 0) {
//                                *current_char = *(current_char - 1);
//                            } else if (*current_char == MENU_COMMS_INPUT_CHARACTER_MIN_VAL) {
//                                *current_char = MENU_COMMS_INPUT_CHARACTER_MAX_VAL;
//                            } else {
//                                *current_char -= 1;
//                            }
//                            is_char_change = true;
//                            break;
                        }
                        update_display = true;
                    } else {
                        btn_down_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_comms) {
                        case MENU_COMMUNICATIONS_AP_MODE:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_EN;
                            break;
#ifdef RESET_SSID_PASS
                        case MENU_COMMUNICATIONS_RESET_SSID_PASS:
                        	 m_comms = MENU_COMMUNICATIONS_RESET_ARE_YOU_SURE_CONFIRM;
                        	break;
                        case MENU_COMMUNICATIONS_RESET_ARE_YOU_SURE_CONFIRM:
                        	  m_comms = MENU_COMMUNICATIONS_RESET_CONFIRMED;
                        	break;
#endif
                        case MENU_COMMUNICATIONS_WIFI_AP:
                            // get Wi-Fi AP info
                        	printf("Line no 3228 \n ");
                            esp_wifi_sta_get_ap_info(&ap_info);
                            memset(data->sta_ssid, 0, sizeof(data->sta_ssid));
                            memcpy(data->sta_ssid, ap_info.ssid, strlen((char *) ap_info.ssid));
                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID;
                            break;
//                        case MENU_COMMUNICATIONS_WPS:// commented for testing
//                            m_comms = MENU_COMMUNICATIONS_WPS_EN;
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_EN:
                        	printf("Line no 3238 \n ");
//                            // enable AP mode
//                            if (comm_wifi_dev->is_wifi_ap_enabled())
//                                comm_wifi_dev->wifi_ap_disable();
//                            else
//                            	comm_wifi_dev->wifi_ap_enable(uniqueDeviceID, ap_password);  // testing
                                //comm_wifi_dev->wifi_ap_enable(comm_wifi_dev->wifi_ap_ssid, comm_wifi_dev->wifi_ap_pw);  // original

                        	// comm_wifi_dev->wifi_ap_enable(uniqueDeviceID, ap_password);
                        	if(esp32_wifi_status != ESP32_WIFI_AP)
                        	{	printf("esp32_wifi_ap_enable \n ");
                        		esp32_wifi_ap_enable(uniqueDeviceID, ap_password);
                        	}
                        	else
                        	{	printf("esp32_wifi_client_enable_Testing_menu \n ");
                        		// esp32_wifi_client_enable(username,password);   // original ...commented on 22jan
                        	     esp32_wifi_client_enable_Testing_menu(username,password);  // testing only ..
                        	}
                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID:
                            m_comms = MENU_COMMUNICATIONS_AP_MODE_SSID_VAL;
                            break;
                        case MENU_COMMUNICATIONS_WIFI_AP_SSID:
                            memset(buf, 0, MENU_COMMS_BUF_MAX_LEN + 1);
                            get_string_from_storage(NVS_LUCIDTRON_SSID_KEY, buf);
                            input_len = strlen(buf);

                            // set current char
                            current_char = &(buf[input_len]);

                            is_char_change = false;

                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE;
                            t_last_char_blink_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            last_char_blink = false;
                            break;
                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD:
                            memset(buf, 0, MENU_COMMS_BUF_MAX_LEN + 1);
                            get_string_from_storage(NVS_LUCIDTRON_PW_KEY, buf);
                            input_len = strlen(buf);

                            // set current char
                            current_char = &(buf[input_len]);

                            is_char_change = false;

                            m_comms = MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE;
                            t_last_char_blink_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            last_char_blink = false;
                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN:  // commented for testing
//                            // WPS
//                            // start WPS
//                            comm_wifi_dev->wps_enable();
//                            m_comms = MENU_COMMUNICATIONS_WPS_EN_INST;
//                            break;
                        case MENU_COMMUNICATIONS_AP_MODE_SSID_VAL:
                            // do nothing
                            break;
//                        case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
//                        case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
//                            current_char++;
//                            *current_char = 0;
//                            ++input_len;
//                            is_char_change = true;
//                            break;
//                        case MENU_COMMUNICATIONS_WPS_EN_INST:// commented for testing
//                            // do nothing
//                            break;
                        }

                        update_display = true;
                    } else {
                        btn_forward_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                }
            }
            prev_btn = *btn;
        }

        if (is_char_change) {
            is_char_change = false;
            update_display = true;
        }

        if ((m_comms == MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE)
            || (m_comms == MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE)) {
            int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((cur_ms - t_last_char_blink_ms) >= 1000) {
                last_char_blink = !last_char_blink;
                t_last_char_blink_ms = cur_ms;
                if (*current_char != 0) {
                    update_display = true;
                }
            }
        }

        // update the display
        if (update_display) {
            printf("m_comms=%d\r\n", m_comms);

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            switch (m_comms) {
            case MENU_COMMUNICATIONS_AP_MODE:
                printf("MENU_COMMUNICATIONS_AP_MODE\r\n");
                display_menu("AP Mode", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS_WIFI_AP:  // removed wifi from lcd..11Nov20
                display_menu("Wi-Fi", DISPLAY_COLOR, "connection", DISPLAY_COLOR);
                printf("MENU_COMMUNICATIONS_WIFI_AP\r\n");
                break;
//            case MENU_COMMUNICATIONS_WPS:
//                printf("MENU_COMMUNICATIONS_WPS\r\n");  // commented for testing
//                display_menu("WPS", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
//                break;
            case MENU_COMMUNICATIONS_AP_MODE_EN:
                printf("MENU_COMMUNICATIONS_AP_MODE_EN\r\n" );
               // comm_wifi_dev->is_wifi_ap_enabled() = (&esp32_wifi_is_ap_enabled); // New Added for testing omnly ..
              //  printf("comm_wifi_dev->is_wifi_ap_enabled() %d",comm_wifi_dev->is_wifi_ap_enabled() );
               // display_menu(comm_wifi_dev->is_wifi_ap_enabled() ? "Disable" : "Enable", DISPLAY_COLOR, "AP mode", DISPLAY_COLOR); //original  Commented  because  it is struck in Firmware..
               //  display_menu(1 ? "Disable" : "Enable", DISPLAY_COLOR, "AP mode", DISPLAY_COLOR); // Commented  Only fpr testiing
               //  display_menu(esp32_wifi_is_ap_enabled() ? "Disable" : "Enable", DISPLAY_COLOR, "AP mode", DISPLAY_COLOR); // Commented  Only fpr testiing
                 display_menu((esp32_wifi_status == ESP32_WIFI_AP) ? "Disable" : "Enable", DISPLAY_COLOR, "AP mode", DISPLAY_COLOR); // Commented  Only fpr testiing

                break;
            case MENU_COMMUNICATIONS_AP_MODE_SSID:
                printf("MENU_COMMUNICATIONS_AP_MODE_SSID\r\n");
                display_menu("SSID", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS_WIFI_AP_SSID:
                printf("MENU_COMMUNICATIONS_WIFI_AP_SSID\r\n");
                display_menu("Network", DISPLAY_COLOR, "Name", DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD:
                printf("MENU_COMMUNICATIONS_WIFI_AP_PASSWORD\r\n");
                display_menu("Password", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS_WPS_EN:
                printf("MENU_COMMUNICATIONS_WPS_EN\r\n");
                display_menu("Enable", DISPLAY_COLOR, "WPS Mode", DISPLAY_COLOR);
                break;
            case MENU_COMMUNICATIONS_AP_MODE_SSID_VAL:
                printf("MENU_COMMUNICATIONS_AP_MODE_SSID_VAL\r\n");

//                printf("comm_wifi_dev->wifi_ap_ssid %s\n",comm_wifi_dev->wifi_ap_ssid);
//                // display_ssid(comm_wifi_dev->wifi_ap_ssid, DISPLAY_COLOR);  // Original
//                display_ssid(comm_wifi_dev->wifi_ap_ssid, DISPLAY_COLOR);   //Testing

                printf("uniqueDeviceID %s\n",uniqueDeviceID);

                // display_ssid(comm_wifi_dev->wifi_ap_ssid, DISPLAY_COLOR);  // Original
                display_ssid(uniqueDeviceID, DISPLAY_COLOR);   //Testing

                break;
            case MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE:
                printf("MENU_COMMUNICATIONS_WIFI_AP_SSID_CHANGE\r\n");
                display_ssid(buf, DISPLAY_COLOR);
                if (last_char_blink) {
                    display_clear_last_char_ssid_pw(*current_char);
                    printf("\r\nlast char %c\r\n", *current_char);
                }
                break;
            case MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE:
                printf("MENU_COMMUNICATIONS_WIFI_AP_PASSWORD_CHANGE\r\n");
                display_password(buf, DISPLAY_COLOR);
                if (last_char_blink) {
                    display_clear_last_char_ssid_pw(*current_char);
                    printf("\r\nlast char %c\r\n", *current_char);
                }
                break;
//            case MENU_COMMUNICATIONS_WPS_EN_INST:
//                printf("MENU_COMMUNICATIONS_WPS_EN_INST\r\n");  // commented for testing
//                display_wps_mode_msg(DISPLAY_COLOR);
//                break;

#ifdef RESET_SSID_PASS
			case MENU_COMMUNICATIONS_RESET_SSID_PASS:
				printf("MENU_COMMUNICATIONS_RESET_SSID_PASS\r\n");
				display_menu("Reset", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
				break;
			case MENU_COMMUNICATIONS_RESET_ARE_YOU_SURE_CONFIRM:
				 printf("MENU_COMMUNICATIONS_RESET_ARE_YOU_SURE_CONFIRM\r\n");
				 display_menu("Are you", DISPLAY_COLOR, "sure?", DISPLAY_COLOR);
				break;
			case MENU_COMMUNICATIONS_RESET_CONFIRMED:
				 printf("MENU_COMMUNICATIONS_RESET_CONFIRMED\r\n");
				 display_menu("Reset", DISPLAY_COLOR, "confirmed!", DISPLAY_COLOR);
				 manaully_reset_ssid_pass_enable = 1;
				//esp_wifi_start();
				 vTaskDelay(5000);

				 erase_storage_all(); // erase flash..

				 esp_restart();
				break;
#endif

            } // end of

            update_display = false;
        }

        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();

    // return new mode
    return next_mode;
}

static app_mode_t menu_settings(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

    uint8_t m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
    bool update_display = true;
    time_t btn_power_press_ms = 0;

    bool is_settings_changed = false;
    int *temp_hysteresis_c = &(data->settings.temperature_hysteresis_celsius);
    int *temp_hysteresis_f = &(data->settings.temperature_hysteresis_fahrenheit);

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!exit) {
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else { // any of the buttons was pressed
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_settings) {
                        case MENU_SETTINGS_TEMPERATURE_UNIT:
#ifdef Menu_dayLight_option
                         case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE:
#endif
                         // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK:

                        case MENU_SETTINGS_PILOT_LIGHT:
                        case MENU_SETTINGS_NIGHT_LIGHT:
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS:
#ifdef HeaterUnderReapir
                        case  MENU_SETTINGS_HEATER_UNDER_REPAIR:
#endif
                            exit = true;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE:
                            m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
                            break;
#ifdef Menu_dayLight_option
						case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN:
								m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE;
								break;
#endif
					// It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK_EN:
                            m_settings = MENU_SETTINGS_CHILD_LOCK;
                            break;


                        case MENU_SETTINGS_PILOT_LIGHT_EN:
                            m_settings = MENU_SETTINGS_PILOT_LIGHT;
                            break;
                        case MENU_SETTINGS_NIGHT_LIGHT_CFG:
                            m_settings = MENU_SETTINGS_NIGHT_LIGHT;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE_WARNING:
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE:
                            m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS;
                            break;
#ifdef HeaterUnderReapir
                        case  MENU_SETTINGS_HEATER_UNDER_REPAIR_EN:
                        	   m_settings = MENU_SETTINGS_HEATER_UNDER_REPAIR ;
                        	    break;
#endif

                        }

                        update_display = true;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        switch (m_settings) {
                        case MENU_SETTINGS_TEMPERATURE_UNIT:
#ifdef Menu_dayLight_option
                         m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE;
#else
                       //  m_settings = MENU_SETTINGS_CHILD_LOCK;  // working // 1st change before day light added
                         m_settings = MENU_SETTINGS_PILOT_LIGHT;
 #endif
                            break;

                            // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK:
                            m_settings = MENU_SETTINGS_PILOT_LIGHT;
                            break;
#ifdef Menu_dayLight_option
						case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE:
							m_settings = MENU_SETTINGS_PILOT_LIGHT;
							break;
#endif
                        case MENU_SETTINGS_PILOT_LIGHT:
                            m_settings = MENU_SETTINGS_NIGHT_LIGHT;
                            break;
                        case MENU_SETTINGS_NIGHT_LIGHT:
                            m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS:
#ifdef HeaterUnderReapir
                            m_settings = MENU_SETTINGS_HEATER_UNDER_REPAIR;
#else
                            m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
#endif
                            break;
#ifdef HeaterUnderReapir
                        case MENU_SETTINGS_HEATER_UNDER_REPAIR:
                        	  m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
                        	  break;
#endif
                        case MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE:
                            is_settings_changed = true;
                            manaully_Temp_unit_change =1;  // New added for event for manually temp unit change..24Nov2020
                            // Fahrenheit <--> Celsius
                            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
                                data->settings.temperature_unit = TEMP_UNIT_FAHRENHEIT;
                            else
                                data->settings.temperature_unit =  TEMP_UNIT_CELSIUS;
                            break;

                            // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK_EN:
                            is_settings_changed = true;
                            // Enable <--> Disable
                            data->settings.is_child_lock_en = !data->settings.is_child_lock_en;
                           // manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS
                            break;
#ifdef Menu_dayLight_option
                    	case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN:
								// m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN;
								data->daylightSaving = !data->daylightSaving;
								printf("data->daylightSaving  %d \n", data->daylightSaving);
								// Save pending //
								set_integer_to_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int)app_data->daylightSaving);
								manually_day_light_on_off_change_enable = 1;
								break;
#endif
#ifdef HeaterUnderReapir
                        case  MENU_SETTINGS_HEATER_UNDER_REPAIR_EN:
                        	  heater_underControl_status = !heater_underControl_status;
                        	  manually_put_heater_under_repair_enable = 1;
                        	  if(heater_underControl_status == 1)
                        	  {  device_health_status = DEVICE_HEATER_UNDER_REPAIR;
                        	     manually_put_heater_under_repair_status_for_malfunctionMonitor = 0;
                        	  }
                        	  else
                        	  {   device_health_status = DEVICE_HEALTH_OK; manually_put_heater_under_repair_status_for_malfunctionMonitor = 1; }
                        	  printf("MENU_SETTINGS_HEATER_UNDER_REPAIR_EN status : %d device_health_status %d \n", heater_underControl_status ,device_health_status );
                        	  break;
#endif

                        case MENU_SETTINGS_PILOT_LIGHT_EN:
                            is_settings_changed = true;
                            // Enable <--> Disable
                            data->settings.is_dim_pilot_light_en = !data->settings.is_dim_pilot_light_en;
                            break;
                        case MENU_SETTINGS_NIGHT_LIGHT_CFG:
                            is_settings_changed = true;
                            // Auto <--> Off
                            data->settings.is_night_light_auto_brightness_en = !data->settings.is_night_light_auto_brightness_en;
                            manaully_night_Light_State_change= 1;  // New Added for manaully_night_Light_State_change notification to AWS
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE:
                            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
                                if (*temp_hysteresis_c < TEMPERATURE_HYSTERESIS_CELSIUS_MAX) {
                                    is_settings_changed = true;
                                    *temp_hysteresis_c += 1;
                                    // set the other unit
//                                    *temp_hysteresis_f = celsius_to_fahr(*temp_hysteresis_c);
                                }
                            } else {
                                if (*temp_hysteresis_f < TEMPERATURE_HYSTERESIS_FAHRENHEIT_MAX) {
                                    is_settings_changed = true;
                                    *temp_hysteresis_f += 1;
                                    // set the other unit
//                                    *temp_hysteresis_c = fahr_to_celsius(*temp_hysteresis_f);
                                }
                            }
                            break;
                        }
                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        switch (m_settings) {
                        case MENU_SETTINGS_TEMPERATURE_UNIT:
#ifdef HeaterUnderReapir
                       	     m_settings = MENU_SETTINGS_HEATER_UNDER_REPAIR;
#else
                            m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS;
#endif
                            break;
#ifdef HeaterUnderReapir
                    case  MENU_SETTINGS_HEATER_UNDER_REPAIR:
                          m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS;
            	          break;
#endif

            	          // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK:
                            m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
                            break;
#ifdef Menu_dayLight_option
						case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE:
							m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
							break;
#endif
                        case MENU_SETTINGS_PILOT_LIGHT:
                           // m_settings = MENU_SETTINGS_CHILD_LOCK;
#ifdef Menu_dayLight_option
                        	m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE;
#else
                         m_settings = MENU_SETTINGS_TEMPERATURE_UNIT;
                        //	m_settings = MENU_SETTINGS_CHILD_LOCK;
#endif
                            break;

                        case MENU_SETTINGS_NIGHT_LIGHT:
                            m_settings = MENU_SETTINGS_PILOT_LIGHT;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS:
                            m_settings = MENU_SETTINGS_NIGHT_LIGHT;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE:
                            is_settings_changed = true;
                            manaully_Temp_unit_change =1;  // New added for event for manually temp unit change..24Nov2020
                            // Fahrenheit <--> Celsius
                            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
                                data->settings.temperature_unit = TEMP_UNIT_FAHRENHEIT;
                            else
                                data->settings.temperature_unit =  TEMP_UNIT_CELSIUS;
                            break;

                            // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK_EN:
                            is_settings_changed = true;
                            // Enable <--> Disable
                            data->settings.is_child_lock_en = !data->settings.is_child_lock_en;
                           // manaully_child_Lock_State_change = 1;  // New Added for manaully_child_Lock_State_change notification to AWS
                            break;

#ifdef Menu_dayLight_option
					case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN:
						// m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN;
						  data->daylightSaving = !data->daylightSaving;
						// Save pending //
						set_integer_to_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int)app_data->daylightSaving);
						manually_day_light_on_off_change_enable = 1;
						break;
#endif

#ifdef HeaterUnderReapir
                        case  MENU_SETTINGS_HEATER_UNDER_REPAIR_EN:
                        	  heater_underControl_status = !heater_underControl_status;
                        	  manually_put_heater_under_repair_enable = 1;
							  if(heater_underControl_status == 1)
							   { device_health_status = DEVICE_HEATER_UNDER_REPAIR; manually_put_heater_under_repair_status_for_malfunctionMonitor = 0;}
							  else
							   {  device_health_status = DEVICE_HEALTH_OK; manually_put_heater_under_repair_status_for_malfunctionMonitor = 1; }
                              printf("MENU_SETTINGS_HEATER_UNDER_REPAIR_EN status : %d device_health_status %d \n", heater_underControl_status ,device_health_status );
                        	  break;
#endif
                        case MENU_SETTINGS_PILOT_LIGHT_EN:
                            is_settings_changed = true;
                            // Enable <--> Disable
                            data->settings.is_dim_pilot_light_en = !data->settings.is_dim_pilot_light_en;
                            break;
                        case MENU_SETTINGS_NIGHT_LIGHT_CFG:
                            is_settings_changed = true;
                            // Auto <--> Off
                            data->settings.is_night_light_auto_brightness_en = !data->settings.is_night_light_auto_brightness_en;
                            manaully_night_Light_State_change= 1;  // New Added for manaully_night_Light_State_change notification to AWS
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE:
                            if (data->settings.temperature_unit == TEMP_UNIT_CELSIUS) {
                                if (*temp_hysteresis_c > TEMPERATURE_HYSTERESIS_CELSIUS_MIN) {
                                    is_settings_changed = true;
                                    *temp_hysteresis_c -= 1;
                                    // set the other unit
//                                    *temp_hysteresis_f = celsius_to_fahr(*temp_hysteresis_c);
                                }
                            } else {
                                if (*temp_hysteresis_f > TEMPERATURE_HYSTERESIS_FAHRENHEIT_MIN) {
                                    is_settings_changed = true;
                                    *temp_hysteresis_f -= 1;
                                    // set the other unit
//                                    *temp_hysteresis_c = fahr_to_celsius(*temp_hysteresis_f);
                                }
                            }
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_settings) {
                        case MENU_SETTINGS_TEMPERATURE_UNIT:
                            m_settings = MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE;
                            break;

                            // It was commented for earlier-- uncommented on 16MArch 2021 again..
                        case MENU_SETTINGS_CHILD_LOCK:
                            m_settings = MENU_SETTINGS_CHILD_LOCK_EN;
                            break;

#ifdef Menu_dayLight_option
					case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE:
						 m_settings = MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN;
						break;
#endif
#ifdef HeaterUnderReapir
                      case  MENU_SETTINGS_HEATER_UNDER_REPAIR:
                              m_settings = MENU_SETTINGS_HEATER_UNDER_REPAIR_EN;
                        	 break;
#endif
                        case MENU_SETTINGS_PILOT_LIGHT:
                            m_settings = MENU_SETTINGS_PILOT_LIGHT_EN;
                            break;
                        case MENU_SETTINGS_NIGHT_LIGHT:
                            m_settings = MENU_SETTINGS_NIGHT_LIGHT_CFG;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS:
                            m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE_WARNING;
                            break;
                        case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE_WARNING:
                            m_settings = MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE;
                            break;
                        }
                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update the display
        if (update_display) {
            printf("m_settings=%d\r\n", m_settings);

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            switch (m_settings) {
            case MENU_SETTINGS_TEMPERATURE_UNIT:
                printf("MENU_SETTINGS_TEMPERATURE_UNIT\r\n");
                display_menu("Temp Unit", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;

                // It was commented for earlier-- uncommented on 16MArch 2021 again..
            case MENU_SETTINGS_CHILD_LOCK:
                printf("MENU_SETTINGS_CHILD_LOCK\r\n");
                display_menu("Child Lock", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
#ifdef Menu_dayLight_option
				case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE:
					printf("MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE\r\n");
					// display_menu("Day Light", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
					display_menu("DST", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
					break;
#endif
            case MENU_SETTINGS_PILOT_LIGHT:
                printf("MENU_SETTINGS_PILOT_LIGHT\r\n");
                display_menu("Auto Dim", DISPLAY_COLOR, "Pilot Light", DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_TEMPERATURE_HYSTERESIS:
                printf("MENU_SETTINGS_TEMPERATURE_HYSTERESIS\r\n");
                display_menu("Hysteresis", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_NIGHT_LIGHT:
                printf("MENU_SETTINGS_NIGHT_LIGHT\r\n");
                display_menu("Night", DISPLAY_COLOR, "Light", DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE:
                printf("MENU_SETTINGS_TEMPERATURE_UNIT_CHANGE\r\n");
                display_menu(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? "Metric(°C)" : "US(°F)", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;

                // It was commented for earlier-- uncommented on 16MArch 2021 again..
            case MENU_SETTINGS_CHILD_LOCK_EN:
                printf("MENU_SETTINGS_CHILD_LOCK_EN\r\n");
                display_menu(data->settings.is_child_lock_en ? "Enabled" : "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;

#ifdef Menu_dayLight_option
             case MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN:
					printf("MENU_SETTINGS_DAY_LIGHT_ON_OFF_CHANGE_EN\r\n");
					display_menu(data->daylightSaving ? "ON" : "OFF", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
					printf("data->daylightSaving  %d \n", data->daylightSaving);
					break;
#endif
#ifdef HeaterUnderReapir
             case  MENU_SETTINGS_HEATER_UNDER_REPAIR:
					printf("MENU_SETTINGS_HEATER_UNDER_REPAIR \r\n");
					display_menu("Under", DISPLAY_COLOR, "Repair", DISPLAY_COLOR);
                   	 break;
             case  MENU_SETTINGS_HEATER_UNDER_REPAIR_EN:
					printf("MENU_SETTINGS_HEATER_UNDER_REPAIR_EN \r\n");
      		    	display_menu(heater_underControl_status ? "ON" : "OFF", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                   	 break;
#endif
            case MENU_SETTINGS_PILOT_LIGHT_EN:
                printf("MENU_SETTINGS_PILOT_LIGHT_EN\r\n");
                display_menu(data->settings.is_dim_pilot_light_en ? "Enabled" : "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE_WARNING:
                printf("MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE_WARNING\r\n");
                display_hysteresis_setting_warning(DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE:
                printf("MENU_SETTINGS_TEMPERATURE_HYSTERESIS_CHANGE\r\n");
                display_temperature(data->settings.temperature_unit == TEMP_UNIT_CELSIUS ? *temp_hysteresis_c : *temp_hysteresis_f, DISPLAY_COLOR);
                break;
            case MENU_SETTINGS_NIGHT_LIGHT_CFG:
                printf("MENU_SETTINGS_NIGHT_LIGHT_CFG\r\n");
                display_menu((data->settings.is_night_light_auto_brightness_en) ? "Auto" : "Off", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            }
            update_display = false;
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();

    // save settings if changed
    if (is_settings_changed)
        set_data_to_storage(STORAGE_KEY_SETTINGS, (void *) &data->settings, sizeof(settings_t));

    // return new mode
    return next_mode;
}
bool get_heater_under_repair_status(void)
{	return heater_underControl_status;}

static app_mode_t menu_display_settings(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

    uint8_t m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS;
    bool update_display = true;
    time_t btn_power_press_ms = 0;

    bool is_display_settings_changed = false;
    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    printf("data->display_settings.auto_screen_off_delay_sec  %d\n ",data->display_settings.auto_screen_off_delay_sec );

    while (!exit) {

        printf("data->display_settings.auto_screen_off_delay_sec  %d\n ",data->display_settings.auto_screen_off_delay_sec );
    	if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else {
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_display_settings) {
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS:
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF:
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY:
                            exit = true;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL:
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_EN:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY_CHANGE:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL_CHANGE:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO_EN:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO;
                            break;
                        }

                        update_display = true;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_UP_STAT)) != (prev_btn & (1 << BUTTON_UP_STAT))) { // up button toggles
                    if ((*btn >> BUTTON_UP_STAT) & 0x01) { // unpressed
                        switch (m_display_settings) {
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_EN:
                            is_display_settings_changed = true;
                            data->display_settings.is_auto_screen_off_en = !data->display_settings.is_auto_screen_off_en;
                            t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY_CHANGE:
                            is_display_settings_changed = true;
                            if (data->display_settings.auto_screen_off_delay_sec < AUTO_SCREEN_OFF_DELAY_SEC_MAX)
                                data->display_settings.auto_screen_off_delay_sec += AUTO_SCREEN_OFF_DELAY_SEC_INCREMENT;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL_CHANGE:
                            is_display_settings_changed = true;
                            if (data->display_settings.display_brightness < DISPLAY_BRIGHTNESS_MAX) {
                                data->display_settings.display_brightness += DISPLAY_BRIGHTNESS_INCREMENT;
                                if (data->display_settings.display_brightness > DISPLAY_BRIGHTNESS_MAX) {
                                    data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_MAX;
                                }
                             }
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO_EN:
                            is_display_settings_changed = true;
                            data->display_settings.is_auto_display_brightness_en = !data->display_settings.is_auto_display_brightness_en;
                            if (data->display_settings.is_auto_display_brightness_en == false) {
                                data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_DEF;
                            }
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_DOWN_STAT)) != (prev_btn & (1 << BUTTON_DOWN_STAT))) { // down button toggles
                    if ((*btn >> BUTTON_DOWN_STAT) & 0x01) { // unpressed
                        switch (m_display_settings) {
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_EN:
                            is_display_settings_changed = true;
                            data->display_settings.is_auto_screen_off_en = !data->display_settings.is_auto_screen_off_en;
                            t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY_CHANGE:
                            is_display_settings_changed = true;
                            if (data->display_settings.auto_screen_off_delay_sec > AUTO_SCREEN_OFF_DELAY_SEC_MIN)
                                data->display_settings.auto_screen_off_delay_sec -= AUTO_SCREEN_OFF_DELAY_SEC_INCREMENT;
                            break;
                       case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL_CHANGE:
                            is_display_settings_changed = true;
                            if (data->display_settings.display_brightness > DISPLAY_BRIGHTNESS_MIN) {
                                data->display_settings.display_brightness -= DISPLAY_BRIGHTNESS_INCREMENT;
                                if (data->display_settings.display_brightness < DISPLAY_BRIGHTNESS_MIN) {
                                    data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_MIN;
                                }
                            }
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO_EN:
                            is_display_settings_changed = true;
                            data->display_settings.is_auto_display_brightness_en = !data->display_settings.is_auto_display_brightness_en;
                            if (data->display_settings.is_auto_display_brightness_en == false) {
                                data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_DEF;
                            }
                            break;
                        }

                        update_display = true;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_display_settings) {
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_EN;
                            break;
                        case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY:
                            m_display_settings = MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY_CHANGE;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL:
                            // if this is entered when auto display brightness is enabled, disable auto display brightness and set brightness to default
                            if (data->display_settings.is_auto_display_brightness_en) {
                                is_display_settings_changed = true;
                                data->display_settings.is_auto_display_brightness_en = false;
                                data->display_settings.display_brightness = DISPLAY_BRIGHTNESS_DEF;
                            }
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL_CHANGE;
                            break;
                        case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO:
                            m_display_settings = MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO_EN;
                            break;
                        }

                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }

        // update the display
        if (update_display) {
            printf("m_display_settings=%d\r\n", m_display_settings);

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            switch (m_display_settings) {
            case MENU_DISPLAY_SETTINGS_BRIGHTNESS:
                display_menu("Brightness", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF:
                display_menu("Auto", DISPLAY_COLOR, "Screen Off", DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY:
                display_menu("Delay", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL:
                display_menu("Manual", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO:
                display_menu("Auto", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_EN:
                display_menu(data->display_settings.is_auto_screen_off_en ? "Enabled" : "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_AUTO_SCREEN_OFF_DELAY_CHANGE:
                display_screen_timeout(data->display_settings.auto_screen_off_delay_sec, DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_BRIGHTNESS_MANUAL_CHANGE:
                display_brightness_val(data->display_settings.display_brightness, DISPLAY_COLOR);
                break;
            case MENU_DISPLAY_SETTINGS_BRIGHTNESS_AUTO_EN:
                display_menu(data->display_settings.is_auto_display_brightness_en ? "Enabled" : "Disabled", DISPLAY_COLOR, NULL, !DISPLAY_COLOR);
                break;
    	    }
	        update_display = false;
	    }
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    // exit with display on
    if (screen_off)
       display_on();

    if (is_display_settings_changed)
        set_data_to_storage(STORAGE_KEY_DISPLAY_SETTINGS, (void *) &data->display_settings, sizeof(display_settings_t));

    // return new mode
    return next_mode;
}

static app_mode_t menu_update(app_data_t *data) {
    int *btn = &(data->button_status);
    int prev_btn = *btn;

    // when changing mode, this task should be completed before starting the next mode
    bool exit = false;
    app_mode_t next_mode = data->mode;

    uint8_t m_update = MENU_UPDATE_STATUS;
    bool update_display = true;
    time_t btn_power_press_ms = 0;

    bool screen_off = false;
    time_t t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    //TODO: bool fw_update_available = is_fw_update_available();

    while (!exit) {
        if (*btn == prev_btn) {
            if (data->display_settings.is_auto_screen_off_en) {
                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_screen_on_ms) >= (data->display_settings.auto_screen_off_delay_sec * 1000)) {
                    display_off();
                    screen_off = true;
                }
            }

            if (!screen_off) {
                if (!((*btn >> BUTTON_POWER_BACK_STAT) & 0x01)) { // power button is pressed
                    int cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if ((cur_ms - btn_power_press_ms) >= HEATER_OFF_LONG_PRESS_DUR_MS) {
                        next_mode = APP_MODE_STANDBY;
                        // new added for stand by mode by manually _30Nov2020
                       heater_On_Off_state_by_command = 0;    	heater_On_Off_state_by_command_ExistFromStandByMode = 0; // Till here
                        exit = true;
                        btn_power_press_ms = cur_ms;
                    }
                }
            }
        } else {
            if (data->display_settings.is_auto_screen_off_en) {
                t_screen_on_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }

            if (screen_off) {
                // enable screen only if all buttons are unpressed
                if (((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) && ((*btn >> BUTTON_UP_STAT) & 0x01)
                    && ((*btn >> BUTTON_DOWN_STAT) & 0x01) && ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01)) {
                    screen_off = false;
                    display_on();
                }
            } else {
                if ((*btn & (1 << BUTTON_POWER_BACK_STAT)) != (prev_btn & (1 << BUTTON_POWER_BACK_STAT))) { // power button toggles
                    if ((*btn >> BUTTON_POWER_BACK_STAT) & 0x01) { // unpressed
                        switch (m_update) {
                        case MENU_UPDATE_TRIGGER_UPDATE:
                            exit = true;
                            break;
                        }

                        update_display = true;
                    } else {
                        btn_power_press_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    }
                } else if ((*btn & (1 << BUTTON_TIMER_FORWARD_STAT)) != (prev_btn & (1 << BUTTON_TIMER_FORWARD_STAT))) { // timer button toggles
                    if ((*btn >> BUTTON_TIMER_FORWARD_STAT) & 0x01) { // unpressed
                        switch (m_update) {
                        case MENU_UPDATE_STATUS:
                            m_update = MENU_UPDATE_TRIGGER_UPDATE;
                            break;
                        case MENU_UPDATE_TRIGGER_UPDATE:
                            //TODO: start_fw_update();
                            m_update = MENU_UPDATE_ONGOING_UPDATE;
                        }

                        update_display = true;
                    }
                }
            }
            prev_btn = *btn;
        }
        // update the display
        if (update_display) {
            update_display = false;
            printf("m_update=%d\r\n", m_update);

            display_clear_screen();
            // display connection status indication if connected
            if (data->is_connected)
                display_wifi_icon(DISPLAY_COLOR);

            switch (m_update) {
            case MENU_UPDATE_STATUS:
            #if 0 //TODO: 
                if (fw_update_available) {
                    //TODO: display_update_status(1, get_kaa_ota_release_key(), DISPLAY_COLOR);
                    vTaskDelay(DISPLAY_UPDATE_STATUS_FLASH_DURATION_MS / portTICK_RATE_MS);
                    m_update = MENU_UPDATE_TRIGGER_UPDATE;
                    update_display = true;
                } else {
            #endif
                    display_update_status(0, -1, DISPLAY_COLOR);
                    vTaskDelay(DISPLAY_UPDATE_STATUS_FLASH_DURATION_MS / portTICK_RATE_MS);
                    exit = true;
                //}
                break;
            case MENU_UPDATE_TRIGGER_UPDATE:
                display_update_trigger_msg(DISPLAY_COLOR);
                break;
            case MENU_UPDATE_ONGOING_UPDATE:
                display_update_installing_msg(DISPLAY_COLOR);
                break;
            }
            }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
    // exit with display on
    if (screen_off)
       display_on();
    // return new mode
    return next_mode;
}


static void temp_sensor_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    bool *currentHeaterState = &(data->lastHeaterState);

    int *ambient_temp_c = &(data->ambient_temperature_celsius);
    int *temp_offset_c = &(data->ambient_temperature_offset_celsius);
    int tempInFehrenniete = 0;
    int Prev_SetTemp = 0;
    int *temp_hysteresis_c = &(data->settings.temperature_hysteresis_celsius);
    int *temp_hysteresis_f = &(data->settings.temperature_hysteresis_fahrenheit);
    int *target_temp_c = &(data->manual_temperature_celsius), *target_temp_f = &(data->manual_temperature_fahrenheit);
    unsigned char hysterisFlag = 0;
    int prevAmbientTemp_Fahraneite = 0;
    int prevAmbientTemp_Calcius = 0;  int lprevAmbientTempForEventTrigger = 0;
   // unsigned char *TimerIntervalThresholdOffset = &(data-> TimerIntervalThresholdOffset);
	#define TIMER_INTERVAL_THRESHOLD_OFFSET 2 // 30 Minute original for logic implementation
    lprevAmbientTempForEventTrigger = *ambient_temp_c;

    int count = 0, lint_temp = 0; float lf_temp =0 ;

    while(1) {

    	//  *ambient_temp_c = tempsensor_get_temperature() + *temp_offset_c;  // Original Line commented on 24Dec2020

// #define TEST_AMBINET_TEMP
#ifdef TEST_AMBINET_TEMP
         count++;
         count = 2;
    	*ambient_temp_c  =  count;
//    	lf_temp = (float)celsius_to_fahr(count);
//    	lf_temp =round(lf_temp);
//    	lint_temp = lf_temp;

    	lint_temp = valueRoundOff(*ambient_temp_c, CONVERT_C_TO_F);

    	if(count > 38)
    	{	count = 0;  vTaskDelay(10000); }

    	printf("\n *ambient_temp_c %d lf_temp: %f lint_temp %d \n ",*ambient_temp_c, lf_temp,lint_temp);

#else
    	*ambient_temp_c = tempsensor_get_temperature() + *temp_offset_c;  // Original Line commented on 24Dec2020
#endif

        if( *ambient_temp_c <=0) // New added for if Temperature is less that Zero, display should display Zero.added on 11Dec 2020.
        	*ambient_temp_c = 0;

        vTaskDelay(TEMP_SENSOR_READ_INTERVAL_MS / portTICK_RATE_MS); // Original
      //  vTaskDelay(20000);  // Testing

    }// end of while
} // end of static void temp_sensor_task(void *param)

static void light_sensor_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    int val;
    int *ambient_light = &(data->ambient_light);

    while(1) {
        val = lightsensor_get_val();
        *ambient_light = (val > 100) ? 100 : val;
       // printf("ambient_light=%d\r\n", *ambient_light);
        vTaskDelay(LIGHT_SENSOR_READ_INTERVAL_MS / portTICK_RATE_MS);
    } // end of while
}

// Original function
//#if 1
//static void pilot_light_task(void *param) {
//    app_data_t *data = (app_data_t *) param;
//    app_mode_t *mode = &(data->mode);
//    bool on = false;
//    int *ambient_light = &(data->ambient_light), prev_ambient_light = -1;
//    int *pilot_br = &(data->pilot_light_brightness);
//    time_t t_set_brightness_ms = 0;
//    bool prev_pilot_light_en_settings = false;
//
//    while (1) {
//        if (*mode == APP_MODE_STANDBY) {
//            if (on) {
//                pilot_light_off();
//            }
//            on = false;
//        } else {
//            if (data->settings.is_dim_pilot_light_en) {
//                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_set_brightness_ms) >= DIM_PILOT_LIGHT_UPDATE_INTERVAL_MS) {
//                    // set brightness based on ambient light
//                    if (*ambient_light != prev_ambient_light) {
//                        *pilot_br = *ambient_light * (AUTO_DIM_PILOT_LIGHT_MAX_BRIGHTNESS - AUTO_DIM_PILOT_LIGHT_MIN_BRIGHTNESS) / AUTO_DIM_PILOT_LIGHT_MAX_BRIGHTNESS + AUTO_DIM_PILOT_LIGHT_MIN_BRIGHTNESS;
//                        pilot_light_set_brightness((uint8_t)*pilot_br);
//                        t_set_brightness_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
//                       // printf("pilot light br=%d\r\n", *pilot_br);
//                    }
//                }
//                prev_pilot_light_en_settings = true;
//            } else {
//                if (!on || prev_pilot_light_en_settings) {
//                    pilot_light_on();
//                }
//                prev_pilot_light_en_settings = false;
//            }
//            on = true;
//        }
//        vTaskDelay(1 / portTICK_RATE_MS);
//    }
//}
//#endif

//Testing ...
#if 1
static void pilot_light_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    app_mode_t *mode = &(data->mode);
    bool on = false;
    int *ambient_light = &(data->ambient_light), prev_ambient_light = -1;
    int *pilot_br = &(data->pilot_light_brightness);
    time_t t_set_brightness_ms = 0;
    bool prev_pilot_light_en_settings = false;

    unsigned char lchLed_ON_OFF = 0;
     // pingDeviceOnFlag =1; // Only for Testing logic 20 Sec

    while (1) { // printf(" pilot_light_task Flag \n ");

    	if(pingDeviceOnFlag == 1){ // printf(" pairing ON Flag \n ");

		  int ping_cur_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
				if ((ping_cur_ms - ping_Dur_ms) >= 20000)
				{
					ping_Dur_ms = ping_cur_ms;
					ping_TwentySecFirstIterationOver = 1;  printf("first time.ping Device.. \n");
				  }

				if(ping_TwentySecFirstIterationOver  == 1)
				{   ping_TwentySecOver ++;ping_TwentySecFirstIterationOver = 0;
				    if(ping_TwentySecOver == 2){
					pingDeviceOnFlag = 0; ping_TwentySecOver = 0; printf("20Secover.ping Device.. \n");}
				}
          // pairOnPilotLedBlinking();
           lchLed_ON_OFF = !lchLed_ON_OFF;
       	if(lchLed_ON_OFF ==1)
       	  { pilot_light_on(); }
       	else { 	pilot_light_off();}

//          display_clear_screen();
//        //   display_menu("Firm_ver", DISPLAY_COLOR, fw_version, DISPLAY_COLOR);
//         display_menu("ping", DISPLAY_COLOR, "Device", DISPLAY_COLOR);
         printf(".ping Device.. \n");
    	}
       	else{
       		        if (*mode == APP_MODE_STANDBY) {
       		            if (on) {
       		                pilot_light_off();
       		            }
       		            on = false;
       		        } else {
       		            if (data->settings.is_dim_pilot_light_en) {
       		                if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_set_brightness_ms) >= DIM_PILOT_LIGHT_UPDATE_INTERVAL_MS) {
       		                    // set brightness based on ambient light
       		                    if (*ambient_light != prev_ambient_light) {
       		                        *pilot_br = *ambient_light * (AUTO_DIM_PILOT_LIGHT_MAX_BRIGHTNESS - AUTO_DIM_PILOT_LIGHT_MIN_BRIGHTNESS) / AUTO_DIM_PILOT_LIGHT_MAX_BRIGHTNESS + AUTO_DIM_PILOT_LIGHT_MIN_BRIGHTNESS;
       		                        pilot_light_set_brightness((uint8_t)*pilot_br);
       		                        t_set_brightness_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
       		                       // printf("pilot light br=%d\r\n", *pilot_br);
       		                    }
       		                }
       		                prev_pilot_light_en_settings = true;
       		            } else {
       		                if (!on || prev_pilot_light_en_settings) {
       		                    pilot_light_on();
       		                }
       		                prev_pilot_light_en_settings = false;
       		            }
       		            on = true;
       		        }

    	}// else of if

      //  vTaskDelay(1 / portTICK_RATE_MS);  // Original
        vTaskDelay(1000);
    }
}
#endif


static void night_light_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    int *ambient_light = &(data->ambient_light), prev_ambient_light = -1;
    int *nlight_cfg = &(data->night_light_cfg);
    bool *nlight_auto_en = &(data->settings.is_night_light_auto_brightness_en);
    bool prev_nlight_auto_en = !(*nlight_auto_en);
    time_t t_set_brightness_ms = 0;

    while (1) {
        if (*nlight_auto_en) {
            // update only by interval
            if (!prev_nlight_auto_en || ((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_set_brightness_ms) >= NIGHT_LIGHT_UPDATE_INTERVAL_MS) {
                // set brightness based on ambient light
                if (*ambient_light != prev_ambient_light) {
                    int nlight_br;
                    if (*ambient_light >= NIGHT_LIGHT_BRIGHTNESS_OFF_THS) {
                        nlight_br = 0;
                        printf("Zero nlight_br \n");
                    } else {
                    // brighter in dark
                        nlight_br = NIGHT_LIGHT_BRIGHTNESS_MAX - (*ambient_light * (NIGHT_LIGHT_BRIGHTNESS_MAX - NIGHT_LIGHT_BRIGHTNESS_MIN) / NIGHT_LIGHT_BRIGHTNESS_MAX + NIGHT_LIGHT_BRIGHTNESS_MIN);
                    }

                     nlight_br_TestingInSynchPacket = nlight_br; // Added only for Testing..

                   // *nlight_cfg = 0xFFFFFF;
                //     nlight_br = 76 ;  // Added for Testing // 77 bright will be less // 99 brightness will be high
//                    int r_br = nlight_br * GET_LED_R_VAL(*nlight_cfg) / 100;  // Exsiting logic
//                    int g_br = nlight_br * GET_LED_G_VAL(*nlight_cfg) / 100;
//                    int b_br = nlight_br * GET_LED_B_VAL(*nlight_cfg) / 100;

                    float r_br = nlight_br * (GET_LED_R_VAL(*nlight_cfg)/2.6) / 100;  // Exsiting logic
                    float g_br = nlight_br * (GET_LED_G_VAL(*nlight_cfg)/2.6) / 100;
                    float b_br = nlight_br * (GET_LED_B_VAL(*nlight_cfg)/2.6) / 100;

//                    float r_br =  GET_LED_R_VAL(*nlight_cfg)/2.6 ;   // New logic with out light sensor calculation.
//                    float g_br =  GET_LED_G_VAL(*nlight_cfg)/2.6;
//                    float b_br =  GET_LED_B_VAL(*nlight_cfg)/2.6;

                    // set night light color and brightness
                	// printf("night_light_task data->night_light_cfg %d \n ",data->night_light_cfg);
                	// printf("app_data->night_light_cfg %d \n ",app_data->night_light_cfg);
                   //  printf("\n\n night light %d %d  %d %d %d %d %d\r\n\n", *ambient_light, *nlight_auto_en,*nlight_cfg, nlight_br,r_br, g_br, b_br);

                    if(rgb_led_state == 1)
                    {   // night_light_set_br(r_br, g_br, b_br);  // Original Line..
                        night_light_set_br((int)r_br, (int)g_br, (int)b_br);  // Original Line..
                    	// night_light_set_br(99, 0, 0); //  night_light_set_br(0, 255, 0);
                        printf("\n\n night light %d %d  %d %d %d %d %d\r\n\n", *ambient_light, *nlight_auto_en,*nlight_cfg, nlight_br,(int)r_br,(int) g_br, (int)b_br);
                       // printf("Led ON in night light task function \n ");
                    }
                    if(rgb_led_state == 0)
                    {     night_light_off();
                         printf("Led OFF in night light enable \n ");
                    }

                   // int temp = 0;
                   // uint_32 temp = 0;
                   // temp = ((b_br | temp) << 16) | (( g_br | temp) <<8) |( r_br | temp);
                  //  printf("\n temp : %d vs nlight_cfg  %d\n ",temp,*nlight_cfg );
                   // night_light_set_br(0, 255, 0);       //  night_light_set_br(50, 255, 50);
                    t_set_brightness_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                   // printf("nlight_br %d \n ",nlight_br);                        // printf("\n\n*ambient_light %d \n ",*ambient_light);         // printf("*nlight_cfg %d \n ",*nlight_cfg);                  // printf("night light %d %d %d\r\n", r_br, g_br, b_br);
                }
            }
            prev_nlight_auto_en = true;
        }// end of if (*nlight_auto_en) {
        else {
#define MODIFIED_LOGIC_RGB_DURING_DISABLE_NIGHT_LIGHT
#ifdef MODIFIED_LOGIC_RGB_DURING_DISABLE_NIGHT_LIGHT
        	// *nlight_cfg = 255255255;

            float r_br_NL_disable = 100 *(GET_LED_R_VAL(*nlight_cfg)/2.6) / 100;  // Exsiting logic
            float g_br_NL_disable = 100 * (GET_LED_G_VAL(*nlight_cfg)/2.6) / 100;
            float b_br_NL_disable = 100 * (GET_LED_B_VAL(*nlight_cfg)/2.6) / 100;

//            float r_br_NL_disable = 99;  // Exsiting logic
//            float g_br_NL_disable = 99;
//            float b_br_NL_disable =99;

            if(rgb_led_state == 1)
			   {   // night_light_set_br(r_br, g_br, b_br);  // Original Line..
				   night_light_set_br((int)r_br_NL_disable, (int)g_br_NL_disable, (int)b_br_NL_disable);  // Original Line..
		         // printf("\n night light disable %d %d  %d %d %d %d\r", *ambient_light, *nlight_auto_en,*nlight_cfg, (int)r_br_NL_disable,(int) g_br_NL_disable, (int)b_br_NL_disable);
			   }
//            else(rgb_led_state == 0)
             else
			   {     night_light_off();
				   //  printf("Led OFF in night light disable \n ");
			   }

              prev_nlight_auto_en = false;
        	 // printf("Night Light mode is disable \n");
#else
            if (prev_nlight_auto_en) {
                night_light_off();
                printf("night light off \n");
            }
            prev_nlight_auto_en = false;
#endif
        }// end of else..
        vTaskDelay(1 / portTICK_RATE_MS);
    }//end of while(1)
}

#if 1
static void display_brightness_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    int *ambient_light = &(data->ambient_light), prev_ambient_light = -1;
    int *display_brightness = &(data->display_settings.display_brightness), prev_display_brightness = -1;
    time_t t_set_brightness_ms = 0;

    while (1) {
        if(data->display_settings.is_auto_display_brightness_en) {
            if (((xTaskGetTickCount() * portTICK_PERIOD_MS) - t_set_brightness_ms) >= DISPLAY_AUTO_BRIGHTNESS_UPDATE_INTERVAL_MS) {
                // set brightness based on ambient light
                if (*ambient_light != prev_ambient_light) {
                    prev_ambient_light = *ambient_light;
                    *display_brightness = *ambient_light;
                    printf("change screen brightness br=%d\r\n", *display_brightness);
                    display_set_brightness(*display_brightness);
                    prev_display_brightness = *display_brightness;
                }

                t_set_brightness_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
        } else {
            if (*display_brightness != prev_display_brightness) {
                printf("change screen brightness\r\n");
                // set screen brightness
                display_set_brightness(*display_brightness);
                prev_display_brightness = *display_brightness;
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}
#endif

uint32_t waiting_results = 0;
esp_err_t ping_results(ping_target_id_t msgType, esp_ping_found * pf) {
    printf("AvgTime:%.1fmS Sent:%d Rec:%d Err:%d min(mS):%d max(mS):%d ", (float)pf->total_time/pf->recv_count, pf->send_count, pf->recv_count, pf->err_count, pf->min_time, pf->max_time );
    printf("Resp(mS):%d Timeouts:%d Total Time:%d\n",pf->resp_time, pf->timeout_count, pf->total_time);
    app_data->internet_conn = pf->send_count > pf->recv_count ? false: true;

    waiting_results = 0;
    return ESP_OK;
}

static void ping_task(void *param) {
    app_data_t *data = (app_data_t *) param;
    app_mode_t *mode = &(data->mode);
    ip4_addr_t ip = {
        .addr = 0x08080808
    }; // 8.8.8.8 or google.com
    uint32_t count = 1; // pings per report
    uint32_t timeout = 1; // seconds to consider timeout
    uint32_t delay = 1; // seconds between pings

    while (*mode == APP_MODE_DEBUG) {
	if (!waiting_results) {
            vTaskDelay(5000 / portTICK_PERIOD_MS);

	    esp_ping_set_target(PING_TARGET_IP_ADDRESS_COUNT, &count, sizeof(uint32_t));
            esp_ping_set_target(PING_TARGET_RCV_TIMEO, &timeout, sizeof(uint32_t));
            esp_ping_set_target(PING_TARGET_DELAY_TIME, &delay, sizeof(uint32_t));
            esp_ping_set_target(PING_TARGET_IP_ADDRESS, &ip.addr, sizeof(uint32_t));
            esp_ping_set_target(PING_TARGET_RES_FN, &ping_results, sizeof(ping_results));
            printf("\nPinging %s\n", inet_ntoa(ip));
            ping_init();
            waiting_results = 1;
        }  vTaskDelay(100 / portTICK_PERIOD_MS); }
    vTaskDelete(NULL);
}

static int wifi_conn_stat(int stat) {
    printf("wifi_conn_stat stat=%d\r\n", stat);
    app_data->is_connected = stat ? true : false;
    display_wifi_icon(stat ? DISPLAY_COLOR : !DISPLAY_COLOR);
    return 0;
}
int app_set_mode(int mode) {
    if (app_data) {
        switch (mode) {
            case APP_MODE_STANDBY:
            case APP_MODE_MANUAL_TEMPERATURE:
            case APP_MODE_TIMER_INCREMENT:
            case APP_MODE_AUTO:
                app_data->mode = mode;
                break;
            default:
                return -1;
        }
        return 0;}    return -1;
}

int app_get_mode(void) {
    if (app_data) {
   // printf("app_get_mode app_data->mode %d",app_data->mode);
    if (app_data->mode == APP_MODE_STANDBY) return (0);
      // working.. condition..
//     else if(app_data->mode == APP_MODE_MANUAL_TEMPERATURE || app_data->mode == APP_MODE_TIMER_INCREMENT || app_data->mode == APP_MODE_AUTO) // || app_data->mode == APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET ||  app_data->mode == APP_MODE_DEBUG || app_data->mode == APP_MODE_MENU)
     else if(app_data->mode == APP_MODE_MANUAL_TEMPERATURE || app_data->mode == APP_MODE_TIMER_INCREMENT || app_data->mode == APP_MODE_AUTO || app_data->mode == APP_MODE_TEMPERATURE_SENSOR_OFFSET_SET ||  app_data->mode == APP_MODE_DEBUG || app_data->mode == APP_MODE_MENU)
   	// else
    return (1);}
    return -1;
}

int app_get_ambient_temp(void) {
//   int temperatureInFehrannite;
//	if (app_data) {
//    	// printf("From app_get_ambient_temp: %d\n", app_data->ambient_temperature_celsius);
////    	 if (app_data->settings.temperature_unit == TEMP_UNIT_CELSIUS)
//          	 return app_data->ambient_temperature_celsius;
////    	 else  //     			app_data->settings.temperature_unit = TEMP_UNIT_FAHRENHEIT
////    	 {
////    		 temperatureInFehrannite = celsius_to_fahr(app_data->ambient_temperature_celsius);
////    	     return temperatureInFehrannite;
////    	 }
//    }
	   int temperatureInFehrannite = 0;
		if (app_data) {
//		    		 temperatureInFehrannite = celsius_to_fahr(app_data->ambient_temperature_celsius);
//	    	 printf("From app_get_ambient_temp: %d temperatureInFehrannite %d\n", app_data->ambient_temperature_celsius, temperatureInFehrannite);
//	         return temperatureInFehrannite;	    }

//	    float lf_temp =0;
//		float lf_temp_roundOff =0;
//		int new_intTemp =0;
//		lf_temp = celsius_to_fahr(app_data->ambient_temperature_celsius);
//		lf_temp_roundOff = round(lf_temp);
//		new_intTemp = lf_temp_roundOff;
		int new_intTemp =0;
		new_intTemp = valueRoundOff(app_data->ambient_temperature_celsius, CONVERT_C_TO_F );

        printf("From app_get_ambient_temp: %d temperatureInFehrannite %d\n", app_data->ambient_temperature_celsius, new_intTemp);

	    return new_intTemp;}
    return 0x80000000;
}

#define FahreneiteLogicSynch_APP_DEV
#ifdef FahreneiteLogicSynch_APP_DEV
int app_set_target_temp(int temp) {
   int temp_c = 0;
   int temp_f = 0 ; float ftemp_f =0;
//   temp_c = temp; printf("temp_c %d\n",temp_c);
//   temp_f = celsius_to_fahr(temp_c); printf("temp_f %d\n",temp_f ); ftemp_f = (float)celsius_to_fahr(temp_c);printf("ftemp_f %f\n",ftemp_f );
   temp_f = temp; printf("temp_f %d\n",temp_f);
   // original
   // temp_c = fahr_to_celsius(temp_f); printf("temp_c %d\n",temp_c);
   // testing

//   float lf_temp =0;
//   float lf_temp_roundOff =0;
//   int n_new_SetTemp =0;
//
//   lf_temp = fahr_to_celsius(temp_f);
//   lf_temp_roundOff = round(lf_temp);
//   n_new_SetTemp = lf_temp_roundOff;
//   temp_c = n_new_SetTemp;

   temp_c = valueRoundOff(temp_f , CONVERT_F_TO_C );

  // temp_c = temp; printf("temp_c %d\n",temp_c);
  // temp_f = celsius_to_fahr(temp_c); printf("temp_f %d\n",temp_f ); ftemp_f = (float)celsius_to_fahr(temp_c);printf("ftemp_f %f\n",ftemp_f );

   if (app_data)
    {
// _celsius
    if (app_data->settings.temperature_unit == TEMP_UNIT_CELSIUS){
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        if (temp_c >= TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN   && temp_c <= TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX)
#else
        if (temp_c >= TEMPERATURE_CELSIUS_VAL_MIN   && temp_c <= TEMPERATURE_CELSIUS_VAL_MAX)  	  // Original Last Firmware Line
#endif
        {
            app_data->manual_temperature_celsius = temp_c; printf("app_data->manual_temperature_celsius %d\n",app_data->manual_temperature_celsius );
            app_data->manual_temperature_fahrenheit = temp_f;  printf("app_data->manual_temperature_fahrenheit %d\n",app_data->manual_temperature_fahrenheit );
            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, app_data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, app_data->manual_temperature_fahrenheit); printf("Temp set by command: %d \n ",temp_c );  return 0;
        }
     }
    else{ // Temperature Value in Fehreanite
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        if (temp_f >= TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN   && temp_f <= TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX)
#else
        if (temp_f >= TEMPERATURE_CELSIUS_VAL_MIN   && temp_f <= TEMPERATURE_CELSIUS_VAL_MAX)  	  // Original Last Firmware Line
#endif
        {
            app_data->manual_temperature_fahrenheit = temp_f; printf("app_data->manual_temperature_fahrenheit: %d\n",app_data->manual_temperature_fahrenheit );
            app_data->manual_temperature_celsius = temp_c; printf("app_data->manual_temperature_celsius %d\n",app_data->manual_temperature_celsius );
            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, app_data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, app_data->manual_temperature_fahrenheit);  printf("Temp set by command in fahrenheit: %d \n ",app_data->manual_temperature_fahrenheit );   return 0;
        } // if
       } // else
    }//  if (app_data)
    return -1;
}
#else
int app_set_target_temp(int temp) {
   int temp_c = 0;
   int temp_f = 0 ; float ftemp_f =0;
   temp_c = temp; printf("temp_c %d\n",temp_c);
   temp_f = celsius_to_fahr(temp_c); printf("temp_f %d\n",temp_f ); ftemp_f = (float)celsius_to_fahr(temp_c);printf("ftemp_f %f\n",ftemp_f );
	if (app_data)
    {
// _celsius
    if (app_data->settings.temperature_unit == TEMP_UNIT_CELSIUS){
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        if (temp_c >= TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MIN   && temp_c <= TEMPERATURE_OPERATING_RANGE_CELSIUS_VAL_MAX)
#else
        if (temp_c >= TEMPERATURE_CELSIUS_VAL_MIN   && temp_c <= TEMPERATURE_CELSIUS_VAL_MAX)  	  // Original Last Firmware Line
#endif
        {
            app_data->manual_temperature_celsius = temp_c; printf("app_data->manual_temperature_celsius %d\n",app_data->manual_temperature_celsius );
            app_data->manual_temperature_fahrenheit = temp_f;  printf("app_data->manual_temperature_fahrenheit %d\n",app_data->manual_temperature_fahrenheit );
            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, app_data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, app_data->manual_temperature_fahrenheit); printf("Temp set by command: %d \n ",temp_c );  return 0;
        }
     }
    else{ // Temperature Value in Fehreanite
#ifdef P_TESTING_TEMP_OPERATING_RANGE_TESTING
        if (temp_f >= TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MIN   && temp_f <= TEMPERATURE_OPERATING_RANGE_FAHRENEIT_VAL_MAX)
#else
        if (temp_f >= TEMPERATURE_CELSIUS_VAL_MIN   && temp_f <= TEMPERATURE_CELSIUS_VAL_MAX)  	  // Original Last Firmware Line
#endif
        {
            app_data->manual_temperature_fahrenheit = temp_f; printf("app_data->manual_temperature_fahrenheit: %d\n",app_data->manual_temperature_fahrenheit );
            app_data->manual_temperature_celsius = temp_c; printf("app_data->manual_temperature_celsius %d\n",app_data->manual_temperature_celsius );
            // update target temperature in flash
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_CELSIUS, app_data->manual_temperature_celsius);
            set_integer_to_storage(STORAGE_KEY_MANUAL_TEMP_FAHRENHEIT, app_data->manual_temperature_fahrenheit);  printf("Temp set by command in fahrenheit: %d \n ",app_data->manual_temperature_fahrenheit );   return 0;
        } // if
       } // else
    }//  if (app_data)
    return -1;
}
#endif
int app_get_target_temp(void) {
    if (app_data) { // Original Lines
    //	if (app_data->settings.temperature_unit == TEMP_UNIT_CELSIUS){
       // printf("From app_get_target_temp in calcius: %d\n", app_data->manual_temperature_celsius);
      //  return app_data->manual_temperature_celsius; }  // Last working one Change on 23Dec2020
       return  app_data->manual_temperature_fahrenheit;} // Only for testing .. Added on 23Dec2020
      //  else{
      //      printf("From app_get_target_temp in fahreneite: %d\n", app_data->manual_temperature_fahrenheit);
      //      return app_data->manual_temperature_fahrenheit;}
    //}
    // printf("From  app_dat not found app_get_target_temp: %d\n", app_data->manual_temperature_celsius);
	 return 0x80000000;  // Original Line
}
int app_set_timer(int timer) {
    if (app_data) {
        if (timer >= 0
            && timer <= TIMER_MAX_VALUE_MINUTES) {
            app_data->current_timer_setting_min = timer;
            app_data->last_timer_setting_min = timer;
            set_integer_to_storage(STORAGE_KEY_LAST_TIMER_SETTING, timer);
            return 0;        }    }    return -1;
}
int app_get_timer(void) {
    if (app_data) {        return app_data->current_timer_setting_min;    }    return 0x80000000;}
int app_activate_child_lock(bool en) {
    if (app_data) {        app_data->is_child_lock_active = en;        return 0;    }    return -1;}
bool app_is_child_lock_activated(void) {
    if (app_data) {        return app_data->is_child_lock_active;    }    return false;}
int app_set_sched(void) {
    if (app_data) {        return 0;    }    return -1;}
int app_get_sched(void) {
    if (app_data) {        return 0;    }    return -1;}
int app_enable_autoset_time_date(bool en) {
    if (app_data) {
        bool *is_auto_time_date_en = &(app_data->is_auto_time_date_en);
        if (en != *is_auto_time_date_en) {
            *is_auto_time_date_en = en;
            if (!ntp_is_enabled() && en) 
                ntp_init(NTP_SERVER);
            else if (ntp_is_enabled() && en == false)
                ntp_deinit();
            // save new value to flash
            set_integer_to_storage(STORAGE_KEY_IS_AUTO_TIME_DATE_EN, (int) *is_auto_time_date_en);
            return 0;        }    }    return -1;
}
bool app_is_autoset_time_date_enabled(void) {
    if (app_data) {        return app_data->is_auto_time_date_en;    }    return false;}
int app_enable_ap_mode(bool en) {
    if (app_data) {
        if (comm_wifi_dev) {
            if (en)
            	comm_wifi_dev->wifi_ap_enable(uniqueDeviceID, ap_password);    //testing
                //comm_wifi_dev->wifi_ap_enable(comm_wifi_dev->wifi_ap_ssid, comm_wifi_dev->wifi_ap_pw);
            else
                comm_wifi_dev->wifi_ap_disable();
            return 0;        }    }    return -1;
}
bool app_is_ap_mode_enabled(void) {
    if (app_data) {        if (comm_wifi_dev) {            return comm_wifi_dev->is_wifi_ap_enabled();        }    }    return false;}
int app_set_sta_mode_ssid(char *ssid, size_t len) {
    if (app_data) {        set_string_to_storage(NVS_LUCIDTRON_SSID_KEY, ssid);        return 0;    }    return -1;}
int app_set_sta_mode_password(char *pw, size_t len) {
    if (app_data) {        set_string_to_storage(NVS_LUCIDTRON_PW_KEY, pw);        return 0;    }    return -1;}
int app_enable_sta_mode(bool en) {
    if (app_data) {
        if (en) {
            if (comm_wifi_dev) {
                get_string_from_storage(NVS_LUCIDTRON_SSID_KEY, app_data->sta_ssid);
                get_string_from_storage(NVS_LUCIDTRON_PW_KEY, app_data->sta_pw);
                comm_wifi_dev->wifi_client_enable(app_data->sta_ssid, app_data->sta_pw);
                return 0;
            }        }    }    return -1;
}
bool app_is_sta_mode_enabled(void) {
    if (app_data) {        return true;    }    return false;}
int app_set_temp_unit(int unit) {
    if (app_data) {
        if (unit == TEMP_UNIT_CELSIUS
            || unit == TEMP_UNIT_FAHRENHEIT) {
            app_data->settings.temperature_unit = unit;  printf("\n app_set_temp_unit app_data->settings.temperature_unit %d \n", app_data->settings.temperature_unit);
            set_data_to_storage(STORAGE_KEY_SETTINGS, (void *) &app_data->settings, sizeof(settings_t));
            return 0;
        }    }    return -1;
}
int app_get_temp_unit(void) {
    if (app_data) {        return app_data->settings.temperature_unit;    }    return -1;}
bool app_get_rgb_state(void) {
      return rgb_led_state;
}

void app_delete_heater(bool value)
{
	if(value == 1){
	display_clear_screen();
	 display_menu("heater", DISPLAY_COLOR, "Deleted!", DISPLAY_COLOR);
	 vTaskDelay(5000);
	 erase_storage_all(); // erase flash..
	 esp_restart();}
}

int app_get_light_LDR_parm(void) {
      return nlight_br_TestingInSynchPacket;
}
bool app_get_anti_freeze_status(void) {
          return en_anti_freeze;}
bool app_get_anti_free_state(void) {
          return en_anti_freeze;}
int app_get_day_light_Saving_status(void) {
// bool app_get_day_light_Saving_status(void) {
    if (app_data) {        return app_data->daylightSaving;    }    return -1;}

int app_get_night_light_state_status(void) {
    if (app_data) {        return app_data->daylightSaving;    }    return -1;}
int app_enable_autodim_pilot_light(bool en) {
    if (app_data) {
        bool *is_dim_pilot_light_en = &(app_data->settings.is_dim_pilot_light_en);
        if (en != *is_dim_pilot_light_en) {
            *is_dim_pilot_light_en = en;
            set_data_to_storage(STORAGE_KEY_SETTINGS, (void *) &app_data->settings, sizeof(settings_t));
        }        return 0;
    }    return -1;
}
bool app_is_autodim_pilot_light_enabled(void) {
    if (app_data) {        return app_data->settings.is_dim_pilot_light_en;    }    return false;}
int app_enable_night_light_auto_brightness(bool en) {
    if (app_data) {
        bool *is_nlight_auto_br_en = &(app_data->settings.is_night_light_auto_brightness_en);
        if (en != *is_nlight_auto_br_en) {
            *is_nlight_auto_br_en = en;
            set_data_to_storage(STORAGE_KEY_SETTINGS, (void *) &app_data->settings, sizeof(settings_t));
            printf("is_nlight_auto_br_en :  %d", *is_nlight_auto_br_en );
        }        return 0;    }    return -1;
}
bool app_is_night_light_auto_brightness_enabled(void) {
    if (app_data) {        return app_data->settings.is_night_light_auto_brightness_en;    }    return false;}   // data->settings.is_night_light_auto_brightness_en
bool app_get_heater_state(void) {
    if (app_data) {        return app_data->lastHeaterState;    }    return false;}
void app_set_heater_state(int heater_state)
{
	heater_On_Off_state_by_command = heater_state ;
	if(heater_On_Off_state_by_command ==1)
	 { heater_On_Off_state_by_command_ExistFromStandByMode = 1;}// heater_on(); }
	else
	{ heater_On_Off_state_by_command_ExistFromStandByMode = 0;}// heater_off(); }

    if(heater_On_Off_state_by_command_ExistFromStandByMode == 0){ // New Added for Coming out of stand by mode
    	app_data->mode  = APP_MODE_STANDBY;   } // printf(" Come to stand by mode by Heater OFF commmand \n");

    // Original Lines
//    if(heater_On_Off_state_by_command_ExistFromStandByMode == 1){ // New Added for Coming out of stand by mode
//    	app_data->mode  = APP_MODE_MANUAL_TEMPERATURE; }  // printf(" Come to stand by mode by Heater OFF commmand \n");

// Testing Lines
    if(heater_On_Off_state_by_command_ExistFromStandByMode == 1){ // New Added for Coming out of stand by mode

#ifdef TEST_ELECTRONIC_ON_AUTOMATIC_CONTROL
    if( menuModeKeypressedFlag ==0)
    	{app_data->mode  = APP_MODE_MANUAL_TEMPERATURE;}
#else
     app_data->mode  = APP_MODE_MANUAL_TEMPERATURE;
#endif

    }  // printf(" Come to stand by mode by Heater OFF commmand \n");

	  app_data->lastHeaterState = heater_state;
//	  	if(app_data->lastHeaterState == 1)
//	       printf("Heater ON from app  \n ");
//	  	else
//          printf("Heater OFF from app \n ");
	  set_integer_to_storage(STORAGE_KEY_LAST_HEATER_STATE, (int)app_data->lastHeaterState);
	 // printf("app_set_heater_state by App app_data->lastHeaterState %d \n",app_data->lastHeaterState);
}
void RGB_LED_ON_OFF(int value)
{	if(value == 1)	{	rgb_led_state = 1;  printf("RGB LED ON \n");}
	else	{rgb_led_state = 0;   printf("RGB LED OFF \n");	}
}
int app_set_night_light_config(int cfg) {
	printf("In app_set_night_light_config function \n ");
    if (app_data) {
       int *nlight_cfg = &(app_data->night_light_cfg);
       // set
       *nlight_cfg = cfg;
       // save
        set_integer_to_storage(STORAGE_KEY_NIGHT_LIGHT_CFG, cfg);
        printf("Remote command set led -*nlight_cfg - %d", *nlight_cfg);  printf("app_set_night_light_config app_data->night_light_cfg %d \n ",app_data->night_light_cfg);         printf("Remote command set led -nlight_cfg - %d", cfg);
       return 0;    }    return -1;
}
int app_get_night_light_config(void) {
    if (app_data) {        return app_data->night_light_cfg;    }    return -1;}
int app_enable_child_lock(bool en) {
    if (app_data) {
        bool *is_child_lock_en = &(app_data->settings.is_child_lock_en);
        if (en != *is_child_lock_en) {
            *is_child_lock_en = en;
            set_data_to_storage(STORAGE_KEY_SETTINGS, (void *) &app_data->settings, sizeof(settings_t));
        }        return 0;    }    return -1;
}
bool app_is_child_lock_enabled(void) {
    if (app_data) {        return app_data->settings.is_child_lock_en;    }    return false;}
int app_enable_autodim_display(bool en) {
    if (app_data) {
        bool *is_auto_display_brightness_en = &(app_data->display_settings.is_auto_display_brightness_en);
        if (en != *is_auto_display_brightness_en) {
            *is_auto_display_brightness_en = en;
            set_data_to_storage(STORAGE_KEY_DISPLAY_SETTINGS, (void *) &app_data->display_settings, sizeof(display_settings_t));
        }        return 0;    }    return -1;
}
bool app_is_autodim_display_enabled(void) {
    if (app_data) {        return app_data->display_settings.is_auto_display_brightness_en;    }    return false;}
int app_set_screen_brightness(int br) {
    if (app_data) {
        if (br >= DISPLAY_BRIGHTNESS_MIN
            && br <= DISPLAY_BRIGHTNESS_MAX) {
            int *display_brightness = &(app_data->display_settings.display_brightness);
            if (br != *display_brightness) {
                *display_brightness = br;
                set_data_to_storage(STORAGE_KEY_DISPLAY_SETTINGS, (void *) &app_data->display_settings, sizeof(display_settings_t));
            }            return 0;        }    }    return -1;
}
int app_get_screen_brightness(void) {
    if (app_data) {        return app_data->display_settings.display_brightness;    }    return 0x80000000;}

void pingDevice(unsigned char p_pingDeviceFlag)
{	pingDeviceOnFlag =p_pingDeviceFlag;}
int app_enable_auto_screen_off(bool en) {
    if (app_data) {
        bool *is_auto_screen_off_en = &(app_data->display_settings.is_auto_screen_off_en);
        if (en != *is_auto_screen_off_en) {
            *is_auto_screen_off_en = en;
            set_data_to_storage(STORAGE_KEY_DISPLAY_SETTINGS, (void *) &app_data->display_settings, sizeof(display_settings_t));
        }        return 0;    }    return -1;
}
bool app_is_auto_screen_off_enabled(void) {
    if (app_data) {        return app_data->display_settings.is_auto_screen_off_en;    }    return false;}
int app_set_auto_screen_off_delay(int delay) {
    if (app_data) {
        if (delay >= AUTO_SCREEN_OFF_DELAY_SEC_MIN
            && delay <= AUTO_SCREEN_OFF_DELAY_SEC_MAX) {
            int *auto_screen_off_delay_sec = &(app_data->display_settings.auto_screen_off_delay_sec);
            if (delay != *auto_screen_off_delay_sec) {
                *auto_screen_off_delay_sec = delay;
                set_data_to_storage(STORAGE_KEY_DISPLAY_SETTINGS, (void *) &app_data->display_settings, sizeof(display_settings_t));
            }            return 0;        }    }    return -1;
}
int app_get_auto_screen_off_delay(void) {
    if (app_data) {        return app_data->display_settings.auto_screen_off_delay_sec;    }    return 0x80000000;}
int app_start_fw_update(void) {
    if (app_data) {
#if 0 //TODO:
        if (is_fw_update_available()) {
            start_fw_update();
            return 0;
        }
#endif
    }    return -1;
}
int app_ota_start(char* loc)
{      return ota_start(loc);  }

