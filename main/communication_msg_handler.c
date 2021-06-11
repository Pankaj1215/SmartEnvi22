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
#include "esp_log.h"
#include "esp_system.h"
#include "communication_server.h"
#include "communication_server_hal.h"
#include "communication_msg_handler.h"

//#include "freertos/FreeRTOS.h"   // Added for TaskDelayFunction ..s FreeRTOS HEADERS
//#include "freertos/task.h"
//#include "freertos/event_groups.h"


#include "common.h"
//TODO: #include "debug.h"
#include "lucidtron_core.h"
#include "non_volatile_lib.h"
#include "sdk_util.h"
#include "app.h"
#include "mqtt.h"
#include "version.h"

#include "display.h"  // Display on Screen

#include "wifi_core.h" // New Added for P_Testing included in wifi_core.h
// #include <math.h>

// int oneTimeRegistrationPacketToAWS = 0;

#define KEYMARK_OPENCBRACKET        '{'
#define KEYMARK_QUOTE               '"'
#define KEYMARK_COLON               ':'
#define KEYMARK_COMMA               ','
#define KEYMARK_CLOSECBRACKET       '}'

struct comm_wifi* cwifi_dev;
struct comm_bluetooth* cbt_dev;
struct comm_sigfox* csig_dev;
struct comm_lora* clora_dev;
struct comm_gsm* cgsm_dev;

int message_label_value_handler(char* label, char* value, char* reply_buff);

#ifdef  P_TESTING

extern int version_major,version_minor,version_revision;
extern char replybuff[150];  // Testing
extern unsigned char commandReceived_SendAck;
extern unsigned char oneTimeRegistrationPacketToAWS;
extern unsigned char keepAliveFlag;
extern unsigned char CommandAck;
extern char uniqueDeviceID[12];
#include "heater.h"  // new Added fot Heater OnOff functions..


extern unsigned char updateDisplayAfterAppCommand;

unsigned char en_anti_freeze;
int hexadecimalToDecimal(char hexVal[]);
void decToHexa(int n);
unsigned char rgb_led_state;
unsigned char heater_On_Off_state_by_command;
unsigned char TimerIntervalThresholdOffset;
#endif

unsigned int hex2int(char hex[]);
long power(long no,long p);

int nlight_br_TestingInSynchPacket; // Only for testing ..

bool pingDeviceOnFlag;

int getNum(char ch);
//function : getNum
//this function will return number corresponding
//0,1,2..,9,A,B,C,D,E,F

int getNum(char ch)
{
    int num=0;
    if(ch>='0' && ch<='9')
    {
        num=ch-0x30;
    }
    else
    {
        switch(ch)
        {
            case 'A': case 'a': num=10; break;
            case 'B': case 'b': num=11; break;
            case 'C': case 'c': num=12; break;
            case 'D': case 'd': num=13; break;
            case 'E': case 'e': num=14; break;
            case 'F': case 'f': num=15; break;
            default: num=0;
        }
    }
    return num;
}


//function : hex2int
//this function will return integer value against
//hexValue - which is in string format

// unsigned int hex2int(unsigned char hex[])
unsigned int hex2int(char hex[])
{
    unsigned int x=0;
   //  x=(getNum(hex[0]))*16+(getNum(hex[1]));

    printf("hex[0] %c\n", hex[0]);
    printf("hex[1] %c\n", hex[1]);
    printf("hex[2] %c\n", hex[2]);
    printf("hex[3] %c\n", hex[3]);
    printf("hex[4] %c\n", hex[4]);
    printf("hex[5] %c\n", hex[5]);

    // x=(getNum(hex[0]))*16+(getNum(hex[1]));
    x=((getNum(hex[0])*16*16*16*16*16)+(getNum(hex[1])*16*16*16*16) + (getNum(hex[2])*16*16*16)+(getNum(hex[3])*16*16)+(getNum(hex[4]))*16) +(getNum(hex[5]));
   // x=((getNum(hex[0])*power(16,5))+(getNum(hex[1])*power(16,4)) + (getNum(hex[2])*power(16,3))+(getNum(hex[3])*power(16,2))+(getNum(hex[4]))*16) +(getNum(hex[5]));
    printf("x %d \n ", x);
    return (x);
}


int mainflux_msg_handler(char* msg, char* response)
{
//	char major[2]={0},minor[2]={0},revision[2]={0}; // Working one ..
	char major[4]={0},minor[4]={0},revision[4]={0}; // Changing for 1.6.10 version

    int i = 0;
    char label[100];
    char value[100];
    char* labelstart = NULL;
    char* labelend = NULL;
    char* valuestart = NULL;
    char* valueend = NULL;
    int testpass = 0;
    int quote_ctr = 0;
    int comma_ctr = 0;
    int colon_ctr = 0;

   //  char replybuff[500];  // Original Line in longHorn Code commneted for making the reply buffer global.

    int messagetype = MSGTYPE_UNKNOWN;
    int is_message_for_me = -1;
    char username[MAX_STR_BUFF_SIZE];

    // printf("mainflux handler [%s]\n", msg);

    //checking msg validity
    while(*(msg+i) != 0)
    {
        if(*(msg+i) == KEYMARK_OPENCBRACKET)    //  {
            testpass++;
        if(*(msg+i) == KEYMARK_CLOSECBRACKET)   //  }
            testpass++;
        if(*(msg+i) == KEYMARK_QUOTE)           //  "
            quote_ctr++;
        if(*(msg+i) == KEYMARK_COLON)           //  :
            colon_ctr++;
        if(*(msg+i) == KEYMARK_COMMA)           //  ,
            comma_ctr++;
        i++;
    }
    if((quote_ctr % 2) != 0)
    {
        printf("missing quotes %d\n", quote_ctr);
        return FAIL;
    }
    else 
        testpass++;
    if((quote_ctr / 4) != colon_ctr)
    {
        printf("missing colon %d %d\n", quote_ctr, colon_ctr);
        return FAIL;
    }
    else 
        testpass++;
    if(colon_ctr != (comma_ctr+1))  //colon is +1 of comma
    {
        printf("missing comma %d %d\n", colon_ctr, comma_ctr);
        return FAIL;
    }
    else 
        testpass++;
    if(testpass == 5)
        printf("all test passed\n");

    //parsing the strings
    i = 0;
    while(*(msg+i) != 0)
    {
        switch(*(msg+i))
        {
            case KEYMARK_OPENCBRACKET:
                //printf("key openbracket\n");
                break;
            case KEYMARK_QUOTE:
                //printf("key quote\n");
                if(labelstart == NULL)
                {
                    labelstart = msg + i + 1;
                    memset(label, 0, 100);
                }
                else if(labelend == NULL)
                {
                    labelend = msg + i;
                }
                else if (valuestart == NULL)
                {
                    valuestart = msg + i + 1;
                    memset(value, 0, 100);
                }
                else if (valueend == NULL)
                {
                    valueend = msg + i;
                }
                break;
            case KEYMARK_COLON:
                //printf("key colon\n");
                break;
            case KEYMARK_COMMA:
            case KEYMARK_CLOSECBRACKET:

                strncpy(label, labelstart, labelend - labelstart); 
                strncpy(value, valuestart, valueend - valuestart); 
                //printf("label [%s]\n", label);
                // printf("value [%s]\n", value);

                if(strcmp(label, "version_major") == 0)
                				{
                					strcpy(major,value); // "1"
                					version_major= atoi(major);
                					//printf("version_major: %s",major);
                				}
                				if(strcmp(label, "version_minor") == 0)
                				{
                					strcpy(minor, value); // "0"
                					version_minor= atoi(minor);
                					//printf("version_minor: %s",minor);
                				}
                				if(strcmp(label, "version_revision") == 0)
                				{
                					strcpy(revision, value);  // "6"
                					version_revision= atoi(revision);
                					//printf("version_revision: %s",revision);
                				}

             //   if(strcmp(label, "cmd") == 0)   // Original
             	if(strcmp(label, "type") == 0)
                {
                    if(strcmp(value, "system") == 0)
                        messagetype = MSGTYPE_SYSTEM;
                    else if(strcmp(value, "set") == 0)
                        messagetype = MSGTYPE_SET;
                    else if(strcmp(value, "get") == 0)
                        messagetype = MSGTYPE_GET;
                }

                if(strcmp(label, "target") == 0)
                {
                	//printf("target label Matched \n");
                    memset(username, 0, MAX_STR_BUFF_SIZE); 

                   // get_string_from_storage(NVS_MQTT_USERNAME, username); // Original Line
                    // if(strcmp(value, "all") == 0) // Original
                   //	if(strcmp(value, "Heater1") == 0)  // Testing
                   	if(strcmp(value, uniqueDeviceID) == 0)  // Testing
                   	{
                   		is_message_for_me = 0;
                   		//printf("Device ID Matched \n");
                   	}
                    else if(strcmp(value, username) == 0)
                        is_message_for_me = 0;
                   	// Added for testing
                    else
                    	printf("Header ID not matched \n");
                }

                //check if the required parameter is passed before proceeding
                if(messagetype != MSGTYPE_UNKNOWN)
                {
                    if(is_message_for_me != -1)
                    {
                        //TODO: call the msg handler callback here
                       // memset(replybuff, 0, 500);  // Original
                         memset(replybuff, 0, 150);  //Testing
                        message_label_value_handler(label, value, replybuff);
                       // printf("reply buff [%s]\n", replybuff);

                        if(strlen(replybuff) > 0)
                        {
                           // printf("publishing message [%s]\n", replybuff);
                           // mqtt_publish_message(replybuff, NULL);  // Original Line Commented for Testing only
							#ifdef P_TESTING
								commandReceived_SendAck = 1;
							#endif
                        }
                    }
                    else
                        printf("still waiting for correct target\n");
                }
                else 
                    printf("still waiting for messagetype\n");

                labelstart = NULL;
                labelend = NULL;
                valuestart = NULL;
                valueend = NULL;
                break;
        }
        i++;
    }

    return SUCCESS;
}

int aurora_msg_handler(char* msg, char* response)
{
    char* equalsign = NULL;
    char duplicate[MAX_STR_BUFF_SIZE];
    char label[MAX_STR_BUFF_SIZE];
    char value[MAX_STR_BUFF_SIZE];

    printf("msg=%s\r\n", msg);

    memset(label, 0, MAX_STR_BUFF_SIZE); 
    memset(value, 0, MAX_STR_BUFF_SIZE); 
    memset(duplicate, 0, MAX_STR_BUFF_SIZE); 

    strcpy(duplicate, msg);
    equalsign = strstr(duplicate, VALUE_KEY);
    if(equalsign != NULL) 
    {
        equalsign++;
        strcpy(value, equalsign);
        equalsign--;
        *equalsign = 0;
        strcpy(label, duplicate);
    }
    
    message_label_value_handler(label, value, response);

    return SUCCESS;
}

static char* str_replace(char* str, char* a, char* b)
{
    int len  = strlen(str);
    int lena = strlen(a), lenb = strlen(b);
    for (char* p = str; (p = strstr(p, a)); ++p) {
        if (lena != lenb) // shift end as needed
            memmove(p+lenb, p+lena, len - (p - str) + lenb);
        memcpy(p, b, lenb);
    }
    return str;
}

static void space_char_decode(char *str)
{
    str_replace(str, "%20", " ");
}

int message_label_value_handler(char* label, char* value, char* reply_buff)
{
    char broker[MAX_STR_BUFF_SIZE];
    char username[MAX_STR_BUFF_SIZE];
    char password[MAX_STR_BUFF_SIZE];
    char channel[MAX_STR_BUFF_SIZE];

    memset(broker, 0, MAX_STR_BUFF_SIZE); 
    memset(username, 0, MAX_STR_BUFF_SIZE); 
    memset(password, 0, MAX_STR_BUFF_SIZE); 
    memset(channel, 0, MAX_STR_BUFF_SIZE); 

    unsigned char luchRGB_MODE = 0;

    space_char_decode(value);

   // printf("I am in message_label_value_handler \n ");

    //TODO: please revise if only, dont use if else if due to limit
    if (strcmp(label, "cmd") == 0) {
        if (strcmp(value, REMOTE_CMD_CMD_WHOAREYOU) == 0) {
            printf("REMOTE_CMD_CMD_WHOAREYOU \n");
            //sprintf(reply_buff, "%s\n%s\n%s", "Longhorn", "Private", "NotSure");
            sprintf(reply_buff, "%s", "Longhorn");
        } else if (strcmp(value, REMOTE_CMD_CMD_OWNDEVICE) == 0) {
            get_string_from_storage(NVS_MQTT_BROKER, broker);
            get_string_from_storage(NVS_MQTT_USERNAME, username);
            get_string_from_storage(NVS_MQTT_PASSWORD, password);
            get_string_from_storage(NVS_MQTT_CHANID, channel);
            sprintf(reply_buff, 
            "{\"broker\":\"%s\","
            "\"username\":\"%s\","
            "\"password\":\"%s\","
            "\"channel\":\"%s\"}", 
            broker,
            username,
            password,
            channel);
        } else if (strcmp(value, REMOTE_CMD_CMD_HI) == 0) {
            sprintf(reply_buff, "%s", "Hello");
        } else if (strcmp(value, REMOTE_CMD_CMD_HELLO) == 0) {
            sprintf(reply_buff, "%s", "Hi");
        }
    } else if (strcmp(label, REMOTE_CMD_EN_AP_MODE) == 0) {
        printf("REMOTE_CMD_EN_AP_MODE %s\r\n", value);
        app_enable_ap_mode(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_AP_MODE_EN) == 0) {
        printf("REMOTE_CMD_IS_AP_MODE_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_ap_mode_enabled());
    } else if (strcmp(label, REMOTE_CMD_SET_STA_MODE_SSID) == 0) {
        printf("REMOTE_CMD_SET_STA_MODE_SSID %s\r\n", value);
        app_set_sta_mode_ssid(value, strlen(value));
    } else if (strcmp(label, REMOTE_CMD_SET_STA_MODE_PASSWORD) == 0) {
        printf("REMOTE_CMD_SET_STA_MODE_PASSWORD %s\r\n", value);
        app_set_sta_mode_password(value, strlen(value));
    } else if (strcmp(label, REMOTE_CMD_EN_STA_MODE) == 0) {
        printf("REMOTE_CMD_EN_STA_MODE %s\r\n", value);
        app_enable_sta_mode(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_STA_MODE_EN) == 0) {
        printf("REMOTE_CMD_IS_STA_MODE_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_sta_mode_enabled());
    } else if (strcmp(label, REMOTE_CMD_GET_FW_VER) == 0) { 
        printf("REMOTE_CMD_GET_FW_VER\r\n");
        get_version(reply_buff);
    } else if (strcmp(label, REMOTE_CMD_START_FW_UPDATE) == 0) {
        printf("REMOTE_CMD_START_FW_UPDATE %s\r\n", value);
        app_start_fw_update();
    } else if (strcmp(label, REMOTE_CMD_MQTT_BROKER) == 0) {
        printf("REMOTE_CMD_MQTT_BROKER %s\r\n", value);
        set_string_to_storage(NVS_MQTT_BROKER, value);
        mqtt_set_broker(value);
        sprintf(reply_buff, "%s", "BROKER OK");
    } else if (strcmp(label, REMOTE_CMD_MQTT_USERNAME) == 0) {
        printf("REMOTE_CMD_MQTT_USERNAME %s\r\n", value);
        set_string_to_storage(NVS_MQTT_USERNAME, value);
        mqtt_set_username(value);
        sprintf(reply_buff, "%s", "USERNAME OK");
    } else if (strcmp(label, REMOTE_CMD_MQTT_PASSWORD) == 0) {
        printf("REMOTE_CMD_MQTT_PASSWORD %s\r\n", value);
        set_string_to_storage(NVS_MQTT_PASSWORD, value);
        mqtt_set_password(value);
        sprintf(reply_buff, "%s", "PASSWORD OK");
    } else if (strcmp(label, REMOTE_CMD_MQTT_CHANNEL) == 0) {
        printf("REMOTE_CMD_MQTT_CHANNEL %s\r\n", value);
        set_string_to_storage(NVS_MQTT_CHANID, value);
        mqtt_set_channel(value);
        sprintf(reply_buff, "%s", "CHANNEL OK");

    /* BELOW HERE ARE ALL LONGHORN SPECIFIC */

    } else if (strcmp(label, REMOTE_CMD_CMD_STATUS) == 0) {
        printf("REMOTE_CMD_GET_MODE %s\r\n", value);
        sprintf(reply_buff, 
            "{\"is_child_lock_en\":\"%d\","  
            "\"get_screen_brightness\":\"%d\"," 
            "\"get_target_temp\":\"%d\","
            "\"is_night_light_auto_brightness_en\":\"%d\","
            "\"get_night_light_config\":\"%d\","
            "\"get_timer_setting\":\"%d\","
            "\"get_mode\":\"%d\","
            "\"is_auto_set_time_date_en\":\"%d\""
            "}", 
            app_is_child_lock_enabled(),
            app_get_screen_brightness(),
            app_get_target_temp(),
            app_is_night_light_auto_brightness_enabled(),
            app_get_night_light_config(),
            app_get_timer(),
            app_get_mode(),
            app_is_autoset_time_date_enabled()
            );

    } else if (strcmp(label, REMOTE_CMD_GET_MODE) == 0) {
        printf("REMOTE_CMD_GET_MODE %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_mode());
    } else if (strcmp(label, REMOTE_CMD_SET_MODE) == 0) {
        printf("REMOTE_CMD_SET_MODE %s\r\n", value);
        app_set_mode(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_GET_AMBIENT_TEMP) == 0) {
        printf("REMOTE_CMD_GET_AMBIENT_TEMP %s\r\n", value);
        sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%d\" ", "cmd", "get","type", "get_temp", "status","success",  "value",app_get_ambient_temp());
       // sprintf(reply_buff, "%d", app_get_ambient_temp());

    } else if (strcmp(label, REMOTE_CMD_SET_TARGET_TEMP) == 0) {
        printf("REMOTE_CMD_SET_TARGET_TEMP %s\r\n", value);
        app_set_target_temp(atoi(value));

        // working one..
       // sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\"", "type", "set","cmd", "set_temp", "status","success", "value", value);
        // Testing one ..
        sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%d\"", "type", "set","cmd", "set_temp", "status","success", "value", value,"temp_unit", app_get_temp_unit());
    	CommandAck = SET_TEMP_ACK;

    	// Added for testing only_begin
    	display_on();
    	display_clear_screen();
    	display_menu_small_font("set temp", DISPLAY_COLOR, value, DISPLAY_COLOR);
    	updateDisplayAfterAppCommand = 1;
        // Testing End

    } else if (strcmp(label, REMOTE_CMD_GET_TARGET_TEMP) == 0) {
        printf("REMOTE_CMD_GET_TARGET_TEMP %s\r\n", value);
        // This for get set temperature command..

         // Working one
        //  sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%d\" \n \t\"%s\" : \"%d\" \n \t\"%s\" : \"%d\" \n \t\"%s\" : \"%d\"", "type", "get","cmd", "get_temp", "status","success",  "value",app_get_ambient_temp(), "s_t", app_get_target_temp(),"t_u",app_get_temp_unit(), "h_s", app_get_heater_state());

        // testing one.with get Temp and Temp unit .. // workimg till 11feb..
        // sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%d\" \n \t\"%s\" : \"%d\"", "type", "get","cmd", "get_temp", "status","success",  "value",app_get_ambient_temp(),"temp_unit", app_get_temp_unit());

        // Testing 11 Feb 2021.. commma was missing after value..
        sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%d\", \n \t\"%s\" : \"%d\"", "type", "get","cmd", "get_temp", "status","success",  "value",app_get_ambient_temp(),"temp_unit", app_get_temp_unit());

        CommandAck = GET_TEMP_ACK;

    } else if (strcmp(label, REMOTE_CMD_SET_TIMER_SETTING) == 0) {
        printf("REMOTE_CMD_SET_TIMER_SETTING %s\r\n", value);
        app_set_timer(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_GET_TIMER_SETTING) == 0) {
        printf("REMOTE_CMD_GET_TIMER_SETTING %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_timer());
    } else if (strcmp(label, REMOTE_CMD_ACTIVATE_CHILD_LOCK) == 0) {
        printf("REMOTE_CMD_ACTIVATE_CHILD_LOCK %s\r\n", value);

//        CommandAck = ACTIVATE_CHILD_LOCK_ACK;
//        if(*value == 1)
//		   sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "en_child_lock", "status","success",  "value",value);
//        else
//    		sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "dis_child_lock", "status","success",  "value",value);
        app_activate_child_lock(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_CHILD_LOCK_ACTIVATED) == 0) {
        printf("REMOTE_CMD_IS_CHILD_LOCK_ACTIVATED %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_child_lock_activated());
    } else if (strcmp(label, REMOTE_CMD_SET_SCHED) == 0) {
        printf("REMOTE_CMD_SET_SCHED %s\r\n", value);
    } else if (strcmp(label, REMOTE_CMD_GET_SCHED) == 0) {
        printf("REMOTE_CMD_GET_SCHED %s\r\n", value);
    } else if (strcmp(label, REMOTE_CMD_EN_AUTO_SET_TIME_DATE) == 0) {
        printf("REMOTE_CMD_EN_AUTO_SET_TIME_DATE %s\r\n", value);
        app_enable_autoset_time_date(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_AUTO_SET_TIME_DATE_EN) == 0) {
        printf("REMOTE_CMD_IS_AUTO_SET_TIME_DATE_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_autoset_time_date_enabled());
    } else if (strcmp(label, REMOTE_CMD_SET_TEMP_UNIT) == 0) {
        printf("REMOTE_CMD_SET_TEMP_UNIT %s\r\n", value);
        CommandAck = SET_TEMP_UNIT_ACK;  //
	    sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "set_temp_unit", "status","success",  "value",value);
        app_set_temp_unit(atoi(value));
    	display_on();
    	display_clear_screen();
        if(atoi(value) == 0)
        	display_menu_small_font("temp unit", DISPLAY_COLOR, "calcius", DISPLAY_COLOR);
        else
        	display_menu_small_font("temp unit", DISPLAY_COLOR, "fahren", DISPLAY_COLOR);
    	updateDisplayAfterAppCommand = 1;

    } else if (strcmp(label, REMOTE_CMD_GET_TEMP_UNIT) == 0) {
        printf("REMOTE_CMD_GET_TEMP_UNIT %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_temp_unit());
    } else if (strcmp(label, REMOTE_CMD_EN_AUTO_DIM_PILOT_LIGHT) == 0) {
        printf("REMOTE_CMD_EN_AUTO_DIM_PILOT_LIGHT %s\r\n", value);
        app_enable_autodim_pilot_light(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_AUTO_DIM_PILOT_LIGHT_EN) == 0) {
        printf("REMOTE_CMD_IS_AUTO_DIM_PILOT_LIGHT_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_autodim_pilot_light_enabled());
    } else if (strcmp(label, REMOTE_CMD_EN_NIGHT_LIGHT_AUTO_BRIGHTNESS) == 0) {
        printf("REMOTE_CMD_EN_NIGHT_LIGHT_AUTO_BRIGHTNESS %s\r\n", value);

        CommandAck = EN_NIGHT_LIGHT_MODE_ACK;

       	sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "en_night_light_mode", "status","success",  "value",value);
//       	if(*value == 1)
//		sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "en_night_light_mode", "status","success",  "value",value);
//        else
//    		sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "dis_night_light_mode", "status","success",  "value",value);

        // app_enable_night_light_auto_brightness(atoi(value));  // original code line commented for Auo,OFF, ON mode
    	display_on();
    	display_clear_screen();

#define RGB_AUTO_ON_OFF_MODE
#ifdef  RGB_AUTO_ON_OFF_MODE
			if (strcmp(value, "01") == 0)
			{
			  app_enable_night_light_auto_brightness(atoi(value));
			  display_menu_small_font("RGB set", DISPLAY_COLOR, "Auto", DISPLAY_COLOR);
			}
			if (strcmp(value, "00") == 0)
			{  // rgb_led_state = 0;
			   app_enable_night_light_auto_brightness(atoi(value));  // here value passed is Zero only, so passed value in that fucntion.
			   printf(" rgb_led_state OFF \n  ");
			   display_menu_small_font("RGB set", DISPLAY_COLOR, "Off", DISPLAY_COLOR);
			}
			if (strcmp(value, "02") == 0)
			{  // rgb_led_state = 1;
			   app_enable_night_light_auto_brightness(atoi(value)); // Zero is passed to disable the night light --
			   printf("rgb_led_state ON \n ");

			   display_menu_small_font("RGB set", DISPLAY_COLOR, "On", DISPLAY_COLOR);
			}
			updateDisplayAfterAppCommand = 1;
#else
       	app_enable_night_light_auto_brightness(atoi(value));
#endif


    } else if (strcmp(label, REMOTE_CMD_IS_NIGHT_LIGHT_AUTO_BRIGHTNESS_EN) == 0) {
        printf("REMOTE_CMD_IS_NIGHT_LIGHT_AUTO_BRIGHTNESS_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_night_light_auto_brightness_enabled());
    } else if (strcmp(label, REMOTE_CMD_SET_NIGHT_LIGHT_CONFIG) == 0) {
        printf("REMOTE_CMD_SET_NIGHT_LIGHT_CONFIG %s\r\n", value);
        CommandAck = SET_RGB_ACK;

        int hex_to_num = 0 ;
        hex_to_num = hex2int(value);
        app_set_night_light_config(hex_to_num);   // Testing
       // app_set_night_light_config(atoi(value));  // Original

		sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "set_rgb_led", "status","success",  "value",value);

    	display_on();
    	display_clear_screen();
    	display_menu_small_font("RGB Colour", DISPLAY_COLOR, "change", DISPLAY_COLOR);
    	updateDisplayAfterAppCommand = 1;

    } else if (strcmp(label, REMOTE_CMD_GET_NIGHT_LIGHT_CONFIG) == 0) {
        printf("REMOTE_CMD_GET_NIGHT_LIGHT_CONFIG %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_night_light_config());
    } else if (strcmp(label, REMOTE_CMD_EN_CHILD_LOCK) == 0) {
        printf("REMOTE_CMD_EN_CHILD_LOCK %s\r\n", value);
        app_enable_child_lock(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_CHILD_LOCK_EN) == 0) {
        printf("REMOTE_CMD_IS_CHILD_LOCK_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_child_lock_enabled());
    } else if (strcmp(label, REMOTE_CMD_EN_AUTO_DIM_DISPLAY) == 0) {
        printf("REMOTE_CMD_EN_AUTO_DIM_DISPLAY %s\r\n", value);
        app_enable_autodim_display(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_AUTO_DIM_DISPLAY_EN) == 0) {
        printf("REMOTE_CMD_IS_AUTO_DIM_DISPLAY_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_autodim_display_enabled());
    } else if (strcmp(label, REMOTE_CMD_SET_SCREEN_BRIGHTNESS) == 0) {
        printf("REMOTE_CMD_SET_SCREEN_BRIGHTNESS %s\r\n", value);
        app_set_screen_brightness(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_GET_SCREEN_BRIGHTNESS) == 0) {
        printf("REMOTE_CMD_GET_SCREEN_BRIGHTNESS %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_screen_brightness());
    } else if (strcmp(label, REMOTE_CMD_EN_AUTO_SCREEN_OFF) == 0) {
        printf("REMOTE_CMD_EN_AUTO_SCREEN_OFF %s\r\n", value);
        app_enable_auto_screen_off(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_IS_AUTO_SCREEN_OFF_EN) == 0) {
        printf("REMOTE_CMD_IS_AUTO_SCREEN_OFF_EN %s\r\n", value);
        sprintf(reply_buff, "%d", app_is_auto_screen_off_enabled());
    } else if (strcmp(label, REMOTE_CMD_SET_AUTO_SCREEN_OFF_DELAY_SEC) == 0) {
        printf("REMOTE_CMD_SET_AUTO_SCREEN_OFF_DELAY_SEC %s\r\n", value);
        app_set_auto_screen_off_delay(atoi(value));
    } else if (strcmp(label, REMOTE_CMD_GET_AUTO_SCREEN_OFF_DELAY_SEC) == 0) {
        printf("REMOTE_CMD_GET_AUTO_SCREEN_OFF_DELAY_SEC %s\r\n", value);
        sprintf(reply_buff, "%d", app_get_auto_screen_off_delay());
    } else if (strcmp(label, REMOTE_CMD_OTA) == 0) {
        printf("REMOTE_CMD_OTA%s\r\n", value);
        app_ota_start(value);
    }
	// else if (strcmp(label, "dev_regis") == 0) {
	 else if (strcmp(label, REMOTE_CMD_DEV_REGIS) == 0) {
		 if(strcmp(value, "success") == 0)
		   printf("dev_registered successfully \n");
		oneTimeRegistrationPacketToAWS= 0;
		keepAliveFlag = 1;

    	display_on();
    	display_clear_screen();
    	display_menu_small_font("Device", DISPLAY_COLOR, "registered", DISPLAY_COLOR);
    	updateDisplayAfterAppCommand = 1;

	}
	 else if (strcmp(label, REMOTE_CMD_HEATER_ON_OFF) == 0) {
		  printf("REMOTE_CMD_HEATER_ON OFF \n");
		  CommandAck = HEATER_ON_OFF_ACK;
		  app_set_heater_state(atoi(value));
	      sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "heater_on_off", "status","success",  "value",value);

	    	display_on();
	    	display_clear_screen();
	    	if(atoi(value) == 1)
	    		display_menu_small_font("Manual", DISPLAY_COLOR, "Mode", DISPLAY_COLOR);
	    	else
	    		display_menu_small_font("Standby", DISPLAY_COLOR, "Mode", DISPLAY_COLOR);
	    	updateDisplayAfterAppCommand = 1;

	 }
	 else if (strcmp(label, REMOTE_CMD_EN_ANTI_FREEZE) == 0) {
		   printf("REMOTE_CMD_EN_ANTI_FREEZE \n");
		   CommandAck = EN_ANTI_FREEZE_ACK;

			if (strcmp(value, "01") == 0)
			{  en_anti_freeze = 1;
			 printf("en_anti_freeze\n ");
			}
			if (strcmp(value, "00") == 0)
			{  en_anti_freeze = 0;
			 printf(" en_anti_freeze disable \n  ");
			}
			 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "en_anti_freeze", "status","success",  "value",value);

			 // app_data->display_settings.is_auto_screen_off_en;

			// app_data->display_settings.is_auto_screen_off_en = false;
		    display_on();
		    display_clear_screen();
			if(atoi(value))
				display_menu_small_font("anti_freeze", DISPLAY_COLOR, "enable", DISPLAY_COLOR);
			else
				display_menu_small_font("anti_freeze", DISPLAY_COLOR, "disable", DISPLAY_COLOR);
			updateDisplayAfterAppCommand = 1;

	      }
	 else if (strcmp(label, REMOTE_CMD_RGB_LED_STATE) == 0) {
			   printf("REMOTE_CMD_RGB_LED_STATE \n");
			   CommandAck = RGB_LED_STATE_ACK;

				if (strcmp(value, "01") == 0)
				{  rgb_led_state = 1;
				   printf("rgb_led_state ON \n ");
				}
				if (strcmp(value, "00") == 0)
				{  rgb_led_state = 0;
				   printf(" rgb_led_state OFF \n  ");
				}
				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "rgb_led_state", "status","success",  "value",value);
				}
	 else if (strcmp(label, REMOTE_CMD_DAY_LIGHT_TIME_STATE) == 0) {
	 			   printf("REMOTE_CMD_DAY_LIGHT_TIME_STATE \n");
	 			   CommandAck = DAY_LIGHT_TIME_STATE_ACK;

	 				if (strcmp(value, "01") == 0)
	 				{  app_data->daylightSaving = 1;
	 				   printf("REMOTE_CMD_DAY_LIGHT_TIME_STATE ON \n ");
	 				}
	 				if (strcmp(value, "00") == 0)
	 				{  app_data->daylightSaving = 0;
	 				   printf(" REMOTE_CMD_DAY_LIGHT_TIME_STATE OFF \n  ");
	 				}
	 				 set_integer_to_storage(STORAGE_KEY_EN_DAY_LIGHT_SAVING, (int)app_data->daylightSaving);

	 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "day_light_time_on", "status","success",  "value",value);

					display_on();
					display_clear_screen();
					if(atoi(value))
						display_menu_small_font("dst", DISPLAY_COLOR, "on", DISPLAY_COLOR);
					else
						display_menu_small_font("dst", DISPLAY_COLOR, "off", DISPLAY_COLOR);
					updateDisplayAfterAppCommand = 1;

	                }
	 else if (strcmp(label, REMOTE_CMD_SET_THRESHOLD_OFFSET_TIME) == 0) {
		 			 CommandAck = SET_THRESHOLD_OFFSET_TIME_ACK;
                      //Put this value in the variable for threshold offset value
		 			 printf("REMOTE_CMD_SET_THRESHOLD_OFFSET_TIME %s\r\n", value);
		 			 app_data-> TimerIntervalThresholdOffset = atoi(value);
 		 			 TimerIntervalThresholdOffset = app_data-> TimerIntervalThresholdOffset;
		 			 set_integer_to_storage(STORAGE_KEY_THRESHOLD_OFFSET_TIME, (int)app_data-> TimerIntervalThresholdOffset);
		 			 printf("app_data-> TimerIntervalThresholdOffset %d  app_data-> TimerIntervalThresholdOffset %d\r\n", app_data-> TimerIntervalThresholdOffset, app_data-> TimerIntervalThresholdOffset);
		 			 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "set_threshold_offset_time", "status","success",  "value",value);
		 			}
	 else if (strcmp(label, REMOTE_CMD_HEATER_CONFIG_SYCH) == 0) {
				 CommandAck = HEATER_CONFIG_SYNCH_ACK;
				 //Put this value in the variable for threshold offset value
				 printf("REMOTE_CMD_HEATER_CONFIG_SYCH \r\n");

				 // working upto thermo state problem..
				 // sprintf(reply_buff,"\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\"","cmd","hcs","type","get", "status","suc","lds",app_get_rgb_state(), "nls",app_is_night_light_auto_brightness_enabled(), "cs",app_get_night_light_config(),"afs",app_get_anti_freeze_status(),"dls",app_get_day_light_Saving_status(),"at",app_get_ambient_temp(),"st",app_get_target_temp(), "tu",app_get_temp_unit(),"hs",app_get_heater_state() );

#ifdef RGB_AUTO_ON_OFF_MODE
//				if(app_is_night_light_auto_brightness_enabled() == 1)
//				{	luchRGB_MODE = 1; }
//				else
//				{ 	if (rgb_led_state == 1)
//						luchRGB_MODE = 2;
//					else
//						luchRGB_MODE = 0; }

			    sprintf(reply_buff,"\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\"","cmd","hcs","type","get", "status","suc","lds",app_get_rgb_state(), "nls",app_is_night_light_auto_brightness_enabled(), "cs",app_get_night_light_config(),"afs",app_get_anti_freeze_status(),"dls",app_get_day_light_Saving_status(),"at",app_get_ambient_temp(),"st",app_get_target_temp(), "tu",app_get_temp_unit(),"hs", app_get_mode());

#else
				 // Working line for app get mode
				 sprintf(reply_buff,"\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\"","cmd","hcs","type","get", "status","suc","lds",app_get_rgb_state(), "nls",app_is_night_light_auto_brightness_enabled(), "cs",app_get_night_light_config(),"afs",app_get_anti_freeze_status(),"dls",app_get_day_light_Saving_status(),"at",app_get_ambient_temp(),"st",app_get_target_temp(), "tu",app_get_temp_unit(),"hs", app_get_mode());
#endif

				 // Testing LDS in for testing data packet in synch..
				// sprintf(reply_buff,"\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%s\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\",\"%s\":\"%d\"","cmd","hcs","type","get", "status","suc","lds",app_get_rgb_state(), "nls",app_is_night_light_auto_brightness_enabled(), "cs",app_get_light_LDR_parm(),"afs",app_get_anti_freeze_status(),"dls",app_get_day_light_Saving_status(),"at",app_get_ambient_temp(),"st",app_get_target_temp(), "tu",app_get_temp_unit(),"hs", app_get_mode());
	 }

	 else if (strcmp(label, REMOTE_CMD_PING_DEVICE) == 0) {
					 CommandAck = PING_DEVICE_ACK;
					 //Put this value in the variable for threshold offset value
					 pingDeviceOnFlag = 1;
					 printf("REMOTE_CMD_PING_DEVICE \r\n");
	 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "ping_device", "status","success",  "value",value);

					display_on();
					display_clear_screen();
					display_menu_small_font("ping device", DISPLAY_COLOR, "active", DISPLAY_COLOR);
					updateDisplayAfterAppCommand = 1;
	 }
	 else if (strcmp(label, REMOTE_CMD_AUTO_SCREEN_OFF) == 0) {
					 CommandAck = AUTO_SCREEN_OFF_EN_ACK;
					 //Put this value in the variable for threshold offset value

					 app_enable_auto_screen_off(atoi(value));

					 printf("REMOTE_CMD_AUTO_SCREEN_OFF \r\n");
	 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "auto_screen_off_en", "status","success",  "value",value);

					display_on();
					display_clear_screen();
					if(atoi(value))
						display_menu_small_font("auto screen off", DISPLAY_COLOR, "on", DISPLAY_COLOR);
					else
						display_menu_small_font("auto screen off", DISPLAY_COLOR, "off", DISPLAY_COLOR);
					updateDisplayAfterAppCommand = 1;

	     }
	 else if (strcmp(label, REMOTE_CMD_DELETE_HEATER) == 0) {
		     if (strcmp(value, "01") == 0){
		         CommandAck = DELETE_HEATER_ACK;
				 //Put this value in the variable for threshold offset value
				 printf("REMOTE_CMD_DELETE_HEATER \r\n");
	 			 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "delete_heater", "status","success",  "value",value);
	 			// app_delete_heater(atoi(value));
	 		 }
	 }
	 else if (strcmp(label, REMOTE_CMD_AUTO_DIM_PILOT_LIGHT_EN) == 0) {
						 CommandAck = AUTO_DIM_PILOT_EN_ACK;
						 //Put this value in the variable for threshold offset value
						 printf("REMOTE_CMD_AUTO_DIM_PILOT_LIGHT_EN \r\n");
		 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "dim_pilot_light_en", "status","success",  "value",value);
		 				app_enable_autodim_pilot_light(atoi(value));

						display_on();
						display_clear_screen();
						if(atoi(value))
							display_menu_small_font("dim pilot led", DISPLAY_COLOR, "on", DISPLAY_COLOR);
						else
							display_menu_small_font("dim pilot led", DISPLAY_COLOR, "off", DISPLAY_COLOR);
						updateDisplayAfterAppCommand = 1;

		 }
	 else if (strcmp(label, REMOTE_CMD_AUTO_DISPLAY_BRIGHTNESS_EN) == 0) {
						 CommandAck = AUTO_DISPLAY_BRIGHTNESS_EN_ACK;
						 //Put this value in the variable for threshold offset value
						 printf("REMOTE_CMD_AUTO_DISPLAY_BRIGHTNESS_EN \r\n");
		 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "auto_display_brightness_en", "status","success",  "value",value);
		 				app_enable_autodim_display(atoi(value));

						display_on();
						display_clear_screen();
						if(atoi(value))
							display_menu_small_font("auto display brightness", DISPLAY_COLOR, "on", DISPLAY_COLOR);
						else
							display_menu_small_font("auto display brightness", DISPLAY_COLOR, "off", DISPLAY_COLOR);
						updateDisplayAfterAppCommand = 1;

		 }

	 else if (strcmp(label, REMOTE_CMD_MANUAL_CHANGE_DISPLAY_BRIGHTNESS) == 0) {
	 						 CommandAck = MANUAL_CHANGE_DISPLAY_BRIGHTNESS_ACK;
	 						 //Put this value in the variable for threshold offset value
	 						 printf("REMOTE_CMD_MANUAL_CHANGE_DISPLAY_BRIGHTNESS  Value %s\r\n", value);
	 		 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "manual_change_display_brightness", "status","success",  "value",value);
	 		 				 app_set_screen_brightness(atoi(value));
							display_on();
							display_clear_screen();
							display_menu_small_font("manual change display brightness", DISPLAY_COLOR, value, DISPLAY_COLOR);
							updateDisplayAfterAppCommand = 1;

	 		 }

	 else if (strcmp(label, REMOTE_CMD_DELAY_AUTO_SCREEN_OFF) == 0) {
	 						 CommandAck = DELAY_AUTO_SCREEN_OFF_ACK;
	 						 //Put this value in the variable for threshold offset value
	 						 printf("REMOTE_CMD_DELAY_AUTO_SCREEN_OFF  Value %s\r\n", value);
	 		 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "delay_for_auto_screen_off", "status","success",  "value",value);
	 		 				app_set_auto_screen_off_delay(atoi(value));
							display_on();
							display_clear_screen();
							display_menu_small_font("delay auto screen off", DISPLAY_COLOR, value, DISPLAY_COLOR);
							updateDisplayAfterAppCommand = 1;
	 		 }

	 else if (strcmp(label, REMOTE_CMD_TIMER_MODE_COUNTER_MIN) == 0) {
		 						 CommandAck = SET_TIMER_MODE_MIN_ACK;
		 						 //Put this value in the variable for threshold offset value
		 						 printf("REMOTE_CMD_TIMER_MODE_COUNTER_MIN  Value %s\r\n", value);
		 		 				 sprintf(reply_buff, "\n \t\"%s\" : \"%s\", \n \t\"%s\" : \"%s\",\n \t\"%s\" : \"%s\",\n\t\"%s\" : \"%s\" ", "type", "set","cmd", "timer_mode_set_min", "status","success",  "value",value);
		 		 				 app_set_timer(atoi(value));
								display_on();
								display_clear_screen();
								display_menu_small_font("timer mode min", DISPLAY_COLOR, value, DISPLAY_COLOR);
								updateDisplayAfterAppCommand = 1;
		 		 }



	 else
	 {
        printf("unhandled label %s %s\r\n", label, value);
     }

    // CommandResponseOnDisplay();

    return SUCCESS;
}

//void CommandResponseOnDisplay(void)
//{
//	if(CommandAck){
//		display_on();
//		display_clear_screen();
//	    switch(CommandAck)
//		{
//		   case SET_TEMP_ACK:   	display_menu("set", DISPLAY_COLOR, "temp", DISPLAY_COLOR); break ;
//                    default : break;
//                }
//        }
//}



