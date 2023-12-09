/*
 * Copyright (C) 2020 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 *  main.c
 *
 *  Minimal main application that initializes BTstack, prepares the example and enters BTstack's Run Loop.
 *
 *  If needed, you can create other threads. Please note that BTstack's API is not thread-safe and can only be
 *  called from BTstack timers or in response to its callbacks, e.g. packet handlers.
 */

#include "config.h"

#include "btstack_port_esp32.h"
#include "btstack_run_loop.h"
#include "btstack_stdio_esp32.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"

#include "string.h"

#include "esp_system.h" //esp_init funtions esp_err_t 
#include "esp_wifi.h" //esp_wifi_init functions and wifi operations
#include "esp_log.h" //for showing logs
#include "esp_event.h" //for wifi event
#include "nvs_flash.h" //non volatile storage
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps

#include "mqtt_client.h" //provides important functions to connect with MQTT
#include "freertos/FreeRTOS.h" //it is important too if you want to run mqtt task independently and provides threads funtionality
#include "freertos/task.h" //MQTT communication often involves asynchronous operations, and FreeRTOS helps handle those tasks effectively


const char *ssid = WIFI_SSID;
const char *pass = WIFI_PASS;

#include <stddef.h>

// warn about unsuitable sdkconfig
#include "sdkconfig.h"
#if !CONFIG_BT_ENABLED
#error "Bluetooth disabled - please set CONFIG_BT_ENABLED via menuconfig -> Component Config -> Bluetooth -> [x] Bluetooth"
#endif
#if !CONFIG_BT_CONTROLLER_ONLY
#error "Different Bluetooth Host stack selected - please set CONFIG_BT_CONTROLLER_ONLY via menuconfig -> Component Config -> Bluetooth -> Host -> Disabled"
#endif
#if ESP_IDF_VERSION_MAJOR >= 5
#if !CONFIG_BT_CONTROLLER_ENABLED
#error "Different Bluetooth Host stack selected - please set CONFIG_BT_CONTROLLER_ENABLED via menuconfig -> Component Config -> Bluetooth -> Controller -> Enabled"
#endif
#endif

int retry_num = 0;
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    if(event_id == WIFI_EVENT_STA_START) {
        printf("WIFI CONNECTING....\n");
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        printf("WiFi CONNECTED\n");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("WiFi lost connection\n");
        if(retry_num<5) {
            esp_wifi_connect();
            retry_num++;
            printf("Retrying to Connect...\n");
        }
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        printf("Wifi got IP...\n\n");
    }
}

void wifi_connection(){
    esp_netif_init(); //network interdace initialization
    esp_event_loop_create_default(); //responsible for handling and dispatching events
    esp_netif_create_default_wifi_sta(); //sets up necessary data structs for wifi station interface
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();//sets up wifi wifi_init_config struct with default values
    esp_wifi_init(&wifi_initiation); //wifi initialised with dafault wifi_initiation
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);//creating event handler register for wifi
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);//creating event handler register for ip event
    wifi_config_t wifi_configuration ={ //struct wifi_config_t var wifi_configuration
        .sta= {
            .ssid = "",
            .password= "" /*we are sending a const char of ssid and password which we will strcpy in following line so leaving it blank*/ 
        }//also this part is used if you donot want to use Kconfig.projbuild
    };
    strcpy((char*)wifi_configuration.sta.ssid,ssid); // copy chars from hardcoded configs to struct
    strcpy((char*)wifi_configuration.sta.password,pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);//setting up configs when event ESP_IF_WIFI_STA
    esp_wifi_start();//start connection with configurations provided in funtion
    esp_wifi_set_mode(WIFI_MODE_STA);//station mode selected
    esp_wifi_connect(); //connect with saved ssid and pass
    printf( "wifi_init_softap finished. SSID:%s  password:%s",ssid,pass);
}

#define TAG "mqtt_connecion"
#define TAG3 "mqtt_data"
extern void setSIPCallStatus(int state);

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) { //here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client; //making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client
    if(event_id == MQTT_EVENT_CONNECTED){
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client,MQTT_TOPIC,0); //in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
        ESP_LOGI(TAG3, "sent subscribe successful" );
    } else if(event_id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG3, "MQTT_EVENT_DISCONNECTED"); //if disconnected
    } else if(event_id == MQTT_EVENT_SUBSCRIBED) {
        ESP_LOGI(TAG3, "MQTT_EVENT_SUBSCRIBED");
    } else if(event_id == MQTT_EVENT_UNSUBSCRIBED)  {
        ESP_LOGI(TAG3, "MQTT_EVENT_UNSUBSCRIBED");
    } else if(event_id == MQTT_EVENT_DATA) {
        ESP_LOGI(TAG3, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        if (strncmp(event->data, "confirmed", event->data_len) == 0) {
            printf("state confirmed\r\n");
            setSIPCallStatus(2);            
        } else if (strncmp(event->data, "early", event->data_len) == 0) {
            printf("state early\r\n");
            setSIPCallStatus(1);            
        } else {
            printf("state terminated\r\n");
            setSIPCallStatus(0);  
        }
    } else if(event_id == MQTT_EVENT_ERROR) {
        ESP_LOGI(TAG3, "MQTT_EVENT_ERROR");
    }
}

static void mqtt_initialize(void) {/*Depending on your website or cloud there could be more parameters in mqtt_cfg.*/
    const esp_mqtt_client_config_t mqtt_cfg={
        .broker.address.uri = BROKER_URL, //Uniform Resource Identifier includes path,protocol
        .credentials = {
            .username=MQTT_USER, //your username
            .authentication.password=MQTT_PASS, //your adafruit io password
        }
    };
    esp_mqtt_client_handle_t client=esp_mqtt_client_init(&mqtt_cfg); //sending struct as a parameter in init client function
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, (esp_event_handler_t)mqtt_event_handler, client);
    esp_mqtt_client_start(client); //starting the process
}

extern int btstack_main(int argc, const char * argv[]);

int app_main(void){
    nvs_flash_init(); // this is important in wifi case to store configurations , code will not work if this is not added
    wifi_connection();  
    vTaskDelay(10000 /portTICK_PERIOD_MS); //delay is important cause we need to let it connect to wifi 
    mqtt_initialize(); // MQTT start app as shown above most important code for MQTT   

    // optional: enable packet logger
    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Enable buffered stdout
    btstack_stdio_init();

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // Setup example
    btstack_main(0, NULL);

    // Enter run loop (forever)
    btstack_run_loop_execute();

    return 0;
}
