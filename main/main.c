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



#include "config.h"

#include "string.h"

#include "esp_system.h" //esp_init funtions esp_err_t
#include "esp_wifi.h" //esp_wifi_init functions and wifi operations
#include "esp_log.h" //for showing logs
#include "esp_event.h" //for wifi event
#include "nvs_flash.h" //non volatile storage
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps
#include "esp_mac.h"

#include "mqtt_client.h" //provides important functions to connect with MQTT
#include "freertos/FreeRTOS.h" //it is important too if you want to run mqtt task independently and provides threads funtionality
#include "freertos/task.h" //MQTT communication often involves asynchronous operations, and FreeRTOS helps handle those tasks effectively
#include "esp_tls.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "mqtt_client.h"

#include <stddef.h>

const char * ssid = WIFI_SSID;
const char * pass = WIFI_PASS;

const char * cacert = CACERT;

char extension[4];
char phoneMac[18];

char * siptopic = NULL;
char * bttopic = NULL;
char * localtopic = NULL;

static bd_addr_t device_addr;

static char btname[9];

#define ZBLF_PERIOD_MS 1000

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

char* strconcat(char *str1, const char *str2)
{
  char *str = NULL;
  size_t len1 = 0;
  size_t len2 = 0;

  if (str1)
    len1 = strlen(str1);
  if (str2)
    len2 = strlen(str2);
  if (!(str = calloc(sizeof(char), (len1 + len2 + 1))))
    return NULL;
  if (str1)
    memcpy(str, str1, len1);
  if (str2)
    memcpy(str + len1, str2, len2);
  return (str);
}

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
  esp_netif_t *netif = esp_netif_create_default_wifi_sta(); //sets up necessary data structs for wifi station interface
  esp_netif_set_hostname(netif, strconcat(HOSTNAME_PREFIX, extension));
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

static void saveConfig(char * configLine) {
  int extnum = 0;
  sscanf( configLine, "%d,%s", &extnum, phoneMac );
  phoneMac[17] = '\0';
  printf( "Ext: %d ; Phone: %s\n", extnum, phoneMac);
  sprintf(extension, "%d", extnum);
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    printf("Done\n");

    printf("Saving Extension ");
    err = nvs_set_str(my_handle, "extension", extension);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    printf("Saving Phone MAC ");
    err = nvs_set_str(my_handle, "phonemac", phoneMac);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
    nvs_close(my_handle);

    for (int i = 10; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
  }

}


#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

esp_mqtt_client_handle_t mqttclient;

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

rmt_channel_handle_t led_chan = NULL;
rmt_tx_channel_config_t tx_chan_config = {
  .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
  .gpio_num = STATUSLED_GPIO,
  .mem_block_symbols = 64, // increase the block size can make the LED less flickering
  .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
  .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
};
rmt_encoder_handle_t led_encoder = NULL;
led_strip_encoder_config_t encoder_config = {
  .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
};
rmt_transmit_config_t tx_config = {
  .loop_count = 0, // no transfer loop
};

void setBTConfig(char * _bttopic, char * _localtopic, char * phoneMac, esp_mqtt_client_handle_t client, char * extension) {
  bttopic = _bttopic;
  localtopic = _localtopic;
  mqttclient = client;
  extension = extension;
  sprintf(btname, "zBLF-%s", extension);
  printf("setBTConfig(%s, %s, %s)\n", bttopic, localtopic, device_addr_string);
}

void initLeds() {
  rmt_new_tx_channel(&tx_chan_config, &led_chan);
  rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
  rmt_enable(led_chan);
}

static void setLedsColor(uint32_t red, uint32_t green, uint32_t blue ) {
  for (int j = 0; j < LED_NUMBERS; j++) {
    led_strip_pixels[j * 3 + 0] = green;
    led_strip_pixels[j * 3 + 1] = red;
    led_strip_pixels[j * 3 + 2] = blue;
  }
  // Flush RGB values to LEDs
  rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config);
  rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
}

int BTCallState = 0;
int BTCallSetup = 0;
int SIPCallState = 0;
int LocalState = 0;
static void setStateColor() {
  if (SIPCallState == 4) {
    printf("Leds WHITE\r\n");
    setLedsColor((int)LED_BRIGHTNESS/4 , (int)LED_BRIGHTNESS/4, (int)LED_BRIGHTNESS/4);
  } else if (SIPCallState == 3) {
    printf("Leds GREEN\r\n");
    setLedsColor(0 , (int)LED_BRIGHTNESS/4, 0);
  } else if ((BTCallState == 0 || BTCallState == 3) && SIPCallState == 0 && LocalState == 0) {
    printf("Leds OFF\r\n");
    setLedsColor(0, 0 ,0);
  } else if (SIPCallState == 1) {
    printf("Leds YELLOW\r\n");
    setLedsColor(LED_BRIGHTNESS, (int)LED_BRIGHTNESS/4 ,0);
  } else if (BTCallState == 1) {
    printf("Leds BLUE\r\n");
    setLedsColor(0, 0,LED_BRIGHTNESS);
  } else if (SIPCallState == 2) {
    printf("Leds RED\r\n");
    setLedsColor(LED_BRIGHTNESS, 0 ,0);
    //printf("Leds RED\r\n");
    //setLedsColor(LED_BRIGHTNESS, 0 ,0);
  } else if (BTCallState == 2) {
    printf("Leds PURPLE\r\n");
    setLedsColor(LED_BRIGHTNESS, 0 , (int)LED_BRIGHTNESS/4);
    //printf("Leds BLUE\r\n");
    //setLedsColor(0 ,0, LED_BRIGHTNESS);
  } else if (LocalState == 1) {
    printf("Leds RED\r\n");
    setLedsColor(LED_BRIGHTNESS, 0 ,0);
  } else {
    printf("Leds TURQUOISE\r\n");
    setLedsColor(0, LED_BRIGHTNESS ,LED_BRIGHTNESS);
  }
}

static void setBTCallStatus(int state) {
  if (state == 4 && BTCallState == 2) {
    state = 2;
  }
  BTCallState = state;
  printf("BTCallState=%d\r\n", BTCallState);
  setStateColor();
  if (state == 1) {
    esp_mqtt_client_publish(mqttclient, bttopic, "early", 0, 0, 1);
  } else if (state == 2) {
    esp_mqtt_client_publish(mqttclient, bttopic, "confirmed", 0, 0, 1);
  } else if (state == 3) {
    esp_mqtt_client_publish(mqttclient, bttopic, "unknown", 0, 0, 1);
  } else {
    esp_mqtt_client_publish(mqttclient, bttopic, "terminated", 0, 0, 1);
  }
}

void setSIPCallStatus(int state) {
  SIPCallState = state;
  printf("SIPCallState=%d\r\n", SIPCallState);
  setStateColor();
}

void toggleLocalStatus() {
  if (LocalState == 0) {
    LocalState = 1;
    esp_mqtt_client_publish(mqttclient, localtopic, "busy", 0, 0, 1);
  } else {
    LocalState = 0;
    esp_mqtt_client_publish(mqttclient, localtopic, "free", 0, 0, 1);
  }
  setStateColor();
}

#define TAG "mqtt_connecion"
#define TAG3 "mqtt_data"
extern void setSIPCallStatus(int state);
extern void toggleLightsStatus();
extern void setLightsColor(uint32_t red, uint32_t green, uint32_t blue);

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) { //here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client; //making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client
  if(event_id == MQTT_EVENT_CONNECTED){
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

    if (strncmp(extension, "xxx", 3) == 0) {
      esp_err_t ret = ESP_OK;
      uint8_t base_mac_addr[6];
      ret = esp_efuse_mac_get_default(base_mac_addr);
      if(ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Aborting");
        abort();
      }
      uint8_t index = 0;
      char macId[50];
      for(uint8_t i=0; i<6; i++){
        index += sprintf(&macId[index], "%02x", base_mac_addr[i]);
      }
      ESP_LOGI(TAG, "MAC address is %s\n", macId);
      char * conftopic = strconcat(MQTT_CONF_TOPIC, macId);
      esp_mqtt_client_subscribe(client, conftopic, 0);
    } else {
      printf("Subscribing to topic %s\n", siptopic);
      esp_mqtt_client_subscribe(client, siptopic, 0); //in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2

    }
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
    if (strncmp(event->topic, MQTT_CONF_TOPIC, strlen(MQTT_CONF_TOPIC)-1) == 0) {
      printf("CONFIG\r\n");
      saveConfig(event->data);
    } else if (strncmp(event->topic, MQTT_TOPIC, strlen(MQTT_TOPIC)-1) == 0) {
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
    }
  } else if(event_id == MQTT_EVENT_ERROR) {
    ESP_LOGI(TAG3, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
      ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
      ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
               strerror(event->error_handle->esp_transport_sock_errno));
    } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
      ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
    } else {
      ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
    }
  }
}

static esp_mqtt_client_handle_t mqtt_initialize(void) {/*Depending on your website or cloud there could be more parameters in mqtt_cfg.*/
  const esp_mqtt_client_config_t mqtt_cfg={
    .broker = {
      .address.uri = BROKER_URL, //Uniform Resource Identifier includes path,protocol
      .verification.certificate = cacert
    },
    .credentials = {
      .username=MQTT_USER, //your MQTT username
      .authentication.password=MQTT_PASS, //your MQTT password
    },
    .network = {
      refresh_connection_after_ms = 3600000,
    },
  };
  esp_mqtt_client_handle_t client=esp_mqtt_client_init(&mqtt_cfg); //sending struct as a parameter in init client function
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, (esp_event_handler_t)mqtt_event_handler, client);
  esp_mqtt_client_start(client); //starting the process
  return client;
}

static void nvsReadConfig () {
  printf("\n");
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    printf("Done\n");

    // Read
    printf("Reading extension from NVS ... ");
    size_t required_size;
    err = nvs_get_str(nvs_handle, "extension", NULL, &required_size);
    switch (err) {
      case ESP_OK:
        printf("Done\n");
        nvs_get_str(nvs_handle, "extension", extension, &required_size);
        printf("extension from config: %s\n", extension);
        err = nvs_get_str(nvs_handle, "phonemac", NULL, &required_size);
        if (err == ESP_OK) {
          nvs_get_str(nvs_handle, "phonemac", phoneMac, &required_size);
        }
        siptopic = strconcat(MQTT_TOPIC, extension);
        bttopic = strconcat(MQTT_BTTOPIC, extension);
        localtopic = strconcat(MQTT_LOCALTOPIC, extension);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        strcpy(extension, "xxx");
        break;
      default :
        printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
  }
}


void ble_ancs_init(void (*_statusCallback)(int), char * _btname);

int waitConnection = 0;
int connectionRetries = 0;
static void zblf_timer_callback(){
  //printf("\nTimer\n");
  if (connectionRetries > 5) {
    esp_restart();
  } else if (connectionStatus == 0) {
    connectionStatus = 2;
    waitConnection = 350;
    connectionRetries++;
    printf("Trying reconnect\n");
    hfp_hf_establish_service_level_connection(device_addr);
  } else if (connectionStatus == 2) {
    waitConnection--;
    //printf("\nWait %d\n", waitConnection);
    if (waitConnection <= 0) {
      connectionStatus = 0;
    }
  }
  int btn = gpio_get_level(BTNBTN_GPIO);
  if (btn == 0) {
    printf("GPIO LEVEL IS %d\n", btn);
    toggleLocalStatus();
  }
}

int app_main(void){
  esp_err_t ret;

  gpio_set_direction(BTNLED_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(BTNLED_GPIO, 1);

  gpio_set_direction(BTNBTN_GPIO, GPIO_MODE_INPUT);

  initLeds();
  nvs_flash_init(); // this is important in wifi case to store configurations , code will not work if this is not added
  nvsReadConfig();

  if (gpio_get_level(BTNBTN_GPIO) == 0) {
    strcpy(extension, "xxx");
    printf("ERASING CONFIG\n");
  }
  if (strncmp(extension, "xxx", 3) == 0) {
    setSIPCallStatus(4);
  } else {
    setSIPCallStatus(3);
  }
  wifi_connection();
  vTaskDelay(10000 /portTICK_PERIOD_MS); //delay is important cause we need to let it connect to wifi

  esp_mqtt_client_handle_t mqttclient = mqtt_initialize(); // MQTT start app as shown above most important code for MQTT

  if (strncmp(extension, "xxx", 3) == 0) {
    vTaskDelay(60000 /portTICK_PERIOD_MS);
  } else {
    vTaskDelay(10000 /portTICK_PERIOD_MS); //delay is important cause we need to let it connect to wifi
    setStateColor();

    setBTConfig(bttopic, localtopic, phoneMac, mqttclient, extension);

    setBTCallStatus(3);
    esp_mqtt_client_publish(mqttclient, localtopic, "free", 0, 0, 1);
    //hfp_hf_establish_service_level_connection(device_addr);
    waitConnection = 60;
    connectionStatus = 2;

    const esp_timer_create_args_t zblf_timer_args = {
            .callback = &zblf_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "zblf"
    };
    esp_timer_handle_t zblf_timer;
    ESP_ERROR_CHECK(esp_timer_create(&zblf_timer_args, &zblf_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(zblf_timer, ZBLF_PERIOD_M));

    ble_ancs_init(&setBTCallStatus, btname);
  }

  return 0;
}
