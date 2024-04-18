
/*
 * Copyright (C) 2014 BlueKitchen GmbH
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
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
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
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "bts.c"

#include "config.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

/*
 * hfp_hs_demo.c
 */

// *****************************************************************************
/* EXAMPLE_START(hfp_hs_demo): HFP HF - Hands-Free
 *
 * @text This  HFP Hands-Free example demonstrates how to receive
 * an output from a remote HFP audio gateway (AG), and,
 * if HAVE_BTSTACK_STDIN is defined, how to control the HFP AG.
 */
// *****************************************************************************


#include "btstack_config.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "btstack.h"
#include "mqtt_client.h"

#include "bts.h"

//#undef HAVE_BTSTACK_STDIN

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

esp_mqtt_client_handle_t mqttclient;

uint8_t hfp_service_buffer[150];
const uint8_t   rfcomm_channel_nr = 1;
const char hfp_hf_service_name[] = "zBLF";

#define ZBLF_PERIOD_MS 1000
static btstack_timer_source_t zblf_timer;

static char * device_addr_string = NULL;
static char * bttopic = NULL;
static char * localtopic = NULL;
static char * extension = NULL;
static char btname[9];

static bd_addr_t device_addr;

#ifdef HAVE_BTSTACK_STDIN
// 80:BE:05:D5:28:48
// prototypes
static void show_usage(void);
#endif
static hci_con_handle_t acl_handle = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t sco_handle = HCI_CON_HANDLE_INVALID;

static int connectionStatus = 0;

static uint8_t codecs[] = {};

static uint16_t indicators[1] = {0x01};
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;
static char cmd;

static const uint8_t adv_data[] = {
    // Flags general discoverable, BR/EDR not supported
    0x02, 0x01, 0x06,
    // Name
    0x05, 0x09, 'Z', 'B', 'L', 'F',
    // Service Solicitation, 128-bit UUIDs - ANCS (little endian)
    0x11,0x15,0xD0,0x00,0x2D,0x12,0x1E,0x4B,0x0F,0xA4,0x99,0x4E,0xCE,0xB5,0x31,0xF4,0x05,0x79
};
static uint8_t adv_data_len = sizeof(adv_data);

static void dump_supported_codecs(void){
  printf("Supported codecs: CVSD");
  if (hci_extended_sco_link_supported()) {
#ifdef ENABLE_HFP_WIDE_BAND_SPEECH
    printf(", mSBC");
#endif
#ifdef ENABLE_HFP_SUPER_WIDE_BAND_SPEECH
    printf(", LC3-SWB");
#endif
    printf("\n");
  } else {
    printf("\nmSBC and/or LC3-SWB disabled as eSCO not supported by local controller.\n");
  }
}

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
  device_addr_string = phoneMac;
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

static void report_status(uint8_t status, const char * message){
  if (status != ERROR_CODE_SUCCESS){
    printf("%s command failed, status 0x%02x\n", message, status);
  } else {
    printf("%s command successful\n", message);
  }
}

static void app_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size){
  UNUSED(channel);
  bd_addr_t event_addr;

  switch (packet_type){

    case HCI_SCO_DATA_PACKET:
      // forward received SCO / audio packets to SCO component
      if (READ_SCO_CONNECTION_HANDLE(event) != sco_handle) break;
      break;

    case HCI_EVENT_PACKET:
      switch (hci_event_packet_get_type(event)){
        case BTSTACK_EVENT_STATE:
          // list supported codecs after stack has started up
          if (btstack_event_state_get_state(event) != HCI_STATE_WORKING) break;
          dump_supported_codecs();
          break;

        case HCI_EVENT_PIN_CODE_REQUEST:
          // inform about pin code request and respond with '0000'
          printf("Pin code request - using '0000'\n");
          hci_event_pin_code_request_get_bd_addr(event, event_addr);
          gap_pin_code_response(event_addr, "0000");
          break;

        case HCI_EVENT_SCO_CAN_SEND_NOW:
          break;

        case SM_EVENT_JUST_WORKS_REQUEST:
          sm_just_works_confirm(sm_event_just_works_request_get_handle(event));
          printf("Just Works Confirmed.\n");
          break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
          printf("Passkey display: %lu\n", sm_event_passkey_display_number_get_passkey(event));
          break;
        default:
          break;
      }
      break;

    default:
      break;
  }

}

int phoneNotification = 0;
static void ancs_callback(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    const char * attribute_name;
    const char * attribute_value;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_ANCS_META) return;
    switch (hci_event_ancs_meta_get_subevent_code(packet)){
        case ANCS_SUBEVENT_CLIENT_CONNECTED:
          connectionStatus = 1;
          gpio_set_level(BTNLED_GPIO, 0);
          setBTCallStatus(0);
          printf("ANCS Client: Connected\n");
          break;
        case ANCS_SUBEVENT_CLIENT_DISCONNECTED:
          connectionStatus = 0;
          gpio_set_level(BTNLED_GPIO, 1);
          setBTCallStatus(0);
          printf("ANCS Client: Disconnected\n");
          break;
        case ANCS_SUBEVENT_CLIENT_NOTIFICATION:
          attribute_name = ancs_client_attribute_name_for_id(ancs_subevent_client_notification_get_attribute_id(packet));
          attribute_value = ancs_subevent_client_notification_get_text(packet);
          if (!attribute_name) break;
          if (strcmp(attribute_name, "AppIdentifier") == 0 && strcmp(attribute_value, "com.apple.mobilephone") == 0) {
            printf("Notification: %s - %s\n", attribute_name, attribute_value);
            phoneNotification = 1;
          }
          if (strcmp(attribute_name, "IDMessage") == 0 && phoneNotification > 0) {
            printf("Notification: %s - %s\n", attribute_name, attribute_value);
            phoneNotification = 0;
            if(strcmp(attribute_value, "Appel entrant") == 0) {
              printf("Ringing\n");
              setBTCallStatus(1);
            } else if(strcmp(attribute_value, "Appel manqué") == 0) {
              printf("Hangup\n");
              setBTCallStatus(0);
            } else if(strcmp(attribute_value, "Appel en cours") == 0) {
              printf("On Phone\n");
              setBTCallStatus(2);
            } else {
              printf("Unidentified value\n");
            }
          }
          break;
        default:
            break;
    }
}


static void hfp_hf_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size){
  UNUSED(channel);
  uint8_t status;
  bd_addr_t event_addr;

  switch (packet_type){

    case HCI_SCO_DATA_PACKET:
      if (READ_SCO_CONNECTION_HANDLE(event) != sco_handle) break;
      break;

    case HCI_EVENT_PACKET:
      switch (hci_event_packet_get_type(event)){
        case BTSTACK_EVENT_STATE:
          if (btstack_event_state_get_state(event) != HCI_STATE_WORKING) break;
          dump_supported_codecs();
          break;

        case HCI_EVENT_PIN_CODE_REQUEST:
          // inform about pin code request
          printf("Pin code request - using '0000'\n");
          hci_event_pin_code_request_get_bd_addr(event, event_addr);
          gap_pin_code_response(event_addr, "0000");
          break;

        case HCI_EVENT_SCO_CAN_SEND_NOW:
          break;

        case HCI_EVENT_HFP_META:
          switch (hci_event_hfp_meta_get_subevent_code(event)) {
            case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_ESTABLISHED:
              status = hfp_subevent_service_level_connection_established_get_status(event);
              if (status != ERROR_CODE_SUCCESS){
                printf("Connection failed, status 0x%02x\n", status);
                break;
              }
              connectionStatus = 1;
              gpio_set_level(BTNLED_GPIO, 0);
              acl_handle = hfp_subevent_service_level_connection_established_get_acl_handle(event);
              hfp_subevent_service_level_connection_established_get_bd_addr(event, device_addr);
              printf("Service level connection established %s.\n\n", bd_addr_to_str(device_addr));
              break;
            case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_RELEASED:
              connectionStatus = 0;
              acl_handle = HCI_CON_HANDLE_INVALID;
              setBTCallStatus(3);
              gpio_set_level(BTNLED_GPIO, 1);
              printf("Service level connection released.\n\n");
              break;
            case HFP_SUBEVENT_AUDIO_CONNECTION_ESTABLISHED:

              status = hfp_subevent_audio_connection_established_get_status(event);
              if (status != ERROR_CODE_SUCCESS){
                printf("Audio connection establishment failed with status 0x%02x\n", status);
                break;
              }
              status =  hfp_hf_release_audio_connection(acl_handle);
              if (BTCallState != 1) {
                setBTCallStatus(2);
              }
              break;

            case HFP_SUBEVENT_CALL_ANSWERED:
              printf("Call answered\n");

              setBTCallStatus(2);
              break;

            case HFP_SUBEVENT_CALL_TERMINATED:
              printf("Call terminated\n");

              setBTCallStatus(0);
              break;

            case HFP_SUBEVENT_AUDIO_CONNECTION_RELEASED:
              sco_handle = HCI_CON_HANDLE_INVALID;
              printf("Audio connection released\n");
              break;
            case  HFP_SUBEVENT_COMPLETE:
              status = hfp_subevent_complete_get_status(event);
              if (status == ERROR_CODE_SUCCESS){
                printf("Cmd \'%c\' succeeded\n", cmd);
              } else {
                printf("Cmd \'%c\' failed with status 0x%02x\n", cmd, status);
              }
              break;

            case HFP_SUBEVENT_AG_INDICATOR_MAPPING:
              printf("AG Indicator Mapping | INDEX %d: range [%d, %d], name '%s'\n",
                     hfp_subevent_ag_indicator_mapping_get_indicator_index(event),
                     hfp_subevent_ag_indicator_mapping_get_indicator_min_range(event),
                     hfp_subevent_ag_indicator_mapping_get_indicator_max_range(event),
                     (const char*) hfp_subevent_ag_indicator_mapping_get_indicator_name(event));
              break;

            case HFP_SUBEVENT_AG_INDICATOR_STATUS_CHANGED:
              printf("AG Indicator Status  | INDEX %d: status 0x%02x, '%s'\n",
                     hfp_subevent_ag_indicator_status_changed_get_indicator_index(event),
                     hfp_subevent_ag_indicator_status_changed_get_indicator_status(event),
                     (const char*) hfp_subevent_ag_indicator_status_changed_get_indicator_name(event));

              if (hfp_subevent_ag_indicator_status_changed_get_indicator_index(event) == 2) {
                if (hfp_subevent_ag_indicator_status_changed_get_indicator_status(event) == 1) {
                  setBTCallStatus(2);
                } else if (hfp_subevent_ag_indicator_status_changed_get_indicator_status(event) == 0) {
                  setBTCallStatus(0);
                }
              } else if (hfp_subevent_ag_indicator_status_changed_get_indicator_index(event) == 3) {
                if (hfp_subevent_ag_indicator_status_changed_get_indicator_status(event) == 0 && BTCallState != 2) {
                  setBTCallStatus(0);
                } else if (hfp_subevent_ag_indicator_status_changed_get_indicator_status(event) == 2) {
                  setBTCallStatus(1);
                }
              }
              break;
            case HFP_SUBEVENT_NETWORK_OPERATOR_CHANGED:
              printf("NETWORK_OPERATOR_CHANGED, operator mode %d, format %d, name %s\n",
                     hfp_subevent_network_operator_changed_get_network_operator_mode(event),
                     hfp_subevent_network_operator_changed_get_network_operator_format(event),
                     (char *) hfp_subevent_network_operator_changed_get_network_operator_name(event));
              break;
            case HFP_SUBEVENT_EXTENDED_AUDIO_GATEWAY_ERROR:
              printf("EXTENDED_AUDIO_GATEWAY_ERROR_REPORT, status 0x%02x\n",
                     hfp_subevent_extended_audio_gateway_error_get_error(event));
              break;
            case HFP_SUBEVENT_START_RINGING:
              printf("** START Ringing **\n");
              setBTCallStatus(1);
              break;
            case HFP_SUBEVENT_RING:
              printf("** Ring **\n");
              setBTCallStatus(1);
              break;
            case HFP_SUBEVENT_STOP_RINGING:
              printf("** STOP Ringing **\n");

              if (BTCallState == 1) {
                setBTCallStatus(0);
              }

              break;
            case HFP_SUBEVENT_NUMBER_FOR_VOICE_TAG:
              printf("Phone number for voice tag: %s\n",
                     (const char *) hfp_subevent_number_for_voice_tag_get_number(event));
              break;
            case HFP_SUBEVENT_SPEAKER_VOLUME:
              printf("Speaker volume: gain %u\n",
                     hfp_subevent_speaker_volume_get_gain(event));
              break;
            case HFP_SUBEVENT_MICROPHONE_VOLUME:
              printf("Microphone volume: gain %u\n",
                     hfp_subevent_microphone_volume_get_gain(event));
              break;
            case HFP_SUBEVENT_CALLING_LINE_IDENTIFICATION_NOTIFICATION:
              printf("Caller ID, number '%s', alpha '%s'\n", (const char *) hfp_subevent_calling_line_identification_notification_get_number(event),
                     (const char *) hfp_subevent_calling_line_identification_notification_get_alpha(event));
              break;
            case HFP_SUBEVENT_ENHANCED_CALL_STATUS:
              printf("Enhanced call status:\n");
              printf("  - call index: %d \n", hfp_subevent_enhanced_call_status_get_clcc_idx(event));
              printf("  - direction : %s \n", hfp_enhanced_call_dir2str(hfp_subevent_enhanced_call_status_get_clcc_dir(event)));
              printf("  - status    : %s \n", hfp_enhanced_call_status2str(hfp_subevent_enhanced_call_status_get_clcc_status(event)));
              printf("  - mode      : %s \n", hfp_enhanced_call_mode2str(hfp_subevent_enhanced_call_status_get_clcc_mode(event)));
              printf("  - multipart : %s \n", hfp_enhanced_call_mpty2str(hfp_subevent_enhanced_call_status_get_clcc_mpty(event)));
              printf("  - type      : %d \n", hfp_subevent_enhanced_call_status_get_bnip_type(event));
              printf("  - number    : %s \n", hfp_subevent_enhanced_call_status_get_bnip_number(event));
              break;

            case HFP_SUBEVENT_VOICE_RECOGNITION_ACTIVATED:
              status = hfp_subevent_voice_recognition_activated_get_status(event);
              if (status != ERROR_CODE_SUCCESS){
                printf("Voice Recognition Activate command failed, status 0x%02x\n", status);
                break;
              }

              switch (hfp_subevent_voice_recognition_activated_get_enhanced(event)){
                case 0:
                  printf("\nVoice recognition ACTIVATED\n\n");
                  break;
                default:
                  printf("\nEnhanced voice recognition ACTIVATED.\n");
                  printf("Start new audio enhanced voice recognition session %s\n\n", bd_addr_to_str(device_addr));
                  status = hfp_hf_enhanced_voice_recognition_report_ready_for_audio(acl_handle);
                  break;
              }
              break;

            case HFP_SUBEVENT_VOICE_RECOGNITION_DEACTIVATED:
              status = hfp_subevent_voice_recognition_deactivated_get_status(event);
              if (status != ERROR_CODE_SUCCESS){
                printf("Voice Recognition Deactivate command failed, status 0x%02x\n", status);
                break;
              }
              printf("\nVoice Recognition DEACTIVATED\n\n");
              break;

            case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_HF_READY_FOR_AUDIO:
              status = hfp_subevent_enhanced_voice_recognition_hf_ready_for_audio_get_status(event);
              report_status(status, "Enhanced Voice recognition: READY FOR AUDIO");
              break;

            case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_READY_TO_ACCEPT_AUDIO_INPUT:
              printf("\nEnhanced Voice recognition AG status: AG READY TO ACCEPT AUDIO INPUT\n\n");
              break;
            case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_IS_STARTING_SOUND:
              printf("\nEnhanced Voice recognition AG status: AG IS STARTING SOUND\n\n");
              break;
            case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_IS_PROCESSING_AUDIO_INPUT:
              printf("\nEnhanced Voice recognition AG status: AG IS PROCESSING AUDIO INPUT\n\n");
              break;

            case HFP_SUBEVENT_ENHANCED_VOICE_RECOGNITION_AG_MESSAGE:
              printf("\nEnhanced Voice recognition AG message: \'%s\'\n", hfp_subevent_enhanced_voice_recognition_ag_message_get_text(event));
              break;

            case HFP_SUBEVENT_ECHO_CANCELING_AND_NOISE_REDUCTION_DEACTIVATE:
              status = hfp_subevent_echo_canceling_and_noise_reduction_deactivate_get_status(event);
              report_status(status, "Echo Canceling and Noise Reduction Deactivate");
              break;
            default:
              break;
          }
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }

}

int waitConnection = 0;
static void zblf_timer_handler(btstack_timer_source_t * ts){
  //printf("\nTimer\n");

  int btn = gpio_get_level(BTNBTN_GPIO);
  if (btn == 0) {
    printf("GPIO LEVEL IS %d\n", btn);
    toggleLocalStatus();
  }


  btstack_run_loop_set_timer(&zblf_timer, ZBLF_PERIOD_MS);
  btstack_run_loop_add_timer(&zblf_timer);
}

/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code.
 * To run a HFP HF service you need to initialize the SDP, and to create and register HFP HF record with it.
 * The packet_handler is used for sending commands to the HFP AG. It also receives the HFP AG's answers.
 * The stdin_process callback allows for sending commands to the HFP AG.
 * At the end the Bluetooth stack is started.
 */

/* LISTING_START(MainConfiguration): Setup HFP Hands-Free unit */
int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){
  (void)argc;
  (void)argv;


  setStateColor();

  // Init protocols
  // init L2CAP
  l2cap_init();

  // setup SM: Display only
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_ONLY);
  sm_set_authentication_requirements( SM_AUTHREQ_BONDING );

  // Register for HCI events
  hci_event_callback_registration.callback = &app_packet_handler;
  hci_add_event_handler(&hci_event_callback_registration);

  // register for SM events
  sm_event_callback_registration.callback = &app_packet_handler;
  sm_add_event_handler(&sm_event_callback_registration);

  // setup ATT server
  att_server_init(profile_data, NULL, NULL);

  // setup ANCS Client
  ancs_client_init();


  // register for ATT Serer events
  att_server_register_packet_handler(app_packet_handler);

  // setup GATT client
  gatt_client_init();

  // register for ancs events
  ancs_client_register_callback(&ancs_callback);

  /// Configure GAP - discovery / connection
  uint16_t adv_int_min = 0x0030;
  uint16_t adv_int_max = 0x0030;
  uint8_t adv_type = 0;
  bd_addr_t null_addr;
  memset(null_addr, 0, 6);
  gap_set_local_name(btname);
  gap_discoverable_control(1);
  gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
  gap_advertisements_enable(1);


  // turn on!
  hci_power_control(HCI_POWER_ON);

  setBTCallStatus(3);
  esp_mqtt_client_publish(mqttclient, localtopic, "free", 0, 0, 1);
  //hfp_hf_establish_service_level_connection(device_addr);

  printf("btstack_run_loop_set_timer_handler()...\n");
  btstack_run_loop_set_timer_handler(&zblf_timer, zblf_timer_handler);
  printf("btstack_run_loop_set_timer()...\n");
  btstack_run_loop_set_timer(&zblf_timer, ZBLF_PERIOD_MS);
  printf("btstack_run_loop_add_timer()...\n");
  btstack_run_loop_add_timer(&zblf_timer);

  return 0;
}
/* LISTING_END */
/* EXAMPLE_END */
