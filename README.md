# zblf-esp32

This project uses the esp-idf framework from Espressif https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

It is based on the hfp_hf_demo example of BTstack from BlueKitchen https://github.com/bluekitchen/btstack
Everything is built around it to add leds, wifi and mqtt.

You will also need the sip2mqtt https://github.com/zubrick/sip2mqtt app and a mqtt broker for it to work with VoIP status

## TODO

- [ ] Some Cleanup of parts of the example that are not needed
- [x] Add mqtts support
- [ ] Make the phone understand there is no sound (for now it auto disconnect the audio on audio connection)
- [ ] Auto reconnect to phone
- [ ] Configurable settings for led, wifi and mqtt
