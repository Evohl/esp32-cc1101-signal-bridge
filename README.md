# ESP32 CC1101 Signal Bridge

An ESP32-based recorder and replay bridge for raw OOK/ASK radio signals. Recordings are stored in non-volatile memory and can be managed from a local web interface or triggered through MQTT. Home Assistant MQTT Discovery is supported, but Home Assistant is optional.

## Features

- Capture and replay raw pulse timings with a CC1101 transceiver.
- Name, inspect, edit, send, and delete recordings in the web interface.
- Keep recordings across reboots using ESP32 NVS.
- Export and import recordings as a portable JSON file.
- Publish one MQTT command button per recording, with Home Assistant MQTT Discovery.
- Update firmware over USB, ArduinoOTA, or the web interface.

## Hardware and limits

The PlatformIO target is an ESP32-DevKitC / ESP32-WROOM-32 and a 3.3 V CC1101 SPI module. This repository intentionally contains no wiring schematic or hardware images. Check the pin labels and electrical limits of the exact board and module before connecting them; never power a 3.3 V CC1101 module from 5 V.

The firmware records raw OOK/ASK pulse timing; it does not decode protocols. It cannot reproduce encrypted or rolling-code signals. Supported frequency ranges are 300-348 MHz, 387-464 MHz, and 779-928 MHz. The CC1101 module and antenna must support the selected band. Follow local rules for frequency, transmit power, and duty cycle, and only test equipment you own or are authorized to operate.

## Build and first start

Install PlatformIO, connect the ESP32 over USB, then build and flash:

```sh
pio run -e cc1101 -t upload
```

On first start without saved Wi-Fi settings, the bridge creates the `CC1101-Setup` access point. Connect using its documented setup password, then open `http://192.168.4.1/` and enter the Wi-Fi settings. MQTT settings are optional. After the bridge joins Wi-Fi, open `http://esp32-cc1101-<mac-suffix>.local/` or use its DHCP address. The lowercase hostname suffix is derived from the ESP32 MAC address.

## Recording and replay

1. Select a supported frequency and a capture duration from 1 to 10 seconds.
2. Start a capture while pressing the remote control button you want to record.
3. Choose a zone and signal name, then save the capture. The web page shows the pulse count and lets you inspect or edit the pulse sequence.
4. Use **Send** to replay a recording. Saved recordings remain in NVS after reboot.

The bridge supports up to 48 recordings and up to 600 pulses per recording. Recordings are raw radio data, not decoded commands; results depend on the original protocol, receiver, antenna, and radio conditions.

The signal manager includes the `office`, `livingroom`, `bedroom`, and `unassigned` zones. Create up to 16 zones with the **Create zone** form; saved zones remain available after reboot and appear in the signal forms. The zone list shows how many signals each zone contains. A zone can only be deleted when empty; delete its signals first. Signals are never moved automatically to `unassigned`.

## MQTT and Home Assistant

Configure an MQTT broker in the web interface to enable MQTT. Each bridge gets a stable six-character ID from its MAC address. Its default command base is `cc1101/<bridge-id>`; Home Assistant MQTT Discovery continues to use the standard `homeassistant` discovery prefix. Each saved recording is announced as a button. Its command topic is:

```text
cc1101/<bridge-id>/signal/<zone>/<name>/set
```

Send the payload `PRESS` to replay the recording. The bridge publishes availability at `cc1101/<bridge-id>/status`. A signal is identified by its zone and name, so the same name can be reused in different zones. Both fields accept 1-15 lowercase letters or digits, with underscores between characters. The canonical office names are `wc_power`, `wc_light`, `wc_light_mode`, `wc_level_1` through `wc_level_6`, `wc_direction`, `wc_timer_1h`, `wc_timer_2h`, `wc_timer_4h`, and `wc_mute`.

```text
cc1101/<bridge-id>/signal/office/wc_power/set
cc1101/<bridge-id>/signal/livingroom/wc_power/set
```

The first boot after this firmware update migrates existing recordings to the new storage format and canonical names. Labels containing `office`, `livingroom`, or `bedroom` determine the zone; other recordings remain in `unassigned`. Duplicate names in a zone are made unique with a numeric suffix. Pulse data is preserved, and zone and name can be changed in the signal editor.

### Example: Windcalm fan remote

The six office fan-speed buttons use `wc_level_1` through `wc_level_6`. A Home Assistant action for the first speed can publish:

```yaml
action: mqtt.publish
data:
	topic: cc1101/<bridge-id>/signal/office/wc_level_1/set
	payload: PRESS
```

The remaining speeds use the corresponding level number. New recordings use the zone and signal name entered in the recorder.

## Backup and transfer

On the signal page, choose **Export JSON** to download `cc1101-signals.json`. On another bridge running compatible firmware, select the file under **Import JSON file**. An imported signal with an existing zone and name updates that entry; other recordings are preserved. Older backups without a zone import into `unassigned`. The backup contains signal metadata and pulse data, not Wi-Fi or MQTT credentials.

## Firmware updates

For USB uploads, use the `cc1101` environment. For ArduinoOTA, use the target's hostname or IP:

```sh
pio run -e cc1101_ota -t upload --upload-port esp32-cc1101-<mac-suffix>.local
```

The web interface also provides a firmware upload page at `/firmware` for a PlatformIO application `.bin` file.

## Security

The web interface has no authentication, including its settings and firmware upload pages. Use it only on a trusted, isolated local network; do not expose it directly to the internet. The setup access-point password is a firmware default and is not a substitute for network security.