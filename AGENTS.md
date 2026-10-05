# ESP32 CC1101 Signal Bridge

## Purpose
Own the ESP32 firmware that captures and replays raw OOK/ASK signals through a CC1101 transceiver and exposes saved signals through MQTT.

## Ownership
- `src/main.cpp` owns firmware behavior and the local web interface.
- `configs/cc1101-signals.json` is the checked-in signal backup.
- `platformio.ini` and `partitions_large_nvs.csv` define the build target and storage layout.

## Local Contracts
- Signal IDs are unique within a zone and are used in MQTT command topics; keep saved IDs aligned with panel/dashboard references.
- Signal data is raw pulse timing, not decoded protocol data; encrypted and rolling-code signals are unsupported.
- Keep one persistent global CC1101 transmit-power preset (low, standard, high, maximum) in the local settings UI; standard preserves the existing default. Treat displayed dBm as approximate and retain the local-frequency and antenna compliance warning.
- Keep Wi-Fi, MQTT, and web-interface credentials out of the signal backup. Treat the web UI and OTA as private-LAN services.

## Work Guidance

## Verification
Build the firmware with `pio run -e cc1101` from this directory. No automated firmware test suite is defined.

## Child DOX Index
None.