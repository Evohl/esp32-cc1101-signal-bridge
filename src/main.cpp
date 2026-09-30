#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <vector>

#ifndef DEVICE_HOSTNAME_SUFFIX
#define DEVICE_HOSTNAME_SUFFIX 0
#endif

namespace {
constexpr uint8_t PIN_CC_CS = 5;
constexpr uint8_t PIN_CC_GDO0 = 4;
constexpr uint8_t PIN_CC_GDO2 = 27;
constexpr uint8_t PIN_CC_SCK = 18;
constexpr uint8_t PIN_CC_MISO = 19;
constexpr uint8_t PIN_CC_MOSI = 23;
constexpr uint8_t CC_IOCFG2 = 0x00;
constexpr uint16_t MAX_PULSES = 600;
constexpr uint8_t MAX_SIGNALS = 48;
constexpr uint8_t MAX_ZONES = 16;
constexpr uint8_t MAX_LOG_ENTRIES = 30;
constexpr uint8_t MAX_CAPTURE_HISTORY = 2;
constexpr uint32_t MQTT_BUFFER_SIZE = 768;
constexpr uint32_t CAPTURE_CARRIER_HOLD_US = 20000;
constexpr uint8_t DEFAULT_LNA_GAIN_REDUCTION_STEP = 3;
char hostname[32];
char bridgeId[7];
constexpr char AP_NAME[] = "CC1101-Setup";
constexpr char MQTT_DISCOVERY_PREFIX[] = "homeassistant";

constexpr uint8_t CC_IOCFG0 = 0x02;
constexpr uint8_t CC_FREQ2 = 0x0D;
constexpr uint8_t CC_FREQ1 = 0x0E;
constexpr uint8_t CC_FREQ0 = 0x0F;
constexpr uint8_t CC_MDMCFG4 = 0x10;
constexpr uint8_t CC_MDMCFG3 = 0x11;
constexpr uint8_t CC_MDMCFG2 = 0x12;
constexpr uint8_t CC_MDMCFG1 = 0x13;
constexpr uint8_t CC_MDMCFG0 = 0x14;
constexpr uint8_t CC_DEVIATN = 0x15;
constexpr uint8_t CC_MCSM0 = 0x18;
constexpr uint8_t CC_FOCCFG = 0x19;
constexpr uint8_t CC_BSCFG = 0x1A;
constexpr uint8_t CC_FREND0 = 0x22;
constexpr uint8_t CC_AGCCTRL2 = 0x1B;
constexpr uint8_t CC_AGCCTRL1 = 0x1C;
constexpr uint8_t CC_AGCCTRL0 = 0x1D;
constexpr uint8_t CC_RSSI = 0x34;
constexpr uint8_t CC_FSCAL3 = 0x23;
// CC_RSSI (status register 0x34) and CC_SRX (strobe command 0x34) share the same
// address; the SPI header bits distinguish them, so both constants are kept.
constexpr uint8_t CC_FSCAL2 = 0x24;
constexpr uint8_t CC_FSCAL1 = 0x25;
constexpr uint8_t CC_FSCAL0 = 0x26;
constexpr uint8_t CC_TEST2 = 0x2C;
constexpr uint8_t CC_TEST1 = 0x2D;
constexpr uint8_t CC_TEST0 = 0x2E;
constexpr uint8_t CC_PARTNUM = 0x30;
constexpr uint8_t CC_VERSION = 0x31;
constexpr uint8_t CC_PATABLE_BURST = 0x7E;
constexpr uint8_t CC_SRES = 0x30;
constexpr uint8_t CC_SRX = 0x34;
constexpr uint8_t CC_STX = 0x35;
constexpr uint8_t CC_SIDLE = 0x36;

struct StoredSignal {
  char zone[16];
  char id[16];
  char label[33];
  float frequencyMHz;
  uint16_t count;
  uint16_t durations[MAX_PULSES];
  uint8_t levels[MAX_PULSES];
};

struct LegacyStoredSignal {
  char id[12];
  char label[33];
  float frequencyMHz;
  uint16_t count;
  uint16_t durations[MAX_PULSES];
  uint8_t levels[MAX_PULSES];
};

struct LogEntry {
  uint32_t atMs;
  char message[80];
};

struct PulseCapture {
  uint32_t id;
  float frequencyMHz;
  uint16_t count;
  uint16_t durations[MAX_PULSES];
  uint8_t levels[MAX_PULSES];
};

Preferences preferences;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);
std::vector<StoredSignal> signals;
std::vector<String> zones;
String wifiSsid;
String wifiPassword;
String mqttHost;
String mqttUser;
String mqttPassword;
String mqttBase;
uint16_t mqttPort = 1883;
bool webStarted = false;
bool otaStarted = false;
bool wifiHostnameSet = false;
bool mdnsStarted = false;
uint32_t lastMqttAttempt = 0;
int lastMqttFailureState = -1;
float activeFrequencyMHz = 433.92f;
String lastAction = "Ready";
bool cc1101Detected = false;
uint8_t cc1101PartNumber = 0xFF;
uint8_t cc1101Version = 0xFF;
LogEntry eventLog[MAX_LOG_ENTRIES]{};
uint8_t eventLogNext = 0;
uint8_t eventLogCount = 0;

volatile uint32_t captureLastEdgeUs = 0;
volatile uint32_t captureLastCarrierUs = 0;
volatile uint16_t captureCount = 0;
volatile uint16_t captureDurations[MAX_PULSES];
volatile uint8_t captureLevels[MAX_PULSES];
volatile uint8_t captureCurrentLevel = LOW;
volatile bool captureFirstEdge = true;
bool captureActive = false;
uint32_t captureDeadline = 0;
uint32_t captureSequence = 0;
float captureFrequencyMHz = 433.92f;
uint8_t captureGainReductionStep = DEFAULT_LNA_GAIN_REDUCTION_STEP;
PulseCapture pulseHistory[MAX_CAPTURE_HISTORY]{};
uint8_t pulseHistoryCount = 0;

void logEvent(const String& message) {
  LogEntry& entry = eventLog[eventLogNext];
  entry.atMs = millis();
  strlcpy(entry.message, message.c_str(), sizeof(entry.message));
  eventLogNext = (eventLogNext + 1) % MAX_LOG_ENTRIES;
  if (eventLogCount < MAX_LOG_ENTRIES) eventLogCount++;
  Serial.println(message);
}

void IRAM_ATTR captureEdge() {
  const uint32_t now = micros();
  if (digitalRead(PIN_CC_GDO2)) {
    captureLastCarrierUs = now;
  } else if (now - captureLastCarrierUs > CAPTURE_CARRIER_HOLD_US) {
    captureFirstEdge = true;
    return;
  }

  const uint8_t level = (uint8_t)digitalRead(PIN_CC_GDO0);
  if (captureFirstEdge) {
    captureLastEdgeUs = now;
    captureCurrentLevel = level;
    captureFirstEdge = false;
    return;
  }

  const uint32_t duration = now - captureLastEdgeUs;
  const uint16_t index = captureCount;
  if (index < MAX_PULSES && duration > 0 && duration <= 65535) {
    captureDurations[index] = (uint16_t)duration;
    captureLevels[index] = captureCurrentLevel;
    captureCount = index + 1;
  }
  captureCurrentLevel = level;
  captureLastEdgeUs = now;
}

void ccSelect() {
  digitalWrite(PIN_CC_CS, LOW);
  delayMicroseconds(2);
}

void ccDeselect() {
  digitalWrite(PIN_CC_CS, HIGH);
  delayMicroseconds(2);
}

void ccStrobe(uint8_t command) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  ccSelect();
  SPI.transfer(command);
  ccDeselect();
  SPI.endTransaction();
}

void ccWriteRegister(uint8_t address, uint8_t value) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  ccSelect();
  SPI.transfer(address);
  SPI.transfer(value);
  ccDeselect();
  SPI.endTransaction();
}

uint8_t ccReadStatusRegister(uint8_t address) {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  ccSelect();
  const uint32_t readyStarted = micros();
  while (digitalRead(PIN_CC_MISO) == HIGH && micros() - readyStarted < 1000) {}
  if (digitalRead(PIN_CC_MISO) == HIGH) {
    ccDeselect();
    SPI.endTransaction();
    return 0xFF;
  }
  SPI.transfer(address | 0xC0);
  const uint8_t value = SPI.transfer(0x00);
  ccDeselect();
  SPI.endTransaction();
  return value;
}

bool isKnownCc1101Version(uint8_t version) {
  return version == 0x04 || version == 0x14 || version == 0x17 || version == 0x03;
}

void ccSetConservativeOokPower() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  ccSelect();
  SPI.transfer(CC_PATABLE_BURST);
  SPI.transfer(0x00);
  SPI.transfer(0x60);
  ccDeselect();
  SPI.endTransaction();
}

void ccSetFrequency(float frequencyMHz) {
  const uint32_t frequencyWord = (uint32_t)(frequencyMHz * 65536.0f / 26.0f);
  ccWriteRegister(CC_FREQ2, (uint8_t)(frequencyWord >> 16));
  ccWriteRegister(CC_FREQ1, (uint8_t)(frequencyWord >> 8));
  ccWriteRegister(CC_FREQ0, (uint8_t)frequencyWord);
}

void ccConfigure(float frequencyMHz, bool receive) {
  ccStrobe(CC_SIDLE);
  ccSetFrequency(frequencyMHz);
  ccWriteRegister(CC_IOCFG2, 0x0E);
  ccWriteRegister(CC_IOCFG0, 0x0D);
  ccWriteRegister(0x08, 0x31);
  ccWriteRegister(0x07, 0x00);
  ccWriteRegister(0x06, 0xFF);
  ccWriteRegister(CC_MDMCFG4, 0xAA);
  ccWriteRegister(CC_MDMCFG3, 0xF8);
  ccWriteRegister(CC_MDMCFG2, 0x30);
  ccWriteRegister(CC_FREND0, 0x11);
  ccWriteRegister(CC_MDMCFG1, 0x22);
  ccWriteRegister(CC_MDMCFG0, 0xF8);
  ccWriteRegister(CC_DEVIATN, 0x15);
  ccWriteRegister(CC_MCSM0, 0x18);
  ccWriteRegister(CC_FOCCFG, 0x16);
  ccWriteRegister(CC_BSCFG, 0x6C);
  // MAX_DVGA_GAIN=1, MAX_LNA_GAIN=captureGainReductionStep (0-7), MAGN_TARGET=3 (33 dB).
  ccWriteRegister(CC_AGCCTRL2, (uint8_t)(0x40 | (captureGainReductionStep << 3) | 0x03));
  // CARRIER_SENSE_ABS_THR fixed at +7 dB; noise rejection comes from the LNA gain limit above.
  ccWriteRegister(CC_AGCCTRL1, 0x47);
  ccWriteRegister(CC_AGCCTRL0, 0x91);
  ccWriteRegister(CC_FSCAL3, 0xE9);
  ccWriteRegister(CC_FSCAL2, 0x2A);
  ccWriteRegister(CC_FSCAL1, 0x00);
  ccWriteRegister(CC_FSCAL0, 0x1F);
  ccWriteRegister(CC_TEST2, 0x81);
  ccWriteRegister(CC_TEST1, 0x35);
  ccWriteRegister(CC_TEST0, 0x09);
  ccSetConservativeOokPower();

  if (receive) {
    pinMode(PIN_CC_GDO0, INPUT);
    ccStrobe(CC_SRX);
  }
}

bool validFrequency(float frequencyMHz) {
  return (frequencyMHz >= 300.0f && frequencyMHz <= 348.0f) ||
         (frequencyMHz >= 387.0f && frequencyMHz <= 464.0f) ||
         (frequencyMHz >= 779.0f && frequencyMHz <= 928.0f);
}

String htmlEscape(const String& value) {
  String escaped = value;
  escaped.replace("&", "&amp;");
  escaped.replace("<", "&lt;");
  escaped.replace(">", "&gt;");
  escaped.replace("\"", "&quot;");
  escaped.replace("'", "&#39;");
  return escaped;
}

bool validSignalId(const String& id) {
  if (id.isEmpty() || id.length() > 15 ||
      !((id[0] >= 'a' && id[0] <= 'z') || (id[0] >= '0' && id[0] <= '9')) ||
      !((id[id.length() - 1] >= 'a' && id[id.length() - 1] <= 'z') ||
        (id[id.length() - 1] >= '0' && id[id.length() - 1] <= '9'))) return false;
  for (size_t index = 0; index < id.length(); index++) {
    const char ch = id[index];
    if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_')) return false;
  }
  return true;
}

String signalIdentity(const String& zone, const String& id) {
  return zone + "/" + id;
}

bool parseSignalIdentity(const String& identity, String& zone, String& id) {
  const int separator = identity.indexOf('/');
  if (separator < 1 || separator == identity.length() - 1 ||
      identity.indexOf('/', separator + 1) >= 0) return false;
  zone = identity.substring(0, separator);
  id = identity.substring(separator + 1);
  return validSignalId(zone) && validSignalId(id);
}

String zoneForLabel(const String& originalLabel) {
  String label = originalLabel;
  label.toLowerCase();
  if (label.indexOf("office") >= 0) return "office";
  if (label.indexOf("livingroom") >= 0 || label.indexOf("living_room") >= 0) return "livingroom";
  if (label.indexOf("bedroom") >= 0) return "bedroom";
  return "unassigned";
}

String actionNameForLabel(const String& originalLabel, const String& currentId) {
  String label = originalLabel;
  label.toLowerCase();
  if (currentId.startsWith("wc_") && validSignalId(currentId)) return currentId;
  if (label == "windcalm_office_power_switch" || currentId == "windcalm_of") return "wc_power";
  if (label == "windcalm_office_light_switch" || currentId == "windcalm__2") return "wc_light";
  if (label == "windcalm_office_light_mode" || currentId == "windcalm__3") return "wc_light_mode";
  const String levelPrefix = "windcalm_office_vent_level_";
  if (label.startsWith(levelPrefix) && label.length() == levelPrefix.length() + 1 &&
      label[label.length() - 1] >= '1' && label[label.length() - 1] <= '6') {
    return "wc_level_" + label.substring(label.length() - 1);
  }
  if (currentId.startsWith("windcalm__") && currentId.length() == 11 &&
      currentId[10] >= '4' && currentId[10] <= '9') {
    return "wc_level_" + String(currentId[10] - '3');
  }
  if (currentId == "windcalm_13") return "wc_direction";
  if (currentId == "windcalm_10") return "wc_timer_1h";
  if (currentId == "windcalm_11") return "wc_timer_2h";
  if (currentId == "windcalm_12") return "wc_timer_4h";
  if (currentId == "windcalm_14") return "wc_mute";

  const String prefix = "windcalm_";
  if (label.startsWith(prefix)) label.remove(0, prefix.length());
  const int separator = label.indexOf('_');
  if (separator >= 0) {
    const String possibleZone = label.substring(0, separator);
    if (possibleZone == "office" || possibleZone == "livingroom" || possibleZone == "bedroom") {
      label.remove(0, separator + 1);
    }
  }
  if (label == "power_switch") return "wc_power";
  if (label == "light_switch") return "wc_light";
  if (label == "light_mode") return "wc_light_mode";
  if (label.startsWith("vent_level_") && label.length() == 12 &&
      label[11] >= '1' && label[11] <= '6') {
    return "wc_level_" + label.substring(11);
  }
  if (label == "direction" || label == "fan_direction") return "wc_direction";
  if (label == "timer_1h") return "wc_timer_1h";
  if (label == "timer_2h") return "wc_timer_2h";
  if (label == "timer_4h") return "wc_timer_4h";
  if (label == "mute" || label == "mute_switch") return "wc_mute";
  if (label.length() > 12) label = label.substring(label.length() - 12);
  if (label.isEmpty()) label = "signal";
  return "wc_" + label;
}

bool signalKeyAvailable(const String& zone, const String& id, size_t exceptIndex = (size_t)-1);

String uniqueActionName(const String& zone, const String& proposed) {
  String candidate = proposed;
  uint8_t suffix = 2;
  while (true) {
    bool duplicate = false;
    for (const StoredSignal& signal : signals) {
      if (zone == signal.zone && candidate == signal.id) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate && signalKeyAvailable(zone, candidate)) return candidate;
    const String suffixText = "_" + String(suffix++);
    const size_t baseLength = candidate.length() + suffixText.length() <= 15
      ? candidate.length()
      : 15 - suffixText.length();
    candidate = proposed.substring(0, baseLength) + suffixText;
  }
}

String oldActionNameForId(const String& id) {
  if (id == "wc_power") return "windcalm_of";
  if (id == "wc_light") return "windcalm__2";
  if (id == "wc_light_mode") return "windcalm__3";
  if (id.startsWith("wc_level_") && id.length() == 10 && id[9] >= '1' && id[9] <= '6') {
    return "windcalm__" + String((uint8_t)(id[9] - '0' + 3));
  }
  if (id == "wc_direction") return "windcalm_13";
  if (id == "wc_timer_1h") return "windcalm_10";
  if (id == "wc_timer_2h") return "windcalm_11";
  if (id == "wc_timer_4h") return "windcalm_12";
  if (id == "wc_mute") return "windcalm_14";
  return id;
}

bool sameSignal(const StoredSignal& first, const StoredSignal& second) {
  if (strcmp(first.zone, second.zone) != 0 || first.count < 8 || first.count != second.count ||
      first.frequencyMHz < second.frequencyMHz - 0.05f ||
      first.frequencyMHz > second.frequencyMHz + 0.05f) return false;

  for (uint16_t index = 0; index < first.count; index++) {
    if (first.levels[index] != second.levels[index]) return false;
    const uint16_t firstDuration = first.durations[index];
    const uint16_t secondDuration = second.durations[index];
    const uint16_t difference = firstDuration > secondDuration
      ? firstDuration - secondDuration
      : secondDuration - firstDuration;
    const uint16_t relativeTolerance = (uint16_t)(firstDuration / 5);
    const uint16_t tolerance = relativeTolerance > 100 ? relativeTolerance : 100;
    if (difference > tolerance) return false;
  }
  return true;
}

String signalKey(const String& zone, const String& id) {
  uint32_t hash = 2166136261UL;
  const String identity = signalIdentity(zone, id);
  for (size_t index = 0; index < identity.length(); index++) {
    hash = (hash ^ (uint8_t)identity[index]) * 16777619UL;
  }
  char key[12];
  snprintf(key, sizeof(key), "z_%08lx", (unsigned long)hash);
  return key;
}

bool signalKeyAvailable(const String& zone, const String& id, size_t exceptIndex) {
  const String key = signalKey(zone, id);
  for (size_t index = 0; index < signals.size(); index++) {
    const StoredSignal& signal = signals[index];
    if (index != exceptIndex && signalKey(signal.zone, signal.id) == key &&
        (zone != signal.zone || id != signal.id)) return false;
  }
  return true;
}

bool saveSignalIndex() {
  JsonDocument document;
  JsonArray entries = document.to<JsonArray>();
  for (const StoredSignal& signal : signals) {
    JsonObject entry = entries.add<JsonObject>();
    entry["zone"] = signal.zone;
    entry["id"] = signal.id;
    entry["label"] = signal.label;
    entry["frequency"] = signal.frequencyMHz;
  }
  String json;
  serializeJson(document, json);
  return preferences.putString("index", json) == json.length();
}

bool zoneExists(const String& zone) {
  for (const String& existing : zones) {
    if (existing == zone) return true;
  }
  return false;
}

bool addZone(const String& zone) {
  if (!validSignalId(zone) || zoneExists(zone) || zones.size() >= MAX_ZONES) return false;
  zones.push_back(zone);
  return true;
}

bool saveZones() {
  JsonDocument document;
  JsonArray entries = document.to<JsonArray>();
  for (const String& zone : zones) entries.add(zone);
  String json;
  serializeJson(document, json);
  if (preferences.getString("zones", "") == json) return true;
  return preferences.putString("zones", json) == json.length();
}

void loadZones() {
  zones.clear();
  const String savedZones = preferences.getString("zones", "");
  if (savedZones.isEmpty()) {
    addZone("office");
    addZone("livingroom");
    addZone("bedroom");
    addZone("unassigned");
  }
  JsonDocument document;
  if (!savedZones.isEmpty() && !deserializeJson(document, savedZones)) {
    for (JsonVariantConst entry : document.as<JsonArrayConst>()) {
      const String zone = entry.as<String>();
      addZone(zone);
    }
  }
  for (const StoredSignal& signal : signals) addZone(signal.zone);
  saveZones();
}

String renderZoneOptions(const String& selectedZone) {
  String options;
  for (const String& zone : zones) {
    options += "<option value='" + htmlEscape(zone) + "'";
    if (zone == selectedZone) options += " selected";
    options += ">" + htmlEscape(zone) + "</option>";
  }
  return options;
}

String renderZoneList() {
  String list = "<ul class='zone-list'>";
  for (const String& zone : zones) {
    uint8_t signalCount = 0;
    for (const StoredSignal& signal : signals) {
      if (zone == signal.zone) signalCount++;
    }
    list += "<li><span><b>" + htmlEscape(zone) + "</b><small class='muted'> · " +
      String(signalCount) + (signalCount == 1 ? " signal" : " signals") + "</small></span>"
      "<form method='post' action='/zone/delete'";
    if (signalCount == 0) list += " onsubmit=\"return confirm('Delete this zone?')\"";
    list += "><input type='hidden' name='zone' value='" + htmlEscape(zone) + "'>";
    if (signalCount == 0) list += "<button>Delete</button>";
    else list += "<button disabled title='Delete the signals in this zone first'>Delete</button>";
    list += "</form></li>";
  }
  list += "</ul>";
  return list;
}

void loadSignals() {
  signals.clear();
  JsonDocument document;
  if (deserializeJson(document, preferences.getString("index", "[]"))) return;
  std::vector<String> obsoleteKeys;
  bool indexChanged = false;
  bool migrationIncomplete = false;
  for (JsonObjectConst entry : document.as<JsonArrayConst>()) {
    const String id = entry["id"] | "";
    if (!validSignalId(id) || signals.size() >= MAX_SIGNALS) continue;
    String zone = entry["zone"] | "";
    if (zone.isEmpty()) {
      LegacyStoredSignal legacy{};
      const String legacyKey = "s_" + id;
      if (preferences.getBytesLength(legacyKey.c_str()) != sizeof(legacy) ||
          preferences.getBytes(legacyKey.c_str(), &legacy, sizeof(legacy)) != sizeof(legacy) ||
          legacy.count > MAX_PULSES) {
        migrationIncomplete = true;
        continue;
      }
      const String label = legacy.label;
      zone = zoneForLabel(label);
      const String actionId = uniqueActionName(zone, actionNameForLabel(label, id));
      StoredSignal migrated{};
      strlcpy(migrated.zone, zone.c_str(), sizeof(migrated.zone));
      strlcpy(migrated.id, actionId.c_str(), sizeof(migrated.id));
      strlcpy(migrated.label, actionId.c_str(), sizeof(migrated.label));
      migrated.frequencyMHz = legacy.frequencyMHz;
      migrated.count = legacy.count;
      memcpy(migrated.durations, legacy.durations, sizeof(migrated.durations));
      memcpy(migrated.levels, legacy.levels, sizeof(migrated.levels));
      const String key = signalKey(zone, actionId);
      if (!signalKeyAvailable(zone, actionId) ||
          preferences.putBytes(key.c_str(), &migrated, sizeof(migrated)) != sizeof(migrated)) {
        migrationIncomplete = true;
        continue;
      }
      signals.push_back(migrated);
      obsoleteKeys.push_back(legacyKey);
      indexChanged = true;
      continue;
    }
    if (!validSignalId(zone)) continue;
    if (!signalKeyAvailable(zone, id)) continue;
    StoredSignal signal{};
    const String key = signalKey(zone, id);
    if (preferences.getBytesLength(key.c_str()) != sizeof(signal)) continue;
    if (preferences.getBytes(key.c_str(), &signal, sizeof(signal)) != sizeof(signal)) continue;
    if (signal.count > MAX_PULSES || strcmp(signal.zone, zone.c_str()) != 0 ||
        strcmp(signal.id, id.c_str()) != 0) continue;
    const String newZone = zone == "unassigned" ? zoneForLabel(signal.label) : zone;
    const String newId = uniqueActionName(newZone, actionNameForLabel(signal.label, id));
    if (newZone != zone || newId != id || String(signal.label) != newId) {
      strlcpy(signal.zone, newZone.c_str(), sizeof(signal.zone));
      strlcpy(signal.id, newId.c_str(), sizeof(signal.id));
      strlcpy(signal.label, newId.c_str(), sizeof(signal.label));
      const String newKey = signalKey(newZone, newId);
      if (preferences.putBytes(newKey.c_str(), &signal, sizeof(signal)) != sizeof(signal)) {
        migrationIncomplete = true;
        continue;
      }
      if (newKey != key) obsoleteKeys.push_back(key);
      indexChanged = true;
    }
    signals.push_back(signal);
  }
  if (indexChanged && !migrationIncomplete && saveSignalIndex()) {
    for (const String& key : obsoleteKeys) preferences.remove(key.c_str());
    logEvent("Migrated saved signals to consistent zone-based names");
  }
}

String discoveryObjectId(const String& zone, const String& signalId) {
  return String("cc1101_") + bridgeId + "_" + zone + "_" + signalId;
}

String discoveryConfigTopic(const String& objectId) {
  return String(MQTT_DISCOVERY_PREFIX) + "/button/" + objectId + "/config";
}

String legacyDiscoveryConfigTopic(const String& signalId) {
  return String(MQTT_DISCOVERY_PREFIX) + "/button/cc1101_" + signalId + "/config";
}

void publishDiscovery(const StoredSignal& signal) {
  if (!mqttClient.connected()) return;
  const String objectId = discoveryObjectId(signal.zone, signal.id);
  const String discoveryTopic = discoveryConfigTopic(objectId);
  const String commandTopic = mqttBase + "/signal/" + signal.zone + "/" + signal.id + "/set";
  const String legacyTopic = legacyDiscoveryConfigTopic(signal.id);
  const String oldDiscoveryTopic = discoveryConfigTopic(String("cc1101_") + bridgeId + "_" + signal.id);
  const String previousId = oldActionNameForId(signal.id);
  const String previousLegacyTopic = legacyDiscoveryConfigTopic(previousId);
  const String previousDiscoveryTopic = discoveryConfigTopic(String("cc1101_") + bridgeId + "_" + previousId);
  char payload[MQTT_BUFFER_SIZE];
  JsonDocument document;
  document["name"] = signal.label;
  document["unique_id"] = objectId;
  document["object_id"] = objectId;
  document["command_topic"] = commandTopic;
  document["payload_press"] = "PRESS";
  document["availability_topic"] = mqttBase + "/status";
  document["payload_available"] = "online";
  document["payload_not_available"] = "offline";
  JsonObject device = document["device"].to<JsonObject>();
  JsonArray identifiers = device["identifiers"].to<JsonArray>();
  identifiers.add(String("cc1101_bridge_") + bridgeId + "_" + signal.zone);
  device["name"] = String("CC1101 Bridge ") + bridgeId + " " + signal.zone;
  device["manufacturer"] = "ESP32 / CC1101";
  device["model"] = "OOK signal bridge";
  const size_t length = serializeJson(document, payload, sizeof(payload));
  if (length > 0) {
    mqttClient.publish(legacyTopic.c_str(), "", true);
    if (previousLegacyTopic != legacyTopic) mqttClient.publish(previousLegacyTopic.c_str(), "", true);
    if (oldDiscoveryTopic != discoveryTopic) mqttClient.publish(oldDiscoveryTopic.c_str(), "", true);
    if (previousDiscoveryTopic != discoveryTopic && previousDiscoveryTopic != oldDiscoveryTopic) {
      mqttClient.publish(previousDiscoveryTopic.c_str(), "", true);
    }
    mqttClient.publish(discoveryTopic.c_str(), payload, true);
  }
}

void publishAllDiscovery() {
  for (const StoredSignal& signal : signals) publishDiscovery(signal);
}

void sendSignal(const String& zone, const String& id) {
  for (const StoredSignal& signal : signals) {
    if (zone != signal.zone || id != signal.id || signal.count == 0) continue;
    ccConfigure(signal.frequencyMHz, false);
    pinMode(PIN_CC_GDO0, OUTPUT);
    digitalWrite(PIN_CC_GDO0, signal.levels[0] ? HIGH : LOW);
    ccStrobe(CC_STX);
    for (uint16_t index = 0; index < signal.count; index++) {
      digitalWrite(PIN_CC_GDO0, signal.levels[index] ? HIGH : LOW);
      delayMicroseconds(signal.durations[index]);
    }
    digitalWrite(PIN_CC_GDO0, LOW);
    ccStrobe(CC_SIDLE);
    pinMode(PIN_CC_GDO0, INPUT);
    ccStrobe(CC_SRX);
    lastAction = "Sent: " + String(signal.zone) + "/" + String(signal.label);
    logEvent(lastAction);
    return;
  }
  lastAction = "Signal not found: " + signalIdentity(zone, id);
  logEvent(lastAction);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  const String receivedTopic(topic);
  const String prefix = mqttBase + "/signal/";
  if (!receivedTopic.startsWith(prefix) || !receivedTopic.endsWith("/set")) return;
  String command;
  command.reserve(length);
  for (unsigned int index = 0; index < length; index++) command += (char)payload[index];
  command.trim();
  command.toUpperCase();
  if (command != "PRESS" && command != "ON" && command != "1") return;
  const String identity = receivedTopic.substring(prefix.length(), receivedTopic.length() - 4);
  String zone;
  String id;
  if (parseSignalIdentity(identity, zone, id)) sendSignal(zone, id);
}

void reconnectMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqttHost.isEmpty() || mqttClient.connected()) return;
  if (millis() - lastMqttAttempt < 5000) return;
  lastMqttAttempt = millis();
  const String clientId = String(hostname) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  const String availabilityTopic = mqttBase + "/status";
  bool connected = false;
  if (mqttUser.isEmpty()) {
    connected = mqttClient.connect(clientId.c_str(), availabilityTopic.c_str(), 0, true, "offline");
  } else {
    connected = mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str(),
                                   availabilityTopic.c_str(), 0, true, "offline");
  }
  if (connected) {
    mqttClient.publish(availabilityTopic.c_str(), "online", true);
    mqttClient.subscribe((mqttBase + "/signal/+/+/set").c_str());
    publishAllDiscovery();
    lastAction = "MQTT connected";
    logEvent("MQTT connected");
    lastMqttFailureState = -1;
  } else {
    const int state = mqttClient.state();
    if (state != lastMqttFailureState) {
      logEvent("MQTT connection failed (rc " + String(state) + ")");
      lastMqttFailureState = state;
    }
  }
}

String pageStart(const String& title) {
  return String("<!doctype html><html lang='en'><meta charset='utf-8'>") +
    "<meta name='viewport' content='width=device-width,initial-scale=1'><title>" + htmlEscape(title) +
    " · CC1101</title><style>body{font:16px system-ui,sans-serif;max-width:900px;margin:0 auto;padding:18px;"
    "background:#101820;color:#e8eff2}header{padding:10px 0 12px;border-bottom:1px solid #52616b}.brand{display:block;font-weight:700;margin-bottom:10px}"
    "nav{display:flex;align-items:center;flex-wrap:nowrap;overflow-x:auto;gap:8px;padding:0 0 4px;white-space:nowrap;scrollbar-width:thin}"
    "a{color:#71d6c5}nav a{display:inline-block;padding:8px 12px;border-radius:4px;background:#71d6c5;color:#102126;text-decoration:none}"
    "nav a:hover{background:#8be3d4}.danger-button{background:#d85a56;color:#fff}.danger-button:hover{background:#e6726d}"
    ".signal-actions button{height:36px;box-sizing:border-box}"
    ".signal-actions .send-signal,.signal-actions .record-signal{display:inline-flex;align-items:center;justify-content:center;"
    "width:40px;height:36px;box-sizing:border-box;padding:0;background:#71d6c5;color:#102126;border:0;line-height:1}"
    ".action-icon{display:block;position:relative;top:-1px;line-height:1}"
    ".record-icon{font-size:28px;color:#d9363e}"
    ".signal-actions .send-signal:hover,.signal-actions .record-signal:hover{background:#8be3d4}"
    "main{padding-top:16px}.panel{padding:14px 0;border-bottom:1px solid #394952}"
    ".signal-row{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:8px 0}"
    ".signal-info{min-width:0;flex:1}.signal-info h3{display:inline;margin:0 8px 0 0;font-size:1em}"
    ".signal-info p{display:inline;margin:0;font-size:.88em}.signal-actions{display:flex;gap:4px;flex-shrink:0}"
    ".signal-actions form{margin:0}.signal-actions button{padding:6px 10px;margin:0}"
    "@media(max-width:560px){.signal-row{flex-wrap:wrap}.signal-info{flex-basis:100%}.signal-actions{margin-left:auto}}"
    "input{font:inherit;max-width:100%;box-sizing:border-box}"
    "input:not([type=range]):not([type=hidden]):not([type=file]),select,textarea{font:inherit;"
    "color:#e8eff2;background:#18242c;border:1px solid #52616b;border-radius:4px;box-sizing:border-box;color-scheme:dark}"
    "input:not([type=range]):not([type=hidden]):not([type=file]),select{min-height:42px;padding:9px;"
    "margin:4px 4px 4px 0;max-width:100%}input[type=range]{min-height:0;padding:0}"
    "button,.button-link,input[type=file]::file-selector-button{display:inline-block;font:inherit;padding:9px 14px;"
    "margin:4px 4px 4px 0;border:0;"
    "border-radius:4px;background:#71d6c5;color:#102126;text-decoration:none;cursor:pointer}"
    "button:hover,.button-link:hover,input[type=file]::file-selector-button:hover{background:#8be3d4}"
    "small,.muted{color:#a7b5bc}.page-footer{margin-top:24px}"
    "form{margin:8px 0}.row{display:flex;gap:8px;flex-wrap:wrap}.row label{display:block}"
    ".setup-form{max-width:680px}.setup-section{padding:14px 0;border-bottom:1px solid #394952}"
    ".setup-section h2{font-size:1.1em;margin:0 0 12px}.form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,260px),1fr));gap:12px 16px}"
    ".form-grid label{display:flex;flex-direction:column;gap:5px;color:#a7b5bc;font-size:.92em}"
    ".form-grid input:not([type=range]):not([type=hidden]):not([type=file]),.form-grid select{width:100%;margin:0}"
    ".zone-list{list-style:none;padding:0;margin:0 0 12px;max-width:680px}"
    ".zone-list li{display:flex;align-items:center;justify-content:space-between;gap:16px;"
    "padding:8px 0;border-bottom:1px solid #394952}.zone-list form{margin:0}.zone-list small{margin-left:5px}"
    ".form-actions{padding-top:14px}"
    ".file-picker{display:flex;align-items:center;gap:8px;flex-wrap:wrap}"
    ".file-picker span{color:#a7b5bc;overflow-wrap:anywhere}"
    ".recorder-step{padding:12px 0;border-bottom:1px solid #394952}"
    ".recorder-step h3{font-size:1em;margin:0 0 10px;color:#71d6c5}.capture-state{padding:10px 12px;margin-top:10px;"
    "background:#18242c;border-left:3px solid #71d6c5;min-height:22px}.capture-state.warn{border-color:#e6ae61}"
    ".pulse-preview{margin-top:10px;padding:10px;background:#18242c}.pulse-waveform{display:block;width:100%;height:120px;background:#101820}"
    ".pulse-values{max-height:140px;overflow:auto;font:12px ui-monospace,monospace;overflow-wrap:anywhere}"
    ".log-list{max-height:360px;overflow:auto;background:#101820;border:1px solid #394952;padding:8px 12px;font:13px ui-monospace,monospace}"
    ".log-entry{padding:4px 0;border-bottom:1px solid #26343c;overflow-wrap:anywhere}.log-entry:last-child{border:0}"
    "button:disabled{opacity:.45;cursor:not-allowed}"
    "code{overflow-wrap:anywhere}</style><header><b class='brand'>" + String(hostname) + "</b><nav><a href='/'>Home</a><a href='/signals'>Signals</a>"
    "<a href='/log'>Log</a><a href='/zones'>Zones</a><a href='/backup'>Backup</a><a href='/settings'>Network</a>"
    "<a href='/firmware'>Firmware</a><a href='/system'>System</a></nav></header><main>";
}

String pageEnd() {
  return "</main></html>";
}

void handleZonesPage() {
  const String body = "<h1>Zone configuration</h1><section class='panel'><h2>Zones</h2>" + renderZoneList() +
    "<form method='post' action='/zone/create' class='row'>"
    "<label>New zone <input name='zone' maxlength='15' pattern='[a-z0-9]+(_+[a-z0-9]+)*'"
    " title='1-15 lowercase letters or digits, with underscores between characters' required></label>"
    "<button>Create zone</button></form></section>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Zones") + body + pageEnd());
}

void handleBackupPage() {
  const String body = "<h1>Signal backup</h1><section class='panel'><h2>Back up / transfer signals</h2><div class='row'>"
  "<a class='button-link' href='/signals/export' download>Export JSON</a>"
  "<div class='file-picker'><button id='signal-file-select' type='button' aria-controls='signal-import-file'>Choose file</button>"
  "<span id='signal-import-file-name' aria-live='polite'>No file selected</span>"
  "<input id='signal-import-file' type='file' accept='.json,application/json' hidden></div>"
  "<button id='signal-import-button' type='button'>Import</button></div>"
  "<p id='signal-import-state' class='muted' aria-live='polite'></p></section>"
  "<script>const signalFileInput=document.getElementById('signal-import-file');"
  "document.getElementById('signal-file-select').addEventListener('click',()=>signalFileInput.click());"
  "signalFileInput.addEventListener('change',()=>{document.getElementById('signal-import-file-name').textContent="
  "signalFileInput.files.length?signalFileInput.files[0].name:'No file selected';});"
  "document.getElementById('signal-import-button').addEventListener('click',async()=>{"
  "const state=document.getElementById('signal-import-state');"
  "if(!signalFileInput.files.length){state.textContent='Select a JSON file first';return;}"
  "const button=document.getElementById('signal-import-button');button.disabled=true;"
  "try{const backup=JSON.parse(await signalFileInput.files[0].text());"
  "if(backup.format!=='cc1101-signals'||backup.version!==1||!Array.isArray(backup.signals))"
  "throw new Error('Unsupported file format or version');"
  "let imported=0;for(const signal of backup.signals){const response=await fetch('/signal/import',{method:'POST',"
  "headers:{'Content-Type':'application/json'},body:JSON.stringify(signal)});"
  "if(!response.ok)throw new Error((await response.text())+' ('+imported+' of '+backup.signals.length+' imported)');"
  "imported++;state.textContent='Imported: '+imported+' of '+backup.signals.length;}"
  "state.textContent='Import complete: '+imported+' signals';location.reload();}"
  "catch(error){state.textContent='Import failed: '+error.message;}finally{button.disabled=false;}});</script>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Backup") + body + pageEnd());
}

String settingsForm(bool firstSetup) {
  const String intro = firstSetup
    ? "<h1>Set up Wi-Fi</h1><p>Connect to the <b>" + String(AP_NAME) +
      "</b> access point (password: <code>cc1101setup</code>). MQTT can be configured later.</p>"
      "<p>After saving, the bridge restarts. Reconnect to your Wi-Fi and open <code>http://" +
      String(hostname) + ".local/</code>. If that name does not resolve, use the IP shown in your router.</p>"
    : "<h1>Network / MQTT</h1>";
  const String wifiName = firstSetup ? "" : htmlEscape(wifiSsid);
  const String wifiPasswordHint = firstSetup ? "" : " placeholder='blank = unchanged'";
  const String mqttPasswordHint = firstSetup ? "" : " placeholder='blank = unchanged'";
  return intro +
    "<form class='setup-form' method='post' action='/settings'>"
    "<section class='setup-section'><h2>Wi-Fi</h2><div class='form-grid'>"
    "<label>Network name<input name='ssid' value='" + wifiName + "' required autocomplete='off'></label>"
    "<label>Wi-Fi password<input type='password' name='wifi_password'" + wifiPasswordHint + " autocomplete='new-password'></label>"
    "</div></section><section class='setup-section'><h2>MQTT</h2><div class='form-grid'>"
    "<label>Broker address<input name='mqtt_host' value='" + htmlEscape(mqttHost) + "' placeholder='e.g. 192.168.1.10'></label>"
    "<label>Port<input type='number' name='mqtt_port' min='1' max='65535' value='" + String(mqttPort) + "'></label>"
    "<label>Username<input name='mqtt_user' value='" + htmlEscape(mqttUser) + "' autocomplete='off'></label>"
    "<label>MQTT password<input type='password' name='mqtt_password'" + mqttPasswordHint + " autocomplete='new-password'></label>"
    "</div></section><div class='form-actions'><button>" +
    String(firstSetup ? "Save and connect" : "Save and restart") +
    "</button></div></form>" +
    (firstSetup ? "" : "<p class='muted'>Leave password fields blank to keep the saved passwords.</p>");
}

String renderSignalRow(const StoredSignal& signal) {
  return "<section class='panel signal-row'><div class='signal-info'><h3>" + htmlEscape(signal.label) + "</h3><p class='muted'>" +
    htmlEscape(signal.zone) + " · " + htmlEscape(signal.id) + " · " +
    String(signal.frequencyMHz, 2) + " MHz · " + String(signal.count) + " pulses</p></div><div class='signal-actions'>"
    "<form method='post' action='/send' class='send-form'>"
    "<input type='hidden' name='id' value='" + htmlEscape(signalIdentity(signal.zone, signal.id)) + "'><button class='send-signal' aria-label='Send signal' title='Send signal'>&#9654;</button></form>"
    "<button type='button' class='record-signal' aria-label='Re-record signal' title='Re-record signal' data-zone='" +
    htmlEscape(signal.zone) + "' data-id='" + htmlEscape(signal.id) + "' data-frequency='" + String(signal.frequencyMHz, 2) + "'><span class='action-icon record-icon' aria-hidden='true'>●</span></button>"
    "<form method='get' action='/signal'>" +
    "<input type='hidden' name='id' value='" + htmlEscape(signalIdentity(signal.zone, signal.id)) + "'><button>Edit</button></form>" +
    "<form method='post' action='/delete' onsubmit=\"return confirm('Delete signal?')\">" +
    "<input type='hidden' name='id' value='" + htmlEscape(signalIdentity(signal.zone, signal.id)) + "'><button class='danger-button'>Delete</button></form></div></section>";
}

void sendSignalList() {
  if (signals.empty()) {
    webServer.sendContent("<p class='muted'>No signals saved yet.</p>");
    return;
  }
  for (const String& zone : zones) {
    bool hasSignals = false;
    for (const StoredSignal& signal : signals) {
      if (zone != signal.zone) continue;
      if (!hasSignals) {
        webServer.sendContent("<h2 class='zone-heading'>" + htmlEscape(zone) + "</h2>");
        hasSignals = true;
      }
      webServer.sendContent(renderSignalRow(signal));
    }
  }
}

void handleSignalsExport() {
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.sendHeader("Content-Disposition", "attachment; filename=cc1101-signals.json");
  webServer.send(200, "application/json; charset=utf-8",
                 "{\"format\":\"cc1101-signals\",\"version\":1,\"signals\":[");
  bool first = true;
  for (const StoredSignal& signal : signals) {
    if (!first) webServer.sendContent(",");
    first = false;
    JsonDocument document;
    document["zone"] = signal.zone;
    document["id"] = signal.id;
    document["label"] = signal.label;
    document["frequencyMHz"] = signal.frequencyMHz;
    JsonArray durations = document["durations"].to<JsonArray>();
    JsonArray levels = document["levels"].to<JsonArray>();
    for (uint16_t index = 0; index < signal.count; index++) {
      durations.add(signal.durations[index]);
      levels.add(signal.levels[index]);
    }
    String signalJson;
    serializeJson(document, signalJson);
    webServer.sendContent(signalJson);
  }
  webServer.sendContent("]}");
  webServer.sendContent("");
}

String renderSignalEditor(const String& identity) {
  String zone;
  String id;
  if (!parseSignalIdentity(identity, zone, id)) {
    return pageStart("Signal not found") + "<h1>Signal not found</h1><p><a href='/signals'>Back</a></p>" + pageEnd();
  }
  for (const StoredSignal& signal : signals) {
    if (zone != signal.zone || id != signal.id) continue;

    String pulseData;
    pulseData.reserve((size_t)signal.count * 9);
    for (uint16_t index = 0; index < signal.count; index++) {
      pulseData += signal.levels[index] ? "H:" : "L:";
      pulseData += String(signal.durations[index]);
      pulseData += '\n';
    }

    String body = "<h1>Edit signal</h1><p class='muted'>" + String(signal.count) +
      " pulses · Logic level and duration in microseconds, one entry per line.</p>"
      "<form method='post' action='/signal/save' class='setup-form'>"
      "<input type='hidden' name='id' value='" + htmlEscape(signalIdentity(signal.zone, signal.id)) + "'>"
      "<section class='setup-section'><div class='form-grid'>"
      "<label>Zone<select name='zone' required>" + renderZoneOptions(signal.zone) + "</select></label>"
      "<label>Signal name<input name='name' maxlength='15' pattern='[a-z0-9]+(_+[a-z0-9]+)*' value='" + htmlEscape(signal.id) + "' title='1-15 lowercase letters or digits, with underscores between characters' required></label>"
      "<label>Frequency (MHz)<input type='number' name='frequency' min='300' max='928' step='0.01' value='" +
      String(signal.frequencyMHz, 2) + "' required></label></div></section>"
      "<section class='setup-section'><label>Pulse sequence (H:us or L:us, one per line)"
      "<textarea name='pulse_data' rows='18' spellcheck='false' required style='display:block;width:100%;box-sizing:border-box;"
      "background:#18242c;color:#e8eff2;border:1px solid #52616b;padding:10px;font:13px ui-monospace,monospace'>" +
      htmlEscape(pulseData) + "</textarea></label></section>"
      "<div class='form-actions'><button>Save changes</button> <a href='/signals'>Cancel</a></div></form>";
    return pageStart("Edit signal") + body + pageEnd();
  }
  return pageStart("Signal not found") + "<h1>Signal not found</h1><p><a href='/signals'>Back</a></p>" + pageEnd();
}

void handleSignalImport() {
  JsonDocument document;
  const DeserializationError error = deserializeJson(document, webServer.arg("plain"));
  if (error) {
    webServer.send(400, "text/plain", "Invalid JSON");
    return;
  }

  const String sourceId = document["id"] | "";
  const String sourceLabel = document["label"] | sourceId;
  const String zone = document["zone"] | zoneForLabel(sourceLabel);
  const String id = actionNameForLabel(sourceLabel, sourceId);
  const String label = id;
  const float frequency = document["frequencyMHz"] | 0.0f;
  JsonArrayConst durations = document["durations"].as<JsonArrayConst>();
  JsonArrayConst levels = document["levels"].as<JsonArrayConst>();
  if (!validSignalId(zone) || !validSignalId(id) || label.isEmpty() || label.length() > 32 ||
      !validFrequency(frequency) || durations.size() < 4 || durations.size() > MAX_PULSES ||
      levels.size() != durations.size()) {
    webServer.send(400, "text/plain", "Invalid signal metadata or pulse sequence");
    return;
  }
  if (!zoneExists(zone)) {
    if (!addZone(zone)) {
      webServer.send(409, "text/plain", "Maximum number of zones reached.");
      return;
    }
    if (!saveZones()) {
      zones.pop_back();
      webServer.send(500, "text/plain", "Could not save zone list.");
      return;
    }
  }

  StoredSignal imported{};
  strlcpy(imported.zone, zone.c_str(), sizeof(imported.zone));
  strlcpy(imported.id, id.c_str(), sizeof(imported.id));
  strlcpy(imported.label, label.c_str(), sizeof(imported.label));
  imported.frequencyMHz = frequency;
  imported.count = (uint16_t)durations.size();
  for (uint16_t index = 0; index < imported.count; index++) {
    const uint32_t duration = durations[index] | 0;
    const int level = levels[index] | -1;
    if (duration < 1 || duration > 65535 || (level != LOW && level != HIGH)) {
      webServer.send(400, "text/plain", "Invalid pulse duration or logic level");
      return;
    }
    imported.durations[index] = (uint16_t)duration;
    imported.levels[index] = (uint8_t)level;
  }

  size_t signalIndex = signals.size();
  for (size_t index = 0; index < signals.size(); index++) {
    if (zone == signals[index].zone && id == signals[index].id) signalIndex = index;
    else if (sameSignal(signals[index], imported)) {
      webServer.send(409, "text/plain", "This pulse sequence is already saved under another ID");
      return;
    }
  }
  if (signalIndex == signals.size() && signals.size() >= MAX_SIGNALS) {
    webServer.send(409, "text/plain", "Maximum number of saved signals reached");
    return;
  }
  if (!signalKeyAvailable(zone, id, signalIndex)) {
    webServer.send(409, "text/plain", "Zone/name storage key collision; choose a different name.");
    return;
  }

  const bool replacing = signalIndex < signals.size();
  const StoredSignal previous = replacing ? signals[signalIndex] : StoredSignal{};
  const String key = signalKey(zone, id);
  if (preferences.putBytes(key.c_str(), &imported, sizeof(imported)) != sizeof(imported)) {
    webServer.send(500, "text/plain", "Could not save signal to NVS");
    return;
  }
  if (replacing) signals[signalIndex] = imported;
  else signals.push_back(imported);
  if (!saveSignalIndex()) {
    if (replacing) {
      signals[signalIndex] = previous;
      preferences.putBytes(key.c_str(), &previous, sizeof(previous));
    } else {
      signals.pop_back();
      preferences.remove(key.c_str());
    }
    webServer.send(500, "text/plain", "Could not save signal index");
    return;
  }
  publishDiscovery(imported);
  lastAction = String(replacing ? "Imported/updated: " : "Imported: ") + label;
  logEvent(lastAction);
  webServer.send(200, "text/plain", "OK");
}

void handleRoot() {
  if (WiFi.status() != WL_CONNECTED) {
    const String body = settingsForm(true);
    webServer.send(200, "text/html; charset=utf-8", pageStart("Wi-Fi setup") + body + pageEnd());
    return;
  }

  const String mqttStatus = mqttHost.isEmpty() ? "Not configured" :
    (mqttClient.connected() ? "Connected" : "Disconnected");
  const String body = "<h1>Welcome to the CC1101 Bridge</h1>"
    "<p class='muted'>Record, organize and replay your saved radio signals.</p>"
    "<section class='panel'><div class='row' style='align-items:center;justify-content:space-between'>"
    "<h2>Bridge status</h2><button type='button' onclick='location.reload()' title='Refresh bridge status'>Refresh</button></div><p>Wi-Fi: Connected · " +
    htmlEscape(WiFi.localIP().toString()) + "</p><p>MQTT: " + mqttStatus +
    "</p><p>Saved signals: " + String(signals.size()) + " / " + String(MAX_SIGNALS) +
    "</p><p>CC1101: " + String(cc1101Detected ? "Detected" : "Not detected") +
    "</p></section><footer class='page-footer muted'>Author: Evohl · "
    "<a href='https://github.com/Evohl/esp32-cc1101-signal-bridge' target='_blank' rel='noopener noreferrer'>GitHub</a> · "
    "<a href='https://github.com/Evohl/esp32-cc1101-signal-bridge/blob/main/LICENSE' target='_blank' rel='noopener noreferrer'>MIT License</a></footer>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Home") + body + pageEnd());
}

void handleSignalsPage() {
  if (WiFi.status() != WL_CONNECTED) {
    const String body = settingsForm(true);
    webServer.send(200, "text/html; charset=utf-8", pageStart("Wi-Fi setup") + body + pageEnd());
    return;
  }

  const String requestedZone = webServer.arg("zone");
  const String selectedZone = zoneExists(requestedZone) ? requestedZone : "office";
  String beforeSignals = "<h1>Signal manager</h1><section class='panel recorder'><h2>Signal recorder</h2>"
    "<div class='recorder-step'><h3>1. Capture a signal</h3>"
    "<form method='post' action='/capture/start' class='form-grid' id='capture-form'>"
    "<label>Frequency (MHz)<input type='number' name='frequency' min='300' max='928' step='0.01' value='433.92' required></label>"
    "<label>Duration (seconds)<input type='number' name='seconds' min='1' max='10' value='3' required></label>"
    "<label>Noise reduction (0-7)<input type='range' name='strength' min='0' max='7' step='1' value='" + String(captureGainReductionStep) + "'><output id='strength-value'></output></label>"
    "<div class='form-actions'><button id='capture-button'>Start capture</button></div></form>"
    "<div id='capture-state' class='capture-state' aria-live='polite'>No capture yet</div>"
    "<div class='pulse-preview'><canvas id='pulse-waveform' class='pulse-waveform' aria-label='Captured pulse waveform'></canvas>"
    "<p id='pulse-summary' class='muted'>The waveform will appear here after capture.</p>"
    "<details><summary>First 120 pulse durations</summary><p id='pulse-values' class='pulse-values'></p></details></div></div>"
    "<div class='recorder-step'><h3>2. Name and save the signal</h3>"
    "<form method='post' action='/capture/save' class='row' id='capture-save-form'>"
    "<label>Zone <select id='recorder-zone' name='zone' required>" + renderZoneOptions(selectedZone) + "</select></label>"
    "<label>Signal name <input name='name' maxlength='15' pattern='[a-z0-9]+(_+[a-z0-9]+)*'"
    " autocapitalize='none' spellcheck='false' title='1-15 lowercase letters or digits, with underscores between characters' required></label>"
    "<input type='hidden' id='recorder-replace' name='replace' value=''><button id='save-button' disabled>Save signal</button></form></div></section>"
    "<h2>Recorded signals</h2>";
  const String afterSignals = "<p id='device-stats' class='muted'></p>"
    "<script>let shownCaptureId=-1;async function loadPulsePreview(){try{const response=await fetch('/api/pulses',{cache:'no-store'});"
    "if(!response.ok)return;const data=await response.json();const durations=data.durations;const levels=data.levels;"
    "if(!durations.length||durations.length!==levels.length)return;const canvas=document.getElementById('pulse-waveform');"
    "const width=canvas.clientWidth;const height=canvas.clientHeight;const ratio=window.devicePixelRatio||1;"
    "canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);const context=canvas.getContext('2d');"
    "context.scale(ratio,ratio);context.clearRect(0,0,width,height);const gapThreshold=5000;"
    "const shownDurations=durations.map((value,index)=>!levels[index]&&value>=gapThreshold?700:value);"
    "const total=shownDurations.reduce((sum,value)=>sum+value,0);const highY=12;const lowY=height-12;let x=0;"
    "context.beginPath();context.moveTo(0,levels[0]?highY:lowY);for(let i=0;i<durations.length;i++){"
    "const isGap=!levels[i]&&durations[i]>=gapThreshold;x+=shownDurations[i]*width/total;"
    "context.lineTo(x,levels[i]?highY:lowY);if(isGap){context.moveTo(x-4,highY+8);context.lineTo(x,highY);"
    "context.lineTo(x+4,highY+8);}if(i+1<levels.length)context.lineTo(x,levels[i+1]?highY:lowY);}"
    "context.strokeStyle='#71d6c5';context.lineWidth=1.5;context.stroke();"
    "const min=Math.min(...durations);const max=Math.max(...durations);document.getElementById('pulse-summary').textContent="
    "+durations.length+' pulses · '+data.frequency.toFixed(2)+' MHz · '+(total/1000).toFixed(1)+' ms displayed · range '+min+'-'+max+' us · LOW gaps >5 ms compressed';"
    "document.getElementById('pulse-values').textContent=durations.slice(0,120).map((value,index)=>"
    "+(levels[index]?'HIGH ':'LOW ')+value+' µs').join(' · ');}catch(error){}}"
    "async function refreshStatus(){try{const response=await fetch('/api/status',{cache:'no-store'});"
    "const data=await response.json();const state=document.getElementById('capture-state');"
    "const captureLabel=data.capture?'Capture in progress':(data.pulses?'Capture ready':'No pulses received');"
    "state.textContent=captureLabel+' · '+data.pulses+' pulses · GPIO4/GDO0 '+(data.gdo0?'HIGH':'LOW')+' · GPIO27/CS '+(data.carrier?'HIGH':'LOW');"
    "state.classList.toggle('warn',!data.pulses&&!data.capture);const captureButton=document.getElementById('capture-button');"
    "captureButton.disabled=data.capture;captureButton.textContent=data.capture?'Capturing...':'Start capture';"
    "document.getElementById('save-button').disabled=data.capture||data.pulses<4;"
    "document.getElementById('device-stats').textContent='RSSI '+(data.rssi_dbm===undefined?'--':data.rssi_dbm.toFixed(1)+' dBm')+' · Signals '+data.signals;"
    "if(!data.capture&&data.pulses>0&&data.capture_id!==shownCaptureId){shownCaptureId=data.capture_id;loadPulsePreview();}"
    "}catch(error){}}"
    "document.getElementById('capture-form').addEventListener('submit',async(event)=>{event.preventDefault();"
    "const form=event.currentTarget;const button=document.getElementById('capture-button');const state=document.getElementById('capture-state');"
    "button.disabled=true;button.textContent='Starting capture...';state.textContent='Starting capture';"
    "try{const response=await fetch(form.action,{method:'POST',headers:{'X-Requested-With':'fetch'},"
    "body:new URLSearchParams(new FormData(form))});"
    "if(!response.ok)throw new Error(await response.text());await refreshStatus();}catch(error){button.disabled=false;"
    "button.textContent='Start capture';state.textContent=error.message;state.classList.add('warn');}});"
    "const strengthInput=document.querySelector('#capture-form input[name=strength]');"
    "const recorderZone=document.getElementById('recorder-zone');"
    "const savedZone=localStorage.getItem('cc1101-recorder-zone');"
    "if([...recorderZone.options].some(option=>option.value===savedZone))recorderZone.value=savedZone;"
    "const replaceInput=document.getElementById('recorder-replace');const saveButton=document.getElementById('save-button');"
    "const clearReplacement=()=>{replaceInput.value='';saveButton.textContent='Save signal';};"
    "const rememberZone=()=>{localStorage.setItem('cc1101-recorder-zone',recorderZone.value);clearReplacement();};"
    "recorderZone.addEventListener('change',rememberZone);rememberZone();"
    "const signalNameInput=document.querySelector('#capture-save-form input[name=name]');"
    "signalNameInput.addEventListener('input',clearReplacement);"
    "document.querySelectorAll('.record-signal').forEach(button=>button.addEventListener('click',()=>{"
    "const zone=button.dataset.zone;const id=button.dataset.id;"
    "recorderZone.value=zone;recorderZone.dispatchEvent(new Event('change'));signalNameInput.value=id;"
    "replaceInput.value=zone+'/'+id;saveButton.textContent='Replace signal';"
    "document.getElementById('capture-form').scrollIntoView({behavior:'smooth',block:'center'});"
    "}));"
    "const gainStepsDb=[0,2.6,6.1,7.4,9.2,11.5,14.6,17.1];"
    "const updateStrengthLabel=()=>{const value=Number(strengthInput.value);document.getElementById('strength-value').value='Level '+value+' (~'+gainStepsDb[value]+' dB)';};"
    "strengthInput.addEventListener('input',updateStrengthLabel);updateStrengthLabel();"
    "document.querySelectorAll('.send-form').forEach(form=>form.addEventListener('submit',async(event)=>{event.preventDefault();"
    "const button=form.querySelector('button');const originalText=button.textContent;button.disabled=true;button.textContent='Sending...';"
    "try{const response=await fetch(form.action,{method:'POST',headers:{'X-Requested-With':'fetch'},"
    "body:new URLSearchParams(new FormData(form))});if(!response.ok)throw new Error(await response.text());await refreshStatus();}"
    "catch(error){button.title=error.message;}finally{button.disabled=false;button.textContent=originalText;}}));"
    "refreshStatus();setInterval(refreshStatus,1000);</script>";
  const String start = pageStart("Signals");
  const String end = pageEnd();
  if (!beforeSignals.startsWith("<h1>Signal manager</h1>") ||
      !beforeSignals.endsWith("<h2>Recorded signals</h2>") || !afterSignals.startsWith("<p id='device-stats'") ||
      !afterSignals.endsWith("</script>")) {
    Serial.printf("[web] /signals static section incomplete: before=%u after=%u heap=%u\n",
                  (unsigned)beforeSignals.length(), (unsigned)afterSignals.length(),
                  (unsigned)ESP.getFreeHeap());
    webServer.send(500, "text/plain; charset=utf-8", "The ESP could not build the complete signals page.");
    return;
  }
  Serial.printf("[web] /signals stream start: signals=%u static=%u heap=%u\n",
                (unsigned)signals.size(), (unsigned)(start.length() + beforeSignals.length() +
                afterSignals.length() + end.length()), (unsigned)ESP.getFreeHeap());
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html; charset=utf-8", "");
  webServer.sendContent(start);
  webServer.sendContent(beforeSignals);
  sendSignalList();
  webServer.sendContent(afterSignals);
  webServer.sendContent(end);
  webServer.sendContent("");
  Serial.printf("[web] /signals stream complete: signals=%u heap=%u\n",
                (unsigned)signals.size(), (unsigned)ESP.getFreeHeap());
}

void handleLogPage() {
  const String body = "<h1>Event log</h1><p class='muted'>Recent bridge activity</p>"
    "<button id='refresh-log' type='button'>Refresh</button>"
    "<div id='event-log' class='log-list' aria-live='polite'>Loading...</div>"
    "<script>async function refreshLog(){try{const response=await fetch('/api/log',{cache:'no-store'});"
    "if(!response.ok)throw new Error('Could not load event log');const data=await response.json();"
    "const log=document.getElementById('event-log');log.replaceChildren();if(!data.log.length){log.textContent='No events yet';return;}"
    "for(let i=data.log.length-1;i>=0;i--){const item=document.createElement('div');item.className='log-entry';"
    "item.textContent=data.log[i].seconds.toFixed(1)+' s · '+data.log[i].message;log.append(item);}}catch(error){"
    "document.getElementById('event-log').textContent=error.message;}}"
    "document.getElementById('refresh-log').addEventListener('click',refreshLog);refreshLog();setInterval(refreshLog,5000);</script>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Event log") + body + pageEnd());
}

void handleCaptureStart() {
  const bool isAjax = webServer.hasHeader("X-Requested-With");
  const float frequency = webServer.arg("frequency").toFloat();
  const long secondsValue = webServer.arg("seconds").toInt();
  const long strengthValue = webServer.hasArg("strength") ? webServer.arg("strength").toInt() : DEFAULT_LNA_GAIN_REDUCTION_STEP;
  if (captureActive) {
    logEvent("Capture start rejected: capture already in progress");
    if (isAjax) { webServer.send(204); return; }
    webServer.sendHeader("Location", "/signals");
    webServer.send(303);
    return;
  }
  if (!validFrequency(frequency)) {
    webServer.send(400, "text/plain", "Invalid frequency. Supported CC1101 bands: 300-348, 387-464, or 779-928 MHz.");
    return;
  }
  if (secondsValue < 1 || secondsValue > 10) {
    webServer.send(400, "text/plain", "Capture duration must be between 1 and 10 seconds.");
    return;
  }
  if (strengthValue < 0 || strengthValue > 7) {
    webServer.send(400, "text/plain", "Noise reduction must be between 0 and 7.");
    return;
  }
  const uint8_t seconds = (uint8_t)secondsValue;
  captureGainReductionStep = (uint8_t)strengthValue;
  noInterrupts();
  captureCount = 0;
  captureFirstEdge = true;
  captureLastCarrierUs = micros() - CAPTURE_CARRIER_HOLD_US;
  interrupts();
  captureSequence++;
  captureFrequencyMHz = frequency;
  activeFrequencyMHz = frequency;
  captureDeadline = millis() + (uint32_t)seconds * 1000;
  captureActive = true;
  ccConfigure(frequency, true);
  attachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0), captureEdge, CHANGE);
  lastAction = "Capture in progress";
  logEvent("Capture started: " + String(frequency, 2) + " MHz, " + String(seconds) + " s, noise reduction level " + String(captureGainReductionStep));
  if (isAjax) { webServer.send(204); return; }
  webServer.sendHeader("Location", "/signals");
  webServer.send(303);
}

void handleZoneCreate() {
  const String zone = webServer.arg("zone");
  if (!validSignalId(zone)) {
    webServer.send(400, "text/plain", "Invalid zone name.");
    return;
  }
  if (zoneExists(zone)) {
    webServer.send(409, "text/plain", "That zone already exists.");
    return;
  }
  if (!addZone(zone)) {
    webServer.send(409, "text/plain", "Maximum number of zones reached.");
    return;
  }
  if (!saveZones()) {
    zones.pop_back();
    webServer.send(500, "text/plain", "Could not save zone list.");
    return;
  }
  logEvent("Zone created: " + zone);
  webServer.sendHeader("Location", "/zones");
  webServer.send(303);
}

void handleZoneDelete() {
  const String zone = webServer.arg("zone");
  size_t zoneIndex = zones.size();
  for (size_t index = 0; index < zones.size(); index++) {
    if (zones[index] == zone) {
      zoneIndex = index;
      break;
    }
  }
  if (zoneIndex == zones.size()) {
    webServer.send(404, "text/plain", "Zone not found.");
    return;
  }
  for (const StoredSignal& signal : signals) {
    if (zone == signal.zone) {
      webServer.send(409, "text/plain", "Delete the signals in this zone before deleting the zone.");
      return;
    }
  }
  zones.erase(zones.begin() + zoneIndex);
  if (!saveZones()) {
    zones.insert(zones.begin() + zoneIndex, zone);
    webServer.send(500, "text/plain", "Could not save zone list.");
    return;
  }
  logEvent("Zone deleted: " + zone);
  webServer.sendHeader("Location", "/zones");
  webServer.send(303);
}

void finishCapture() {
  if (!captureActive) return;
  detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));
  captureActive = false;
  ccStrobe(CC_SIDLE);
  ccStrobe(CC_SRX);
  lastAction = "Capture finished";
  logEvent("Capture finished: " + String(captureCount) + " pulses, GPIO4/GDO0 " +
           String(digitalRead(PIN_CC_GDO0) ? "HIGH" : "LOW"));
}

void handleCaptureSave() {
  if (captureActive) finishCapture();
  const String id = webServer.arg("name");
  const String zone = webServer.arg("zone").isEmpty() ? "unassigned" : webServer.arg("zone");
  const String label = id;
  const String replaceIdentity = webServer.arg("replace");
  size_t replaceIndex = signals.size();
  if (label.isEmpty() || captureCount < 4) {
    logEvent("Not saved: fewer than 4 pulses or missing signal ID");
    webServer.send(400, "text/plain", "A signal ID is required and the capture must contain at least four pulses.");
    return;
  }
  if (!validSignalId(zone) || !validSignalId(id)) {
    webServer.send(400, "text/plain", "Invalid zone or signal name: use 1-15 lowercase letters or digits, with underscores only between characters.");
    return;
  }
  if (!zoneExists(zone)) {
    webServer.send(400, "text/plain", "Create the zone before saving a signal.");
    return;
  }
  if (!replaceIdentity.isEmpty()) {
    String replaceZone;
    String replaceId;
    if (!parseSignalIdentity(replaceIdentity, replaceZone, replaceId) || replaceZone != zone || replaceId != id) {
      webServer.send(400, "text/plain", "Replacement target must match the selected zone and signal name.");
      return;
    }
    for (size_t index = 0; index < signals.size(); index++) {
      if (zone == signals[index].zone && id == signals[index].id) {
        replaceIndex = index;
        break;
      }
    }
    if (replaceIndex == signals.size()) {
      webServer.send(404, "text/plain", "Signal selected for replacement no longer exists.");
      return;
    }
  }
  const bool replacing = replaceIndex < signals.size();
  if (!replacing && signals.size() >= MAX_SIGNALS) {
    webServer.send(400, "text/plain", "Maximum number of saved signals reached.");
    return;
  }
  for (size_t index = 0; index < signals.size(); index++) {
    const StoredSignal& existing = signals[index];
    if (index != replaceIndex && zone == existing.zone && id == existing.id) {
      webServer.send(409, "text/plain", "This signal ID is already in use.");
      return;
    }
  }
  if (!signalKeyAvailable(zone, id, replaceIndex)) {
    webServer.send(409, "text/plain", "Zone/name storage key collision; choose a different name.");
    return;
  }
  StoredSignal signal{};
  strlcpy(signal.zone, zone.c_str(), sizeof(signal.zone));
  strlcpy(signal.id, id.c_str(), sizeof(signal.id));
  strlcpy(signal.label, label.c_str(), sizeof(signal.label));
  signal.frequencyMHz = captureFrequencyMHz;
  noInterrupts();
  signal.count = captureCount;
  for (uint16_t index = 0; index < signal.count; index++) {
    signal.durations[index] = captureDurations[index];
    signal.levels[index] = captureLevels[index];
  }
  interrupts();

  for (size_t index = 0; index < signals.size(); index++) {
    if (index == replaceIndex) continue;
    const StoredSignal& existing = signals[index];
    if (!sameSignal(existing, signal)) continue;
    lastAction = "Duplicate not saved: " + label;
    logEvent(lastAction);
    webServer.sendHeader("Location", "/signals?zone=" + zone);
    webServer.send(303);
    return;
  }
  const String key = signalKey(zone, id);
  const StoredSignal previous = replacing ? signals[replaceIndex] : StoredSignal{};
  if (preferences.putBytes(key.c_str(), &signal, sizeof(signal)) != sizeof(signal)) {
    lastAction = "Save failed: NVS full or write error";
    logEvent(lastAction);
    webServer.send(500, "text/plain", lastAction);
    return;
  }
  if (replacing) signals[replaceIndex] = signal;
  else signals.push_back(signal);
  if (!saveSignalIndex()) {
    if (replacing) {
      signals[replaceIndex] = previous;
      preferences.putBytes(key.c_str(), &previous, sizeof(previous));
    } else {
      signals.pop_back();
      preferences.remove(key.c_str());
    }
    lastAction = "Save failed: could not write signal index";
    logEvent(lastAction);
    webServer.send(500, "text/plain", lastAction);
    return;
  }
  publishDiscovery(signal);
  lastAction = String(replacing ? "Re-recorded: " : "Saved: ") + zone + "/" + label;
  logEvent(lastAction + " (" + String(signal.count) + " pulses)");
  webServer.sendHeader("Location", "/signals?zone=" + zone);
  webServer.send(303);
}

void handleSend() {
  const bool isAjax = webServer.hasHeader("X-Requested-With");
  String zone;
  String id;
  if (!parseSignalIdentity(webServer.arg("id"), zone, id)) {
    if (isAjax) {
      webServer.send(400, "text/plain", "Invalid signal identity.");
      return;
    }
    webServer.sendHeader("Location", "/signals");
    webServer.send(303);
    return;
  }
  sendSignal(zone, id);
  if (isAjax) {
    webServer.send(204);
    return;
  }
  webServer.sendHeader("Location", "/signals");
  webServer.send(303);
}

void handleSignalEditor() {
  webServer.send(200, "text/html; charset=utf-8", renderSignalEditor(webServer.arg("id")));
}

void handleSignalSave() {
  String oldZone;
  String oldId;
  if (!parseSignalIdentity(webServer.arg("id"), oldZone, oldId)) {
    webServer.send(400, "text/plain", "Invalid signal identity.");
    return;
  }
  size_t signalIndex = signals.size();
  for (size_t index = 0; index < signals.size(); index++) {
    if (oldZone == signals[index].zone && oldId == signals[index].id) {
      signalIndex = index;
      break;
    }
  }
  if (signalIndex == signals.size()) {
    webServer.send(404, "text/plain", "Signal not found.");
    return;
  }

  const float frequency = webServer.arg("frequency").toFloat();
  const String newZone = webServer.arg("zone");
  const String newId = webServer.arg("name");
  if (!validSignalId(newZone) || !validSignalId(newId) || !validFrequency(frequency)) {
    webServer.send(400, "text/plain", "Invalid signal name or frequency.");
    return;
  }
  if (!zoneExists(newZone)) {
    webServer.send(400, "text/plain", "Create the zone before assigning a signal to it.");
    return;
  }

  StoredSignal updated = signals[signalIndex];
  strlcpy(updated.zone, newZone.c_str(), sizeof(updated.zone));
  strlcpy(updated.id, newId.c_str(), sizeof(updated.id));
  strlcpy(updated.label, newId.c_str(), sizeof(updated.label));
  updated.frequencyMHz = frequency;
  updated.count = 0;
  const String pulseData = webServer.arg("pulse_data");
  size_t lineStart = 0;
  while (lineStart < pulseData.length()) {
    const int lineEnd = pulseData.indexOf('\n', lineStart);
    const size_t end = lineEnd < 0 ? pulseData.length() : (size_t)lineEnd;
    String line = pulseData.substring(lineStart, end);
    line.trim();
    line.toUpperCase();
    if (!line.isEmpty()) {
      if (line.length() < 3 || (line[0] != 'H' && line[0] != 'L') || line[1] != ':' ||
          updated.count >= MAX_PULSES) {
        webServer.send(400, "text/plain", "Invalid or too many pulse entries.");
        return;
      }
      const String durationText = line.substring(2);
      for (size_t digit = 0; digit < durationText.length(); digit++) {
        if (!isDigit(durationText[digit])) {
          webServer.send(400, "text/plain", "Pulse duration must be a number.");
          return;
        }
      }
      const long duration = durationText.toInt();
      if (duration < 1 || duration > 65535) {
        webServer.send(400, "text/plain", "Pulse duration must be between 1 and 65535 us.");
        return;
      }
      updated.levels[updated.count] = line[0] == 'H' ? HIGH : LOW;
      updated.durations[updated.count] = (uint16_t)duration;
      updated.count++;
    }
    if (lineEnd < 0) break;
    lineStart = (size_t)lineEnd + 1;
  }
  if (updated.count < 4) {
    webServer.send(400, "text/plain", "At least four pulses are required.");
    return;
  }

  for (size_t index = 0; index < signals.size(); index++) {
    if (index != signalIndex && newZone == signals[index].zone && newId == signals[index].id) {
      webServer.send(409, "text/plain", "This name is already used in that zone.");
      return;
    }
    if (index != signalIndex && sameSignal(signals[index], updated)) {
      webServer.send(409, "text/plain", "This pulse sequence is already saved under another name.");
      return;
    }
  }
  if (!signalKeyAvailable(newZone, newId, signalIndex)) {
    webServer.send(409, "text/plain", "Zone/name storage key collision; choose a different name.");
    return;
  }

  const StoredSignal previous = signals[signalIndex];
  const String previousKey = signalKey(oldZone, oldId);
  const String key = signalKey(newZone, newId);
  if (preferences.putBytes(key.c_str(), &updated, sizeof(updated)) != sizeof(updated)) {
    webServer.send(500, "text/plain", "Could not save changes to NVS.");
    return;
  }
  signals[signalIndex] = updated;
  if (!saveSignalIndex()) {
    signals[signalIndex] = previous;
    if (key != previousKey) preferences.remove(key.c_str());
    preferences.putBytes(previousKey.c_str(), &previous, sizeof(previous));
    webServer.send(500, "text/plain", "Could not save signal index.");
    return;
  }
  if (key != previousKey) preferences.remove(previousKey.c_str());
  if (mqttClient.connected() && (oldZone != newZone || oldId != newId)) {
    const String previousTopic = discoveryConfigTopic(discoveryObjectId(oldZone, oldId));
    const String previousLegacyTopic = discoveryConfigTopic(String("cc1101_") + bridgeId + "_" + oldId);
    mqttClient.publish(previousTopic.c_str(), "", true);
    if (previousLegacyTopic != previousTopic) mqttClient.publish(previousLegacyTopic.c_str(), "", true);
  }
  publishDiscovery(updated);
  lastAction = "Signal updated: " + newId;
  logEvent(lastAction);
  webServer.sendHeader("Location", "/signals");
  webServer.send(303);
}

void handleDelete() {
  String zone;
  String id;
  if (!parseSignalIdentity(webServer.arg("id"), zone, id)) {
    webServer.sendHeader("Location", "/signals");
    webServer.send(303);
    return;
  }
  for (size_t index = 0; index < signals.size(); index++) {
    if (zone != signals[index].zone || id != signals[index].id) continue;
    if (mqttClient.connected()) {
      const String discoveryTopic = discoveryConfigTopic(discoveryObjectId(zone, id));
      const String legacyTopic = legacyDiscoveryConfigTopic(id);
      const String oldDiscoveryTopic = discoveryConfigTopic(String("cc1101_") + bridgeId + "_" + id);
      const String previousId = oldActionNameForId(id);
      const String previousLegacyTopic = legacyDiscoveryConfigTopic(previousId);
      const String previousDiscoveryTopic = discoveryConfigTopic(String("cc1101_") + bridgeId + "_" + previousId);
      mqttClient.publish(discoveryTopic.c_str(), "", true);
      mqttClient.publish(legacyTopic.c_str(), "", true);
      if (previousLegacyTopic != legacyTopic) mqttClient.publish(previousLegacyTopic.c_str(), "", true);
      if (oldDiscoveryTopic != discoveryTopic) mqttClient.publish(oldDiscoveryTopic.c_str(), "", true);
      if (previousDiscoveryTopic != discoveryTopic && previousDiscoveryTopic != oldDiscoveryTopic) {
        mqttClient.publish(previousDiscoveryTopic.c_str(), "", true);
      }
    }
    preferences.remove(signalKey(zone, id).c_str());
    signals.erase(signals.begin() + index);
    saveSignalIndex();
    break;
  }
  webServer.sendHeader("Location", "/signals");
  webServer.send(303);
}

void handleApiStatus() {
  JsonDocument document;
  document["capture"] = captureActive;
  document["pulses"] = captureCount;
  document["capture_id"] = captureSequence;
  document["signals"] = signals.size();
  document["mqtt"] = mqttClient.connected();
  document["gdo0"] = digitalRead(PIN_CC_GDO0) == HIGH;
  document["carrier"] = digitalRead(PIN_CC_GDO2) == HIGH;
  const uint8_t rawRssi = ccReadStatusRegister(CC_RSSI);
  if (rawRssi != 0xFF) {
    const int16_t signedRssi = rawRssi >= 0x80 ? (int16_t)rawRssi - 256 : rawRssi;
    document["rssi_dbm"] = signedRssi / 2.0f - 74.0f;
  }
  document["cc1101"] = cc1101Detected;
  document["partnum"] = cc1101PartNumber;
  document["version"] = cc1101Version;
  String json;
  serializeJson(document, json);
  webServer.send(200, "application/json", json);
}

void handleApiLog() {
  JsonDocument document;
  JsonArray entries = document["log"].to<JsonArray>();
  const uint8_t first = (eventLogNext + MAX_LOG_ENTRIES - eventLogCount) % MAX_LOG_ENTRIES;
  for (uint8_t index = 0; index < eventLogCount; index++) {
    const LogEntry& entry = eventLog[(first + index) % MAX_LOG_ENTRIES];
    JsonObject item = entries.add<JsonObject>();
    item["seconds"] = entry.atMs / 1000.0f;
    item["message"] = entry.message;
  }
  String json;
  serializeJson(document, json);
  webServer.send(200, "application/json", json);
}

void handleApiPulses() {
  if (captureActive) {
    webServer.send(409, "text/plain", "Capture is still in progress.");
    return;
  }
  JsonDocument document;
  document["frequency"] = captureFrequencyMHz;
  JsonArray durations = document["durations"].to<JsonArray>();
  JsonArray levels = document["levels"].to<JsonArray>();
  for (uint16_t index = 0; index < captureCount; index++) {
    durations.add(captureDurations[index]);
    levels.add(captureLevels[index]);
  }
  String json;
  serializeJson(document, json);
  webServer.send(200, "application/json", json);
}

void handleSettings() {
  if (webServer.hasArg("ssid")) {
    const String newSsid = webServer.arg("ssid");
    const String newWifiPassword = webServer.arg("wifi_password");
    preferences.putString("ssid", newSsid);
    if (!newWifiPassword.isEmpty() || newSsid != wifiSsid) {
      preferences.putString("wpass", newWifiPassword);
    }
  }
  if (webServer.hasArg("mqtt_host")) {
    preferences.putString("mhost", webServer.arg("mqtt_host"));
    preferences.putUShort("mport", (uint16_t)webServer.arg("mqtt_port").toInt());
    preferences.putString("muser", webServer.arg("mqtt_user"));
    if (!webServer.arg("mqtt_password").isEmpty()) {
      preferences.putString("mpass", webServer.arg("mqtt_password"));
    }
  }
  webServer.send(200, "text/html; charset=utf-8", pageStart("Settings saved") +
    "<h1>Settings saved</h1><p>Restarting the bridge.</p>" + pageEnd());
  delay(800);
  ESP.restart();
}

void handleSettingsPage() {
  const String body = settingsForm(false);
  webServer.send(200, "text/html; charset=utf-8", pageStart("Network") + body + pageEnd());
}

void handleFirmwarePage() {
  const String body = String("<h1>Firmware update</h1><form id='firmware-form' method='post' action='/firmware' enctype='multipart/form-data'>") +
    "<div class='file-picker'><button id='firmware-file-select' type='button' aria-controls='firmware-file'>Choose firmware file</button>"
    "<span id='firmware-file-name' aria-live='polite'>No file selected</span>"
    "<input id='firmware-file' type='file' name='firmware' accept='.bin' hidden></div>"
    "<p id='firmware-state' class='muted' aria-live='polite'></p><button id='firmware-install'>Install firmware</button></form>"
    "<script>const firmwareFile=document.getElementById('firmware-file');"
    "document.getElementById('firmware-file-select').addEventListener('click',()=>firmwareFile.click());"
    "firmwareFile.addEventListener('change',()=>{document.getElementById('firmware-file-name').textContent="
    "firmwareFile.files.length?firmwareFile.files[0].name:'No file selected';});"
    "document.getElementById('firmware-form').addEventListener('submit',event=>{"
    "if(!firmwareFile.files.length){event.preventDefault();document.getElementById('firmware-state').textContent='Choose a firmware file first.';return;}"
    "document.getElementById('firmware-state').textContent='Uploading firmware...';"
    "document.getElementById('firmware-install').disabled=true;});</script>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Firmware") + body + pageEnd());
}

void handleSystemPage() {
  const String ccStatus = cc1101Detected ? "Detected" : "Not detected";
  const String ccPart = cc1101PartNumber == 0xFF ? "Unavailable" : "0x" + String(cc1101PartNumber, HEX);
  const String ccVersion = cc1101Version == 0xFF ? "Unavailable" : "0x" + String(cc1101Version, HEX);
  const String body = "<h1>System</h1><section class='panel'><h2>ESP32</h2>"
    "<p>Host: " + htmlEscape(String(hostname)) + ".local</p><p>DHCP hostname: " +
    String(wifiHostnameSet ? "Applied" : "Not applied") + "</p><p>mDNS: " +
    String(mdnsStarted ? "Started" : "Failed to start") + "</p><p>Chip: " + String(ESP.getChipModel()) +
    " revision " + String(ESP.getChipRevision()) + " · CPU " + String(ESP.getCpuFreqMHz()) + " MHz</p>"
    "<p>ESP-IDF: " + String(ESP.getSdkVersion()) + "</p><p>Free heap: " +
    String(ESP.getFreeHeap() / 1024) + " / " + String(ESP.getHeapSize() / 1024) + " KiB</p>"
    "<p>Flash: " + String(ESP.getFlashChipSize() / 1024) + " KiB · Free app space: " +
    String(ESP.getFreeSketchSpace() / 1024) + " KiB</p></section>"
    "<section class='panel'><h2>CC1101</h2><p>Status: " + ccStatus + "</p><p>Part number: " +
    ccPart + " · Version: " + ccVersion + "</p></section>"
    "<section class='panel'><h2>Restart</h2><form method='post' action='/restart'>"
    "<button>Restart bridge</button></form></section>"
    "<section class='panel'><h2>Factory reset</h2><p>This erases Wi-Fi and MQTT settings, zones, and every saved signal.</p>"
    "<form method='post' action='/factory-reset' onsubmit=\"return confirm('Erase all bridge configuration and saved signals?')\">"
    "<label>Type RESET to confirm <input name='confirm' pattern='RESET' required autocomplete='off'></label> "
    "<button class='danger-button'>Erase all configuration</button></form></section>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("System") + body + pageEnd());
}

void handleRestart() {
  webServer.send(200, "text/html; charset=utf-8", pageStart("Restarting") +
    "<h1>Restarting bridge</h1><p>The device will be back shortly.</p>" + pageEnd());
  delay(300);
  ESP.restart();
}

void handleFactoryReset() {
  if (webServer.arg("confirm") != "RESET") {
    webServer.send(400, "text/plain; charset=utf-8", "Type RESET to confirm factory reset.");
    return;
  }
  if (!preferences.clear()) {
    webServer.send(500, "text/plain; charset=utf-8", "Could not erase bridge configuration.");
    return;
  }
  webServer.send(200, "text/html; charset=utf-8", pageStart("Factory reset") +
    "<h1>Factory reset complete</h1><p>Restarting into setup mode. Connect to the " +
    String(AP_NAME) + " Wi-Fi access point.</p>" + pageEnd());
  delay(500);
  ESP.restart();
}

void handleFirmwareUpload() {
  HTTPUpload& upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) Update.printError(Serial);
  }
}

void handleFirmwareDone() {
  const bool success = !Update.hasError();
  webServer.send(success ? 200 : 500, "text/plain", success ? "Update successful. Restarting." : "Firmware update failed.");
  if (success) {
    delay(500);
    ESP.restart();
  }
}

void startWebServer() {
  if (webStarted) return;
  const char* requestHeaders[] = {"X-Requested-With"};
  webServer.collectHeaders(requestHeaders, 1);
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/signals", HTTP_GET, handleSignalsPage);
  webServer.on("/log", HTTP_GET, handleLogPage);
  webServer.on("/zones", HTTP_GET, handleZonesPage);
  webServer.on("/backup", HTTP_GET, handleBackupPage);
  webServer.on("/signals/export", HTTP_GET, handleSignalsExport);
  webServer.on("/signal/import", HTTP_POST, handleSignalImport);
  webServer.on("/api/status", HTTP_GET, handleApiStatus);
  webServer.on("/api/log", HTTP_GET, handleApiLog);
  webServer.on("/api/pulses", HTTP_GET, handleApiPulses);
  webServer.on("/zone/create", HTTP_POST, handleZoneCreate);
  webServer.on("/zone/delete", HTTP_POST, handleZoneDelete);
  webServer.on("/capture/start", HTTP_POST, handleCaptureStart);
  webServer.on("/capture/save", HTTP_POST, handleCaptureSave);
  webServer.on("/send", HTTP_POST, handleSend);
  webServer.on("/signal", HTTP_GET, handleSignalEditor);
  webServer.on("/signal/save", HTTP_POST, handleSignalSave);
  webServer.on("/delete", HTTP_POST, handleDelete);
  webServer.on("/settings", HTTP_GET, handleSettingsPage);
  webServer.on("/settings", HTTP_POST, handleSettings);
  webServer.on("/firmware", HTTP_GET, handleFirmwarePage);
  webServer.on("/firmware", HTTP_POST, handleFirmwareDone, handleFirmwareUpload);
  webServer.on("/system", HTTP_GET, handleSystemPage);
  webServer.on("/restart", HTTP_GET, []() {
    webServer.sendHeader("Location", "/system");
    webServer.send(303);
  });
  webServer.on("/restart", HTTP_POST, handleRestart);
  webServer.on("/factory-reset", HTTP_POST, handleFactoryReset);
  webServer.begin();
  webStarted = true;
}

void startOta() {
  if (otaStarted || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();
  mdnsStarted = MDNS.begin(hostname);
  if (mdnsStarted) MDNS.addService("http", "tcp", 80);
  logEvent(String("mDNS ") + (mdnsStarted ? "started: " : "failed: ") + hostname + ".local");
  Serial.printf("[net] mDNS hostname=%s.local started=%s ip=%s\n", hostname,
                mdnsStarted ? "yes" : "no", WiFi.localIP().toString().c_str());
  otaStarted = true;
}

void connectWifi() {
  if (wifiSsid.isEmpty()) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME, "cc1101setup");
    logEvent("Setup access point started");
    startWebServer();
    return;
  }
  const bool hostnameConfigured = WiFi.setHostname(hostname);
  const bool stationModeStarted = WiFi.mode(WIFI_STA);
  wifiHostnameSet = hostnameConfigured && stationModeStarted;
  Serial.printf("[net] DHCP hostname=%s configured=%s sta-mode=%s\n", hostname,
                hostnameConfigured ? "yes" : "no", stationModeStarted ? "yes" : "no");
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < 15000) {
    delay(250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    logEvent("Wi-Fi connected: " + WiFi.localIP().toString());
    startWebServer();
    startOta();
  } else {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_NAME, "cc1101setup");
    logEvent("Wi-Fi unavailable; setup access point started");
    startWebServer();
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
#if DEVICE_HOSTNAME_SUFFIX > 0
  snprintf(bridgeId, sizeof(bridgeId), "%06d", DEVICE_HOSTNAME_SUFFIX);
#else
  uint8_t stationMac[6]{};
  if (esp_read_mac(stationMac, ESP_MAC_WIFI_STA) == ESP_OK) {
    snprintf(bridgeId, sizeof(bridgeId), "%02x%02x%02x", stationMac[3], stationMac[4], stationMac[5]);
  } else {
    const uint64_t efuseMac = ESP.getEfuseMac();
    const uint32_t macSuffix = (((efuseMac >> 24) & 0xFF) << 16) |
                               (((efuseMac >> 32) & 0xFF) << 8) |
                               ((efuseMac >> 40) & 0xFF);
    snprintf(bridgeId, sizeof(bridgeId), "%06x", macSuffix);
  }
#endif
  snprintf(hostname, sizeof(hostname), "esp32-cc1101-%s", bridgeId);
  logEvent("Bridge started: " + String(hostname));
  pinMode(PIN_CC_CS, OUTPUT);
  digitalWrite(PIN_CC_CS, HIGH);
  pinMode(PIN_CC_GDO0, INPUT);
  pinMode(PIN_CC_GDO2, INPUT);
  SPI.begin(PIN_CC_SCK, PIN_CC_MISO, PIN_CC_MOSI, PIN_CC_CS);

  preferences.begin("cc1101", false);
  wifiSsid = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("wpass", "");
  mqttHost = preferences.getString("mhost", "");
  mqttPort = preferences.getUShort("mport", 1883);
  mqttUser = preferences.getString("muser", "");
  mqttPassword = preferences.getString("mpass", "");
  mqttBase = preferences.getString("mbase", String("cc1101/") + bridgeId);
  loadSignals();
  loadZones();

  ccStrobe(CC_SRES);
  delay(5);
  cc1101PartNumber = ccReadStatusRegister(CC_PARTNUM);
  cc1101Version = ccReadStatusRegister(CC_VERSION);
  cc1101Detected = isKnownCc1101Version(cc1101Version);
  if (cc1101Detected) {
    logEvent("CC1101 detected: PARTNUM 0x" + String(cc1101PartNumber, HEX) +
             ", VERSION 0x" + String(cc1101Version, HEX));
  } else {
    logEvent("CC1101 not detected: PARTNUM 0x" + String(cc1101PartNumber, HEX) +
             ", VERSION 0x" + String(cc1101Version, HEX) + " - check SPI and power");
  }
  ccConfigure(activeFrequencyMHz, true);
  mqttClient.setServer(mqttHost.c_str(), mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  mqttClient.setKeepAlive(30);
  connectWifi();
  Serial.println("CC1101 Bridge gestartet");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    startWebServer();
    startOta();
    ArduinoOTA.handle();
    reconnectMqtt();
    mqttClient.loop();
  }
  if (webStarted) webServer.handleClient();
  if (captureActive && (int32_t)(millis() - captureDeadline) >= 0) finishCapture();
  delay(2);
}