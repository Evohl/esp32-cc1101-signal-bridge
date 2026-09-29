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
constexpr float TEST_SIGNAL_FREQUENCY_MHZ = 433.92f;
constexpr uint16_t MAX_PULSES = 600;
constexpr uint8_t MAX_SIGNALS = 20;
constexpr uint8_t MAX_LOG_ENTRIES = 30;
constexpr uint8_t MAX_CAPTURE_HISTORY = 2;
constexpr uint32_t MQTT_BUFFER_SIZE = 768;
constexpr uint32_t CAPTURE_CARRIER_HOLD_US = 20000;
constexpr uint8_t DEFAULT_LNA_GAIN_REDUCTION_STEP = 3;
char hostname[16];
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
String wifiSsid;
String wifiPassword;
String mqttHost;
String mqttUser;
String mqttPassword;
String mqttBase;
uint16_t mqttPort = 1883;
bool webStarted = false;
bool otaStarted = false;
uint32_t lastMqttAttempt = 0;
int lastMqttFailureState = -1;
float activeFrequencyMHz = 433.92f;
String lastAction = "Bereit";
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

String slugify(const String& value) {
  String result;
  result.reserve(11);
  for (size_t index = 0; index < value.length() && result.length() < 11; index++) {
    const char ch = value[index];
    if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')) {
      result += ch;
    } else if (ch >= 'A' && ch <= 'Z') {
      result += (char)(ch - 'A' + 'a');
    } else if (result.length() > 0 && result[result.length() - 1] != '_') {
      result += '_';
    }
  }
  while (result.endsWith("_")) result.remove(result.length() - 1);
  if (result.isEmpty()) result = "signal";
  return result;
}

String uniqueSignalId(const String& label) {
  const String base = slugify(label);
  bool baseInUse = false;
  for (const StoredSignal& signal : signals) {
    if (base == signal.id) {
      baseInUse = true;
      break;
    }
  }
  if (!baseInUse) return base;

  for (uint8_t suffixNumber = 2; suffixNumber < 100; suffixNumber++) {
    const String suffix = "_" + String(suffixNumber);
    String candidate = base;
    const size_t maxBaseLength = 11 - suffix.length();
    if (candidate.length() > maxBaseLength) candidate.remove(maxBaseLength);
    candidate += suffix;
    bool candidateInUse = false;
    for (const StoredSignal& signal : signals) {
      if (candidate == signal.id) {
        candidateInUse = true;
        break;
      }
    }
    if (!candidateInUse) return candidate;
  }
  return "";
}

bool sameSignal(const StoredSignal& first, const StoredSignal& second) {
  if (first.count < 8 || first.count != second.count ||
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

String signalKey(const String& id) {
  return "s_" + id;
}

bool saveSignalIndex() {
  JsonDocument document;
  JsonArray entries = document.to<JsonArray>();
  for (const StoredSignal& signal : signals) {
    JsonObject entry = entries.add<JsonObject>();
    entry["id"] = signal.id;
    entry["label"] = signal.label;
    entry["frequency"] = signal.frequencyMHz;
  }
  String json;
  serializeJson(document, json);
  return preferences.putString("index", json) == json.length();
}

void loadSignals() {
  signals.clear();
  JsonDocument document;
  if (deserializeJson(document, preferences.getString("index", "[]"))) return;
  for (JsonObjectConst entry : document.as<JsonArrayConst>()) {
    const String id = entry["id"] | "";
    if (id.isEmpty() || id.length() > 11 || signals.size() >= MAX_SIGNALS) continue;
    StoredSignal signal{};
    const String key = signalKey(id);
    if (preferences.getBytesLength(key.c_str()) != sizeof(signal)) continue;
    if (preferences.getBytes(key.c_str(), &signal, sizeof(signal)) != sizeof(signal)) continue;
    if (signal.count > MAX_PULSES) continue;
    signals.push_back(signal);
  }
}

String discoveryObjectId(const String& signalId) {
  return String("cc1101_") + bridgeId + "_" + signalId;
}

String discoveryConfigTopic(const String& objectId) {
  return String(MQTT_DISCOVERY_PREFIX) + "/button/" + objectId + "/config";
}

String legacyDiscoveryConfigTopic(const String& signalId) {
  return String(MQTT_DISCOVERY_PREFIX) + "/button/cc1101_" + signalId + "/config";
}

void publishDiscovery(const StoredSignal& signal) {
  if (!mqttClient.connected()) return;
  const String objectId = discoveryObjectId(signal.id);
  const String discoveryTopic = discoveryConfigTopic(objectId);
  const String commandTopic = mqttBase + "/signal/" + signal.id + "/set";
  const String legacyTopic = legacyDiscoveryConfigTopic(signal.id);
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
  identifiers.add(String("cc1101_bridge_") + bridgeId);
  device["name"] = String("CC1101 Bridge ") + bridgeId;
  device["manufacturer"] = "ESP32 / CC1101";
  device["model"] = "OOK signal bridge";
  const size_t length = serializeJson(document, payload, sizeof(payload));
  if (length > 0) {
    mqttClient.publish(legacyTopic.c_str(), "", true);
    mqttClient.publish(discoveryTopic.c_str(), payload, true);
  }
}

void publishAllDiscovery() {
  for (const StoredSignal& signal : signals) publishDiscovery(signal);
}

void sendSignal(const String& id) {
  for (const StoredSignal& signal : signals) {
    if (id != signal.id || signal.count == 0) continue;
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
    lastAction = "Gesendet: " + String(signal.label);
    logEvent("Gesendet: " + String(signal.label));
    return;
  }
  lastAction = "Signal nicht gefunden: " + id;
  logEvent(lastAction);
}

void sendTestSignal() {
  if (!cc1101Detected || captureActive) {
    lastAction = captureActive ? "Testsignal abgelehnt: Aufnahme laeuft" : "Testsignal abgelehnt: CC1101 nicht erkannt";
    logEvent(lastAction);
    return;
  }

  ccConfigure(TEST_SIGNAL_FREQUENCY_MHZ, false);
  pinMode(PIN_CC_GDO0, OUTPUT);
  digitalWrite(PIN_CC_GDO0, LOW);
  ccStrobe(CC_STX);
  for (uint8_t burstIndex = 0; burstIndex < 12; burstIndex++) {
    for (uint8_t pulseIndex = 0; pulseIndex < 12; pulseIndex++) {
      digitalWrite(PIN_CC_GDO0, HIGH);
      delayMicroseconds(800);
      digitalWrite(PIN_CC_GDO0, LOW);
      delayMicroseconds(800);
    }
    delayMicroseconds(8000);
  }
  ccStrobe(CC_SIDLE);
  pinMode(PIN_CC_GDO0, INPUT);
  ccConfigure(TEST_SIGNAL_FREQUENCY_MHZ, true);
  lastAction = "Testsignal gesendet auf 433.92 MHz";
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
  const String id = receivedTopic.substring(prefix.length(), receivedTopic.length() - 4);
  sendSignal(id);
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
    mqttClient.subscribe((mqttBase + "/signal/+/set").c_str());
    publishAllDiscovery();
    lastAction = "MQTT verbunden";
    logEvent("MQTT verbunden");
    lastMqttFailureState = -1;
  } else {
    const int state = mqttClient.state();
    if (state != lastMqttFailureState) {
      logEvent("MQTT-Verbindung fehlgeschlagen (rc " + String(state) + ")");
      lastMqttFailureState = state;
    }
  }
}

String pageStart(const String& title) {
  return String("<!doctype html><html lang='de'><meta charset='utf-8'>") +
    "<meta name='viewport' content='width=device-width,initial-scale=1'><title>" + htmlEscape(title) +
    " · CC1101</title><style>body{font:16px system-ui,sans-serif;max-width:900px;margin:0 auto;padding:18px;"
    "background:#101820;color:#e8eff2}nav{display:flex;gap:16px;padding:12px 0;border-bottom:1px solid #52616b}"
    "a{color:#71d6c5}main{padding-top:16px}.panel{padding:14px 0;border-bottom:1px solid #394952}"
    "input,button{font:inherit;padding:9px;margin:4px 4px 4px 0}input{max-width:100%;box-sizing:border-box}"
    "button{background:#71d6c5;border:0;color:#102126;cursor:pointer}small,.muted{color:#a7b5bc}"
    "form{margin:8px 0}.row{display:flex;gap:8px;flex-wrap:wrap}.row label{display:block}"
    ".setup-form{max-width:680px}.setup-section{padding:14px 0;border-bottom:1px solid #394952}"
    ".setup-section h2{font-size:1.1em;margin:0 0 12px}.form-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,260px),1fr));gap:12px 16px}"
    ".form-grid label{display:flex;flex-direction:column;gap:5px;color:#a7b5bc;font-size:.92em}"
    ".form-grid input{width:100%;margin:0;background:#18242c;color:#e8eff2;border:1px solid #52616b;border-radius:4px}"
    ".form-actions{padding-top:14px}.form-actions button{padding:10px 18px;border-radius:4px}"
    ".recorder{max-width:760px}.recorder-step{padding:12px 0;border-bottom:1px solid #394952}"
    ".recorder-step h3{font-size:1em;margin:0 0 10px;color:#71d6c5}.capture-state{padding:10px 12px;margin-top:10px;"
    "background:#18242c;border-left:3px solid #71d6c5;min-height:22px}.capture-state.warn{border-color:#e6ae61}"
    ".pulse-preview{margin-top:10px;padding:10px;background:#18242c}.pulse-waveform{display:block;width:100%;height:120px;background:#101820}"
    ".pulse-values{max-height:140px;overflow:auto;font:12px ui-monospace,monospace;overflow-wrap:anywhere}"
    ".log-list{max-height:360px;overflow:auto;background:#101820;border:1px solid #394952;padding:8px 12px;font:13px ui-monospace,monospace}"
    ".log-entry{padding:4px 0;border-bottom:1px solid #26343c;overflow-wrap:anywhere}.log-entry:last-child{border:0}"
    "button:disabled{opacity:.45;cursor:not-allowed}"
    "code{overflow-wrap:anywhere}</style><nav><b>CC1101 Bridge</b><a href='/'>Signale</a>"
    "<a href='/settings'>Netzwerk</a><a href='/firmware'>Firmware</a><a href='/restart'>Neustart</a></nav><main>";
}

String pageEnd() {
  return "</main></html>";
}

String settingsForm(bool firstSetup) {
  const String intro = firstSetup
    ? "<h1>WLAN einrichten</h1><p>Mit dem Access Point <b>" + String(AP_NAME) +
      "</b> (Passwort: <code>cc1101setup</code>) verbinden. MQTT kann spaeter eingerichtet werden.</p>"
    : "<h1>Netzwerk / MQTT</h1>";
  const String wifiName = firstSetup ? "" : htmlEscape(wifiSsid);
  const String wifiPasswordHint = firstSetup ? "" : " placeholder='leer = unveraendert'";
  const String mqttPasswordHint = firstSetup ? "" : " placeholder='leer = unveraendert'";
  return intro +
    "<form class='setup-form' method='post' action='/settings'>"
    "<section class='setup-section'><h2>WLAN</h2><div class='form-grid'>"
    "<label>WLAN-Name<input name='ssid' value='" + wifiName + "' required autocomplete='off'></label>"
    "<label>WLAN-Passwort<input type='password' name='wifi_password'" + wifiPasswordHint + " autocomplete='new-password'></label>"
    "</div></section><section class='setup-section'><h2>MQTT</h2><div class='form-grid'>"
    "<label>Broker-Adresse<input name='mqtt_host' value='" + htmlEscape(mqttHost) + "' placeholder='z. B. 192.168.1.10'></label>"
    "<label>Port<input type='number' name='mqtt_port' min='1' max='65535' value='" + String(mqttPort) + "'></label>"
    "<label>Benutzername<input name='mqtt_user' value='" + htmlEscape(mqttUser) + "' autocomplete='off'></label>"
    "<label>MQTT-Passwort<input type='password' name='mqtt_password'" + mqttPasswordHint + " autocomplete='new-password'></label>"
    "</div></section><div class='form-actions'><button>" +
    String(firstSetup ? "Speichern und verbinden" : "Speichern und neu starten") +
    "</button></div></form>" +
    (firstSetup ? "" : "<p class='muted'>Leere Passwortfelder behalten das gespeicherte Passwort.</p>");
}

String renderSignalList() {
  String body;
  if (signals.empty()) body += "<p class='muted'>Noch keine Signale gespeichert.</p>";
  for (const StoredSignal& signal : signals) {
    body += "<section class='panel'><h3>" + htmlEscape(signal.label) + "</h3><p class='muted'>" +
      String(signal.frequencyMHz, 2) + " MHz · " + String(signal.count) + " Pulse · ID <code>" +
      htmlEscape(signal.id) + "</code></p><div class='row'><form method='get' action='/signal'>" +
      "<input type='hidden' name='id' value='" + htmlEscape(signal.id) + "'><button>Daten</button></form>"
      "<form method='post' action='/send'>" +
      "<input type='hidden' name='id' value='" + htmlEscape(signal.id) + "'><button>Senden</button></form>" +
      "<form method='post' action='/delete' onsubmit=\"return confirm('Signal loeschen?')\">" +
      "<input type='hidden' name='id' value='" + htmlEscape(signal.id) + "'><button>Loeschen</button></form></div></section>";
  }
  return body;
}

void handleSignalsExport() {
  String json;
  size_t pulseCount = 0;
  for (const StoredSignal& signal : signals) pulseCount += signal.count;
  json.reserve(48 + signals.size() * 96 + pulseCount * 7);
  json = "{\"format\":\"cc1101-signals\",\"version\":1,\"signals\":[";
  bool first = true;
  for (const StoredSignal& signal : signals) {
    if (!first) json += ',';
    first = false;
    JsonDocument document;
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
    json += signalJson;
  }
  json += "]}";
  webServer.sendHeader("Content-Disposition", "attachment; filename=cc1101-signals.json");
  webServer.send(200, "application/json; charset=utf-8", json);
}

String renderSignalEditor(const String& id) {
  for (const StoredSignal& signal : signals) {
    if (id != signal.id) continue;

    String pulseData;
    pulseData.reserve((size_t)signal.count * 9);
    for (uint16_t index = 0; index < signal.count; index++) {
      pulseData += signal.levels[index] ? "H:" : "L:";
      pulseData += String(signal.durations[index]);
      pulseData += '\n';
    }

    String body = "<h1>Daten bearbeiten</h1><p class='muted'>" + String(signal.count) +
      " Pulse · Pegel und Dauer in Mikrosekunden, ein Eintrag pro Zeile.</p>"
      "<form method='post' action='/signal/save' class='setup-form'>"
      "<input type='hidden' name='id' value='" + htmlEscape(signal.id) + "'>"
      "<section class='setup-section'><div class='form-grid'>"
      "<label>Name<input name='name' maxlength='32' value='" + htmlEscape(signal.label) + "' required></label>"
      "<label>Frequenz (MHz)<input type='number' name='frequency' min='300' max='928' step='0.01' value='" +
      String(signal.frequencyMHz, 2) + "' required></label></div></section>"
      "<section class='setup-section'><label>Pulsfolge (H:µs oder L:µs, pro Zeile)"
      "<textarea name='pulse_data' rows='18' spellcheck='false' required style='display:block;width:100%;box-sizing:border-box;"
      "background:#18242c;color:#e8eff2;border:1px solid #52616b;padding:10px;font:13px ui-monospace,monospace'>" +
      htmlEscape(pulseData) + "</textarea></label></section>"
      "<div class='form-actions'><button>Aenderungen speichern</button> <a href='/'>Abbrechen</a></div></form>";
    return pageStart("Signal bearbeiten") + body + pageEnd();
  }
  return pageStart("Signal nicht gefunden") + "<h1>Signal nicht gefunden</h1><p><a href='/'>Zurueck</a></p>" + pageEnd();
}

void handleSignalImport() {
  JsonDocument document;
  const DeserializationError error = deserializeJson(document, webServer.arg("plain"));
  if (error) {
    webServer.send(400, "text/plain", "JSON ungueltig");
    return;
  }

  const String id = document["id"] | "";
  const String label = document["label"] | "";
  const float frequency = document["frequencyMHz"] | 0.0f;
  JsonArrayConst durations = document["durations"].as<JsonArrayConst>();
  JsonArrayConst levels = document["levels"].as<JsonArrayConst>();
  if (id.isEmpty() || id.length() > 11 || label.isEmpty() || label.length() > 32 ||
      !validFrequency(frequency) || durations.size() < 4 || durations.size() > MAX_PULSES ||
      levels.size() != durations.size()) {
    webServer.send(400, "text/plain", "Signal-Metadaten oder Pulsfolge ungueltig");
    return;
  }
  for (size_t index = 0; index < id.length(); index++) {
    const char ch = id[index];
    if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_')) {
      webServer.send(400, "text/plain", "Signal-ID ungueltig");
      return;
    }
  }

  StoredSignal imported{};
  strlcpy(imported.id, id.c_str(), sizeof(imported.id));
  strlcpy(imported.label, label.c_str(), sizeof(imported.label));
  imported.frequencyMHz = frequency;
  imported.count = (uint16_t)durations.size();
  for (uint16_t index = 0; index < imported.count; index++) {
    const uint32_t duration = durations[index] | 0;
    const int level = levels[index] | -1;
    if (duration < 1 || duration > 65535 || (level != LOW && level != HIGH)) {
      webServer.send(400, "text/plain", "Pulsdauer oder Pegel ungueltig");
      return;
    }
    imported.durations[index] = (uint16_t)duration;
    imported.levels[index] = (uint8_t)level;
  }

  size_t signalIndex = signals.size();
  for (size_t index = 0; index < signals.size(); index++) {
    if (id == signals[index].id) signalIndex = index;
    else if (sameSignal(signals[index], imported)) {
      webServer.send(409, "text/plain", "Pulsfolge bereits unter einer anderen ID gespeichert");
      return;
    }
  }
  if (signalIndex == signals.size() && signals.size() >= MAX_SIGNALS) {
    webServer.send(409, "text/plain", "Maximale Anzahl gespeicherter Signale erreicht");
    return;
  }

  const bool replacing = signalIndex < signals.size();
  const StoredSignal previous = replacing ? signals[signalIndex] : StoredSignal{};
  const String key = signalKey(id);
  if (preferences.putBytes(key.c_str(), &imported, sizeof(imported)) != sizeof(imported)) {
    webServer.send(500, "text/plain", "NVS konnte das Signal nicht speichern");
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
    webServer.send(500, "text/plain", "Signalindex konnte nicht gespeichert werden");
    return;
  }
  publishDiscovery(imported);
  lastAction = String(replacing ? "Importiert/aktualisiert: " : "Importiert: ") + label;
  logEvent(lastAction);
  webServer.send(200, "text/plain", "OK");
}

void handleRoot() {
  if (WiFi.status() != WL_CONNECTED) {
    const String body = settingsForm(true);
    webServer.send(200, "text/html; charset=utf-8", pageStart("WLAN Setup") + body + pageEnd());
    return;
  }

  String body = "<h1>Signalverwaltung</h1><section class='panel recorder'><h2>Recorder</h2>"
    "<div class='recorder-step'><h3>1. Signal aufnehmen</h3>"
    "<form method='post' action='/capture/start' class='form-grid' id='capture-form'>"
    "<label>Frequenz (MHz)<input type='number' name='frequency' min='300' max='928' step='0.01' value='433.92' required></label>"
    "<label>Dauer (Sekunden)<input type='number' name='seconds' min='1' max='10' value='3' required></label>"
    "<label>Stoerfilter / Mindeststaerke (0-7)<input type='range' name='strength' min='0' max='7' step='1' value='" + String(captureGainReductionStep) + "'><output id='strength-value'></output></label>"
    "<div class='form-actions'><button id='capture-button'>Aufnahme starten</button></div></form>"
    "<div id='capture-state' class='capture-state' aria-live='polite'>Noch keine Aufnahme</div>"
    "<div class='pulse-preview'><canvas id='pulse-waveform' class='pulse-waveform' aria-label='Zeitdiagramm der empfangenen Pulse'></canvas>"
    "<p id='pulse-summary' class='muted'>Nach einer Aufnahme erscheint hier das Zeitdiagramm.</p>"
    "<details><summary>Erste 120 Pulsdauern</summary><p id='pulse-values' class='pulse-values'></p></details></div></div>"
    "<div class='recorder-step'><h3>2. Aufnahme benennen und speichern</h3>"
    "<form method='post' action='/capture/save' class='row'>"
    "<label>Name <input name='name' maxlength='32' required></label>"
    "<button id='save-button' disabled>Signal speichern</button></form></div>"
    "<div class='recorder-step'><h3>3. CC1101-Funktest</h3><p class='muted'>Sendet einen kurzen OOK-Testburst auf 433.92 MHz, keinen Ventilatorbefehl. Am zweiten Empfaenger eine Aufnahme auf 433.92 MHz starten.</p>"
    "<form method='post' action='/test-send'><button>Testsignal senden</button></form></div></section>" +
    "<section class='panel'><h2>Signale sichern / uebertragen</h2><div class='row'>"
    "<a href='/signals/export' download>JSON-Datei exportieren</a>"
    "<label>JSON-Datei importieren <input id='signal-import-file' type='file' accept='.json,application/json'></label>"
    "<button id='signal-import-button' type='button'>Importieren</button></div>"
    "<p id='signal-import-state' class='muted' aria-live='polite'></p></section>" + renderSignalList() +
    "<section class='panel'><h2>Letzte Ereignisse</h2><p id='device-stats' class='muted'></p>"
    "<div id='event-log' class='log-list' aria-live='polite'>Noch keine Ereignisse</div></section>" +
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
    "+durations.length+' Pulse · '+data.frequency.toFixed(2)+' MHz · '+(total/1000).toFixed(1)+' ms angezeigt · Dauer '+min+'-'+max+' µs · LOW-Pausen >5 ms komprimiert';"
    "document.getElementById('pulse-values').textContent=durations.slice(0,120).map((value,index)=>"
    "+(levels[index]?'HIGH ':'LOW ')+value+' µs').join(' · ');}catch(error){}}"
    "async function refreshStatus(){try{const response=await fetch('/api/status',{cache:'no-store'});"
    "const data=await response.json();const state=document.getElementById('capture-state');"
    "const captureLabel=data.capture?'Aufnahme laeuft':(data.pulses?'Aufnahme bereit':'Keine Pulse empfangen');"
    "state.textContent=captureLabel+' · '+data.pulses+' Pulse · GPIO4/GDO0 '+(data.gdo0?'HIGH':'LOW')+' · GPIO27/CS '+(data.carrier?'HIGH':'LOW');"
    "state.classList.toggle('warn',!data.pulses&&!data.capture);const captureButton=document.getElementById('capture-button');"
    "captureButton.disabled=data.capture;captureButton.textContent=data.capture?'Aufnahme laeuft':'Aufnahme starten';"
    "document.getElementById('save-button').disabled=data.capture||data.pulses<4;"
    "document.getElementById('device-stats').textContent='RSSI '+(data.rssi_dbm===undefined?'--':data.rssi_dbm.toFixed(1)+' dBm')+' · Log '+data.log_entries+'/'+data.log_capacity+' · Signale '+data.signals;"
    "if(!data.capture&&data.pulses>0&&data.capture_id!==shownCaptureId){shownCaptureId=data.capture_id;loadPulsePreview();}"
    "const log=document.getElementById('event-log');log.replaceChildren();if(!data.log.length){log.textContent='Noch keine Ereignisse';return;}"
    "for(let i=data.log.length-1;i>=0;i--){const item=document.createElement('div');item.className='log-entry';"
    "item.textContent=(data.log[i].seconds.toFixed(1)+' s · '+data.log[i].message);log.append(item);}}catch(error){}}"
    "document.getElementById('capture-form').addEventListener('submit',async(event)=>{event.preventDefault();"
    "const form=event.currentTarget;const button=document.getElementById('capture-button');const state=document.getElementById('capture-state');"
    "button.disabled=true;button.textContent='Aufnahme startet...';state.textContent='Aufnahme wird gestartet';"
    "try{const response=await fetch(form.action,{method:'POST',headers:{'X-Requested-With':'fetch'},"
    "body:new URLSearchParams(new FormData(form))});"
    "if(!response.ok)throw new Error(await response.text());await refreshStatus();}catch(error){button.disabled=false;"
    "button.textContent='Aufnahme starten';state.textContent=error.message;state.classList.add('warn');}});"
    "const strengthInput=document.querySelector('#capture-form input[name=strength]');"
    "const gainStepsDb=[0,2.6,6.1,7.4,9.2,11.5,14.6,17.1];"
    "const updateStrengthLabel=()=>{const value=Number(strengthInput.value);document.getElementById('strength-value').value='Stufe '+value+' (~'+gainStepsDb[value]+' dB)';};"
    "strengthInput.addEventListener('input',updateStrengthLabel);updateStrengthLabel();"
    "document.getElementById('signal-import-button').addEventListener('click',async()=>{"
    "const input=document.getElementById('signal-import-file');const state=document.getElementById('signal-import-state');"
    "if(!input.files.length){state.textContent='Bitte zuerst eine JSON-Datei auswaehlen';return;}"
    "const button=document.getElementById('signal-import-button');button.disabled=true;"
    "try{const backup=JSON.parse(await input.files[0].text());"
    "if(backup.format!=='cc1101-signals'||backup.version!==1||!Array.isArray(backup.signals))"
    "throw new Error('Dateiformat oder Version wird nicht unterstuetzt');"
    "let imported=0;for(const signal of backup.signals){const response=await fetch('/signal/import',{method:'POST',"
    "headers:{'Content-Type':'application/json'},body:JSON.stringify(signal)});"
    "if(!response.ok)throw new Error((await response.text())+' ('+imported+' von '+backup.signals.length+' importiert)');"
    "imported++;state.textContent='Importiert: '+imported+' von '+backup.signals.length;}"
    "state.textContent='Import abgeschlossen: '+imported+' Signale';location.reload();}"
    "catch(error){state.textContent='Importfehler: '+error.message;}finally{button.disabled=false;}});"
    "refreshStatus();setInterval(refreshStatus,1000);</script>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Signale") + body + pageEnd());
}

void handleCaptureStart() {
  const bool isAjax = webServer.hasHeader("X-Requested-With");
  const float frequency = webServer.arg("frequency").toFloat();
  const long secondsValue = webServer.arg("seconds").toInt();
  const long strengthValue = webServer.hasArg("strength") ? webServer.arg("strength").toInt() : DEFAULT_LNA_GAIN_REDUCTION_STEP;
  if (captureActive) {
    logEvent("Aufnahmestart abgelehnt: Aufnahme laeuft bereits");
    if (isAjax) { webServer.send(204); return; }
    webServer.sendHeader("Location", "/");
    webServer.send(303);
    return;
  }
  if (!validFrequency(frequency)) {
    webServer.send(400, "text/plain", "Frequenz ungueltig. Erlaubte CC1101-Baender: 300-348, 387-464 oder 779-928 MHz.");
    return;
  }
  if (secondsValue < 1 || secondsValue > 10) {
    webServer.send(400, "text/plain", "Aufnahmedauer muss zwischen 1 und 10 Sekunden liegen.");
    return;
  }
  if (strengthValue < 0 || strengthValue > 7) {
    webServer.send(400, "text/plain", "Stoerfilter muss zwischen 0 und 7 liegen.");
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
  lastAction = "Aufnahme laeuft";
  logEvent("Aufnahme gestartet: " + String(frequency, 2) + " MHz, " + String(seconds) + " s, Stoerfilter Stufe " + String(captureGainReductionStep));
  if (isAjax) { webServer.send(204); return; }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void finishCapture() {
  if (!captureActive) return;
  detachInterrupt(digitalPinToInterrupt(PIN_CC_GDO0));
  captureActive = false;
  ccStrobe(CC_SIDLE);
  ccStrobe(CC_SRX);
  lastAction = "Aufnahme beendet";
  logEvent("Aufnahme beendet: " + String(captureCount) + " Pulse, GPIO4/GDO0 " +
           String(digitalRead(PIN_CC_GDO0) ? "HIGH" : "LOW"));
}

void handleCaptureSave() {
  if (captureActive) finishCapture();
  const String label = webServer.arg("name");
  if (label.isEmpty() || captureCount < 4) {
    logEvent("Nicht gespeichert: weniger als 4 Pulse oder Name fehlt");
    webServer.send(400, "text/plain", "Name fehlt oder keine brauchbare Aufnahme vorhanden");
    return;
  }
  if (signals.size() >= MAX_SIGNALS) {
    webServer.send(400, "text/plain", "Maximale Anzahl gespeicherter Signale erreicht");
    return;
  }
  const String id = uniqueSignalId(label);
  if (id.isEmpty()) {
    webServer.send(400, "text/plain", "Keine eindeutige Signal-ID verfuegbar");
    return;
  }
  StoredSignal signal{};
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

  for (const StoredSignal& existing : signals) {
    if (!sameSignal(existing, signal)) continue;
    lastAction = "Duplikat nicht gespeichert: " + label;
    logEvent(lastAction);
    webServer.sendHeader("Location", "/");
    webServer.send(303);
    return;
  }
  const String key = signalKey(id);
  if (preferences.putBytes(key.c_str(), &signal, sizeof(signal)) != sizeof(signal)) {
    lastAction = "Speichern fehlgeschlagen: NVS voll oder Schreibfehler";
    logEvent(lastAction);
    webServer.send(500, "text/plain", lastAction);
    return;
  }
  signals.push_back(signal);
  if (!saveSignalIndex()) {
    signals.pop_back();
    preferences.remove(key.c_str());
    lastAction = "Speichern fehlgeschlagen: Signalindex konnte nicht geschrieben werden";
    logEvent(lastAction);
    webServer.send(500, "text/plain", lastAction);
    return;
  }
  publishDiscovery(signal);
  lastAction = "Gespeichert: " + label;
  logEvent("Signal gespeichert: " + label + " (" + String(signal.count) + " Pulse)");
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleSend() {
  sendSignal(webServer.arg("id"));
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleSignalEditor() {
  webServer.send(200, "text/html; charset=utf-8", renderSignalEditor(webServer.arg("id")));
}

void handleSignalSave() {
  const String id = webServer.arg("id");
  size_t signalIndex = signals.size();
  for (size_t index = 0; index < signals.size(); index++) {
    if (id == signals[index].id) {
      signalIndex = index;
      break;
    }
  }
  if (signalIndex == signals.size()) {
    webServer.send(404, "text/plain", "Signal nicht gefunden");
    return;
  }

  const float frequency = webServer.arg("frequency").toFloat();
  const String label = webServer.arg("name");
  if (!validFrequency(frequency) || label.isEmpty()) {
    webServer.send(400, "text/plain", "Name oder Frequenz ungueltig");
    return;
  }

  StoredSignal updated = signals[signalIndex];
  strlcpy(updated.label, label.c_str(), sizeof(updated.label));
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
        webServer.send(400, "text/plain", "Pulsfolge ungueltig oder zu lang");
        return;
      }
      const String durationText = line.substring(2);
      for (size_t digit = 0; digit < durationText.length(); digit++) {
        if (!isDigit(durationText[digit])) {
          webServer.send(400, "text/plain", "Pulsdauer muss eine Zahl sein");
          return;
        }
      }
      const long duration = durationText.toInt();
      if (duration < 1 || duration > 65535) {
        webServer.send(400, "text/plain", "Pulsdauer muss zwischen 1 und 65535 us liegen");
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
    webServer.send(400, "text/plain", "Mindestens vier Pulse sind erforderlich");
    return;
  }

  for (size_t index = 0; index < signals.size(); index++) {
    if (index != signalIndex && sameSignal(signals[index], updated)) {
      webServer.send(409, "text/plain", "Diese Pulsfolge ist bereits unter einem anderen Namen gespeichert");
      return;
    }
  }

  const StoredSignal previous = signals[signalIndex];
  const String key = signalKey(id);
  if (preferences.putBytes(key.c_str(), &updated, sizeof(updated)) != sizeof(updated)) {
    webServer.send(500, "text/plain", "NVS konnte die Aenderung nicht speichern");
    return;
  }
  signals[signalIndex] = updated;
  if (!saveSignalIndex()) {
    signals[signalIndex] = previous;
    preferences.putBytes(key.c_str(), &previous, sizeof(previous));
    webServer.send(500, "text/plain", "Signalindex konnte nicht gespeichert werden");
    return;
  }
  publishDiscovery(updated);
  lastAction = "Signal bearbeitet: " + label;
  logEvent(lastAction);
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleTestSend() {
  sendTestSignal();
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleDelete() {
  const String id = webServer.arg("id");
  for (size_t index = 0; index < signals.size(); index++) {
    if (id != signals[index].id) continue;
    if (mqttClient.connected()) {
      const String discoveryTopic = discoveryConfigTopic(discoveryObjectId(id));
      const String legacyTopic = legacyDiscoveryConfigTopic(id);
      mqttClient.publish(discoveryTopic.c_str(), "", true);
      mqttClient.publish(legacyTopic.c_str(), "", true);
    }
    preferences.remove(signalKey(id).c_str());
    signals.erase(signals.begin() + index);
    saveSignalIndex();
    break;
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleApiStatus() {
  JsonDocument document;
  document["capture"] = captureActive;
  document["pulses"] = captureCount;
  document["capture_id"] = captureSequence;
  document["log_entries"] = eventLogCount;
  document["log_capacity"] = MAX_LOG_ENTRIES;
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
    webServer.send(409, "text/plain", "Aufnahme laeuft noch");
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
  webServer.send(200, "text/html; charset=utf-8", pageStart("Gespeichert") +
    "<h1>Einstellungen gespeichert</h1><p>Der Bridge startet neu.</p>" + pageEnd());
  delay(800);
  ESP.restart();
}

void handleSettingsPage() {
  const String body = settingsForm(false);
  webServer.send(200, "text/html; charset=utf-8", pageStart("Netzwerk") + body + pageEnd());
}

void handleFirmwarePage() {
  const String body = String("<h1>Firmware aktualisieren</h1><form method='post' action='/firmware' enctype='multipart/form-data'>") +
    "<input type='file' name='firmware' accept='.bin' required><button>Firmware installieren</button></form>";
  webServer.send(200, "text/html; charset=utf-8", pageStart("Firmware") + body + pageEnd());
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
  webServer.send(success ? 200 : 500, "text/plain", success ? "Update erfolgreich, Neustart" : "Firmware-Update fehlgeschlagen");
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
  webServer.on("/signals/export", HTTP_GET, handleSignalsExport);
  webServer.on("/signal/import", HTTP_POST, handleSignalImport);
  webServer.on("/api/status", HTTP_GET, handleApiStatus);
  webServer.on("/api/pulses", HTTP_GET, handleApiPulses);
  webServer.on("/capture/start", HTTP_POST, handleCaptureStart);
  webServer.on("/capture/save", HTTP_POST, handleCaptureSave);
  webServer.on("/send", HTTP_POST, handleSend);
  webServer.on("/signal", HTTP_GET, handleSignalEditor);
  webServer.on("/signal/save", HTTP_POST, handleSignalSave);
  webServer.on("/test-send", HTTP_POST, handleTestSend);
  webServer.on("/delete", HTTP_POST, handleDelete);
  webServer.on("/settings", HTTP_GET, handleSettingsPage);
  webServer.on("/settings", HTTP_POST, handleSettings);
  webServer.on("/firmware", HTTP_GET, handleFirmwarePage);
  webServer.on("/firmware", HTTP_POST, handleFirmwareDone, handleFirmwareUpload);
  webServer.on("/restart", HTTP_GET, []() {
    webServer.send(200, "text/plain", "Neustart");
    delay(300);
    ESP.restart();
  });
  webServer.begin();
  webStarted = true;
}

void startOta() {
  if (otaStarted || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();
  MDNS.begin(hostname);
  otaStarted = true;
}

void connectWifi() {
  if (wifiSsid.isEmpty()) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME, "cc1101setup");
    logEvent("Setup-Access-Point gestartet");
    startWebServer();
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < 15000) {
    delay(250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    logEvent("WLAN verbunden: " + WiFi.localIP().toString());
    startWebServer();
    startOta();
  } else {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_NAME, "cc1101setup");
    logEvent("WLAN nicht erreichbar, Setup-Access-Point gestartet");
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
  snprintf(hostname, sizeof(hostname), "cc1101-%s", bridgeId);
  logEvent("Bridge gestartet: " + String(hostname));
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

  ccStrobe(CC_SRES);
  delay(5);
  cc1101PartNumber = ccReadStatusRegister(CC_PARTNUM);
  cc1101Version = ccReadStatusRegister(CC_VERSION);
  cc1101Detected = isKnownCc1101Version(cc1101Version);
  if (cc1101Detected) {
    logEvent("CC1101 erkannt: PARTNUM 0x" + String(cc1101PartNumber, HEX) +
             ", VERSION 0x" + String(cc1101Version, HEX));
  } else {
    logEvent("CC1101 nicht erkannt: PARTNUM 0x" + String(cc1101PartNumber, HEX) +
             ", VERSION 0x" + String(cc1101Version, HEX) + " - SPI/Versorgung pruefen");
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