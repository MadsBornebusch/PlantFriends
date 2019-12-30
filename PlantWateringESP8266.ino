#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>

#include "AsyncElegantOtaSpiffs.h"
#include "secret.h"

#define SW_VERSION "1.0.0"

// Define the access point SSID and password
#define WIFI_AP_SSID "PlantWateringESP8266"
#define WIFI_AP_PASSWORD "plantsarecool"

// Default sleep time in minutes
// Note that this can never be lower than "SLEEP_INTERVAL"
#define DEFAULT_SLEEP_TIME (10U)

// Default minimum time between watering plant (minutes) and watering delay
#define DEFAULT_WATERING_DELAY (60U)

// Default watering threshold in clock cycles
#define DEFAULT_WATERING_THRESHOLD (3200U)

// Default watering time in seconds
#define DEFAULT_WATERING_TIME (3U)

// Define number of soil measurements
#define N_SOIL_MEAS (4U)

// Sleep interval can safely be set from 1 to 30 minutes (possibly higher, but there is a limitation to sleep time with the esp8266). Should be smaller or equal to SLEEP_TIME
#define SLEEP_INTERVAL (10U)

// Calculate sleep interval and number of sleeps
#define SLEEP_INTERVAL_US (SLEEP_INTERVAL * 60UL * 1000000UL)

// RTC memory address to use for sleep counter - note that the first 128 bytes are used by the OTA and the offset is set in block of 4 bytes, so we set the value to 128/4=32, so OTA still works
#define SLEEP_DATA_ADDR (32U)

// Define whether to post to Thingspeak
#define POST_TO_THINGSPEAK

// Define soil output and measurement pins
#define SOIL_OUT (14U)
#define SOIL_IN (12U)

// Define watering pin
#define WATERING_OUT (13U)

// Define LED pin for built in LED (ESP-12 board)
#define LED_PIN (2U)

// UART pins - if these are shorted at boot, then it will turn on the access point
#define TX_PIN (1U)
#define RX_PIN (3U)

// This is used to determine if the EEPROM and RTC memory has been configured
static const uint64_t MAGIC_NUMBER = 0x3b04cd9e94521f9a;

typedef struct {
  // Can be set via the web interface
  uint64_t magic_number; // Used to determine if the EEPROM have been configured or not
  char wifi_ssid[33]; // SSID should be a maximum of 32 characters according to the specs
  char wifi_password[33]; // Restrict password length to 32 characters
  char thingspeak_api_key[17]; // The API is 16 characters
  char mqtt_host[51]; // Restrict URLs to 50 characters
  uint16_t mqtt_port;
  char mqtt_username[33]; // Just set these to 32 as well
  char mqtt_password[33];
  char mqtt_base_topic[33];

  // Can be set via MQTT and the web interface
  uint8_t sleep_time;
  uint16_t watering_delay;
  uint16_t watering_threshold;
  uint8_t watering_time;
  bool automatic_ota; // Flag used to enable automatic OTA
  bool override_retained_config_topic; // Used to override the retained config topic, so the values set via the web interface does not get overriden
} eeprom_config_t;

typedef struct {
  uint64_t magic_number; // Used to determine if the RTC memory has been configured or not
  uint8_t sleep_num; // Stores the current sleep interval number
  uint8_t watering_delay_cycles; // Stores the number of cycles the board must sleep before it will water the plant again
  uint16_t firmware_update_counter; // Used to only check for updates every 24 hrs
} __attribute__ ((packed, aligned(4))) sleep_data_t; // The struct needs to be 4-byte aligned in the RTC memory

// Wifi
WiFiClient client;
//#define WIFI_SSID "YourWifiSSID" // Defined in secret.h
//#define WIFI_PASSWORD "YourWifiPassword" // Defined in secret.h

// MQTT
AsyncMqttClient mqttClient;
//#define MQTT_HOST "192.168.1.10" // Defined in secret.h
//#define MQTT_PORT 1883 // Defined in secret.h
//#define MQTT_USERNAME "" // Defined in secret.h
//#define MQTT_PASSWORD "" // Defined in secret.h
//#define MQTT_BASE_TOPIC "" // Defined in secret.h

// ThingSpeak variables
//#define THINGSPEAK_API_KEY "YourWriteApiKey" // Defined in secret.h
const char* thingspeak_server = "api.thingspeak.com";
const char* thingspeak_resource = "/update?api_key=";

// Used for the AP (Access Point) - will be turned on at first boot
static AsyncWebServer httpServer(80);

// Used for updating the firmware and file system via HTTP
static AsyncElegantOtaSpiffs elegantUpdater(LED_PIN, LOW);

// Flag and timer used for measuring the time it takes for the voltage to rise over the capacitive sensor
static volatile bool soil_timer_flag;
static volatile uint32_t soil_timer;

// Used to check if a MQTT package was succesfully sent
static volatile uint16 publishedPacketId = -1;

/*
long getMedian(long &array, uint8_t n){
  //n = sizeof(array[0])/sizeof(array);

  // Sort array
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      if (array[j] > array[i]){
        int tmp = array[i];
        array[i] = array[j];
        array[j] = tmp;
      }

  for(i=0, i<sizeof(array[0])/sizeof(array), i++)

}
*/

static long getMean(int *array, uint8_t n) {
  long sum = 0;
  for (uint8_t i = 0; i < n; i++)
    sum += array[i];
  return sum/n;
}

static void handleLongSleep(sleep_data_t *sleep_data) {
  if (sizeof(sleep_data_t) % 4 != 0) {
    Serial.println(F("Sleep data is NOT 4 byte aligned! Rebooting..."));
    delay(5000);
    ESP.restart();
  }

  if (!ESP.rtcUserMemoryRead(SLEEP_DATA_ADDR, (uint32_t*)sleep_data, sizeof(sleep_data_t)))
    Serial.println(F("Failed to read RTC user memory"));

  if (sleep_data->magic_number != MAGIC_NUMBER) {
    Serial.println(F("Failed to read sleep data from the RTC memory"));
    sleep_data->magic_number = MAGIC_NUMBER;
    sleep_data->sleep_num = 1;
    sleep_data->watering_delay_cycles = 1;
    sleep_data->firmware_update_counter = 0;
  } else {
    // We just woke up, so increment the counter
    sleep_data->firmware_update_counter++;
  }

  Serial.print(F("Sleep number: ")); Serial.print(sleep_data->sleep_num);
  Serial.print(F(", watering delay cycles: ")); Serial.print(sleep_data->watering_delay_cycles);
  Serial.print(F(", firmware update counter: ")); Serial.println(sleep_data->firmware_update_counter);

  // Check if we should go back to sleep immediately
  if (--sleep_data->sleep_num != 0) {
    Serial.print(F("Going to sleep again. Times left to sleep: ")); Serial.println(sleep_data->sleep_num);
    if (!ESP.rtcUserMemoryWrite(SLEEP_DATA_ADDR, (uint32_t*)sleep_data, sizeof(sleep_data_t)))
      Serial.println(F("Failed to write RTC user memory"));
    ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
  }
}

static void longSleep(const eeprom_config_t *eeprom_config, sleep_data_t *sleep_data) {
  if (sizeof(sleep_data_t) % 4 != 0) {
    Serial.println(F("Sleep data is NOT 4 byte aligned! Rebooting..."));
    delay(5000);
    ESP.restart();
  }

  sleep_data->magic_number = MAGIC_NUMBER;
  sleep_data->sleep_num = eeprom_config->sleep_time / SLEEP_INTERVAL; // Set the number of times it should wakeup and go back to sleep again immediately
  Serial.print(F("Number of sleeps left: ")); Serial.println(sleep_data->sleep_num);
  if (!ESP.rtcUserMemoryWrite(SLEEP_DATA_ADDR, (uint32_t*)sleep_data, sizeof(sleep_data_t)))
    Serial.println(F("Failed to write RTC user memory"));
  ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
}

// Copy of ESP.getCycleCount(), but can safely be called inside an interrupt
static inline ICACHE_RAM_ATTR uint32_t GetCycleCountIRQ() {
  uint32_t ccount;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

static void ICACHE_RAM_ATTR soilInterrupt() {
  soil_timer = GetCycleCountIRQ();
  soil_timer_flag = true;
}

static bool readSoil(int *soil_measurements, uint8_t n_meas) {
  pinMode(SOIL_OUT, OUTPUT);
  pinMode(SOIL_IN, INPUT);

  bool result = true;
  for (uint8_t i = 0; i < n_meas; i++) {
    Serial.print(i); Serial.print(": ");
    digitalWrite(SOIL_OUT, LOW);
    delay(10); // Wait for the voltage to drop
    attachInterrupt(digitalPinToInterrupt(SOIL_IN), soilInterrupt, RISING);
    soil_timer_flag = false; // Reset the flag used for telling when the interrupt has triggered
    uint32_t timer_start = ESP.getCycleCount(); // Reset the counter
    digitalWrite(SOIL_OUT, HIGH); // Now set the pin high and measure the rise time using the "SOIL_IN" pin
    int timeout = 1 * 100; // 1 second(s)
    while (!soil_timer_flag && (timeout-- > 0))
      delay(10);
    detachInterrupt(digitalPinToInterrupt(SOIL_IN));
    if (timeout == 0) {  // It took more than x seconds for the voltage to rise
      result = false;
      break;
    } else {
      soil_measurements[i] = soil_timer - timer_start;
      Serial.println(soil_measurements[i]);
    }
  }

  pinMode(SOIL_OUT, INPUT); // Save power
  return result;
}

static void blinkLED() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);
  pinMode(LED_PIN, INPUT); // Save power
}

static void onMqttPublish(uint16_t packetId) {
  publishedPacketId = packetId;
  //Serial.printf("Publish acknowledged - packet ID: %u\n", packetId);
}

static bool mqttPublishBlocking(String topic, const char *buffer, size_t length, bool retain, int timeout) {
  publishedPacketId = -1;
  uint16_t packetId = mqttClient.publish(topic.c_str(), 2, retain, buffer, length);
  while (publishedPacketId != packetId && (timeout-- > 0))
    delay(100);
  return timeout > 0;
}

static void startAsyncHotspot(bool *p_run_hotspot, eeprom_config_t *eeprom_config) {
  // Configure the hotspot
  // Note that we set the maximum number of connection to 1
  int channel = 1, ssid_hidden = 0, max_connection = 1;
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, channel, ssid_hidden, max_connection)) {
    Serial.println(F("Failed to start hotspot! Rebooting..."));
    delay(5000);
    ESP.restart();
  }

  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  IPAddress ip = WiFi.softAPIP();
  Serial.print(F("AP IP address: ")); Serial.println(ip);

  MDNS.begin(F("plantwateringesp8266"));
  MDNS.addService(F("http"), F("tcp"), 80);
  Serial.println(F("Hostname: http://plantwateringesp8266.local"));

  if (!SPIFFS.begin()) {
    Serial.println(F("An Error has occurred while mounting SPIFFS! Rebooting..."));
    delay(5000);
    ESP.restart();
  }

  elegantUpdater.begin(&httpServer); // Start ElegantOTA

  // Add routes for the different files and pages
  httpServer.on("/pure-min.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Use Pure to style the from: https://purecss.io/forms/
    request->send(SPIFFS, F("/pure-min.css"), F("text/css"));
  });
  httpServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Som additional css to make it look a little nicer
    request->send(SPIFFS, F("/style.css"), F("text/css"));
  });

  // This is the main page with the form for configuring the device
  httpServer.on("/", HTTP_GET, [&eeprom_config, &p_run_hotspot](AsyncWebServerRequest *request) {
    auto processor = [&eeprom_config](const String &var) {
      if (var == F("WIFI_SSID"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->wifi_ssid : WIFI_SSID);
      else if (var == F("WIFI_PASSWORD"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->wifi_password : WIFI_PASSWORD);
      else if (var == F("THINGSPEAK_API_KEY"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->thingspeak_api_key : THINGSPEAK_API_KEY);
      else if (var == F("MQTT_HOST"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->mqtt_host : MQTT_HOST);
      else if (var == F("MQTT_PORT"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->mqtt_port : MQTT_PORT);
      else if (var == F("MQTT_USERNAME"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->mqtt_username : MQTT_USERNAME);
      else if (var == F("MQTT_PASSWORD"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->mqtt_password : MQTT_PASSWORD);
      else if (var == F("MQTT_BASE_TOPIC"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->mqtt_base_topic : MQTT_BASE_TOPIC);
      else if (var == F("sleep_time"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->sleep_time : DEFAULT_SLEEP_TIME);
      else if (var == F("watering_delay"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->watering_delay : DEFAULT_WATERING_DELAY);
      else if (var == F("watering_threshold"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->watering_threshold : DEFAULT_WATERING_THRESHOLD);
      else if (var == F("watering_time"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? eeprom_config->watering_time : DEFAULT_WATERING_TIME);
      else if (var == F("automatic_ota"))
        return String(eeprom_config->magic_number == MAGIC_NUMBER ? (eeprom_config->automatic_ota ? "checked" : "") : "checked");
      return String();
    };

    // Send the index as a template
    request->send(SPIFFS, F("/index.html"), F("text/html"), false, processor);
  });

  // Handle the post request
  httpServer.on("/", HTTP_POST, [&eeprom_config, &p_run_hotspot](AsyncWebServerRequest *request) {
    if (!request->hasArg(F("wifi_ssid")) || !request->hasArg(F("wifi_password"))
      || !request->hasArg(F("thingspeak_api_key"))
      || !request->hasArg(F("mqtt_host")) || !request->hasArg(F("mqtt_port"))
      || !request->hasArg(F("mqtt_username")) || !request->hasArg(F("mqtt_password")) || !request->hasArg(F("mqtt_base_topic"))
      || !request->hasArg(F("sleep_time")) || !request->hasArg(F("watering_delay"))
      || !request->hasArg(F("watering_threshold")) || !request->hasArg(F("watering_time"))) {
      request->send(400, F("text/plain"), F("400: Invalid request"));
      return;
    }

    strncpy(eeprom_config->wifi_ssid, request->arg(F("wifi_ssid")).c_str(), sizeof(eeprom_config->wifi_ssid) - 1);
    eeprom_config->wifi_ssid[sizeof(eeprom_config->wifi_ssid) - 1] = '\0'; // Make sure the buffer is null-terminated

    strncpy(eeprom_config->wifi_password, request->arg(F("wifi_password")).c_str(), sizeof(eeprom_config->wifi_password) - 1);
    eeprom_config->wifi_password[sizeof(eeprom_config->wifi_password) - 1] = '\0'; // Make sure the buffer is null-terminated

    strncpy(eeprom_config->thingspeak_api_key, request->arg(F("thingspeak_api_key")).c_str(), sizeof(eeprom_config->thingspeak_api_key) - 1);
    eeprom_config->thingspeak_api_key[sizeof(eeprom_config->thingspeak_api_key) - 1] = '\0'; // Make sure the buffer is null-terminated

    strncpy(eeprom_config->mqtt_host, request->arg(F("mqtt_host")).c_str(), sizeof(eeprom_config->mqtt_host) - 1);
    eeprom_config->mqtt_host[sizeof(eeprom_config->mqtt_host) - 1] = '\0'; // Make sure the buffer is null-terminated

    eeprom_config->mqtt_port = request->arg(F("mqtt_port")).toInt();

    strncpy(eeprom_config->mqtt_username, request->arg(F("mqtt_username")).c_str(), sizeof(eeprom_config->mqtt_username) - 1);
    eeprom_config->mqtt_username[sizeof(eeprom_config->mqtt_username) - 1] = '\0'; // Make sure the buffer is null-terminated

    strncpy(eeprom_config->mqtt_password, request->arg(F("mqtt_password")).c_str(), sizeof(eeprom_config->mqtt_password) - 1);
    eeprom_config->mqtt_password[sizeof(eeprom_config->mqtt_password) - 1] = '\0'; // Make sure the buffer is null-terminated

    strncpy(eeprom_config->mqtt_base_topic, request->arg(F("mqtt_base_topic")).c_str(), sizeof(eeprom_config->mqtt_base_topic) - 1);
    eeprom_config->mqtt_base_topic[sizeof(eeprom_config->mqtt_base_topic) - 1] = '\0'; // Make sure the buffer is null-terminated

    decltype(eeprom_config_t::sleep_time) sleep_time = request->arg(F("sleep_time")).toInt();
    decltype(eeprom_config_t::watering_delay) watering_delay = request->arg(F("watering_delay")).toInt();
    decltype(eeprom_config_t::watering_threshold) watering_threshold = request->arg(F("watering_threshold")).toInt();
    decltype(eeprom_config_t::watering_time) watering_time = request->arg(F("watering_time")).toInt();
    decltype(eeprom_config_t::automatic_ota) automatic_ota = request->hasArg(F("automatic_ota")); // The argument is only present when the checkbox is checked

    bool changed = eeprom_config->sleep_time != sleep_time ||
      eeprom_config->watering_delay != watering_delay ||
      eeprom_config->watering_threshold != watering_threshold ||
      eeprom_config->watering_time != watering_time ||
      eeprom_config->automatic_ota != automatic_ota;

    // Check if the values has changed
    if (changed) {
      eeprom_config->override_retained_config_topic = true; // Make sure the config topic gets overriden on the next boot
      eeprom_config->sleep_time = sleep_time;
      eeprom_config->watering_delay = watering_delay;
      eeprom_config->watering_threshold = watering_threshold;
      eeprom_config->watering_time = watering_time;
      eeprom_config->automatic_ota = automatic_ota;
    }
    // The values where succesfully configured
    eeprom_config->magic_number = MAGIC_NUMBER;

    // Close the connection
    AsyncWebServerResponse *response = request->beginResponse(200, F("text/plain"), F("OK"));
    response->addHeader(F("Connection"), F("close"));
    response->addHeader(F("Access-Control-Allow-Origin"), F("*"));
    request->send(response);

    *p_run_hotspot = false; // Stop the hotspot when the user submits new values
  });

  httpServer.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, F("text/plain"), F("404: Not Found"));
  });

  // Start the async HTTP server
  httpServer.begin();

  Serial.println(F("HTTP server started"));

  // Indicate to the user that the access point is on
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Wait for the values to be configured
  while (*p_run_hotspot || eeprom_config->magic_number != MAGIC_NUMBER) {
    elegantUpdater.loop(); // This will restart the ESP if a new binary is uploaded
    MDNS.update();
    yield();
  }

  printEEPROMConfig(eeprom_config);

  // Replace values in RAM with the modified values
  // This does NOT make any changes to flash, all data is still in RAM
  EEPROM.put(0, *eeprom_config);

  // Stop the EEPROM emulation and transfer all the data that might have been update from RAM to flash
  EEPROM.end();

  // Give it some time to send the response
  delay(1000);

  Serial.println(F("Turning off hotspot. Rebooting..."));
  httpServer.end();
  MDNS.end();
  WiFi.softAPdisconnect(true);
  ESP.reset();
}

static void printEEPROMConfig(const eeprom_config_t *eeprom_config) {
    // The passwords and ThingSpeak API key are not printed for security reasons
  Serial.printf("WiFi SSID: %s\n", eeprom_config->wifi_ssid);
  Serial.printf("MQTT host: %s, port: %u, username: %s, base topic: %s\n",
    eeprom_config->mqtt_host, eeprom_config->mqtt_port,
    eeprom_config->mqtt_username, eeprom_config->mqtt_base_topic);
  Serial.printf("Sleep time: %u, watering delay: %u, watering threshold: %u, watering time: %u, automatic ota: %u\n",
    eeprom_config->sleep_time, eeprom_config->watering_delay,
    eeprom_config->watering_threshold, eeprom_config->watering_time,
    eeprom_config->automatic_ota);
}

void setup() {
  // Detect if we should go into AP mode by turning on the pull-up resistor on the TX pin
  // then set the RX pin low and read the TX pin
  pinMode(TX_PIN, INPUT_PULLUP);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(RX_PIN, LOW);

  // If the pin is now low, then it means that RX and TX are shorted together
  bool run_hotspot = !digitalRead(TX_PIN);

  // Save power
  pinMode(TX_PIN, INPUT);
  pinMode(RX_PIN, INPUT);

  // It is okay to turn on the serial interface even if the pins are shorted,
  // as it will simply just transmit the values directly to itself
  Serial.begin(115200);
  Serial.println(F("\nBooting"));
  Serial.printf("Firmware version: %s\n", SW_VERSION);
  Serial.flush();

  // Make sure Wifi is off
  // This is needed before trying to turn on the access point, as the ESP8266 can not have both on at the same time
  // See: https://github.com/esp8266/Arduino/issues/1094
  WiFi.mode(WIFI_OFF);

  // Use x bytes of ESP8266 flash for "EEPROM" emulation
  // This loads x bytes from the flash into a array stored in RAM
  EEPROM.begin(sizeof(eeprom_config_t));

  // Read the configuration from EEPROM and check if it is valid
  eeprom_config_t eeprom_config;
  EEPROM.get(0, eeprom_config);

  if (run_hotspot || eeprom_config.magic_number != MAGIC_NUMBER) {
    if (eeprom_config.magic_number != MAGIC_NUMBER)
      Serial.println(F("Starting hotspot, as the device has not been configured"));
    startAsyncHotspot(&run_hotspot, &eeprom_config); // This will block and restart the ESP
  } else
    Serial.println(F("Read configuration values from EEPROM"));

  printEEPROMConfig(&eeprom_config);

  // Handle long sleep
  sleep_data_t sleep_data;
  handleLongSleep(&sleep_data);

  // Read battery voltage
  float voltage = (float)analogRead(A0) * 4.1f;
  Serial.print(F("Battery voltage: ")); Serial.println(voltage);

  blinkLED();

  // Read soil moisture
  int soil_measurements[N_SOIL_MEAS];
  if (!readSoil(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]))) {
    Serial.println(F("Timeout reading the soil measurement! Rebooting..."));
    delay(5000);
    ESP.restart();
  }
  Serial.println(F("Read soil moisture"));
  int soil_moisture = getMean(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  Serial.print(F("Successfully found soil moisture: ")); Serial.println(soil_moisture);

  // Water plant
  if (soil_moisture <= eeprom_config.watering_threshold && sleep_data.watering_delay_cycles <= 1) {
    Serial.println(F("Watering plant!!"));
    pinMode(WATERING_OUT, OUTPUT);
    analogWrite(WATERING_OUT, 1023U * 3U / 4U);
    delay(1000U * eeprom_config.watering_time);
    digitalWrite(WATERING_OUT, LOW);
    pinMode(WATERING_OUT, INPUT); // Save power
    Serial.println(F("Done watering plant"));
    sleep_data.watering_delay_cycles = eeprom_config.watering_delay / eeprom_config.sleep_time; // This will be written to the RTC memory futher down
  } else if (sleep_data.watering_delay_cycles > 1)
    sleep_data.watering_delay_cycles--;

  // Connect to Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(eeprom_config.wifi_ssid, eeprom_config.wifi_password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(F("Connection Failed! Rebooting..."));
    delay(5000);
    ESP.restart();
  }
  Serial.println(F("Wifi connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  if (strlen(eeprom_config.mqtt_base_topic) > 0) {
    // Topic used to configuring the device over MQTT
    const String config_topic = String(F("plant/")) + eeprom_config.mqtt_base_topic + F("/config");

    // Connect to the MQTT broker
    mqttClient.onConnect([](bool sessionPresent) {
      Serial.println(F("Connected to MQTT"));
      //Serial.printf("Session present: %d\n", sessionPresent);
    });
    mqttClient.onDisconnect([](AsyncMqttClientDisconnectReason reason) {
      Serial.printf("Disconnected from MQTT: %d\n", reason);
    });
    mqttClient.onSubscribe([](uint16_t packetId, uint8_t qos) {
      Serial.printf("Subscribe acknowledged for packetId: %u, QoS: %u\n", packetId, qos);
    });
    mqttClient.onUnsubscribe([](uint16_t packetId) {
      Serial.printf("Unsubscribe acknowledged for packetId: %u\n", packetId);
    });
    mqttClient.onMessage([&config_topic, &eeprom_config](char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
      if (config_topic == topic) {
        StaticJsonDocument<JSON_OBJECT_SIZE(5)> jsonDoc; // Create a document with room for the five objects
        DeserializationError error = deserializeJson(jsonDoc, payload);
        if (error) { // Test if parsing succeeds
          Serial.print(F("deserializeJson() failed: ")); Serial.println(error.c_str());
          serializeJson(jsonDoc, Serial);
          Serial.println();
          return;
        }

        JsonVariant sleep_time_variant = jsonDoc[F("sleep_time")];
        JsonVariant watering_delay_variant = jsonDoc[F("watering_delay")];
        JsonVariant watering_threshold_variant = jsonDoc[F("watering_threshold")];
        JsonVariant watering_time_variant = jsonDoc[F("watering_time")];
        JsonVariant automatic_ota_variant = jsonDoc[F("automatic_ota")];

        if (sleep_time_variant.isNull() || watering_delay_variant.isNull() ||
          watering_threshold_variant.isNull() || watering_time_variant.isNull() ||
          automatic_ota_variant.isNull()) {
            Serial.print(F("The JSON payload does not contain all the keys: ")); serializeJson(jsonDoc, Serial);
            Serial.println();
            return;
        }

        // Read the values
        auto sleep_time = sleep_time_variant.as<decltype(eeprom_config_t::sleep_time)>();
        auto watering_delay = watering_delay_variant.as<decltype(eeprom_config_t::watering_delay)>();
        auto watering_threshold = watering_threshold_variant.as<decltype(eeprom_config_t::watering_threshold)>();
        auto watering_time = watering_time_variant.as<decltype(eeprom_config_t::watering_time)>();
        auto automatic_ota = automatic_ota_variant.as<decltype(eeprom_config_t::automatic_ota)>();

        if (eeprom_config.magic_number == MAGIC_NUMBER) {
          bool changed = eeprom_config.sleep_time != sleep_time ||
            eeprom_config.watering_delay != watering_delay ||
            eeprom_config.watering_threshold != watering_threshold ||
            eeprom_config.watering_time != watering_time ||
            eeprom_config.automatic_ota != automatic_ota;

          // TODO: Take a mutex
          if (changed) {
            eeprom_config.sleep_time = sleep_time;
            eeprom_config.watering_delay = watering_delay;
            eeprom_config.watering_threshold = watering_threshold;
            eeprom_config.watering_time = watering_time;
            eeprom_config.automatic_ota = automatic_ota;

            Serial.println(F("Received new MQTT config"));
            printEEPROMConfig(&eeprom_config);

            // Replace values in RAM with the modified values
            // This does NOT make any changes to flash, all data is still in RAM
            EEPROM.put(0, eeprom_config);
          }
        } else
          Serial.println(F("Ignoring new MQTT config, as EEPROM has not been configured"));
      } else {
        Serial.print(F("Unknown MQTT message received - topic: "));
        Serial.printf("%s, QoS: %u, dup: %d, retain: %d, length: %u, index: %u, total: %u\n",
          topic, properties.qos, properties.dup, properties.retain, len, index, total);
      }
    });
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(eeprom_config.mqtt_host, eeprom_config.mqtt_port);
    mqttClient.setCredentials(eeprom_config.mqtt_username, eeprom_config.mqtt_password);
    mqttClient.connect();

    int timeout = 5 * 10; // 5 seconds
    while (!mqttClient.connected() && (timeout-- > 0))
      delay(100);

    // Publish the data via MQTT
    if (mqttClient.connected()) {
      DynamicJsonDocument jsonDoc(1024); // Create a JSON document - you need to allocate more memory if you change the code below
      char jsonBuffer[350]; // Buffer used for storing the payload

      // Capitilize the first character
      String first_char = String(eeprom_config.mqtt_base_topic).substring(0, 1);
      first_char.toUpperCase();
      String name = first_char + String(eeprom_config.mqtt_base_topic).substring(1);

      // Used for the unique device identifier(s)
#ifdef ESP8266
      uint32_t chip_id = ESP.getChipId();
#else
      uint64_t chip_id = ESP.getEfuseMac();
#endif

      // Override the retained config topic, so we do not use the old values after they have been changed via the web interface
      if (eeprom_config.override_retained_config_topic) {
        jsonDoc.clear(); // Make sure we start with a blank document
        jsonDoc[F("sleep_time")] = eeprom_config.sleep_time;
        jsonDoc[F("watering_delay")] = eeprom_config.watering_delay;
        jsonDoc[F("watering_threshold")] = eeprom_config.watering_threshold;
        jsonDoc[F("watering_time")] = eeprom_config.watering_time;
        jsonDoc[F("automatic_ota")] = eeprom_config.automatic_ota;

        size_t n = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
        if (mqttPublishBlocking(config_topic, jsonBuffer, n, true, 5 * 10)) {
          eeprom_config.override_retained_config_topic = false; // Will be writen to the EEPROM further down
          Serial.printf("Successfully overrode MQTT config topic: %s, length: %u\n", jsonBuffer, n);
        } else
          Serial.println(F("Failed to override MQTT config topic"));
      } else {
        // Subscribe to the config topic, so the user can set the value via MQTT
        uint16_t packetIdSub = mqttClient.subscribe(config_topic.c_str(), 0);
        Serial.printf("Subscribed to topic \"%s\", QoS 0, packetId: %u\n", config_topic.c_str(), packetIdSub);
      }

      // Send messsages, so the sensor is auto discovered by Home Assistant - see: https://www.home-assistant.io/docs/mqtt/discovery/
      jsonDoc.clear(); // Make sure we start with a blank document
      jsonDoc[F("name")] = name + F(" Soil Moisture");
      jsonDoc[F("~")] = String(F("plant/")) + eeprom_config.mqtt_base_topic;
      jsonDoc[F("stat_t")] = F("~/state");
      jsonDoc[F("json_attr_t")] = F("~/state");
      jsonDoc[F("val_tpl")] = F("{{value_json.soil_moisture}}");
      jsonDoc[F("unit_of_meas")] = F("clk");
      jsonDoc[F("ic")] = F("mdi:sprout");
      jsonDoc[F("frc_upd")] = true; // Make sure that the sensor value is always stored and not just when it changes
      jsonDoc[F("uniq_id")] = String(chip_id) + F("_soil_moisture");

      // Set device information used for the device registry
      jsonDoc[F("device")][F("name")] = name + F(" Plant");
      jsonDoc[F("device")][F("sw")] = SW_VERSION;
      jsonDoc[F("device")].createNestedArray(F("ids")).add(String(chip_id));

      size_t n = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
      if (mqttPublishBlocking(String(F("homeassistant/sensor/")) + String(eeprom_config.mqtt_base_topic) + F("S/config"), jsonBuffer, n, true, 5 * 10))
        Serial.printf("Successfully sent MQTT message: %s, length: %u\n", jsonBuffer, n);
      else
        Serial.println(F("Failed to send soil moisture discovery message due to timeout"));

      // Send the voltage "sensor" as well
      jsonDoc.clear(); // Make sure we start with a blank document
      jsonDoc[F("name")] = name + F(" Voltage");
      jsonDoc[F("~")] = String(F("plant/")) + eeprom_config.mqtt_base_topic;
      jsonDoc[F("stat_t")] = F("~/state");
      jsonDoc[F("json_attr_t")] = F("~/state");
      jsonDoc[F("val_tpl")] = F("{{value_json.voltage}}");
      jsonDoc[F("unit_of_meas")] = F("V");
      jsonDoc[F("ic")] = F("mdi:solar-panel-large");
      jsonDoc[F("frc_upd")] = true; // Make sure that the sensor value is always stored and not just when it changes
      jsonDoc[F("uniq_id")] = String(chip_id) + F("_voltage");

      // Set device information used for the device registry
      jsonDoc[F("device")][F("name")] = name + F(" Plant");
      jsonDoc[F("device")][F("sw")] = SW_VERSION;
      jsonDoc[F("device")].createNestedArray(F("ids")).add(String(chip_id));

      n = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
      if (mqttPublishBlocking(String(F("homeassistant/sensor/")) + String(eeprom_config.mqtt_base_topic) + F("V/config"), jsonBuffer, n, true, 5 * 10))
        Serial.printf("Successfully sent MQTT message: %s, length: %u\n", jsonBuffer, n);
      else
        Serial.println(F("Failed to send voltage discovery message due to timeout"));

      jsonDoc.clear(); // Make sure we start with a blank document
      jsonDoc[F("soil_moisture")] = soil_moisture;
      jsonDoc[F("voltage")] = voltage / 1000.0f;
      jsonDoc[F("sleep_time")] = eeprom_config.sleep_time;
      jsonDoc[F("watering_delay")] = eeprom_config.watering_delay;
      jsonDoc[F("watering_threshold")] = eeprom_config.watering_threshold;
      jsonDoc[F("watering_time")] = eeprom_config.watering_time;
      jsonDoc[F("automatic_ota")] = eeprom_config.automatic_ota;
      jsonDoc[F("sleep_num")] = sleep_data.sleep_num;
      jsonDoc[F("watering_delay_cycles")] = sleep_data.watering_delay_cycles;
      jsonDoc[F("firmware_update_counter")] = sleep_data.firmware_update_counter;
      jsonDoc[F("version")] = SW_VERSION;

      n = serializeJson(jsonDoc, jsonBuffer, sizeof(jsonBuffer));
      if (mqttPublishBlocking(String(F("plant/")) + String(eeprom_config.mqtt_base_topic) + F("/state"), jsonBuffer, n, false, 5 * 10))
        Serial.printf("Successfully sent MQTT message: %s, length: %u\n", jsonBuffer, n);
      else
        Serial.println(F("Failed to send MQTT message due to timeout"));
    } else
      Serial.println(F("Failed to connect to MQTT broker"));

    // Give it some time to receive a new configuration
    delay(1000); // TODO: Is this needed?
  }

#ifdef POST_TO_THINGSPEAK
  if (strlen(eeprom_config.thingspeak_api_key) > 0) {
    // Post to ThingSpeak
    if (client.connect(thingspeak_server, 80)) {
      client.print(String("GET ") + thingspeak_resource + eeprom_config.thingspeak_api_key +
          "&field1=" + soil_moisture + "&field2=" + voltage + "&field3=" + 0.0 +
                  " HTTP/1.1\r\n" + "Host: " + thingspeak_server + "\r\n" + "Connection: close\r\n\r\n");

      int timeout = 5 * 10; // 5 seconds
      while(!client.available() && (timeout-- > 0))
        delay(100);

      while(client.available()){
        Serial.write(client.read());
      }
      Serial.println();
      Serial.println(F("Successfully posted to Thingspeak!\n"));
    }else{
      Serial.println(F("Problem posting data to Thingspeak."));
      //delay(5000);
      //ESP.restart();
    }
    client.stop();
  }
#endif

  if (eeprom_config.automatic_ota && sleep_data.firmware_update_counter >= 24U * 60U / SLEEP_INTERVAL) { // Only check for updates every 24 hrs
    sleep_data.firmware_update_counter = 0;

    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off.
    ESPhttpUpdate.setLedPin(LED_PIN, LOW);

    // Lambda function for printing the returned HTTP update return value
    auto http_update_status = [](t_httpUpdate_return ret) {
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
          break;
        case HTTP_UPDATE_OK:
          Serial.println(F("HTTP_UPDATE_OK"));
          break;
      }
    };

    // The source code for the update server is available at:https://github.com/Lauszus/PlantFriendsApp
    String url = F("https://plant.lauszus.com/update");

    Serial.println(F("Checking for SPIFFS update"));
    ESPhttpUpdate.rebootOnUpdate(false); // Do not reboot after flashing the SPIFFS, as we need to update the firmware as well
    t_httpUpdate_return ret = ESPhttpUpdate.updateSpiffs(client, url, SW_VERSION);
    if (ret == HTTP_UPDATE_OK) {
      Serial.println(F("Updating firmware..."));
      ESPhttpUpdate.rebootOnUpdate(true); // Reboot after we are done
      ret = ESPhttpUpdate.update(client, url, SW_VERSION);
      http_update_status(ret);
    } else
      http_update_status(ret);
  }

  // Replace values in RAM with the modified values
  // This does NOT make any changes to flash, all data is still in RAM
  EEPROM.put(0, eeprom_config);

  // Stop the EEPROM emulation and transfer all the data that might have been update from RAM to flash
  EEPROM.end();

  Serial.print(F("Going into deep sleep for ")); Serial.print(eeprom_config.sleep_time); Serial.println(F(" min"));

  // Go into long deep sleep
  longSleep(&eeprom_config, &sleep_data);
}

void loop() {
  Serial.println(F("Somewhere we took a wrong turn and ended up in the main loop..."));
  ESP.restart();
}
