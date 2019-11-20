#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>

#include "secret.h"

// Define the access point SSID and password
#define WIFI_AP_SSID "PlantWateringESP8266"
#define WIFI_AP_PASSWORD "plantsarecool"

// Sleep time in minutes
#define SLEEP_TIME 10
// Sleep interval can safely be set from 1 to 30 minutes (possibly higher, but there is a limitation to sleep time with the esp8266). Should be smaller or equal to SLEEP_TIME
#define SLEEP_INTERVAL 10
// Calculate sleep interval and number of sleeps
#define SLEEP_INTERVAL_US SLEEP_INTERVAL*60UL*1000000UL
#define SLEEP_NUM SLEEP_TIME/SLEEP_INTERVAL
// RTC memory address to use for sleep counter
#define SLEEP_DATA_ADDR 256
#define SLEEP_ID 0x2353
// Minimum time between watering plant (minutes) and watering delay
#define MIN_WATERING_TIME 60
#define WATERING_DELAY MIN_WATERING_TIME/SLEEP_TIME

// Define whether to post to Thingspeak
#define POST_TO_THINGSPEAK

// Define soil output and measurement pins
#define SOIL_OUT 14
#define SOIL_IN 12
// Define number of soil measurements
#define N_SOIL_MEAS 4

// Define watering threshold and time
#define WATERING_THRESHOLD 3200
#define WATERING_TIME 3
// Define watering pin
#define WATERING_OUT 13

// Define LED pin for built in LED (ESP-12 board)
#define LED_PIN 2

// The EEPROM is used to store the configuration values between updates
static const uint64_t EEPROM_MAGIC_NUMBER = 0x3b04cd9e94521f9a;

typedef struct {
    uint64_t magic_number; // Used to determine if the EEPROM have been configured or not
    char wifi_ssid[33]; // SSID should be a maximum of 32 characters according to the specs
    char wifi_password[33]; // Restrict password length to 32 characters
    char thingspeak_api_key[17]; // The API is 16 characters
    uint32_t mqtt_host; // We only support IPv4 for now
    uint16_t mqtt_port;
    char mqtt_username[33]; // Just set these to 32 as well
    char mqtt_password[33];
    char mqtt_base_topic[33];
} eeprom_config_t;

// Wifi
WiFiClient client;
//#define WIFI_SSID "YourWifiSSID" // Defined in secret.h
//#define WIFI_PASSWORD "YourWifiPassword" // Defined in secret.h

// MQTT
AsyncMqttClient mqttClient;
//#define MQTT_HOST IPAddress(192, 168, 1, 10) // Defined in secret.h
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
static DNSServer dnsServer;

// UART pins - if these are shorted at boot, then it will turn on the access point
#define TX_PIN  (1U)
#define RX_PIN  (3U)

volatile unsigned long soil_timer;

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

long getMean(long *array, uint8_t n){
  long sum = 0;
  for(uint8_t i = 0; i<n; i++)
    sum += array[i];
  return sum/n;
}

long getMeanWithoutMinMax(long *array, uint8_t n){
  long sum = 0;
  long min = 1000000;
  long max = 0;
  for(uint8_t i = 0; i<n; i++){
    if(min > array[i])
      min = array[i];
    if(max < array[i])
      max = array[i];
  }
  uint8_t n_meas = 0;
  for(uint8_t i = 0; i<n; i++){
    if(array[i] != min && array[i] != max){
      sum += array[i];
      n_meas++;
    }
  }
  return sum/n_meas;
}

void longSleep(){
  uint8_t sleepnum = SLEEP_NUM;
  Serial.print("Number of sleep intervals: "); Serial.println(sleepnum);
  uint32_t sleep_data = (SLEEP_ID << 16) + (0x00FF & sleepnum);
  ESP.rtcUserMemoryWrite(SLEEP_DATA_ADDR, &sleep_data, sizeof(sleep_data));
  ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
}

void handleLongSleep(){
  // Check if we should wake up
  uint8_t sleepnum;
  uint32_t sleep_data;
  // Read data from RTC memory and check if configured for sleeping (set by the two highest bytes)
  ESP.rtcUserMemoryRead(SLEEP_DATA_ADDR, &sleep_data, sizeof(sleep_data));
  if((sleep_data >> 16) == SLEEP_ID)
    sleepnum = (uint8_t)(sleep_data & 0xFF);
  else
    sleepnum = 1;

  if (--sleepnum != 0){
    Serial.print("Going to sleep again. Times left to sleep: "); Serial.println(sleepnum);
    sleep_data = (SLEEP_ID << 16) + (0x00FF & sleepnum);
    ESP.rtcUserMemoryWrite(SLEEP_DATA_ADDR, &sleep_data, sizeof(sleep_data));
    ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
  }
}

uint8_t readWateringDelay(){
  uint32_t watering_data;
  ESP.rtcUserMemoryRead(SLEEP_DATA_ADDR + 1, &watering_data, sizeof(watering_data));
  if((watering_data >> 16) == SLEEP_ID)
    return (0x00FF & watering_data);
  else
    return 1;
}

void writeWateringDelay(uint8_t wateringDelay){
  uint32_t watering_data;
  watering_data = (SLEEP_ID << 16) + (0x00FF & wateringDelay);
  ESP.rtcUserMemoryWrite(SLEEP_DATA_ADDR + 1, &watering_data, sizeof(watering_data));
}

void readSoil(long *soil_measurements, int n_meas){
  pinMode(SOIL_OUT, OUTPUT);
  pinMode(SOIL_IN, INPUT);

  int current_val;

  for(int i=0; i<n_meas; i++){
    Serial.print(i); Serial.print(": ");
    digitalWrite(SOIL_OUT, LOW);
    delay(10);
    attachInterrupt(digitalPinToInterrupt(SOIL_IN), interrupt_routine, CHANGE);
    unsigned long timer_start = ESP.getCycleCount();
    digitalWrite(SOIL_OUT, HIGH);
    delay(1000);
    detachInterrupt(digitalPinToInterrupt(SOIL_IN));
    current_val = (soil_timer-timer_start);
    Serial.println(current_val);

    soil_measurements[i] = current_val;
  }
}

void blinkLED(){
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);
  pinMode(LED_PIN, INPUT);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println(F("Connected to MQTT"));
  //Serial.printf("Session present: %d\n", sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println(F("Disconnected from MQTT"));
}

volatile uint16 publishedPacketId = -1;

void onMqttPublish(uint16_t packetId) {
  publishedPacketId = packetId;
  //Serial.printf("Publish acknowledged - packet ID: %u\n", packetId);
}

bool mqttPublishBlocking(String topic, char *buffer, size_t length, int timeout) {
  publishedPacketId = -1;
  uint16_t packetId = mqttClient.publish(topic.c_str(), 2, false, buffer, length);
  while (publishedPacketId != packetId && (timeout-- > 0))
    delay(100);
  return timeout > 0;
}

bool mqttPublishBlocking(String topic, String payload, int timeout) {
  publishedPacketId = -1;
  uint16_t packetId = mqttClient.publish(topic.c_str(), 2, false, payload.c_str());
  while (publishedPacketId != packetId && (timeout-- > 0))
    delay(100);
  return timeout > 0;
}

void printEEPROMConfig(const eeprom_config_t &eeprom_config) {
    // The passwords and ThingSpeak API key are not printed for security reasons
  Serial.printf("WiFi SSID: %s\n", eeprom_config.wifi_ssid);
  Serial.printf("MQTT host: %s, port: %u, username: %s, base topic: %s\n",
    IPAddress(eeprom_config.mqtt_host).toString().c_str(), eeprom_config.mqtt_port,
    eeprom_config.mqtt_username, eeprom_config.mqtt_base_topic);
}

void setup() {
  // Use x bytes of ESP8266 flash for "EEPROM" emulation
  // This loads x bytes from the flash into a array stored in RAM
  EEPROM.begin(sizeof(eeprom_config_t));

  // Read the configuration from EEPROM and check if it is valid
  eeprom_config_t eeprom_config;
  EEPROM.get(0, eeprom_config);

  // Detect if we should go into AP mode by turning on the pull-up resistor on the TX pin
  // then set the RX pin low and read the TX pin
  pinMode(TX_PIN, INPUT_PULLUP);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(RX_PIN, LOW);

  // If the pin is now low, then it means that RX and TX are shorted together
  // Simply reset the magic number, so it goes into AP mode
  if (!digitalRead(TX_PIN))
    eeprom_config.magic_number = 0;

  // It is okay to turn on the serial interface even if the pins are shorted,
  // as it will simply just transmit the values directly to itself
  Serial.begin(115200);
  Serial.println("\nBooting");
  Serial.flush();

  if (eeprom_config.magic_number != EEPROM_MAGIC_NUMBER) {
    // Configure the hotspot
    // Note that we set the maximum number of connection to 1
    int channel = 1, ssid_hidden = 0, max_connection = 1;
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, channel, ssid_hidden, max_connection)) {
      Serial.println(F("Failed to start hotspot! Rebooting..."));
      delay(5000);
      ESP.restart();
    }
    IPAddress ip = WiFi.softAPIP();
    Serial.print(F("AP IP address: ")); Serial.println(ip);

    if (dnsServer.start(53, "*", ip)) // Redirect all requests to the our IP address
      Serial.println(F("DNS server started"));
    else
      Serial.println(F("Failed to start DNS server"));

    if (SPIFFS.begin()) {
      httpServer.on("/pure-min.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Use Pure to style the from: https://purecss.io/forms/
        request->send(SPIFFS, "/pure-min.css", "text/css");
      });
      httpServer.on("/pure-extra-min.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Som additional css to make it look a little nicer
        request->send(SPIFFS, "/pure-extra-min.css", "text/css");
      });
      //httpServer.serveStatic(filename, SPIFFS, filename);
      //httpServer.serveStatic("/fs", SPIFFS, "/"); // Attach filesystem root at URL /fs
    } else
      Serial.println("An Error has occurred while mounting SPIFFS");

    // Show a form on the root page
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncResponseStream *response = request->beginResponseStream(F("text/html"));
      response->addHeader(F("Cache-Control"), F("no-cache,no-store,must-revalidate"));
      response->addHeader(F("Pragma"), F("no-cache"));
      response->addHeader(F("Expires"), F("-1"));

      // Format the HTML response
      response->print(F("<html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0\">"));
      response->print(F("<link rel=\"stylesheet\" type=\"text/css\" href=\"pure-min.css\">"));
      response->print(F("<link rel=\"stylesheet\" type=\"text/css\" href=\"pure-extra-min.css\">"));
      response->print(F("</head><body style=\"margin:50px auto;width:50%\">"));

      response->print(F("<form action=\"/config\" method=\"POST\" class=\"pure-form pure-form-aligned\">"));

      response->print(F("<div class=\"pure-control-group\">"));
      response->print(F("<label></label>")); // Added, so it's the heading gets aligned with the inputs
      response->print(F("<h2>Settings</h2>"));
      response->print(F("</div>"));

      response->print(F("<fieldset class=\"pure-group\">"));
      response->print(F("<div class=\"pure-control-group\">"));
      response->print(F("<label for=\"wifi_ssid\">WiFi SSID</label>"));
      response->print(F("<input type=\"text\" name=\"wifi_ssid\" value=\"")); response->print(WIFI_SSID); response->print(F("\">"));
      response->print(F("</div><div class=\"pure-control-group\">"));
      response->print(F("<label for=\"wifi_password\">WiFi password</label>"));
      response->print(F("<input type=\"password\" name=\"wifi_password\" value=\"")); response->print(WIFI_PASSWORD); response->print(F("\">"));
      response->print(F("</div></fieldset>"));

      response->print(F("<fieldset class=\"pure-group\">"));
      response->print(F("<div class=\"pure-control-group\">"));
      response->print(F("<label for=\"thingspeak_api_key\">ThingSpeak API key</label>"));
      response->print(F("<input type=\"password\" name=\"thingspeak_api_key\" value=\"")); response->print(THINGSPEAK_API_KEY); response->print(F("\">"));
      response->print(F("</div></fieldset>"));

      response->print(F("<fieldset class=\"pure-group\">"));
      response->print(F("<div class=\"pure-control-group\">"));
      response->print(F("<label for=\"mqtt_host\">MQTT host</label>"));
      response->print(F("<input type=\"text\" name=\"mqtt_host\" value=\"")); response->print(MQTT_HOST); response->print(F("\">"));
      response->print(F("</div><div class=\"pure-control-group\">"));
      response->print(F("<label for=\"mqtt_port\">MQTT port</label>"));
      response->print(F("<input type=\"text\" name=\"mqtt_port\" value=\"")); response->print(MQTT_PORT); response->print(F("\">"));
      response->print(F("</div><div class=\"pure-control-group\">"));
      response->print(F("<label for=\"mqtt_username\">MQTT username</label>"));
      response->print(F("<input type=\"text\" name=\"mqtt_username\" value=\"")); response->print(MQTT_USERNAME); response->print(F("\">"));
      response->print(F("</div><div class=\"pure-control-group\">"));
      response->print(F("<label for=\"mqtt_password\">MQTT password</label>"));
      response->print(F("<input type=\"password\" name=\"mqtt_password\" value=\"")); response->print(MQTT_PASSWORD); response->print(F("\">"));
      response->print(F("</div><div class=\"pure-control-group\">"));
      response->print(F("<label for=\"mqtt_base_topic\">MQTT base topic</label>"));
      response->print(F("<input type=\"text\" name=\"mqtt_base_topic\" value=\"")); response->print(MQTT_BASE_TOPIC); response->print(F("\">"));
      response->print(F("</div></fieldset>"));

      response->print(F("<div class=\"pure-controls\">"));
      response->print(F("<button type=\"submit\" class=\"pure-button pure-button-primary\">Submit</button"));
      response->print(F("</div></form></body></html>"));

      request->send(response); // Send the response
    });

    // Handle the post request
    httpServer.on("/config", HTTP_POST, [&eeprom_config](AsyncWebServerRequest *request) {
      if (!request->hasArg("wifi_ssid") || !request->hasArg("wifi_password")
        || !request->hasArg("thingspeak_api_key")
        || !request->hasArg("mqtt_host") || !request->hasArg("mqtt_port")
        || !request->hasArg("mqtt_username") || !request->hasArg("mqtt_password") || !request->hasArg("mqtt_base_topic")) {
        request->send(400, F("text/plain"), F("400: Invalid request"));
        return;
      }

      strncpy(eeprom_config.wifi_ssid, request->arg("wifi_ssid").c_str(), sizeof(eeprom_config.wifi_ssid) - 1);
      eeprom_config.wifi_ssid[sizeof(eeprom_config.wifi_ssid) - 1] = '\0'; // Make sure the buffer is null-terminated

      strncpy(eeprom_config.wifi_password, request->arg("wifi_password").c_str(), sizeof(eeprom_config.wifi_password) - 1);
      eeprom_config.wifi_password[sizeof(eeprom_config.wifi_password) - 1] = '\0'; // Make sure the buffer is null-terminated

      strncpy(eeprom_config.thingspeak_api_key, request->arg("thingspeak_api_key").c_str(), sizeof(eeprom_config.thingspeak_api_key) - 1);
      eeprom_config.thingspeak_api_key[sizeof(eeprom_config.thingspeak_api_key) - 1] = '\0'; // Make sure the buffer is null-terminated

      IPAddress mqtt_host;
      if (!mqtt_host.fromString(request->arg("mqtt_host"))) {
        Serial.println(F("Failed to parse MQTT host IP address"));
        request->send(400, F("text/plain"), F("400: Invalid request"));
        return;
      }
      eeprom_config.mqtt_host = mqtt_host;

      eeprom_config.mqtt_port = request->arg("mqtt_port").toInt();

      strncpy(eeprom_config.mqtt_username, request->arg("mqtt_username").c_str(), sizeof(eeprom_config.mqtt_username) - 1);
      eeprom_config.mqtt_username[sizeof(eeprom_config.mqtt_username) - 1] = '\0'; // Make sure the buffer is null-terminated

      strncpy(eeprom_config.mqtt_password, request->arg("mqtt_password").c_str(), sizeof(eeprom_config.mqtt_password) - 1);
      eeprom_config.mqtt_password[sizeof(eeprom_config.mqtt_password) - 1] = '\0'; // Make sure the buffer is null-terminated

      strncpy(eeprom_config.mqtt_base_topic, request->arg("mqtt_base_topic").c_str(), sizeof(eeprom_config.mqtt_base_topic) - 1);
      eeprom_config.mqtt_base_topic[sizeof(eeprom_config.mqtt_base_topic) - 1] = '\0'; // Make sure the buffer is null-terminated

      // The values where succesfully configured
      eeprom_config.magic_number = EEPROM_MAGIC_NUMBER;

      request->redirect(F("/")); // Redirect to the root
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
    while (eeprom_config.magic_number != EEPROM_MAGIC_NUMBER)
      delay(100);

    printEEPROMConfig(eeprom_config);

    // Replace values in RAM with the modified values
    // This does NOT make any changes to flash, all data is still in RAM
    EEPROM.put(0, eeprom_config);

    // Stop the EEPROM emulation and transfer all the data that might have been update from RAM to flash
    EEPROM.end();

    // Give it some time to send the response
    delay(1000);

    Serial.println(F("Rebooting..."));
    httpServer.end();
    WiFi.softAPdisconnect(true);
    ESP.reset();
  } else
    Serial.println(F("Read configuration values from EEPROM"));

  // Stop the EEPROM emulation
  EEPROM.end();

  printEEPROMConfig(eeprom_config);

  // Make sure Wifi is off
  WiFi.mode(WIFI_OFF);

  // Handle long sleep
  handleLongSleep();

  // Read battery voltage
  float voltage = analogRead(A0) * 4.1;
  Serial.print("Successfully read battery voltage: "); Serial.println(voltage);

  blinkLED();

  // Read soil moisture
  long soil_measurements[N_SOIL_MEAS];
  readSoil(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  Serial.println("Read soil moisture");
  int soil_moisture = getMean(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  Serial.println("Calculated mean soil moisture");
  int soil_moisture_filtered = getMeanWithoutMinMax(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  Serial.print("Successfully found soil moisture: "); Serial.println(soil_moisture);
  Serial.print("Filtered soil moisture: "); Serial.println(soil_moisture_filtered);

  // Water plant
  uint8_t watering_delay = readWateringDelay();
  Serial.print("Watering delay: "); Serial.println(watering_delay);
  if(soil_moisture <= WATERING_THRESHOLD && watering_delay <= 1){
    Serial.println("Watering plant!!");
    pinMode(WATERING_OUT, OUTPUT);
    analogWrite(WATERING_OUT,1023/4*3);
    delay(WATERING_TIME*1000);
    digitalWrite(WATERING_OUT, LOW);
    pinMode(WATERING_OUT, INPUT);
    writeWateringDelay(WATERING_DELAY);
  }else if(watering_delay > 1){
    writeWateringDelay(watering_delay - 1);
  }

  // Connect to Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(eeprom_config.wifi_ssid, eeprom_config.wifi_password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Wifi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (strlen(eeprom_config.mqtt_base_topic) > 0) {
    // Connect to the MQTT broker
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    //mqttClient.onSubscribe(onMqttSubscribe);
    //mqttClient.onUnsubscribe(onMqttUnsubscribe);
    //mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.setServer(eeprom_config.mqtt_host, eeprom_config.mqtt_port);
    mqttClient.setCredentials(eeprom_config.mqtt_username, eeprom_config.mqtt_password);
    mqttClient.connect();

    int timeout = 5 * 10; // 5 seconds
    while (!mqttClient.connected() && (timeout-- > 0))
      delay(100);

    // Publish the data via MQTT
    if (mqttClient.connected()) {
      StaticJsonDocument<JSON_OBJECT_SIZE(2)> jsonDoc; // Create a document with room for the two objects
      jsonDoc["soil_moisture"] = soil_moisture;
      jsonDoc["voltage"] = voltage / 1000.0f;

      char buffer[64];
      size_t n = serializeJson(jsonDoc, buffer);
      if (mqttPublishBlocking(eeprom_config.mqtt_base_topic, buffer, n, 5 * 10))
        Serial.printf("Successfully sent MQTT message: %s, length: %u\n", buffer, n);
      else
        Serial.print(F("Failed to send MQTT message due to timeout"));
    } else
      Serial.println(F("Failed to connect to MQTT broker"));
  }

#ifdef POST_TO_THINGSPEAK
  if (strlen(eeprom_config.thingspeak_api_key) > 0) {
    // Post to ThingSpeak
    if (client.connect(thingspeak_server, 80)) {
      client.print(String("GET ") + thingspeak_resource + eeprom_config.thingspeak_api_key +
          "&field1=" + soil_moisture_filtered + "&field2=" + 0.0 +
          "&field3=" + soil_moisture + "&field4=" + voltage + "&field5=" + 0.0 +
                  " HTTP/1.1\r\n" + "Host: " + thingspeak_server + "\r\n" + "Connection: close\r\n\r\n");

      int timeout = 5 * 10; // 5 seconds
      while(!client.available() && (timeout-- > 0))
        delay(100);

      while(client.available()){
        Serial.write(client.read());
      }
      Serial.println();
      Serial.println("Successfully posted to Thingspeak!\n");
    }else{
      Serial.println("Problem posting data to Thingspeak.");
      //delay(5000);
      //ESP.restart();
    }
    client.stop();
  }
#endif

/*
  #include <ESP8266httpUpdate.h>
  ESPhttpUpdate.setLedPin(LED_PIN);
  ESPhttpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, "192.168.0.115", 80, "/esp/update/arduino.php", "optional current version string here");
  switch(ret) {
      case HTTP_UPDATE_FAILED:
          Serial.printf("[Firmware] HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
      case HTTP_UPDATE_NO_UPDATES:
          Serial.println("[Firmware] HTTP_UPDATE_NO_UPDATES\n");
          break;
      case HTTP_UPDATE_OK:
          Serial.println("[Firmware] HTTP_UPDATE_OK\n");
          break;
  }
*/

  Serial.print("Going into deep sleep for "); Serial.print(SLEEP_TIME); Serial.println(" min");
  // Go into long deep sleep
  longSleep();
}

void loop() {
  Serial.println("Somewhere we took a wrong turn and ended up in the main loop...");
  ESP.restart();
}

void ICACHE_RAM_ATTR interrupt_routine(){
  soil_timer = ESP.getCycleCount();
}
