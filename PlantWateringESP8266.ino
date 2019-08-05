#include <ESP8266WiFi.h>
#include <OneWire.h>
#include "secret.h"

// Using RTC memory for longer sleeps

// TODO:
// Try using the median soil moisture measurement
// Try a watering quarantine of x hours after watering (wake/sleep mode with a data type. Byte 2 of sleep data: 0xFF00)
// Try long sleeps and only wake rf on the last wakeup
// Define for printing stuff to serial
// Other ways to read soil moisture?
// Check power consumption in sleep and awake


// Sleep time in minutes
#define SLEEP_TIME 60
// Sleep interval can safely be set from 1 to 30 minutes (possibly higher, but there is a limitation to sleep time with the esp8266)
#define SLEEP_INTERVAL 30
// Calculate sleep interval and number of sleeps
#define SLEEP_INTERVAL_US SLEEP_INTERVAL*60UL*1000000UL
#define SLEEP_NUM SLEEP_TIME/SLEEP_INTERVAL
// RTC memory address to use for sleep counter
#define SLEEP_DATA_ADDR 0 
#define SLEEP_ID 0x2353

// Define whether to post to Thingspeak
#define POST_TO_THINGSPEAK

// Define soil output and measurement pins
#define SOIL_OUT 14
#define SOIL_IN 12
// Define number of soil measurements
#define N_SOIL_MEAS 4

// Define watering threshold and time
#define WATERING_THRESHOLD 2500
#define WATERING_TIME 3
// Define watering pin
#define WATERING_OUT 13

// Define LED pin for built in LED (ESP-12 board)
#define LED_PIN 2

// Wifi
WiFiClient client;
//const char* ssid = "YourWifiSSID"; // Defined in secret.h
//const char* password = "YourWifiPassword"; // Defined in secret.h

// ThingSpeak variables
// const char* writeAPIKey = "YourWriteApiKey"; // Defined in secret.h
const char* server = "api.thingspeak.com";
const char* resource = "/update?api_key=";

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


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  // Make sure Wifi is off
  WiFi.mode(WIFI_OFF);

  delay(2000); //TODO remove. Used to wait a bit after serial is initialised before printing voltage
  
  // Handle long sleep
  handleLongSleep();

  // Read battery voltage
  float voltage = analogRead(A0) * 4.1;
  Serial.print("Successfully read battery voltage: "); Serial.println(voltage);
  

  // Read soil moisture
  long soil_measurements[N_SOIL_MEAS];
  readSoil(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  int soil_moisture = getMean(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  int soil_moisture_filtered = getMeanWithoutMinMax(soil_measurements, sizeof(soil_measurements)/sizeof(soil_measurements[0]));
  Serial.println("Successfully read soil moisture: "); Serial.println(soil_moisture);
  Serial.println("Filtered soil moisture: "); Serial.println(soil_moisture_filtered);

  blinkLED();

  // Water plant
  if(soil_moisture <= WATERING_THRESHOLD){
    Serial.println("Watering plant!!");
    pinMode(WATERING_OUT, OUTPUT);
    analogWrite(WATERING_OUT,1023/4*3);
    delay(WATERING_TIME*1000);
    digitalWrite(WATERING_OUT, LOW);
    pinMode(WATERING_OUT, INPUT);
  }


  // Connect to Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Wifi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  #ifdef POST_TO_THINGSPEAK 
  // Post to Thingspeak
  if (client.connect(server,80)) {
    
    client.print(String("GET ") + resource + writeAPIKey + 
        "&field1=" + soil_moisture_filtered + "&field2=" + 0.0 +
        "&field3=" + soil_moisture + "&field4=" + voltage + "&field5=" + 0.0 + 
                " HTTP/1.1\r\n" + "Host: " + server + "\r\n" + "Connection: close\r\n\r\n");
  
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
  #endif

  Serial.print("Going into deep sleep for "); Serial.print(SLEEP_TIME); Serial.println(" min");
  // Go into long deep sleep
  longSleep();

}


void loop() {
  Serial.println("Somewhere we took a wrong turn and ended up in the main loop...");
  ESP.restart();
}

void interrupt_routine(){
  soil_timer = ESP.getCycleCount();
}
