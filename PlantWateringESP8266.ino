#include <ESP8266WiFi.h>
#include <OneWire.h>
#include "secret.h"

// Using RTC memory for longer sleeps

// TODO:
// Try a watering quarantine of x hours after watering
// Try long sleeps and only wake rf on the last wakeup
// Define for printing stuff to serial
// Other ways to read soil moisture?
// Check power consumption in sleep and awake


// Sleep time and interval in minutes
#define SLEEP_TIME 360
#define SLEEP_INTERVAL 30
// Calculate sleep interval and number of sleeps
#define SLEEP_INTERVAL_US SLEEP_INTERVAL*60UL*1000000UL
#define SLEEP_NUM SLEEP_TIME/SLEEP_INTERVAL
// RTC memory address to use for sleep counter
#define SLEEP_NUM_ADDR 0 

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

void longSleep(){
  uint8_t sleepnum = SLEEP_NUM-1;
  Serial.print("Number of sleep intervals: "); Serial.println(sleepnum);
  ESP.rtcUserMemoryWrite(SLEEP_NUM_ADDR, &sleepnum, sizeof(sleepnum));
  ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
}

void handleLongSleep(){
  // Check if we should wake up
  uint8_t sleepnum;
  ESP.rtcUserMemoryRead(SLEEP_NUM_ADDR, &sleepnum, sizeof(sleepnum));
  if (--sleepnum != 0){
    Serial.print("Going to sleep again. Times left to sleep: "); Serial.println(sleepnum);
    ESP.rtcUserMemoryWrite(SLEEP_NUM_ADDR, &sleepnum, sizeof(sleepnum));
    ESP.deepSleep(SLEEP_INTERVAL_US, WAKE_RF_DEFAULT);
  }
}

int readSoil(int n_meas){
  pinMode(SOIL_OUT, OUTPUT);
  pinMode(SOIL_IN, INPUT);

  int current_val;
  long avg = 0;

  for(int i=0; i<N_SOIL_MEAS; i++){
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

    avg += current_val;
  }
  return (int)(avg/n_meas);
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

  delay(2000); //TODO remove. Used to wat a bit after serial.begin before printing serial voltage
  
  // Handle long sleep
  handleLongSleep();

  // Read battery voltage
  float voltage = analogRead(A0) * 4.1;
  Serial.print("Successfully read battery voltage: "); Serial.println(voltage);
  

  // Read soil moisture
  int soil_moisture = readSoil(N_SOIL_MEAS);
  Serial.print("Successfully read soil moisture: "); Serial.println(soil_moisture);

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
        "&field1=" + 0.0 + "&field2=" + 0.0 +
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
