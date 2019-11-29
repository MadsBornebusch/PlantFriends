/*
 * Based on: https://github.com/ayushsharma82/AsyncElegantOTA
 * Modified by: Kristian Sloth Lauszus (lauszus@gmail.com)
 */

#ifndef __AsyncElegantOTSpiff_h
#define __AsyncElegantOTSpiff_h

#include <ESPAsyncWebServer.h>
#include <elegantWebpage.h>
#include <Updater.h>

#ifndef U_SPIFFS
// Needed for backwards compatibility: https://github.com/esp8266/Arduino/commit/a389a995fb12459819e33970ec80695f1eaecc58
#define U_SPIFFS U_FS
#endif

class AsyncElegantOtaSpiffs {
public:
    AsyncElegantOtaSpiffs(int ledPin = -1, uint8_t ledOn = LOW) : ledPin(ledPin), ledOn(ledOn) {};

    void begin(AsyncWebServer *server) {
        server->on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response = request->beginResponse_P(200, F("text/html"), ELEGANT_HTML, ELEGANT_HTML_SIZE);
            response->addHeader(F("Content-Encoding"), F("gzip"));
            request->send(response);
        });

        server->on("/update", HTTP_POST, [this](AsyncWebServerRequest *request) {
            // the request handler is triggered after the upload has finished...
            // create the response, add header, and send response
            AsyncWebServerResponse *response = request->beginResponse(Update.hasError() ? 500 : 200, F("text/plain"), Update.hasError() ? F("FAIL") : F("OK"));
            response->addHeader(F("Connection"), F("close"));
            response->addHeader(F("Access-Control-Allow-Origin"), F("*"));
            request->send(response);
            restartRequired = true;
        }, [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            // Upload handler chunks in data
            if (!index) {
                size_t content_len = request->contentLength();
                int cmd = filename.indexOf(F("spiffs")) > -1 ? U_SPIFFS : U_FLASH;
#if defined(ESP8266)
                Update.runAsync(true);
#endif
                if (!Update.begin(content_len, cmd, ledPin, ledOn))
                    Update.printError(Serial);
            }

            // Write chunked data to the free sketch space
            if (Update.write(data, len) != len)
                Update.printError(Serial);

            if (final) { // If the final flag is set then this is the last frame of data
                if (!Update.end(true)) // True to set the size to the current progress
                    Update.printError(Serial);
            }
        });
    }

    void loop() {
        if (restartRequired) {
            delay(1000);
#if defined(ESP8266)
            ESP.restart();
#elif defined(ESP32)
            esp_task_wdt_init(1, true);
            esp_task_wdt_add(NULL);
            while (true)
                ;
#endif
        }
    }

private:
    bool restartRequired = false;
    int ledPin;
    uint8_t ledOn;
};

#endif // __AsyncElegantOTSpiff_h
