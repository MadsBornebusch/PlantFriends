/*
 * Based on: https://github.com/ayushsharma82/AsyncElegantOTA
 * Modified by: Kristian Sloth Lauszus (lauszus@gmail.com)
 */

#ifndef __AsyncElegantOTSpiff_h
#define __AsyncElegantOTSpiff_h

#include <ESPAsyncWebServer.h>
#include <elegantWebpage.h>

#ifndef U_SPIFFS
// Needed for backwards compatibility: https://github.com/esp8266/Arduino/commit/a389a995fb12459819e33970ec80695f1eaecc58
#define U_SPIFFS U_FS
#endif

class AsyncElegantOtaSpiffsClass {
public:
    void begin(AsyncWebServer *server) {
        server->on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
            AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", ELEGANT_HTML, ELEGANT_HTML_SIZE);
            response->addHeader("Content-Encoding", "gzip");
            request->send(response);
        });

        server->on("/update", HTTP_POST, [this](AsyncWebServerRequest *request) {
            // the request handler is triggered after the upload has finished...
            // create the response, add header, and send response
            AsyncWebServerResponse *response = request->beginResponse(Update.hasError() ? 500 : 200, "text/plain", Update.hasError() ? "FAIL" : "OK");
            response->addHeader("Connection", "close");
            response->addHeader("Access-Control-Allow-Origin", "*");
            request->send(response);
            restartRequired = true;
        }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            // Upload handler chunks in data
            if (!index) {
                size_t content_len = request->contentLength();
                int cmd = filename.indexOf("spiffs") > -1 ? U_SPIFFS : U_FLASH;
#if defined(ESP8266)
                Update.runAsync(true);
#endif
                if (!Update.begin(content_len, cmd))
                    Update.printError(Serial);
            }

            // Write chunked data to the free sketch space
            if (Update.write(data, len) != len)
                Update.printError(Serial);

            if (final) { // if the final flag is set then this is the last frame of data
                if (Update.end(true)) { // true to set the size to the current progress
                }
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
};

AsyncElegantOtaSpiffsClass AsyncElegantOtaSpiffs;

#endif // __AsyncElegantOTSpiff_h
