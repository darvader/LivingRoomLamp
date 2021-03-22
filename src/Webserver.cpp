#ifdef ESP8266
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif
#ifdef ESP32
      #define HARDWARE "ESP32"
    #include "ESPAsyncWebServer.h"
    #include "AsyncTCP.h"
#endif
#include <WebSerial.h>
#include <Webserver.h>

AsyncWebServer server(8080);

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(size_t i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void setupWebSerial() {
    // WebSerial is accessible at "<IP Address>/webserial" in browser
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
    server.begin();
}

