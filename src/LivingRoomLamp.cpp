#include <Arduino.h>
#include <ArduinoOTA.h>
#include <cstring>
#include <WifiUpdate.h>
#include <Webserver.h>
#include <WebSerial.h>
#include <EEPROM.h>
#include <fauxmoESP.h>

//#define CHRISTMAS_TREE

#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#ifdef ESP32
  #include "WiFi.h"
  #include "AsyncTCP.h"
  #include "ESPAsyncWebServer.h"
  #include "Udp.h"
#endif

#ifdef ESP8266
  #include <WiFiUdp.h>
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
#endif
// Which pin on the Arduino is connected to the NeoPixels?

// How many NeoPixels are attached to the Arduino?
// NeoPixel color format & data rate. See the strandtest example for
// information on possible values.
const int RELAISE1 = 5;
const int RELAISE2 = 4;
const int RELAISE3 = 0;
const int RELAISE4 = 2;
const int RELAISE5 = 14;
const int RELAISE6 = 12;
const int RELAISE7 = 13;
const int lamps[] = {RELAISE1, RELAISE2, RELAISE3, RELAISE4, RELAISE5, RELAISE6, RELAISE7};
WebSerialClass WebSerial;

const int ESP_BUILTIN_LED = 2;
// Rather than declaring the whole NeoPixel object here, we just create
// a pointer for one, which we'll then allocate later...

int delayval = 25; // delay for half a second


fauxmoESP fauxmo;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[200];
char replyPacket[] = "LivingRoomLamp";
IPAddress masterIp;
int lampsOn = 7;
const int NUM_LAMPS = 7;


void setupPins();
void setupUdp();
void receiveUdp();
void setupFauxmo();

void save(int address, int mode) {
  EEPROM.write(address, mode);
  EEPROM.commit();
}

inline void turnLampsOn(int lampsOn) {
  // turn number of lamps on
  for (int i = 0; i < lampsOn; i++)
  {
    digitalWrite(lamps[i], LOW);
  }

  // turn the remaining lamps off
  for (int i = lampsOn; i < NUM_LAMPS; i++)
  {
    digitalWrite(lamps[i], HIGH);
  }

  save(0, lampsOn);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  EEPROM.begin(1);

  // read old mode
  lampsOn = EEPROM.read(0);

  setupPins();
  turnLampsOn(lampsOn);
  setupWifiUpdate();
  setupWebSerial();
  setupUdp();
  setupFauxmo();
}

inline void off() {
  for (int i = 0; i < NUM_LAMPS; i++)
  {
    digitalWrite(lamps[i], HIGH);
  }
  save(0, 0);
}

inline void on() {
  for (int i = 0; i < NUM_LAMPS; i++)
  {
    digitalWrite(lamps[i], LOW);
  }
  save(0, 7);
}

void setupFauxmo() {
  fauxmo.addDevice("Lampe");
  fauxmo.setPort(80); // required for gen3 devices
  fauxmo.enable(true);

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
      Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

      if (std::strcmp(device_name,"Lampe") == 0) {
        if (state) {
  
          float perc = value/255.0*100;
          if (perc>=70) {
            turnLampsOn(7);
          } else 
          if (perc>=60) {
            turnLampsOn(6);
          } else
          if (perc>=50) {
            turnLampsOn(5);
          } else
          if (perc>=40) {
            turnLampsOn(4);
          } else
          if (perc>=30) {
            turnLampsOn(3);
          } else
          if (perc>=20) {
            turnLampsOn(2);
          } else
          if (perc>=10) {
            turnLampsOn(1);
          } 

        } else off();
        return;
      }
  });
}

void setupPins() {
  pinMode(ESP_BUILTIN_LED, OUTPUT);
  for (int i = 0; i < NUM_LAMPS; i++)
  {
    pinMode(lamps[i], OUTPUT);
  }
}

void setupUdp() {
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  // WebSerial.println("Udp configured.");
}

void printRSSI() {
  Serial.printf("Signal strength %d\n", WiFi.RSSI());
}

void loop() {
  ArduinoOTA.handle();
  fauxmo.handle();
  printRSSI();
  receiveUdp();

  delay(100);
}

void receiveUdp() {
  int packetSize = Udp.parsePacket();
  //Serial.println("In Loop");
  //delay(1000);
  if (packetSize)
  {
    // receive incoming UDP packets
    // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 900);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }
    // WebSerial.print("UDP packet contents: ");
    // WebSerial.println(incomingPacket);

    if (std::strcmp(incomingPacket,"Detect") == 0) {
      // send back a reply, to the IP address and port we got the packet from
      masterIp = Udp.remoteIP();
      Udp.beginPacket(masterIp, 4445);
      Udp.write(replyPacket, 14);
      Udp.endPacket();
      // rotatePixels();
      return;
    } 
    if (std::strcmp(incomingPacket,"off") == 0) {
      off();
      return;
     } 

    if (strstr(incomingPacket,"numLamps=") != NULL) {
      lampsOn = incomingPacket[9];
      turnLampsOn(lampsOn);
      return;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}