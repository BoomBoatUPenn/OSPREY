
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#define MAX_VAL 32769.0

char packetBuffer[255]; 

const char* ssid     = "Room 309";
const char* password = "Tpi4uaouty";

WiFiUDP UdpCommand;
WiFiUDP UdpTelem;

unsigned int UDPPortCommands = 5005;
unsigned int UDPPortTelem = 5006;

IPAddress ipTarget(192, 168, 1, 255);
IPAddress ipLocal(192, 168, 1, 6);

#define ESC_PIN 18
#define RUDDER_SERVO_PIN 5

const int ESCChannel = 4;
const int RudderServoChannel = 5;

const int resolution = 13;
const int freq = 50;

long heartbeat = 0;
int BoatNum = 0;

void setup() {                  
  Serial.begin(115200);  
 
  WiFi.softAP("Boat 1");
  WiFi.softAPConfig(IPAddress(192, 168, 1, 6),  IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

  UdpCommand.begin(UDPPortCommands);
  UdpTelem.begin(UDPPortTelem);

  ledcAttachPin(ESC_PIN, ESCChannel);
  ledcAttachPin(RUDDER_SERVO_PIN, RudderServoChannel);
/**
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.println("no wifi");
  }
 **/ 
}

void recieve_commands(){
  int packetSize = UdpCommand.parsePacket();

  if (packetSize) {
    Serial.println(packetBuffer);

    int len = UdpCommand.read(packetBuffer, 255);

    if (len > 0) {
      packetBuffer[len] = 0;
    }

    if (packetBuffer[0] == 'c'){
      char *token;
      Serial.println(packetBuffer);
      // discard 'c' character
    }

    if (packetBuffer[0] == 's'){
      float val = atof(packetBuffer+1);
      Serial.println(val);
    }

    if (packetBuffer[0] == 'r'){
      float val = atof(packetBuffer+1);
    }
  }
}

void loop(){
  recieve_commands();
}
