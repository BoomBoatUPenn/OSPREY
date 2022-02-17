
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#define MAX_VAL 32769.0

char packetBuffer[255]; 

const char* ssid     = "BoomBoat";
const char* password = "BoomBoat";

WiFiUDP UdpCommand;
WiFiUDP UdpTelem;

unsigned int UDPPortCommands = 5005;
unsigned int UDPPortTelem = 5006;

IPAddress ipTarget(172, 16, 12, 255);
IPAddress ipLocal(172, 16, 12, 10);
IPAddress dns(172, 16, 12, 1);

#define ESC_PIN 23
#define RUDDER_SERVO_PIN 15

const float ServoNeutral = 0.18;

const int ESCChannel = 1;
const int RudderServoChannel = 2;

const int resolution = 13;
const int freq = 50;

long heartbeat = 0;
int BoatNum = 0;

float Throttle = 0;
float RudderAngle = 0;


void setup() {                  
  Serial.begin(115200);  
 
  WiFi.config(ipLocal,  dns, IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password); 

  UdpCommand.begin(UDPPortCommands);
  UdpTelem.begin(UDPPortTelem);

  ledcSetup(ESCChannel, freq, resolution);
  ledcSetup(RudderServoChannel, freq, resolution);

  ledcAttachPin(ESC_PIN, ESCChannel);
  ledcAttachPin(RUDDER_SERVO_PIN, RudderServoChannel);

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.println("no wifi");
  }
}

void recieve_commands(){
  int packetSize = UdpCommand.parsePacket();

  if (packetSize) {

    int len = UdpCommand.read(packetBuffer, 255);

    if (len > 0) {
      packetBuffer[len] = 0;
    }

    if (packetBuffer[0] == 'c'){
      char *token;
      // discard 'c' character
    }

    if (packetBuffer[0] == 't'){
      Throttle = atof(packetBuffer+1);
      update_motors();
    }

    if (packetBuffer[0] == 'r'){
      RudderAngle = atof(packetBuffer+1);
      update_motors();
    }
    if (packetBuffer[0] == 'h'){
      heartbeat = 1000;
    }
  }
}

int angleToDC(float angle){
     return ((angle - 0.0) * (820.0 - 410.0) / (1.0 - 0.0)) + 410.0;
}

void update_motors(){
  int t = 0;
  int r = 0;

  if (heartbeat > 0){
    t = angleToDC(Throttle);
    r = angleToDC(RudderAngle);
  } else{
    t = angleToDC(0.);
    r = angleToDC(ServoNeutral);
  }

  Serial.print("Throttle");
  Serial.println(t);
  ledcWrite(ESCChannel, t);
  Serial.print("rudder angle");
  Serial.println(r);
  ledcWrite(RudderServoChannel, r);
  
}

void loop(){
  static long last = millis();
  if(millis()>last && heartbeat > 0){
    last = millis();
    heartbeat--;
  } else if (heartbeat == 0){
    update_motors();
  }
  recieve_commands();

}
