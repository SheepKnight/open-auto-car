#include <IBusBM.h>
#include <ESP32Servo.h>
#include "MPU9250.h"
#include "WiFi.h"
#include <DNSServer.h> 
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "ArduinoJson.h"

MPU9250 IMU(Wire,0x68);
#define WIFI_CONTROLS
const char* ssid     = "🚗 Tes-Ici 🚙";

const byte        DNS_PORT = 53;          // Capture DNS requests on port 53
IPAddress         apIP(10, 10, 10, 1);    // Private network for server
DNSServer         dnsServer;     
AsyncWebServer server(80);
AsyncWebSocket ws("/websocket");

//#define ENABLE_PID_STEER
#define KP .12
#define KI .0003
#define KD 0

#define SERVO_PIN 12
#define THRUST_PIN 13

#define FREE_MS 10

#define RFRemote 1
#define WiFiController 2
#define RPIController 3

uint8_t current_control = RFRemote;
int steer_val = 1500;
int thrust_val = 1500;

Servo steer;
Servo thrust;

IBusBM IBus;

#ifdef ENABLE_PID_STEER
double steer, setPoint, outputVal;
AutoPID myPID(&steer, &setPoint, &outputVal, 1150, 1850, KP, KI, KD);
#endif

class CaptiveRequestHandler : public AsyncWebHandler {
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    //request->addInterestingHeader("ANY");
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<!DOCTYPE html><html><head><title>RC-WiFi Car</title></head><body>");
    response->printf("<meta http-equiv=\"refresh\" content=\"1;URL=%s\">", WiFi.softAPIP().toString().c_str());
    response->print("</body></html>");
    request->send(response);
  }
};

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client_web, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT){
    Serial.println("#Websocket client connection received");
    current_control = WiFiController;
    
  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
    current_control = RFRemote;
  } else if(type == WS_EVT_DATA){
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
      Serial.print("#deserializeJson() failed");
      return;
    }
    if(strcmp(doc["type"], "controller")){
      steer_val = doc["steering"];
      thrust_val = doc["thrust"];
    }
  }
}

void setup() {
  Serial.begin(115200);     // debug info
  if(!SPIFFS.begin(true)){
    Serial.println("#An Error has occurred while mounting SPIFFS");
    return;
  }
  if (IMU.begin() < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1) {}
  }
  
  #ifdef WIFI_CONTROLS
  Serial.print("#Beginning WiFi      : ");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid);
  
  ws.onEvent(onWsEvent);

  server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, NULL);
  });
  
  server.on("/joy.js", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/joy.js", String(), false, NULL);
  });
  server.on("/canvasjs.min.js", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/canvasjs.min.js", String(), false, NULL);
  });
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/leaf.ico", String(), false, NULL);
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
  server.begin();
  Serial.println("Done");
  
  #endif
  
  Serial.print("#Beginning servos    : ");
  IBus.begin(Serial2,1);    // iBUS object connected to serial2 RX2 pin and use timer 1
  steer.attach(SERVO_PIN); // attaches the servo on pin 18 to the servo object (using timer 0)
  thrust.attach(THRUST_PIN); // attaches the servo on pin 18 to the servo object (using timer 0)
  Serial.println("Done");
  #ifdef ENABLE_PID_STEER
  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(3);
  //set PID update interval to 4000ms
  myPID.setTimeStep(4000);
  #endif

  
}


void loop() {
  dnsServer.processNextRequest();
  
  if(current_control == RFRemote){

    #ifdef ENABLE_PID_STEER
  
    #else
    steer_val = max(1150,min(1850,(int)IBus.readChannel(0)));
    #endif
  
    int thrust_stick = (int)IBus.readChannel(2)/2-500;
    int limit_switch = (int)IBus.readChannel(4)/2-500;
    thrust_stick = map(thrust_stick, 0, 500, 0, limit_switch);
    if(IBus.readChannel(8) < 1350){
      thrust_val = 1500+thrust_stick;
    }else if(IBus.readChannel(8) > 1850){
      thrust_val = 1500-thrust_stick;
    }else{
      thrust_val = 1500;
    }
    
  }
  
  StaticJsonDocument<500> doc;
  doc["AccelX"] = IMU.getAccelX_mss();
  doc["AccelY"] = IMU.getAccelY_mss();
  doc["AccelZ"] = IMU.getAccelZ_mss();
  doc["GyroX"] = IMU.getGyroX_rads();
  doc["GyroY"] = IMU.getGyroY_rads();
  doc["GyroZ"] = IMU.getGyroZ_rads();
  doc["MagX"] = IMU.getMagX_uT();
  doc["MagY"] = IMU.getMagY_uT();
  doc["MagZ"] = IMU.getMagZ_uT();
  
  
  char output[200];
  serializeJson(doc, output);
  ws.textAll(output);
  
  steer.writeMicroseconds(steer_val);
  thrust.writeMicroseconds(thrust_val);
  /*Serial.print("#Values are :");
  Serial.print(steer_val);
  Serial.print(", ");
  Serial.println(thrust_val);
  */
  delay(FREE_MS);
}
