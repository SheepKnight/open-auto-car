/* ==============================================
 * author   :   Albin Mouton
 * date     :   11/10/2020
 * ==============================================
 */

//Libraries :
#include <IBusBM.h> //https://github.com/bmellink/IBusBM
#include <ESP32Servo.h>


//Uncomment next line to enable PID Steering with accelerometer (not yet supported)
//#define ENABLE_PID_STEER
#define KP .12
#define KI .0003
#define KD 0

#define SERVO_PIN 12 //Servomotor controlling the car's direction
#define THRUST_PIN 13 //Motor's ESC

#define FREE_MS 10 //Delay at the end of the main loop (mandatory for the IBus)

//Keywords for ways to control the car (not yet supported)
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
//AutoPID myPID(&steer, &setPoint, &outputVal, 1150, 1850, KP, KI, KD);
#endif

void setup() {
  Serial.begin(115200);     // debug info
  Serial.print("#Beginning servos    : ");
  IBus.begin(Serial2,1);    // iBUS object connected to serial2 RX2 pin and use timer 1
  steer.attach(SERVO_PIN); // attaches the servo on pin 18 to the servo object (using timer 0)
  thrust.attach(THRUST_PIN); // attaches the servo on pin 18 to the servo object (using timer 0)
  Serial.println("Done");
  #ifdef ENABLE_PID_STEER
  //myPID.setBangBang(3);
  //myPID.setTimeStep(4000);
  #endif

  
}


void loop() {

  if(current_control == RFRemote){

    #ifdef ENABLE_PID_STEER
      #error PID STEER NOT YET SUPPORTED
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
  
  steer.writeMicroseconds(steer_val);
  thrust.writeMicroseconds(thrust_val);
  delay(FREE_MS);
}
