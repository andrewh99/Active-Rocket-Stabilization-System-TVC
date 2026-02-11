#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

// servo objects
Servo servoLow; 
Servo servoUp;  

const int pinLow = 0;
const int pinUp = 1;

// calibration
const float trimLow = -7.5; 
const float trimUp = 5.0; 

const float homeLow = 90.0 + trimLow;
const float homeUp = 90.0 + trimUp;

// gear ratios
const float ratioLow = 5.0;   
const float ratioUp  = 6.91;  

const int limitLow = 25; 
const int limitUp = 35;  

// imu settings
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// offsets
const float offsetLow = 0.28;
const float offsetUp  = -0.24;

void setup() {
  Serial.begin(115200);

  // home positions
  servoLow.write(homeLow); 
  servoUp.write(homeUp);
  
  pinMode(pinLow, OUTPUT);
  digitalWrite(pinLow, LOW);
  pinMode(pinUp, OUTPUT);
  digitalWrite(pinUp, LOW);
  
  delay(10); 

  // attach
  servoLow.attach(pinLow);
  servoUp.attach(pinUp);

  if(!bno.begin()) {
    Serial.println("BNO055 NOT FOUND");
    while(1);
  }

  bno.setExtCrystalUse(true);
  
  // initial position lock

  unsigned long startWait = millis();
  while(millis() - startWait < 100) {
      servoLow.write(homeLow);
      servoUp.write(homeUp);
      delay(100);
  }
}

void loop() {
  // gravity vector
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  // offset mapping
  float tiltLow = gravity.y() - offsetLow;
  float tiltUp  = gravity.z() - offsetUp;

  // deadzone
  if (abs(tiltLow) < 0.05) tiltLow = 0;
  if (abs(tiltUp) < 0.05) tiltUp = 0;

  // calculate target
  float gain = 8.0; 
  float targetLow = homeLow + (tiltLow * gain * ratioLow);
  float targetUp  = homeUp  + (tiltUp  * gain * ratioUp);

  // constraints
  targetLow = constrain(targetLow, homeLow - limitLow, homeLow + limitLow);
  targetUp  = constrain(targetUp, homeUp - limitUp, homeUp + limitUp);

  // update
  servoLow.write(targetLow);
  servoUp.write(targetUp);

  // debug
  Serial.print("L:"); Serial.print(tiltLow);
  Serial.print(" | U:"); Serial.println(tiltUp);

  delay(20); 
}