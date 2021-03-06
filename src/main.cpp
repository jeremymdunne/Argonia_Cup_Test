#include <Arduino.h>

#include <Servo.h> 
#include <PID.h> 
#include <ICM20948.h> 

/*
Argonia Cup Firmware V.01 

Purpose: Guide payload to a GPS location as accurately as possible 

Sensors: GPS, Magnetometer, Gyroscope, (accelerometer, barometer, temperature)

Locomotion Method: Gridfins actuated by servos.

Controls: Yaw stabalization, Lateral Control (LAT&LONG) 

First TODO: Yaw stabalization. 
  How Do: Orient vehicle to a heading to prove control authority. Rotate grid fins to counter the spin/current heading of the vehicle

*/ 

// Hi Jacob 

#define SERVO_1_PIN PA0
#define SERVO_2_PIN PA1 
#define SERVO_3_PIN PA2
#define SERVO_4_PIN PA3


#define SERVO_1_MIDPOINT 90 
#define SERVO_2_MIDPOINT 90
#define SERVO_3_MIDPOINT 90
#define SERVO_4_MIDPOINT 90 

#define Y_COMP 5.84
#define X_COMP 47.37
#define Y_SCALE 1
#define X_SCALE 1 



Servo servo1; 
Servo servo2; 
Servo servo3;
Servo servo4; 

PID yawPid(1,0,0,100);
ICM20948 IMU(Wire, 0x69);
int status;

void writeServo(float servo1, float servo2, float servo3, float servo4);

void setup(){
  delay(2000);
  Serial.begin(115200); 
  Serial.println("Hello World!"); 
  status = IMU.begin();
  Serial.print("status = ");
  Serial.println(status);
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  float aFloat = 3.456; 
  //Serial.println("This is a Float: " + (aFloat)); 
  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  servo3.attach(SERVO_3_PIN);
  servo4.attach(SERVO_4_PIN);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);


}

float getHeading(){
  IMU.readSensor();
  float heading = (atan2(IMU.getMagY_uT() + Y_COMP,IMU.getMagX_uT() + X_COMP) * 180) / PI;
 
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  return heading; 
}


void writeServo(float servo1Val, float servo2Val, float servo3Val, float servo4Val){
  servo1.write(servo1Val + SERVO_1_MIDPOINT);
  servo2.write(servo2Val + SERVO_2_MIDPOINT); 
  servo3.write(servo3Val + SERVO_3_MIDPOINT);
  servo4.write(servo4Val + SERVO_4_MIDPOINT);
}

void yaw_stabalization(){ 
  // get our orientation 
  float orientation = getHeading(); 
  Serial.println("Orientation: " + String(orientation)); 

  // calculate grid fin angles to get back to 'home' 

  float finAngle = yawPid.calculate(orientation); 
  writeServo(finAngle, -1* finAngle, -1*finAngle, finAngle); 
  Serial.println("Fin Angle: " + String(finAngle)); 

}


void loop(){
  long start = millis(); 
  yaw_stabalization(); 
  long stop = millis(); 
  delay(100 - (stop-start)); 
  Serial.println("Loop"); 
}