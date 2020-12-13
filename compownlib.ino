#include <Wire.h>
#include <LSM6.h>
#include "complementary.hpp"
#include "nonBlockingMillis.h"
#include "motor.hpp"
#include "pidLib.h"
#include "imuCalc.hpp"

#define KP_VAL 1.0   // increase response time, but it'll increase oscillation also
#define KI_VAL 0.5   // minimise the total error 
#define KD_VAL 1.0   // if there is a huge changing

LSM6 imu;
ComplementFilter<LSM6> cmp(imu);
imuCalculator imclc;

//create the functions which are called periodically
void imuTask(void);
taskInsert imuTaskIns(imuTask, 10);

//right and left motor instances
myMotor<uint8_t> leftMotor(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotor(R_DIR_PIN, R_PWM_PIN);

PID pidMotor(KP_VAL, KI_VAL, KD_VAL);

void imuTask(void) {
  static int staticTime = 0;
  //update the filter values
  cmp.updateFilter();
  //get the mean of accel.
  //Serial.print(cmp.getFilteredX()); Serial.print(" ");
  //Serial.print(cmp.getFilteredY()); Serial.print(" ");
  //Serial.print(cmp.getFilteredZ()); Serial.println(" ");
  //Serial.print(imclc.acceleration); Serial.println(" ");
  Serial.print("X: "); Serial.print(imclc.Xnew);
  Serial.print(" Y: "); Serial.print(imclc.Ynew);
  Serial.print(" D: "); Serial.println(imclc.distance);
  Serial.print(" R: "); Serial.println(imclc.rotation);
  imclc.getCoordinate(cmp.getFilteredZ(), cmp.getFilteredY());
  /*
    staticTime++;
    static bool isEntered = false;
    if ( staticTime > 400 && staticTime < 650) {
    //This code snippet is designed for checking the acc, velocity
    //distance in a certain time.
    if (!imclc.getRotation(cmp.getFilteredZ(), 90) && !isEntered) {
    float v = pidMotor.updateValue(IMU_MSPEED, imclc.velocity);
    leftMotor.motorControl(IMU_MSPEED);
    rightMotor.motorControl(-IMU_MSPEED);
    } else {
    isEntered = true;
    imclc.acceleration = 0.0;
    leftMotor.motorControl(0);
    rightMotor.motorControl(0);
    }
    } else {
    imclc.acceleration = 0.0;
    leftMotor.motorControl(0);
    rightMotor.motorControl(0);
    }


    Serial.print(" Velocity: "); Serial.print(imclc.velocity);
    Serial.print(" Distance: "); Serial.print(imclc.distance);
    Serial.print(" Mean acc: "); Serial.println(imclc.acceleration);
  */
  staticTime++;
  if ( staticTime > 500 && staticTime < 650) {
    //This code snippet is designed for checking the acc, velocity
    //distance in a certain time.
    imclc.getCoordinate(cmp.getFilteredZ(), cmp.getFilteredY());
    float v = pidMotor.updateValue(40, imclc.velocity);
    leftMotor.motorControl(v);
    rightMotor.motorControl(v);
  } else {
    imclc.acceleration = 0.0;
    leftMotor.motorControl(0);
    rightMotor.motorControl(0);
  }

}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Wire.begin();

  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  delay(1000); // Wait for sensor to stabilize
  pidMotor.reset();

  for (int i = 0; i < 1000; i++) {
    //wait till sensor stabilize
    cmp.updateFilter();
    imclc.rotation = 0;
  }

}

void loop() {

  taskInsert::executeTasks();

  imuTaskIns.callMyTask();

}



