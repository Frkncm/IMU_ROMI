#include <Wire.h>
#include <LSM6.h>
#include <Filters.h>
#include "complementary.hpp"
#include "nonBlockingMillis.h"
#include "motor.hpp"
#include "pidLib.h"
#include "imuCalc.hpp"

#define KP_VAL 1   // increase response time, but it'll increase oscillation also
#define KI_VAL 1   // minimise the total error 
#define KD_VAL 1   // if there is a huge changing

LSM6 imu;
ComplementFilter<LSM6> cmp(imu);
imuCalculator imclc;

//create the functions which are called periodically
void imuTask(void);
taskInsert imuTaskIns(imuTask, 10);

//right and left motor instances
myMotor<uint8_t> leftMotor(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotor(R_DIR_PIN, R_PWM_PIN);

//PID instances for left motor and right motor
PID pidForLeft(KP_VAL, KI_VAL, KD_VAL);
PID pidForRight(KP_VAL, KI_VAL, KD_VAL);


void imuTask(void) {


  static int staticTime = 0;
  //update the filter values
  cmp.updateFilter();

  //Serial.print(cmp.getFilteredX()); Serial.println(" ");
  //Serial.print(cmp.getFilteredY()); Serial.println(" ");
  //Serial.print(cmp.getFilteredZ()); Serial.println();
  imclc.updateAcc(cmp.getFilteredY(), imuCalculator::FORWARD);
  Serial.println(imclc.getAcc());
  
  staticTime++;
  if ( staticTime > 500 && staticTime < 700) {
    leftMotor.motorControl(-60);
    rightMotor.motorControl(-60);
  } else {
    leftMotor.motorControl(0);
    rightMotor.motorControl(0);
  }


}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();

  delay(1000); // Wait for sensor to stabilize

  pidForRight.reset();
  pidForLeft.reset();

}

void loop() {

  taskInsert::executeTasks();

  imuTaskIns.callMyTask();

}




