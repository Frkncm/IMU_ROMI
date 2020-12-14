#include <Wire.h>
#include <LSM6.h>
#include "complementary.hpp"
#include "nonBlockingMillis.h"
#include "motor.hpp"
#include "pidLib.h"
#include "imuCalc.hpp"
#include "Romi.h"

#define KP_VAL 1.0   // increase response time, but it'll increase oscillation also
#define KI_VAL 0.5   // minimise the total error 
#define KD_VAL 1.0   // if there is a huge changing

#define SQUARE_DISTANCE 300

LSM6 imu;
ComplementFilter<LSM6> cmp(imu);
imuCalculator imclc;

//Bluetooth instance
HardwareSerial& bthc05(Serial1);

//These variables for handling the finite state machine
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

//right and left motor instances
myMotor<uint8_t> leftMotor(L_DIR_PIN, L_PWM_PIN);
myMotor<uint8_t> rightMotor(R_DIR_PIN, R_PWM_PIN);

PID pidMotor(KP_VAL, KI_VAL, KD_VAL);

int8_t turnRight = 1;
int8_t turnLeft  = 1;

uint8_t countTask{ 0 };

//square implementation
bool taskHandler() {
  if (countTask == 0) {
    if (imclc.Xnew <= SQUARE_DISTANCE) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 1) {
    if (imclc.Ynew >= -SQUARE_DISTANCE) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 2) {
    if (imclc.Xnew >= 0) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 3) {
    if (imclc.Ynew <= 0) {
      return false;
    }
    else {
      countTask++;
      return true;
    }
  }
}

//create the function which is called periodically
void imuTask(void);
taskInsert imuTaskIns(imuTask, 10);

void imuTask(void) {
  static int staticTime = 0;

  //update the filter values
  cmp.updateFilter();

  //get the coordinate using IMU sensor parameters
  imclc.getCoordinate(cmp.getFilteredZ(), cmp.getFilteredY());

  /*
    Serial.print("X: ");  Serial.print(imclc.Xnew);
    Serial.print(" Y: "); Serial.print(imclc.Ynew);
    Serial.print(" D: "); Serial.print(imclc.distance);
    Serial.print(" R: "); Serial.println(imclc.rotation);
  */
  //Send values trough bluetooth
  bthc05.print(imclc.Ynew); bthc05.print("\t");
  bthc05.print(imclc.Xnew); bthc05.print("\t");
  bthc05.println(imclc.rotation);

  //float v = pidMotor.updateValue(30, imclc.velocity);
  leftMotor.motorControl(turnLeft * -30);
  rightMotor.motorControl(turnRight * -30);

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

  delay(4000);

  for (int i = 0; i < 1000; i++) {
    //wait till sensor stabilize
    cmp.updateFilter();
    imclc.rotation = 0;
  }

  bthc05.begin(9600);
  GO_HANDLE(IDLE_STATE); // start with handling IDLE state

}

void loop() {

  //Update the time for non-blocking tasks
  taskInsert::executeTasks();

  switch (current_state) {

    case IDLE_STATE: {
        /*Start the state machine with the idle state (do nothing here)
          GO_HANDLE macro and the others are defined in the Romi.h file.
          They are used to navigate the flow of control. */

        GO_HANDLE(PATH_TRACING);
        break;
      }

    case PATH_TRACING: {
        if (!taskHandler()) {
          turnLeft = 1;
          turnRight = 1;
          imuTaskIns.callMyTask();
          GO_HANDLE(PATH_TRACING);
        } else if (countTask >= 4) {
          BREAK_AND_GO(STOP_SYSTEM)
        } else {
          //Reset the accel. value and motor speed
          imclc.acceleration = 0.0;
          leftMotor.motorControl(0);
          rightMotor.motorControl(0);
          GO_HANDLE(WAIT_BEFORE_TURNING);
        }

        break;
      }

    case WAIT_BEFORE_TURNING: {
        WAIT_NONBLOCKING_SAME_MS(500, WAIT_BEFORE_TURNING);
        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    case TURN_ROMI_STATE: {
        if (!imclc.turnDegree(-90)) {
          //turn motor right
          turnLeft = 1;
          turnRight = -1;
          imuTaskIns.callMyTask();
          GO_HANDLE(TURN_ROMI_STATE);
        } else {
          imclc.acceleration = 0.0;
          leftMotor.motorControl(0);
          rightMotor.motorControl(0);
          GO_HANDLE(MOTOR_STOP);
        }
        break;
      }

    case MOTOR_STOP: {
        cmp.updateFilter();//stabilize its z axis value
        WAIT_NONBLOCKING_SAME_MS(500, MOTOR_STOP);
        leftMotor.motorControl(0);
        rightMotor.motorControl(0);
        GO_HANDLE(PATH_TRACING);
        break;
      }

    case STOP_SYSTEM: {
        leftMotor.motorControl(0);
        rightMotor.motorControl(0);
        break;
      }
    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }

  }
}



