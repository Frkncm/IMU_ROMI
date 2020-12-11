#ifndef _IMU_CALC_
#define _IMU_CALC_

#define VEL_GAIN   10
#define OFFSET_DOWN_VAL 2.0
#define OFFSET_UP_VAL 3.0

class imuCalculator {

  private:
    uint64_t timer;
    float rotation{0};
    float accOffsetDown{OFFSET_DOWN_VAL};
    float accOffsetUp{OFFSET_UP_VAL};
  public:
    float acceleration{0};
    float distance{0};
    float velocity{0};

    imuCalculator() {
      //default ctor.
    }

    imuCalculator(float offsetUp, float offsetDown) :
      accOffsetUp{offsetUp}, accOffsetDown{offsetDown}  {

    }

    auto getAcc()->float {
      return acceleration;
    }

    void calcVelDist() {
      double dt_ = (double)(micros() - timer) / 1000000.0; // Calculate delta time
      timer = micros();
      velocity =  acceleration * dt_ * VEL_GAIN;
      distance += 0.5 * velocity * dt_;
    }

    bool getAccelMean(float acc) {
      //Get the mean of value between two limit.
      static float collectValue{ 0 };
      static int numberReading{ 0 };
      static bool trigger{ false };

      //Update the distance and velocity params
      calcVelDist();
      if ((acc > accOffsetDown) && (acc < accOffsetUp)) {
        trigger = true;
        collectValue += acc;
        numberReading++;
        return true;
      } else {
        if (trigger && (acc < accOffsetDown)) {
          //get the mean of values
          collectValue /= (float)numberReading;
          acceleration = collectValue;
          //Reset number of reading val
          numberReading = 0;
          collectValue = 0;
          trigger = false;
          return true;
        }
      }
      return false;
    }

    //we need the acceleration obtained instantaneously
    float getRotation(float rot, float desiredAngle) {
      static float initialAngle{ 0 };
      static bool  enterTrigger{ true };

      if (enterTrigger) {
        initialAngle = rot;
        enterTrigger = false;
      }

      if (desiredAngle > 0) {
        //Check whether angle difference exceed the desired angle
        if (desiredAngle > (rot - initialAngle)) {
          rotation = rot;
        } else {
          enterTrigger = true;
          return rotation;
        }
      } else if (desiredAngle < 0) {
        //Check whether angle difference exceed the desired angle
        if (desiredAngle < (rot - initialAngle)) {
          rotation = rot;
        }
        else {
          enterTrigger = true;
          return rotation;
        }
      }
    }
};

#endif _IMU_CALC_

