#ifndef _IMU_CALC_
#define _IMU_CALC_

#define VEL_GAIN   10000
#define OFFSET_DOWN_VAL 2.0
#define OFFSET_UP_VAL 3.0
#define ANG_COEF 0.07
#define IMU_MSPEED 40
#define ROT_MEAN 44.15

class imuCalculator {

  private:
    uint64_t timer;
    float accOffsetDown{OFFSET_DOWN_VAL};
    float accOffsetUp{OFFSET_UP_VAL};
  public:
    float acceleration{0};
    float distance{0};
    float velocity{0};
    float rotation{0};
    float Xnew{0}, Ynew {0}; // Y - coordinate

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

    void getCoordinate (float rot, float acc) {
      //calculate x - y position
      static float last_distance {0};
      getAccelMean(acc);
      updateRotation(rot);
      Ynew += (distance - last_distance) * sin(rotation * PI / 180);
      Xnew += (distance - last_distance) * cos(rotation * PI / 180);
      last_distance = distance;
    }

    float updateRotation(float rot) {
      //get the real-time rotation value
      if ((rot - ROT_MEAN) > 1.0) {
        rotation += (rot - ROT_MEAN) * ANG_COEF;
      } else if ((rot - ROT_MEAN) < -1.0 ) {
        rotation += (rot - ROT_MEAN) * ANG_COEF;
      }
      return rotation;
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

    boolean turnDegree(float rot, float desiredAngle) {
      static float initialAngle{ 0 };
      static float locRot{ 0 };
      static bool  enterTrigger{ true };

      if (enterTrigger) {
        locRot = 0;
        //calTime = 0;
        //locTime = micros();
        initialAngle = rot * ANG_COEF;
        enterTrigger = false;
      }

      rotation += (rot * ANG_COEF - initialAngle);

      if (desiredAngle > 0) {
        //compare the angle difference for positive values
        if (desiredAngle > rotation) {
          return false;
        } else {
          enterTrigger = true;
          return true;
        }
      } else if (desiredAngle < 0) {
        //compare the angle difference for positive values
        if (desiredAngle <= rotation) {
          return false;
        }
        else {
          enterTrigger = true;
          return true;
        }
      }
      return false;
    }
};

#endif _IMU_CALC_


