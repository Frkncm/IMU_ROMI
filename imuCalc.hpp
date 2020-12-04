#ifndef _IMU_CALC_
#define _IMU_CALC_

#define ARR_SIZE 10
#define OFFSET_VAL 1.5

class imuCalculator {

  private:
    uint64_t timer;
    float rotation{0};
    float previousAcc{0};
    float accOffset{OFFSET_VAL};

  public:
    float acceleration{0};
    float distance{0};
    float velocity{0};

    enum tasks {
      DETECT_ACCELERATION,
      TASK_DONE,
    };
    enum imuDirection {
      FORWARD,
      BACKWARD,
      ROTATION,
      STOP
    };

    imuCalculator() {

    }

    imuCalculator(float offset) : accOffset{offset} {

    }

    auto getAcc()->float {
      return acceleration;
    }

    void calcVelDist() {
      double dt_ = (double)(micros() - timer) / 1000000.0; // Calculate delta time
      timer = micros();
      velocity = acceleration * dt_ * 10;
      distance += 0.5 * velocity * dt_;
    }

    //we need the acceleration obtained instantaneously
    boolean updateAcc(float acc, imuDirection dr ) {

      calcVelDist();

      if (dr == FORWARD) {
        if ((acc > accOffset) && (acc > 0)) {
          acceleration = acc;
          if ( previousAcc > acceleration)
            acceleration = previousAcc;
          previousAcc = acceleration;
          return true;
        }
      } else if (dr == STOP) {

      }
      return false;
    }

};

#endif _IMU_CALC_
