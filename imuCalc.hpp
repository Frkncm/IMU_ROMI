#ifndef _IMU_CALC_
#define _IMU_CALC_

#define ARR_SIZE 10
#define OFFSET_VAL 2.0

class imuCalculator {

  private:

    float distance{0};
    float rotation{0};
    float acceleration{0};
    float previousAcc{0};
    float accOffset{OFFSET_VAL};
  public:

    enum tasks {
      DETECT_ACCELERATION,
      TASK_DONE,
    };
    enum imuDirection {
      FORWARD,
      BACKWARD,
      ROTATION,
    };

    imuCalculator() {

    }

    imuCalculator(float offset) : accOffset{offset} {

    }

    auto getAcc()->float {
      return acceleration;
    }
    void swap(float * const xp, float * const yp)
    {
      int temp = *xp;
      *xp = *yp;
      *yp = temp;
    }

    void bubbleSort(float arr[], uint8_t n)
    {
      int i, j;
      for (i = 0; i < n - 1; i++)

        // Last i elements are already in place
        for (j = 0; j < n - i - 1; j++)
          if (arr[j] > arr[j + 1])
            swap(&arr[j], &arr[j + 1]);
    }


    //we need the acceleration obtained instantaneously
    boolean updateAcc(float acc, imuDirection dr ) {
      //try to find the changes of acc between previous value
      static float accArr[ARR_SIZE] = {0};
      static uint8_t indx = 0;

      for (int i = 0; i < (ARR_SIZE - 1); i++) {
        accArr[i] = accArr[i + 1];
      }

      bubbleSort(accArr, ARR_SIZE);

      if (dr == FORWARD) {
        if ( (accArr[ARR_SIZE - 1] - accArr[0]) > accOffset) {
          //if acceleration has a plateu, it is the max rate
          if (accArr[indx] > 0) {
            acceleration = accArr[indx];
            if ( previousAcc > acceleration)
              acceleration = previousAcc;
            previousAcc = acceleration;
          }
          accArr[indx++] = acc;
          if (indx >= ARR_SIZE)
            indx = ARR_SIZE - 1;
          if (( accArr[indx] < 0) || (accArr[indx] < accArr[indx - 1]))
            return false;
          return true;
        } else {
          accArr[indx++] = acc;
          if (indx >= ARR_SIZE)
            indx = ARR_SIZE - 1;
        }
      }

      return false;
    }

};

#endif _IMU_CALC_
