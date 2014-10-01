#ifndef __MOTION_H_
#define __MOTION_H_

double calib_vec_length = 0.0;
#define CALIB_VEC_TIMES 10
#define VEC_LENGTH_THD_GAIN 0.3
#define VEC_LENGTH_THD_GAIN_FOR_TRIGGER_FLAG 0.8
#define CALIB_VEC_DELAY 50

struct _vec_cal {
  double cos_vec;
  double length1;
  double length2;
};

//double get_cos_two_vector((_vec_cal*) v, int16_t sx, int16_t sy, int16_t sz, int16_t tx, int16_t ty, int16_t tz);
double get_cos_two_vector(double* v, int16_t sx, int16_t sy, int16_t sz, int16_t tx, int16_t ty, int16_t tz);

#define NOT_READY_FLAG 0
#define READY_FLAG 1

uint8_t motion_flag = NOT_READY_FLAG;  // ready to make sound = 1

void set_motion_flag();
uint8_t get_motion_flag();
void clear_motion_flag();

uint16_t trigger_flag_timeout_count = 0;
#define TRIGGER_FLAG_TIMEOUT 500 // automatically clear trigger flag

#endif
