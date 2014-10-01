#include "motion.h"

//double get_cos_two_vector((_vec_cal*) v, int16_t sx, int16_t sy, int16_t sz, int16_t tx, int16_t ty, int16_t tz)
double get_cos_two_vector(double* v, int16_t sx, int16_t sy, int16_t sz, int16_t tx, int16_t ty, int16_t tz)
{
  double cos_vec = 0.0;

//  Serial.print("vecs = (");  
//  Serial.print(sx); Serial.print(":"); Serial.print(sy); Serial.print(":"); Serial.print(sz); Serial.print(" ) , ( ");
//  Serial.print(tx); Serial.print(":"); Serial.print(ty); Serial.print(":"); Serial.print(tz); Serial.println(")");

  double s_leng = sqrt(sx * sx + sy * sy + sz * sz);
  double t_leng = sqrt(tx * tx + ty * ty + tz * tz);  

//  Serial.print("lengths = ");    
//  Serial.print(s_leng); Serial.print(", "); Serial.println(t_leng);
  
  cos_vec = (sx * tx + sy * tz + sz * tz) / s_leng / t_leng;
  
//  Serial.print("cos = "); Serial.println(cos_vec);
  
  v[0] = cos_vec;
  v[1] = s_leng;  
  v[2] = t_leng;
  
  return cos_vec;
}


void set_motion_flag()
{
  motion_flag = READY_FLAG;
  Serial.println("&&&&&&&&&&&&&&&&&&&&&&&");
  Serial.println("&&& SET MOTION FLAG &&&");
  Serial.println("&&&&&&&&&&&&&&&&&&&&&&&");
}
uint8_t get_motion_flag()
{
  return motion_flag;
}  
void clear_motion_flag()
{
  motion_flag = NOT_READY_FLAG;
  trigger_flag_timeout_count = 0;  
  Serial.println("=========================");
  Serial.println("=== CLEAR MOTION FLAG ===");
  Serial.println("=========================");
}

