#ifndef _CONVERSION_
#define _CONVERSION_

//#define PI 3.141592653589793f

inline float toRad(float x){
  return (x * PI/180.0f);
}

inline float toDeg(float x){
  return (x * 180.0f/PI);
}

#endif
