#ifndef __CHANNEL_CHANGES_H__
#define __CHANNEL_CHANGES_H__
#include "main.h"
void translational_control();
void compound_control();
void rotate_control();
typedef struct
{
  float k;//????
  float lVal;//?????
}rcPara_t;

#define PI 3.1415926f
#endif