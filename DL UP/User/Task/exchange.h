#ifndef EXCHANGE_H
#define EXCHANGE_H
#include "struct_typedef.h"
void exchangeangle_task(void const *pvParameter);
typedef struct INS_DATA
{
    fp32 accel_offset[3];
    fp32 gyro_offset[3];
    fp32 accel[3];
    fp32 temp;
    fp32 gyro[3];
    fp32 angle[3];
    fp32 INS_quat[4];  
} ins_data_t;
#endif