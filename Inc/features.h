#ifndef FEATURES_H
#define FEATURES_H

#include <math.h>

#define WINDOW_SIZE 150
#define EFFECTIVE_WINDOW_SIZE 100
#define M_PI_2 1.57079632679489661923f

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} SensorData;

// 特征索引定义 (按您的列表顺序)
#define FEATURE_COUNT 14
#define ACC_MAG_MIN 0
#define C5_TRUNK_ORIENTATION 1
#define C4_ANGLE_Z_AXIS_VERTICAL 2
#define ROT_Z_STD 3
#define C2_SUM_VECTOR_MAGNITUDE_HORIZONTAL 4
#define C13_ACTIVITY_SIGNAL_MAGNITUDE_AREA_HORIZONTAL 5
#define C14_VELOCITY_APPROX 6
#define C11_SIGNAL_MAGNITUDE_AREA_HORIZONTAL 7
#define ACC_Y_MEAN 8
#define ACC_MAG_KURTOSIS 9
#define ROT_Y_STD 10
#define C9_STD_MAGNITUDE 11
#define ACC_Z_MEAN 12
#define ACC_X_ZERO_CROSS 13
#define C6_ORIENTATION_CHANGE_HORIZONTAL 14
#define ACC_X_STD 15
#define ACC_MAG_STD 16
#define C8_STD_MAGNITUDE_HORIZONTAL 17
#define ROT_Y_ZERO_CROSS 18
#define C3_MAX_PEAK_TO_PEAK_AMPLITUDE 19

void get_feature(SensorData* window, int window_head, float t, float* features);

#endif