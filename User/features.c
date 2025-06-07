#include "features.h"

// 获取循环缓冲区中的数据索引
static inline int get_buffer_index(int i, int head) {
    return (i + head + 50) % WINDOW_SIZE;
}

// 计算峰度 (简化版本)
static float calc_kurtosis(float* data, int size, float mean, float std) {
    if (std < 1e-6f) return 0.0f;
    
    float sum4 = 0.0f;
    float inv_std4 = 1.0f / (std * std * std * std);
    
    for (int i = 0; i < size; i++) {
        float diff = data[i] - mean;
        sum4 += diff * diff * diff * diff;
    }
    
    return (sum4 * inv_std4 / size) - 3.0f;
}

// 计算过零点数量
static int calc_zero_crossings_circular(SensorData* window, int head, int size, int axis) {
    int count = 0;
    float prev_val, curr_val;
    
    // 获取第一个值
    int idx = get_buffer_index(0, head);
    switch(axis) {
        case 0: prev_val = window[idx].ax; break;  // acc_x
        case 1: prev_val = window[idx].gy; break;  // rot_y
        default: return 0;
    }
    
    for (int i = 1; i < size; i++) {
        idx = get_buffer_index(i, head);
        switch(axis) {
            case 0: curr_val = window[idx].ax; break;
            case 1: curr_val = window[idx].gy; break;
            default: return 0;
        }
        
        if ((prev_val > 0 && curr_val < 0) || (prev_val < 0 && curr_val > 0)) {
            count++;
        }
        prev_val = curr_val;
    }
    return count;
}

void get_feature(SensorData* window, int window_head, float t, float* features) {
    float dt = t / EFFECTIVE_WINDOW_SIZE;
    
    // 临时数组存储需要重复访问的数据
    float acc_mag[EFFECTIVE_WINDOW_SIZE];
    float acc_mag_horiz[EFFECTIVE_WINDOW_SIZE];
    
    // 累积统计值 - 单次遍历计算多个统计量
    float sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gz = 0, sum_gy = 0;
    float sum_sq_ax = 0, sum_sq_ay = 0, sum_sq_az = 0, sum_sq_gz = 0, sum_sq_gy = 0;
    float sum_acc_mag = 0, sum_sq_acc_mag = 0;
    float min_acc_mag = INFINITY;
    float max_ax = -INFINITY, min_ax = INFINITY;
    float max_ay = -INFINITY, min_ay = INFINITY;
    float max_az = -INFINITY, min_az = INFINITY;
    
    // 主循环 - 使用循环缓冲区索引
    for (int i = 0; i < EFFECTIVE_WINDOW_SIZE; i++) {
        int idx = get_buffer_index(i, window_head);
        
        float ax = window[idx].ax;
        float ay = window[idx].ay;
        float az = window[idx].az;
        float gy = window[idx].gy;
        float gz = window[idx].gz;
        
        // 累积求和
        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gy += gy; sum_gz += gz;
        sum_sq_ax += ax * ax; sum_sq_ay += ay * ay; sum_sq_az += az * az;
        sum_sq_gy += gy * gy; sum_sq_gz += gz * gz;
        
        // 计算加速度模长
        acc_mag[i] = sqrtf(ax*ax + ay*ay + az*az);
        acc_mag_horiz[i] = sqrtf(ax*ax + az*az);
        
        sum_acc_mag += acc_mag[i];
        sum_sq_acc_mag += acc_mag[i] * acc_mag[i];
        
        // 最值更新
        if (acc_mag[i] < min_acc_mag) min_acc_mag = acc_mag[i];
        if (ax > max_ax) max_ax = ax; if (ax < min_ax) min_ax = ax;
        if (ay > max_ay) max_ay = ay; if (ay < min_ay) min_ay = ay;
        if (az > max_az) max_az = az; if (az < min_az) min_az = az;
    }
    
    // 计算均值
    float mean_ax = sum_ax / EFFECTIVE_WINDOW_SIZE;
    float mean_ay = sum_ay / EFFECTIVE_WINDOW_SIZE;
    float mean_az = sum_az / EFFECTIVE_WINDOW_SIZE;
    float mean_gy = sum_gy / EFFECTIVE_WINDOW_SIZE;
    float mean_gz = sum_gz / EFFECTIVE_WINDOW_SIZE;
    float mean_acc_mag = sum_acc_mag / EFFECTIVE_WINDOW_SIZE;
    
    // 计算标准差
    float var_ax = (sum_sq_ax / EFFECTIVE_WINDOW_SIZE) - (mean_ax * mean_ax);
    float var_ay = (sum_sq_ay / EFFECTIVE_WINDOW_SIZE) - (mean_ay * mean_ay);
    float var_az = (sum_sq_az / EFFECTIVE_WINDOW_SIZE) - (mean_az * mean_az);
    float var_gy = (sum_sq_gy / EFFECTIVE_WINDOW_SIZE) - (mean_gy * mean_gy);
    float var_gz = (sum_sq_gz / EFFECTIVE_WINDOW_SIZE) - (mean_gz * mean_gz);
    float var_acc_mag = (sum_sq_acc_mag / EFFECTIVE_WINDOW_SIZE) - (mean_acc_mag * mean_acc_mag);
    
    float std_ax = sqrtf(var_ax > 0 ? var_ax : 0);
    float std_ay = sqrtf(var_ay > 0 ? var_ay : 0);
    float std_az = sqrtf(var_az > 0 ? var_az : 0);
    float std_gy = sqrtf(var_gy > 0 ? var_gy : 0);
    float std_gz = sqrtf(var_gz > 0 ? var_gz : 0);
    float std_acc_mag = sqrtf(var_acc_mag > 0 ? var_acc_mag : 0);
    
    // 直接赋值的特征
    features[ACC_MAG_MIN] = min_acc_mag;
    features[ACC_Y_MEAN] = mean_ay;
    features[ACC_Z_MEAN] = mean_az;
    features[ROT_Z_STD] = std_gz;
    features[ROT_Y_STD] = std_gy;
    
    // C2: 水平面矢量模 RMS
    float sum_sq_horiz = 0;
    for (int i = 0; i < EFFECTIVE_WINDOW_SIZE; i++) {
        sum_sq_horiz += acc_mag_horiz[i] * acc_mag_horiz[i];
    }
    features[C2_SUM_VECTOR_MAGNITUDE_HORIZONTAL] = sqrtf(sum_sq_horiz / EFFECTIVE_WINDOW_SIZE);
    
    // C4: Z轴与竖直方向夹角
    features[C4_ANGLE_Z_AXIS_VERTICAL] = atan2f(sqrtf(mean_ax*mean_ax + mean_az*mean_az), -mean_ay);
    
    // C5: 躯干方向角度标准差
    float sum_angles = 0, sum_sq_angles = 0;
    for (int i = 0; i < EFFECTIVE_WINDOW_SIZE; i++) {
        int idx = get_buffer_index(i, window_head);
        float ay_val = window[idx].ay;
        float angle = (ay_val != 0) ? atanf(acc_mag_horiz[i] / ay_val) : M_PI_2;
        sum_angles += angle;
        sum_sq_angles += angle * angle;
    }
    float mean_angle = sum_angles / EFFECTIVE_WINDOW_SIZE;
    float var_angle = (sum_sq_angles / EFFECTIVE_WINDOW_SIZE) - (mean_angle * mean_angle);
    features[C5_TRUNK_ORIENTATION] = sqrtf(var_angle > 0 ? var_angle : 0);
    
    // C9: 三轴标准差模
    features[C9_STD_MAGNITUDE] = sqrtf(std_ax*std_ax + std_ay*std_ay + std_az*std_az);
    
    // C11: 水平面信号模面积 (使用梯形积分)
    float integral_x = 0, integral_z = 0;
    for (int i = 0; i < EFFECTIVE_WINDOW_SIZE; i++) {
        int idx = get_buffer_index(i, window_head);
        integral_x += fabsf(window[idx].ax);
        integral_z += fabsf(window[idx].az);
    }
    features[C11_SIGNAL_MAGNITUDE_AREA_HORIZONTAL] = (integral_x + integral_z) * dt / EFFECTIVE_WINDOW_SIZE;
    
    // C13: 水平面活动信号模面积
    float sum_horiz_mag = 0;
    for (int i = 0; i < EFFECTIVE_WINDOW_SIZE; i++) {
        sum_horiz_mag += acc_mag_horiz[i];
    }
    features[C13_ACTIVITY_SIGNAL_MAGNITUDE_AREA_HORIZONTAL] = sum_horiz_mag / EFFECTIVE_WINDOW_SIZE;
    
    // C14: 近似速度
    float vx = sum_ax * dt;
    float vz = sum_az * dt;
    features[C14_VELOCITY_APPROX] = sqrtf(vx*vx + vz*vz) / EFFECTIVE_WINDOW_SIZE;
    
    // 过零点计算 (直接从循环缓冲区计算)
    features[ACC_X_ZERO_CROSS] = (float)calc_zero_crossings_circular(window, window_head, EFFECTIVE_WINDOW_SIZE, 0);
    
    // 峰度计算
    features[ACC_MAG_KURTOSIS] = calc_kurtosis(acc_mag, EFFECTIVE_WINDOW_SIZE, mean_acc_mag, std_acc_mag);
}