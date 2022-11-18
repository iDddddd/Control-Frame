//
// Created by David9686 on 2022/11/18.
//

#ifndef RM_FRAME_C_IMU_H
#define RM_FRAME_C_IMU_H

#include "Device.h"

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;

typedef struct
{
    float ax;
    float ay;
    float az;

    float mx;
    float my;
    float mz;

   // float temp;

    float gx;
    float gy;
    float gz;

    float ax_offset;
    float ay_offset;
    float az_offset;

    float gx_offset;
    float gy_offset;
    float gz_offset;
} imu_t;

imu_t imu_data;


void delay_us(uint16_t us);



class IMU{
    private:



    public:
    imu_t imu;


    uint8_t ist8310_init(void);
    uint8_t ist8310_read_one(uint8_t reg);
    void ist8310_write_one(uint8_t reg,uint8_t data);
    void ist8310_read_mul(uint8_t reg,uint8_t *buf,uint8_t len);
    void ist8310_write_mul(uint8_t reg,uint8_t *data,uint8_t len);
    void ist8310_update();

    void BMI088_init();
    void bmi088_read(float accelerometer[3],float gyro[3]);
    void BMI088_update();
    void zero_offset();

    void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
    void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void get_angle(float q[4],float *yaw,float *pitch,float *roll);


    float invSqrt(float x);
    void IMU_Init();
    void IMU_Handle();








};

#endif //RM_FRAME_C_IMU_H
