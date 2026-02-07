#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"                  // Device header
#include<math.h>



extern I2C_HandleTypeDef hi2c1;
#define hi2c_MPU6050 hi2c1


void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/************************* 宏定义 *************************/
#define M_PI        3.1415926f
#define SAMPLING_TIME   0.01f       // 采样周期10ms
#define ALPHA       0.98f           // 互补滤波系数
// 量程转换系数
#define ACC_SENSITIVITY 4.882812e-4f  // ±8g加速度计转换系数
#define GYRO_SENSITIVITY 6.1035e-2f   // ±500°/s陀螺仪转换系数

/************************* 外部全局变量声明 *************************/
// 原始数据
extern int16_t AX, AY, AZ, GX, GY, GZ;
// 转换后物理量
extern float ax, ay, az;    // 加速度(g)
extern float gx, gy, gz;    // 陀螺仪(°/s)
// 姿态角
extern float Yaw;           // 融合后偏航角
extern float Yaw_Gyro;      // 陀螺仪积分偏航角
extern float Yaw_Acc;       // 加速度计解算偏航角
// 校准偏移值
extern float Gx_Offset, Gy_Offset, Gz_Offset;

/************************* 函数声明 *************************/
/**
 * @brief  MPU6050陀螺仪零漂校准
 * @param  无
 * @retval 无
 */
void MPU6050_Calibrate(void);

/**
 * @brief  MPU6050原始数据转换为物理单位
 * @param  AX/AY/AZ: 加速度原始数据
 * @param  GX/GY/GZ: 陀螺仪原始数据
 * @retval 无
 */
void MPU6050_ConvertData(int16_t AX, int16_t AY, int16_t AZ, int16_t GX, int16_t GY, int16_t GZ);

/**
 * @brief  互补滤波解算偏航角
 * @param  无
 * @retval 无
 */
void MPU6050_CalcYaw_Complementary(void);
#endif
