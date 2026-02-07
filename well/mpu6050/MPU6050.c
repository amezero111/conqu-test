#include "MPU6050_Reg.h"
#include "MPU6050.h"
#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址
// 原始传感器数据
int16_t AX, AY, AZ, GX, GY, GZ;
// 转换后物理量
float ax, ay, az;
float gx, gy, gz;
// 姿态角
float Yaw = 0.0f;
float Yaw_Gyro = 0.0f;
float Yaw_Acc = 0.0f;
// 陀螺仪零漂偏移
float Gx_Offset = 0.0f, Gy_Offset = 0.0f, Gz_Offset = 0.0f;
/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
  HAL_I2C_Mem_Write(&hi2c_MPU6050, MPU6050_ADDRESS, RegAddress, 1, &Data, 1, 10000);
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */ 
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	HAL_I2C_Mem_Read(&hi2c_MPU6050, MPU6050_ADDRESS, RegAddress, 1, &Data, 1, 10000);
	
	return Data;
}

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
void MPU6050_Init(void)
{
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}

/**
  * 函    数：MPU6050获取ID号
  * 参    数：无
  * 返 回 值：MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}
// 陀螺仪零漂校准
void MPU6050_Calibrate(void)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint16_t i;

    // 采集500次静止数据求平均
    for(i = 0; i < 500; i++)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        gx_sum += GX;
        gy_sum += GY;
        gz_sum += GZ;
        HAL_Delay(1);
    }

    Gx_Offset = (float)gx_sum / 500.0f;
    Gy_Offset = (float)gy_sum / 500.0f;
    Gz_Offset = (float)gz_sum / 500.0f;
}

// 原始数据转换为物理单位
void MPU6050_ConvertData(int16_t AX, int16_t AY, int16_t AZ, int16_t GX, int16_t GY, int16_t GZ)
{
    // 加速度计转换
    ax = (float)AX * ACC_SENSITIVITY;
    ay = (float)AY * ACC_SENSITIVITY;
    az = (float)AZ * ACC_SENSITIVITY;

    // 陀螺仪转换（去除零漂）
    gx = (float)(GX - Gx_Offset) * GYRO_SENSITIVITY;
    gy = (float)(GY - Gy_Offset) * GYRO_SENSITIVITY;
    gz = (float)(GZ - Gz_Offset) * GYRO_SENSITIVITY;
}

// 互补滤波解算偏航角
void MPU6050_CalcYaw_Complementary(void)
{
    // 1. 获取原始数据
    MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
    // 2. 数据单位转换
    MPU6050_ConvertData(AX, AY, AZ, GX, GY, GZ);

    // 3. 陀螺仪积分计算偏航角
    Yaw_Gyro += gz * SAMPLING_TIME;

    // 4. 静止状态下，加速度计校准偏航角
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    if(fabsf(acc_mag - 1.0f) < 0.1f)
    {
        Yaw_Acc = atan2f(ay, ax) * 180.0f / M_PI;
    }

    // 5. 互补滤波融合
    Yaw = (ALPHA * Yaw_Gyro + (1.0f - ALPHA) * Yaw_Acc)*1.384f;

    // 6. 角度限幅（-180° ~ 180°）
    if(Yaw > 180.0f)
        Yaw -= 360.0f;
    if(Yaw < -180.0f)
        Yaw += 360.0f;
}
