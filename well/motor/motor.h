#ifndef __MOTOR_H
#define __MOTOR_H



/* 包含依赖的头文件 */
#include "main.h"
#include "tim.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
/************************* 宏定义 *************************/
#define ENCODER_PPR      2496    // MC130电机PPR
#define SAMPLING_PERIOD  0.01f   // 10ms采样周期
#define SEC_PER_MIN      60.0f   // 分钟转秒系数
#define PWM_ARR          1000    // PWM分辨率
// 积分分离阈值（可根据实际调试调整，建议为目标转速的10%~20%）
#define INTEGRAL_SEP_THRESHOLD 20.0f
// 积分限幅值（独立于输出限幅，建议为输出最大值的50%）
#define INTEGRAL_LIMIT         500.0f
#define MAX_MOTOR_RPM  100.0f
#define MIN_MOTOR_RPM  0.0f
// 角度范围定义
#define ANGLE_MAX  180.0f
#define ANGLE_MIN  -180.0f
/************************* PID结构体定义 *************************/
typedef struct
{
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float kd;                 // 微分系数
    float outputMax;          // 输出最大值
    float outputMin;          // 输出最小值
    float integralSepThreshold;// 积分分离阈值
    float integralLimit;      // 积分项限幅值

    float target;             // 目标值
    float actual;             // 实际值
    float error;              // 当前误差
    float lastError;          // 上一时刻误差
    float integral;           // 积分累加值
    float output;             // PID输出值
} PID_TypeDef;

/************************* 外部全局变量声明 *************************/
extern PID_TypeDef pidLeft, pidRight;
extern float leftSpeed_rpm, rightSpeed_rpm;
extern float leftTargetRpm, rightTargetRpm;
extern int leftPwm, rightPwm;                             // 当前实测偏航角
extern PID_TypeDef pidYaw;                     // 航向PID结构体
/************************* 函数声明 *************************/
/**
 * @brief  PID参数初始化
 * @param  pid: PID结构体指针
 * @param  kp/ki/kd: PID三个系数
 * @param  outputMax/outputMin: 输出限幅
 * @param  integralSepThreshold: 积分分离阈值
 * @param  integralLimit: 积分限幅
 * @retval 无
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float outputMax, float outputMin,
              float integralSepThreshold, float integralLimit);

/**
 * @brief  位置式PID计算函数
 * @param  pid: PID结构体指针
 * @param  target: 目标转速
 * @param  actual: 实际转速
 * @retval PID计算输出值
 */
float PID_Calculate(PID_TypeDef *pid, float target, float actual);

/**
 * @brief  获取编码器计数差值
 * @param  htim: 定时器句柄
 * @retval 编码器脉冲差值
 */
int getTIMx_DetaCnt(TIM_HandleTypeDef *htim);

/**
 * @brief  计算电机实时转速（单位：rpm）
 * @param  无
 * @retval 无
 */
void Calculate_Motor_Speed(void);

/**
 * @brief  设置PWM占空比
 * @param  ch: PWM通道
 * @param  duty: 占空比数值
 * @retval 无
 */
void Set_PWM_Duty(uint32_t ch, int duty);

/**
 * @brief  定时器中断回调函数（重定义HAL库弱函数）
 * @param  htim: 定时器句柄
 * @retval 无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MotorSpeed_CompensateByYaw(float leftTargetRpm, float rightTargetRpm,float Yaw_Target );
void first_road(void);
void second_road(void);
void third_road(void);
void forth_road(void);
void fifth_road(void);
void sixth_road(void);

#endif /* __MOTOR_CONTROL_H */