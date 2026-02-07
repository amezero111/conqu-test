#include "motor.h"
#include "MPU6050.h"
/************************* 全局变量定义 *************************/
PID_TypeDef pidLeft, pidRight,pidYaw;
float leftSpeed_rpm = 0.0f, rightSpeed_rpm = 0.0f;
float leftTargetRpm = 0.0f, rightTargetRpm = 0.0f;
int leftPwm = 0, rightPwm = 0;
float leftCompensatedRpm = 0.0f;             // 补偿后左转速
float rightCompensatedRpm = 0.0f;            // 补偿后右转速
/************************* 函数实现 *************************/
// PID初始化
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float outputMax, float outputMin,
              float integralSepThreshold, float integralLimit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->outputMax = outputMax;
    pid->outputMin = outputMin;
    pid->integralSepThreshold = integralSepThreshold;
    pid->integralLimit = integralLimit;
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

// PID计算（位置式）
float PID_Calculate(PID_TypeDef *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->error = target - actual;

    // 积分分离 + 独立积分限幅
    if (fabsf(pid->error) < pid->integralSepThreshold)
    {
        // 小偏差：开启积分，且积分限幅
        pid->integral += pid->error * SAMPLING_PERIOD;
        // 双向积分限幅
        if (pid->integral > pid->integralLimit)
        {
            pid->integral = pid->integralLimit;
        }
        else if (pid->integral < -pid->integralLimit)
        {
            pid->integral = -pid->integralLimit;
        }
    }
    else
    {
        // 大偏差：关闭积分，清零积分项
        pid->integral = 0.0f;
    }

    // 微分计算
    float derivative = (pid->error - pid->lastError) / SAMPLING_PERIOD;
    // PID输出计算
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (pid->output > pid->outputMax)
        pid->output = pid->outputMax;
    if (pid->output < pid->outputMin)
        pid->output = pid->outputMin;

    pid->lastError = pid->error;
    return pid->output;
}

// 获取编码器计数差值
int getTIMx_DetaCnt(TIM_HandleTypeDef *htim)
{
    int cnt;
    cnt = htim->Instance->CNT - 0x7FFF;
    htim->Instance->CNT = 0x7FFF;
    return cnt;
}

// 计算电机转速
void Calculate_Motor_Speed(void)
{
    int left_pulse = getTIMx_DetaCnt(&htim2);
    int right_pulse = getTIMx_DetaCnt(&htim4);

    leftSpeed_rpm = ((float)left_pulse / ENCODER_PPR) / SAMPLING_PERIOD * SEC_PER_MIN;
    rightSpeed_rpm = ((float)right_pulse / ENCODER_PPR) / SAMPLING_PERIOD * SEC_PER_MIN;
}

// 设置PWM占空比
void Set_PWM_Duty(uint32_t ch, int duty)
{
    // 限幅：防止超出PWM分辨率范围
    if (duty < 0)
        duty = 0;
    if (duty > PWM_ARR)
        duty = PWM_ARR;

    // 设置CCR寄存器值
    __HAL_TIM_SET_COMPARE(&htim1, ch, duty);
}
//走直线
static float Normalize_Angle(float angle)
{
    while (angle > ANGLE_MAX)  angle -= 360.0f;
    while (angle < ANGLE_MIN)  angle += 360.0f;
    return angle;
}
static float Calculate_Angle_Error(float target, float current)
{
    float error = target - current;
    // 处理角度环绕，取最短旋转方向误差
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}
void MotorSpeed_CompensateByYaw(float leftTargetRpm, float rightTargetRpm,float Yaw_Target )
{
    // 1. 计算最优角度误差（解决跨180°/-180°跳变问题）
    float angle_error = Calculate_Angle_Error(Yaw_Target, Yaw);

    // 2. 航向PID计算：使用角度误差输出补偿值
    float yawError_Comp = PID_Calculate(&pidYaw, 0.0f, angle_error);

    // 3. 核心补偿逻辑
    leftCompensatedRpm  = leftTargetRpm - yawError_Comp;
    rightCompensatedRpm = rightTargetRpm + yawError_Comp;

    // 4. 转速限幅保护
    if (leftCompensatedRpm < MIN_MOTOR_RPM)
        leftCompensatedRpm = MIN_MOTOR_RPM;
    else if (leftCompensatedRpm > MAX_MOTOR_RPM)
        leftCompensatedRpm = MAX_MOTOR_RPM;

    if (rightCompensatedRpm < MIN_MOTOR_RPM)
        rightCompensatedRpm = MIN_MOTOR_RPM;
    else if (rightCompensatedRpm > MAX_MOTOR_RPM)
        rightCompensatedRpm = MAX_MOTOR_RPM;
	 // 1. 读取实际转速
    Calculate_Motor_Speed();
    // 2. PID计算
    leftPwm = (int)PID_Calculate(&pidLeft, leftCompensatedRpm, leftSpeed_rpm);
    rightPwm = (int)PID_Calculate(&pidRight, rightCompensatedRpm, rightSpeed_rpm);
    Set_PWM_Duty(TIM_CHANNEL_1, leftPwm);
    Set_PWM_Duty(TIM_CHANNEL_2, rightPwm);
	
}
void first_road(void){
  uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +400;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 0);
    // 延时1ms，极小阻塞，既让出CPU，又不影响执行频率
    osDelay(1);
	  start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -10);
    // 延时1ms，极小阻塞，既让出CPU，又不影响执行频率
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -20);
    // 延时1ms，极小阻塞，既让出CPU，又不影响执行频率
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -40);
    // 延时1ms，极小阻塞，既让出CPU，又不影响执行频率
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -80);
    osDelay(1);
  }
  }
}
//2
void second_road(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +1000;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -80);
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +900;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -45);
    osDelay(1);
  }
 start_tick = osKernelGetTickCount();
   end_tick = start_tick +400;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 0);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +500;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm,30);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +500;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 60);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +2400;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 0);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +500;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -45);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +700;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -90);
    osDelay(1);
  }
}
//3
void third_road(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +500;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -135);
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +6000;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -180);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 135);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +2500;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 90);
    osDelay(1);
  }
  
}
void forth_road(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +200;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 45);
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +4000;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 0);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +600;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -45);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +1500;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -90);
    osDelay(1);
  }
  
}
void fifth_road(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +400;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, -45);
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +1800;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 0);
    osDelay(1);
  }
  
  
}
void sixth_road(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +2800;

  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 90);
    osDelay(1);
  }
   start_tick = osKernelGetTickCount();
   end_tick = start_tick +800;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 135);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +6000;
  while (osKernelGetTickCount() < end_tick)
  {
    MotorSpeed_CompensateByYaw(leftTargetRpm, rightTargetRpm, 180);
    osDelay(1);
  }
  
}
void stop_motor(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +4000;

  while (osKernelGetTickCount() < end_tick)
  {
    Set_PWM_Duty(TIM_CHANNEL_1, 0);
    Set_PWM_Duty(TIM_CHANNEL_2, 0);
    osDelay(1);
  }
	
	
}