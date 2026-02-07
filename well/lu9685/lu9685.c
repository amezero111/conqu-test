#include "lu9685.h"

void LU9685_SoftReset(UART_HandleTypeDef *huart)
{
    uint8_t cmd[] = {LU9685_CMD_HEAD, LU9685_ADDR, 0xFB, 0xFB, LU9685_CMD_TAIL};
    HAL_UART_Transmit(huart, cmd, sizeof(cmd), 100);
}

void LU9685_SingleServoCtrl(UART_HandleTypeDef *huart, uint8_t channel, uint8_t angle)
{
    // 通道范围 0-19，角度 0-180，大于200为关闭输出
    if(channel > 19) return;
    uint8_t cmd[] = {LU9685_CMD_HEAD, LU9685_ADDR, channel, angle, LU9685_CMD_TAIL};
    HAL_UART_Transmit(huart, cmd, sizeof(cmd), 100);
}

void LU9685_AllServoCtrl(UART_HandleTypeDef *huart, uint8_t angles[20])
{
    uint8_t cmd[25] = {LU9685_CMD_HEAD, LU9685_ADDR, 0XFD};
    for(int i=0; i<20; i++)
    {
        cmd[3+i] = angles[i];
    }
    cmd[23] = LU9685_CMD_TAIL;
    HAL_UART_Transmit(huart, cmd, sizeof(cmd), 200);
}
void first_arm(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +1000;
 uint8_t all_angles[20];
	 for(int i=0; i<20; i++) all_angles[i] =200;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=40;
	all_angles[0]=180;
	all_angles[8] =50;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
	start_tick = osKernelGetTickCount();
   end_tick = start_tick +1000;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=40;
	all_angles[0]=180;
	all_angles[8] =30;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +1000;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=50;
	all_angles[0]=150;
	all_angles[8] =30;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
}
void second_arm(void){
	uint32_t start_tick = osKernelGetTickCount();
  uint32_t end_tick = start_tick +1000;
 uint8_t all_angles[20];
	 for(int i=0; i<20; i++) all_angles[i] =200;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=50;
	all_angles[0]=150;
	all_angles[8] =30;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
	start_tick = osKernelGetTickCount();
   end_tick = start_tick +1000;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=40;
	all_angles[0]=180;
	all_angles[8] =30;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
  start_tick = osKernelGetTickCount();
   end_tick = start_tick +1000;
  while (osKernelGetTickCount() < end_tick)
  {
   all_angles[1]=40;
	all_angles[0]=180;
	all_angles[8] =50;
	 LU9685_AllServoCtrl(&huart1, all_angles);
    osDelay(1);
  }
}