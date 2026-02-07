#ifndef __LU9685_H
#define __LU9685_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
// 指令头和尾
#define LU9685_CMD_HEAD    0xFA
#define LU9685_CMD_TAIL    0xFE
#define LU9685_ADDR        0x00

// 软复位指令
void LU9685_SoftReset(UART_HandleTypeDef *huart);

// 单路舵机控制
void LU9685_SingleServoCtrl(UART_HandleTypeDef *huart, uint8_t channel, uint8_t angle);

// 20路舵机同步控制
void LU9685_AllServoCtrl(UART_HandleTypeDef *huart, uint8_t angles[20]);
void first_arm(void);
void second_arm(void);
void stop_motor(void);
#endif