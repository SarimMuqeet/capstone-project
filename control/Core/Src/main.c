/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//servo board driver
#include "pca9685.h"
//for IK
#include <math.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
//#include <iostream>
//#include <cstring>
//#include <cmath>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define constants for the links inverse kinematics
#define L12 15.0  // Length of link 1
#define L23 15.0   // Length of link 2
#define L34 8.5   // Length of link 3

#define STEPS_PER_REVS 4096 //motor and step type defined
#define HALF_STEPS 8
#define MIN_TO_US (60*1000000)
#define NUM_SEQ (STEPS_PER_REVS/HALF_STEPS)

//define pins for first stepper
// ---- for synchronized step
#define M1_IN1_PIN GPIO_PIN_3
#define M1_IN1_PORT GPIOE
#define M1_IN2_PIN GPIO_PIN_4
#define M1_IN2_PORT GPIOE
#define M1_IN3_PIN GPIO_PIN_5
#define M1_IN3_PORT GPIOE
#define M1_IN4_PIN GPIO_PIN_6
#define M1_IN4_PORT GPIOE
int stepNumber;          // which step the motor is on

//define pins for second stepper
#define M2_IN1_PIN GPIO_PIN_5
#define M2_IN1_PORT GPIOA
#define M2_IN2_PIN GPIO_PIN_6
#define M2_IN2_PORT GPIOA
#define M2_IN3_PIN GPIO_PIN_5
#define M2_IN3_PORT GPIOB
#define M2_IN4_PIN GPIO_PIN_6
#define M2_IN4_PORT GPIOB

#define TOTAL_REV_ANGLE 360

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for parsingTask */
osThreadId_t parsingTaskHandle;
const osThreadAttr_t parsingTask_attributes = {
  .name = "parsingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for gripperTask */
osThreadId_t gripperTaskHandle;
const osThreadAttr_t gripperTask_attributes = {
  .name = "gripperTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for step1Task */
osThreadId_t step1TaskHandle;
const osThreadAttr_t step1Task_attributes = {
  .name = "step1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for step2Task */
osThreadId_t step2TaskHandle;
const osThreadAttr_t step2Task_attributes = {
  .name = "step2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

//for handling UART and StartArmTask synchronization
osSemaphoreId_t armTaskSemaphore;
//semaphore for stepper motor control
osSemaphoreId_t stepperTask1Semaphore;
osSemaphoreId_t stepperTask2Semaphore;

//buffer to store UART messages
uint8_t uartRxBuffer[17];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void StartParsingTask(void *argument);
void StartArmTask(void *argument);
void StartGripperTask(void *argument);
void StartStep1Task(void *argument);
void startStep2Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to calculate inverse kinematics
bool ikinematics(float positionx, float positiony, float gamma, float *J1, float *J2, float *J3) {
    float xe = positionx;
    float ye = positiony;
    float g = gamma;

    // Calculate the position P3
    float x3 = xe - (L34 * cosf(g * M_PI / 180.0));  // Convert gamma to radians
    float y3 = ye - (L34 * sinf(g * M_PI / 180.0));
    float C = sqrtf(x3 * x3 + y3 * y3);

    if ((L12 + L23) > C) {
        // Calculate angles a and B
        float a = acosf((L12 * L12 + L23 * L23 - C * C) / (2 * L12 * L23));
        float B = acosf((L12 * L12 + C * C - L23 * L23) / (2 * L12 * C));

        // Joint angles for elbow-down configuration
        float J1a = ((atan2f(y3, x3) - B)* 180.0 / M_PI);
        float J2a = 180.0 - (a * 180.0 / M_PI);
        float J3a = g - J1a - J2a;

        // Joint angles for elbow-up configuration
        float J1b = ((atan2f(y3, x3) + B)* 180.0 / M_PI);
        float J2b = -(180.0 - (a * 180.0 / M_PI));
        float J3b = g - J1b - J2b;

        // Output the results (for example, printing to the serial monitor)
//        printf("The joint 1, 2, and 3 angles are (%f, %f, %f) respectively for elbow-down configuration.\n", J1a, J2a, J3a);
//        printf("The joint 1, 2, and 3 angles are (%f, %f, %f) respectively for elbow-up configuration.\n", J1b, J2b, J3b);

        //return A-Configuration for now
        *J1 = J1a;
        *J2 = J2a;
        *J3 = J3a;

        //return success
        return true;

    } else {
//        printf("     Dimension error!\n");
//        printf("     End-effector is outside the workspace.\n");
    	return false;
    }
}


//delay function for stepper motor control
void delay(uint16_t us)
{
	//init counter to 0, wait for it to reach designated val,
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1) < us);

}

//functions as delay (lower delay = faster, higher delay = slower)
//min speed 1 rpm, max speed ~13 rpm, want to go from microseconds/step (us) to rev/min
void set_rpm(int rpm)
{
//	delay(MIN_TO_US/(STEPS_PER_REVS*rpm));

//	osDelay(MIN_TO_US/(STEPS_PER_REVS*rpm));
//	osDelay(pdMS_TO_TICKS(500));
//	osDelay(MIN_TO_US / (STEPS_PER_REVS * rpm) / 1000); // Convert µs to ms

	uint32_t delay_us = (60 * 1000000) / (STEPS_PER_REVS * rpm);
	delay(delay_us); // Use microsecond-precision delay
}

void motorDelay(uint32_t delay)
{
//    __HAL_TIM_SET_COUNTER(&htim1, 0);
//    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
	  //works for synchro between motors
      osDelay(pdMS_TO_TICKS(delay / 1000));  // Convert µs to RTOS ticks
}

void motor2Delay(uint32_t delay)
{
//    __HAL_TIM_SET_COUNTER(&htim4, 0);
//    while (__HAL_TIM_GET_COUNTER(&htim4) < delay);
      osDelay(pdMS_TO_TICKS(delay / 1000));  // Convert µs to RTOS ticks
}


void motorOff()
{
    // Switch off the idle current to the motor
    // Otherwise L298N module will heat up
    HAL_GPIO_WritePin(M1_IN1_PORT, M1_IN1_PIN, GPIO_PIN_RESET); // IN1
    HAL_GPIO_WritePin(M1_IN2_PORT, M1_IN2_PIN, GPIO_PIN_RESET); // IN2
    HAL_GPIO_WritePin(M1_IN3_PORT, M1_IN3_PIN, GPIO_PIN_RESET); // IN3
    HAL_GPIO_WritePin(M1_IN4_PORT, M1_IN4_PIN, GPIO_PIN_RESET); // IN4

    HAL_GPIO_WritePin(M2_IN1_PORT, M2_IN1_PIN, GPIO_PIN_RESET); // IN1
    HAL_GPIO_WritePin(M2_IN2_PORT, M2_IN2_PIN, GPIO_PIN_RESET); // IN2
    HAL_GPIO_WritePin(M2_IN3_PORT, M2_IN3_PIN, GPIO_PIN_RESET); // IN3
    HAL_GPIO_WritePin(M2_IN4_PORT, M2_IN4_PIN, GPIO_PIN_RESET); // IN4
}

void stepCCV (int steps, uint16_t delay, GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN, GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, GPIO_TypeDef* IN3_PORT, uint16_t IN3_PIN, GPIO_TypeDef* IN4_PORT, uint16_t IN4_PIN)
{
//	for (int x = 0; x < steps; x++) {
//		switch (x % 4) {
//			case 0: // Energize IN4
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 1: // Energize IN3
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 2: // Energize IN2
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
//				break;
//			case 3: // Energize IN1
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//		}
//		motorDelay(delay); // Delay between steps
//	}
	for (int x = 0; x < steps; x++) {
		switch (x % 4) {
			case 0: // Energize IN4
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 1: // Energize IN3
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 2: // Energize IN2
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 3: // Energize IN1
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
		}
		motorDelay(delay); // Delay between steps
	}

}

void stepCV (int steps, uint16_t delay, GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN, GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, GPIO_TypeDef* IN3_PORT, uint16_t IN3_PIN, GPIO_TypeDef* IN4_PORT, uint16_t IN4_PIN)
{
//    for (int x = 0; x < steps; x++) {
//        switch (x % 4) {
//            case 0: // Energize IN1
//                HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
//                HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//                break;
//            case 1: // Energize IN2
//                HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
//                break;
//            case 2: // Energize IN3
//                HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
//                HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//                break;
//            case 3: // Energize IN4
//                HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//                HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
//                HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//                break;
//        }
//        motorDelay(delay); // Delay between steps
//    }

	//working
//	for (int x = 0; x < steps; x++) {
//		switch (x % 4) {
//			case 0: // Energize IN4
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 1: // Energize IN3
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 2: // Energize IN2
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
//				break;
//			case 3: // Energize IN1
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//		}
//		motorDelay(delay); // Delay between steps
//	}

	for (int x = 0; x < steps; x++) {
		switch (x % 4) {
			case 0: // Energize IN4
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 1: // Energize IN3
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 2: // Energize IN2
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 3: // Energize IN1
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
		}
		motorDelay(delay); // Delay between steps
	}
}




void step2CCV (int steps, uint16_t delay, GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN, GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, GPIO_TypeDef* IN3_PORT, uint16_t IN3_PIN, GPIO_TypeDef* IN4_PORT, uint16_t IN4_PIN)
{

	for (int x = 0; x < steps; x++) {
		switch (x % 4) {
			case 0: // Energize IN4
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 1: // Energize IN3
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 2: // Energize IN2
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 3: // Energize IN1
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
		}
		motor2Delay(delay); // Delay between steps
	}

}

void step2CV (int steps, uint16_t delay, GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN, GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, GPIO_TypeDef* IN3_PORT, uint16_t IN3_PIN, GPIO_TypeDef* IN4_PORT, uint16_t IN4_PIN)
{
//	for (int x = 0; x < steps; x++) {
//		switch (x % 4) {
//			case 0: // Energize IN4
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 1: // Energize IN3
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//			case 2: // Energize IN2
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
//				break;
//			case 3: // Energize IN1
//				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
//				break;
//		}
//		motor2Delay(delay); // Delay between steps
//	}


	for (int x = 0; x < steps; x++) {
		switch (x % 4) {
			case 0: // Energize IN4
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 1: // Energize IN3
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
				break;
			case 2: // Energize IN2
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
			case 3: // Energize IN1
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
				break;
		}
		motor2Delay(delay); // Delay between steps
	}
}


/*
//controlling motor with angle specified
void step_angle_control(float angle, int direction, int rpm)
{
	//rotating 360 degrees in 512 sequences of 8 steps each
	float anglesPerSeq = TOTAL_REV_ANGLE/NUM_SEQ;

	int numSeq = (int)(angle/anglesPerSeq);

	//direction control
	for (int seq=0; seq<numSeq; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=7; step>=0; step--) {
				half_stepper_control(step);
				set_rpm(rpm);
			}
		}
		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<=7; step++)
			{
				half_stepper_control(step);
				set_rpm(rpm);
			}
		}
	}
}
*/

/*
//keep track of current stepper angle
float currAngle = 0;
void Stepper_rotate (int angle, int rpm)
{
	int deltaAngle = 0;
	deltaAngle = angle-currAngle;  // calculate the angle by which the motor needed to be rotated
	if (deltaAngle > 0.71)  // CW
	{
		step_angle_control(deltaAngle, 0, rpm);
		currAngle = angle;  // save the angle as current angle
	}
	else if (deltaAngle < 0.71) // CCW
	{
		deltaAngle = -(deltaAngle);
		step_angle_control(deltaAngle, 1, rpm);
		currAngle = angle;
	}
}
*/





/* UART Interrupt Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if(huart->ErrorCode == HAL_UART_ERROR_NONE) {
        	//release arm task
            osSemaphoreRelease(armTaskSemaphore);
        }
        //potentially resetting UART reception after buffer has been cleared
        HAL_UART_Receive_IT(&huart2, uartRxBuffer, sizeof(uartRxBuffer));
    }
}

//clear buffer after every input
void clearBuffer(uint8_t *buffer, size_t size) {
	memset(buffer, 0, size);
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //start timer
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim4);

  //enable UART interrupt to receive messages
//  uint8_t uartRxBuffer[30];
  HAL_UART_Receive_IT(&huart2, uartRxBuffer, sizeof(uartRxBuffer));

  //enable encoder mode timer
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  //global semaphore, used for UART and StartArmTask Synchronization
  armTaskSemaphore = osSemaphoreNew(1, 0, NULL);
  //semaphore used for stepper tasks control
  stepperTask1Semaphore = osSemaphoreNew(1, 0, NULL);
  stepperTask2Semaphore = osSemaphoreNew(1, 0, NULL);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of parsingTask */
  parsingTaskHandle = osThreadNew(StartParsingTask, NULL, &parsingTask_attributes);

  /* creation of armTask */
  armTaskHandle = osThreadNew(StartArmTask, NULL, &armTask_attributes);

  /* creation of gripperTask */
  gripperTaskHandle = osThreadNew(StartGripperTask, NULL, &gripperTask_attributes);

  /* creation of step1Task */
  step1TaskHandle = osThreadNew(StartStep1Task, NULL, &step1Task_attributes);

  /* creation of step2Task */
  step2TaskHandle = osThreadNew(startStep2Task, NULL, &step2Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //freeRTOS tasks used instead
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  //for stepper encoder ----------------
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6
                           LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin PB5 PB6 */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartParsingTask */
/**
* @brief Function implementing the parsingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParsingTask */
void StartParsingTask(void *argument)
{
  /* USER CODE BEGIN StartParsingTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartParsingTask */
}

/* USER CODE BEGIN Header_StartArmTask */
/**
* @brief Function implementing the armTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartArmTask */
void StartArmTask(void *argument)
{
  /* USER CODE BEGIN StartArmTask */

	int status = PCA9685_Init(&hi2c1);
	uint8_t ActiveServo = 15;

	// for stepper control
	float targetAngle = 360.0f; //arbitrary for now
	float currStepperAngle = 0.0f;
	int stepperRPM = 15;

	//angles to track inverse kinematics func return
	float j1_angle, j2_angle, j3_angle;



//	osSemaphoreRelease(armTaskSemaphore);


	for(;;) {
		char str[64] = {0};
		snprintf(str, sizeof(str), "\nWaiting for Semaphore (UART interrupt)\n");
		//use non blocking UART Transmit instead of regular (which blocks)
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)str, sizeof (str));

		//Acquire Semaphore (Wait for UART Signal)
//		osSemaphoreAcquire(armTaskSemaphore, osWaitForever); // COMMENT WHEN TESTING MOTORS BY THEMSELVES, UNCOMMENT WHEN WITH NANO


		///*

		//working grab water bottle and go up!!

//		PCA9685_SetServoAngle(12, 20);  // Set to 0°
//
//		//--- New Stepper Control Section ---//
//	    // Release both stepper semaphores simultaneously to complete stepper tasks
//	    osSemaphoreRelease(stepperTask1Semaphore);
//	    osSemaphoreRelease(stepperTask2Semaphore);
//
//		//wait until stepper tasks completed
//	    osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//	    osSemaphoreAcquire(armTaskSemaphore, osWaitForever);


//		osSemaphoreAcquire(armTaskSemaphore, osWaitForever);

		PCA9685_SetServoAngle_DS3225(15, 90);  // Set to 0°
		PCA9685_SetServoAngle_DS3225(14, 180);  // Set to 0°
		PCA9685_SetServoAngle(13, 90);  // Set to 0°
		PCA9685_SetServoAngle(12, 80);
		osDelay(pdMS_TO_TICKS(2000));

		PCA9685_SetServoAngle(12, 20);

		//potential small delay before gripper
		osDelay(pdMS_TO_TICKS(800));

		PCA9685_SetServoAngle_DS3225(15, 55);  // Set to 0°
		osDelay(pdMS_TO_TICKS(800));


		PCA9685_SetServoAngle_DS3225(14, 50);  // Set to 0°
		osDelay(pdMS_TO_TICKS(800));


		PCA9685_SetServoAngle(13, 30);
		osDelay(pdMS_TO_TICKS(3000));


		//--- New Stepper Control Section ---//
		// Release both stepper semaphores simultaneously to complete stepper tasks
		osSemaphoreRelease(stepperTask1Semaphore);
		osSemaphoreRelease(stepperTask2Semaphore);

		//wait until stepper tasks completed
		osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
		osSemaphoreAcquire(armTaskSemaphore, osWaitForever);

		//let go of object
//		PCA9685_SetServoAngle_DS3225(15, 90);  // Set to 0°
//		PCA9685_SetServoAngle_DS3225(14, 180);  // Set to 0°
//		PCA9685_SetServoAngle(13, 90);  // Set to 0°
//		PCA9685_SetServoAngle(12, 80);
//		osDelay(pdMS_TO_TICKS(2000));

//		motorOff();







		//encoder funcs -----------
//		//read encoder count (position) only relying on A+, B+
//		int16_t encoder_position = __HAL_TIM_GET_COUNTER(&htim3);
//		//to reset counter
//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//		//get current direction:
//		uwDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);


//		//TO TEST - encoder control with steppers:
//        // Read encoder position
//        encoder_position = __HAL_TIM_GET_COUNTER(&htim3);
//        //target position should be obtained from UART interrupt
//        error = target_position - encoder_position;
//
//        // Move stepper motors until target position is reached
//        while (abs(error) > ERROR_THRESHOLD) {
//            if (error > 0) {
//                // Move forward (clockwise)
//                synchronizedStep(1, 5000); // Move 1 step forward
//            } else {
//                // Move backward (counter-clockwise)
//                synchronizedStep(-1, 5000); // Move 1 step backward
//            }
//
//            // Update encoder position and error
//            encoder_position = __HAL_TIM_GET_COUNTER(&htim3);
//            error = target_position - encoder_position;
//        }
//
//        // Reset encoder counter (optional)
//        __HAL_TIM_SET_COUNTER(&htim3, 0);






		//test servo control
		//		//default values
//		PCA9685_SetServoAngle_DS3225(15, 0);  // Set to 0°
//		PCA9685_SetServoAngle_DS3225(14, 0);  // Set to 0°
//		PCA9685_SetServoAngle(13, 0);  // Set to 0°
//		PCA9685_SetServoAngle(12, 0);  // Set to 0°
//		PCA9685_SetServoAngle(0, 0);  // Set to 0°
//		osDelay(pdMS_TO_TICKS(2000));
//		PCA9685_SetServoAngle_DS3225(15, 90);  // Set to 90°
//		PCA9685_SetServoAngle_DS3225(14, 180);  // Set to 90°
//		PCA9685_SetServoAngle(13, 90);  // Set to 90°
//		PCA9685_SetServoAngle(12, 80);  // Set to 80°

		//tested values
//		PCA9685_SetServoAngle_DS3225(15, 80);  // Set to 90°
//		PCA9685_SetServoAngle_DS3225(14, 155);  // Set to 90°
//		PCA9685_SetServoAngle(13, 50);  // Set to 90°
//		PCA9685_SetServoAngle(12, 0);  // Set to 80°

//		    PCA9685_SetServoAngle(0, 80);  // Set to 0°

//		PCA9685_SetServoAngle_DS3225(15, 80);
//		PCA9685_SetServoAngle_DS3225(14, 155);
//		PCA9685_SetServoAngle(13, 50);


		 // */



//	    //uart integration ------------------------------ TO TEST
//	    // Check start and end delimiters
//		if (uartRxBuffer[0] == '!' && uartRxBuffer[14] == '#' && uartRxBuffer[15] == '#')
//		{
//			uint8_t command_type = uartRxBuffer[1];
//
//			/* Message Format: StartDelimiter(!) | Command(1B) | X(4B) | Y(4B) | Z(4B) | EndDelimiter(#)
//			* message = struct.pack('!cB3f3s',
//			* b'!', # Start delimiter
//			* command_type, # 1 byte
//			* x, y, z, # 3 floats (4 bytes each)
//			* b'##') # End delimiter (2 bytes)
//			*/
//
//
//			// Extract x, y, z from uart msg(convert from big-endian to little-endian)
//			uint32_t x_uint, y_uint, z_uint;
//			float x, y, z;
//
//			memcpy(&x_uint, &uartRxBuffer[2], 4);
//			x_uint = __REV(x_uint); // Reverse bytes for little-endian
//			memcpy(&x, &x_uint, sizeof(float));
//
//			memcpy(&y_uint, &uartRxBuffer[6], 4);
//			y_uint = __REV(y_uint);
//			memcpy(&y, &y_uint, sizeof(float));
//
//			memcpy(&z_uint, &uartRxBuffer[10], 4);
//			z_uint = __REV(z_uint);
//			memcpy(&z, &z_uint, sizeof(float));
//
//			// Debug message
//			char debugMsg[64];
//			sprintf(debugMsg, "CMD: %d, X: %.2f, Y: %.2f, Z: %.2f\n", command_type, x, y, z);
//			HAL_UART_Transmit_IT(&huart3, (uint8_t*)debugMsg, strlen(debugMsg));
//
//
//		    // Release both stepper semaphores simultaneously to complete stepper tasks
//		    osSemaphoreRelease(stepperTask1Semaphore);
//		    osSemaphoreRelease(stepperTask2Semaphore);
//
//			//wait until stepper tasks completed to continue (platform reaches expected height)
//		    osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//		    osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//
//
//			// Process the command (PICK or PLACE)
//			// Use inverse kinematics to calculate joint angles
//			float j1, j2, j3;
//			bool ik_success = ikinematics(x, y, 90.0f, &j1, &j2, &j3); // Assuming gamma is 90 degrees
//
//			if (ik_success) {
//				// Set servo angles based on IK results
//				PCA9685_SetServoAngle_DS3225(15, j1);
//				PCA9685_SetServoAngle_DS3225(14, j2);
//				PCA9685_SetServoAngle(13, j3);
//
////				//potential small delay before gripper
////				osDelay(pdMS_TO_TICKS(800));
//
//				// control gripper based on command type
//				if (command_type == 0) { // PICK - close gripper
//					PCA9685_SetServoAngle(0, 0);
//				} else { // PLACE - open gripper
//					PCA9685_SetServoAngle(0, 80);
//				}
//			} else {
//				HAL_UART_Transmit_IT(&huart3, (uint8_t*)"IK Error\n", 9);
//			}
//		} else {
//			HAL_UART_Transmit_IT(&huart3, (uint8_t*)"Invalid UART Msg\n", 14);
//		}
//
//		// clear buffer for extra safety (next reception will overwrite)
//		memset(uartRxBuffer, 0, sizeof(uartRxBuffer));







//		//harcoded gripper test
//		osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//
//		PCA9685_SetServoAngle_DS3225(15, 90);
//		PCA9685_SetServoAngle_DS3225(14, 180);
//		PCA9685_SetServoAngle(13, 90);
//		PCA9685_SetServoAngle(12, 80);
//		osDelay(pdMS_TO_TICKS(2000));
//
////		//potential small delay before gripper
////		osDelay(pdMS_TO_TICKS(800));
////		PCA9685_SetServoAngle(12, 0);
////
////		osDelay(pdMS_TO_TICKS(800));
////
////		PCA9685_SetServoAngle_DS3225(15, 55);  // Set to 0°
////		osDelay(pdMS_TO_TICKS(800));
////
////
////		PCA9685_SetServoAngle_DS3225(14, 125);  // Set to 0°
////		osDelay(pdMS_TO_TICKS(800));
////
////
////		PCA9685_SetServoAngle(13, 30);
////		osDelay(pdMS_TO_TICKS(800));







//	    //test just UART msg with stepper for now:
//		if (uartRxBuffer[0] == '!' && uartRxBuffer[14] == '#' && uartRxBuffer[15] == '#')
//		{
//			uint8_t command_type = uartRxBuffer[1];
//
//			/* Message Format: StartDelimiter(!) | Command(1B) | X(4B) | Y(4B) | Z(4B) | EndDelimiter(#)
//			* message = struct.pack('!cB3f3s',
//			* b'!', # Start delimiter
//			* command_type, # 1 byte
//			* x, y, z, # 3 floats (4 bytes each)
//			* b'##') # End delimiter (2 bytes)
//			*/
//
//
//			// Extract x, y, z from uart msg(convert from big-endian to little-endian)
//			uint32_t x_uint, y_uint, z_uint;
//			float x, y, z;
//
//			memcpy(&x_uint, &uartRxBuffer[2], 4);
//			x_uint = __REV(x_uint); // Reverse bytes for little-endian
//			memcpy(&x, &x_uint, sizeof(float));
//
//			memcpy(&y_uint, &uartRxBuffer[6], 4);
//			y_uint = __REV(y_uint);
//			memcpy(&y, &y_uint, sizeof(float));
//
//			memcpy(&z_uint, &uartRxBuffer[10], 4);
//			z_uint = __REV(z_uint);
//			memcpy(&z, &z_uint, sizeof(float));
//
//			// Debug message
//			char debugMsg[64];
//			snprintf(debugMsg, sizeof(debugMsg), "CMD: %d, X: %.2f, Y: %.2f, Z: %.2f\n", command_type, x, y, z);
////			sprintf(debugMsg, "CMD: %d, X: %d.%02d, Y: %d.%02d, Z: %d.%02d\n",
////			    command_type,
////			    (int)x, (int)(x * 100) % 100,
////			    (int)y, (int)(y * 100) % 100,
////			    (int)z, (int)(z * 100) % 100);
//			HAL_UART_Transmit_IT(&huart3, (uint8_t*)debugMsg, strlen(debugMsg));
//
//
////			// Release both stepper semaphores simultaneously to complete stepper tasks
////			osSemaphoreRelease(stepperTask1Semaphore);
////			osSemaphoreRelease(stepperTask2Semaphore);
////
////			//wait until stepper tasks completed to continue (platform reaches expected height)
////			osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
////			osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//
////			//test servo control
////			PCA9685_SetServoAngle_DS3225(15, 0);  // Set to 0°
////			PCA9685_SetServoAngle_DS3225(14, 0);  // Set to 0°
////			PCA9685_SetServoAngle(13, 0);  // Set to 0°
////			PCA9685_SetServoAngle(12, 0);  // Set to 0°
//////			PCA9685_SetServoAngle(0, 0);  // Set to 0°
////
////			char str2[64] = {0};
////			snprintf(str2, sizeof(str2), "Servo Set to 0\n");
////			HAL_UART_Transmit_IT(&huart3, (uint8_t*)str2, sizeof (str2));
////
////	////	    HAL_Delay(500);  // Wait for 500 ms
////		    osDelay(pdMS_TO_TICKS(2000));
////			PCA9685_SetServoAngle_DS3225(15, 90);  // Set to 0°
////			PCA9685_SetServoAngle_DS3225(14, 90);  // Set to 0°
////			PCA9685_SetServoAngle(13, 90);  // Set to 0°
////			PCA9685_SetServoAngle(12, 80);  // Set to 0°
//
//
//			//harcoded gripper test
////			osSemaphoreAcquire(armTaskSemaphore, osWaitForever);
//
////			PCA9685_SetServoAngle_DS3225(15, 90);
////			PCA9685_SetServoAngle_DS3225(14, 180);
////			PCA9685_SetServoAngle(13, 90);
////			PCA9685_SetServoAngle(12, 80);
////			osDelay(pdMS_TO_TICKS(2000));
//
//			//potential small delay before gripper
//			osDelay(pdMS_TO_TICKS(800));
//			PCA9685_SetServoAngle(12, 0);
//
//			osDelay(pdMS_TO_TICKS(800));
//
//			PCA9685_SetServoAngle_DS3225(15, 55);  // Set to 0°
//			osDelay(pdMS_TO_TICKS(800));
//
//
//			PCA9685_SetServoAngle_DS3225(14, 125);  // Set to 0°
//			osDelay(pdMS_TO_TICKS(800));
//
//
//			PCA9685_SetServoAngle(13, 30);
//			osDelay(pdMS_TO_TICKS(800));
//
//
//		    clearBuffer(uartRxBuffer, sizeof(uartRxBuffer));
//	//	    HAL_UART_Receive_IT(&huart2, uartRxBuffer, sizeof(uartRxBuffer));
//
//		} else {
//			HAL_UART_Transmit_IT(&huart3, (uint8_t*)"Invalid UART Msg\n", 14);
//		}
//
//		// clear buffer for extra safety (next reception will overwrite)
//		memset(uartRxBuffer, 0, sizeof(uartRxBuffer));
//
//


	}
  /* USER CODE END StartArmTask */
}

/* USER CODE BEGIN Header_StartGripperTask */
/**
* @brief Function implementing the gripperTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGripperTask */
void StartGripperTask(void *argument)
{
  /* USER CODE BEGIN StartGripperTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGripperTask */
}

/* USER CODE BEGIN Header_StartStep1Task */
/**
* @brief Function implementing the step1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStep1Task */
void StartStep1Task(void *argument)
{
  /* USER CODE BEGIN StartStep1Task */
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreAcquire(stepperTask1Semaphore, osWaitForever);
	// Run motor 1 movement
	//up    -     8000 delay works
	stepCV(16000, 28000000, M1_IN1_PORT, M1_IN1_PIN, M1_IN2_PORT, M1_IN2_PIN,
		  M1_IN3_PORT, M1_IN3_PIN, M1_IN4_PORT, M1_IN4_PIN);
	//down
//	stepCCV(800, 600000, M1_IN1_PORT, M1_IN1_PIN, M1_IN2_PORT, M1_IN2_PIN,
//		  M1_IN3_PORT, M1_IN3_PIN, M1_IN4_PORT, M1_IN4_PIN);
	osSemaphoreRelease(armTaskSemaphore); // Notify completion
//    osDelay(1);
  }
  /* USER CODE END StartStep1Task */
}

/* USER CODE BEGIN Header_startStep2Task */
/**
* @brief Function implementing the step2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startStep2Task */
void startStep2Task(void *argument)
{
  /* USER CODE BEGIN startStep2Task */
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreAcquire(stepperTask2Semaphore, osWaitForever);
	// Run motor 2 movement
	//down
//	step2CV(800, 600000, M2_IN1_PORT, M2_IN1_PIN, M2_IN2_PORT, M2_IN2_PIN,
//			M2_IN3_PORT, M2_IN3_PIN, M2_IN4_PORT, M2_IN4_PIN);
	//up
	step2CCV(16000, 28000000, M2_IN1_PORT, M2_IN1_PIN, M2_IN2_PORT, M2_IN2_PIN,
			M2_IN3_PORT, M2_IN3_PIN, M2_IN4_PORT, M2_IN4_PIN);
	osSemaphoreRelease(armTaskSemaphore); // Notify completion
//    osDelay(1);
  }
  /* USER CODE END startStep2Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
