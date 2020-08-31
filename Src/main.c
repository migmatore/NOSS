/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stepper.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[100];
/* USER CODE END 0 */
//#define SERVO_180 8200
//#define SERVO_0 1800
//
//#define TURNIGY_TG9E
//
////
////	Defines
////
//
//// For Turnigy TG9e
//#ifdef    TURNIGY_TG9E
//#define SERVO_MIN 1000
//#define SERVO_MAX 2000
//#define ANGLE_MIN 0
//#define ANGLE_MAX 180
//#endif
//
//typedef struct {
//    TIM_HandleTypeDef *htim;
//    uint32_t channel;
//} servo_t;
//
//// Функция устанавливает позицию вала (в градусах)
//void set_pos(uint8_t pos) {
//    uint32_t tmp = (SERVO_180 - SERVO_0) / 180;
//    TIM2->CCR1 = SERVO_0 + tmp * pos;
//}
//
//void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, uint32_t _channel) {
//    servo->htim = _htim;
//    servo->channel = _channel;
//
//    HAL_TIM_PWM_Start(servo->htim, servo->channel);
//}

//
//	map help function
//
//long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
//int map(int st1, int fn1, int st2, int fn2, int value) {
//    return (1.0 * (value - st1)) / ((fn1 - st1) * 1.0) * (fn2 - st2) + st2;
//}
//
//void servo_write(int angle) {
//    htim2.Instance->CCR1 = map(0, 180, 50, 250, angle);
//}
//
////
////	Servo set angle function
////
//void Servo_SetAngle(servo_t *servo, uint16_t angle) {
//    if (angle < 0) angle = 0;
//    if (angle > 180) angle = 180;
//
//    uint16_t tmp = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
//    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, tmp);
//}
//
////
////	Servo set angle fine function
////
//void Servo_SetAngleFine(servo_t *servo, float angle) {
//    if (angle < 0) angle = 0;
//    if (angle > 180) angle = 180;
//
//    uint16_t tmp = map(angle, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
//    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, tmp);
//}

uint16_t angle;
//servo_t servo1, servo2;
stepper_t stepper;


void delay(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void stepper_set_rpm(int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
    delay(60000000 / 4096 / rpm);
}

void stepper_half_drive(int step) {
    switch (step) {
        case 0:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);   // IN4
            break;

        case 1:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);   // IN4
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);   // IN4
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_SET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);   // IN4
            break;

        case 4:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_RESET);   // IN4
            break;

        case 5:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_SET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);   // IN4
            break;

        case 6:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);   // IN4
            break;

        case 7:
            HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);   // IN1
            HAL_GPIO_WritePin(GPIOB, IN2_Pin, GPIO_PIN_RESET);   // IN2
            HAL_GPIO_WritePin(GPIOB, IN3_Pin, GPIO_PIN_RESET);   // IN3
            HAL_GPIO_WritePin(GPIOB, IN4_Pin, GPIO_PIN_SET);   // IN4
            break;

    }
}

void stepper_step_angle(float angle, int direction, int rpm) {
    float anglepersequence = 0.703125;  // 360 = 512 sequences
    int numberofsequences = (int) (angle / anglepersequence);

    for (int seq = 0; seq < numberofsequences; seq++) {
        if (direction == 0)  // for clockwise
        {
            for (int step = 7; step >= 0; step--) {
                stepper_half_drive(step);
                stepper_set_rpm(rpm);
            }

        } else if (direction == 1)  // for anti-clockwise
        {
            for (int step = 0; step < 8; step++) {
                stepper_half_drive(step);
                stepper_set_rpm(rpm);
            }
        }
    }
}

int setpoint = 0;   // заданная величина, которую должен поддерживать регулятор
int input = 0;      // сигнал с датчика (например температура, которую мы регулируем)
int output = 0;     // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
int pidMin = -255;     // минимальный выход с регулятора
int pidMax = 255;   // максимальный выход с регулятора
// коэффициенты
float Kp = 1.0;
float Ki = 1.0;
float Kd = 0/5;
float _dt_s = 0.1; // время итерации в секундах
// вспомогательные переменные
int prevInput = 0;
float integral = 0.0;

#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

int computePID() {
    float error = setpoint - input;           // ошибка регулирования
    float delta_input = prevInput - input;    // изменение входного сигнала
    prevInput = input;
    output = 0;
    output += (float) error * Kp;                  // пропорционально ошибке регулирования
    output += (float) delta_input * Kd / _dt_s;    // дифференциальная составляющая
    integral += (float) error * Ki * _dt_s;        // расчёт интегральной составляющей
    // тут можно ограничить интегральную составляющую!
    output += integral;                           // прибавляем интегральную составляющую
    output = constrain(output, pidMin, pidMax);   // ограничиваем выход
    return output;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    HAL_TIM_Base_Start(&htim1);
    //MX_TIM2_Init();
    //Servo_Init(&servo1, &htim2, TIM_CHANNEL_1);

    /* USER CODE BEGIN 2 */
    while (MPU6050_Init(&hi2c1) == 1);
    /* USER CODE END 2 */
    //uint8_t i;
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    //uint32_t t = HAL_GetTick();

//    stepper_init(&stepper, 200, in1, in2, in3, in4);
//    setSpeed(&stepper, 60);
//    step(&stepper, 100);

    while (1) {

        //uint32_t t = HAL_GetTick();
//        step(&stepper, 100);
//        HAL_Delay(1000);
//        step(&stepper, -100);
//        HAL_Delay(1000);
        /* USER CODE END WHILE */
        MPU6050_Read_All(&hi2c1, &MPU6050);
        HAL_Delay(10);
//
        double y = MPU6050.KalmanAngleY;
//        double x = MPU6050.KalmanAngleX;
        input = (int)y;

        int out = computePID();

        if (out > 0) {
            stepper_step_angle((float)out, 1, 13);
        } else {
            uint out1 = out * -1;
            stepper_step_angle((float)out1, 0, 13);
        }

//        int32_t tmp1 = (int32_t)y;
//        double tmp_frac = y - tmp1;
//        int32_t tmp2 = trunc(tmp_frac * 1000);

        //sprintf(str, "Y: %d.%04d \r\n", tmp1, tmp2);
//        HAL_UART_Transmit(&huart2, y, sizeof(str), 100);

        char msg[31];
        uint16_t len = sprintf(msg, "y: %d out: %d \r\n", (int16_t) input, (int16_t) out);
        HAL_UART_Transmit(&huart2, (uint8_t *) msg, len, HAL_MAX_DELAY);
//        for (i=0;i<=180;i++) {
//            HAL_Delay(1000);
//            set_pos(i);
//        }
//        for(i = 0; i <= 180; i++)
//        {
//            servo_write(i);
//            HAL_Delay(10);
//        }
//        for(i = 180; i >= 0; i--)
//        {
//            servo_write(i);
//            HAL_Delay(10);
//        }
        //Servo_SetAngle(&servo1, 0);
        //Servo_SetAngle(&servo2, 90);
        //HAL_Delay(1000);

//        Servo_SetAngle(&servo1, 90);
//        Servo_SetAngle(&servo2, 180);
//        HAL_Delay(1000);
        //HAL_Delay(100);
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
                                         | RCC_PERIPHCLK_TIM1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
