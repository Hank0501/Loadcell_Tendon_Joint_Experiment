/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control_table.h"
#include "my_dynamixel.h"
#include <stdlib.h> // for boolean type
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AS5048_PWM_RESOLUSION 4096
#define CLK_PWM 4119
#define CLK_ZERO 16

// ****** Manual Modify in the Calibration Mode ******//
// four situations of orgAngleAS5048A
// 1 , EX: ext->flex ===== 20->70
// 2 , EX: ext->flex ===== 250->160
// 3 , EX: ext->flex ===== 300->360(0)->40
// 4 , EX: ext->flex ===== 40->0(360)->300
#define TYPE_PIP_AS5048A 2
#define TYPE_DIP_AS5048A 2

// ****** Manual Modify in the Calibration Mode ******//
// *************** integer number of clk/counter *******************//
#define AS5048A_PIP_FLEX_LIM 1916
#define AS5048A_DIP_FLEX_LIM 1630
#define AS5048A_PIP_EXT_LIM 3012
#define AS5048A_DIP_EXT_LIM 2371

#define HS30B_PIP_FLEX_LIM 972
#define HS30B_DIP_FLEX_LIM 3723
#define HS30B_PIP_EXT_LIM 3969
#define HS30B_DIP_EXT_LIM 351

// ****** Manual Modify for the lowpasss filter ******//
#define ALPHA_AS5048A 0.8
#define ALPHA_LOADCELL 0.99

// ****** Coefficient of Error Compensation ******//

// Index 20250605
#define P3 -5.0203e-07
#define P2 -2.0258e-04
#define P1 0.0273
#define P0 -0.0014

#define D3 1.1005e-05
#define D2 -0.0011
#define D1 1.0273e-04
#define D0 0.0412

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile ServoXM4340 myServo;

float goalCurrent;
float goalPosition;
float goalVelocity;

float curPosition;
float curCurrent;
float curVelocity;

int tensionRecordFlag_PIP = 0;
int tensionRecordFlag_DIP = 0;
int angleRecordFlag_PIP = 0;
int angleRecordFlag_DIP = 0;

int tareRecordFlag;

float cur_Velocity_PIP;
float cur_Velocity_DIP;
float curF_Velocity_PIP;
float curF_Velocity_DIP;

/*--- used in AS5048A capture func. ---*/
volatile uint32_t IC_Val1_PIP = 0, IC_Val2_PIP = 0;
volatile uint32_t IC_Val1_DIP = 0, IC_Val2_DIP = 0;
int angClkAS5048A_PIP;
int angClkAS5048A_DIP;

/* Aligned angle in deg
/* deg 0 to deg max -----------*/
float cur_AngAS5048A_PIP;
float cur_AngAS5048A_DIP;

float pre_AngAS5048A_PIP;
float pre_AngAS5048A_DIP;

/* Aligned clk and counter
/* 0 to max -----------*/
int AlignedAngClkAS5048A_PIP;
int AlignedAngClkAS5048A_DIP;

/* angle for filter*/
/*------------------------------*/
float curF_AngAS5048A_PIP;
float preF_AngAS5048A_PIP;

float curF_AngAS5048A_DIP;
float preF_AngAS5048A_DIP;

/*--- used for error compensation ---*/
float comp_AngAS5048A_DIP;
float comp_AngAS5048A_PIP;

/*--- used for loadcell ---*/
uint32_t cur_loadcell;
uint32_t pre_loadcell;
float filtered_cur_loadcell;
float filtered_pre_loadcell;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void getJointAngle(void);
float compErrCalculate(float angle, float c3, float c2, float c1, float c0);
int alignAS5048A(int type, int angClkAS5048A, int extLim);

void getLoadCell(void);
void loadCellInit(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);

  DXL_InitServo(&myServo, 2, &huart6, GPIOC, GPIO_PIN_9);

  HAL_UART_DeInit(&huart6);
  huart6.Init.BaudRate = 4500000;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(100);

  HAL_ADC_Start(&hadc1);

  /* ------------------------- Initialize ------------------------------- */
  IC_Val1_PIP = 0;
  IC_Val2_PIP = 0;
  IC_Val1_DIP = 0;
  IC_Val2_DIP = 0;
  angClkAS5048A_PIP = 0;
  angClkAS5048A_DIP = 0;
  AlignedAngClkAS5048A_PIP = 0;
  AlignedAngClkAS5048A_DIP = 0;
  cur_AngAS5048A_PIP = 0;
  cur_AngAS5048A_DIP = 0;
  curF_AngAS5048A_PIP = 0;
  curF_AngAS5048A_DIP = 0;
  comp_AngAS5048A_DIP = 0;
  comp_AngAS5048A_PIP = 0;
  pre_AngAS5048A_PIP = AlignedAngClkAS5048A_PIP;
  pre_AngAS5048A_DIP = AlignedAngClkAS5048A_DIP;
  preF_AngAS5048A_PIP = curF_AngAS5048A_PIP;
  preF_AngAS5048A_PIP = curF_AngAS5048A_DIP;

  cur_Velocity_PIP = 0;
  cur_Velocity_DIP = 0;
  curF_Velocity_PIP = 0;
  curF_Velocity_DIP = 0;

  cur_loadcell = 0;
  pre_loadcell = 0;
  filtered_cur_loadcell = 0;
  filtered_pre_loadcell = 0;

  tareRecordFlag = 0;

  /* ---------------------------------------------------------------------------------------*/
  /* ------------------ Torsion Spring Coefficient And Tension Measurement -----------------*/
  /* ---------------------------------------------------------------------------------------*/

  HAL_Delay(1000);

  // Loadcell Tare
  // ------------------------------------------------------------------ //
  loadCellInit();

  uint32_t startTick0 = HAL_GetTick();
  tareRecordFlag = 1;
  while (HAL_GetTick() - startTick0 < 3000)
  {
    getLoadCell();
  }
  tareRecordFlag = 0;

  HAL_Delay(200);

  // ------------------Servo Setting ------------------- //

  // setServo_OperatingMode(&myServo, Current_CtrlMode);
  // setServo_OperatingMode(&myServo, CurrentBased_POS_CtrlMode);
  setServo_OperatingMode(&myServo, Velocity_CtrlMode);
  getServo_CurrentLimit(&myServo);
  getServo_OperatingMode(&myServo);
  setServo_TorqueENA(&myServo, TORQUE_ENABLE);
  // setServo_TorqueENA(&myServo, TORQUE_DISABLE);

  // ------------------ Parameter Test Part ------------------- //
  // // Check posCmd_Start, posCmd_MAX, servoReleaseCurCmd, servoWindingMin, servoWindingMax
  // goalCurrent = 500;
  // goalPosition = 340;
  // goalVelocity = -0.25; // rpm

  // while (1)
  // {
  //   // setServo_GoalCurrent(&myServo, goalCurrent);
  //   setServo_GoalPosition(&myServo, goalPosition);
  //   // setServo_GoalVelocity(&myServo, goalVelocity);

  //   getJointAngle();
  //   curPosition = getServo_PresentPosition(&myServo);
  //   curCurrent = getServo_PresentCurrent(&myServo);
  //   curVelocity = getServo_PresentVelocity(&myServo);

  //   // Read Loadcell
  //   pre_loadcell = cur_loadcell;
  //   filtered_pre_loadcell = filtered_cur_loadcell;
  //   if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
  //   {
  //     cur_loadcell = HAL_ADC_GetValue(&hadc1);
  //   }
  //   // Low Pass Filter
  //   filtered_cur_loadcell = ALPHA_LOADCELL * filtered_pre_loadcell +
  //                           (1 - ALPHA_LOADCELL) * ((float)cur_loadcell + (float)pre_loadcell) / 2;

  //   HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
  // }

  // ------------------ Manual Modified Needed ------------------- //
  int servo_dir = -1;

  // Attention for posCmd_MAX Selection
  // The PIP/DIP joint limtation should be notice.
  // float posCmd_Start = servo_dir * 40; // mA
  // float posCmd_MAX = servo_dir * 170;  // mA
  // float posCmd_Step = servo_dir * 10;  // mA

  // float delataCur = servo_dir * 3; // mA
  uint32_t steadyInterval = 200; // ms
  float alphaVelocityFilter = 0.99;
  float jointVelocityThreshold = 0.05; // rpm

  float servoWindingMin = 340; // WindingMin means the angle of servo when the tendon finger is in the maximum Flexion position
  float servoWindingMax = 82;  // WindingMax means the angle of servo when the tendon finger is in the Extension position

  float posCmd_Start = 165;
  float posCmd_End = servoWindingMax;
  float posCmd_Step = servo_dir * 8;

  float servoDiffThreshold = 2; // deg

  float servoReleaseVelCmd = -servo_dir * 8;        // rpm
  float servoWindingVelCmd_Slow = servo_dir * 0.25; // rpm
  float servoWindingVelCmd_Fast = servo_dir * 8;    // rpm

  // ------------------ Manual Modified State ------------------- //
  // state = 1 for Flexion Measurement
  // state = -1 for Extehnsion Measurement
  int state = -1;

  float pre_comp_AngAS5048A_PIP;
  float pre_comp_AngAS5048A_DIP;
  float pre_Velocity_PIP;
  float pre_Velocity_DIP;
  float preF_Velocity_PIP;
  float preF_Velocity_DIP;
  for (float posCmd = posCmd_Start; servo_dir * posCmd <= servo_dir * posCmd_End; posCmd += posCmd_Step)
  {
    goalVelocity = servoWindingVelCmd_Fast;
    setServo_GoalVelocity(&myServo, goalVelocity);

    // Joint Stable Detection
    // ------------------------------------------------------------------ //
    getJointAngle();
    pre_comp_AngAS5048A_PIP = comp_AngAS5048A_PIP;
    pre_comp_AngAS5048A_DIP = comp_AngAS5048A_DIP;

    int isStable = 0;

    angleRecordFlag_PIP = 0;
    angleRecordFlag_DIP = 0;

    uint32_t startTick1 = HAL_GetTick();
    while (!isStable)
    {
      getJointAngle();
      getLoadCell();
      curPosition = getServo_PresentPosition(&myServo);

      if (fabs(curPosition - posCmd) < servoDiffThreshold)
      {
        goalVelocity = 0; // stop the servo
        setServo_GoalVelocity(&myServo, goalVelocity);
        if (HAL_GetTick() - startTick1 >= 3000)
        {
          isStable = 1;
          angleRecordFlag_PIP = 1;
          angleRecordFlag_DIP = 1;
          HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
        }
      }
      else
        startTick1 = HAL_GetTick();
      pre_comp_AngAS5048A_PIP = comp_AngAS5048A_PIP;
      pre_comp_AngAS5048A_DIP = comp_AngAS5048A_DIP;
    }

    startTick1 = HAL_GetTick();
    while (HAL_GetTick() - startTick1 < 1000)
    {
      getJointAngle();
      getLoadCell();
    }
    angleRecordFlag_PIP = 0;
    angleRecordFlag_DIP = 0;

    // Main Experiment
    // ------------------------------------------------------------------ //
    int isMoving_PIP = 0;
    int isMoving_DIP = 0;

    tensionRecordFlag_PIP = 0;
    tensionRecordFlag_DIP = 0;

    uint32_t startTime_PWM_Sampling;
    getJointAngle();
    getLoadCell();

    pre_comp_AngAS5048A_PIP = comp_AngAS5048A_PIP;
    pre_comp_AngAS5048A_DIP = comp_AngAS5048A_DIP;

    pre_Velocity_PIP = 0;
    pre_Velocity_DIP = 0;
    preF_Velocity_PIP = 0;
    preF_Velocity_DIP = 0;
    cur_Velocity_PIP = 0;
    cur_Velocity_DIP = 0;
    curF_Velocity_PIP = 0;
    curF_Velocity_DIP = 0;

    goalVelocity = state * servoWindingVelCmd_Slow;
    setServo_GoalVelocity(&myServo, goalVelocity);

    while (!isMoving_PIP || !isMoving_DIP)
    // while (1)
    {
      HAL_GPIO_TogglePin(GPIOD, LD5_Pin);

      uint32_t startTick2 = HAL_GetTick();
      getJointAngle();

      // Get Joint Velocity
      uint32_t dt = HAL_GetTick() - startTime_PWM_Sampling;
      if (dt > 0)
      {
        cur_Velocity_PIP = ((comp_AngAS5048A_PIP - pre_comp_AngAS5048A_PIP) / dt) * 1000 / 6; // deg/ms to rpm
        cur_Velocity_DIP = ((comp_AngAS5048A_DIP - pre_comp_AngAS5048A_DIP) / dt) * 1000 / 6; // deg/ms to rpm
      }
      // Velocity Low Pass Filter
      curF_Velocity_PIP = alphaVelocityFilter * preF_Velocity_PIP + (1 - alphaVelocityFilter) * (cur_Velocity_PIP + pre_Velocity_PIP) / 2;
      curF_Velocity_DIP = alphaVelocityFilter * preF_Velocity_DIP + (1 - alphaVelocityFilter) * (cur_Velocity_DIP + pre_Velocity_DIP) / 2;
      pre_Velocity_PIP = cur_Velocity_PIP;
      pre_Velocity_DIP = cur_Velocity_DIP;
      preF_Velocity_PIP = curF_Velocity_PIP;
      preF_Velocity_DIP = curF_Velocity_DIP;

      pre_comp_AngAS5048A_PIP = comp_AngAS5048A_PIP;
      pre_comp_AngAS5048A_DIP = comp_AngAS5048A_DIP;

      // Record the time when getJointAngle() run
      startTime_PWM_Sampling = startTick2;

      // Read Loadcell
      getLoadCell();

      curPosition = getServo_PresentPosition(&myServo);

      if (fabs(curF_Velocity_PIP) > jointVelocityThreshold && !isMoving_PIP)
      {
        isMoving_PIP = 1;
        tensionRecordFlag_PIP = 1;
      }

      if (fabs(curF_Velocity_DIP) > jointVelocityThreshold && !isMoving_DIP)
      {
        isMoving_DIP = 1;
        tensionRecordFlag_DIP = 1;
      }

      HAL_Delay(5);
    }

    uint32_t startTick3 = HAL_GetTick();
    while (HAL_GetTick() - startTick3 < steadyInterval)
    {
      getJointAngle();
      getLoadCell();
      curPosition = getServo_PresentPosition(&myServo);
    }
    tensionRecordFlag_PIP = 0;
    tensionRecordFlag_DIP = 0;

    HAL_GPIO_TogglePin(GPIOD, LD6_Pin);

    // Release tendon
    goalVelocity = servoReleaseVelCmd;
    setServo_GoalVelocity(&myServo, goalVelocity);
    while ((servoWindingMax - servoWindingMin) * (servoWindingMin - curPosition) <= 0)
    {
      getJointAngle();
      getLoadCell();
      curPosition = getServo_PresentPosition(&myServo);
    }
    goalVelocity = 0;
    setServo_GoalVelocity(&myServo, goalVelocity);
  }

  /* ---------------------------------------------------------------------------*/
  /* ---------------------------------------------------------------------------*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
    HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void loadCellInit(void)
{
  // Read Loadcell
  if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
  {
    cur_loadcell = HAL_ADC_GetValue(&hadc1);
  }
  pre_loadcell = cur_loadcell;
  filtered_cur_loadcell = cur_loadcell;
  filtered_pre_loadcell = filtered_cur_loadcell;
}

void getLoadCell(void)
{
  // Read Loadcell
  if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
  {
    cur_loadcell = HAL_ADC_GetValue(&hadc1);
  }
  // Low Pass Filter
  filtered_cur_loadcell = ALPHA_LOADCELL * filtered_pre_loadcell +
                          (1 - ALPHA_LOADCELL) * ((float)cur_loadcell + (float)pre_loadcell) / 2;
  pre_loadcell = cur_loadcell;
  filtered_pre_loadcell = filtered_cur_loadcell;
}

void getJointAngle(void)
{
  // getAngClkAS5048A(); // original clk
  IC_Val1_PIP = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
  IC_Val2_PIP = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
  IC_Val2_DIP = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
  IC_Val1_DIP = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
  angClkAS5048A_PIP = (int)(IC_Val2_PIP + 1) * CLK_PWM / (IC_Val1_PIP + 1) - CLK_ZERO;
  angClkAS5048A_DIP = (int)(IC_Val2_DIP + 1) * CLK_PWM / (IC_Val1_DIP + 1) - CLK_ZERO;

  // Alignment();
  AlignedAngClkAS5048A_PIP = alignAS5048A(TYPE_PIP_AS5048A, angClkAS5048A_PIP, AS5048A_PIP_EXT_LIM);
  AlignedAngClkAS5048A_DIP = alignAS5048A(TYPE_DIP_AS5048A, angClkAS5048A_DIP, AS5048A_DIP_EXT_LIM);

  // getRealAngle();
  cur_AngAS5048A_PIP = (float)AlignedAngClkAS5048A_PIP * 360 / 4096;
  cur_AngAS5048A_DIP = (float)AlignedAngClkAS5048A_DIP * 360 / 4096;

  // getFilterAngle();
  curF_AngAS5048A_PIP = ALPHA_AS5048A * preF_AngAS5048A_PIP + (1 - ALPHA_AS5048A) * (cur_AngAS5048A_PIP + pre_AngAS5048A_PIP) / 2;
  curF_AngAS5048A_DIP = ALPHA_AS5048A * preF_AngAS5048A_DIP + (1 - ALPHA_AS5048A) * (cur_AngAS5048A_DIP + pre_AngAS5048A_DIP) / 2;
  pre_AngAS5048A_PIP = cur_AngAS5048A_PIP;
  pre_AngAS5048A_DIP = cur_AngAS5048A_DIP;
  preF_AngAS5048A_PIP = curF_AngAS5048A_PIP;
  preF_AngAS5048A_DIP = curF_AngAS5048A_DIP;

  float ePIP = compErrCalculate(curF_AngAS5048A_PIP, P3, P2, P1, P0);
  float eDIP = compErrCalculate(curF_AngAS5048A_DIP, D3, D2, D1, D0);

  comp_AngAS5048A_PIP = curF_AngAS5048A_PIP + ePIP;
  comp_AngAS5048A_DIP = curF_AngAS5048A_DIP + eDIP;
}

float compErrCalculate(float angle, float c3, float c2, float c1, float c0)
{
  float pow[4] = {1, 1, 1, 1};
  for (int i = 1; i <= 3; i++)
  {
    pow[i] = pow[i - 1] * angle;
  }

  return c3 * pow[3] + c2 * pow[2] + c1 * pow[1] + c0;
}

int alignAS5048A(int type, int angClkAS5048A, int extLim)
{
  switch (type)
  {
  case 1:
    // EX: ext->flex ===== 20->70
    return angClkAS5048A - extLim;
    break;
  case 2:
    // EX: ext->flex ===== 250->160
    return extLim - angClkAS5048A;
    break;
  case 3:
  {
    // EX: ext->flex ===== 300->360(0)->40
    int val = AS5048_PWM_RESOLUSION - extLim;
    if (angClkAS5048A <= AS5048_PWM_RESOLUSION / 2)
    {
      return angClkAS5048A + val;
    }
    else if (angClkAS5048A > AS5048_PWM_RESOLUSION / 2)
    {
      return angClkAS5048A - AS5048_PWM_RESOLUSION + val;
    }
    break;
  }
  case 4:
  {
    // EX: ext->flex ===== 40->0(360)->300
    int val = extLim;
    if (angClkAS5048A <= AS5048_PWM_RESOLUSION / 2)
    {
      return -angClkAS5048A + val;
    }
    else if (angClkAS5048A > AS5048_PWM_RESOLUSION / 2)
    {
      return -angClkAS5048A + AS5048_PWM_RESOLUSION + val;
    }
    break;
  }
  default:
    break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    DXL_SetTxFinished(true);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART6)
  {
    UNUSED(huart);
    DXL_AssignRxBufferToServo(&myServo);
    DXL_SetServoResponse_RxFinished(&myServo, true);
  }
}
/* USER CODE END 4 */

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

#ifdef USE_FULL_ASSERT
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
