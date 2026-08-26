/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_key.h"
#include "bsp_light.h"
#include "bsp_encoder.h"
#include "bsp_can_error.h"
#include "bsp_nm.h"
#include "bsp_uds.h"
#include "OLED.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_can.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t txMsgId2 = 0;
uint8_t txMsgId3 = 0;
uint8_t txMsgId5 = 0;
uint8_t key1count = 0;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

uint8_t last_close = 0;
int32_t lastangle = 0;
int32_t recangle = 0;

extern LightState currentLightState;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  Bsp_Can_Init();
  Bsp_Light_Init();
  Bsp_Encoder_Init();
  Bsp_CanError_Init();
  Bsp_Nm_Init();
  Bsp_Uds_Init();

  OLED_ShowString(1, 1, "Indicator:");
  OLED_ShowString(2, 1, "Dir:");
  OLED_ShowString(3, 1, "CAN:OK  ");
  OLED_ShowString(4, 1, "NM:ON   ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t lastCanErrCheck = 0;
    static uint32_t lastDispUpdate = 0;
    uint32_t now = HAL_GetTick();

    int32_t angle = Bsp_Encoder_GetCount();

    Bsp_AFS_Update(recangle);

    Bsp_Light_Update();

    Bsp_Uds_SelfTest_Update();
    Bsp_Uds_SessionTimer_Check();

    if (now - lastCanErrCheck >= 100) {
      lastCanErrCheck = now;
      Bsp_CanError_Check();
    }

    Bsp_Nm_Update();

    if (now - lastDispUpdate >= 500) {
      lastDispUpdate = now;
      switch (Bsp_CanError_GetState()) {
        case CAN_STATE_ACTIVE:  OLED_ShowString(3, 1, "CAN:OK  "); break;
        case CAN_STATE_WARNING: OLED_ShowString(3, 1, "CAN:WARN"); break;
        case CAN_STATE_PASSIVE: OLED_ShowString(3, 1, "CAN:PASS"); break;
        default:                OLED_ShowString(3, 1, "CAN:BOFF"); break;
      }
      if (Bsp_Nm_PeerOnline()) {
        OLED_ShowString(4, 1, "NM:ON   ");
      } else {
        OLED_ShowString(4, 1, "NM:LOST ");
      }
    }
  
    if(angle == 0)
    {
      OLED_ShowString(2, 5, "C");
      OLED_ShowNum(2, 6, angle, 3);
      if(currentLightState == LOW_BEAM_ON && angle != lastangle)
      {
        uint8_t d[4];
        d[0] = (angle >> 24) & 0xFF;
        d[1] = (angle >> 16) & 0xFF;
        d[2] = (angle >> 8)  & 0xFF;
        d[3] = angle         & 0xFF;
        Bsp_Can_SendSecure(CAN_ID_AFS_ANGLE, d, 4);
        lastangle = angle;
      }
    }
    else if(angle > 0)
    {
      OLED_ShowString(2, 5, "L");
      OLED_ShowNum(2, 6, angle, 3);
      if(currentLightState == LOW_BEAM_ON && angle != lastangle)
      {
        uint8_t d[4];
        d[0] = (angle >> 24) & 0xFF;
        d[1] = (angle >> 16) & 0xFF;
        d[2] = (angle >> 8)  & 0xFF;
        d[3] = angle         & 0xFF;
        Bsp_Can_SendSecure(CAN_ID_AFS_ANGLE, d, 4);
        lastangle = angle;
      }
    }
    else if (angle < 0)
    {
      OLED_ShowString(2, 5, "R");
      OLED_ShowNum(2, 6, -angle, 3);
      if(currentLightState == LOW_BEAM_ON && angle != lastangle)
      {
        uint8_t d[4];
        d[0] = (angle >> 24) & 0xFF;
        d[1] = (angle >> 16) & 0xFF;
        d[2] = (angle >> 8)  & 0xFF;
        d[3] = angle         & 0xFF;
        Bsp_Can_SendSecure(CAN_ID_AFS_ANGLE, d, 4);
        lastangle = angle;
      }
    }

    if (KeyNum != 0) {
      if (KeyNum == 1)     // PB12 - 近光/远光/关闭循环
      {
        uint8_t cmd;
        uint16_t id;
        if (key1count == 0) { cmd = 0x10; id = CAN_ID_LOW_BEAM_ON; }
        else if (key1count == 1) { cmd = 0x20; id = CAN_ID_HIGH_BEAM_ON; }
        else { cmd = 0x11; id = CAN_ID_LOW_BEAM_OFF; }
        Bsp_Can_SendSecure(id, &cmd, 1);

        if(key1count == 0)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "LB");
          Bsp_Light_LowBeam_On();
        }
        else if(key1count == 1)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "HB");
          Bsp_Light_HighBeam_On();
        }
        else if(key1count == 2)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "OFF");
          Bsp_Light_LowBeam_Off();
        }

        key1count++;
        if (key1count >= 3) key1count = 0;
      }
      else if (KeyNum == 2)   // PB13 - 左转向灯
      {
        uint8_t cmd = (txMsgId2 == 0) ? 0x30 : 0x31;
        uint16_t id = (txMsgId2 == 0) ? CAN_ID_LEFT_TURN_ON : CAN_ID_LEFT_TURN_OFF;
        Bsp_Can_SendSecure(id, &cmd, 1);
        if(txMsgId2 == 0)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "TL");
          Bsp_Light_LeftTurn_On();
        }
        else
        {
          OLED_ShowString(1, 11, "OFF");
          Bsp_Light_LeftTurn_Off();
          last_close = 1;
        }
        txMsgId2++;
        if (txMsgId2 > 1) txMsgId2 = 0;
      }
      else if (KeyNum == 3)   // PB14 - 右转向灯
      {
        uint8_t cmd = (txMsgId3 == 0) ? 0x40 : 0x41;
        uint16_t id = (txMsgId3 == 0) ? CAN_ID_RIGHT_TURN_ON : CAN_ID_RIGHT_TURN_OFF;
        Bsp_Can_SendSecure(id, &cmd, 1);
        if(txMsgId3 == 0)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "TR");
          Bsp_Light_RightTurn_On();
        }
        else
        {
          OLED_ShowString(1, 11, "OFF");
          Bsp_Light_RightTurn_Off();
          last_close = 1;
        }
        txMsgId3++;
        if (txMsgId3 > 1) txMsgId3 = 0;
      }
      else if (KeyNum == 4)   // PB15 - 双闪(危险警示灯)
      {
        uint8_t cmd = (txMsgId5 == 0) ? 0x50 : 0x51;
        uint16_t id = (txMsgId5 == 0) ? CAN_ID_HAZARD_ON : CAN_ID_HAZARD_OFF;
        Bsp_Can_SendSecure(id, &cmd, 1);
        if(txMsgId5 == 0)
        {
          OLED_ShowString(1, 11, "   ");
          OLED_ShowString(1, 11, "HAZ");
          Bsp_Light_Hazard_On();
        }
        else
        {
          OLED_ShowString(1, 11, "OFF");
          Bsp_Light_Hazard_Off();
          last_close = 1;
        }
        txMsgId5++;
        if (txMsgId5 > 1) txMsgId5 = 0;
      }

      KeyNum = 0;
    }

    if (Bsp_Can_ReceiveFlag())
    {
      CanRxStatus rxSt = Bsp_Can_Receive(&RxHeader, RxData);
      if (rxSt == CAN_RX_OK || rxSt == CAN_RX_CC_ERROR)
      {
        if (RxHeader.StdId == UDS_REQ_ID) {
          Bsp_Uds_TpFeed(RxData, RxHeader.DLC);
        }
        else if (RxHeader.StdId == CAN_ID_NM_HEARTBEAT) {
          Bsp_Nm_OnHeartbeat(RxData);
        }
        else if(RxHeader.StdId == 0x100)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x10) 
          {
            Bsp_Light_LowBeam_On();
          } 
        }
      }
      else if(RxHeader.StdId == 0x101)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x11) 
          {
            Bsp_Light_LowBeam_Off();
          }
        }
      }
      else if(RxHeader.StdId == 0x200)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x20) 
          {
            Bsp_Light_HighBeam_On();
          }
        }
      }
      else if(RxHeader.StdId == 0x201)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x21) 
          {
            Bsp_Light_HighBeam_Off();
          }
        }
      }
      else if(RxHeader.StdId == 0x300)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x30) 
          {
            Bsp_Light_LeftTurn_On();
          }
        }
      }
      else if(RxHeader.StdId == 0x301)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x31) 
          {
            Bsp_Light_LeftTurn_Off();
            last_close = 1;
          }
        }
      }
      else if(RxHeader.StdId == 0x400)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x40) 
          {
            Bsp_Light_RightTurn_On();
          }
        }
      }
      else if(RxHeader.StdId == 0x401)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x41) 
          {
            Bsp_Light_RightTurn_Off();
            last_close = 1;
          }
        }
      }
      else if(RxHeader.StdId == 0x500)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x50) 
          {
            Bsp_Light_Hazard_On();
          }
        }
      }
      else if(RxHeader.StdId == 0x501)
      {
        if (RxHeader.DLC == CAN_SECURE_LEN) 
        {
          if (RxData[0] == 0x51) 
          {
            Bsp_Light_Hazard_Off();
            last_close = 1;
          }
        }
      }
      else if (RxHeader.StdId == 0x600)
        {
          if (RxHeader.DLC == CAN_SECURE_LEN)
          {
            recangle = (int32_t)(
            ((uint32_t)RxData[0] << 24) |
            ((uint32_t)RxData[1] << 16) |
            ((uint32_t)RxData[2] << 8)  |
            (uint32_t)RxData[3]
            );
          }
        }
      }
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
