/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "note.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/*we get an square wave signal with an duration which is half the duration of our frequency on the output
our frequency describes the pitch of our tone  */
static void tone_set(int frequency){ // Funktion für die Einstellung für die Frequenz
      
      int Counter_Period = (32000000/frequency);
      __HAL_TIM_SET_AUTORELOAD(&htim2, Counter_Period);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Counter_Period/2);
 }

/*tone_start is an function which starts our notes */
   static void tone_start(void){ 
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  } 
/*tone_stop is an function that stops our notes during the song if needed*/
  static void tone_stop(void){ // Funktion für den Ton Stop
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  }


// we use the note_duration  function to calculate our different note durations
static int note_duration(int value, int bpm){  // 

    if(value == 0){
  return ((60000/bpm)*4);
}
else if(value == 2){
  return((60000/bpm)*2);
}
else if(value == 4){
  return((60000/bpm));
}
else if(value == 8){
  return((60000/bpm)/2);
}
else if(value == 16){
  return((60000/bpm)/4);
}


      // punctured notes
else if(value == 00){
  return (((60000/bpm)*4)+(1/2)*((60000/bpm)*4));
}
else if (value ==  22){
  return (((60000/bpm)*2)+(1/2)*((60000/bpm)*2));
}
else if (value ==  44){
  return (((60000/bpm))+(1/2)*((60000/bpm)));
}
else if (value ==  88){
  return (((60000/bpm)/2)+(1/2)*((60000/bpm)/2));
}
else if (value ==  1616){
  return (((60000/bpm)/4)+(1/2)*((60000/bpm)/4));
}
else {
  return 0;
}

}
  

void setcolor(int r, int g, int b){
  htim1.Instance->CCR4 = r * 255;
  htim1.Instance->CCR1 = g * 255;
  htim1.Instance->CCR3 = b *255;
}


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
 void colorset(int frequency){
      // r g b
  //Grün
  if(frequency == 169 || frequency == 349 ||frequency == 698){ 
    setcolor(0,255,0); 
  }
  // Rot
  else if(frequency == 370 || frequency == 392 || frequency == 494){
    setcolor(255,0,0);
  }
  //Blau
    else if(frequency == 165 || frequency == 466 || frequency == 622){
    setcolor(0,0,255);
  }
  //Orange
  else if(frequency == 784 || frequency == 988 || frequency == 80){
    setcolor(255,127,0);
  }
  //Gelb
  else if(frequency == 220 || frequency == 233 || frequency == 247){
    setcolor(255,255,0);
  }
  //Grün-Gelb
  else if(frequency == 207 || frequency == 740 || frequency == 932){
    setcolor(255,0,127);
  }
  //Mint-Grün
  else if(frequency == 329 || frequency == 659 || frequency == 155){
    setcolor(0,255,127);
  }
  //Türkis 
  else if(frequency == 147 || frequency == 311 || frequency == 415){
    setcolor(0,255,255);
  }
  //Hellblau
  else if(frequency == 174 || frequency == 830 || frequency == 587){
    setcolor(0,127,255);
  }
  //Violett
  else if(frequency == 185 || frequency == 293 || frequency == 440){
    setcolor(127,0,255);
  }
  //Rose
  else if(frequency == 138 || frequency == 277 || frequency == 554){
    setcolor(255,0,255);
  }
  //Magenta 
  else if(frequency == 131 || frequency == 262 || frequency == 523){
    setcolor(255,0,127);
  }

}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();




char Taster;
int Zeit; 
int mode = 0;
int x; 

Taster = HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin); 
HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);   HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_4);   HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_3);


while(1) {

    
    if(mode == 0) {

    

            for(int i = 0; i < song3_len; i++) {  //hochzählen von den einzelnen Noten von Lied 1
            colorset(song3[i].frequency); // farbeinstellung der jeweiligen Noten im Song
            tone_set(song3[i].frequency); // einstellen der Tonfrequenzen im 1.Lied
            tone_start();
            x = note_duration(song3[i].value, 120); //takt des liedes auf bpm gesetzt
            Zeit = HAL_GetTick();
            while ((HAL_GetTick()-Zeit) < x)
            {
                if(Taster == GPIO_PIN_SET) { //überprüfen ob der taster gedrückt wird 
                HAL_Delay(100); //entprellung
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 1;
                break;
            }
            }
            
            tone_stop();
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            if(HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin) == GPIO_PIN_SET) {
                HAL_Delay(100);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 1;
                break;
            }
        }

    }
    //wechsel von lied 3 in lied 4
      else if(mode == 1) {
        for(int i = 0; i < song4_len; i++) {
            colorset(song4[i].frequency);
            tone_set(song4[i].frequency);
            tone_start();
            x = note_duration(song4[i].value, 130);
            Zeit = HAL_GetTick(); 
            while (HAL_GetTick()-Zeit < x)
            {
                if(HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin) == GPIO_PIN_SET) {
                HAL_Delay(100);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 2;
                break;
            }
            }
            
            tone_stop();
             __HAL_TIM_SET_COUNTER(&htim2, 0);
            if(HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin) == GPIO_PIN_SET) {
                HAL_Delay(100);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 2;
                break;
            }
        }
    }
    

      else if(mode == 2){
            for(int i = 0; i < song2_len; i++) {
            colorset(song2[i].frequency);
            tone_set(song2[i].frequency);
            tone_start();
            x = note_duration(song2[i].value, 138);
            Zeit = HAL_GetTick();
            while ((HAL_GetTick()-Zeit) < x)
            {
          
            if(HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin) == GPIO_PIN_SET) {
                HAL_Delay(100);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 0;
                break;
            }
            }
            
            tone_stop();
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            if(HAL_GPIO_ReadPin(TASTER_GPIO_Port, TASTER_Pin) == GPIO_PIN_SET) {
                HAL_Delay(100);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                mode = 0;
                break;
            }
        }
  }


  /* USER CODE END 3 */
}
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 72727;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 72727/2 ;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 115100;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TASTER_GPIO_Port, TASTER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TASTER_Pin */
  GPIO_InitStruct.Pin = TASTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TASTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
