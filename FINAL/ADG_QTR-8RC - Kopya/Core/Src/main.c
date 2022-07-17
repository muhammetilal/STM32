/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "string.h"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
	char data_hcr[32] = "";
	char data_qtr[32] = "";
	char data_error[32] = "";
	
	int sensor_read = 0x00000000;
	int white = 1;
	int error = 0;
	int keskin = 0;

	float	P , I, D, pre_error=0,PID_val=0,pre_I;
	float Kp=31, Ki=0, Kd=950;//PID degerleri

	
	uint32_t time;
  uint16_t distance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;	
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
	
	
}

void user_pwm_setvalue3(uint16_t value)
{
	
 HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
 TIM_OC_InitTypeDef sConfigOC;
 
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = value;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 
 HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); 
}

void user_pwm_setvalue4(uint16_t value)
{
	
HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_1);
 TIM_OC_InitTypeDef sConfigOC;
 
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = value;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 
 HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
}


int QTR8_read (void) 
{	
	HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, 1);
	
	Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
	Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
	Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
	Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);
	
	HAL_GPIO_WritePin (SENSOR1_GPIO_Port, SENSOR1_Pin, 1);
	HAL_GPIO_WritePin (SENSOR2_GPIO_Port, SENSOR2_Pin, 1);
	HAL_GPIO_WritePin (SENSOR3_GPIO_Port, SENSOR3_Pin, 1);
	HAL_GPIO_WritePin (SENSOR4_GPIO_Port, SENSOR4_Pin, 1);
	HAL_GPIO_WritePin (SENSOR5_GPIO_Port, SENSOR5_Pin, 1);
	HAL_GPIO_WritePin (SENSOR6_GPIO_Port, SENSOR6_Pin, 1);
	HAL_GPIO_WritePin (SENSOR7_GPIO_Port, SENSOR7_Pin, 1);
	HAL_GPIO_WritePin (SENSOR8_GPIO_Port, SENSOR8_Pin, 1);
	
	delay_us(12);
	
	Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
	Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
	Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
	Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);
	
	// Threshold
	delay_us(6000);
	
	sensor_read = 0x00000000;

	
	if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin)) {
		sensor_read |= 0x00000001;

	}
	if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
		sensor_read |= 0x00000010;

  }
	if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin)) {
		sensor_read |= 0x00000100;

  }
	if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin)) {
		sensor_read |= 0x00001000;

  }
	if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin)) {
		sensor_read |= 0x00010000;

  }
	if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin)) {
		sensor_read |= 0x00100000;

  }
	if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin)) {
		sensor_read |= 0x01000000;

  }
	if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin)) {
		sensor_read |= 0x10000000;

  }


		
		

	
	
if(sensor_read == 0x00000000) {
error = 0;
keskin = 0;}
else if (sensor_read == 0x00011000) {
error = 0;
} else if (sensor_read == 0x00001000 || sensor_read == 0x00001100) {
error = 1;
} else if (sensor_read == 0x00000100 || sensor_read == 0x00000110 || sensor_read == 0x00001110) {
error = 2;
} else if (sensor_read == 0x00000010 || sensor_read == 0x00000011 || sensor_read == 0x00000111) {
error = 3;
} else if (sensor_read == 0x00000001) {
error = 4;
} else if (sensor_read == 0x00010000 || sensor_read == 0x00110000) {
error = -1;
} else if (sensor_read == 0x00100000 || sensor_read == 0x01100000|| sensor_read == 0x01110000) {
error = -2;
} else if (sensor_read == 0x01000000 || sensor_read == 0x11000000 || sensor_read == 0x11100000) {
error = -3;
} else if (sensor_read == 0x10000000) {
error = -4;
}else if (sensor_read == 0x00011011 || sensor_read == 0x00011111 || sensor_read == 0x00001011 || sensor_read == 0x00001111 || sensor_read == 0x00010011
|| sensor_read == 0x00010111) {
error = 5;
keskin = 1;
} else if (sensor_read == 0x11011000 || sensor_read == 0x11111000 || sensor_read == 0x11001000 || sensor_read == 0x11101000 || sensor_read == 0x11010000
|| sensor_read == 0x11110000) {
error = -5;
keskin = -1;

	

}


		P = error;
    I = I + error;
    D = error-pre_error;
    PID_val = (Kp*P) + (Ki*I) + (Kd*D);
    pre_I=I;
    pre_error=error;





	  float pwm_adg=220;//Hiz ayari


	
	  if(error > 0){
						user_pwm_setvalue3(pwm_adg +1*(Kp*P));
			user_pwm_setvalue4(pwm_adg -1*(Kp*P)/2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
		}
		else if(error < 0){
						user_pwm_setvalue3(pwm_adg -1*(Kp*P));
			user_pwm_setvalue4(pwm_adg +1*(Kp*P)/2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
		}
		else{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
			
		}
		
if (keskin == -1){
		if (PID_val < 0) {
			user_pwm_setvalue3(pwm_adg +5*(Kp*P));
			user_pwm_setvalue4(pwm_adg -5*(Kp*P));
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
			keskin = 0;
			HAL_Delay(100);
			
		} else {
			user_pwm_setvalue3(pwm_adg+10*(Kp*P));
			user_pwm_setvalue4(pwm_adg -(Kp*P)*10);
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
			keskin = 0;
			HAL_Delay(100);
			
		}
	}

	
else if (keskin == 1){
		if (PID_val < 0) {
			user_pwm_setvalue3(pwm_adg -(Kp*P)*10);
			user_pwm_setvalue4(pwm_adg+10*(Kp*P));
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
			keskin = 0;
			HAL_Delay(100);
			
		} else {
			user_pwm_setvalue3(pwm_adg+10*(Kp*P));
			user_pwm_setvalue4(pwm_adg -10*(Kp*P));
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
			keskin = 0;
			HAL_Delay(100);
			
		}
	}	

	else if (sensor_read == 0x10100000 ){
					user_pwm_setvalue3(pwm_adg+1*(Kp*P));
					user_pwm_setvalue4(pwm_adg +1*(Kp*P));
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
			HAL_Delay(25);
}

	else if (sensor_read == 0x00000101 ){
							user_pwm_setvalue3(pwm_adg+1*(Kp*P));
					user_pwm_setvalue4(pwm_adg +1*(Kp*P));
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,1);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,0);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
			HAL_Delay(25);

	}


	
		else{
					if (PID_val < 0) {
					user_pwm_setvalue3(pwm_adg+1*(Kp*P));
					user_pwm_setvalue4(pwm_adg +1*(Kp*P));
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
			
		} else {
					user_pwm_setvalue3(pwm_adg+1*(Kp*P));
					user_pwm_setvalue4(pwm_adg +1*(Kp*P));
			
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,1);


  	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);  ///FORWARD 0 1 AND 1 0 BACK
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);

			
		}
		}
			
//			HAL_UART_Receive(&huart3,(uint8_t*)data_error,50,1000);
//		sprintf(data_error," %d \n\r",error);
//		HAL_UART_Transmit(&huart3,(uint8_t*)data_error,strlen(data_error),1000);
}

//uint32_t Read_HCSR04(){

//uint32_t local_time = 0;
//	
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);
//delay_us(10);
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);

//	
//while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));
//	
//while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))
//	{

//local_time++;
//delay_us(1);

//}
//return local_time;

//}






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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_1, 100);
	__HAL_TIM_SET_COMPARE (&htim3, TIM_CHANNEL_2, 100);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, 0);
//	HAL_Delay(2000);
	


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*
		time = Read_HCSR04();
		distance = time/29;
		sprintf(data_hcr,"\f %d \n\r",distance);
		HAL_UART_Transmit(&huart1,(uint8_t*)data_hcr,strlen(data_hcr),1000);
		
		if(distance>5){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		
		}
		else{
		
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		*/
	//////////////////////////////////////////////

//		sprintf(data_qtr," %08X \n\r",sensor_read);
//		HAL_UART_Transmit(&huart1,(uint8_t*)data_qtr,strlen(data_qtr),1000);

		QTR8_read();


//		if(error > 0){
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
//		}
//		else if(error < 0){
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//		}
//		else{
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
//			
//		}
		
		
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
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SENSOR8_Pin|SENSOR7_Pin|SENSOR6_Pin|SENSOR5_Pin
                          |SENSOR4_Pin|SENSOR3_Pin|SENSOR2_Pin|SENSOR1_Pin
                          |trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDON_Pin|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR8_Pin SENSOR7_Pin SENSOR6_Pin SENSOR5_Pin
                           SENSOR4_Pin SENSOR3_Pin SENSOR2_Pin SENSOR1_Pin */
  GPIO_InitStruct.Pin = SENSOR8_Pin|SENSOR7_Pin|SENSOR6_Pin|SENSOR5_Pin
                          |SENSOR4_Pin|SENSOR3_Pin|SENSOR2_Pin|SENSOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDON_Pin */
  GPIO_InitStruct.Pin = LEDON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LEDON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : trigger_Pin */
  GPIO_InitStruct.Pin = trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trigger_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
