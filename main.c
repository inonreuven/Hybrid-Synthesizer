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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include  "stm32f4xx_it.h"
#include  "stm32f4xx_hal_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SS_DIGIT 4
#define RCC_GPIO_DIG __HAL_RCC_GPIOA_CLK_ENABLE
#define RCC_GPIO_SEG __HAL_RCC_GPIOB_CLK_ENABLE
#define RCC_GPIO_DIG __HAL_RCC_GPIOC_CLK_ENABLE
#define GPIO_DIG GPIOC
#define GPIO_SEG GPIOB
#define GPIO_PIN_DIG_0 GPIO_PIN_11
#define GPIO_PIN_DIG_1 GPIO_PIN_10
#define GPIO_PIN_DIG_2 GPIO_PIN_9
#define GPIO_PIN_DIG_3 GPIO_PIN_8
#define GPIO_PIN_SEG_A GPIO_PIN_7
#define GPIO_PIN_SEG_B GPIO_PIN_6
#define GPIO_PIN_SEG_C GPIO_PIN_5
#define GPIO_PIN_SEG_D GPIO_PIN_4
#define GPIO_PIN_SEG_E GPIO_PIN_3
#define GPIO_PIN_SEG_F GPIO_PIN_2
#define GPIO_PIN_SEG_G GPIO_PIN_1

#define bufferSize 100
#define pitchSize 128
#define IDLE 0x00
#define NOTEON 0x90
#define NOTEOFF 0x80
#define VEL_ZERO 0x00

#define CC_MIN 0x61
#define CC_MAX 0x63
#define FREQ_MUL_ONE 0
#define FREQ_MUL_TWO 3
#define FREQ_MUL_THREE 6
#define FREQ_MUL_FOUR 9
#define FREQ_MUL_EIGHT 12
#define FREQ_MUL_SIXT 15
#define FREQ_MUL_HALF 18
#define FREQ_MUL_QU  21

#define SIXTEEN_STEPS 16
#define TWO_STEPS 2
#define THREE_STEPS 3
#define FOUR_STEPS 4
#define FIVE_STEPS 5
#define EIGHT_STEPS 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim2_ch3_up;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer[3]= {0x90,0x44,0x90};
uint8_t bufferTx[36]=
	  {	0x90,0x40,0x40,
		0x90,0x43,0x40,
		0x80,0x43,0x00,
		0x90,0x45,0x40,
		0x80,0x45,0x00,
		0x80,0x40,0x00,
		0x90,0x3C,0x40,
		0x90,0x47,0x40,
		0x80,0x47,0x00,
		0x90,0x48,0x40,
		0x80,0x48,0x00,
		0x80,0x3C,0x40 };
uint8_t bufferTx1[36]=
	  {	0x90,0x24,0x40,
		0xD0,0x43,
		0xD0,0x60,
		0xD0,0x00,
		0xD0,0x45,
		0xD0,0x50,
		0xD0,0x00,
		0xD0,0x40,
		0xD0,0x40,
		0xD0,0x40,
		0xD0,0x47,
		0xD0,0x60,
		0xD0,0x00,
		0xD0,0x48,
		0xD0,0x30,
		0xD0,0x00,
		0x80,0x24,0x40 };

uint16_t time = 0;
uint16_t NumOfMessages = 0;
uint16_t messageLong = 3;
int pointerToMidiBuff = 0;

uint8_t bufferRx[7]; //= {1,2,3,4,5,6,7};

// init midi buffer
uint32_t CNT[bufferSize];
uint8_t midiBuffer[bufferSize];
//uint8_t typeOfMidiMess = 0;
HAL_StatusTypeDef return_val;
const uint32_t font[10] = { 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B };
uint8_t digits[4];
uint32_t flag = 1;
int bpm = 0;
int bpmval =0;
float freqMul = 1;
int EncVal = 0;
int bpmToMiliSec = 500;
int encoderValForSteps = 0;


uint8_t flagForNotStatus = 1;
uint8_t pitchVal=0;
GPIO_PinState pushButton;
uint8_t lastNotePointer = 0x00;
uint8_t copylastNotePointer = 0x00;
uint8_t bufferByte ;
int flagForInit = 0;
int velocity[pitchSize];
uint8_t Notes[pitchSize];
uint8_t MSB = IDLE;
uint8_t msbForTest = IDLE;
int noteOnStatus = 0;
int byteNum = 0;
int numOfNoteOnNote = 0;
int currNoteOnByte = 0;
int currNoteOffByte = 0;
int endOfMessage = 0;
uint16_t totalStart = 0;
uint16_t totalStop = 0;
int counter = 0;
int i = 0;
int numOfSteps = 0;

int MIDI_ON_FLAG = 0;
int NUM_OF_MIDI_MESSAGE = 0;
uint8_t PITCH_VAL;

uint8_t currStatus = IDLE;
uint8_t currNoteOnRunnigStatus = IDLE;
uint8_t newNoteOnRunnigStatus = IDLE;
uint8_t newStatus = IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void TIM2_define(void);
void Led7SegmentInit(void);
void ScreenUpdate(void);
void BPMUpdate(void);
//void BPMUpdateVer2(void);
void PrintNumber(uint16_t number);
float FrequencyMultiplier(int EncValInt);
int selectNumOfSteps(int x);
uint8_t findAdd(uint8_t addToFind, uint8_t headPointer);
void uartToMidi (uint8_t v, uint8_t w);
void midiOn(uint8_t message);
void upDateLastNote(uint8_t e, int Mode);
void addNote(uint8_t o);
void moveNote(uint8_t NMN);
void noteOnCase();
void noteOffCase();
void defaultCase();
void idleToNoteOnStatus(uint8_t myBuffer);
void noteOnToNoteOnStatus(uint8_t myBufferStatus);
void runningStatus(uint8_t d);
void noteOffStatus(uint8_t g);
int readEncoder(int encoderValuToRead);
void setEncoderVal();
void togglePinForMUX(int selectorPin);
void turnOnMIDI(void);
uint32_t tuning(uint8_t note);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define selectpinsPort GPIOC
const int selectPins[4] = {S0_Pin,S1_Pin,S2_Pin,S3_Pin};
/*
  ******************************************************************************
  * Function Name: selectMuxPin
  * Parameter:
  * Input- 		   int pin
  * Output- 	   None
  * Description:   this function gets pin number between 0 to 15 and set or reset the selected pins
  ******************************************************************************
  */
void selectMuxPin(int pin)
{
	for (int i=0; i<4; i++)
	{
		if (pin & (1<<i))
			{HAL_GPIO_WritePin(selectpinsPort,selectPins[i],GPIO_PIN_SET);}
		else
			{HAL_GPIO_WritePin(selectpinsPort,selectPins[i],GPIO_PIN_RESET);}
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
	pointerToMidiBuff = pointerToMidiBuff + NumOfMessages;
	}

	//while(1);
	/*
	for(int i = 0;i <= 7 ;i++)
	{
		bufferRx[i] = bufferTx[i];
	}
	*/
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);// TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(GPIOC,EN_Pin,GPIO_PIN_RESET);
	for (int j=0; j<4; j++)
		HAL_GPIO_WritePin(selectpinsPort,selectPins[j],GPIO_PIN_SET);
  int t=0;
  int f = 0;
  uint8_t initNotes = 0x00;
  uint16_t pointerToBuff = 0;
  if (flagForInit == 0)
  {
	  flagForInit = 1;
	  for (t=0; t < pitchSize; t++)
	  {
		  velocity[t]=0;
	  }
	  for (f=0; f < pitchSize; f++)
	  {
		  Notes[f]= initNotes;
		  initNotes++;
	  }
  }
  //HAL_UART_Receive_IT(&huart1,bufferRx,7);
 // HAL_UART_Transmit_IT(&huart1,bufferTx,7);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Led7SegmentInit();
 // int i = 0;
  //int k = 0;
  //int numOfSteps = 0;
  uint8_t pitchVal= 0;
  uint8_t bufferByte ;
  //static int time = -1;
  float dutyCycle = 100.0/49.0;
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  uint8_t noteVal = lastNotePointer ^ 0x80;
  //htim2.Instance->CCR1 = dutyCycle+noteVal*dutyCycle;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	    //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
			//HAL_Delay(bpmToMiliSec);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
			HAL_Delay(bpmToMiliSec);
	  	  	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
			//HAL_Delay(bpmToMiliSec);


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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 36;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Period = 559;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
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
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 14;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S0_Pin|S1_Pin|S2_Pin|S3_Pin 
                          |EN_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin 
                           EN_Pin PC5 */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin 
                          |EN_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_Pin PB14 */
  GPIO_InitStruct.Pin = SW_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void TIM2_define(void)
{
       TIM_Base_InitTypeDef TIM_TimeBaseStruct;
//       NVIC_InitTypeDef NVIC_InitStruct;

       /* Peripheral clock enable */
       __HAL_RCC_TIM2_CLK_ENABLE();
       TIM_TimeBaseStruct.Prescaler = 71;
       TIM_TimeBaseStruct.Period = 999;
       TIM_TimeBaseStruct.ClockDivision = TIM_CLOCKDIVISION_DIV1;
       TIM_TimeBaseStruct.CounterMode = TIM_COUNTERMODE_UP;
//       TIM_TimeBaseStruct.RepetitionCounter = 0;
//
       TIM_Base_SetConfig(TIM2, &TIM_TimeBaseStruct);


//       TIM_CCxChannelCmd(TIMx, Channel, ChannelState)
       HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
       HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void noteOnCase()
{
	newStatus = NOTEON;
	numOfNoteOnNote++;
	currNoteOnByte = 1;
}
void noteOffCase()
{
	newStatus = NOTEOFF;
	currNoteOffByte++;
	numOfNoteOnNote--;
	currNoteOnByte = 0;
}
void defaultCase()
{
	if (currStatus == NOTEON && currNoteOnByte >= 1);
	{
		numOfNoteOnNote--;
	}
	currStatus == IDLE;
	currNoteOnRunnigStatus = IDLE;
	currNoteOnByte = 0;
}
void idleToNoteOnStatus(uint8_t bufferForData)
{

	if (currNoteOnByte == 1)
	{
		currNoteOnByte++;
		pitchVal = bufferForData;
	}
	else if (currNoteOnByte == 2)
	{
		currNoteOnByte--;
		if (bufferForData != VEL_ZERO) //TODO
		{
			velocity[pitchVal] = 1;
			addNote(pitchVal);
			currStatus = NOTEON;
			newStatus = IDLE;
			pitchVal = 0;
			endOfMessage = 1;
		}
		else
		{
			velocity[pitchVal] = 0;
			moveNote(pitchVal);
			numOfNoteOnNote--;
			if (numOfNoteOnNote <= 0)
			{
				numOfNoteOnNote = 0;
				currNoteOnByte = 0;
			}
			currStatus = IDLE;
			newStatus = IDLE;
			pitchVal = 0;
		}
	}
}
void noteOnToNoteOnStatus(uint8_t myBufferByte)
{
	if (currNoteOnByte == 1)
	{
		currNoteOnByte++;
		pitchVal = myBufferByte;
	}
	else if (currNoteOnByte == 2)
	{
		currNoteOnByte--;
		if (myBufferByte != VEL_ZERO)
		{
			if (velocity[pitchVal] == 1)
			{
				numOfNoteOnNote--;
			}
			if (numOfNoteOnNote < 0)
			{
				numOfNoteOnNote = 0;
			}
			velocity[pitchVal] = 1;
			addNote(pitchVal);
			newStatus = IDLE;
		}
		else
		{
			velocity[pitchVal] = 0;
			numOfNoteOnNote = numOfNoteOnNote - 2;
			if (numOfNoteOnNote <= 0)
			{
				numOfNoteOnNote = 0;
				currNoteOnByte = 0;
			}
			moveNote(pitchVal);
			newStatus = IDLE;
		}
		pitchVal = 0;
	}
}
void runningStatus(uint8_t newBufferByte)
{
	if (currNoteOnByte == 1)
	{
		currNoteOnByte++;
		pitchVal = newBufferByte;
	}
	else if (currNoteOnByte == 2)
	{
		currNoteOnByte--;
		if (newBufferByte != VEL_ZERO)
		{
			if (velocity[pitchVal] == 0)
			{
				numOfNoteOnNote++;
				velocity[pitchVal] = 1;
			}
			addNote(pitchVal);
		}
		else
		{
			velocity[pitchVal] = 0;
			moveNote(pitchVal);
			numOfNoteOnNote--;
			if (numOfNoteOnNote <= 0)
			{
				numOfNoteOnNote = 0;
				currNoteOnByte = 0;
				currStatus = IDLE;
			}
		}
		pitchVal = 0;
	}
}
void noteOffStatus(uint8_t noteOffBuffer)
{
	if (currNoteOffByte == 1)
	{
		currNoteOffByte++;
		pitchVal = noteOffBuffer;
	}
	else
	{
		currNoteOffByte = 0;
		velocity[pitchVal] = 0;
		currStatus = IDLE;
		newStatus = IDLE;
		moveNote(pitchVal);
		pitchVal = 0;
	}
}

void turnOnMIDI(void)
{
	uint8_t pitch = 0;
	pushButton = HAL_GPIO_ReadPin(GPIOB, SW_Pin);
	  if (pushButton == GPIO_PIN_RESET)
	  {
		  if (huart1.RxXferCount == 0)
		  {
			  totalStop = 0;
			  totalStart = 0;
			  for (int y= 0; y < 100; y++)
			  {
				  midiBuffer[y] = 0;
				  CNT[y] = 0;
			  }
		  }
		  HAL_UART_Receive_IT(&huart1, midiBuffer, 100);
		  	  if (totalStop != 100 - huart1.RxXferCount)
		  	  {
		  		totalStop = 100 - huart1.RxXferCount;
		  		for(int k =totalStart; k <= totalStop; k++)
					  {
						bufferByte = midiBuffer[k];
						if(MIDI_ON_FLAG)
						{
							MIDI_ON_FLAG = 0;
							pitch = (midiBuffer[k]);
							htim4.Instance->CCR3 = tuning(pitch);
							CNT[k] = htim4.Instance->CCR3;
							CNT[99] = CNT[99]+1;
							break;
						}

						if (bufferByte == 0x90)
							{

								htim4.Instance->CCR4 = 10000;
								if (midiBuffer[k+1] == 0)

									{
									MIDI_ON_FLAG = 1;
									totalStart = k+1;
									break;
									}

								else
								{
									pitch = (midiBuffer[k+1]);
									htim4.Instance->CCR3 = tuning(pitch);
									CNT[k+1] = htim4.Instance->CCR3;
									CNT[99] = CNT[99]+1;
								}

							}

						if (bufferByte == 0x80)
						  {
							htim4.Instance->CCR4 = 0;
						  }
						totalStart = totalStop;
					  }
		  	  }
	  }


}
uint32_t tuning(uint8_t note)
{
	uint32_t tune = 0;
 if (note % 36 <= 12)
 {
	 tune = (note-36)*141.6667;
 }
 else if (note % 36 <= 24)
  {
 	 tune = 1700 + (note-48)*141.67;
  }
 else
	 tune = 3400 + (note-60)*141.667;
 return tune;
}

/*
 * *****************************************************************************
 * Function Name: uartToMidi
 * Parameters:
 * Input- 		  uint8_t theMSB = DATA or Status, uint8_t the Buffer = current byte we got
 * Output-        NONE
 * Description:	  translate the data from UART format to MIDI protocol again
 ******************************************************************************
 */
void midiOn(uint8_t message)
{
	// TurnOn message
	if (message == 0x90)
	{
		MIDI_ON_FLAG = 1;
		NUM_OF_MIDI_MESSAGE++;
	}
	// TurnOff message
	else if (message == 0x80)
		MIDI_ON_FLAG = 0;
	// velocity message or pitch value
	else
	{
		if (NUM_OF_MIDI_MESSAGE)
			PITCH_VAL = message;

	}

}
/*
 * *****************************************************************************
 * Function Name: uartToMidi
 * Parameters:
 * Input- 		  uint8_t theMSB = DATA or Status, uint8_t the Buffer = current byte we got
 * Output-        NONE
 * Description:	  translate the data from UART format to MIDI protocol again
 ******************************************************************************
 */
void uartToMidi(uint8_t theMSB,uint8_t theBuffer)
{
	if (theMSB)
	{
		switch (theBuffer)
		{
			case NOTEON:
			{
				flagForNotStatus = 1;
				noteOnCase();
				break;
			}
			case NOTEOFF:
			{
				flagForNotStatus = 1;
				noteOffCase();
				break;
			}
			default:
				//if((theBuffer >= CC_MIN) & (theBuffer <= CC_MAX))
				flagForNotStatus = 0;
					//defaultCase();
		}
	}
	else
	{
		if(~flagForNotStatus)
			endOfMessage = 1;
		if ((newStatus == NOTEON) && (currStatus != NOTEON))
			idleToNoteOnStatus(theBuffer);
		if ((newStatus == NOTEON) & (currStatus == NOTEON) & (endOfMessage != 1))
			noteOnToNoteOnStatus(theBuffer);
		else if ((newStatus == IDLE) & (currStatus == NOTEON) & (endOfMessage != 1))
			runningStatus(theBuffer);
		else if (newStatus == NOTEOFF)
			noteOffStatus(theBuffer);
	}
}

/*
 * *****************************************************************************
  * Function Name: findAdd
 * Parameters:
 * Input- 		  uint8_t valToAdd = the value to find,uint8_t pinter list = hold the curr note played
 * Output-        uint8_t currAdd = the address witch hold the value in Notes array
 * Description:	  this function gets note value and find the address in the array witch hold the note in recursive TODO: check this function
 ******************************************************************************
 */
uint8_t findAdd(uint8_t valToAdd, uint8_t pointerToList)
{
	uint8_t currAdd = pointerToList;
	int stopForCase = 0;
	while ((Notes[0x7F & currAdd]) != (0x80 ^ valToAdd) )
	{
		currAdd = Notes[0x7F & currAdd];
		stopForCase++;
		if (stopForCase == 128)
			break;
	}
	return currAdd;
}
/*
 * *****************************************************************************
  * Function Name: addNote
 * Parameters:
 * Input- 		  uint8_t noteOn = new note to add to nodes
 * Output-        NONE
 * Description:	  this function gets note value and update the note to be the pointer list as well init Notes array
 ******************************************************************************
 */
void addNote(uint8_t noteOn)
{
	uint8_t statusNoteON;
	if ((lastNotePointer & 1 << 7))
	{
		if (Notes[noteOn] != noteOn)
			moveNote(noteOn);
		if (lastNotePointer == 0x00)
		{
			Notes[noteOn] =  0x80 ^ noteOn;
			lastNotePointer = 0x80 ^ noteOn;
		}
		else
		{
			Notes[noteOn] = lastNotePointer;
			lastNotePointer = 0x80 ^ noteOn;
		}
	}
	else
	{
		Notes[noteOn] =  0x80 ^ noteOn;
		lastNotePointer = 0x80 ^ noteOn;
	}
}
/*
 * *****************************************************************************
  * Function Name: moveNote
 * Parameters:
 * Input- 		  uint8_t moveNoteOn =  note add. to move from Notes array
 * Output-        NONE
 * Description:	  this function gets note value and move it from the list
 ******************************************************************************
 */
void moveNote(uint8_t moveNoteOn)
{
	uint8_t status;
	// if we are NoteOn
	if ((lastNotePointer & 1 << 7))
	{
		// if we are moving the last note
		if (lastNotePointer == (0x80 ^ moveNoteOn))
		{
			//if the last note is also the first note
			if (Notes[moveNoteOn] == (0x80 ^ moveNoteOn))
			{
				Notes[moveNoteOn] = moveNoteOn;
				copylastNotePointer = lastNotePointer ;
				lastNotePointer = 0x00;//TODO find default value
			}
			//the last note we are moving is not the only noteOn note
			else
			{
				lastNotePointer = Notes[moveNoteOn];
				Notes[moveNoteOn] = moveNoteOn;
			}
		}
		//if the note is the first note
		else if (Notes[moveNoteOn] == (0x80 ^ moveNoteOn))
		{
			Notes[moveNoteOn] = Notes[moveNoteOn] ^ 0x80;
			uint8_t u = 0x00;
//			u = findAdd((0x80 ^ moveNoteOn),lastNotePointer);
			//fond witch node hold the note addres
			for (int r = 0; r <= pitchSize; r++)
			{
				if (Notes[r] == (0x80 ^ moveNoteOn))
				{
					Notes[r] = u ^ 0x80;
					break;
				}
				u++;
			}
			u = 0x00;
		}
		//if this is a central note
		else
		{
			//find the node that hold the note address
			uint8_t u = 0x00;
	//		u = findAdd((0x80 ^ moveNoteOn),lastNotePointer);
			for (int r = 0; r <= pitchSize; r++)
				{
					if (Notes[r] == (0x80 ^ moveNoteOn))
					{
						Notes[r] = Notes[moveNoteOn];
						break;
					}
					u++;
				}
			Notes[moveNoteOn] = moveNoteOn;
		}

	}
	// if we are Noteoff
	else
	{
		//TODO
	}
}
/*
 * *****************************************************************************
  * Function Name: FrequencyMultiplier
 * Parameters:
 * Input- 		  EncVal = encoder value for the frequency multiplier
 *  					   value 0-2,3-5,6-8,9-11,12-14,15-17,18-20,21-23
 * Output-        currFreqMul = current value for the frequency multiplier
 * 								value: 1/2/3/4/8/16/0.5/0.25
 * Description:	  this function calculate the encoder value to translate for current frequency
 ******************************************************************************
 */
float FrequencyMultiplier(int EncoderVal)
{
	float currFreqMul = 1.0;
	if (EncoderVal == 0)
	{
		currFreqMul  = 1;
	}
	else if (EncoderVal == 1)
	{
		currFreqMul = 2;
	}
	else if (EncoderVal == 2)
	{
		currFreqMul = 3;
	}
	else if (EncoderVal == 3)
	{
		currFreqMul = 4;
	}
	else if (EncoderVal == 4)
	{
		currFreqMul = 8;
	}
	else if (EncoderVal == 5)
	{
		currFreqMul = 16;
	}
	else if (EncoderVal == 6)
	{
		currFreqMul = 0.5;
	}
	else if (EncoderVal == 7)
	{
		currFreqMul = 0.25;
	}
	return currFreqMul;
}

/*
float FrequencyMultiplier(int EncVal)
{
	float currFreqMul = 1.0;
	if (EncVal < FREQ_MUL_TWO)
			{
			currFreqMul  = 1;
			}
		else if ((EncVal >= FREQ_MUL_TWO) & (EncVal < FREQ_MUL_THREE))
			{
			currFreqMul = 2;
			}
		else if ((EncVal >= FREQ_MUL_THREE) & (EncVal < FREQ_MUL_FOUR))
			{
			currFreqMul = 3;
			}
		else if ((EncVal >= FREQ_MUL_FOUR) & (EncVal < FREQ_MUL_EIGHT))
			{
			currFreqMul = 4;
			}
		else if ((EncVal >= FREQ_MUL_EIGHT) & (EncVal < FREQ_MUL_SIXT))
			{
			currFreqMul = 8;
			}
		else if ((EncVal >= FREQ_MUL_SIXT) & (EncVal < FREQ_MUL_HALF))
			{
			currFreqMul = 16;
			}
		else if ((EncVal >= FREQ_MUL_HALF) & (EncVal < FREQ_MUL_QU))
			{
			currFreqMul = 0.5;
			}
		else if ((EncVal >= FREQ_MUL_QU))
			{
			currFreqMul = 0.25;
			}
	return currFreqMul;
}
*/
/*
 * *****************************************************************************
  * Function Name: selectNumOfSteps
 * Parameters:
 * Input- 		  int x =  the encoder value to count number of steps in the sequencer
 * Output-        int 16/2/3/4/5/8 = number of steps
 * Description:	  this function calculate the encoder value to translate for number of steps
 ******************************************************************************
 */
int selectNumOfSteps(int x)
{

	if (x == 0)
		return 16;
	else
		return x;
}
  /*
  ******************************************************************************
  * Function Name: Led7SegmentInit
  * Parameters:
  * Input- None
  * Output- None
  * Description:   this function Configure the pins for the led 7-seg screen
  ******************************************************************************
  */
void Led7SegmentInit()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_GPIO_DIG();
	GPIO_InitStruct.Pin = GPIO_PIN_DIG_0 | GPIO_PIN_DIG_1 | GPIO_PIN_DIG_2 | GPIO_PIN_DIG_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIO_DIG, &GPIO_InitStruct);

	RCC_GPIO_SEG();
	GPIO_InitStruct.Pin = GPIO_PIN_SEG_A | GPIO_PIN_SEG_B | GPIO_PIN_SEG_C | GPIO_PIN_SEG_D |
			GPIO_PIN_SEG_E | GPIO_PIN_SEG_F | GPIO_PIN_SEG_G;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIO_SEG, &GPIO_InitStruct);

//	__HAL_RCC_GPIOC_CLK_ENABLE();
//	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
/*
  ******************************************************************************
  * Function Name: selectMuxPinVer2
  * Parameter:
  * Input- 		   int pin
  * Output- 	   None
  * Description:   this function gets pin number between 0 to 15 and set or reset the selected pins
  ******************************************************************************
  */
/*
void selectMuxPinVer2(int pinForMux)
{
	for (int i=0; i<4; i++)
	{
		if (pin & (1<<i))
			{HAL_GPIO_WritePin(selectpinsPort,selectPins[i],GPIO_PIN_SET);}
		else
			{HAL_GPIO_WritePin(selectpinsPort,selectPins[i],GPIO_PIN_RESET);}
	}
}
*/
/*
  ******************************************************************************
  * Function Name: ScreenUpdate
  * Parameters:
  * Input- None
  * Output- None
  * Description:   this function update the 7-segments using font array witch hold a MASK for each digit.
  *  			   when we get TIM2 interrupt every 1ms, we update the screen accordingly
  ******************************************************************************
  */
void ScreenUpdate()
{
	static uint8_t digit = 0;

	GPIO_DIG->ODR = (1 << ((SS_DIGIT - 1 - digit) + 8)) & 0x0F00;

	// We black out the Most Significant Digit
	if (digit == 3) {
		GPIO_SEG->ODR = 0xFF;
	} else {
	GPIO_SEG->ODR = (~font[digits[digit]] << 1) & 0xFF;
	}

	digit = (digit + 1) % SS_DIGIT;
}
/*
  ******************************************************************************
  * Function Name: readEncoder
  * Parameters:
  * Input- None
  * Output- None
  * Description:
  ******************************************************************************
  */
int readEncoder(int timNum)
{
	if (timNum == 1)
	{
		return TIM1->CNT;
	}
	if (timNum == 3)
	{
		return TIM3->CNT;
	}
	if (timNum == 5)
	{
		return TIM5->CNT;
	}
}
/*
  ******************************************************************************
  * Function Name: setEncoderVal
  * Parameters:
  * Input- None
  * Output- None
  * Description:   cal bpm to 7-seg, number of steps, freqmul
  ******************************************************************************
  */
void setEncoderVal()
{
	numOfSteps = readEncoder(1);
	encoderValForSteps  = selectNumOfSteps(numOfSteps/4);
	i = readEncoder(3);
	bpm= 60 + i/4;
	PrintNumber(bpm);

	EncVal = readEncoder(5);//(TIM5->CNT);
	freqMul = FrequencyMultiplier(EncVal/2);

	bpmval = bpm*freqMul;
	bpmToMiliSec  = (1 / (bpmval / 60.0 / 1000.0))*0.5;
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
	//HAL_Delay(bpmToMiliSec);


}

/*
  ******************************************************************************
  * Function Name: BPMUpdate
  * Parameters:
  * Input- None
  * Output- None
  * Description:   this function holds uint16_t t parameter witch update every 1 ms
  * 			   and using TIM2 to change the Mux selector key.
  * 			   K parameter is the selector Pin witch change with modulo 16
  ******************************************************************************
  */
void BPMUpdate()
{
	static uint16_t t = 0; // time is in ms

	static int k = 0;
	t++; // update is being called each 1 ms

	if (t >= 1 / ((bpm*freqMul)/ 60.0 / 1000.0))
	{

		k = (k + 1) % encoderValForSteps;
		t = 0;
	}
	selectMuxPin(k);
}

void togglePinForMUX(int currPin)
{
	if (currPin % 2 == 1)
	{
		HAL_GPIO_TogglePin(selectpinsPort,S0_Pin);
	}
	if (currPin % 4 == 3)
	{
		HAL_GPIO_TogglePin(selectpinsPort,S1_Pin);
	}
	if (currPin % 8 == 7)
	{
		HAL_GPIO_TogglePin(selectpinsPort,S2_Pin);
	}
if (currPin % 16 == 15)
	{
	HAL_GPIO_TogglePin(selectpinsPort,S3_Pin);
	}
//S0_Pin,S1_Pin,S2_Pin,S3_Pin;
	//static int pinSelctor = 0;
	//selectMuxPin(pinSelctor);
	//pinSelctor = (pinSelctor + 1) % encoderValForSteps;

}

/*
  ******************************************************************************
  * Function Name: PrintNumber
  * Parameter:
  * Input- uint16_t number
  * Output- None
  * Description:   this function divides the number to array by the power
  ******************************************************************************
  */
void PrintNumber(uint16_t number)
{
	if (number > 9999)
	{
		number = 9999;
	}
	digits[3] = number / 1000;
	digits[2] = number % 1000 / 100;
	digits[1] = number % 100 / 10;
	digits[0] = number % 10;
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
