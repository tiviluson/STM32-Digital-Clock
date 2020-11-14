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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define threshold1 100
#define threshold2 120
#define step1 50
#define step2 10
#define MODIFIED 61
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
enum modeEnum{init, Mode0, Mode1, Mode2, Mode3, Mode4};
enum buttonEnum{notPressed, pressed, pressedMoreThan1s};
enum buttonEnum buttonMode = notPressed /*, buttonAdjust = notPressed;*/;
enum buttonStateEnum{reset, pressLessThan1s, pressAtLeast1s};
enum buttonStateEnum buttonState = reset;
enum modeEnum mode = init;
uint32_t globalCount = 0;
uint8_t buf[50];
char uart_buf[50];
int uart_buf_len;

uint32_t hour = 0; //0-23     //normalclock
uint32_t minute = 0; //0-59
uint32_t second = 0; //0-59s

uint8_t isSetHour = 0;
uint8_t isSetMinute = 0;
uint8_t isSetSecond = 0;
uint32_t countHour = 0;  // for counting hour,minute, second in setTime
uint32_t countMinute = 0;  // for counting hour,minute, second in setTime
uint32_t countSecond = 0;  // for counting hour,minute, second in setTime
uint32_t hourModified = 0;
uint32_t minuteModified = MODIFIED;
uint32_t secondModified = MODIFIED;

enum normalClockEnum{startNormalClock, runNormalClock, secondRestart, minuteRestart};
enum normalClockEnum normalClockState = startNormalClock;
__IO uint8_t firstRead = 0;
__IO uint8_t secondRead = 0;
uint8_t unchange = 0;
uint8_t pressing = 0;
uint8_t isPressedOneTime = 0;
uint8_t isPressedMoreThan1s = 0;
uint32_t count1 = 0;
uint32_t count2 = 0;
uint32_t count = 0;

enum stopWatchEnum{initStopWatch, increaseHund, increaseSec, increaseMin};
enum stopWatchEnum stopWatchState = initStopWatch;

__IO uint8_t firstReadMode = 0;
__IO uint8_t secondReadMode = 0;
uint32_t stopMin = 0; // Minute value of the stopwatch
uint32_t stopSec = 0; // Second value of the stopwatch
uint32_t stopHun = 0; // Hundredth second value of the stopwatch
uint8_t stopWatchPause = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void readButton(void);
void printCounter(void);
void TurnOnLedBlue(void);
void TurnOffLedBlue(void);
void TurnOnLedRed(void);
void TurnOnLedRed(void);
void SetGlobalMode(void);

void updateTime(void); //mode 0
void printTimeAdjust(void);
void setHour(void); //mode 1
void setMinute(void); //mode 2
void setSecond(void); //mode 3
void stopWatch(void); //mode 4

void displayClock(void); //mode 0
void normalClock(void);
void printClock(uint32_t hour, uint32_t minute, uint32_t second);
void readButtonAdjust(void);

void readButtonMode(void);
void printStopWatchTime(void);
void stopWatch(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TurnOnLedBlue(void) {
	if (globalCount % 40 > 20)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void TurnOffLedBlue(void) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

void TurnOnLedRed(void) {
	if (globalCount % 40 > 20)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void TurnOffLedRed(void) {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}

void TurnOnLedYellow(void) {
	if (globalCount % 40 > 20)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void TurnOffLedYellow(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void updateTime(void) {
	if(isSetHour) {
			hour = (countHour + hourModified) % 24;
			isSetHour = 0;
			countHour = 0;
			}
	if(isSetMinute) {
			minute = (countMinute + minuteModified) % 60;
			isSetMinute = 0;
			countMinute = 0;
			}
	if(isSetSecond) {
			second = (countSecond + secondModified) % 60;
			isSetSecond = 0;
			countSecond = 0;
			}
}

void printTimeAdjust(void){
	uart_buf_len = sprintf(uart_buf, "%lu:%lu:%lu\r\n",
							(countHour+hourModified)%24,
							(countMinute+minuteModified)%60,
							(countSecond+secondModified)%60);
	HAL_UART_Transmit(&huart3, (uint8_t *) uart_buf, uart_buf_len, 100);
}

void setHour(void){
	if (!isSetHour){
		hourModified = hour;
		minuteModified = (minuteModified == MODIFIED)?minute:minuteModified;
		secondModified = (secondModified == MODIFIED)?second:secondModified;
		isSetHour = (countHour!=0);
	}

	if (isPressedOneTime) {
		countHour++; printTimeAdjust();
	}

	if (isPressedMoreThan1s) {
		countHour++;
		printTimeAdjust();
	}
}

void setMinute(void){
	if(!isSetMinute){
		isSetMinute = (countMinute!=0);
	}

	if(isPressedOneTime) {countMinute++; printTimeAdjust();}

	if(isPressedMoreThan1s) {
			countMinute++;
			printTimeAdjust();
	}
}

void setSecond(void){
	if(!isSetSecond){
		isSetSecond = (countSecond!=0);
	}

	if(isPressedOneTime) {countSecond++; printTimeAdjust();}

	if(isPressedMoreThan1s) {
			countSecond++;
			printTimeAdjust();
	}
}

void displayClock(void)
{
	static uint8_t period = 0;
	if (++period==100)
	{
		period = 0;
		normalClock();
		if (mode == Mode0) printClock(hour, minute, second);
	}
}

void normalClock(void) {
	switch(normalClockState) {
	case startNormalClock:
		hour = 21;
		minute = 30;
		second = 0;
		normalClockState = runNormalClock;
		break;
	case runNormalClock:
		second++;
		if (second == 60) {
			second = 0;
			normalClockState = secondRestart;
		} else {
			normalClockState = runNormalClock;
			break;
		}
	case secondRestart:
		minute++;
		if (minute == 60) {
			minute = 0;
			normalClockState = minuteRestart;
		} else {
			normalClockState = runNormalClock;
			break;
		}
	case minuteRestart:
		hour++;
		normalClockState = runNormalClock;
		if (hour == 24) hour = 0;
		break;
	}
}

void printClock(uint32_t hour, uint32_t minute, uint32_t second) {
	uart_buf_len = sprintf(uart_buf, "%lu:%lu:%lu\r\n", hour,minute,second);
	HAL_UART_Transmit(&huart3, (uint8_t *) uart_buf, uart_buf_len, 100);
}

void printStopWatchTime(void) {
	uart_buf_len = sprintf(uart_buf, "Min: %lu\tSec: %lu.%lu\r\n", stopMin, stopSec, stopHun);
	HAL_UART_Transmit(&huart3, (uint8_t *) uart_buf, uart_buf_len, 100);
}

void stopWatch(void) {
	if (isPressedOneTime) {
		stopWatchPause = !stopWatchPause;
	}
	if (!stopWatchPause) {
		switch (stopWatchState) {
		case initStopWatch: // Initialize values of stopWatch when change to mode 4
			stopMin = 0;
			stopSec = 0;
			stopHun = 0;
			stopWatchState = increaseHund;
			break;
		case increaseHund: // Increase the hundredth second value from 0 to 99
			stopHun++;
			if (stopHun == 100) {
				stopHun = 0;
				stopSec++;
				stopWatchState = increaseSec;
				printStopWatchTime();
			} else {
				stopWatchState = increaseHund;
				printStopWatchTime();
				break;
			}
		case increaseSec: // Increase the second value from 0 to 59
			if (stopSec == 60) {
				stopHun = 0;
				stopSec = 0;
				stopMin++;
				stopWatchState = increaseMin;
				printStopWatchTime();
			} else {
				stopWatchState = increaseHund;
				printStopWatchTime();
				break;
			}
		case increaseMin: // Increase the min value from 0 to 59
			if (stopMin == 60) {
				stopHun = 1;
				stopSec = 0;
				stopMin = 0;
			} else {
				stopWatchState = increaseHund;
			}
			printStopWatchTime();
			break;
		}
	}
}

void readButtonMode(void) {
	firstReadMode = secondReadMode;
	secondReadMode = !(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin));
	if (!firstReadMode && secondReadMode) {
		buttonMode = pressed;
	} else {
		buttonMode = notPressed;
	}
}

void readButtonAdjust(void) {
	firstRead = secondRead;
	secondRead = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));

	unchange = (secondRead == firstRead);
	isPressedOneTime = !unchange && !pressing;
	pressing = unchange && firstRead;

	switch(buttonState) {
	case reset:
		count1 = 0;
		count2 = 0;
		if (pressing) buttonState = pressLessThan1s;
		else buttonState = reset;
		break;
	case pressLessThan1s:
		count1++; //first pseudo-counter for the first time interval (first 1s)
		if (!pressing) buttonState = reset;
		else if (count1 == threshold1) buttonState = pressAtLeast1s;
		else buttonState = pressLessThan1s;
		break;
	case pressAtLeast1s:
		count2++;
		if (!pressing) buttonState = reset;
		else {
			if (count2==(threshold2-threshold1)) {
				isPressedMoreThan1s = 1;
				count2 = 0;
			} else {
				isPressedMoreThan1s = 0;
			}
		}
		break;
	}
}


/*void printCounter(void) {
	uart_buf_len = sprintf(uart_buf, "%lu\r\n", count);
	HAL_UART_Transmit(&huart3, (uint8_t *) uart_buf, uart_buf_len, 100);
}

void checkReadButton(void)
{
	if (isPressedOneTime) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
		count++;
		printCounter();
	}
	if (isPressedMoreThan1s) {
		count++;
		printCounter();
	}
}

void readButton(void) {
	firstRead = secondRead;
	secondRead = !(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));

	unchange = (secondRead == firstRead);
	isPressedOneTime = !unchange && !pressing;
	pressing = unchange && firstRead;

	switch(state) {
	case reset:
		count1 = 0;
		count2 = 0;
		if (pressing) state = pressLessThan1s;
		else state = reset;
		break;
	case pressLessThan1s:
		count1++; //first pseudo-counter for the first time interval (first 1s)
		if (!pressing) state = reset;
		else if (count1 == threshold1) state = pressAtLeast1s;
		else state = pressLessThan1s;
		break;
	case pressAtLeast1s:
		count2++;
		if (!pressing) state = reset;
		else {
			if (count2==(threshold2-threshold1)) {
				isPressedMoreThan1s = 1;
				count2 = 0;
			} else {
				isPressedMoreThan1s = 0;
			}
		}
		break;
	}
}*/

void SetGlobalMode(void) {
	globalCount++;
	readButtonMode();
	readButtonAdjust();
	switch(mode) {
	case init:
		TurnOffLedYellow();
		TurnOffLedRed();
		TurnOffLedBlue();
		//reset all other global variables
		mode = Mode0;
		break;
	case Mode0:
		//displayClock();
		//printClock(hour, minute, second);
		if (buttonMode == pressed) mode = Mode1;
		else mode = Mode0;
		break;
	case Mode1:
		setHour();
		TurnOnLedYellow();
		if (buttonMode == pressed) mode = Mode2;
		else mode = Mode1;
		break;
	case Mode2:
		setMinute();
		TurnOffLedYellow();
		TurnOnLedBlue();
		if (buttonMode == pressed) mode = Mode3;
		else mode = Mode2;
		break;
	case Mode3:
		setSecond();
		TurnOffLedBlue();
		TurnOnLedRed();
		if (buttonMode == pressed) mode = Mode4;
		else mode = Mode3;
		break;
	case Mode4:
		stopWatch();
		TurnOffLedRed();
		if (buttonMode == pressed) {
			stopWatchPause = 0;
			updateTime();
			mode = Mode0;
		}
		else mode = Mode4;
		break;
	}
	//uart_buf_len = sprintf(uart_buf, "%d\r\n", mode);
	//HAL_UART_Transmit(&huart3, (uint8_t *) uart_buf, uart_buf_len, 100);

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
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 9600-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim16)
	{
		displayClock();
		SetGlobalMode();
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
