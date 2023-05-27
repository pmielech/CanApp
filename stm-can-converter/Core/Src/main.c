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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define UART_RX_BUFFER 20
#define MSG_BUFFER_SIZE 8
#define HALF_MSG_BUFFER 4
typedef enum{
    SPEED_125_KBITS,
    SPEED_250_KBITS,
    SPEED_500_KBITS,
    SPEED_1000_KBITS

}can_speed_t;

typedef enum{
    RESET_SYSTEM,
    SET_CAN_SPEED

}master_command_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef Button_TxHeader;

uint32_t TxMailbox;

uint16_t uartToCanMsg_ID = 0;
uint8_t uartToCanMsg_DLC = 0;
uint8_t uartToCanMsg_Data[8] = {0};

uint16_t canToUartMsg_ID = 0;
uint8_t canToUartMsg_DLC = 0;
uint8_t canToUartMsg_Data[8] = {0};

uint8_t TxData[8];
uint8_t RxData[8];
uint8_t uartMessageState = 0u;
uint8_t led_state = 0;
uint8_t can_status = 0;
uint8_t uart_status = 0;
uint8_t button_msg[8] = {0};

uint8_t UartRxBuffer[UART_RX_BUFFER] = {0};
uint8_t ReceiveBuf[UART_RX_BUFFER];
uint8_t OperationalBuf[UART_RX_BUFFER];
uint8_t canSpeedOnStartup __attribute__ ((section (".no_init")));;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

void vPrint_message(){
	printf("%04x ", canToUartMsg_ID);
	printf("%02x ", canToUartMsg_DLC);
	for(int i = 0; i < sizeof(canToUartMsg_Data); i++)
		{
			printf("%02x ", canToUartMsg_Data[i]);
		} printf("\n\r");
	}

void vConvertToCan(uint16_t *size){
	uint8_t buildID[HALF_MSG_BUFFER] = {0};
	for(int j=0; j < HALF_MSG_BUFFER; j++){
		buildID[j] = OperationalBuf[j];
	} sscanf(buildID, "%04x", &uartToCanMsg_ID);
	for(uint8_t i=0u; i < ((uint)size-6)/4; i++){
			if(i == 0){
				uint8_t buildDLC[2] = {0};
				for(int j=0; j < 2; j++){
					buildDLC[j] = OperationalBuf[j+5];
					}
				sscanf(buildDLC, "%02x", &uartToCanMsg_DLC);
			} else if(i >= 1){

				for(int j=0; j < uartToCanMsg_DLC; j++){
					uint8_t buildByte[2] = {0};
					for(int c=0; c < 2; c++){
						buildByte[j+c] = OperationalBuf[c+8];
					} sscanf(buildByte, "%02x", &uartToCanMsg_Data[j]);
					}
				break;
			}
	}

	TxHeader.DLC = uartToCanMsg_DLC;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = uartToCanMsg_ID;
	TxHeader.TransmitGlobalTime = DISABLE;
	memset(OperationalBuf, 0, UART_RX_BUFFER);
	can_status += HAL_CAN_AddTxMessage(&hcan, &TxHeader, uartToCanMsg_Data, &TxMailbox);

}

void resetUartDmaRxBuffer(UART_HandleTypeDef  *huart, uint16_t Size) {
    __HAL_DMA_DISABLE(huart->hdmarx);
    huart->hdmarx->Instance->CNDTR = Size; // reset counter
    __HAL_DMA_ENABLE(huart->hdmarx);
}


void vMasterCommand(uint8_t cmd, uint8_t value){

    switch (cmd) {
        case RESET_SYSTEM:
            NVIC_SystemReset();
            break;
        case SET_CAN_SPEED:
            if(value >= 0 && value <= 3){
            	canSpeedOnStartup = value;
            }
            break;
        default:
            break;
    }

    canToUartMsg_ID = cmd;
    canToUartMsg_DLC = 0x01;
    canToUartMsg_Data[0] = value;
    vPrint_message();

}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	static uint8_t callbackHandler = 0;

	/*
	static uint8_t messageReceived = 0;

	if(huart->Instance == USART2){

		if(strstr((const char *)UartRxBuffer, (const char *)'\n')){

			if(strlen((const char *)UartRxBuffer) > 10){
					memcpy(OperationalBuf, UartRxBuffer, UART_RX_BUFFER);
					memset(UartRxBuffer, 0, UART_RX_BUFFER);
					vConvertToCan(&Size);
					messageReceived = 1;

				} else if(strlen((const char *)UartRxBuffer) >= 4 && strstr((const char *)UartRxBuffer, (const char *)'F')){
					memcpy(OperationalBuf, UartRxBuffer, UART_RX_BUFFER);
					memset(UartRxBuffer, 0, UART_RX_BUFFER);
					uint8_t master_counter = 0;
					for(uint8_t i = 0u; i < HALF_MSG_BUFFER; i++){
						if(OperationalBuf[i] == 'F'){
							master_counter++;
						}
					}
					if(master_counter == 2){
						vMasterCommand(OperationalBuf[2] - '0', OperationalBuf[3]-'0');
						messageReceived = 1;

					}

					}

				} else{
					callbackHandler++;

				}

		} else{

		callbackHandler++;
		}

		if(messageReceived){
			callbackHandler = 0;
			resetUartDmaRxBuffer(&huart2, UART_RX_BUFFER);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
			messageReceived = 0;

		}

		if(callbackHandler > UART_RX_BUFFER -1){
			memset(UartRxBuffer, 0, UART_RX_BUFFER);
			resetUartDmaRxBuffer(&huart2, UART_RX_BUFFER);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		}

*/

	if(huart->Instance == USART2 && strlen(UartRxBuffer) > 10 && strstr(UartRxBuffer, "\n\0")){
		callbackHandler = 0;
		memcpy(OperationalBuf, UartRxBuffer, UART_RX_BUFFER);
		memset(UartRxBuffer, 0, UART_RX_BUFFER);
		vConvertToCan(&Size);

		memset(OperationalBuf, 0, UART_RX_BUFFER);
		resetUartDmaRxBuffer(&huart2, UART_RX_BUFFER);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	} else if(huart->Instance == USART2 && strlen(UartRxBuffer) >= 4 && strstr(UartRxBuffer, "\n\0") && strstr(UartRxBuffer, "F\0")){
		callbackHandler = 0;
		memcpy(OperationalBuf, UartRxBuffer, UART_RX_BUFFER);
		memset(UartRxBuffer, 0, UART_RX_BUFFER);
		if(strlen(OperationalBuf) >= 10){
			vConvertToCan(&Size);
		} else {
			vMasterCommand(OperationalBuf[2] - '0', OperationalBuf[3]-'0');

		}
		memset(OperationalBuf, 0, UART_RX_BUFFER);
		resetUartDmaRxBuffer(&huart2, UART_RX_BUFFER);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	}


	else {
		callbackHandler++;
	}

	if(callbackHandler > UART_RX_BUFFER-1){
		memset(OperationalBuf, 0, UART_RX_BUFFER);
		memset(UartRxBuffer, 0, UART_RX_BUFFER);
		resetUartDmaRxBuffer(&huart2, UART_RX_BUFFER);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}


}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can_status += HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(can_status == 0){

		canToUartMsg_ID = RxHeader.StdId;
		canToUartMsg_DLC= RxHeader.DLC;

		if (canToUartMsg_DLC != 0) {
			int i;
			for(i=0; i < sizeof(canToUartMsg_Data); i++){
				canToUartMsg_Data[i] = RxData[i];
			}
		} else{
			memset(canToUartMsg_Data, 0, 8);
		}
		vPrint_message();

	}
}

void vCan_messages_init(){

	Button_TxHeader.DLC = 0x01;
	Button_TxHeader.ExtId = 0;
	Button_TxHeader.IDE = CAN_ID_STD;
	Button_TxHeader.RTR = CAN_RTR_DATA;
	Button_TxHeader.StdId = 0x350;
	Button_TxHeader.TransmitGlobalTime = DISABLE;

	TxHeader.DLC = 0x00;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x500;
	TxHeader.TransmitGlobalTime = DISABLE;



}
void vButton_message(){
	button_msg[0] = led_state;
	can_status += HAL_CAN_AddTxMessage(&hcan, &Button_TxHeader, button_msg, &TxMailbox);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BLUE_BUTTON_Pin){
		led_state = !led_state;
		vButton_message();
	}

}

void setCanSpeed(can_speed_t canSpeed){

    hcan.Instance = CAN;
    switch(canSpeed){

    case SPEED_125_KBITS:
        hcan.Init.Prescaler = 72;
        break;

    case SPEED_250_KBITS:
        hcan.Init.Prescaler = 36;
        break;

    case SPEED_500_KBITS:
        hcan.Init.Prescaler = 18;
        break;

    case SPEED_1000_KBITS:
        hcan.Init.Prescaler = 9;
        break;

    default:
        hcan.Init.Prescaler = 18;
        canSpeedOnStartup = 2;
        break;

    }

    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	can_status += HAL_CAN_Start(&hcan);
	can_status += HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	vCan_messages_init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UartRxBuffer, UART_RX_BUFFER);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	setCanSpeed(canSpeedOnStartup);

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
	/*
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  */
  /* USER CODE BEGIN CAN_Init 2 */

  CAN_FilterTypeDef can_filter_config;

  can_filter_config.FilterActivation = CAN_FILTER_ENABLE;
  can_filter_config.FilterBank = 10;
  can_filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_config.FilterIdHigh = 0;
  can_filter_config.FilterIdLow = 0x0000;
  can_filter_config.FilterMaskIdHigh = 0;			// decides which bits in id should be compared
  can_filter_config.FilterMaskIdLow = 0x0000;
  can_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_config.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &can_filter_config);



  /* USER CODE END CAN_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

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
