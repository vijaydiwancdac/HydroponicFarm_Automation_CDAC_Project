/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bare-metal HC-SR04 demo (no RTOS, no SEGGER)
  ******************************************************************************
  * Builds with STM32CubeIDE default SysTick time-base (no TIM1 conflicts).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "SEGGER_SYSVIEW.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "event_groups.h"
#include <stdbool.h>
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
/* trigger pin for ultrasonic sensor */
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOA

/*adc buffer length to store 20 samples */
#define NUM_SAMPLES 20

#define DHT_TIMEOUT 100  // Timeout in us

#define DHT22_PORT GPIOA
#define DHT22_PIN  GPIO_PIN_5
#define height_tank 50


#define FAN_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_6
#define vol_factor (1000/height_tank) // based on height of tank let 50 cm height of tank is equivalent to 1000ml
typedef struct {
    float Temperature;
    float Humidity;
} DHT22_Data_t;


typedef struct{
	DHT22_Data_t dhtData;
	uint32_t waterLevel;
	float tdsValue;

	uint8_t dht22Ready;
	uint8_t tdsReady;
    uint8_t ultrasonicReady;

}SensorData;

SensorData sensorData;


/* variables for ultrasonic sensor*/
volatile uint32_t sec_counter = 0;
volatile uint32_t icVal1 = 0;
volatile uint32_t icVal2 = 0;
volatile uint8_t isFirstCaptured = 0;  /* flag to capture whether it
 	 	 	 	 	 	 	 	 	 	 is first time triggering*/
volatile uint32_t distance_cm = 0; /*ultrasonic measurement value*/

/*variables for tds sensor*/
volatile uint32_t adc_samples[NUM_SAMPLES];  /*buffer to store 20 samples
												for precise sensor reading*/
volatile uint8_t sample_index = 0;  /* index to track how
									many samples have been captured*/
volatile uint8_t start_adc_sampling = 0; /*flag set after every 2 sec for adc conversion */
volatile uint8_t adc_ready_to_process = 0; /* flag set when 20 sample captured */


//volatile DHT22_Data_t dhtDataGlobal;

/* threshold variables   */

volatile float tdsThreshold=70.0;//for checking //300.0
volatile uint32_t waterLevelThreshold = 25;      // %
volatile float temperatureThreshold = 26.0;//for checking //28.0;      // °C
volatile float humidityThreshold=100.0;

volatile uint8_t warningActive = 0;  // 0 = No warning, 1 = Warning active


volatile char uart_rx_buffer[64];  // Adjust size as per max expected input
volatile uint8_t uart_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void UltrasonicTask(void *argument);
void TdsTask(void *argument);
void DHT22Task(void *argument);
void CommTask(void *argument);
void DisplayTask(void *param);
void ControlTask(void *param);
void DWT_Init(void);
void Ultrasonic_us(uint32_t us);

/*void DHT22_Delay_Init(void);*/
void DHT22_us(uint16_t us);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT22_Start(void);
uint8_t DHT22_CheckResponse(void);
uint8_t DHT22_ReadBit(void) ;
uint8_t DHT22_ReadByte(void) ;
uint8_t DHT22_ReadData(DHT22_Data_t *data);

void LCD_Display(DHT22_Data_t Data, float tds, uint32_t level);
void LCD_Init(void) ;
void LCD_Clear(void);
void LCD_Print(char *str, uint8_t row, uint8_t col) ;
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data) ;
void LCD_SendNibble(uint8_t nibble);

//static void trigger_pulse(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define STACK_SIZE 512

//create Queue for UART Data storage

SemaphoreHandle_t xSensorDataMutex;
EventGroupHandle_t xSensorSyncGroup;


// Binary semaphores for each sensor task completion

#define BIT_DHT22   (1 << 0)
#define BIT_TDS     (1 << 1)
#define BIT_ULTRA   (1 << 2)


TaskHandle_t xDHT22TaskHandle = NULL;
SemaphoreHandle_t xEchoSemaphore = NULL;
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();
  DWT_Init();

  // Set priority and enable TIM3 interrupt

  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);  // Lower than sensor tasks
  HAL_NVIC_EnableIRQ(UART5_IRQn);

  HAL_NVIC_SetPriority(ADC_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(ADC_IRQn);

  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim3);

    xEchoSemaphore = xSemaphoreCreateBinary();

    xSensorDataMutex = xSemaphoreCreateMutex();
    xSensorSyncGroup = xEventGroupCreate();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//ensure proper priority grouping for freeRTOS
    assert_param(xTaskCreate(UltrasonicTask, "UltrasonicTask", STACK_SIZE, NULL, 3, NULL) == pdPASS);
    assert_param(xTaskCreate(TdsTask, "TdsTask", STACK_SIZE, NULL, 3, NULL) == pdPASS);
    assert_param(xTaskCreate(DHT22Task, "DHT22Task", STACK_SIZE, NULL, 3,&xDHT22TaskHandle) == pdPASS);
    assert_param(xTaskCreate(CommTask, "CommTask", STACK_SIZE, NULL, 4, NULL) == pdPASS);
    assert_param(xTaskCreate(ControlTask, "ControlTask", STACK_SIZE, NULL, 4,NULL) == pdPASS);
    assert_param(xTaskCreate(DisplayTask, "DisplayTask", STACK_SIZE, NULL, 4, NULL) == pdPASS);


    vTaskStartScheduler();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
	__HAL_RCC_UART5_CLK_ENABLE();
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD0 PD1 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void UltrasonicTask(void *argument)
{

  while (1)
  {
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    Ultrasonic_us(2);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Ultrasonic_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset first
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);

    if (xSemaphoreTake(xEchoSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
    	uint32_t water_height=height_tank-distance_cm;

    	SEGGER_SYSVIEW_PrintfHost("Distance: %lu cm",water_height);
    	if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

    		sensorData.waterLevel = distance_cm;
            sensorData.ultrasonicReady = 1;
            xSemaphoreGive(xSensorDataMutex);
    	}
    	xEventGroupSetBits(xSensorSyncGroup,BIT_ULTRA);  // Similar for others
    }

    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}
void TdsTask(void *argument)
{


	uint32_t sum = 0;
	    float voltage = 0.0f;

	    while (1)
	    {
	        if (start_adc_sampling)
	        {
	            start_adc_sampling = 0;
	            adc_ready_to_process = 0;
	            sample_index = 0;

	            HAL_ADC_Start_IT(&hadc1);  // Start ADC conversion with interrupt
	        }

	        if (adc_ready_to_process)
	        {
	            adc_ready_to_process = 0;

	            sum = 0;
	            for (int i = 0; i < NUM_SAMPLES; i++)
	                sum += adc_samples[i];

	            uint16_t average = sum / NUM_SAMPLES;

	            voltage = (average * 3.3f) / 4095.0f;  // For 12-bit ADC and 3.3V
	            float tds_value = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5;
	            average= 0;
	            int t10 = (int)(tds_value * 100);
	            SEGGER_SYSVIEW_PrintfHost("Tds : %d.%d%%",t10 / 100, t10 % 100);
	            if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
	            	sensorData.tdsValue = tds_value;
	                sensorData.tdsReady = 1;
	                xSemaphoreGive(xSensorDataMutex);
	            }
	            xEventGroupSetBits(xSensorSyncGroup,BIT_TDS);
	        }
	        vTaskDelay(20/portTICK_PERIOD_MS);
	    }
}

void DHT22Task(void *params)
{
	DHT22_Data_t Data;
	const TickType_t xMaxBlockTime = (6000/portTICK_PERIOD_MS); // Max wait time
    while (1) {
    	ulTaskNotifyTake(pdTRUE, xMaxBlockTime); // Clear notification on entry, wait up to xMaxBlockTime
    	/********************************************/
    	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);  // Ensure line is high
    	vTaskDelay(pdMS_TO_TICKS(2));
    	// Wait 2ms before next cycle
    	/********************************************/
    	/*
    	if (DHT22_ReadData(&Data) == 0) {*/
    	/************************************/
    	taskENTER_CRITICAL();  //  Lock interrupts to preserve timing
    	int status = DHT22_ReadData(&Data);
        taskEXIT_CRITICAL();   //  Restore interrupts after read

    	if (status == 0){

    		int t10,h10;

    		t10 = (int)(Data.Temperature * 10); // e.g. 24.3°C → 243
    	    h10 = (int)(Data.Humidity * 10);    // e.g. 65.7% → 657

    	    SEGGER_SYSVIEW_PrintfHost("🌡️ Temp: %d.%d°C | 💧Humidity: %d.%d%%",t10 / 10, t10 % 10, h10 / 10, h10 % 10);
    		if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                sensorData.dhtData = Data;
                sensorData.dht22Ready = 1;
                xSemaphoreGive(xSensorDataMutex);
    		}
    		xEventGroupSetBits(xSensorSyncGroup, BIT_DHT22);  // Similar for others

    	}
    	else
    	{
                SEGGER_SYSVIEW_PrintfHost("DHT22Task: Sensor read failed");
    	}
    	vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

void CommTask(void *argument){
	char uartBuffer[128];

	while(1) {
		xEventGroupWaitBits(
		    xSensorSyncGroup,
		    BIT_DHT22 | BIT_TDS | BIT_ULTRA,  // Wait for all
		    pdTRUE,     // Clear bits after receiving
		    pdTRUE,     // Block until ALL bits set
		    portMAX_DELAY
		);
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        	uint32_t water_height=(height_tank - sensorData.waterLevel);
        	snprintf(uartBuffer, sizeof(uartBuffer),"%.1f,%.1f,%.2f,%ld\r\n",sensorData.dhtData.Temperature,sensorData.dhtData.Humidity,sensorData.tdsValue,water_height);
        	HAL_UART_Transmit(&huart5, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

        	SEGGER_SYSVIEW_PrintfHost("Comm task");
        	sensorData.dht22Ready = 0;
            sensorData.tdsReady = 0;
            sensorData.ultrasonicReady = 0;

            xSemaphoreGive(xSensorDataMutex);
        }
        else
        {
                    // Mutex not available: optional UART debug
            char *errorMsg = "CommTask: Failed to acquire mutex\r\n";
            HAL_UART_Transmit(&huart5, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
        }
        //}
        vTaskDelay(20/portTICK_PERIOD_MS); // Avoid tight loop
	}

}

void DisplayTask(void *param) {

    while(1){

        // Wait for all 3 sensor tasks to signal
    	xEventGroupWaitBits(
    		xSensorSyncGroup,
    		BIT_DHT22 | BIT_TDS | BIT_ULTRA,  // Wait for all
    	    pdTRUE,     // Clear bits after receiving
			pdTRUE,     // Block until ALL bits set
    	    portMAX_DELAY
    	);
    	if(warningActive==0){
    		if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        	LCD_Init();
        	LCD_Clear();
            vTaskDelay(2/portTICK_PERIOD_MS);
            LCD_Display(sensorData.dhtData,sensorData.tdsValue, sensorData.waterLevel);

            // Reset flags
            sensorData.dht22Ready = 0;
            sensorData.tdsReady = 0;
            sensorData.ultrasonicReady = 0;

            xSemaphoreGive(xSensorDataMutex);
            vTaskDelay(20/portTICK_PERIOD_MS); // Avoid tight loop
    		}
       }
    }
}

void ControlTask(void *param){
	//char msg[128];
	/*humidity 100percent temp 28 degree  tds 300ppm  ultrasonic <50% water >=90%
	 *- for humidity and temp threshold then turn on fan
	 *- for tds , calculate the required nutrition value to added or water to neutralise it
	 *- if tds is lower then the threshold then display to be added nutrition
	 *- Increase in ppm= (Nutrition added in ml)×(500 mg/ml)
	 *-                  ----------------------------------------
	 *-                     Water Volume in L
​		Nutrition to Add (ml)= (Target PPM−Current PPM)×Water Volume (L)
                               ------------------------------------------   ×100
                                   Standard PPM per Litre of Nutrition
	 */
	while(1){

		xEventGroupWaitBits(
	    		xSensorSyncGroup,
	    		BIT_DHT22 | BIT_TDS | BIT_ULTRA,  // Wait for all
	    	    pdTRUE,     // Clear bits after receiving
				pdTRUE,     // Block until ALL bits set
	    	    portMAX_DELAY
		);
		if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
			uint32_t water_height=(height_tank - sensorData.waterLevel);
			uint32_t water_now = ((float)water_height/(float) height_tank)*100;
			SEGGER_SYSVIEW_PrintfHost("water %ld%",water_now);
			bool waterWarning = false;
			bool tdsWarning = false;
			    char line1[64] = {0};
			    char line2[64] = {0};


			if (water_now <= waterLevelThreshold) {
				/*SEGGER_SYSVIEW_PrintfHost("water check ");*/
				waterWarning = true;
		        snprintf(line1,sizeof(line1),"H2O Low! %ld%% ",water_now);
			}

		            // 🧠 2. Check TDS Threshold
			if (sensorData.tdsValue <= tdsThreshold) {
				tdsWarning = true;
		         float nutrientToAdd = (((tdsThreshold - sensorData.tdsValue)*water_height*vol_factor*100.0)/ 500.0);
		         /*SEGGER_SYSVIEW_PrintfHost("tds check ");*/
		         snprintf(line2, sizeof(line2), "Add %.0fml Nutr", nutrientToAdd);
			}
			if (waterWarning || tdsWarning) {
			        LCD_Init();
			        LCD_Print(line1, 0, 0);  // Line 1
			        LCD_Print(line2, 1, 0);  // Line 2
			        warningActive = 1;

			}
			else{
				warningActive=0;
			}
		            // 🧠 3. Temperature control → Fan ON/OFF
			if (sensorData.dhtData.Temperature >= temperatureThreshold || sensorData.dhtData.Humidity >= humidityThreshold) {

		         HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);  // Fan ON
			}
			else {
		         HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);  // Fan OFF
			}

			xSemaphoreGive(xSensorDataMutex);
			SEGGER_SYSVIEW_PrintfHost("control task ");
		}
		vTaskDelay(20/portTICK_PERIOD_MS);

	}
}


void LCD_Display(DHT22_Data_t Data, float tds, uint32_t level) {
	char buf[128];
	uint32_t water_height=(height_tank - level);
	uint32_t water_now = ((water_height*100)/height_tank);
	sprintf(buf,"T:%.1fC H:%.1f%%",Data.Temperature,Data.Humidity);
	LCD_Print(buf, 0, 0);
	vTaskDelay(100/portTICK_PERIOD_MS); // Avoid tight loop
	sprintf(buf,"N:%.2f%% W:%ld%%",tds,water_now);
	LCD_Print(buf, 1, 0);
	/*SEGGER_SYSVIEW_PrintfHost("display");*/


}

void LCD_Init(void) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd init");*/
	vTaskDelay(40/portTICK_PERIOD_MS); // Wait for LCD to power up

    LCD_SendCommand(0x33); // Initialization sequence part 1
    LCD_SendCommand(0x32); // Switch to 4-bit mode
    LCD_SendCommand(0x28); // Function set: 4-bit, 2 line
    LCD_SendCommand(0x0C); // Display ON, cursor OFF
    LCD_SendCommand(0x06); // Entry mode: Increment cursor
    LCD_Clear();
}
void LCD_Clear(void) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd clear");*/
    LCD_SendCommand(0x01); // Clear display
    vTaskDelay(2/portTICK_PERIOD_MS);         // Allow time to clear
}

void LCD_Print(char *str, uint8_t row, uint8_t col) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd print");*/
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendCommand(addr);   // Set DDRAM address

    while (*str) {
        LCD_SendData(*str++);
    }
}

void LCD_SendCommand(uint8_t cmd) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd send command");*/
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // RS = 0
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // RW = 0
    LCD_SendNibble(cmd >> 4);  // High nibble
    LCD_SendNibble(cmd & 0x0F); // Low nibble
}

void LCD_SendData(uint8_t data) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd send data");*/
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);   // RS = 1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // RW = 0
    LCD_SendNibble(data >> 4);
    LCD_SendNibble(data & 0x0F);
}
void LCD_SendNibble(uint8_t nibble) {
	/*SEGGER_SYSVIEW_PrintfHost("lcd send nibble");*/
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, (nibble >> 0) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, (nibble >> 1) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, (nibble >> 2) & 0x01);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, (nibble >> 3) & 0x01);

    // Toggle Enable
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    vTaskDelay(1/portTICK_PERIOD_MS); // Or use a fine delay like usleep if needed
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

void DHT22_us(uint16_t us) {
	uint32_t start = DWT->CYCCNT;
	    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
	    while ((DWT->CYCCNT - start) < ticks);

}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	/*SEGGER_SYSVIEW_PrintfHost("set pin op");*/
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_DeInit(DHT22_PORT, DHT22_PIN);
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    DHT22_us(10);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	/*SEGGER_SYSVIEW_PrintfHost("set pin ip");*/
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_DeInit(DHT22_PORT, DHT22_PIN);
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    DHT22_us(10);
}

void DHT22_Start(void) {
	/*SEGGER_SYSVIEW_PrintfHost("dht22 start");*/
    Set_Pin_Output(GPIOA, GPIO_PIN_5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    DHT22_us(2000);  // >1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    DHT22_us(30);  // 20–40us
    Set_Pin_Input(GPIOA, GPIO_PIN_5);
}

uint8_t DHT22_CheckResponse(void) {
	/*SEGGER_SYSVIEW_PrintfHost("check response");*/
	 uint32_t timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET && timeout<DHT_TIMEOUT){
	    	DHT22_us(1);
	    	timeout++;
	    }
	    /*SEGGER_SYSVIEW_PrintfHost("Low wait timeout1 cr = %lu", timeout);*/
	    if (timeout >=DHT_TIMEOUT) {
	    	return 1;
	    }
	    timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET){
	    	DHT22_us(1);
	    	timeout++;
	    }
	    /*SEGGER_SYSVIEW_PrintfHost("Low wait timeout2 cr = %lu", timeout);*/
	    if (timeout >=DHT_TIMEOUT){
	    	return 1;
	    }
	    timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET){
	    	DHT22_us(1);
	    	timeout++;
	     }
	    /*SEGGER_SYSVIEW_PrintfHost("Low wait timeout3 cr = %lu", timeout);*/
	     if (timeout >=DHT_TIMEOUT) {
	    	 return 1;
	     }
	    return 0;
}

uint8_t DHT22_ReadBit(void) {
	uint32_t timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET && timeout<DHT_TIMEOUT){
	    	DHT22_us(1);
	    	timeout++;
	    }
	    /*SEGGER_SYSVIEW_PrintfHost("Low wait timeout1 rb = %lu", timeout);*/
	    if (timeout >= DHT_TIMEOUT)
	    	return 0xFF;
	    DHT22_us(40); // Wait 40us
	    uint8_t bit_value = 0;

	    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))
	    	bit_value= 1;
	    else
	    	bit_value= 0;


	    timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET && timeout<DHT_TIMEOUT){
	    	DHT22_us(1);
	    	 timeout++;
	    }
	    /*SEGGER_SYSVIEW_PrintfHost("Low wait timeout2 rb = %lu", timeout);*/
	    if (timeout >= DHT_TIMEOUT)
	    	 return 0xFF;
	    return bit_value;

}

uint8_t DHT22_ReadByte(void) {
	uint8_t i, byte = 0;
	    for (i = 0; i < 8; i++) {
	        uint8_t bit = DHT22_ReadBit();
	        if (bit == 0xFF) return 0xFF; // error
	        byte = (byte << 1) | bit;
	    }
	    return byte;
}

uint8_t DHT22_ReadData(DHT22_Data_t *data)  {
	memset(data, 0, sizeof(DHT22_Data_t));
	if (data == NULL) {
	    return 1;
	}
    DHT22_Start();
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, checksum;
    uint16_t RH, TEMP;
    float humidity, temperature;


    if (DHT22_CheckResponse() != 0){
    	/*SEGGER_SYSVIEW_PrintfHost("response fails  ");*/
    	return 1;
    }
    /*SEGGER_SYSVIEW_PrintfHost("got response in read data \n");*/
    Rh_byte1 = (int) DHT22_ReadByte();
    Rh_byte2 =(int) DHT22_ReadByte();
    Temp_byte1 = (int)DHT22_ReadByte();
    Temp_byte2 = (int)DHT22_ReadByte();
    checksum = (int)DHT22_ReadByte();
    if(Rh_byte1 ==0xFF || Rh_byte2==0xFF || Temp_byte1== 0xFF || Temp_byte2 ==0xFF || checksum == 0xFF){
    	/*SEGGER_SYSVIEW_PrintfHost("timeout ");*/
    	return 1;
    }
    uint8_t sum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
    if (sum != checksum){
    	/*SEGGER_SYSVIEW_PrintfHost("checksum failed ");*/
    	return 1;
    }
    RH = ((uint16_t)Rh_byte1 << 8) | Rh_byte2;


	humidity= (float)(RH / 10.0f);
    data->Humidity =humidity;
    TEMP = (((uint16_t)Temp_byte1 & 0x7F) << 8) | Temp_byte2;
    if (Temp_byte1 & 0x80) {
        TEMP *=-1;
    }
    temperature=(float)(TEMP/10.0f);
    data->Temperature = temperature;
    Rh_byte1 =0;
    Rh_byte2 =0;
    Temp_byte1 =0;
    Temp_byte2=0;
    sum=0;
    checksum=0;
    /*int t10 = (int)(data->Temperature * 10); // e.g. 24.3°C → 243
    int h10 = (int)(data->Humidity * 10);    // e.g. 65.7% → 657
    SEGGER_SYSVIEW_PrintfHost("🌡️ in read data Temp: %d.%d°C | 💧Humidity: %d.%d%%",
            t10 / 10, t10 % 10, h10 / 10, h10 % 10);*/
    return 0;
}


void DWT_Init(void)
{
	//DWT->CYCCNT = 0;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Ultrasonic_us(uint32_t us)
{
	// In main.c (under USER CODE BEGIN 4, or wherever your DWT functions are)
	  uint32_t start_cycles = DWT->CYCCNT;
	    // Use HAL_RCC_GetHCLKFreq() to get the actual HCLK frequency at runtime.
	    // This is the clock that DWT->CYCCNT runs on.
	    uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
	    uint32_t cycles_per_us = hclk_freq / 1000000U;

	    // Calculate the number of cycles needed for the requested delay
	    uint32_t cycles_to_wait = us * cycles_per_us;

	    // Add a small, empirically determined overhead compensation if needed.
	    // Based on your 15us overhead for 30us request, this is ~15 * 84 = 1260 cycles.
	    // We'll start with this and you can fine-tune.
	    const uint32_t DWT_CALIBRATED_OVERHEAD_CYCLES = 1260; // For 84MHz HCLK, 15us overhead

	    if (cycles_to_wait > DWT_CALIBRATED_OVERHEAD_CYCLES) {
	        cycles_to_wait -= DWT_CALIBRATED_OVERHEAD_CYCLES;
	    } else {
	        cycles_to_wait = 1; // Ensure a minimum wait for very small delays
	    }

	    // Wait until the required number of cycles has passed
	    while ((DWT->CYCCNT - start_cycles) < cycles_to_wait);

}
 /*TIM2 Interrupt Handler */
void TIM2_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC2) != RESET)
    {
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);

      if (isFirstCaptured == 0)
      {
        icVal1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
        isFirstCaptured = 1;

        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
      }
      else
      {
        icVal2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);

        if (icVal2 > icVal1)
        {
          uint32_t diff = icVal2 - icVal1;
          distance_cm = (diff * 0.0343) / 2;
        }
        else
        {
          distance_cm = 0;
        }

        isFirstCaptured = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xEchoSemaphore, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}

void TIM3_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)  // Check update flag
	    {
	        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET) // Check interrupt source
	        {
	            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);  // Clear interrupt
	            sec_counter++;
	            if(sec_counter>6){
	            	sec_counter=1;
	            }
	            if(sec_counter%3==0){
	            	start_adc_sampling = 1;
	            }

	            if(sec_counter%5==0){
	            	 if (xDHT22TaskHandle != NULL) {
	               	// Notify DHT22Task to perform a reading
	           		vTaskNotifyGiveFromISR(xDHT22TaskHandle, &xHigherPriorityTaskWoken);
	           		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	            	 }
	            }

	        }
	    }

}


void ADC_IRQHandler(void)
{
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))
    {
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);

        adc_samples[sample_index++] = HAL_ADC_GetValue(&hadc1);

        if (sample_index >= NUM_SAMPLES)
        {
            HAL_ADC_Stop_IT(&hadc1);
            adc_ready_to_process = 1;
            sample_index = 0;
        }
        else
        {
            HAL_ADC_Start_IT(&hadc1);  // Trigger next sample
        }
    }
}

void UART5_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE))
    {
        char received = (char)(huart5.Instance->DR & 0xFF);

        if (received == '\n')  // End of input
        {
            uart_rx_buffer[uart_rx_index] = '\0';
            uart_rx_index = 0;
            UBaseType_t uxSavedInterruptStatus;
            /*int t1=tdsThreshold*100;
            SEGGER_SYSVIEW_PrintfHost("Threshold before update : %d.%d\n", t1/100,t1%100);*/
            uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
			float new_threshold= atof((char*)uart_rx_buffer);  // Direct update
			if(new_threshold>0){
				tdsThreshold=new_threshold;
			}/*
			else{
				SEGGER_SYSVIEW_PrintfHost("negative threshold received ");
			}
            t1=tdsThreshold*100;
            SEGGER_SYSVIEW_PrintfHost("Threshold Updated via IRQ: %d.%d\n", t1/100,t1%100);*/

            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        }
        else
        {
        	if (uart_rx_index < sizeof(uart_rx_buffer) - 1)
        	{
            uart_rx_buffer[uart_rx_index++] = received;
        	}
        }


    __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_RXNE);
    }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
	SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s on line %d\r\n", file, line);
				while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
