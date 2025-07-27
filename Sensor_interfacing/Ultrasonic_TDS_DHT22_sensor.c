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
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* trigger pin for ultrasonic sensor */
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOA

/*adc buffer length to store 20 samples */
#define NUM_SAMPLES 20

#define DHT_TIMEOUT 100  // Timeout in us

#define DHT22_PORT GPIOA
#define DHT22_PIN  GPIO_PIN_5

typedef struct {
    float Temperature;
    float Humidity;
    uint8_t status; // 0 = OK, 1 = Error
} DHT22_Data_t;
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
volatile float tds_value = 0.0f;  /* tds value after final averaging */

volatile DHT22_Data_t dhtDataGlobal;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void UltrasonicTask(void *argument);
void TdsTask(void *argument);
void DHT22Task(void *argument);

void DWT_Init(void);
void Ultrasonic_us(uint32_t us);

void DHT22_Delay_Init(void);
void DHT22_us(uint16_t us);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT22_Start(void);
uint8_t DHT22_CheckResponse(void);
uint8_t DHT22_ReadBit(void) ;
uint8_t DHT22_ReadByte(void) ;
uint8_t DHT22_ReadData(DHT22_Data_t *data);

//static void trigger_pulse(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define STACK_SIZE 256

//create Queue for UART Data storage

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();
  DWT_Init();

  // Set priority and enable TIM3 interrupt
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_NVIC_SetPriority(ADC_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(ADC_IRQn);


    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim3);

    xEchoSemaphore = xSemaphoreCreateBinary();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	//ensure proper priority grouping for freeRTOS
    assert_param(xTaskCreate(UltrasonicTask, "UltrasonicTask", STACK_SIZE, NULL, 2, NULL) == pdPASS);
    assert_param(xTaskCreate(TdsTask, "TdsTask", STACK_SIZE, NULL, 2, NULL) == pdPASS);
    assert_param(xTaskCreate(DHT22Task, "DHT22Task", STACK_SIZE, NULL, 2,&xDHT22TaskHandle) == pdPASS);


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
  htim4.Init.Prescaler = 83;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
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
      SEGGER_SYSVIEW_PrintfHost("Distance: %lu cm", distance_cm);
    }
    else
    {
      SEGGER_SYSVIEW_PrintfHost("Sensor Timeout or No Echo");
    }

    vTaskDelay(2500/portTICK_PERIOD_MS);
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
	            tds_value = (133.42 * voltage * voltage * voltage - 255.86 * voltage * voltage + 857.39 * voltage) * 0.5;
	            average= 0;
	            int t10 = (int)(tds_value * 100);
	            SEGGER_SYSVIEW_PrintfHost("Tds : %d.%d%%",t10 / 100, t10 % 100);
	        }

	        vTaskDelay(50/portTICK_PERIOD_MS); // Avoid tight loop
	    }
}

void DHT22Task(void *params) {
	DHT22_Data_t sensorData;
	const TickType_t xMaxBlockTime = (7000/portTICK_PERIOD_MS); // Max wait time


    while (1) {
    	ulTaskNotifyTake(pdTRUE, xMaxBlockTime); // Clear notification on entry, wait up to xMaxBlockTime

    	if (DHT22_ReadData(&sensorData) == 0) {
    		int t10 = (int)(sensorData.Temperature * 10); // e.g. 24.3°C → 243
    		int h10 = (int)(sensorData.Humidity * 10);    // e.g. 65.7% → 657

        	SEGGER_SYSVIEW_PrintfHost("🌡️ Temp: %d.%d°C | 💧Humidity: %d.%d%%",
    		    	    t10 / 10, t10 % 10, h10 / 10, h10 % 10);

    	    dhtDataGlobal = sensorData;
    	} else {
    	    SEGGER_SYSVIEW_PrintfHost("⚠️ DHT22_ReadData failed — skipping global update");
    	}
    }
}
void DHT22_us(uint16_t us) {
	uint32_t start = DWT->CYCCNT;
	    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
	    while ((DWT->CYCCNT - start) < ticks);

}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
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
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_DeInit(DHT22_PORT, DHT22_PIN);
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
    DHT22_us(10);
}

void DHT22_Start(void) {
    Set_Pin_Output(GPIOA, GPIO_PIN_5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    DHT22_us(2000);  // >1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    DHT22_us(30);  // 20–40us
    Set_Pin_Input(GPIOA, GPIO_PIN_5);
}

uint8_t DHT22_CheckResponse(void) {
	 uint32_t timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET && timeout<DHT_TIMEOUT){
	    	DHT22_us(1);
	    	timeout++;
	    }
	    if (timeout >=DHT_TIMEOUT) {
	    	return 1;
	    }
	    timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET){
	    	DHT22_us(1);
	    	timeout++;
	    }
	    if (timeout >=DHT_TIMEOUT){
	    	return 1;
	    }
	    timeout = 0;
	    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET){
	    	DHT22_us(1);
	    	timeout++;
	     }
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
	if (data == NULL) {
	    return 1;
	}
    DHT22_Start();
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, checksum;
    uint16_t RH, TEMP;
    float humidity, temperature;


    if (DHT22_CheckResponse() != 0)
    	return 1;
    Rh_byte1 = (int) DHT22_ReadByte();
    Rh_byte2 =(int) DHT22_ReadByte();
    Temp_byte1 = (int)DHT22_ReadByte();
    Temp_byte2 = (int)DHT22_ReadByte();
    checksum = (int)DHT22_ReadByte();
    if(Rh_byte1 ==0xFF || Rh_byte2==0xFF || Temp_byte1== 0xFF || Temp_byte2 ==0xFF || checksum == 0xFF){
    	return 1;
    }
    uint8_t sum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
    if (sum != checksum){
    	SEGGER_SYSVIEW_PrintfHost("checksum failed ");
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
  /*uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
  while ((DWT->CYCCNT - start) < ticks);*/
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
	            if(sec_counter%2==0){
	            	start_adc_sampling = 1;
	            }

	            if(sec_counter%6==0){
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
  if (htim->Instance == TIM1) {
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
	SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s on line %d\r\n", file, line);
				while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
