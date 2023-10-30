/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
	 int button_id, button_status;
	 float sensor_value;
} command;

void printer(char msg){
	ITM_SendChar(msg);
	ITM_SendChar('\n');

}

char* osStatusToString(osStatus_t status) {
    switch (status) {
        case osOK:
            return "OK: Operation completed successfully\r\n";
        case osError:
            return "osError: Unspecified error\r\n";
        case osErrorTimeout:
            return "Timeout: osErrorTimeout:  Operation timed out\r\n";
        case osErrorResource:
            return "Resource: osErrorResource: Resource not available\r\n";
        case osErrorParameter:
            return "Parameter: osErrorParameter: Parameter error\r\n";
        case osErrorNoMemory:
            return "NoMemory: osErrorNoMemory: System is out of memory\r\n";
        default:
            return "Unknown osStatus_t\r\n";
    }
}

float MAP(int int_IN)
{
	 if (int_IN < rpm_min){return 0.0;}
	 else if (int_IN > rpm_max){return pwm_max;}
	 else{
		return ((((int_IN - rpm_min)*(pwm_max - pwm_min))/(rpm_max - rpm_min)) + pwm_min);

	 }

}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for BT_reader */
osThreadId_t BT_readerHandle;
const osThreadAttr_t BT_reader_attributes = {
  .name = "BT_reader",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MT_controller */
osThreadId_t MT_controllerHandle;
const osThreadAttr_t MT_controller_attributes = {
  .name = "MT_controller",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sensor_Read */
osThreadId_t Sensor_ReadHandle;
const osThreadAttr_t Sensor_Read_attributes = {
  .name = "Sensor_Read",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Input_queue */
osMessageQueueId_t Input_queueHandle;
const osMessageQueueAttr_t Input_queue_attributes = {
  .name = "Input_queue"
};
/* Definitions for BT_send */
osMessageQueueId_t BT_sendHandle;
const osMessageQueueAttr_t BT_send_attributes = {
  .name = "BT_send"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void BT_reader_funct(void *argument);
void MT_controller_funct(void *argument);
void Sensor_reader_funct(void *argument);

/* USER CODE BEGIN PFP */
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Input_queue */
  Input_queueHandle = osMessageQueueNew (16, sizeof(command), &Input_queue_attributes);

  /* creation of BT_send */
  BT_sendHandle = osMessageQueueNew (16, sizeof(char*), &BT_send_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BT_reader */
  BT_readerHandle = osThreadNew(BT_reader_funct, NULL, &BT_reader_attributes);

  /* creation of MT_controller */
  MT_controllerHandle = osThreadNew(MT_controller_funct, NULL, &MT_controller_attributes);

  /* creation of Sensor_Read */
  Sensor_ReadHandle = osThreadNew(Sensor_reader_funct, NULL, &Sensor_Read_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 15;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start_IT(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = pwm_max;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BREAK_1_Pin */
  GPIO_InitStruct.Pin = BREAK_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BREAK_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BREAK_2_Pin */
  GPIO_InitStruct.Pin = BREAK_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BREAK_2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */













int FON_UART_Receive(char *received, uint16_t timeout,UART_HandleTypeDef *huartX) {
    HAL_StatusTypeDef status;
    unsigned char receivedChar;
    int index = 0;

    while (1) {
        status = HAL_UART_Receive(huartX, &receivedChar, 1, timeout);

        if (status == HAL_OK) {
            if (receivedChar == '\n') {
            	received[index] = '\0';

                return 1;
            } else {
                received[index] = receivedChar;
                index++;
            }
        } else {
            received[0] = '\0';
            return 0;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	command com;

	if (GPIO_Pin == BREAK_1_Pin ) {
		com.button_id 		= 1;
		if (HAL_GPIO_ReadPin(BREAK_1_GPIO_Port, GPIO_Pin) == GPIO_PIN_SET) {
			// Your code for rising edge -> apertou
			com.button_status  	= 0;
		} else {
			// Your code for falling edge -> soltou
			com.button_status  	= 1;
		}

		osStatus_t status 	=  osMessageQueuePut(Input_queueHandle, &com, 0, 0);
		if (status != osOK){
			char* str = osStatusToString(status);
			osMessageQueuePut(BT_sendHandle, &str, 0, 0);
		}
	}
	else if (GPIO_Pin == BREAK_2_Pin ) {
		com.button_id 		= 1;
		if (HAL_GPIO_ReadPin(BREAK_2_GPIO_Port, GPIO_Pin) == GPIO_PIN_SET) {
			// Your code for rising edge -> apertou
			com.button_status  	= 0;
		} else {
			// Your code for falling edge -> soltou
			com.button_status  	= 1;
		}

		osStatus_t status 	=  osMessageQueuePut(Input_queueHandle, &com, 0, 0);
		if (status != osOK){
			char* str = osStatusToString(status);
			osMessageQueuePut(BT_sendHandle, &str, 0, 0);
		}
	}
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	char str[30];
    //osMessageQueuePut(BT_sendHandle, &str, 0, 2000);
//    if (FON_UART_Receive(str,500, huart)){
//			osMessageQueuePut(BT_sendHandle, &str, 0, 2000);
//    }
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	uint32_t counter = __HAL_TIM_GET_COUNTER(htim);

//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_BT_reader_funct */
















/* USER CODE END Header_BT_reader_funct */
void BT_reader_funct(void *argument)
{
  /* USER CODE BEGIN 5 */
	unsigned char str[14] ="\r\nIniciando \r\n";
	HAL_UART_Transmit(&huart1, str, sizeof(str), 500);

	char* res;
    char receivedData[32];
	command com;


	while (1) {
	        if (FON_UART_Receive(receivedData,500, &huart1)){
	        	if (strlen(receivedData)==3){
	        		com.button_id 		= receivedData[0] - '0';
					com.button_status  	= 10*(receivedData[1] - '0') + (receivedData[2] - '0');
					osMessageQueuePut(Input_queueHandle, &com, 0, 2000);
	        	}
	        }

	        while (1){
	        	if (osMessageQueueGet(BT_sendHandle, &res, NULL, 250) == osOK) {
	        	    HAL_UART_Transmit(&huart1, res, strlen(res), 1000);
	        	}
	        	if (osMessageQueueGetCount(BT_sendHandle)==0){
	        		break;
	        	}

	        }

	        //osDelay(1000);
	        osThreadYield();
	}



  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MT_controller_funct */
/**
* @brief Function implementing the MT_controller thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MT_controller_funct */
void MT_controller_funct(void *argument)
{
  /* USER CODE BEGIN MT_controller_funct */
  /* Infinite loop */

	int lock = 0;
	int brk = 0;
	float freq, output = 0.0;
	int freq_int;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  TIM1->CCR1 = output;
	command com;
	char* input_command,decision;
	//int speed=0; // max = 255

	while(1){
		if (osMessageQueueGet(Input_queueHandle, &com, NULL, 2000)== osOK){
			if(com.button_id==9){
				lock = com.button_status;
				if (lock){input_command = "lock\n";}
				else{input_command = "UNlock\n";}
			}
			else if(com.button_id==1){
				brk = com.button_status;
				if (brk){ input_command = "FREIOU \n";}
				else{ input_command = "SOLTOU\n";}
			}
			else if(com.button_id==2){
					freq = (com.sensor_value)*1000;
					input_command = "";
					freq_int = (int) freq;
					asprintf(&input_command, "RPM: %d\n", freq_int);
			}
			osMessageQueuePut(BT_sendHandle, &input_command, 0, 2000);

		}
		else{
			input_command = "No new input\n";
			osMessageQueuePut(BT_sendHandle, &input_command, 0, 2000);

		}


		if(!lock){
			if (!brk){
				output = MAP(freq);

			}
			else{
			output = 666;
			}
		}
		else{
			//call locking function
			output = 666;
		}

		//activate motor
		TIM2->CCR1 = output;
		//return status to phone
		asprintf(&decision, "OUTPUT: %d\n", (int)output);
		osMessageQueuePut(BT_sendHandle, &decision, 0, 2000);
        //osThreadYield();
		osDelay(1000);

	}
  /* USER CODE END MT_controller_funct */
}

/* USER CODE BEGIN Header_Sensor_reader_funct */
/**
* @brief Function implementing the Sensor_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sensor_reader_funct */
void Sensor_reader_funct(void *argument)
{
  /* USER CODE BEGIN Sensor_reader_funct */
  /* Infinite loop */
	int delay = 2000;
	float rpm_input, rpm_old = 0;
	int counter;
	command com;
	com.button_id = 2;
	while(1){
        counter = __HAL_TIM_GET_COUNTER(&htim1);
        rpm_input =(float) counter/(PulsesPerRound*delay);
		com.sensor_value = rpm_input;
		if (counter != rpm_old){
			osMessageQueuePut(Input_queueHandle, &com, 0, 500);
		}
		__HAL_TIM_SET_COUNTER(&htim1,0);

		rpm_old = counter;
		osDelay(delay);
        //osThreadYield();
	}
  /* USER CODE END Sensor_reader_funct */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
	char* str = "ERROR_HANDLER CALLED";
	osMessageQueuePut(BT_sendHandle, &str, 0, 0);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
