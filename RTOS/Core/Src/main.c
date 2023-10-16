/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
} command;

void printer(char msg){
	ITM_SendChar(msg);
	ITM_SendChar('\n');

}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
/* Definitions for BT_queue */
osMessageQueueId_t BT_queueHandle;
const osMessageQueueAttr_t BT_queue_attributes = {
  .name = "BT_queue"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
  /* creation of BT_queue */
  BT_queueHandle = osMessageQueueNew (16, sizeof(command), &BT_queue_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int FON_UART_Receive(char *received, uint16_t timeout) {
    HAL_StatusTypeDef status;
    unsigned char receivedChar;
    int index = 0;
//	unsigned char str[7] ="CHAMOU ";
//    HAL_UART_Transmit(&huart1, str, sizeof(str), 500);

    while (1) {
        status = HAL_UART_Receive(&huart1, &receivedChar, 1, timeout);

        if (status == HAL_OK) {
            if (receivedChar == '\n') {
                // Received a newline character, terminate the string and return 1
            	//received[index] = '\n';
            	received[index] = '\0';

                return 1;
            } else {
                // Store the received character in the buffer
                received[index] = receivedChar;
                index++;
            }
        } else {
            // Handle error or timeout
            received[0] = '\0'; // Null-terminate the string to indicate no data received
            return 0;
        }
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_BT_reader_funct */
/* USER CODE END Header_BT_reader_funct */
void BT_reader_funct(void *argument)
{
  /* USER CODE BEGIN 5 */
	unsigned char str[14] ="\r\nIniciando \r\n";
    char receivedData[32];
	command com;

    HAL_UART_Transmit(&huart1, str, sizeof(str), 500);

	//printer('2');
//		sprintf(str, "\r\nTemp: %i\r\n",cont);
		//		HAL_UART_Transmit(&huart1, str, sizeof(str), 500);
//		cont++;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COLOCAR SEMAFORO e interrupção
	while (1) {
	        if (FON_UART_Receive(receivedData,500)){
	        	// Replace huart1 with your UART handle
	        	if (strlen(receivedData)==3){
	        		com.button_id 		= receivedData[0] - '0';
					com.button_status  	= 10*(receivedData[1] - '0') + (receivedData[2] - '0');
					osMessageQueuePut(BT_queueHandle, &com, 0, 2000);
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

//le da memoria de longa duracao
	int auth = 0;
	command com;

	while(1){
		if (osMessageQueueGet(BT_queueHandle, &com, NULL, 2000)== osOK){
			if(com.button_id==9){
				auth = com.button_status;
				if (auth){
					unsigned char str[15];
				    sprintf(str, "\r\n ABRIU %i \r\n", auth);
					HAL_UART_Transmit(&huart1, str, sizeof(str), 1000);
				}
				else{
					unsigned char str[15];
				    sprintf(str, "\r\n FECHOU %i \r\n", auth);
					HAL_UART_Transmit(&huart1, str, sizeof(str), 1000);
					}
				}
			//else if !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!TRATAR TODOS OS OUTROS COMs

		}
		if(auth){}
		//osDelay(1000);
        osThreadYield();

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
  for(;;)
  {
    osDelay(1);
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
