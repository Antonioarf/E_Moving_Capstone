# Capstone Project

This repository documents the development of the capstone project, a partnership between students from INSPER (BR) and Texas A&M (USA), with E-Moving as client. The full scope of the project aims at start the technological independence processes for the company, stating from the bike’s speed controlled. In the RTOS folder you will find all the files with the program to be implemented in the MCU inside the controller board, responsible for the sensor read and decision-making.

## MicroController for E-Bike

As previously described, the STM32 MCU software was developed one module at a time, in the following sequence: Overall FreeRTOS structures and tasks; FreeRTOS queues; Bluetooth communication and lock command; brake sensors; pedal encoder; motor and relays actuation; Timers for automatic lock and user safety.

| Component | Peripheral | Pin(s) | Details |
| --- | --- | --- | --- |
| BlueTooth Module | USART1 | Tx: PC4
Rx: PC5 | Baud Rate: 9600
Mode: Async |
| Brakes | GPIO_EXTI | Left: PA5
Right: PA6 | Pull Up activated |
| Lock Control | GPIO_OUTPUT | PC8 |  |
| Pedal | TIM1 | CH1: PA12 | Internal CLK: ETR2 |
| Motor PWM | TIM2 | CH2: PA1 |  |

All FreeRTOS tasks follow the same structures, a first block of code to be run in the initialization, and an infinite loop to be executed when the task is called by the kernel.

```c
void Sensor_reader_funct(void *argument){
`/* USER CODE BEGIN Sensor_reader_funct */`
`/* Infinite loop */`
`while(1){
}
`/* USER CODE END Sensor_reader_funct */`
}
```

For the two queues, both with the maximum length of 16 messages, the input_queue was implemented to send a struct type called “command” was created, with three attributes, an ID, a  binary Status and a float Value for quantitative inputs. The bluetooth_queue carries pointers to lists of char variables, independent of the size of said list.

```c
`/* Create the queue(s) */`
`/* creation of input_queue */`
`input_queueHandle = osMessageQueueNew (16, sizeof(command), &input_queue_attributes);`
`/* creation of bluetooth_queue */`
`bluetooth_queueHandle = osMessageQueueNew (16, sizeof(char*), &bluetooth_queue_attributes)
```

The Bluetooth communication was handled by the  USART1 peripheral (appendix 2), in asynchronous mode, connected to the HC-05 module, which was responsible for the pairing process with the phone. As described, the codes received by the board were composed of a three-digit number, the first for referencing the command ID, and the last two, the status information.

| Comand | ID | Binary Status | Float Value |
| --- | --- | --- | --- |
| Brakes | 1 | Released: 0
Pressed:  1 | N/A |
| Pedal | 2 | N/A | Frequency of the pedal in HZ |
| Lock | 9 | Unlock: 0
Locking:  1 | N/A |
| Stop | 3 | Is Parked:1
Moving: 0 | N/A |

The software interface for the communication was done in the assigned task, where in the initialization phase an “Init” message is sent to inform  the phone that, either the communication was beginning, or the board was reset for any reason.

```c
void BT_reader_funct(void *argument){
/* USER CODE BEGIN 5 */
unsigned char str[8] ="\nInit\n";  //initial post to confirm
HAL_UART_Transmit(&huart1, str, sizeof(str), 500);
char* res;
char receivedData[32];
command com;
```

In the task’s main loop, first the Bluetooth buffer was emptied via the FON_UART_Receive function call, and in case the received data matched the described format, a command object is inserted in the input_queue.

```c
while (1) {
        if (FON_UART_Receive(receivedData,500, &huart1)){
        	if (strlen(receivedData)==3){
        		com.button_id 		= receivedData[0] - '0'; 					com.button_status  	= 10*(receivedData[1] - '0') + (receivedData[2] - '0'); 					osMessageQueuePut(input_queueHandle, &com, 0, 2000);
        	}
        }
```

 Since the available function HAL_UART_Receive recives as an argument the predetermined amount of bytes this function  was developed to  read until finds the “\n” char.

```c

int FON_UART_Receive(char *received, uint16_t timeout,UART_HandleTypeDef *huartX) {
    HAL_StatusTypeDef status;
    unsigned char receivedChar;
    int index = 0;
    while (1) {
        status = HAL_UART_Receive(huartX, &receivedChar, 1, timeout); //tries to read next availabe byte in the buffer

        if (status == HAL_OK) { //verifies if the read was succesfull
            if (receivedChar == '\n') {
            	received[index] = '\0';
                return 1; //returns success value
            }
            else {
                received[index] = receivedChar;
                index++; //keeps reading
            }
        }
        else {
            received[0] = '\0';
            return 0; //returns error value
        }
    }
}
```

After the receiving part of the task, a new loop is open that keeps iterating while the bluetooth_queue has objects to send, before breaking the loop and calling the osDelay function to stay blocked for half a second.

```c
while (1){
        	if (osMessageQueueGet(bluetooth_queueHandle, &res, NULL, 250) == osOK) {
        	    HAL_UART_Transmit(&huart1, res, strlen(res), 1000);
        	}
        	else{
        		break;
        	}
        }
```

The phone side of the communication was simulated via the Serial Bluetooth Terminal app, using macros to send consistent info to the board and visualizing the returned status of the program

![print1.png](img/print1.png)

![print2.png](img/print2.png)

Once the Bluetooth communication was properly implemented, and able to be used as a debugging tool, the next step was to implement the first and simplest sensor, the brakes. They were connected to the PA5 and PA6 pins configured as “External Interrupt Mode” with both rising and falling edges detection triggers with a pull-up connected, so the interruption is called on both the push and release motion of the handle.

The interruption routine is called with the triggered pin as argument, but since different pins may have the same predefined callback function, two checks need to be done, first to verify which pin called the interruption, and second to identify the edge being detected.

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
command com;
if (GPIO_Pin == BREAK_1_Pin ) {
	com.button_id 		= 1;
	if (HAL_GPIO_ReadPin(BREAK_1_GPIO_Port, GPIO_Pin) == GPIO_PIN_SET) {
		// Your code for rising edge -> released
		com.button_status  	= 0;
	}else {
		// Your code for falling edge -> pressed
		com.button_status  	= 1;
	}
}
```

After the command struct is configured, it is inserted in the input queue, but since functions called from interruptions must have the timeout argument equals zero, a check is made to ensure the operation was successful, otherwise the error message is sent to the Bluetooth queue to notify the user.

```c
osStatus_t status 	=  osMessageQueuePut(input_queueHandle, &com, 0, 0); //this block tries to send to the  input_queue
if (status != osOK){													 // in case it fails,
	char* str = osStatusToString(status);								 // then sent the error to the bluetooth_queue
	osMessageQueuePut(bluetooth_queueHandle, &str, 0, 0);
}
```

The only change between the switch used for prototyping and the final button was referring to the relation of each signal edge to the user’s physical movement since the first try used hardware where the pins were disconnected on the default state, while in the final one, the idle state keeps both pins connected.

Next was the implementation of the pedal encoder, which happened in 3 phases, the first using the KY-040 rotary encoder, as a simple attempt to read and count the pulses emitted. To do so the TIM1 timer was configured with the clock source set as ETR2, to read the pin PA12 as its clock.

![encoder1.png](img/encoder1.png)

The TIM1 module has the pre-build function to get the number of clocks passed, as well as reset said count to 0, making it so to calculate the frequency with the encoder is rotating it is only necessary the time between reads and the number of pulses expected per round of the encoder

```c
counter = __HAL_TIM_GET_COUNTER(&htim1); //getter for the nummber of pulses
hz_input =(**float**) 1.0*counter/(PulsesPerRound*delay);
```

The second test used an F249 optical encoder, connected to a DC motor with a gear, to simulate multiple continuous pulses, to ensure the board was able to calculate the rotational speed with consistency, before validating with the real sensor in the bike.

![encoder2.png](img/encoder2.png)

Once all the essential sensors were implemented, the next step was to actuate the motor according to said sensors. For prototyping, while the new motor speed controller is in development, a HobbyKing™ SS Series 190-200A electronic speed controller was used, with its input connected to TIM2 channel 1, configured as PWM Generation, on the pin PA0. The timer parameters were set as Prescaler value of 720 and Counter Period equal to 1000, so that the value written to the peripheral corresponds to the duty cycle in milliseconds.

To calculate the output to the ESC a function was implemented to re-map the output from the operating range of the pedal (to ignore really slow movements) to the range from the ESC protocol (between 100 and 200 milliseconds).

```c
float MAP(float int_IN){
		if(int_IN < pedal_min){return pwm_min;}	
		else if(int_IN > pedal_max){return pwm_max;}
		else{
			return((((int_IN - pedal_min)*(pwm_max - pwm_min))/(pedal_max - pedal_min)) + pwm_min);
		}
}
```

Regarding the Disabling mechanism, all that was necessary was to control a digital pin, in this case PC8, set as Output Push-Pull, to be set as HIGH when enabling the bike to operate, and turning to LOW when the bike is locked.

For safety reasons, the pin to disable the bicycle couldn't be changed while moving, so a software timer of three minutes was introduced. This timer is started or reset when movement is detected on the pedal, and after it times out, it sends a command to allow changes on the PC8 status, considering that the bicycle has fully stopped and is safe to disable without endangering the user.

The second timer, integrated with the RTC (real-time clock), is set to activate one week after receiving a user authentication signal. This configuration defines the interval before requiring another authentication. Once the timer expires, a locking signal is sent to the motor.

With all the sensors and timers implemented the next step was to design the finite states machine to handle the decision-making. To do so, four binary variables were used as described.

| Variable name | Values | Comments |
| --- | --- | --- |
| lock | 1 if the bike is locked
0 if is able to work |  |
| stationary | 1 if no input has been detected in the pedal for more than 3 minutes |  |
| brk | 1 if the brakes are pressed
0 if not | In C the word brake is reserved |
| onhold |  | Holds the value to update the lock variable while the bike is moving |

In the state machine code, the first check is if the bike is not moving, and if it is the case, the value of the lock can change according to the onhold variable. Next, it is verified if the bike should be enabled, if so, the next check is regarding the brake, if they are not pressed, the output is set according to the MAP function previously described, if one of the brakes is pressed, then the output is the the minimal value of the PWM, to keep the motor idle. After all the tests, the relay control pin is set to 1 and the output value is written to TIM2. If the first check about the lock fails, then the same 2 values are set, but this time as  pwm_min and 0, to stop the speed controller and switch the relay to short-circuit the motor.

```c
if(stationary){
		lock=onhold;
}
if(!lock){
		if(!brk){
				//normal movement
				output = MAP(freq);
		}
		else{
				//breaking
				output = pwm_min;
		}

		HAL_GPIO_WritePin(rele_ctrl_GPIO_Port, rele_ctrl_Pin, 1);
		htim2.Instance->CCR1 = output;
}
else{
		//locked
		HAL_GPIO_WritePin(rele_ctrl_GPIO_Port, rele_ctrl_Pin, 0);
		htim2.Instance->CCR1 = pwm_min;
}
```