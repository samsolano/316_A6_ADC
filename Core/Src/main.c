#include "main.h"
#include "stdio.h"
#include "string.h"


void SystemClock_Config(void);
void ADC_init(void);
void UART_print(char *str);
void UART_init(void);
uint32_t voltage_calibration(uint32_t volt_reading);


# define TOTAL_DATA_COUNT 20
# define RESET 0
# define LARGE_NUMBER 50000

uint8_t conversion_ready = RESET;
uint16_t reading_data = RESET;
uint16_t values[TOTAL_DATA_COUNT] = {RESET};
uint8_t value_count = RESET;
uint16_t max = RESET;
uint16_t min = LARGE_NUMBER;
uint32_t average = RESET;


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  ADC_init();
  UART_init();
  NVIC_EnableIRQ(USART2_IRQn);
  __enable_irq();
  UART_print("\033[2J\033[H");   //clear and go top left


  ADC1->CR |= ADC_CR_ADSTART; //start conversion

  while (1)
  {

	  //if the conversion is ready and we havent gotten 20 pieces of data yet
	  if(conversion_ready & (value_count <= TOTAL_DATA_COUNT))
	  {
		  values[value_count++] = reading_data; //put values into array
		  conversion_ready = RESET; //get ready for another conversion
		  ADC1->CR |= ADC_CR_ADSTART; //start another conversion
	  }
	  if((value_count > TOTAL_DATA_COUNT)) //if were larger than 20 start processing data
	  {
		  for(uint8_t i = 0; i < TOTAL_DATA_COUNT; i++) //get mins maxs and average
		  {
			  if(values[i] < min)
			  {
				  min = values[i];
			  }
			  if(values[i] > max)
			  {
				  max = values[i];
			  }
			  average += values[i];
		  }

		  average /= 20;

		  //all 10's and 100's are to select the three different digits individually to get the voltage value to two decimal points
		  UART_print("max: ");
		  UART_print('0' + (voltage_calibration(max) / 100));
		  UART_print('0' + ((voltage_calibration(max) / 10) % 10));
		  UART_print('0' + (voltage_calibration(max) % 10));

		  UART_print("min: ");
		  UART_print('0' + (voltage_calibration(min) / 100));
		  UART_print('0' + ((voltage_calibration(min) / 10) % 10));
		  UART_print('0' + (voltage_calibration(min) % 10));

		  UART_print("ave: ");
		  UART_print('0' + (voltage_calibration(average) / 100));
		  UART_print('0' + ((voltage_calibration(average) / 10) % 10));
		  UART_print('0' + (voltage_calibration(average) % 10));

		  //Reset
		  value_count = RESET;
		  max = RESET;
		  min = LARGE_NUMBER;
		  UART_print("\033[H");
		  average = RESET;



	  }


  }
}







uint32_t voltage_calibration(uint32_t volt_reading)
{
	//calibrated
	return ((812 * volt_reading) - 1600) / 10000;
}






void ADC1_2_IRQHandler(void)
{
	//check to see if conversion is done
	if(ADC1->ISR & ADC_ISR_EOC)
	{
		//clear flag, set global variable and read into another global variable
		ADC1->ISR &= ~ADC_ISR_EOC;
		conversion_ready = 1;
		reading_data = ADC1->DR;
	}

}











void ADC_init(void) {
    // Enable the ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Set ADC clock to synchronous HCLK / 1
    ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

    // Power up the ADC and voltage regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;

    // Wait for 20 µs for the voltage regulator to stabilize
    for (uint32_t i = 0; i < 50000; i++);

    // Configure PA0 as analog input
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |= (GPIO_MODER_MODE0);  // PA0 in analog mode
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
    GPIOA->ASCR |= GPIO_ASCR_ASC0;  // Enable analog switch for PA0

    // Single-ended mode for channel 5 (PA0)
    ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

    // Calibrate the ADC
    ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // Ensure ADC is disabled and single-ended calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to complete

    // Enable the ADC
    ADC1->ISR |= (ADC_ISR_ADRDY); // Clear ready bit
    ADC1->CR |= (ADC_CR_ADEN);
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready

    // Configure sequence for single channel (channel 5)
    ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

    // Configure for single conversion, 12-bit resolution, right-aligned data
    ADC1->CFGR = 0;

    // Configure sample time to 2.5 ADC clock cycles
    ADC1->SMPR1 &= ~(ADC_SMPR1_SMP5);
    ADC1->SMPR1 |= (0 << ADC_SMPR1_SMP5_Pos); // 2.5 cycles for channel 5

    // Enable end-of-conversion interrupt
    ADC1->IER |= (ADC_IER_EOC);
    // Enable ADC interrupt in NVIC
    NVIC_EnableIRQ(ADC1_2_IRQn);
}



void UART_init(void)
{
	 // Enable clocks for GPIOA and USART2
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // Enable USART2 clock

	// Configure PA2 as USART2_TX and PA3 as USART2_RX
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Clear mode bits
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); // Set alternate function mode

//	GPIOA->AFR[0] |= ~(GPIO_AFRL_AFSEL2_3);  // Set AF7 (USART2) for PA2
//	GPIOA->AFR[0] |= ~(GPIO_AFRL_AFSEL3_3); // Set AF7 (USART2) for PA3

	GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos); // Set AF7 (USART2) for PA2
	GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL3_Pos); // Set AF7 (USART2) for PA3

	// Configure USART2 for 115200 baud rate (assuming 4 MHz clock)
	USART2->BRR = 208; // Set baud rate divisor for 115200 baud (4 MHz / (16 * 115200) ≈ 35)

	// Enable USART2, transmitter, and receiver and receive data register
	USART2->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE);
}


void UART_print(char *str)
{
	while (*str != '\0')
	{               // Loop until the end of the string
		while (!(USART2->ISR & USART_ISR_TXE)) {}
		USART2->TDR = *str++;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
