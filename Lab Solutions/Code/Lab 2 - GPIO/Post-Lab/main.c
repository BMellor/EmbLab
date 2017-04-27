
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


int main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable the GPIOA clock in the RCC
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
    
    // Initialize pins PC6 - PC9, explicitly clearing bits
    GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18); // Set PC6 - PC9 to output mode
    GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
    GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)); // Set push-pull output
    GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18)); // Set low speed
    GPIOC->PUPDR &= ~(0x000FF000); // Set no-pull up/down (clear bits 12-19)
  
    // Initialize button pin PA0
    GPIOA->MODER &= ~((1 << 1) | (1 << 0)); // Set PA0 to input mode
    GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 12)); // Set low speed
    GPIOC->PUPDR |= ~(1 << 1); // Set pull-down
    GPIOC->PUPDR &= ~(1 << 0);
    
    
    GPIOC->ODR |= (1 << 6); // Start with PC6 set (RED)
    uint8_t debouncer = 0;
    while (1) {
        HAL_Delay(10); // Delay 10ms 
        
        debouncer = debouncer << 1; // Shift debouncer left by one
        if(GPIOA->IDR & (1 << 0)) {
            debouncer |= (1 << 0); // Set lowest bit if button is pressed
        }
        
        if(debouncer == 0x7F) { // Detect stable rising edge on button
            
            if(GPIOC->ODR & (1 << 6)) { // If PC6 (RED) is set
                GPIOC->ODR &= ~(1 << 6); // Clear RED
                GPIOC->ODR |= (1 << 9);  // Set GREEN
            } else if(GPIOC->ODR & (1 << 9)) { // If PC9 (GREEN) is set  
                GPIOC->ODR &= ~(1 << 9); // Clear GREEN
                GPIOC->ODR |= (1 << 7);  // Set BLUE
            } else if(GPIOC->ODR & (1 << 7)) { // If PC7 (BLUE) is set  
                GPIOC->ODR &= ~(1 << 7); // Clear BLUE
                GPIOC->ODR |= (1 << 8);  // Set ORANGE
            } else { // PC8 (ORANGE) is set  
                GPIOC->ODR &= ~(1 << 8); // Clear ORANGE
                GPIOC->ODR |= (1 << 6);  // Set RED
            }
            
            // Is more efficient to use the BSRR
            /*
            if(GPIOC->ODR & (1 << 6)) { // If PC6 (RED) is set
                GPIOC->BSRR = (1 << 22) | (1 << 9); // Clear RED, Set GREEN
            } else if(GPIOC->ODR & (1 << 9)) { // If PC9 (GREEN) is set  
                GPIOC->BSRR = (1 << 25) | (1 << 7); // Clear GREEN, Set BLUE
            } else if(GPIOC->ODR & (1 << 7)) { // If PC7 (BLUE) is set  
                GPIOC->BSRR = (1 << 23) | (1 << 8); // Clear BLUE, Set ORANGE
            } else { // PC8 (ORANGE) is set  
                GPIOC->BSRR = (1 << 24) | (1 << 6); // Clear ORANGE,Set RED
            }
            */
            
        }
    }
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
