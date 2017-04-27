/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#define RED (1 << 6)
#define GREEN (1 << 9)
#define BLUE (1 << 7)
#define ORANGE (1 << 8)

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Initialize LED pins -------------------------------------------------------*/
static inline void Init_LED(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Possibly redundant, but ensures that peripheral is active
    
    // Initialize pins PC6 - PC9 using temp variable (prevents momentary unknown states when modifying bits)
    uint32_t mode = GPIOC->MODER & ~(0x000FF000); // Set temp variable to MODER state with all LED bits cleared
             mode |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18); // Set PC6 - PC9 to output mode
    GPIOC->MODER = mode; // Write all LED bits to proper state at once 
    GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)); // Set push-pull output
    GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18)); // Set low speed
    GPIOC->PUPDR &= ~(0x000FF000); // Set no-pull up/down (clear bits 12-19)
    GPIOC->BRR = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9); // Clear all LED pins
}

/* Initialize button pin -------------------------------------------------------*/
static inline void Init_Button(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Possibly redundant, but ensures that peripheral is active
    
    GPIOA->MODER &= ~((1 << 1) | (1 << 0)); // Clear button bits & set to input mode
    GPIOA->PUPDR &= ~(1 << 0);
    GPIOA->PUPDR |= (1 << 1); // Set pull-down
}

int main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock
	
    // Enable Peripheral Clocks in RCC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    
    Init_LED(); // Intialize all LED and button pins
    Init_Button();

    // Configure the EXTI PA0 Interrupt 
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set EXTICR1 pin mux to PA0
    EXTI->IMR |= (1 << 0); // Enable/unmask EXTI0 interrupt
    EXTI->RTSR |= (1 << 0); // Enable rising-edge trigger
    NVIC_SetPriority(EXTI0_1_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
    
    GPIOC->BSRR = GREEN; // Start with green LED set
    while (1) {
        __WFI(); // Main loop does nothing but put processor to sleep
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
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
