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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //gpiob
RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //gpioc
RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //I2C2 

//PC6 red
GPIOC->MODER |= (0x01 << 12);
GPIOC->OTYPER &= ~(0x1 << 6);
GPIOC->OSPEEDR &= ~(0X3 << 12);
GPIOC->PUPDR &= ~(0X3 << 12);

//PC7 blue
GPIOC->MODER |= (0x01 << 14);
GPIOC->OTYPER &= ~(0x1 << 7); 
GPIOC->OSPEEDR &= ~(0X3 << 14);
GPIOC->PUPDR &= ~(0X3 << 14);

//PC8 orange
GPIOC->MODER |= (0x01 << 16);
GPIOC->OTYPER &= ~(0x1 << 8); 
GPIOC->OSPEEDR &= ~(0X3 << 16);
GPIOC->PUPDR &= ~(0X3 << 16);

//PC9 green
GPIOC->MODER |= (0x01 << 18);
GPIOC->OTYPER &= ~(0x1 << 9); 
GPIOC->OSPEEDR &= ~(0X3 << 18);
GPIOC->PUPDR &= ~(0X3 << 18);


// PB11
GPIOB->MODER |= (0x02 << 22);
GPIOB->AFR[1] |= (0x1 << 12); 
GPIOB->OTYPER = (0x1 << 11); 
GPIOB->PUPDR |= (0x1 << 22); 

// PB13
GPIOB->MODER |= (0x02 << 26);
GPIOB->AFR[1] |= (0x5 << 20); // AF5
GPIOB->OTYPER = (0x1 << 13); 
GPIOB->PUPDR |= (0x1 << 26); 

//PB15
GPIOB->MODER |= (0x0 << 30); // input mode

// PB14 
GPIOB->MODER |= (0x01 << 28); 
GPIOB->OTYPER &= ~(0x0 << 14); 
GPIOB->BSRR = (1 << 14);

// PC0
GPIOC->MODER |= (0x01 << 0);
GPIOC->BSRR = (0x1 << 0);

// configure bus timing for 100kHz standard mode I2C2
I2C2->TIMINGR |= (1 << 28); 
I2C2->TIMINGR |= (0x13 << 0); // SCLL 0x13
I2C2->TIMINGR |= (0xF << 8); // SCLH 0xF
I2C2->TIMINGR |= (0x2 << 16); // SDADEL 0x2
I2C2->TIMINGR |= (0x4 << 20); // SCLDEL 0x4

I2C2->CR1 |= I2C_CR1_PE;

  /* USER CODE END 2 */
/* 5.4 who am I

I2C2->CR2 = (1 << 16) | (0x6B << 1) | I2C_CR2_START; 

// 0 in loop
while(!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		GPIOC->ODR ^= (0x1 << 6); // error led red
	}
}

I2C2->TXDR = 0x0F; // who am i register

// write check
while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		// LED error
		GPIOC->ODR ^= (0x1 << 9);
	}
}

I2C2->CR2 = (1 << 16) | (0x6B << 1) | I2C_CR2_START | I2C_CR2_RD_WRN; // NBYTES to 42 and SADD to 0x14 and start and read


//reg not empty
while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led orange error
		GPIOC->ODR ^= (0x1 << 8);
	}
// continue if RXNE flag is set
}


//receive/read 
while(!(I2C2->ISR & (I2C_ISR_TC |  I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
// blue led error
		GPIOC->ODR ^= (0x1 << 7);
	}
}

I2C2->CR2 |= I2C_CR2_STOP; // stop bit in cr2 register to release I2C bus


if(I2C2->RXDR == 0xD4)
{
	// blue LED
	GPIOC->ODR ^= (0x1 << 7);
}
*/ 

//gyroscope
//Initializing 
I2C2->CR2 = (2 << 16) | (0x6B << 1) | I2C_CR2_START;  // #of bytes 2 & write mode & start

while(!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//red led error
		GPIOC->ODR ^= (0x1 << 6);
	}
}

I2C2->TXDR = 0x20; // CTRL_REG1 addr

while(!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		// red led error
		GPIOC->ODR ^= (0x1 << 6);
	}
}

// x, y axis en
I2C2->TXDR = 0xB;

//write
while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led error
		GPIOC->ODR ^= (0x1 << 9);
	}
}

I2C2->CR2 |= I2C_CR2_STOP;

// infinite loop
while (1) {
// x ax enable
I2C2->CR2 = (1 << 16) | (0x6B << 1) | I2C_CR2_START; 

while(!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//red led error
		GPIOC->ODR ^= (0x1 << 6);
	}
}

I2C2->TXDR = 0xA8; //reg x low n hgih

//TC wait
while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led error
		GPIOC->ODR ^= (0x1 << 9);
	}
}


int16_t xHigh;
int16_t xLow;
I2C2->CR2 = (2 << 16) | (0x6B << 1) | I2C_CR2_START | I2C_CR2_RD_WRN;

//not empty check
while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led orange error
		GPIOC->ODR ^= (0x1 << 8);
	}
}

// read data 
xHigh = I2C2->RXDR;

//recieve reg not empty
while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led orange error
		GPIOC->ODR ^= (0x1 << 8);
	}
}

// read data 
xLow = I2C2->RXDR;

//TC wait
while(!(I2C2->ISR & (I2C_ISR_TC |  I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led blue error
		GPIOC->ODR ^= (0x1 << 7);
	}
}


//y axis enable write
I2C2->CR2 = (1 << 16) | (0x6B << 1) | I2C_CR2_START; 

while(!(I2C2->ISR & (I2C_ISR_NACKF | I2C_ISR_TXIS)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led red error
		GPIOC->ODR ^= (0x1 << 6);
	}
}

I2C2->TXDR = 0xAA; // read y high and low

// check write
while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//error led light 
		GPIOC->ODR ^= (0x1 << 9);
	}
}


//y enable high/low
int16_t yHigh ;
int16_t yLow;
I2C2->CR2 = (2 << 16) | (0x6B << 1) | I2C_CR2_START | I2C_CR2_RD_WRN;

// check receive register not empty
while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
{
	if(I2C2->ISR & I2C_ISR_NACKF)
	{
		//led orange error
		GPIOC->ODR ^= (0x1 << 8);
	}
}

// read data
yHigh = I2C2->RXDR;

// check receive register not empty
while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
{
if(I2C2->ISR & I2C_ISR_NACKF)
{
// errror orange LED, data not received by slave
GPIOC->ODR ^= (0x1 << 8);
}
// continue if RXNE flag is set
}

//read data
yLow = I2C2->RXDR;

//receive/read 
while(!(I2C2->ISR & (I2C_ISR_TC |  I2C_ISR_NACKF)))
{
if(I2C2->ISR & I2C_ISR_NACKF)
	{
	// blue led error
		GPIOC->ODR ^= (0x1 << 7);
	}
}


int16_t x = (xHigh << 8) | xLow;
int16_t y = (yHigh << 8) | yLow;


if(x > 4000) // green
{
GPIOC->ODR |= (0x1 << 9);
GPIOC->ODR &= ~(0x1 << 8);
}
else if(x < -4000) // orange
{ 
GPIOC->ODR |= (0x1 << 8);
GPIOC->ODR &= ~(0x1 << 9);
}
else
{
GPIOC->ODR &= ~(0x1 << 8);
GPIOC->ODR &= ~(0x1 << 9);
}


if(y > 4000) // red
{
GPIOC->ODR |= (0x1 << 6);
GPIOC->ODR &= ~(0x1 << 7);
} 
else if(y < -4000) // blue
{
GPIOC->ODR |= (0x1 << 7);
GPIOC->ODR &= ~(0x1 << 6);
}
else
{
GPIOC->ODR &= ~(0x1 << 6);
GPIOC->ODR &= ~(0x1 << 7);
}

HAL_Delay(100);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
