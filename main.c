/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_host.h"
#include "stdio.h"
#include "arm_math.h"
#include <stdint.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Choose which process to use for convolution.
// (Only use one at a time)
//#define STANDARD_CONVOLUTION 0 // Regular convolution process.
//#define UNROLLED_CONVOLUTION 1 // Convolution but unrolled with a factor of 3.
//#define INTRINSIC_CONVOLUTION 2 // Convolution but using a MAC intrinsic instruction (SMLABB).
#define INTRINSIC_UNROLLED_CONVOLUTION 3 // Convolution but using a MAC intrinsic instruction (SMLABB) and loop unrolling.

#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000+4*n)))

/* This is the start address of where we are going to put our input data */
#define DATA_START_ADDRESS	0x08020000

/* 64px images by 64px images with 3 channels for RGB */
#define IMG_WIDTH 64
#define IMG_HEIGHT 64
#define OUT_WIDTH 64
#define OUT_HEIGHT 64
#define CHANNELS 3
#define TOTAL_IMAGES 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

// The pointers below points to the location where the combined.bin is loaded.
// This pointer moves at each pixel of the image.
volatile int8_t* image_pixel = (volatile int8_t *)DATA_START_ADDRESS;
// This pointer moves at each image contained in combined.bin.
volatile int8_t* image_pointer = (volatile int8_t *)DATA_START_ADDRESS;

// Define a simple 3x3 kernel. This is an edge filter as an example.
// The following formatting is in Q4.4 where 0xF0 denotes -1 (decimal)
// and 0x40 denotes 4 (decimal).
int8_t kernel_edge[3*3] = {0x00, 0xF0, 0x00,
					       0xF0, 0x40, 0xF0,
					       0x00, 0xF0, 0x00};
// This is an identity kernel as an example. In Q4.4 format, 0x40 denotes 1.
int8_t kernel_identity[3*3] = {0x00, 0x00, 0x00,
					           0x00, 0x10, 0x00,
					           0x00, 0x00, 0x00};
// This is a blur kernel as an example. In Q4.4 format,0x02 denotes 0.125.
int8_t kernel_blur[3*3] = {0x02, 0x02, 0x02,
					       0x02, 0x02, 0x02,
					       0x02, 0x02, 0x02};
uint8_t kernel_size = 3;

// This contains the red, green, and blue pixels read from memory.
uint8_t image_block_red[3*3];
uint8_t image_block_green[3*3];
uint8_t image_block_blue[3*3];

// The filtered images on the stack. There are five images.
uint8_t output[IMG_WIDTH*IMG_HEIGHT*CHANNELS*TOTAL_IMAGES];

#ifdef INTRINSIC_CONVOLUTION
uint8_t chosen_process = 2;
#elif UNROLLED_CONVOLUTION
uint8_t chosen_process = 1;
#elif INTRINSIC_UNROLLED_CONVOLUTION
uint8_t chosen_process = 3;
#else
uint8_t chosen_process = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void process_image(int8_t *kernel);
void standard_convolution(int16_t *accumulator, int8_t *kernel);
void intrinsic_convolution(int16_t *accumulator, int8_t *kernel);
void unrolled_convolution(int16_t *accumulator, int8_t *kernel);
void intrinsic_unrolled_convolution(int16_t *accumulator, int8_t *kernel);
uint8_t saturate(int16_t accumulate);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  ITM_Port32(31) = 1;
  process_image(kernel_edge);
  ITM_Port32(31) = 2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char* ptr, int len){
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

/**
 * @brief This function performs image convolution reading from the STM32's memory
 * 		  to fetch the image data and then storing the filtered image in another 1D array.
 * 		  This implementation reduces the resolution of the images depending on the size
 * 		  of the kernel.
 * @param This is the 3x3 kernel to use to filter the image.
 * @retval None
 */
void process_image(int8_t *kernel){
	// This will total to kernel_size^2, taking only the pixels intersecting with the kernel.
	int input_index = 0;
	// The accumulators for each pixel convolution is initialized to zero.
	// The ordering is red, green, blue.
	int16_t accumulator[3] = {0x0000, 0x0000, 0x0000};
	uint16_t output_index = 0x0000;

	for (int image = 0; image < TOTAL_IMAGES; image++) {
		volatile int8_t* inner_pointer;
		for(int height = 0; height < OUT_HEIGHT; height++){
			// The first term is the start address of combined.bin.
			// The second term is the start address of the images depending on the image iteration.
			// The third term indicates the image row.
			image_pointer = image_pixel + (image*IMG_WIDTH*IMG_HEIGHT*CHANNELS) + (IMG_WIDTH*CHANNELS*height);
			for(int width = 0; width < OUT_WIDTH; width++){
				input_index = 0;
				// Reset accumulator for the next set of kernel to image overlaps.
				accumulator[0] = 0x0000;
				accumulator[1] = 0x0000;
				accumulator[2] = 0x0000;

				image_pointer = image_pointer + CHANNELS;
				inner_pointer = image_pointer;
				// Store the pixels that is intersecting with the kernel.
				for(int k = 0; k < kernel_size; k++){
					for(int l = 0; l < kernel_size; l++){

						//handling the edge case
						if (((width < 1 && l < 1) || (width >= IMG_WIDTH-1 && l > 1)) ||
							((height < 1 && k < 1) || (height >= IMG_HEIGHT-1 && k > 1))) {
							image_block_red[input_index] = 0x00;
							image_block_green[input_index] = 0x00;
							image_block_blue[input_index] = 0x00;
						} else {
							image_block_red[input_index] = *(inner_pointer + (3*l));
							image_block_green[input_index] = *(inner_pointer + (3*l) + 1 );
							image_block_blue[input_index] = *(inner_pointer + (3*l) + 2 );
						}
						input_index++;
					}
					inner_pointer = inner_pointer + (IMG_WIDTH*CHANNELS);
				}

				/*Ensure that only one of the convolution functions is running to avoid redundancy.*/
				// Perform instrinsic convolution using SMLABB.
				if (chosen_process == 2) {
					intrinsic_convolution(accumulator, kernel);
				}
				// Perform unrolled convolution with a factor of 3.
				else if (chosen_process == 1) {
					unrolled_convolution(accumulator, kernel);
				}
				else if (chosen_process == 3) {
					intrinsic_unrolled_convolution(accumulator, kernel);
				}
				// Perform standard convolution on the gathered pixels against the kernel.
				else {
					standard_convolution(accumulator, kernel);
				}
				// Perform saturation, for a Q4.12 number.
				output[output_index] = saturate(accumulator[0]);
				output[output_index+1] = saturate(accumulator[1]);
				output[output_index+2] = saturate(accumulator[2]);
				output_index+=CHANNELS;
			}
		}
	}
}

/**
 * @brief Perform standard convolution processes unoptimized.
 * @param accumulator holds three values for the accumulate of red, green, and blue pixels.
 * 		  kernel typically holds 9 values forming 3x3 filter.
 * @retval None
 */
void standard_convolution(int16_t *accumulator, int8_t *kernel) {
	for(int m = 0; m<(kernel_size*kernel_size); m++){
		 accumulator[0] = accumulator[0] + kernel[m]*image_block_red[m];
		 accumulator[1] = accumulator[1] + kernel[m]*image_block_green[m];
		 accumulator[2] = accumulator[2] + kernel[m]*image_block_blue[m];
	}
}

/**
 * @brief Perform convolution using one of the MAC intrinsics (SMLABB).
 * @param accumulator holds three values for the accumulate of red, green, and blue pixels.
 * 		  kernel typically holds 9 values forming 3x3 filter.
 * @retval None
 */
void intrinsic_convolution(int16_t *accumulator, int8_t *kernel) {
	for(int m = 0; m<(kernel_size*kernel_size); m++){
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[0])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_red[m]), [acc] "r" (accumulator[0])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[1])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_green[m]), [acc] "r" (accumulator[1])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[2])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_blue[m]), [acc] "r" (accumulator[2])
						);
	}
}

/**
 * @brief Perform convolution but with unrolled factor of 3.
 * @param accumulator holds three values for the accumulate of red, green, and blue pixels.
 * 		  kernel typically holds 9 values forming 3x3 filter.
 * @retval None
 */
void unrolled_convolution(int16_t *accumulator, int8_t *kernel) {
	for(int m = 0; m<(kernel_size*kernel_size); m+=3){
		accumulator[0] = accumulator[0] +
						(kernel[m]*image_block_red[m]) +
						(kernel[m+1]*image_block_red[m+1]) +
						(kernel[m+2]*image_block_red[m+2]);
		accumulator[1] = accumulator[1] +
						(kernel[m]*image_block_green[m]) +
						(kernel[m+1]*image_block_green[m+1]) +
						(kernel[m+2]*image_block_green[m+2]);
		accumulator[2] = accumulator[2] +
						(kernel[m]*image_block_blue[m]) +
						(kernel[m+1]*image_block_blue[m+1]) +
						(kernel[m+2]*image_block_blue[m+2]);
	}
}

void intrinsic_unrolled_convolution(int16_t *accumulator, int8_t *kernel) {
	for(int m = 0; m<(kernel_size*kernel_size); m+=3){
		// Process red
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[0])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_red[m]), [acc] "r" (accumulator[0])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[0])
						:[op1] "r" (kernel[m+1]), [op2] "r" (image_block_red[m+1]), [acc] "r" (accumulator[0])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[0])
						:[op1] "r" (kernel[m+2]), [op2] "r" (image_block_red[m+2]), [acc] "r" (accumulator[0])
						);
		 // Process green
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[1])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_green[m]), [acc] "r" (accumulator[1])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[1])
						:[op1] "r" (kernel[m+1]), [op2] "r" (image_block_green[m+1]), [acc] "r" (accumulator[1])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[1])
						:[op1] "r" (kernel[m+2]), [op2] "r" (image_block_green[m+2]), [acc] "r" (accumulator[1])
						);
		 // Process blue
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[2])
						:[op1] "r" (kernel[m]), [op2] "r" (image_block_blue[m]), [acc] "r" (accumulator[2])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[2])
						:[op1] "r" (kernel[m+1]), [op2] "r" (image_block_blue[m+1]), [acc] "r" (accumulator[2])
						);
		 __asm volatile("SMLABB %[result], %[op1], %[op2], %[acc]"
						:[result] "=r" (accumulator[2])
						:[op1] "r" (kernel[m+2]), [op2] "r" (image_block_blue[m+2]), [acc] "r" (accumulator[2])
						);
	}
}

/**
 * @brief Perform saturation, for a Q4.12 number the maximum pixel range is 0x0FFF.
		  The minimum pixel range is 0x0000 for a Q4.12 number.
		  When saturating a Q4.12 format, only consider the fractional bits and
		  revert back to Q0.8 format.
   @param accumulate is the value to saturate.
   @retval The saturated number.
 */
uint8_t saturate(int16_t accumulate) {
	if(accumulate > 0x0FFF){
		accumulate = 0x0FFF;
	}
	else if(accumulate < 0x0000){
		accumulate = 0x0000;
	}
	return (uint8_t)(accumulate >> 4);
}

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
