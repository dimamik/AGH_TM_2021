
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include "dbgu.h"
#include "term_io.h"
#include "ansi.h"

#include "usbh_platform.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FREERTOS_LAB 1

extern ApplicationTypeDef Appli_state;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  debug_init(&huart3);
  xprintf(ANSI_BG_MAGENTA "\nNucleo-144 F429ZI nucleo-basic-v4 project" ANSI_BG_DEFAULT "\n");
  xprintf(ANSI_FG_YELLOW "Funkcja xprintf dziala." ANSI_FG_DEFAULT "\n");
  printf(ANSI_FG_GREEN "Funkcja printf tez dziala." ANSI_FG_DEFAULT "\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_INT_Pin */
  GPIO_InitStruct.Pin = EXT_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXT_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

//a general-purpose statically allocated buffer for file system operations
#define FS_BUFFER_SIZE	(8*1024)
static uint8_t fs_buffer[FS_BUFFER_SIZE];

void waitForUsbMsd(void)
{
  xprintf("\nwaiting for USB device...\n");
	do{
		vTaskDelay(100);
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}while(Appli_state != APPLICATION_READY);
	xprintf("USB device ready!\n");
	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
}


/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN 5 */
//  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  MX_DriverVbusFS(0);   //wlacza zasilanie urzadzenia USB
  waitForUsbMsd();
   /*
   * ================================================
   * This is a nice place to create additional tasks,
   * queues, and semaphores for this lab exercise :)
   * ================================================
   */


	xprintf("entering StartDefaultTask main loop");
	xprintf("\n");

	while(1)
	{
	  HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
	  vTaskDelay(100);
	    char key = inkey();
	    switch(key)
	    {
			case '!':
				xprintf("Format the USB drive? Send Y to confirm, any other to abort\n");
				do
				{
					key = inkey();
					if(key == 'Y')
					{
						xprintf("Format in progress...\n");
						FRESULT res = f_mkfs("0:",FM_FAT|FM_FAT32,512,fs_buffer,512);
						xprintf("f_mkfs result code: %d\n",res);
					}
					else if(key != 0)
					{
						xprintf("abort\n");
					}
				}while(!key);
				key = 0;
				break;
	        case '0':
	        	MX_DriverVbusFS(1);
	        	xprintf("Connected USB Device is OFF\n");
	        	break;
	        case '1':
	        	MX_DriverVbusFS(0);
	        	xprintf("Connected USB Device is ON\n");
	        	waitForUsbMsd();
	        	break;
	        case 'i':
	        	xprintf("Re-initializing FatFS...");
	        	MX_FATFS_Init();
	        	xprintf(" reinit done\n");
	        	break;
			case 'w':
			{
			  xprintf("write-append test\n");
			  FRESULT res;
			  UINT bw;
			  FIL file;
			  const int TEXTBUF_SIZE = 128;
			  char text[TEXTBUF_SIZE];
			  sprintf(text,"Linia tekstu. Czas systemowy to: %08d\n",(int)xTaskGetTickCount());
			  xprintf("f_open... ");
			  res = f_open(&file,"0:/test.txt",FA_WRITE|FA_OPEN_APPEND);
			  xprintf("res=%d\n",res);
			  if(res) break;
			  xprintf("f_write... ");
			  res = f_write(&file,text,strlen(text),&bw);
			  xprintf("res=%d, bw=%d\n",res,bw);
			  f_close(&file);
			  break;
			}

			case 'r':	//read and disp in text-only mode
			case 'h':	//read and disp in text-hex mode
			{
			  xprintf("read test!\n");
			  FIL file;
			  FRESULT res = f_open(&file,"0:/test.txt",FA_READ);
			  xprintf("f_open res=%d\n",res);
			  if(res) break;

			  const uint32_t BUF_SIZE = 64;
			  char buf[BUF_SIZE];
			  int dispSizeLimit = 4*1024;

			  UINT br;
			  xprintf(ANSI_FG_YELLOW "File contents:" ANSI_FG_DEFAULT "\n");
			  do
			  {
				xprintf("\n" ANSI_FG_YELLOW "f_read...");
				res = f_read(&file,buf,BUF_SIZE,&br);
				xprintf("f_read res=%d" ANSI_FG_DEFAULT "\n",res);

				if((res == FR_OK) && (br))
				{
				  if(key=='h')
				  {
					  debug_dump(buf,br);
				  }
				  else
				  {
					  debug_txt_limit(buf,br);
				  }
				  dispSizeLimit -= br;
				}
				else
				{
				  break;
				}
			  }while( (br>0) && (dispSizeLimit > 0) );
			  f_close(&file);
			  break;
			}

			case 'c':{ // c - create (new file)

				// wszystko zwiazane z plikami jest w pliku ff.h

				xprintf("Enter new file name\n");
				char filename[256];
				// teraz wczytujemy nazwe pliku znak po znaku az natrafimy na enter, wtedy przerywamy wczytywanie i mamy nazwe pliku
				int i = 0;
				for(; i < 255; i++){
					char readChar;

					readChar = inkey();
					vTaskDelay(100);

					xprintf("your char: %c\n", readChar);

					if(readChar == '\n' || readChar == '\r' || readChar == '\r\n'){
						break; // jezeli podano enter
					}
					else{
						filename[i] = readChar;
					}
				}

				filename[i] = '\0'; // koniec nazwy pliku (znak konca napisu)

				FIL file;
				FRESULT res = f_open(&file, filename, FA_WRITE|FA_CREATE_ALWAYS);

				if(!res)
					xprintf("Your file %s has been created\n", filename);
				else
					xprintf("File %s could not be created\n", filename);

				f_close(&file);
				break;
			}

			case 'a':{ //a - add (string to file)
				xprintf("Enter filename\n");
				char filename[256];

				// wczytywanie dokladnie tak samo jak bylo wyzej
				// teraz wczytujemy nazwe pliku znak po znaku az natrafimy na enter, wtedy przerywamy wczytywanie i mamy nazwe pliku
				int i = 0;
				for(; i < 255; i++){
					char readChar;

					readChar = inkey();
					vTaskDelay(100);

					xprintf("your char: %c\n", readChar); // comment later,

					if(readChar == '\n' || readChar == '\r' || readChar == '\r\n'){
						break; // jezeli podano enter
					}
					else{
						filename[i] = readChar;
					}
				}

				filename[i] = '\0'; // koniec nazwy pliku (znak konca napisu)

				FIL file;
				UINT byte_count;
				FRESULT res = f_open(&file, filename, FA_OPEN_APPEND|FA_WRITE);

				if(!res)
					xprintf("Your file %s has been opened\n", filename);
				else
					xprintf("File %s could not be opened\n", filename);

				const int APPENDED_STRING_MAX_SIZE = 256;
				char appendedString[APPENDED_STRING_MAX_SIZE];
				sprintf(appendedString, "This text will be appended to file\n");

				f_write(&file, appendedString, strlen(appendedString), &byte_count);

				xprintf("To file %s has been written %d bytes\n", filename, byte_count);

				f_close(&file);
				break;
			}

			case 'o':{ // o - open file (and write to terminal)
				xprintf("Enter filename\n");
				char filename[256];

				// wczytywanie dokladnie tak samo jak bylo wyzej
				// teraz wczytujemy nazwe pliku znak po znaku az natrafimy na enter, wtedy przerywamy wczytywanie i mamy nazwe pliku
				int i = 0;
				for(; i < 255; i++){
					char readChar;

					readChar = inkey();
          // TODOQ Po co ten delay?
					vTaskDelay(100);

					xprintf("your char: %c\n", readChar); // comment later,

					if(readChar == '\n' || readChar == '\r' || readChar == '\r\n'){
						break; // jezeli podano enter
					}
					else{
						filename[i] = readChar;
					}
				}

				filename[i] = '\0'; // koniec nazwy pliku (znak konca napisu)

				const int FILE_CONTENT_MAX_SIZE = 1024; // ograniczenie do 1 KB zgodnie z poleceniem
				char fileContent[FILE_CONTENT_MAX_SIZE];

				FIL file;
				UINT byte_count;
				FRESULT res = f_open(&file, filename, FA_READ);

				if(!res)
					xprintf("Your file %s has been opened\n", filename);
				else
					xprintf("File %s could not be opened\n", filename);

				res = f_read(&file, fileContent, FILE_CONTENT_MAX_SIZE, &byte_count);

				if(!res){
					debug_txt_limit(fileContent, byte_count); // wypisanie stringa z fileContent o dlugosci byte_count
					xprintf("\nTo file %s has been written %d bytes\n", filename, byte_count);
				}
				else{
					xprintf("File %s cannot be read\n", filename);
				}

				f_close(&file);
				break;
			}

			case 'd':{ // d - directory (show files)
				DIR directory;
				FRESULT res;
				FILINFO finfo;

				res = f_opendir(&directory, "/"); // katalog glowny

				if(!res)
					xprintf("Root dir is opened. Printing content...\n");
				else{
					xprintf("Root dir cannot be opened.\n");
					break; // dalej juz nic nie zrobimy
				}

				while(1){
					res = f_readdir(&directory, &finfo);

					if(finfo.fattrib & AM_DIR) // jezeli jest to katalog
						xprintf("/%s | DIR\n", finfo.fname);
					else // jezeli plik
						xprintf("/%s | FILE\n", finfo.fname);
				}

				f_closedir(&directory);
				break;
			}

			case 's':{ // s - speed test
				FIL file;
				FRESULT res;
				UINT byte_count;

				res = f_open(&file, "speedTest", FA_CREATE_ALWAYS|FA_WRITE|FA_READ);

				if(!res)
					xprintf("File speedTest has been opened\n");
				else
					xprintf("File speedTest could not be opened\n", filename);

				 // tworzymy przykladowy file content i wpisujemy go do pliku "speedTest" tyle razy zeby plik mial 4MiB,
				// przy okazji mierzymy czas zapisu do pliku
				const int FILE_CONTENT_MAX_SIZE = 128;
				char fileContent[FILE_CONTENT_MAX_SIZE];
				sprintf(fileContent, "TESTtest");

				// TODO: sprawdzic czy faktycznie wpisze sie 4MiB do pliku

				// pomiar szybkosci zapisu
        // TODO Uwaga, tu zapisujemy nie 4MB a 4*1024*1024 razy 8B (bo rozmiar tekstu)
				TickType_t start = xTaskGetTickCount();
				for(int i = 0; i < 4096 * 1024 / FILE_CONTENT_MAX_SIZE; i++){
					f_write(&file, fileContent, FILE_CONTENT_MAX_SIZE, &byte_count);
				}
				TickType_t end = xTaskGetTickCount();

				xprintf("Writing time for 4MiB file: %d ms\n", (end - start));

				// wynika z proporcji:
				/*
				 * 4 MiB - time * 0.001 s
				 * y MiB - 1 s
				 * => 4 MiB * 1s = time * 0.001s * y MiB
				 * */
				xprintf("Writing speed: %d MiB/s\n", 4 * 1000 / (end - start));

				f_rewind(&file); // zaczynamy zabawe od nowa

				start = xTaskGetTickCount();
				for(int i = 0; i < 4096 * 1024 / FILE_CONTENT_MAX_SIZE; i++){ // tym razem odczyt
					f_read(&file, fileContent, FILE_CONTENT_MAX_SIZE, &byte_count);
				}
				end = xTaskGetTickCount();

				xprintf("Reading time for 4MiB file: %d ms\n", (end - start));
				xprintf("Reading speed: %d MiB/s\n", 4 * 1000 / (end - start));

				f_close(&file);
				break;
			}
	    }
	}

  #if FREERTOS_LAB==0

	  /* Infinite loop */
	  for(;;)
	  {
		//LD3 zmieni stan (toggle)
		HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);

		//LD2 wlaczona, czekamy 250 tickow i wylaczamy
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
		osDelay(250);
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
		osDelay(250);
	  }
  #endif

  /* USER CODE END 5 */ 
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
