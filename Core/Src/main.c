/* USER CODE BEGIN Header */
/**
  * ****************************************************************************
  *
  * This is a simple implementation of the game BOUNCE!
  * It's a dumbed down version of the Breakout game invented by Steve Wozniak.
  * The game has no bricks at the top and the goal is just to bounce the ball
  * arround for as long as possible. Depending on which region of the paddle the
  * ball hits, you can get different amounts of points.
  *
  * This code is really basic, but I tried to comment this code as much as
  * possible. Feel free to modify and add functionalities.
  *
  * The purpose of this simple game implementations is to make use of the
  * ssd1306 oled 64p x 128p 0.96'' module with the STM32F411 (Black Pill) or
  * any STM32 board, provided that you modify the <ssd1306_BlackPill_conf.h> to
  * the device you are using.
  *
  * Here we use the SPI1 peripheral with 2 as the pre-scaler, but make it slower
  * if you need to. Use PA5 (Clock) and PA7(MOSI) as the SPI1 pins and turn the
  * pull-ups ON.
  *
  * PB0 is attached to the Res Pin of the Display.
  * PB1 is attached to the DC Pin of the Display.
  * PB2 is attached to the CS Pin of the Display.
  * PB9 is attached to the button and is called GAME_BUTTON with the pull-up ON.
  * 	When the button is pushed, it has to pull the pin down.
  * PA1 is attached to the center tap of the potentiometer.
  *
  * Code by ISAAC JERÔNIMO MOREIRA
  *
  * January 9th, 2023
  * Fortaleza, Ceará, Brasil
  *
  * ****************************************************************************
  *
  * The ssd1306 library was originally written by Olivier Van den Eede (4ilo) in
  * 2016.
  * Some refactoring was done and SPI support was added by Aleksander Alekseev
  * (afiskon) in 2018.
  *
  * *****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
	 * In order to understand this, bare in mind that the display begins it's coordinate system at the top left corner
	 * where the X and Y coordinates are 0.
	 * */
	#define edgeOffset 3 //Defines the offset of the border. It's equals the radius of the ball plus one pixel
	#define leftBorder edgeOffset // Defines the boundaries of the left border
	#define rightBorder (SSD1306_WIDTH - edgeOffset) // Defines the boundaries of the right Border. It's equals the Display Width minus the Edge Offset
	#define topBorder edgeOffset // Same as leftBorder
	#define bottonBorder (SSD1306_HEIGHT - edgeOffset) // Same logic as rightBorder
	#define paddleYborder (SSD1306_HEIGHT - edgeOffset - 1) // The Paddle is 1 pixel higher than the botton border.
	#define maxLife 5 // Every player starts with the maximum number of lives. Change this as you please,
	                  // or create a logic in game that can change this in different levels.
	#define adcmaxval 4096 // This is the resolution of the ADC inside the STM32 (12-bits)
	#define paddlelength 32 // This is the length of the paddle. Feel free to change it or create a logic in game that can change this in different levels.
	#define PotConverter (SSD1306_WIDTH - paddlelength)/adcmaxval // Just a small MAP function to convert the value read from the potentiometer
	                                                              // and convert it to the resolution of the screen. Feel free to change it as you need it.
	#define paddleCorner 6 // Defines the size of the corners of the paddle. The corners reflect the ball in a different way and give more points to the player
	#define MaxRefreshPeriod 50 // The speed of the game is ruled by a delay at the end of the loop. It's crude, but it works.


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// 8-Bits Unsigned Variables

uint8_t Xpos = edgeOffset, // Variable that tracks the X coordinate of the ball.
		Ypos = edgeOffset, // Variable that tracks the Y coordinate of the ball.
		Xvelocity = 3, // X component of the ball velocity.
		Yvelocity = 2, // Y component of the ball velocity.
		BGNDcolour = White, // Background colour
		ELMTScolour = Black, // Colour of the graphic elements;
		InGame = 0, // Flag that signals if the game is beeing played (1) or not (0).
		GameStarted = 0, //Flag that signals if the game has ever started (1) or not (0)
		playerLifenum = maxLife; // Variable to keep track of the playes lifes, initiated at the maximum.

// 16-Bits Unsignd Variables

  uint16_t paddlePos; // The position of the Paddle has to be 16 bits this variable is used multiple times
                      // to convert the 12-Bits ADC value read from the potentiometer to the correct range.
                      // We do this in order to conserve RAM as good practice, though this program isn't
                      // very complex or big;

// 32-Bites unsigned Variables (ARM-32)
  int StartTime = 0, // Stores the TickTimer current value. This number can be huge, so an INT is needed.
	  PresentTime = 0; // Keeps track of the time elapsed from the start of the game to the current time.
                       // Maybe the player is very good and has infinite patience to play this very exciting game
                       // so we have to accommodate them.

// Float Variables

  float potVal = 0;// It has to be a float in order to receive a multiplication by a float later. It will all make sense soon, I promise

// Signed Integer Variables

  signed int PaddleInteractions = 0,// keeps track of the paddle interactions to build the score
		     playerScorenum = 0, //keeps track of the score
			 topScorenum = 0, // keeps track of the highest score after each game
			 RefreshPeriod = MaxRefreshPeriod; // Used to control the refresh rate of the game.

  // Text buffers
  char playerLifetext[20], // Used to store the text converted from the number of Lives remaining
       playerScoretext[20], //Same logic
	   topScoretext[30]; // Same logic

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /*
    * Ok, lets start now
    *
    * First of all, the display must be initiated. The library ssd1306.h has this function and it sends to
    * the ssd1306 display module a sequence of commands in order to initiate the displaying of graphics
    *
    * */


   ssd1306_Init();//Initiate display.

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(GameStarted && InGame )//If the game has ever started and the player is playing
	  {
	  potVal = 0; // Starts the value read from the potentiometer as 0, otherwise it will start calculating averages with a previous read value
	  for(uint8_t i = 0; i < 50; i++)//acquires the ADC value 50 times
	  {
	  HAL_ADC_Start(&hadc1); //Starts the ADC module
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);// Start conversion from Analog to Digital
	  potVal += HAL_ADC_GetValue(&hadc1);// Sums the value read to the previously read values
	  }

	  potVal = potVal/50;//Calculates by the average

	  paddlePos = potVal * PotConverter;//Small MAP conversion from the value read and averaged from the ADC

	  // Draws the paddle, which is consisted of 3 lines. The top line is shorter to make visible the corners.
	  // this could be done with a function, but given how short this code is, I just can't be bothered
	  ssd1306_Fill(BGNDcolour);//Background
	  ssd1306_Line(paddlePos, 63, paddlePos + paddlelength, 63, ELMTScolour);//paddle
	  ssd1306_Line(paddlePos, 62, paddlePos + paddlelength, 62, ELMTScolour);//paddle
	  ssd1306_Line(paddlePos  + paddleCorner, 61, paddlePos + paddlelength - paddleCorner, 61, ELMTScolour);//paddle

	  //Draws the score to the screen
	  ssd1306_SetCursor(2, 2);// sets the cursor to a fixed position
	  sprintf(playerScoretext, "Score: %d", playerScorenum);// converts the sore number to the score text
	  ssd1306_WriteString(playerScoretext, Font_6x8, ELMTScolour);//draws it to the screen

	  //Draws the players Lives to the screen. The logic is the same as the score drawing, but a little math is needed first
	  playerLifenum = (playerLifenum <= 0)? maxLife : playerLifenum;//just make sure that there is no negative lives.
	  sprintf(playerLifetext, "Lives: %d", playerLifenum);
	  ssd1306_SetCursor(79, 2);
	  ssd1306_WriteString(playerLifetext, Font_6x8, ELMTScolour);

	  //Here the ball is drawn
	  ssd1306_FillCircle(Xpos,Ypos,2, ELMTScolour);//The ball is a filled circle
	  ssd1306_DrawPixel(Xpos-1, Ypos-1, BGNDcolour );//with a little pixel as the reflection

	  //Finally the screen is updated
	  ssd1306_UpdateScreen();

      /*Everything that follows will be shown at the next frame.
       *
       * This is done so the start of the game is shown the first time it starts. Something like a DO WHILE loop.
       *
	   */

	  //First the position of the ball is calculated
	  //Here we use the uniform linear motion, simplified. The time base is the frame rate.
	  Xpos += Xvelocity;
	  Ypos += Yvelocity;

	  //Here We implement the paddle interaction with the ball
	  if(Ypos >= paddleYborder)// If the ball is ad the Y coordinate of the paddle
	  {
		if(((Xpos >= (paddlePos + paddleCorner))&&(Xpos <= (paddlePos + paddlelength - paddleCorner))))//and if the ball is at the center portion of the paddle
		{
			if(Xpos == (paddlePos + (paddlelength/2)))//and if the ball is at the absolute center of the paddle
			{
				uint8_t swap = Yvelocity;//we swap the X and Y components of the velocity to give the game a different dinamic
				Yvelocity = -Xvelocity;//The paddle reflects the ball as the physics mandates
				Xvelocity = swap;
				PaddleInteractions += 100;//This is very difficult to do, so the player deserves a lot of points
			}else
			{
			Yvelocity = - Yvelocity;// If the ball is at the center portion but not at the absolute center,
			PaddleInteractions += 5;// The payer gets just 5 points, because it's easy to hit those spots
			RefreshPeriod--;//the game gets a little bit faster
			}
		}else{
			if(
			   ((Xpos > (paddlePos))&&((Xpos < (paddlePos + paddleCorner)))) ||
		      ((Xpos > (paddlePos + paddlelength - paddleCorner)) && (Xpos < (paddlePos + paddlelength)))
			  )//If the ball hits the edges of the paddle
			{
			Yvelocity = -Yvelocity; // The ball reverses the vector of it's motion
			Xvelocity = -Xvelocity; // at a slight angle
			PaddleInteractions += 10; //The player gets 10 points because its a little more difficult to hit the edges
			RefreshPeriod -= 2; // The game gets a little bit faster
			}
		}
	  }

	  //Here We implement the interactions with the walls of the display

	  // if the ball hits the right or left walls, it bounces IE it
	  // reverses the direction of the X velocity vector.
	  if(Xpos < leftBorder)
	  {
		  Xpos = leftBorder;
		  Xvelocity = - Xvelocity;
	  }
	  if(Xpos > rightBorder)
	  {
		  Xpos = rightBorder;
		  Xvelocity = - Xvelocity;
	  }

	  // If the ball hits the botton
	  if(Ypos > bottonBorder){
		  uint8_t swap = Yvelocity;//we swap the X and Y components of the velocity to give the game a different dinamic
		  Yvelocity = -Xvelocity;//The paddle reflects the ball as the physics mandates
		  Xvelocity = swap;
	 	  playerLifenum--;//Takes one life from the player
	 	  PaddleInteractions -= 10;//Takes 10 points from the player
	 	  Xpos = edgeOffset;// Restarts the position of the ball so it goes back to the top left corner
	 	  Ypos = edgeOffset;
	 	  RefreshPeriod -=2;//Makes the game a lot faster
	  }
	  if(Ypos < topBorder){//If it hits the top, it bounces. U know the drill
	 	  Ypos = topBorder;
	 	  Yvelocity = - Yvelocity;
	  }

	  //Here we handle the game state
	  InGame = (playerLifenum == 0)? 0 : 1;//If the player looses all its lives, we flag the game to stop

	  PresentTime = (HAL_GetTick() - StartTime)/1000;//Every time the frame is updated, we update the time elapsed from the start of the game
	  playerScorenum = PaddleInteractions + PresentTime;// The score is calculated. It takes into account the paddle interactions
	                                                    // and how long the player lasted in game.

	  }else
	  {
		  if(GameStarted)//If the game has ever started, we show a game over screen
		  {
			  //HAL_Delay(500);
			  ssd1306_Fill(Black);//filed
			  ssd1306_SetCursor(16, 2);
			  ssd1306_WriteString("GAME OVER", Font_11x18, White);

			  topScorenum = (playerScorenum > topScorenum)? playerScorenum : topScorenum;//If the player has the highest score, the top Score is updated

			  //Here we display the information
			  ssd1306_SetCursor(15, 29);
			  sprintf(topScoretext, "Top Score: %d", topScorenum);
			  ssd1306_WriteString(topScoretext, Font_7x10, White);

			  ssd1306_SetCursor(25, 40);
			  sprintf(playerScoretext, "Score: %d", playerScorenum);
			  ssd1306_WriteString(playerScoretext, Font_7x10, White);
			  ssd1306_SetCursor(31, 51);
			  ssd1306_WriteString("Press START", Font_6x8, White);
			  ssd1306_UpdateScreen();

		  }else
		  {
			  //This is the start screen
			  ssd1306_Fill(Black);//filled
			  ssd1306_SetCursor(10, 11);
			  ssd1306_WriteString("BOUNCE!", Font_16x26, White);
			  ssd1306_SetCursor(31, 40);
			  ssd1306_WriteString("Press START", Font_6x8, White);
			  ssd1306_SetCursor(13, 55);
			  ssd1306_WriteString("BY: ISAAC MOREIRA", Font_6x8, White);
			  ssd1306_UpdateScreen();
		  }
		  if(0 == HAL_GPIO_ReadPin(GAME_BUTTON_GPIO_Port, GAME_BUTTON_Pin))// Reads the Start Button input
			                                                               // Here we don't make any debouncing. In this case, it isn't needed
		  {
		 	   InGame = 1;//Flag that the game has started
		 	   GameStarted = 1; //We have played at least once
		 	   StartTime = HAL_GetTick();// Gets the time the game started
		 	   Ypos = edgeOffset; //Puts the ball at the left top corner
		 	   Xpos = edgeOffset;
		 	   PaddleInteractions = 0; //There has been no interactions with the paddle
		 	   RefreshPeriod = MaxRefreshPeriod; //The game starts slow
		 	   Xvelocity = 3;//We reset the velocity components
		 	   Yvelocity = 2;//
		   }

	  }

	  RefreshPeriod = (RefreshPeriod < 10)? 10 : RefreshPeriod;//Makes sure that the game doesn't run too fast
	  HAL_Delay(RefreshPeriod);//Dictates the frame rate of the game

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
  RCC_OscInitStruct.PLL.PLLN = 200;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_Res_Pin|OLED_DC_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_Res_Pin OLED_DC_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_Res_Pin|OLED_DC_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B6_SCL_Pin B7_SDA_Pin */
  GPIO_InitStruct.Pin = B6_SCL_Pin|B7_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B8_Pin */
  GPIO_InitStruct.Pin = B8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GAME_BUTTON_Pin */
  GPIO_InitStruct.Pin = GAME_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GAME_BUTTON_GPIO_Port, &GPIO_InitStruct);

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
