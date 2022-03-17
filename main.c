/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
ADC_HandleTypeDef hadc;

COMP_HandleTypeDef hcomp1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t RX_response[30];
uint8_t Low_Byte_ADCvalue_MSB,Low_Byte_ADCvalue_LSB;// golobal
uint8_t Device_Found;
uint8_t Low_Byte_Pressure_hPa_MSB,Low_Byte_Pressure_hPa_LSB,High_Byte_Pressure_hPa_MSB,High_Byte_Pressure_hPa_LSB=0;// doivent etre declarés comme global
int32_t Raw_Pressure = 0;
uint8_t LPS25HB_STATUS = 0;
uint32_t Pressure_hPa=0;
float Pressure;
uint16_t ADCvalue;	


/*---------------------------------Addresses  LPS25HB Pressure sensor----------------------------------------------------------*/
#define LPS25HB_ADDRESS 							0x5C	//Note that SA0 = 0 so LPS25HB_ADDRESS=SAD is 1011100= (0x5C)h
                                            //SAD+R/W=?==>>SAD+R=(B9h) and SAD+w=(B8h) 
#define LPS25HB_DEVICE_ID							0xBD	//Device ID, the Default value in the WHO_AM_I 	Register =	0xBD
/*---------------------------------Register Locations LPS25HB Pressure sensor--------------------------------------------------*/
#define LPS25HB_WHO_AM_I							0x0F	//Who am I register location
#define LPS25HB_STATUS_REG						0x27	//Tells whether the Pressure Data is ready or is being overrun
#define LPS25HB_PRESS_OUT_XL					0x28	//(LSB) Pressure output value
#define LPS25HB_PRESS_OUT_L						0x29  //(mid part) Pressure output value
#define LPS25HB_PRESS_OUT_H						0x2A	//(MSB) Pressure output value
#define LPS25HB_CTRL_REG1							0x20	//Contains PD, BDU and more
#define LPS25HB_CTRL_REG2							0x21	//Contains one-shot mode and FIFO settings
#define LPS25HB_RES_CONF							0x10	//Pressure and temperature Resolution
/*---------------------------------Configuration Bits LPS25HB Pressure sensor-------------------------------------------------*/
#define LPS25HB_CTRL_REG1_PD					0x80	//Power Down when 0, active mode when 1 (Default 0); PD=1==>>1 active mode is selected
#define LPS25HB_CTRL_REG1_BDU					0x4		//Block Data Update: 0 Continuous mode, 1 read LSB,Mid,MSB first; BDU=1 ==>> output registers not updated until MSB and LSB have been read)
#define LPS25HB_CTRL_REG2_ONE_SHOT		0x1		//One shot mode enabled, obtains a new dataset
#define LPS25HB_RES_CONF_AVGP0				0x1		//Pressure resolution Configuration
#define LPS25HB_RES_CONF_AVGP1				0x2		//Pressure resolution Configuration. AVGP0=AVGP1=1==>> Nr. internal average=512
#define LPS25HB_STATUS_REG_PDA				0x2		//Pressure data available;(0: new data for pressure is not yet available;1: new data for pressure is available)
                                            // PDA=1 is selected
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_COMP1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t LPS25HB_Init(void)
	{
	Device_Found =0;// doit etre globale
	
	//Read data from register and check signature	
  //I2C_Read_Reg(LPS25HB_ADDRESS,LPS25HB_WHO_AM_I); //lire le contenue du registre WHO_AM_I
		                                              //LPS25HB_ADDRESS=SAD is 1011100= (0x5C)h. but: SAD+R=(B9h) and SAD+w=(B8h)
	HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x000F,1,&RX_response[2],1,0xff); // je ne suis pas sure pour le syntaxe et les valeures des adresses 
	//Check if device signature is correct                            // est ce qu'on met LPS25HB_ADDRESS=0x5C OU SAD+R=(B9h)?
	//if (I2C1->RXDR == LPS25HB_DEVICE_ID) 
	if (RX_response[2]==LPS25HB_DEVICE_ID)	
	  {
		Device_Found = 1;   //Device ID has the Default value in the WHO_AM_I 	Register =	0xBD
	  }
	else
		{
		Device_Found = 0;
	}
	if(Device_Found){
		  //Power on the device and Block Data Update
		//I2C_Write_Reg(LPS25HB_ADDRESS,LPS25HB_CTRL_REG1,(LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_BDU));// ecrire(0x80|0x4)=10000100 ds CTRL_REG1 pour activer le bit PD et bit BDU
                                                                                                //LPS25HB_ADDRESS=SAD=0x5C.et SAD+w=(B8h)
	uint8_t Power_on=0;
  Power_on=(LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_BDU);
	HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0020,1,&Power_on,1,0xff);
		//Configure the resolution for pressure for 128 internal averages
//	HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0020,1,&test,1,0xff);// je ne suis pas sure
	//HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x0020,1,&RX_response[3],1,0xff);
		//Configure the resolution for pressure for 128 internal averages	
		//HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0020,1,(uint8_t*)(LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_BDU),1,0xff);// je ne suis pas sure
		//Configure the resolution for pressure for 128 internal averages
  //I2C_Write_Reg(LPS25HB_ADDRESS,LPS25HB_RES_CONF,LPS25HB_RES_CONF_AVGP0); //AVGP1=0 je l'ai ecrasé par les 00000001 de la LPS25HB_RES_CONF_AVGP0
	uint8_t averages=0;                                                        // 8 pour temp et 32 pour pression
  averages=(LPS25HB_RES_CONF_AVGP0);
HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0010,1,&averages,1,0xff);	
		//I2C_Write_Reg(LPS25HB_ADDRESS,LPS25HB_RES_CONF,(LPS25HB_RES_CONF_AVGP0 | LPS25HB_RES_CONF_AVGP1));// pour plus de flexibilité 512 internal averages
	}
	return(Device_Found);
}
	

float LPS25HB_Pressure_Read(void)
	{
	
	//Local Variables
	uint8_t PRESS_OUT_XL = 0;
	uint8_t PRESS_OUT_L = 0;
	uint8_t PRESS_OUT_H = 0;
	//float Pressure = 0;
//uint32_t Raw_Pressure = 0;
	//uint8_t LPS25HB_STATUS = 0;
	//uint32_t Pressure_hPa=0;
	
	//Start a temperature conversion
  //I2C_Write_Reg(LPS25HB_ADDRESS,LPS25HB_CTRL_REG2,LPS25HB_CTRL_REG2_ONE_SHOT);// LPS25HB_CTRL_REG2_ONE_SHOT=0x1
	uint8_t Start_Temp=0;
	//uint8_t RX;
  Start_Temp=(LPS25HB_CTRL_REG2_ONE_SHOT);
	HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0021,1,&Start_Temp,1,0xff); 
 // HAL_I2C_Mem_Write(&hi2c1,0x00B8,0x0021,1,(uint8_t*)(LPS25HB_CTRL_REG2_ONE_SHOT),1,0xff);// je ne suis pas sure	
	
	//Wait for Temperature data to be ready
	do{
  //I2C_Read_Reg(LPS25HB_ADDRESS,LPS25HB_STATUS_REG);
	HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x0027,1,&RX_response[3],1,0xff); // je ne suis pas sure	
		//LPS25HB_STATUS = I2C1->RXDR;
		LPS25HB_STATUS =RX_response[3];
	}while((LPS25HB_STATUS & LPS25HB_STATUS_REG_PDA) == 0); //Temperature data & pressure data are ready(attendre les deux oblegatoirement)
	
	//Read the pressure output registers
  //I2C_Read_Reg(LPS25HB_ADDRESS,LPS25HB_PRESS_OUT_XL);
	HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x0028,1,&RX_response[4],1,0xff); // je ne suis pas sure	
	//PRESS_OUT_XL = I2C1->RXDR;
	PRESS_OUT_XL =RX_response[4];
  //I2C_Read_Reg(LPS25HB_ADDRESS,LPS25HB_PRESS_OUT_L);
	HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x0029,1,&RX_response[5],1,0xff); // je ne suis pas sure	
	//PRESS_OUT_L = I2C1->RXDR;
	PRESS_OUT_L = RX_response[5];
  //I2C_Read_Reg(LPS25HB_ADDRESS,LPS25HB_PRESS_OUT_H);
	HAL_I2C_Mem_Read(&hi2c1,0x00B9,0x002A,1,&RX_response[6],1,0xff); // je ne suis pas sure	
	//PRESS_OUT_H = I2C1->RXDR;
	PRESS_OUT_H =RX_response[6]; 
	
	//Read the reference Register
	
	/*	Combine pressure into 24 bit value
			PRESS_OUT_H is the High bits 	23 - 16
			PRESS_OUT_L is the mid bits 	15 - 8
			PRESS_OUT_XL is the lsb				7 - 0
	*/
	Raw_Pressure = ((PRESS_OUT_H << 16) | (PRESS_OUT_L << 8) | (PRESS_OUT_XL));
	
	//convert the 2's complement 24 bit to 2's complement 32 bit
	if (Raw_Pressure & 0x00800000)    // nomre negatif 24ème bit =1
		{
			Raw_Pressure |= 0xFF000000;       // conversion de Complement à 2 sur 24 bits vers comlement à 2 sur 32 bits
	  }
	
	//Calculate Pressure in mbar
	Pressure = Raw_Pressure/4096.0f;// POURQHOI sur 4096.0f? ds datasheet 4096.resultat type float
		///////////////////////// convertir la valeure reelle de Pressure en hexadecimal pour l'ecrire par SPI ds User Memory//////////
	Pressure_hPa=(int)Pressure;// je ne suis pas sure si la conversion de float vers int se fait comme ça en C?
	High_Byte_Pressure_hPa_MSB=Pressure_hPa>> 24;/// on prends seulement 24 bits LSB (le maximum peut etre representé seulement sur 24 bits)
	High_Byte_Pressure_hPa_LSB=Pressure_hPa>> 16;
	Low_Byte_Pressure_hPa_MSB=Pressure_hPa>> 8;
	Low_Byte_Pressure_hPa_LSB=Pressure_hPa;

return(Pressure); // est ce que pour les variables (Low_Byte_Pressure_hPa_LSB,...)declarées comme globale retournent les bonne valeurs? 
}
 void SL900_Write_One_Bytes (uint16_t addresse , uint8_t data)
	 { 
	uint8_t AdressH,AdressL;
	AdressH =((addresse>>8)&0x07);	 
	AdressL=(addresse);
	HAL_GPIO_WritePin(SL900_CS_GPIO_Port, SL900_CS_Pin, GPIO_PIN_SET);//set CS =1
	HAL_SPI_Transmit(&hspi1,&AdressH,1,100);
	HAL_SPI_Transmit(&hspi1,&AdressL,1,100);			
	HAL_SPI_Transmit(&hspi1,&data,1,100);	
	HAL_GPIO_WritePin(SL900_CS_GPIO_Port, SL900_CS_Pin, GPIO_PIN_RESET);// Reset SC=0	
	 }
	uint16_t ADCvalue=0;
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
  MX_ADC_Init();
  MX_COMP1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
//HAL_SuspendTick ();
	  HAL_COMP_Start (&hcomp1 );
	//	PWR->CR |= PWR_CR_ULP ;
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON ,PWR_STOPENTRY_WFI ); // Enter stop mode as a rest mode 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_I2C_DeInit(&hi2c1);//Deintialisate the I2C peripheral to save power
	  HAL_SPI_DeInit (&hspi1 );//Deintialisate the SPI peripheral to save power
	//	HAL_COMP_DeInit (&hcomp1 );
		
		//Get ADC value
		//HAL_Delay (500);
		HAL_ADC_Init (&hadc );
		HAL_ADC_Start (&hadc ); // Start the ADC conversion
	//	HAL_ADC_PollForConversion (&hadc ,100);
		ADCvalue = HAL_ADC_GetValue (&hadc ); // Get value and stock it in the ADCvalue register
		HAL_ADC_Stop (&hadc );// Stop conversion
		// Deviding the 16 bit register value into two registers of 8 bit
		// The lowest bits
  	 Low_Byte_ADCvalue_MSB = ADCvalue >>8 ; // the highest bits
		Low_Byte_ADCvalue_LSB= ADCvalue ;
	//	HAL_Delay (500);
    HAL_ADC_DeInit(&hadc );		// Deintialisate the ADC peripheral to save power
  	// Get I2C value
    HAL_I2C_Init (&hi2c1);	// Initialise the I2C perepheral
		HAL_PWREx_EnableLowPowerRunMode(); // Entreing low power run mode
		HAL_SuspendTick ();
    LPS25HB_Init ();	// call the LPS25HB_Init function	
    LPS25HB_Pressure_Read ();//call the LPS25HB_Pressure_Read 
    HAL_I2C_DeInit (&hi2c1 ); // Deintialisate the I2C peripheral to save power
		//Write I2C and ADC value in the SL900 chip
	   // HAL_SPI_Init (&hspi1 ); // initialise the SPI peripheral	
    //Send the ADC and SPI collected data into the SL900 chip
	//	HAL_Delay (500);
	  MX_SPI1_Init();
	  // uint8_t data = 0x00;
	  //SL900_Write_One_Bytes(0x0064,data); 
		SL900_Write_One_Bytes(0x0065,Low_Byte_ADCvalue_MSB); 
    SL900_Write_One_Bytes(0x0066,Low_Byte_ADCvalue_LSB);	
		SL900_Write_One_Bytes(0x0067,RX_response[6]);
		SL900_Write_One_Bytes(0x0068,RX_response[5]);
		SL900_Write_One_Bytes(0x0069,RX_response[4]);
		HAL_SPI_DeInit (&hspi1 );//Deintialisate the SPI peripheral to save power
    // enter to Stop mode
	  //HAL_PWREx_DisableLowPowerRunMode ();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON ,PWR_STOPENTRY_WFI );
		
//	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_10B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_VREFINT;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.LPTIMConnection = COMP_LPTIMCONNECTION_DISABLED;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Mode = COMP_POWERMODE_ULTRALOWPOWER;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

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
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CRCPolynomial = 7;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SL900_CS_GPIO_Port, SL900_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA8 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SL900_CS_Pin */
  GPIO_InitStruct.Pin = SL900_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SL900_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
