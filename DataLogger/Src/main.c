
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
int16_t values[8];								// signed variable to store calibration values for BMP180
uint16_t Val[3];								// unsigned variable to store calibration values for BMP180
#define 	AC1 					values[0]	// label for calibration value
#define 	AC2 					values[1]	// label for calibration value
#define 	AC3 					values[2]	// label for calibration value
#define 	AC4 					Val[0]		// label for calibration value
#define 	AC5 					Val[1]		// label for calibration value
#define 	AC6 					Val[2]		// label for calibration value
#define 	B1 						values[3]	// label for calibration value
#define 	B2 						values[4]	// label for calibration value
#define 	MB 						values[5]	// label for calibration value
#define 	MC 						values[6]	// label for calibration value
#define 	MD 						values[7]	// label for calibration value
#define 	g 						2			// g value to setup Accelerometer
#define 	ACC_RAW_VALUE_ADDR 		0X28		// address to read raw values from accelerometer
#define 	ACC_G_CONFIG_REG_ADDR 	0x23		// address to configure G values in accelerometer
#define 	ACC_CONFIG_REG_ADDR 	0x20		// address to configure accelerometer
#define 	BMP180_RAW_VALUE_ADDR 	0XF6		// address to read raw values from BMP180
#define 	BMP180_CNTL_REG_ADDR 	0xF4		// address to configure BMP180
#define 	ACC_READ_ID_REG_ADDR	0x0F		// address to read ID from accelerometer
#define 	ACC_EN_DEVICE 			0x27		// enable x,y,z axis, enable normal mode, and low sample rate of IIS328DQ
#define 	RECURCISE_READ_EN 		0X80		// command to read succesive data bytes from IIS328DQ
#define 	LOW_POWER 				0x00		// LOW POWER MODE of BMP180 sensor
#define 	STANDARD 				0x01		// STANDARD MODE of BMP180 sensorCommand to read temperature from BMP180
#define 	HIGH_RESOLUTION 		0x02		// HIGH RESOLUTION MODE of BMP180 sensor
#define 	ULTRA_HIGH_RESOLUTION 	0x03		// ULTRA HIGH RESOLUTION MODE of BMP180 sensor
#define 	READ_TEMP_CMD 			0x2E		// Command to read temperature from BMP180
#define 	READ_PRESSURE_CMD 		0x34		// Command to read temperature from BMP180
#define 	PRESSURE 				0x01
/*Functions to perform various actions*/
void WriteRegister(uint8_t DevAddress,char address,char content);
void ReadRegister(uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length);
int32_t ReadSensor(char data,char mode,uint8_t length);
int16_t ReadInt(char address);
uint16_t ReadUInt(char address);
void ReadBMP180();
void ReadCalPara();
void send(int32_t data,uint8_t type);
void ReadAccelerometer();
void SetupAccelerometer();
void SetupBMP();
void sendFloat(float value,char type);
//void toChar(uint32_t content,uint8_t * address,uint8_t length);
uint8_t addr = 0x18;			//i2c address for IIS328DQ
uint8_t IIS328DQ_ADDR=0;
uint8_t BMP180_ADDR = 0xEE;		//i2c address for BMP180
float a_x = 0,a_y = 0,a_z = 0;	//Accelerometer data variable
uint64_t timer1=0,timer2=0;		//variable to store time related parameters
int16_t X = 0, Y = 0, Z = 0;	//variables to store raw accelerometer data
uint8_t a_value[6]={0,0,0,0,0,0};	//Buffer to extract values from Accelerometer
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM3){
		ReadAccelerometer();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	IIS328DQ_ADDR= addr<<1;		// create space to append RW bit with address
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  	timer2=(HAL_GetTick()/1000U);	//read seconds in timer2 variable
  	timer1=HAL_GetTick();		//read milliseconds in timer2 variable
    SetupBMP();							//Configure BMP180 sensor
    SetupAccelerometer();				//Configure IIS328DQ Accelerometer sensor
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//	  if(HAL_GetTick()-timer1>=100){//check for 100 milliseconds time laps
//		  ReadAccelerometer();				//Read the data from accelerometer and display on UART
//		  timer1=HAL_GetTick();		//update timer1 variable
//	      }
	  if((HAL_GetTick()/1000U)-timer2>=10){// check for 10 seconds time laps
		  ReadBMP180();							//Read the data from BMP180 sensor and display on UART
	      timer2=(HAL_GetTick()/1000U);		//update timer2 variable
	      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301341;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 48000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*Variable to read Acceleration data*/
void ReadAccelerometer(){
	ReadRegister(IIS328DQ_ADDR,ACC_RAW_VALUE_ADDR | RECURCISE_READ_EN,a_value,6);//read 6 byte acceleration raw data
	X = (a_value[1]<<8) |a_value[0];//store the result in signed integer variable for X axis
	Y = (a_value[3]<<8) |a_value[2];//store the result in signed integer variable for Y axis
	Z = (a_value[5]<<8) |a_value[4];//store the result in signed integer variable for Z axis
	a_x =(( X * g )/32768.0);		//store X axis acceleration in float
	a_y =(( Y * g )/32768.0);		//store Y axis acceleration in float
	a_z =(( Z * g )/32768.0);		//store Z axis acceleration in float
	sendFloat(a_x,'X');				//print result on UART
	sendFloat(a_y,'Y');				//print result on UART
	sendFloat(a_z,'Z');				//print result on UART
}
/*Function to read calibration parameters */
void ReadCalPara(){
    AC1 = ReadInt(0xAA);	//Read value for AC1 register
    AC2 = ReadInt(0xAC);	//Read value for AC2 register
    AC3 = ReadInt(0xAE);	//Read value for AC3 register
    AC4 = ReadUInt(0xB0);	//Read value for AC4 register
    AC5 = ReadUInt(0xB2);	//Read value for AC5 register
    AC6 = ReadUInt(0xB4);	//Read value for AC6 register
    B1 = ReadInt(0xB6);		//Read value for B1 register
    B2 = ReadInt(0xB8);		//Read value for B2 register
    MB = ReadInt(0xBA);		//Read value for MB register
    MC= ReadInt(0xBC);		//Read value for MC register
    MD = ReadInt(0xBE);		//Read value for MD register
}
/*Read signed integer from I2C bus*/
int16_t ReadInt(char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	int16_t result=0;								// variable to be return
	ReadRegister(BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}
/*Read unsigned integer from I2C bus*/
uint16_t ReadUInt(char address){
	uint8_t temp[2];									// Temporary variable to store raw value
	uint16_t result=0;								// variable to be return
	ReadRegister(BMP180_ADDR,address,temp,2);		// Read raw value from I2C bus
	result=((temp[0]<<8)|temp[1]);					// convert the raw value to integer
	return result;									// Return integer
}
/*Read values from BMP180 sensor*/
void ReadBMP180(){
	int32_t X1,X2,B5,ut,p,B6,X3,B3;					// Temporary signed variable to calculate pressure and temperature
	uint32_t B4,B7;									// Temporary unsigned variable to calculate pressure and temperature
	char sampling_mode = LOW_POWER;		// Working mode for BMP180 sensor
	float temperature=0.0;							// variable to carry temperature data
	int32_t UT,UP;									// variable to carry integer value of Temperature and pressure
	UT=ReadSensor(READ_TEMP_CMD,sampling_mode,2);		// Read Temperature and given mode
	if(UT!=0){										// check for valid reading
		X1=(UT-AC6) * AC5>>15;						// calculation to obtain
		X2=(MC<<11)/(X1+MD);						// Temperature from raw
		B5=X1+X2;									// values for more details
		ut=(B5+8)>>4;								// refer section 3.5 in BMP180
		temperature = ut *0.1;						// sensor datasheet.
		sendFloat(temperature,'T');					// Print temperature on UART
		UP=ReadSensor(READ_PRESSURE_CMD,sampling_mode,3);			// read raw values of pressure
		if(UP!=0){									// check for valid reading
			UP = (UP>>(8-sampling_mode));
			B6=B5-4000;								// calculations to extract pressure
			X1=(B2*((B6*B6)>>12))>>11;				// reading from BMP180 sensor
			X2=(AC2*B6)>>11;						// all the calculations shown
			X3=X1+X2;								// in the datasheet are carry
			B3=((((AC1*4)+X3)<<sampling_mode)+2)/4;// forward in this step
			X1=(AC3*B6)>>13;						// for more details on this
			X2=(B1*((B6*B6)>>12))>>16;				// please refer section 3.5 in
			X3 = (X1 + X2 + 2) >> 2;				// BMP180 sensor datasheet.
			B4=(uint32_t)AC4*(uint32_t)(X3+32768)>>15;					//
			B7=(uint32_t)(UP-B3)*(50000>>sampling_mode);		//
			if (B7 < 0x80000000)					//
				p = (B7 * 2) / B4;					//
			else									//
				p = (B7 / B4) * 2;					//
			X1 = (p >> 8) * (p >> 8);				//
			X1 = (X1 * 3038) >> 16;					//
			X2 = (-7357 * p) >> 16;					//
			p = p + ((X1 + X2 + 3791) >> 4);		//
			send(p,PRESSURE);						// Print PRESSURE on UART
		}
	}
}
/*Function to Read the raw values from the sensor*/
int32_t ReadSensor(char data,char mode,uint8_t length){
	uint8_t response[3],x=0;							//Local variables declaration
	data = data | (mode<<6);						//append mode in control register
	WriteRegister(BMP180_ADDR,BMP180_CNTL_REG_ADDR,data);//start pressure sensor in given mode
	do{
		x++;										//increment x to each milliseconds
		HAL_Delay(1);									//delay for 1 milliseconds
		ReadRegister(BMP180_ADDR,BMP180_CNTL_REG_ADDR,&response[0],1);//read busy status of sensor
		if(x>=30)									//if no response received with
			return 0;								// 30 milliseconds then return 0.
	}while((response[0]&0x20)!=0x00);				// wait until sensor is busy
	ReadRegister(BMP180_ADDR,BMP180_RAW_VALUE_ADDR,response,length);//read the raw data from sensor
	int32_t result= 0 ;
	if(length==2)
		result = ((response[0]<<8) | response[1]);	// convert bytes into signed integer
	else if(length==3)
		result = ((response[0]<<16) | (response[1]<<8) | response[2]);//convert bytes into signed integer
	return result;									//return the result
}
/*function to setup BMP180 sensor*/
void SetupBMP(){
	uint8_t prxbuf[1];									// local variable
	ReadRegister(BMP180_ADDR,0xD0,prxbuf,1);		// read BMP180 sensor ID
//	Debug.printf("BMP ID : %02x \r\n",prxbuf[0]);	// display ID
	ReadCalPara();									// read calibration data from the sensor
}
/*function to read RAW values from I2C bus*/
void ReadRegister(uint8_t DevAddress,uint8_t Address,uint8_t * Buffer,uint8_t length){
	HAL_I2C_Master_Transmit(&hi2c1,DevAddress&0xFE,&Address,1,10);// send device address and register address
	HAL_I2C_Master_Receive(&hi2c1,DevAddress|0x01,Buffer,length,10);// read data from sensor
}

/*function to write RAW values on I2C bus*/
void WriteRegister(uint8_t DevAddress,char address,char content){
	uint8_t data[2]={address,content};					// local variable to store addres and data to be written
	HAL_I2C_Master_Transmit(&hi2c1,DevAddress&0xFE,data,2,10);//	write the address and data to the I2C bus
}
/*function to setup IIS328DQ accelerometer*/
void SetupAccelerometer(){
	uint8_t rxbuf[1];						// local variable declaration
	ReadRegister(IIS328DQ_ADDR,ACC_READ_ID_REG_ADDR,rxbuf,1);		// read id from IIS328DQ

	if(rxbuf[0]==0x32){
		WriteRegister(IIS328DQ_ADDR,ACC_CONFIG_REG_ADDR,ACC_EN_DEVICE);	//turn on device and set sampling rate
		if(g==2){
			WriteRegister(IIS328DQ_ADDR,ACC_G_CONFIG_REG_ADDR,0x00);	//update G value in IIS328DQ to 2
		}
		else if(g==4){
			WriteRegister(IIS328DQ_ADDR,ACC_G_CONFIG_REG_ADDR,0x10);	//update G value in IIS328DQ to 4
		}
		else if(g==8){
			WriteRegister(IIS328DQ_ADDR,ACC_G_CONFIG_REG_ADDR,0x30);	//update G value in IIS328DQ to 8
		}
	}
}
/*Function to send the integer on uart*/
void send(int32_t data,uint8_t type){
	char buffer[30],len=0;									//Local variable declaration
	if(type==PRESSURE){
		len=sprintf(buffer,"Pressure : %d Pa\r\n",data);	// convert the integer to character array
	}
	HAL_UART_Transmit(&huart3,buffer,len,100);				//send character array to UART
}

/*Function to send the float on uart*/
void sendFloat(float value,char type){
	char buffer[30];							//local variable declaration
	uint8_t length=0;							//local variable declaration
	char *Sign = (value < 0) ? "-" : "";		//extract sign of float
	float Val = (value < 0) ? -value : value;	//check for negative float
	int Int1 = Val;								//convert float to integer
	float Frac = Val - Int1;					//Extract fractional of float
	int Int2 = (uint16_t)(Frac * 100);			// convert fractional part to integer
	if(type!='T'){
		uint8_t temp[4]={type,' ',':',' '};		//Send parameter name
		HAL_UART_Transmit(&huart3,temp,4,1000);	//on UART
	}
	else{
		uint8_t temp[]="Temperature : ";					//Send parameter name
		HAL_UART_Transmit(&huart3,temp,sizeof(temp),1000);	//on UART
	}
	if( type=='T')
		length=sprintf (buffer, "%s%d.%02d degC\t",Sign, Int1, Int2);// send temperature data on UART
	else if(type=='Z')
		length=sprintf (buffer, "%s%d.%02d\r\n",Sign, Int1, Int2);	// send Z axis data of accelerometer on UART
	else
		length=sprintf (buffer, "%s%d.%02d\t",Sign, Int1, Int2);	//send X and Y axis data of accelerometer on UART
	HAL_UART_Transmit(&huart3,buffer,length,1000);
}
/* USER CODE END 4 */

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
