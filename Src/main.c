/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 11/06/2014 17:37:14
  * Description        : Main program body
  */

#define ARM_MATH_CM4
#define FILTER_ORDER 8
#define MAX_AMPLITUDE 3
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "../Drivers/CMSIS/Include/arm_math.h"
#include <string.h>
#include <stdio.h>
#include "arm_const_structs.h"
#include "hannCoeffs.h"

typedef struct
{
	uint32_t startBin,binCount;
}channelSelect;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN 0 */
float32_t fftData[2][512*2];
float32_t fftOut[512];
float32_t adcPeak[2];
uint16_t valIndex;
uint8_t dataRow,dataFilled;

float32_t dcOffset,gain;
const uint32_t pwmPeriod = 500-1;
const float32_t freqPerBin = (84000000/(4200))/512;
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);

const float32_t IIRa[FILTER_ORDER + 1]={1,2.38697983,3.369601161,2.964098919,1.797474309,0.738048925,0.200473711,0.032394288,0.002381868};
const float32_t IIRb[FILTER_ORDER + 1]={0.048794738,0.390357907,1.366252673,2.732505346,3.415631682,2.732505346,1.366252673,0.390357907,0.048794738};

inline uint8_t incIndex(uint8_t index)
{
	index % 8 == 0 ? index = 0 : index++;
	
	return index;
}
float32_t butterFilter(float32_t adcIn)
{
	static float32_t x[FILTER_ORDER + 1]={0},y[FILTER_ORDER + 1]={0};
	static uint8_t index=0;
	uint8_t tmpIndex2;

	for(uint8_t i=8; i>0;i--)
	{
		x[i] = x[i-1];
		y[i] = y[i-1];
	}
	x[0] = adcIn;
	y[0] = IIRb[0] * x[0];
	for(uint8_t i=1;i<=8;i++)
	{
		y[0] += IIRb[i] * x[i] - IIRa[i] * y[i];
	}
	return y[0];
		
	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_ADC_Start_IT(&hadc1);
}

float32_t absF(float32_t x)
{
	if (x < 0)
		return x*-1;
	else
		return x;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Read in ADC data and store it in the buffer as a positive value
	float32_t absIn;
	fftData[dataRow][valIndex] = gain*3*((float32_t)HAL_ADC_GetValue(hadc)/(float32_t)4095) - dcOffset;
	absIn = absF(fftData[dataRow][valIndex]);
	if(absIn > adcPeak[dataRow])
	{
		adcPeak[dataRow] = absIn;
	}
	fftData[dataRow][valIndex+1] = 0;
	valIndex += 2;
	
	if(valIndex > 1024)
	{
		valIndex = 0;
		dataRow ^= 0x01;
		adcPeak[dataRow] = 0;
		dataFilled = 1;
	}
	
}


float32_t average(float32_t* array, float32_t size)
{
	float32_t tempSum=0;
	
	for(uint32_t i=0;i<(size*2);i+=2)
	{
		tempSum += array[i];
	}
		
	tempSum /= size;
	
	return tempSum;
}

void calculateChannel(channelSelect* inChannel,float32_t startFreq,float32_t endFreq)
{
	if(endFreq <= startFreq)
			return;
	
	for(uint32_t i=0;i<512;i++)
	{
		if(i*freqPerBin >= startFreq)
		{
			inChannel->startBin = i > 0 ? i-1 : 1;
			break;
		}
	}
	
	for(uint32_t i=0;i<512;i++)
	{
		if(i*freqPerBin >=endFreq)
		{
			inChannel->binCount = i-inChannel->startBin + 1;
			break;
		}
	}
	
}
void setPulse(TIM_TypeDef * timer, uint32_t channel, uint32_t pulse)
{
	switch(channel)
	{
		case 0:
			timer->CCR1 = pulse;
			break;
		case 1:
			timer->CCR2 = pulse;
			break;
		case 2:
			timer->CCR3 = pulse;
			break;
		case 3:
			timer->CCR4 = pulse;
			break;
		default:
			break;
	}
	
}

float32_t arrayMax(float32_t* inArray, uint32_t nVals)
{
	float32_t max =0;

	for(uint32_t i=0;i<nVals;i++)
	{
		if( inArray[i] > max)
			max = inArray[i];
	}
	
	return max;
}
//Eliminates signals below a certain threshold
void threshCut(float32_t* inVals,uint32_t nVals, float32_t thresh)
{
	for(uint32_t i=0;i<nVals;i++)
	{
		inVals[i] = inVals[i] >= thresh ? inVals[i] : 0;
	}
	
	
}
int main(void)
{

  /* USER CODE BEGIN 1 */
	char outString[256];
	char inChar = 0;
	channelSelect channels[6];
	float32_t channelFreqs[6][2] = {{0,70},
																	{100,200},
																	{250,400},
																	{450,1000},
																	{1200,2300},
																	{2500,6000}};
	uint32_t prevPulse[6] = {0,0,0,0,0,0};
	uint32_t newPulse[6];
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  gain = 1;
  MX_GPIO_Init();
  MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  
	for(uint32_t i=0;i<6;i++)
	{
		calculateChannel(&channels[i],channelFreqs[i][0],channelFreqs[i][1]);
	}

  /* USER CODE BEGIN 2 */
  TIM2->SR = 0;
	TIM2->CNT = 0;
	valIndex = 0;
	
	HAL_TIM_Base_Start_IT(&htim2);
	while(!dataFilled);
	HAL_TIM_Base_Stop_IT(&htim2);
	dataFilled = 0;
	valIndex = 0;
	dataRow = 0;
	dcOffset = average(fftData[0],512);
	
	for(uint32_t i=0;i<6;i++)
	{
		setPulse(i<3 ? TIM3 : TIM4,i % 3,pwmPeriod);
		HAL_Delay(500);
		setPulse(i<3 ? TIM3 : TIM4,i % 3,0);
	}
	
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
	
  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
		HAL_UART_Receive(&huart2,(uint8_t*)&inChar,1,0);
		if(inChar)
		{
			if(inChar == 'd')
			{
				for(uint32_t i=0;i<512;i++)
				{
					sprintf(outString,"%f,",fftOut[i]);
					HAL_UART_Transmit(&huart2,(uint8_t *)outString,strlen(outString)+1,1000);
				}		
			
			}
			inChar = 0;
		}
		if(dataFilled)
		{
			//Adjust gain (AGC)
			if((adcPeak[dataRow^0x01] < MAX_AMPLITUDE*0.95) && (adcPeak[dataRow^0x01] > 0.5))
			{
				gain += 0.01;
			}
			else if(adcPeak[dataRow^0x01] > MAX_AMPLITUDE*0.95)
			{
				gain -= 0.01;
			}
			
			//Calculate FFT
			arm_cfft_f32(&arm_cfft_sR_f32_len512,fftData[dataRow^0x01],0,1);
			arm_cmplx_mag_f32(fftData[dataRow^0x01],fftOut,512);
			for(uint32_t i=1;i<256;i++)
			{
				fftOut[i]/=256;
			}
			
			threshCut(fftOut,256,0.06);
			
			for(uint32_t i=0;i<6;i++)
			{
				newPulse[i] = arrayMax(&fftOut[channels[i].startBin],channels[i].binCount)*pwmPeriod;
				if((newPulse[i] < prevPulse[i]) && i>=2 && i<=5){
					newPulse[i] = newPulse[i] + ((prevPulse[i]- newPulse[i]))* (i==2 ? 0.3 :0.75);
				}

				setPulse(i<3 ? TIM3 : TIM4,i % 3,newPulse[i]);
				prevPulse[i] = newPulse[i];
			}
			dataFilled = 0;	
		}
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
static void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_MultiModeTypeDef multimode;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 840-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = pwmPeriod;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = pwmPeriod;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim4);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	 /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
