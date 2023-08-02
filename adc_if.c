/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc_if.c
  * @author  MCD Application Team
  * @brief   Read status related to the chip (battery level, VREF, chip temperature)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "adc_if.h"
#include "sys_app.h"

/* USER CODE BEGIN Includes */
#include "tim.h"

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/**
  * @brief ADC handle
  */
extern ADC_HandleTypeDef hadc;
/* USER CODE BEGIN EV */


/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define TEMPSENSOR_TYP_CAL1_V          (( int32_t)  760)        /*!< Internal temperature sensor, parameter V30 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define TEMPSENSOR_TYP_AVGSLOPE        (( int32_t) 2500)        /*!< Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */

/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE	200
#define ADC_CHANNEL_COUNT 6
#define NR_ITER 4
#define VDDA_APPLI                       ((uint32_t)3300)
#define BURDEN_RES 43 /* value of resistor which acts as safety between the transformer's terminals (Ohms) */
#define TURNS_RATIO 					 ((uint16_t) 800) /* turns ratio for specific current transformer sensor == ratio of (primary turns/secondary turns) */

/* Enum for ON and OFF state
 */
uint16_t max_value = 0;
//enum state_of_device {OFF, ON}; //Moved to "adc_if.h"
enum state_of_device state = OFF;
#define Method1_ADC_OFFSET 2110
#define ADC_OFFSET 32 /* shifting 2080 (0V) to 2048 middle of range ((2*12)/2) so that we can also measure the negative values */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t	ADC_Buffer[ADC_BUFFER_SIZE][ADC_CHANNEL_COUNT];
/* Variables for ADC conversion data computation to physical values */
uint16_t  aADCxConvertedData_Voltage_mVolt[ADC_BUFFER_SIZE];  /* Value of voltage calculated from ADC conversion data (unit: mV) (array of data) */
uint32_t   Secondary_current[ADC_BUFFER_SIZE];  /* Value of current on the secondary side of the current transformer */
uint32_t  squared_current[ADC_BUFFER_SIZE]; /* Squared value of calculated current */
uint32_t  ss = 0; /* sum of squares of current values */
uint32_t  ms = 0; /* mean of sum of squares of current values */
uint32_t  RMS_i = 0; /*rms value in ith iteration */
uint32_t  RMS_i1 = 0; /*rms value in the ith + 1 iteration */
uint32_t  Primary_current[ADC_BUFFER_SIZE];  /* Value of current on the primary side of the current transformer (side measured) */


//uint32_t	ADC2_RMS = 0;
//uint32_t	ADC3_RMS = 0;
//uint32_t	ADC7_RMS = 0;
//uint32_t	ADC8_RMS = 0;
float	ADC2_RMS = 0;
float	ADC3_RMS = 0;
float	ADC7_RMS = 0;
float	ADC8_RMS = 0;
uint32_t	ADC_Temp = 0;
uint32_t	ADC_Vref = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief This function reads the ADC channel
  * @param channel channel number to read
  * @return adc measured level value
  */
static uint32_t ADC_ReadChannels(uint32_t channel);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void SYS_InitMeasurement(void)
{
  /* USER CODE BEGIN SYS_InitMeasurement_1 */
//	MX_DMA_Init();
	MX_ADC_Init();
	MX_TIM2_Init();
  /* USER CODE END SYS_InitMeasurement_1 */
  hadc.Instance = ADC;
  /* USER CODE BEGIN SYS_InitMeasurement_2 */

	if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc))
		Error_Handler();

	if (HAL_OK != HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADC_Buffer, ADC_BUFFER_SIZE*ADC_CHANNEL_COUNT))
		Error_Handler();
	if (HAL_OK != HAL_TIM_Base_Start(&htim2))
		Error_Handler();

  /* USER CODE END SYS_InitMeasurement_2 */
}

void SYS_DeInitMeasurement(void)
{
  /* USER CODE BEGIN SYS_DeInitMeasurement_1 */

  /* USER CODE END SYS_DeInitMeasurement_1 */
}

int16_t SYS_GetTemperatureLevel(void)
{
  /* USER CODE BEGIN SYS_GetTemperatureLevel_1 */

  /* USER CODE END SYS_GetTemperatureLevel_1 */
  int16_t temperatureDegreeC = 0;
  uint32_t measuredLevel = 0;
  uint16_t batteryLevelmV = SYS_GetBatteryLevel();

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_TEMPSENSOR);

  /* convert ADC level to temperature */
  /* check whether device has temperature sensor calibrated in production */
  if (((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) != 0)
  {
    /* Device with temperature sensor calibrated in production:
       use device optimized parameters */
    temperatureDegreeC = __LL_ADC_CALC_TEMPERATURE(batteryLevelmV,
                                                   measuredLevel,
                                                   LL_ADC_RESOLUTION_12B);
  }
  else
  {
    /* Device with temperature sensor not calibrated in production:
       use generic parameters */
    temperatureDegreeC = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(TEMPSENSOR_TYP_AVGSLOPE,
                                                              TEMPSENSOR_TYP_CAL1_V,
                                                              TEMPSENSOR_CAL1_TEMP,
                                                              batteryLevelmV,
                                                              measuredLevel,
                                                              LL_ADC_RESOLUTION_12B);
  }

  APP_LOG(TS_ON, VLEVEL_L, "temp= %d\n\r", temperatureDegreeC);

  /* from int16 to q8.7*/
  temperatureDegreeC <<= 8;

  return (int16_t) temperatureDegreeC;
  /* USER CODE BEGIN SYS_GetTemperatureLevel_2 */

  /* USER CODE END SYS_GetTemperatureLevel_2 */
}

uint16_t SYS_GetBatteryLevel(void)
{
  /* USER CODE BEGIN SYS_GetBatteryLevel_1 */

  /* USER CODE END SYS_GetBatteryLevel_1 */
  uint16_t batteryLevelmV = 0;
  uint32_t measuredLevel = 0;

  measuredLevel = ADC_ReadChannels(ADC_CHANNEL_VREFINT);

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    if ((uint32_t)*VREFINT_CAL_ADDR != (uint32_t)0xFFFFU)
    {
      /* Device with Reference voltage calibrated in production:
         use device optimized parameters */
      batteryLevelmV = __LL_ADC_CALC_VREFANALOG_VOLTAGE(measuredLevel,
                                                        ADC_RESOLUTION_12B);
    }
    else
    {
      /* Device with Reference voltage not calibrated in production:
         use generic parameters */
      batteryLevelmV = (VREFINT_CAL_VREF * 1510) / measuredLevel;
    }
  }

  return batteryLevelmV;
  /* USER CODE BEGIN SYS_GetBatteryLevel_2 */

  /* USER CODE END SYS_GetBatteryLevel_2 */
}

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//METHOD1: Very simple RMS calculation for 4 ADC channels
//*******************************************************//


	uint32_t A[4] = {0};
	int32_t temp;


	//__NOP();
	for (uint8_t i=0;i<ADC_BUFFER_SIZE;i++) {
		temp = ADC_Buffer[i][0]-Method1_ADC_OFFSET;
		A[0] += (uint32_t)(temp*temp);				// A[0] = A[0]+temp^2

		temp = ADC_Buffer[i][1]-Method1_ADC_OFFSET;
		A[1] += (uint32_t)(temp*temp);

		temp = ADC_Buffer[i][2]-Method1_ADC_OFFSET;
		A[2] += (uint32_t)(temp*temp);

		temp = ADC_Buffer[i][3]-Method1_ADC_OFFSET;
		A[3] += (uint32_t)(temp*temp);
	}


	//	ADC2_RMS = A[0]/ADC_BUFFER_SIZE;
	//	ADC3_RMS = A[1]/ADC_BUFFER_SIZE;
	//	ADC7_RMS = A[2]/ADC_BUFFER_SIZE;
	//	ADC8_RMS = A[3]/ADC_BUFFER_SIZE;
	//	sqrt(X) = (X + Y) / (2*sqrt(Y))
	//	Y is selected: 5
	ADC2_RMS = ((float)(A[0]/ADC_BUFFER_SIZE) + 25.0f) / 10.0f;
	ADC3_RMS = ((float)(A[1]/ADC_BUFFER_SIZE) + 25.0f) / 10.0f;
	ADC7_RMS = ((float)(A[2]/ADC_BUFFER_SIZE) + 25.0f) / 10.0f;
	ADC8_RMS = ((float)(A[3]/ADC_BUFFER_SIZE) + 25.0f) / 10.0f;


	ADC_Temp = ADC_Buffer[0][4];
	ADC_Vref = ADC_Buffer[0][5];


//METHOD2: Provided by Alkinoos
//******************************//

	uint32_t tmp_index = 0;
	uint8_t iter = 0;
	ss = 0;

	/* Computation of ADC conversions raw data to physical values               */
	/* using LL ADC driver helper macro.                                        */
	/* Management of the whole buffer */
	for (tmp_index = 0; tmp_index < (ADC_BUFFER_SIZE); tmp_index++)
	{
	aADCxConvertedData_Voltage_mVolt[tmp_index] = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI, (ADC_Buffer[tmp_index][2] - ADC_OFFSET));
	if (aADCxConvertedData_Voltage_mVolt[tmp_index] >= (VDDA_APPLI/2)) {
		Secondary_current[tmp_index] = ((aADCxConvertedData_Voltage_mVolt[tmp_index]*1000 - (VDDA_APPLI/2)*1000) / BURDEN_RES); // in uA
	}
	else {
		Secondary_current[tmp_index] = (((VDDA_APPLI/2)*1000 - aADCxConvertedData_Voltage_mVolt[tmp_index]*1000) / BURDEN_RES); // in uA
	}

	squared_current[tmp_index] = (uint32_t)(Secondary_current[tmp_index]*Secondary_current[tmp_index]);
	ss += squared_current[tmp_index];

	//Primary_current[tmp_index] = TURNS_RATIO * Secondary_current;

	}

	ms = ss / ADC_BUFFER_SIZE; // mean of sum of squares of current values
	RMS_i = ms;
	for (iter = 0; iter < 18; iter++) {
	  RMS_i1 = 0.5 * (RMS_i + (ms/RMS_i));
	  RMS_i = RMS_i1;
	}

	/* Functionality for detecting ON/OFF state */

	max_value = aADCxConvertedData_Voltage_mVolt[0];

	for (tmp_index = 1; tmp_index < (ADC_BUFFER_SIZE); tmp_index++)
	{
	if (aADCxConvertedData_Voltage_mVolt[tmp_index] > max_value) {
		max_value = aADCxConvertedData_Voltage_mVolt[tmp_index];
	}
	}

	if (max_value > 1700) {
	  state = ON;
	}
	else {
	  state = OFF;
	}

}

/* USER CODE END PrFD */

static uint32_t ADC_ReadChannels(uint32_t channel)
{
  /* USER CODE BEGIN ADC_ReadChannels_1 */

#if 0
  /* USER CODE END ADC_ReadChannels_1 */
  uint32_t ADCxConvertedValues = 0;
  ADC_ChannelConfTypeDef sConfig = {0};

  MX_ADC_Init();

  /* Start Calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_Start(&hadc) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  /** Wait for end of conversion */
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

  /** Wait for end of conversion */
  HAL_ADC_Stop(&hadc) ;   /* it calls also ADC_Disable() */

  ADCxConvertedValues = HAL_ADC_GetValue(&hadc);

  HAL_ADC_DeInit(&hadc);

  return ADCxConvertedValues;
  /* USER CODE BEGIN ADC_ReadChannels_2 */
#endif

  uint32_t ADCxConvertedValues = 0;

  switch(channel) {
  case ADC_CHANNEL_2:				ADCxConvertedValues = ADC2_RMS;		break;
  case ADC_CHANNEL_3:				ADCxConvertedValues = ADC3_RMS;		break;
  case ADC_CHANNEL_7:				ADCxConvertedValues = ADC7_RMS;		break;
  case ADC_CHANNEL_8:				ADCxConvertedValues = ADC8_RMS;		break;
  case ADC_CHANNEL_TEMPSENSOR:		ADCxConvertedValues = ADC_Temp;		break;
  case ADC_CHANNEL_VREFINT:			ADCxConvertedValues = ADC_Vref;		break;
  default:
	  break;
  }

  return ADCxConvertedValues;

  /* USER CODE END ADC_ReadChannels_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
