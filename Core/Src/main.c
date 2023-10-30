/* USER CODE BEGIN Header */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"lcd1.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define fact_val 3
#define fact_val_addr *((uint16_t*)0x1FFF75AA)
#define Reso 4095
#define DMA_Time_Out 100
#define NumOfADCChan 3
#define EachChanSample 160

//Trip value
#define OverVoltagetripvalue 259.0455 // it is equal to 260
#define UnderVoltagetripvalue 165     // it is equal to 160
#define normalVoltagevalue 162        // it is equal to 162
#define normalDiffvalue 1
#define OverCurrenttripvalue 15

//Relay
#define RelayOFFValue 0
#define RelayONValue 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
char text[100];
uint16_t Id =0;
uint32_t Start_Time=0;
uint16_t Diff_Time=0;
typedef enum
{
	Fun_Ok, //Represents FADC status that it is OK
	Fun_Error, //Represents FADC Error
	Fun_Timeout //Represents that FADC is timed out
}Fun_Status;
typedef enum
{
    ADC_DMA_Init,
    ADC_DMA_Ready,
    ADC_DMA_Started,
    ADC_DMA_Completed,
	ADC_DMA_ERROR
}ADC_DMA_Status;
typedef enum
{
	DeviceInit,
	safesupply,
	OverVoltage,
	UnderVoltage,
	OverCurrent,
	RelayOn,
	RelayOFF
}SupplyStatus;
//
typedef struct{
    uint8_t diffV ;
    uint8_t ctmxLt ;
    SupplyStatus OutPutStatus;
    SupplyStatus RelayStatus;
    uint16_t NrmVolt ;
    uint16_t OvVolt;
    uint16_t UnVlt ;

}Abnormalpara;
typedef struct{
    float Vraw1;
    float Vraw2;
    float Vraw3;
    float Vref;
    float K;
    float DCoffset;
    float Vrms1;
    float Vrms2;
    float Vrms3;
    ADC_DMA_Status StatusDMA;
    ADC_HandleTypeDef* HADC;
    uint16_t ADC_DMA_Sample[NumOfADCChan*EachChanSample];
}ADCPar;
ADCPar ADCVar;
Abnormalpara AbnormaVar;
//Limitsfloat TripIrmsSum = 0;
const uint8_t TripIrmsSample = 4;
double TripVrmsSum = 0;
const uint8_t TripVrmsSample = 2;
const uint8_t NormalSample = 30;
uint32_t NormalLoopTime = 0;
const uint16_t NormalTimeOut = 7000;
const uint8_t NormalStatusCount = 9;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Vrefint_init(ADC_HandleTypeDef* HADC)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  HADC->Instance = ADC1;
  HADC->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  HADC->Init.Resolution = ADC_RESOLUTION_12B;
  HADC->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  HADC->Init.ScanConvMode = ADC_SCAN_DISABLE;
  HADC->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HADC->Init.LowPowerAutoWait = DISABLE;
  HADC->Init.LowPowerAutoPowerOff = DISABLE;
  HADC->Init.ContinuousConvMode = DISABLE;
  HADC->Init.NbrOfConversion = 1;
  HADC->Init.DiscontinuousConvMode = DISABLE;
  HADC->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  HADC->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  HADC->Init.DMAContinuousRequests = DISABLE;
  HADC->Init.Overrun = ADC_OVR_DATA_PRESERVED;
  HADC->Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  HADC->Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  HADC->Init.OversamplingMode = DISABLE;
  HADC->Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {

  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {

  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

Fun_Status GetVref(ADCPar * ADCValues)
{
	  Vrefint_init(ADCValues->HADC);
	  if(HAL_ADC_Start(ADCValues->HADC) != HAL_OK)
	  {
		return Fun_Error;
	  }
	  if(HAL_ADC_PollForConversion(ADCValues->HADC, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return Fun_Error;
	  }
	  ADCValues->Vref = HAL_ADC_GetValue(ADCValues->HADC);
	  if(HAL_ADC_Stop(ADCValues->HADC) != HAL_OK)
	  {
		  return Fun_Error;
	  }
	  ADCValues->Vref =(fact_val * fact_val_addr)/ADCValues->Vref;
	  ADCValues->K=ADCValues->Vref/Reso;
	  //3 channel init
	  MX_ADC1_Init();
	  HAL_Delay(10);
	  HAL_ADCEx_Calibration_Start(&hadc1);
	  HAL_Delay(10);
	  return Fun_Ok;
}

Fun_Status FADC_Get_VRMS(ADCPar* FADC_p){


    if((FADC_p->StatusDMA==ADC_DMA_Ready)||(FADC_p->StatusDMA==ADC_DMA_ERROR))
    {
        //
        Start_Time=HAL_GetTick();
        HAL_ADC_Start_DMA(FADC_p->HADC, (uint32_t*)&FADC_p->ADC_DMA_Sample,NumOfADCChan*EachChanSample);
        FADC_p->StatusDMA=ADC_DMA_Started;
    }

    if(FADC_p->StatusDMA==ADC_DMA_Completed)
    {
        uint16_t id=0;
        long double Vrmssqr1=0,Vrmssqr2=0,Vrmssqr3=0;
         for(id=0;id<EachChanSample;id++)
         {
             FADC_p->Vrms1 = FADC_p->ADC_DMA_Sample[id*3];
             FADC_p->Vrms2 = FADC_p->ADC_DMA_Sample[(id*3)+1];
             FADC_p->Vrms3 = FADC_p->ADC_DMA_Sample[(id*3)+2];

             FADC_p->Vrms1  =  (FADC_p->Vrms1)-2047.5;
             FADC_p->Vrms2  =  (FADC_p->Vrms2)-2047.5;
             FADC_p->Vrms3  =  (FADC_p->Vrms3)-2047.5;

             Vrmssqr1+=FADC_p->Vrms1*FADC_p->Vrms1 ;
             Vrmssqr2+=FADC_p->Vrms2*FADC_p->Vrms2 ;
             Vrmssqr3+=FADC_p->Vrms3*FADC_p->Vrms3 ;
         }

         //each channel mean
         Vrmssqr1=Vrmssqr1/id;
         Vrmssqr2=Vrmssqr2/id;
         Vrmssqr3=Vrmssqr3/id;

         FADC_p->Vrms1 = sqrt(Vrmssqr1);
         FADC_p->Vrms2 = sqrt(Vrmssqr2);
         FADC_p->Vrms3 = sqrt(Vrmssqr3);

         FADC_p->Vraw1 = FADC_p->Vrms1;
         FADC_p->Vraw2 = FADC_p->Vrms2;
         FADC_p->Vraw3 = FADC_p->Vrms3;


         FADC_p->Vrms1 *= ((0.3543)*(FADC_p->Vref/3.3));
         FADC_p->Vrms2 *= ((0.4966)*(FADC_p->Vref/3.3));
         FADC_p->Vrms3 *= ((0.4966)*(FADC_p->Vref/3.3));
         FADC_p->StatusDMA=ADC_DMA_Ready;

         //check condition

    }
    return Fun_Ok;
}

Fun_Status CheckStatus(ADCPar * FADC_p,Abnormalpara * AbnormalValues)
{
	//To Trip the Relay when supply is abnormal
 if ((FADC_p->Vrms1 > AbnormalValues->OvVolt) && ((AbnormalValues->RelayStatus == RelayOn) || (AbnormalValues->RelayStatus == DeviceInit) || (AbnormalValues->OutPutStatus != OverVoltage))) // check OV
    {

    }
    else if ((FADC_p->Vrms1 < AbnormalValues->UnVlt) && ((AbnormalValues->RelayStatus == RelayOn) || (AbnormalValues->RelayStatus == DeviceInit) || (AbnormalValues->OutPutStatus != UnderVoltage))) // check UV
    {

    }

    // If relay is not Trip
    if (((AbnormalValues->RelayStatus == RelayOFF) || (AbnormalValues->RelayStatus == DeviceInit)) && (FADC_p->Vrms1 < (AbnormalValues->OvVolt - AbnormalValues->diffV)) && (FADC_p->Vrms1 > (AbnormalValues->NrmVolt + AbnormalValues->diffV))) // NV
    {

    }
    return 0;
}


void CheckOVUVOCNV(ADCPar * FADC_p,Abnormalpara * AbnormalValues, const char Operation)
{
//Setting Variables
    uint8_t TempSample = 0, TempLyt = 0, NormalStatusLoopCount = 0;
    uint8_t Normalled = 0;
    float TripVrmsSum = 0;
    float TripIrmsSum = 0;
    ++TempSample;
    TripVrmsSum += FADC_p->Vrms1;
//    TripIrmsSum += RMSS.Irms;

    switch (Operation)
    {
    case 'O':
    case 'U':
        TempLyt = TripVrmsSample;
        break;
    case 'C':
        TempLyt = TripIrmsSample;
        break;
    case 'N':
        TempLyt = NormalSample;
        NormalLoopTime = HAL_GetTick();
        break;
    }

    while (TempLyt >= TempSample)
    {
    	//Getting Present RMS
		uint32_t MeasTime = HAL_GetTick();
        if(FADC_p->StatusDMA != ADC_DMA_Ready)
        {
        	FADC_p->StatusDMA = ADC_DMA_Ready;
        }
		while((HAL_GetTick() - MeasTime)>20)
		{
			FADC_Get_VRMS(FADC_p);
			if(FADC_p->StatusDMA == ADC_DMA_Ready)
			{
				break;
			}
		}

        ++TempSample;
        TripVrmsSum += FADC_p->Vrms1;
//        TripIrmsSum += RMSS.Irms;

        if (TempSample == TripVrmsSample)
        { // Checking the abnormal voltage

        	FADC_p->Vrms1 = TripVrmsSum / TempSample;



            if ((FADC_p->Vrms1 > AbnormalValues->OvVolt) && ((AbnormalValues->RelayStatus == RelayOn) || (AbnormalValues->RelayStatus == DeviceInit) || (AbnormalValues->OutPutStatus != OverVoltage))) // check OV
            { // Checking Over voltage
            	AbnormalValues->RelayStatus = RelayOFF;
            	AbnormalValues->OutPutStatus = OverVoltage;
//                sprintf(text, "S,%.2f,%c,%c,%u,%.2f,%.2f,E", RMSS.Vref, mode.Tpst, mode.Status, RMSS.OCcount, RMSS.Vrms, RMSS.Irms);
            	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, RelayOFFValue);//Relay is Trip

            	//On HighPTLED

            	GetVref(FADC_p);
                return;
            }

            if ((FADC_p->Vrms1 < AbnormalValues->UnVlt) && ((AbnormalValues->RelayStatus == RelayOn) || (AbnormalValues->RelayStatus == DeviceInit) || (AbnormalValues->OutPutStatus != UnderVoltage))) // check UV
            { // Checking Under voltage
            	AbnormalValues->RelayStatus = RelayOFF;
            	AbnormalValues->OutPutStatus = UnderVoltage;

//                sprintf(text, "S,%.2f,%c,%c,%u,%.2f,%.2f,E", RMSS.Vref, mode.Tpst, mode.Status, RMSS.OCcount, RMSS.Vrms, RMSS.Irms);
            	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, RelayOFFValue);//Relay is Trip

            	//On HighPTLED

            	GetVref(FADC_p);

                return;
            }
        }


        if (TempSample == NormalSample)
        { // Checking normal state count



        	FADC_p->Vrms1 = TripVrmsSum / TempSample;
            /*
            Serial.print("peak = ");
            Serial.print(peakVoltage,4);
            Serial.print("RMSS.Vrms = ");
            Serial.println(RMSS.Vrms, 5);*/

            if ( ((AbnormalValues->RelayStatus == RelayOFF) || (AbnormalValues->RelayStatus == DeviceInit))&& (FADC_p->Vrms1 < (AbnormalValues->OvVolt - AbnormalValues->diffV))&& (FADC_p->Vrms1 > (AbnormalValues->NrmVolt  + AbnormalValues->diffV)))
            { // Checking normal state condition

            	// For Blink
                ++NormalStatusLoopCount;
                Normalled = Normalled ? 0 : 1;
//                digitalWrite(led[0], LOW);
//                digitalWrite(led[1], Normalled);

                TripIrmsSum = 0;
                FADC_p->Vrms1 = 0;
                TripVrmsSum = 0;
                TempSample = 0;
                if (NormalStatusLoopCount >= NormalStatusCount)
                {
                    //
                	HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, RelayONValue);//Relay is ON
                    //OFF the HighPTLED


                	AbnormalValues->RelayStatus =  RelayOn;
                	AbnormalValues->OutPutStatus = safesupply;

                    GetVref(FADC_p);


                    return;
                }
            }
            if ((HAL_GetTick() - NormalLoopTime) > NormalTimeOut)
            {
                // readVref();
//                sprintf(text, "S,%.2f,%c,%c,%u,%.2f,%.2f,E", RMSS.Vref, mode.Tpst, mode.Status, RMSS.OCcount, RMSS.Vrms, RMSS.Irms);

                return;
            }
        }
    }

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
	  ADCVar.StatusDMA = ADC_DMA_Completed;
    // This function will be called when ADC conversion is complete
    // You can process the ADC data here
      Diff_Time=HAL_GetTick()-Start_Time;

  }
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	  if(hadc->Instance == ADC1)
	  {
		  ADCVar.StatusDMA = ADC_DMA_ERROR;
		  Diff_Time=HAL_GetTick()-Start_Time;
	  }
}


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
//uint8_t Sample = 0;
//float SampleVolatge = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_Print("  LOL....");
  HAL_Delay(1000);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(10);
  ADCVar.StatusDMA = ADC_DMA_Ready;
  ADCVar.HADC =&hadc1;
  //Set the Limit values
  AbnormaVar.NrmVolt = normalVoltagevalue;
  AbnormaVar.OvVolt = OverVoltagetripvalue;
  AbnormaVar.UnVlt = UnderVoltagetripvalue;
  AbnormaVar.ctmxLt = OverCurrenttripvalue;
  AbnormaVar.diffV = normalDiffvalue;
  AbnormaVar.OutPutStatus = DeviceInit;



  GetVref(&ADCVar);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(M_RV_GPIO_Port, M_RV_Pin, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(M_FW_GPIO_Port, M_FW_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);

	  uint32_t timeout = HAL_GetTick();
	  while((HAL_GetTick()-timeout)<1000)
	  {
//		  GetVref(&ADCVar);
		  FADC_Get_VRMS(&ADCVar);

	  }





	  LCD_Clear();
	  LCD_SetCursor(0, 0);
	  sprintf(text,"Voltage %d,%d",Diff_Time,(uint16_t)(ADCVar.Vraw1*100));
	  LCD_Print(text);
	  LCD_SetCursor(1, 0);
	  sprintf(text,"%d,%d,%d",(uint16_t)(ADCVar.Vrms1),Id,(uint16_t)(ADCVar.Vref*10));
	  LCD_Print(text);
	  HAL_Delay(500);
//	  GetVref(&ADCVar);


//	  for(uint32_t id =0;id<=20000;id++)
//	  {
//
//	  }
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(M_RV_GPIO_Port, M_RV_Pin, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(M_FW_GPIO_Port, M_FW_Pin, GPIO_PIN_RESET);
//	  for(uint32_t id =0;id<=20000;id++)
//	  {
//
//	  }
//	  HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
//	  LCD_Clear();
//	  LCD_SetCursor(0, 0);
//	  LCD_Print("L_!@#$%^^&*)_");
//	  LCD_SetCursor(1, 0);
//	  LCD_Print("_+&%#@&_");


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Relay_Pin|M_FW_Pin|M_RV_Pin|RW_Pin
                          |E_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Relay_Pin M_FW_Pin M_RV_Pin */
  GPIO_InitStruct.Pin = Relay_Pin|M_FW_Pin|M_RV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RS_Pin */
  GPIO_InitStruct.Pin = RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RW_Pin E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RW_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
