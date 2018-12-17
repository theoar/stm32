#include "stm32f0xx.h"
#include <math.h>

#define DAC_OUTPUT_REGISTER_ADDR 0x40007408

#define PI_2 6.28318530718
#define PI_1 3.14159265359

#define ADC_RES ((1<<12)-1)
#define DAC_RES ((1<<12)-1)

#define CALC_VALUE_TO_ADC(VAL,RES,REF) ((uint32_t)(((VAL)/REF)*RES))
#define CALC_ADC_TO_VALUE(VAL,RES,REF) (((float)(VAL)/RES)*REF)

#define LED_BLUE GPIO_Pin_8
#define LED_GREEN GPIO_Pin_9

#define ledOn(x) GPIO_WriteBit(GPIOC, x, Bit_SET)
#define ledOff(x) GPIO_WriteBit(GPIOC, x, Bit_RESET)
#define ledToggle(x) GPIO_WriteBit(GPIOC, x, !GPIO_ReadOutputDataBit(GPIOC,x));

#define PROBES_LENGTH 2048

#define DIALS_NUMBER 3
#define FREQ_INDEX 0
#define AMP_INDEX 1
#define DUTY_INDEX 2

#define VOLTAGE_REFERENCE 3.0
#define DC 1.5
#define MAX_AMP (VOLTAGE_REFERENCE-DC)
#define MIN_AMP 0.0
#define MAX_DUTY 1.0
#define MIN_DUTY 0.0
#define MAX_FREQ 100
#define MIN_FREQ 0.01

#define AVERAGE_TIME 500
#define ADC_CHANGE_MARGIN 3

#define UPDATE_NONE 0
#define UPDATE_FREQ (1<<0)
#define UPDATE_AMP (1<<1)
#define UPDATE_DUTY (1<<2)
#define SET_UPDATE_FLAG(X) (1<<X)
#define SET_ALL_UPDATE_FLAG (UPDATE_FREQ | UPDATE_AMP | UPDATE_DUTY)

#define SINCOFF (PI_2/(PROBES_LENGTH*2))

#define BUTTON_PIN GPIO_Pin_0
#define BUTTON_OSC_TIME 512
#define BUTTON_HOLD_TIME (8*BUTTON_OSC_TIME)
#define BUTTON_LONG_HOLD_TIME (4*BUTTON_HOLD_TIME)
#define BUTTON_DELAY -128

#define BUTTON_NO_STATE (0)
#define BUTTON_OSC (1<<0)
#define BUTTON_CLICKED (1<<1)
#define BUTTON_LONG (1<<2)

uint8_t ButtonState = 0;
int16_t ButtonTime = 0;

uint16_t Probes[PROBES_LENGTH];

uint16_t AdcRawValues[DIALS_NUMBER];
float AdcFilteredValues[DIALS_NUMBER];
float LastFilteredValues[DIALS_NUMBER];
float RequiredParameters[DIALS_NUMBER];
float RealParameters[DIALS_NUMBER];

#define FUNCTION_NUMBER 4
uint8_t Function = 0;

uint16_t Period = 1000-1;
uint16_t Prescaler = 48-1;

#define STATE_NUMBER 4
uint8_t CurrentState = 1;

uint16_t TriangleFirstDiff = 0;
uint16_t TriangleSecondDiff = 0;	

uint32_t FirstTriangleDiff = 0;
uint32_t SecondTriangleDiff = 0;
uint32_t FirstTriangle = 0;
uint32_t SecondTriangle = 0;

uint16_t HighSquare = 0;
uint16_t LowSquare = 0;

uint16_t DutyLength = 0;

static void initGPIO(void);
static void initDAC(void);
static void initTIM2(void);
static void initTIM3(void);
static void initADC(void);
static void initDMA(void);

static void startWaveform(void);

static void probeADC(void);

static uint8_t filterAdcRawValues(void);
static void initAdcFilter(void);

static void updateRequiredParameters(uint8_t Which);
static void updateRealParameters(uint8_t Which);

static void setupTimerParams(void);
static void updateTimerParams(void);

static void calculate_sinus(void);
static void calculate_triange(void);
static void calculate_sawtooth(void);
static void calculate_square(void);

static void calculate_sinus_1(void);
static void calculate_triange_1(void);
static void calculate_square_1(void);
static void calculate_sawtooth_1(void);

uint8_t poll_buttons(void);

static void stopWaveform(void);


#define DC_COFF (DC*(ADC_RES/VOLTAGE_REFERENCE))


volatile uint32_t Time = 0;
volatile float FinalTime = 0;

uint32_t Test1;
uint32_t Test2;

void TIM3_IRQHandler()
{		
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {		
		TIM_Cmd(TIM14, ENABLE);
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, 1);	
		
		CurrentState++;
		CurrentState = CurrentState%STATE_NUMBER;
		
		uint16_t Offset = (CurrentState%2)*PROBES_LENGTH/2;
		uint16_t RealIndex = CurrentState*PROBES_LENGTH/2;
		uint16_t Border = PROBES_LENGTH/2+RealIndex;	
		
		uint16_t X;
		switch(Function)
		{
			case 0:
			{
				for(uint16_t X = 0; X<(PROBES_LENGTH/2); ++X)	
					Probes[X+Offset] = (uint16_t)(2*DC_COFF)-Probes[Offset+X];
			}
			break;
			
			case 1:
			{					
				for(X = RealIndex; X<Border && X<PROBES_LENGTH; ++X)					
					Probes[Offset+X-RealIndex] = (FirstTriangle+X*FirstTriangleDiff) >> 16;
				for(; X<Border; ++X)
					Probes[Offset+X-RealIndex] = (SecondTriangle-(X-PROBES_LENGTH)*FirstTriangleDiff) >> 16;				
			}
			break;
			
			case 2:
			{				
				for(X = RealIndex; X<Border && X< DutyLength; ++X)		
					Probes[Offset+X-RealIndex] = HighSquare;				
				for(; X<Border; ++X)
					Probes[Offset+X-RealIndex] = LowSquare;	
			}
			break;	
			
			case 3:
			{			
				for(X = RealIndex; X<DutyLength && X<Border; ++X)
					Probes[Offset+X-RealIndex] = FirstTriangle+X*FirstTriangleDiff;
				for(;X<Border; ++X)
					Probes[Offset+X-RealIndex] = SecondTriangle-(X-DutyLength)*SecondTriangleDiff;				
			}
			break;	
			
		}
			GPIO_WriteBit(GPIOC, GPIO_Pin_4, 0);	
		
		TIM_Cmd(TIM14, DISABLE);	
		FinalTime =((TIM_GetCounter(TIM14)+(float)Time*(float)UINT16_MAX)/(SystemCoreClock/1000000));
		TIM_SetCounter(TIM14, 0);
		Time = 0;
	}
	
};

void TIM14_IRQHandler()
{
	if (TIM_GetITStatus(TIM14, TIM_IT_Update) == SET)
  {
		TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
		Time++;		
	}
}
int main(void)
{		
	initGPIO();			
	initADC();	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	TIM_TimeBaseInitTypeDef Tm;	
	TIM_DeInit(TIM14);	
	Tm.TIM_ClockDivision  = 0;
	Tm.TIM_Prescaler = 0;
	Tm.TIM_CounterMode = TIM_CounterMode_Up;
	Tm.TIM_Period = UINT16_MAX;	
	TIM_TimeBaseInit(TIM14, &Tm);		
	TIM_SelectOutputTrigger(TIM14, TIM_TRGOSource_Update);	
	TIM_ARRPreloadConfig(TIM14, ENABLE);
	TIM_SetCounter(TIM14, 0);		
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);	
	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
	
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM14_IRQn;
	nvic.NVIC_IRQChannelPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);		
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_4, 0);		
	
	updateRequiredParameters(SET_ALL_UPDATE_FLAG);
	setupTimerParams();
	updateRealParameters(SET_ALL_UPDATE_FLAG);		

	void (*Functions[])(void) = { &calculate_sinus_1, &calculate_triange_1, &calculate_square_1, &calculate_sawtooth_1};	
	
	Functions[Function]();
	
	Test1=RealParameters[0];
	Test2=RealParameters[1];
	
	initTIM3();
	initTIM2();	
	initDAC();
	initDMA();
	
	updateTimerParams();		
	startWaveform();	
	
	uint8_t WhichParamChanged = 0;
	uint8_t ButtonAction = 0;
	while(1)		
	{
		probeADC();
		if((ButtonAction=poll_buttons())==BUTTON_CLICKED)
			Function=(Function+1)%FUNCTION_NUMBER;
		if((WhichParamChanged=filterAdcRawValues())!=0 || ButtonAction !=0)
		{
				ledOn(LED_GREEN);
				GPIO_WriteBit(GPIOC, GPIO_Pin_4, 1);	
			
				stopWaveform();
			
				updateRequiredParameters(WhichParamChanged);
				setupTimerParams();
				updateRealParameters(WhichParamChanged);					
			
				CurrentState = 1;
			
				Functions[Function]();
		
				updateTimerParams();

				startWaveform();

				ledOff(LED_GREEN);
				GPIO_WriteBit(GPIOC, GPIO_Pin_4, 0);				
		}		

	}	
	return 0;
}

void initGPIO(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef Leds;
	Leds.GPIO_Mode = GPIO_Mode_OUT;
	Leds.GPIO_Pin = LED_BLUE | LED_GREEN | GPIO_Pin_4;
	Leds.GPIO_OType = GPIO_OType_PP;
	Leds.GPIO_PuPd = GPIO_PuPd_NOPULL;
	Leds.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOC, &Leds);		
	
	GPIO_InitTypeDef Button;
	Leds.GPIO_Mode = GPIO_Mode_IN;
	Leds.GPIO_Pin = BUTTON_PIN;
	Leds.GPIO_OType = GPIO_OType_PP;
	Leds.GPIO_PuPd = GPIO_PuPd_NOPULL;
	Leds.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &Button);		
	
	ledOff(LED_BLUE);
	ledOff(LED_GREEN);
}

void initDAC(void)
{		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	
	GPIO_InitTypeDef DacOutput;
	DacOutput.GPIO_Mode = GPIO_Mode_AN;
	DacOutput.GPIO_Pin = GPIO_Pin_4;	
	DacOutput.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA, &DacOutput);		  			
		
	DAC_InitTypeDef DacConfig;
	
	DAC_DeInit();  
	DacConfig.DAC_Trigger = DAC_Trigger_T2_TRGO;
	DacConfig.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
	DacConfig.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DacConfig.DAC_OutputBuffer = DAC_OutputBuffer_Enable;	
	
	DAC_Init(DAC_Channel_1, &DacConfig);
  
}

void initDMA(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitTypeDef DmaConfig;
	
	DMA_DeInit(DMA1_Channel3); 	

  DmaConfig.DMA_PeripheralBaseAddr = DAC_OUTPUT_REGISTER_ADDR;
  DmaConfig.DMA_MemoryBaseAddr = (uint32_t)Probes;
  DmaConfig.DMA_DIR = DMA_DIR_PeripheralDST;
  DmaConfig.DMA_BufferSize = PROBES_LENGTH;
  DmaConfig.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DmaConfig.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DmaConfig.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DmaConfig.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DmaConfig.DMA_Mode = DMA_Mode_Circular;
  DmaConfig.DMA_Priority = DMA_Priority_High; 
	DmaConfig.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel3, &DmaConfig);
}	

void initTIM2(void)
{	
	TIM_TimeBaseInitTypeDef TimConfig;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	
	TIM_DeInit(TIM2);
	
  TIM_TimeBaseStructInit(&TimConfig); 
  TimConfig.TIM_Period = Period;          
  TimConfig.TIM_Prescaler = Prescaler;           
	TimConfig.TIM_ClockDivision = 0;     
  TimConfig.TIM_CounterMode = TIM_CounterMode_Up;  	
  TIM_TimeBaseInit(TIM2, &TimConfig);	
	TIM_InternalClockConfig(TIM2);	
	
	TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Disable);
	
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);		
	TIM_ARRPreloadConfig(TIM2, ENABLE);	
}

void initTIM3(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseInitTypeDef Tm;
	
	TIM_DeInit(TIM3);	
	Tm.TIM_ClockDivision  = 0;
	Tm.TIM_Prescaler = 0;
	Tm.TIM_CounterMode = TIM_CounterMode_Up;
	Tm.TIM_Period = PROBES_LENGTH/2-1;	
	TIM_TimeBaseInit(TIM3, &Tm);		
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_SetCounter(TIM3, 0);
		
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_External1);	
	TIM_ITRxExternalClockConfig(TIM3, TIM_TS_ITR1);		
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Disable);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);		
}

void initADC(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	
	GPIO_InitTypeDef AdcPinConf;
	AdcPinConf.GPIO_Mode = GPIO_Mode_AN;
	AdcPinConf.GPIO_OType = GPIO_OType_PP;
	AdcPinConf.GPIO_PuPd = GPIO_PuPd_NOPULL;
	AdcPinConf.GPIO_Speed = GPIO_Speed_Level_1;
	
	const uint8_t Pins[] = {GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3};	
	uint8_t Sizeof = sizeof(Pins);
	
	for(uint8_t X = 0; X<Sizeof; ++X)
	{
		AdcPinConf.GPIO_Pin = Pins[X];
		GPIO_Init(GPIOA, &AdcPinConf);
	}
	
	ADC_InitTypeDef AdcConf;
	
	ADC_DeInit(ADC1);
	ADC_StructInit(&AdcConf);
	
	AdcConf.ADC_Resolution = ADC_Resolution_12b;
	AdcConf.ADC_DataAlign = ADC_DataAlign_Right;
	AdcConf.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	AdcConf.ADC_ContinuousConvMode = DISABLE;
	AdcConf.ADC_ScanDirection = ADC_ScanDirection_Upward;		
	
	ADC_DiscModeCmd(ADC1, ENABLE);	
	
	ADC_Init(ADC1, &AdcConf);	

	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_13_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_13_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_13_5Cycles);
	
	ADC_Cmd(ADC1, ENABLE);	
}

static void startWaveform(void)
{
	DMA_Cmd(DMA1_Channel3, ENABLE);	
	DAC_Cmd(DAC_Channel_1, ENABLE);	
	DAC_DMACmd(DAC_Channel_1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

static void stopWaveform(void)
{
	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	DMA_Cmd(DMA1_Channel3, DISABLE);	
	DAC_Cmd(DAC_Channel_1, DISABLE);	
	DAC_DMACmd(DAC_Channel_1, DISABLE);
	initDMA();
			
	TIM_SetCounter(TIM2, 0);	
	TIM_SetCounter(TIM3, 0);
}

void probeADC(void)
{
		for(uint8_t X = 0; X<DIALS_NUMBER; ++X)
		{			
			ADC_StartOfConversion(ADC1);
			while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
			AdcRawValues[X] = ADC_GetConversionValue(ADC1);
		}
}

void initAdcFilter(void)
{
	probeADC();		
	for(uint8_t X=0; X<DIALS_NUMBER; ++X)	
	{
		AdcFilteredValues[X] = 0;
		LastFilteredValues[X] = AdcRawValues[X];			
	}
}

uint8_t filterAdcRawValues(void)
{
	const uint16_t AverageTime = AVERAGE_TIME;
	const uint8_t AdcChangeMargin = ADC_CHANGE_MARGIN;
	static uint16_t CurrentTime = 0;	
	
	CurrentTime++;
	
	for(uint8_t X = 0; X<DIALS_NUMBER; ++X)
		AdcFilteredValues[X] = AdcFilteredValues[X]+(float)AdcRawValues[X]/(float)AverageTime;
	
	if(CurrentTime>=AverageTime)
	{
		CurrentTime = 0;
		uint8_t UpdateFlag = 0;
		for(uint8_t X = 0; X<DIALS_NUMBER; ++X)
		{
			if(AdcFilteredValues[X]<LastFilteredValues[X]-AdcChangeMargin || AdcFilteredValues[X]>LastFilteredValues[X]+AdcChangeMargin)
			{
				UpdateFlag|=SET_UPDATE_FLAG(X);
				LastFilteredValues[X] = AdcFilteredValues[X];
			}
			AdcFilteredValues[X] = 0;
		}
		
		return UpdateFlag;
	}
	else
	{
		return 0;
	}
}

void updateRequiredParameters(uint8_t Which)
{	
	for(uint8_t X = 0; X<DIALS_NUMBER; ++X)
	{
		switch(Which & SET_UPDATE_FLAG(X))
		{			
			case UPDATE_FREQ:
				RequiredParameters[FREQ_INDEX] = CALC_ADC_TO_VALUE(LastFilteredValues[FREQ_INDEX],ADC_RES,(MAX_FREQ-MIN_FREQ))+MIN_FREQ;
				break;
			case UPDATE_DUTY:
				RequiredParameters[DUTY_INDEX] = CALC_ADC_TO_VALUE(LastFilteredValues[DUTY_INDEX], ADC_RES,(MAX_DUTY-MIN_DUTY))+MIN_DUTY;
				break;
			case UPDATE_AMP:
				RequiredParameters[AMP_INDEX] = CALC_ADC_TO_VALUE(LastFilteredValues[AMP_INDEX], ADC_RES, (MAX_AMP-MIN_AMP))+ MIN_AMP;
				break;
		}
	}		
}

void setupTimerParams(void)
{
	float Product = SystemCoreClock/((PROBES_LENGTH*2)*RequiredParameters[FREQ_INDEX]);
	uint32_t IntProduct = (Product - (uint32_t)Product < 0.5) ? (uint32_t)Product : (uint32_t)(Product+1);
	
	const uint8_t Divers[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 31};
	const uint8_t Sizeof = sizeof(Divers);
	
	if(IntProduct < 2)
	{		
		Prescaler = 0;
		Period = 1;
		return;
	}
	
	uint32_t Multipler = 1;	
	uint32_t ProductCpy = IntProduct;
	uint32_t Rest = 0;
	uint32_t LastRest = 255;
	uint8_t LastRestIndex = 255;

	if(IntProduct > 65536)	
	{
		uint8_t X = 0;
		while(X < Sizeof)
		{						
			Rest = ProductCpy % Divers[X];
			if(Rest == 0)
			{
				ProductCpy/=Divers[X];
				Multipler*=Divers[X];
				
				if(ProductCpy <= 65536)	
					break;					
				else
					continue;
			}
			else
			{
				if(Rest <= LastRest)
				{
					LastRest = Rest;
					LastRestIndex = X;
				}
				X++;
			}
		}
		
		if(Multipler*ProductCpy == IntProduct && ProductCpy <= 65536 && Multipler <= 65536)
		{
			Prescaler = Multipler - 1;
			Period = ProductCpy - 1;
		}
		else
		{
			Multipler*=Divers[LastRestIndex];
			ProductCpy = IntProduct/Multipler;				
			if(ProductCpy<=65536)
			{
				Prescaler = Multipler - 1;
				Period = ProductCpy - 1;
			}		
			else
			{
				while(ProductCpy>65536)
				{
					ProductCpy/=2;
					Multipler*=2;
				}
				Prescaler = Multipler - 1;
				Period = ProductCpy - 1;
			}		
		}		
	}	
	else
	{
		Prescaler = 0;
		Period = IntProduct-1;
	}	
}

void updateTimerParams(void)
{
	TIM_SetAutoreload(TIM2, Period);
	TIM_PrescalerConfig(TIM2, Prescaler, TIM_PSCReloadMode_Update);
}

void updateRealParameters(uint8_t Which)
{
	uint32_t Tmp;
	for(uint8_t X = 0; X<DIALS_NUMBER; ++X)
	{		
		switch(Which & SET_UPDATE_FLAG(X))
		{
			case UPDATE_FREQ:
				RealParameters[FREQ_INDEX] =  SystemCoreClock;
				RealParameters[FREQ_INDEX] /=  (float)((Period+1)*(Prescaler+1));	
				RealParameters[FREQ_INDEX] /= (PROBES_LENGTH*2);				
				break;
			case UPDATE_DUTY:
				Tmp = RequiredParameters[DUTY_INDEX]*PROBES_LENGTH*2;
				RealParameters[DUTY_INDEX] = (float)(Tmp)/(float)(PROBES_LENGTH*2);
				break;
			case UPDATE_AMP:
				Tmp = CALC_VALUE_TO_ADC(RequiredParameters[AMP_INDEX], DAC_RES, VOLTAGE_REFERENCE);
				RealParameters[AMP_INDEX] = CALC_ADC_TO_VALUE(Tmp, DAC_RES, VOLTAGE_REFERENCE);
				break;
		}
	}
}



void calculate_sinus(void)
{	
	float Tmp;	
	for(uint16_t X = 0; X<PROBES_LENGTH; ++X)	
	{
		Tmp = DC+RealParameters[AMP_INDEX]*sinf(SINCOFF*X);	
		Probes[X] = CALC_VALUE_TO_ADC(Tmp, DAC_RES, VOLTAGE_REFERENCE);		
	}
}


void calculate_sinus_1(void)
{	
	for(uint16_t X = 0; X<PROBES_LENGTH/2+1; ++X)		
		Probes[X] = RealParameters[AMP_INDEX]*((sinf(SINCOFF*X)*ADC_RES)/VOLTAGE_REFERENCE)+DC_COFF;		
	for(uint16_t X = 1; X<PROBES_LENGTH/2; ++X)		
		Probes[X+PROBES_LENGTH/2] = Probes[PROBES_LENGTH/2-X];	
}

//void calculate_triange_1(void)
//{	
//	float A = (RealParameters[AMP_INDEX]/(PROBES_LENGTH))*2;
//	float First = 2*DC-A;	
//	for(uint16_t X = 0; X<PROBES_LENGTH; ++X)	
//		Probes[X] = ((First+A*X*ADC_RES)/VOLTAGE_REFERENCE);
//	
//	TriangleFirstDiff = (A*ADC_RES)/VOLTAGE_REFERENCE;
//}

void calculate_triange_1(void)
{	
	float A1 = ((RealParameters[AMP_INDEX]*2)/PROBES_LENGTH);	
	FirstTriangleDiff = (A1*ADC_RES)/VOLTAGE_REFERENCE;		
	FirstTriangle = ((DC-RealParameters[AMP_INDEX])*ADC_RES)/VOLTAGE_REFERENCE;
	
	float Tmp = (A1*ADC_RES)/VOLTAGE_REFERENCE;	
	FirstTriangleDiff = (((uint16_t)(Tmp)) << 16);
	FirstTriangleDiff |= (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);	
	
	Tmp = ((DC-RealParameters[AMP_INDEX])*ADC_RES)/VOLTAGE_REFERENCE;
	FirstTriangle = ((uint16_t)(Tmp) << 16); 
	FirstTriangle |=  (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);
	
	SecondTriangle=FirstTriangle+FirstTriangleDiff*PROBES_LENGTH;
	
	for(uint16_t X = 0; X<PROBES_LENGTH; ++X)	
		Probes[X] = (FirstTriangle+FirstTriangleDiff*X) >> 16;
}

void calculate_square_1(void)
{	
	DutyLength  = RealParameters[DUTY_INDEX]*PROBES_LENGTH*2;		
	HighSquare = (((RealParameters[AMP_INDEX]+DC)*ADC_RES)/VOLTAGE_REFERENCE);
	LowSquare = (((-RealParameters[AMP_INDEX]+DC)*ADC_RES)/VOLTAGE_REFERENCE);
	for(uint16_t X = 0; X<DutyLength &&	X<PROBES_LENGTH; ++X)
		Probes[X] = HighSquare;
	for(uint16_t X = DutyLength; X<PROBES_LENGTH;++X)
		Probes[X] = LowSquare;
}

void calculate_sawtooth_1(void)
{	
	DutyLength = RealParameters[DUTY_INDEX]*PROBES_LENGTH*2;
	
	float A1 = DutyLength != 0 ? ((RealParameters[AMP_INDEX]*2)/DutyLength) : 0;
	float A2 = (PROBES_LENGTH*2-DutyLength) != 0 ? ((RealParameters[AMP_INDEX]*2)/(PROBES_LENGTH*2-DutyLength)) : 0;	
	float Tmp;

	
	Tmp = (A1*ADC_RES)/VOLTAGE_REFERENCE;	
	FirstTriangleDiff = (((uint16_t)(Tmp)) << 16);
	FirstTriangleDiff |= (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);	
	
	Tmp = ((DC-RealParameters[AMP_INDEX])*ADC_RES)/VOLTAGE_REFERENCE;
	FirstTriangle = ((uint16_t)(Tmp) << 16); 
	FirstTriangle |=  (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);
	
	Tmp = (A2*ADC_RES)/VOLTAGE_REFERENCE;	
	SecondTriangleDiff = (((uint16_t)(Tmp)) << 16);
	SecondTriangleDiff |= (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);	
	
	Tmp = ((DC+RealParameters[AMP_INDEX])*ADC_RES)/VOLTAGE_REFERENCE;
	SecondTriangle = ((uint16_t)(Tmp) << 16); 
	SecondTriangle |=  (uint16_t)((Tmp-(uint16_t)(Tmp))*65536);
	
	for(int16_t X = 0; X<DutyLength && X<PROBES_LENGTH; ++X)	
		Probes[X] = (FirstTriangle+X*FirstTriangleDiff)>>16;
	for(int16_t X = DutyLength; X<PROBES_LENGTH; ++X)
		Probes[X] = (SecondTriangle-(X-DutyLength)*SecondTriangleDiff)>>16;		
}

uint8_t poll_buttons(void)
{	
	uint8_t Action = 0;
	uint8_t State = GPIO_ReadInputDataBit(GPIOA, BUTTON_PIN);	
	if(State == 1)
	{
		switch(ButtonState)
		{
			case BUTTON_NO_STATE:
			{
				ButtonState = BUTTON_OSC;	
			}
			break;
			
			case BUTTON_OSC:
			{
					if(ButtonTime>= BUTTON_OSC_TIME)
					{						
						Action = BUTTON_CLICKED;
						ButtonState = BUTTON_CLICKED;
						ButtonTime=0;
					}
					else
						ButtonTime++;
			}
			break;
			
			case BUTTON_CLICKED:
			{
					if(ButtonTime>= BUTTON_HOLD_TIME)
					{	
						Action = BUTTON_LONG;
						ButtonState=BUTTON_LONG;
						ButtonTime=0;
					}
					else
						ButtonTime++;
			}
			break;
			
			case BUTTON_LONG:
			{
				if(ButtonTime>= BUTTON_LONG_HOLD_TIME)
				{
					Action = BUTTON_LONG;
					ButtonTime=0;
				}
				else
					ButtonTime++;
			}
			break;		
		}
	}
	else
	{
		switch(ButtonState)
		{
			case BUTTON_OSC:
			{
				ButtonTime--;
				if(ButtonTime<=BUTTON_DELAY)
					ButtonState=BUTTON_NO_STATE;
			}
			break;			
			
			case BUTTON_CLICKED:
			{
				ButtonState=BUTTON_NO_STATE;
				ButtonTime=0;
			}
			break;
			
			case BUTTON_LONG:
			{
				ButtonState=BUTTON_NO_STATE;
				ButtonTime=0;
			}
			break;
			
			case BUTTON_NO_STATE:				
			{
				ButtonTime=0;
			}
			
			break;
		}
	}	
	return Action;
}

void calculate_triange(void)
{
	float A = (RealParameters[AMP_INDEX]/(PROBES_LENGTH));
	for(uint16_t X = 0; X<PROBES_LENGTH/2+1; ++X)	
		Probes[X] = ((A*(X-(PROBES_LENGTH)/4)+DC)/VOLTAGE_REFERENCE)*((1<<12)-1);	
	for(uint16_t X = PROBES_LENGTH-1; X>PROBES_LENGTH/2 ; --X)
		Probes[X] = Probes[PROBES_LENGTH-X];	
}

void calculate_sawtooth(void)
{
	int16_t FirstTriangle = RealParameters[DUTY_INDEX]*PROBES_LENGTH;
	int16_t SecondTriangle = PROBES_LENGTH - FirstTriangle;
	float A1 = FirstTriangle != 0 ? (RealParameters[AMP_INDEX]/FirstTriangle)*2 : 0;
	float A2 = FirstTriangle!=PROBES_LENGTH ? -(RealParameters[AMP_INDEX]/(PROBES_LENGTH-FirstTriangle))*2 : 0;	
	float Tmp;
	
	for(int16_t X = 0; X<FirstTriangle; ++X)	
	{
		Tmp = (A1*((X-(FirstTriangle/2)))+DC);
		Probes[X] = CALC_VALUE_TO_ADC(Tmp, DAC_RES, VOLTAGE_REFERENCE);		
	}
	for(int16_t X = FirstTriangle; X<PROBES_LENGTH; ++X)
	{
		Tmp = A2*(X-(FirstTriangle+SecondTriangle/2))+DC;
		Probes[X] = CALC_VALUE_TO_ADC(Tmp, DAC_RES, VOLTAGE_REFERENCE);				
	}
}

void calculate_square(void)
{
	uint16_t High = RealParameters[DUTY_INDEX]*PROBES_LENGTH;		
	uint16_t ValueHigh = ((RealParameters[AMP_INDEX]+DC)/VOLTAGE_REFERENCE)*((1<<12)-1);
	uint16_t ValueLow = ((-RealParameters[AMP_INDEX]+DC)/VOLTAGE_REFERENCE)*((1<<12)-1); 
	for(uint16_t X = 0; X<High; ++X)
		Probes[X] = ValueHigh;
	for(uint16_t X = High; X<PROBES_LENGTH;++X)
		Probes[X] = ValueLow;
}

