#include "stm32f0xx.h"

#define LED_BLUE GPIO_Pin_8
#define LED_GREEN GPIO_Pin_9

#define ledOn(x) GPIO_WriteBit(GPIOC, x, Bit_SET)
#define ledOff(x) GPIO_WriteBit(GPIOC, x, Bit_RESET)
#define ledToggle(x) GPIO_WriteBit(GPIOC, x, !GPIO_ReadOutputDataBit(GPIOC,x));

enum ButtonStates
{
	UNACTIVE = 0,
	CLICKED = 1,
	HOLD = 2
};

typedef struct BUTTON_STRUCTURE
{
	uint8_t PinNo;	
	GPIO_TypeDef *Port;
	
	uint8_t State;
	uint16_t HoldTime;
	uint16_t CurrentTime;
	
} Button ;
#define HOLDTIME 100

typedef struct ENKODER
{
	uint8_t PinANo, PinBNo;
	GPIO_TypeDef *PortAPtr, *PortBPtr;	
	Button * ButtonPtr;
	
	uint8_t Type;	
	uint8_t State;
	
} Enkoder;


Enkoder Enc1, Enc2;
Button Button1, Button2;
Enkoder *EnkPtr[] = {&Enc1, &Enc2};
Button *BuuttonBtr[] = {&Button1, &Button2};
uint8_t Size = 2;

void initGPIO_RCC(GPIO_TypeDef * Port);
void initGPIO_INPUT(GPIO_TypeDef * Port, uint8_t Pin);
void initEnc(Enkoder * Enk, uint8_t PinA, uint8_t PinB, GPIO_TypeDef * PortA, GPIO_TypeDef * PortB, Button *ButtonPointer);
void initButton(Button *Btt, uint8_t Pin, GPIO_TypeDef * Port, uint16_t HoldTime);

void initTIM3(void);
void initLeds(void);

void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
  {	
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		
		uint8_t NewState;		
		Enkoder *Tested;
		for(uint8_t X = 0; X<Size; ++X)
		{
			Tested = EnkPtr[X];
			NewState = (GPIO_ReadInputDataBit(Tested->PortAPtr, Tested->PinANo) << 1) | (GPIO_ReadInputDataBit(Tested->PortBPtr, Tested->PinBNo) << 0);
		
			if(NewState!=(Tested->State & 3))
			{	
				Tested->State <<= 2;			
				Tested->State |= NewState;
				Tested->State &= 15;
				
				if (Tested->State==13 )  //|| Enc1.State==1 || Enc1.State==8 || Enc1.State==7)	
				{	
					ledToggle(LED_GREEN);				
				}
				else 
				{
					if(Tested->State==14 )// || Enc1.State==2 || Enc1.State==11 || Enc1.State==4)		
						ledToggle(LED_GREEN);
				}
			}
			
			if(Tested->ButtonPtr!=0)
			{
				if(GPIO_ReadInputDataBit(Tested->ButtonPtr->Port, Tested->ButtonPtr->PinNo) == 0)
				{
					if(Tested->ButtonPtr->State == UNACTIVE)
					{
						ledOn(LED_BLUE);
						Tested->ButtonPtr->State = CLICKED;
						Tested->ButtonPtr->CurrentTime = Tested->ButtonPtr->HoldTime;
					}
					else
					{
						if(Tested->ButtonPtr->State==CLICKED || Tested->ButtonPtr->State == HOLD)
							Tested->ButtonPtr->CurrentTime--;
						if(Tested->ButtonPtr->CurrentTime==0)
						{
							ledToggle(LED_BLUE);
							Tested->ButtonPtr->State = HOLD;
							Tested->ButtonPtr->CurrentTime = Tested->ButtonPtr->HoldTime;
						}
					}
				}	
				else
				{					
					Tested->ButtonPtr->State = UNACTIVE;
					ledOff(LED_BLUE);
				}
			}
		}		
	}
}

int main(void)
{
	initLeds();
	
	initButton(&Button1, GPIO_Pin_1, GPIOB, HOLDTIME);
	initButton(&Button2, GPIO_Pin_2, GPIOB, HOLDTIME);
	
	initEnc(&Enc1, GPIO_Pin_4, GPIO_Pin_5, GPIOA, GPIOA, &Button1);
	initEnc(&Enc2, GPIO_Pin_6, GPIO_Pin_7, GPIOA, GPIOA, &Button2);
	
	initTIM3();
	

	while(1)
	{

	}	
	
}

void initEnc(Enkoder * Enk, uint8_t PinA, uint8_t PinB, GPIO_TypeDef * PortA, GPIO_TypeDef * PortB, Button *ButtonPointer)
{
	Enk->PinANo = PinA;
	Enk->PinBNo = PinB;
	Enk->PortAPtr = PortA;
	Enk->PortBPtr = PortB;
	Enk->ButtonPtr = ButtonPointer;	
	
	Enk->State = 3;
	
	initGPIO_RCC(PortA);
	initGPIO_RCC(PortB);	
	
	initGPIO_INPUT(PortA, PinA);
	initGPIO_INPUT(PortB, PinB);
}

void initButton(Button *Btt, uint8_t Pin, GPIO_TypeDef * Port, uint16_t HoldTime)
{
	Btt->PinNo = Pin;
	Btt->HoldTime = HoldTime;
	Btt->Port = Port;
	Btt->CurrentTime = 0;
	Btt->State = UNACTIVE;
	
	initGPIO_RCC(Port);	
	initGPIO_INPUT(Port, Pin);
}

void initGPIO_RCC(GPIO_TypeDef * Port)
{
	uint8_t Size = 6;
	const GPIO_TypeDef *Ports[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF};
	const uint32_t RccStruct[] = {RCC_AHBPeriph_GPIOA, RCC_AHBPeriph_GPIOB, RCC_AHBPeriph_GPIOC, RCC_AHBPeriph_GPIOD, RCC_AHBPeriph_GPIOE, RCC_AHBPeriph_GPIOF};
	
	for(uint8_t X = 0; X<Size; ++X)
	{
		if(Port == Ports[X])
		{
			RCC_AHBPeriphClockCmd(RccStruct[X], ENABLE);
			break;
		}
	}
}

void initGPIO_INPUT(GPIO_TypeDef * Port, uint8_t Pin)
{
	GPIO_InitTypeDef Init;
	Init.GPIO_Mode = GPIO_Mode_IN;
	Init.GPIO_OType = GPIO_OType_PP;
	Init.GPIO_PuPd = GPIO_PuPd_UP;
	Init.GPIO_Speed = GPIO_Speed_Level_1;	
	Init.GPIO_Pin = Pin;
	GPIO_Init(Port, &Init);
	GPIO_WriteBit(Port, Pin, 1);	
}

void initTIM3(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef Conf;
	Conf.TIM_ClockDivision = 0;
	Conf.TIM_CounterMode = TIM_CounterMode_Up;
	Conf.TIM_Prescaler = 4800-1;
	Conf.TIM_Period = 100-1;
	Conf.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &Conf);
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	
	NVIC_InitTypeDef Nvic;
	Nvic.NVIC_IRQChannel = TIM3_IRQn;
	Nvic.NVIC_IRQChannelPriority = 0;
	Nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&Nvic);
	
	TIM_Cmd(TIM3, ENABLE);
}

void initLeds(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef Leds;
	Leds.GPIO_Mode = GPIO_Mode_OUT;
	Leds.GPIO_Pin = LED_BLUE | LED_GREEN;
	Leds.GPIO_PuPd = GPIO_PuPd_NOPULL;
	Leds.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOC, &Leds);		
	
	ledOff(LED_BLUE);
	ledOff(LED_GREEN);
}

	