#include "stm32f4xx.h"
#include "stm32f4xx_GPIO.h"
#include <stm32f4xx_tim.h>

uint16_t IC2Value = 0;
uint16_t DutyCycle = 0;
uint32_t Frequency = 0;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t freq = 600; //[kHz]
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

void Delay_Decrement(uint32_t n)
{
	while(--n);
}

void InitPinLED(void)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_Speed = GPIO_Fast_Speed;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_Init(GPIOA, &gpio);
}

void InitInput(void)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_Speed = GPIO_Low_Speed;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_Init(GPIOC, &gpio);
}

void HSEInit(void)
{
	RCC_HSEConfig(RCC_HSE_ON);
	while(RCC_WaitForHSEStartUp() == ERROR);
	
	if (RCC_WaitForHSEStartUp() == SUCCESS)
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
	else
		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
}

void PLLInit(void)
{
	RCC_PLLConfig(RCC_GetSYSCLKSource(), 8, 200, 8, 4);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
}

static void TIM_PWM_Input_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;  
  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM2 chennel2 configuration : PA.0 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);

  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

   /* --------------------------------------------------------------------------- 
    TIM2 configuration: PWM Input mode
     The external signal is connected to TIM2 CH2 pin (PB.03)
     TIM2 CCR2 is used to compute the frequency value 
     TIM2 CCR1 is used to compute the duty cycle value

    In this example TIM2 input clock (TIM2CLK) is set to APB1 clock (PCLK1), since
    APB1 prescaler is set to 1.
      TIM2CLK = PCLK1 = HCLK = SystemCoreClock

    External Signal Frequency = SystemCoreClock / TIM2_CCR2 in Hz.
    External Signal DutyCycle = (TIM2_CCR1*100)/(TIM2_CCR2) in %.
  Note: 
  SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
  Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
  function to update SystemCoreClock variable value. Otherwise, any configuration
  based on this variable will be incorrect.
  --------------------------------------------------------------------------- */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* Select the TIM2 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
   TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}

void TIM_PWM_Output_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef nvicStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	
	/* Enable the TIM3 global Interrupt */
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

int main(void) 
{
	RCC_DeInit(); 
	HSEInit();
	//PLLInit();
	TIM_PWM_Input_Config();
	TIM_PWM_Output_Config();
	SystemCoreClockUpdate();
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 2000 / freq) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 4000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
	
	//InitPinLED();
	//InitInput();
	
	while (1)
	{
		//int i;
//		if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
//		{
//			GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
//			Delay_Decrement(2000000);
//		}
		//Delay_Decrement(100000);
	}
}

void TIM2_IRQHandler(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	/* Clear TIM2 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	
	/* Get the Input Capture value */
	IC2Value = TIM_GetCapture2(TIM2);
	
	if (IC2Value != 0)
	{
		/* Duty cycle computation */
		DutyCycle = (TIM_GetCapture1(TIM2) * 100) / IC2Value;
		TIM3->CCR1 = (DutyCycle * 4000 / 100);
		TIM3->CCR2 = (DutyCycle * 4000 / 100);
		TIM3->CCR3 = (DutyCycle * 4000 / 100);
		TIM3->CCR4 = (DutyCycle * 4000 / 100);
		/* Frequency computation */
		Frequency = SystemCoreClock / IC2Value;
	}
	else
	{
		DutyCycle = 0;
		Frequency = 0;
	}
}

void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		TIM3->CCR3 = (DutyCycle * 4000 / 100);
	}
}
