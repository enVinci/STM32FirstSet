#include "stm32f4xx.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_pwr.h"

void GPIOPB12_EXTInputInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable clock for GPIOB */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
 /* Set pin as input */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
	
	/* PB12 is connected to EXTI_Line12 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    /* Enable interrupt */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStructure);
		
		/* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStructure);
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
	RCC_PCLK1Config(RCC_SYSCLK_Div2);
	RCC_PLLConfig(RCC_GetSYSCLKSource(), 4, 100, 2, 4);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
}

void SleepMode()
{
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT, ENABLE);
	__WFI();
}

void StopMode()
{
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

int main(void) 
{
	/* System init */
	SystemInit();
	RCC_DeInit();
	HSEInit();
	PLLInit();
	
	/* Configure PA5 as LED */
	InitPinLED();
	 /* Configure PB12 as interrupt */
    GPIOPB12_EXTInputInit();
	
	//SleepMode();
	
	
	while (1) {
		StopMode();
	}
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        /* Do your stuff when PB12 is changed */
			
        GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
		
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
