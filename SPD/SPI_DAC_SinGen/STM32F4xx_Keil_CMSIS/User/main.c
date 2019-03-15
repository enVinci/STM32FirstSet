#include "stm32f4xx.h"
#include "stm32f4xx_GPIO.h"
#include "stm32f4xx_spi.h"
#include <stm32f4xx_tim.h>
#include <math.h>

# define n_points 1024

static int sin_data[n_points];  // sine LUT Array

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

void CS_PB3_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
		// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* Configure the chip select pin
	   in this case we will use PB3 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIOB->BSRRL |= GPIO_Pin_3; // set PB3 high
}

void SPI1_Master_init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

		// enable peripheral clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
    /* configure pins used by SPI3
     * PA5 = SCK
     * PA6 = MISO
     * PA7 = MOSI
     */
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
		// connect SPI1 pins to SPI alternate function
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	
	  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b; // one packet of data is 8 bits wide
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // SPI frequency is
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
    SPI_Init(SPI1, &SPI_InitStruct);

   SPI_Cmd(SPI1, ENABLE); // enable SPI1
}

void InitializeTimer(uint16_t freq)
{
		TIM_TimeBaseInitTypeDef timerInitStructure; 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
    timerInitStructure.TIM_Prescaler = 10000 - 1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 800 / freq / n_points - 1;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void EnableTimerInterrupt()
{
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure.NVIC_IRQChannelSubPriority = 1;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}

uint8_t SPI1_send(uint8_t data)
{

	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	//while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

void createSine_Array()
{
    float pi = 3.141592;
    float w;
    float yi;
    float phase = 0;
    int i;
    w = 2*pi;
    w = w/n_points;
    for (i = 0; i <= n_points; i++)
    {
        yi = 2047 * sin(phase);
        phase += w;
        sin_data[i] = 2047+yi; // write dc offset translated for a 12 bit DAC into array
    }
}

int main(void) 
{
	RCC_DeInit(); 
	HSEInit();
	//PLLInit();
	
	createSine_Array();
	SPI1_Master_init();
	CS_PB3_Init();
	InitializeTimer(500);
	EnableTimerInterrupt();
	//InitPinLED();
	//InitInput();
	
	while (1)
	{
		int i;
//		if (!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
//		{
//			GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
//			Delay_Decrement(2000000);
//		}
		for (i = 0; i <= n_points; ++i)
		{
			//GPIOB->BSRRH |= GPIO_Pin_3; // set PB3 (CS) low
			//SPI1_send(sin_data[i]); // transmit data
			//Delay_Decrement(100);
			//GPIOB->BSRRL |= GPIO_Pin_3; // set PB3 (CS) high
			//Delay_Decrement(100);
		}
		//Delay_Decrement(100000);
	}
}

void TIM2_IRQHandler()
{
		static uint16_t index;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
				GPIOB->BSRRH |= GPIO_Pin_3; // set PB3 (CS) low
		//Delay_Decrement(100);
			  SPI1_send(sin_data[index++]); // transmit data
				//if (index >= n_points) index = 0;
				GPIOB->BSRRL |= GPIO_Pin_3; // set PB3 (CS) high
				//GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
        //GPIO_ToggleBits(GPIOB, GPIO_Pin_3);
    }
}
