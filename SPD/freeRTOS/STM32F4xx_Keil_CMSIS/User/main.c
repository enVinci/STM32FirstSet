//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "semphr.h"
#include "stdio.h"
//******************************************************************************
#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/

typedef struct
{
	char data[16];
}message_t;

void vLedBlink(void *pvParameters);
void peripheralInit(void);
void Czytelnik(void *pvParameters);
void Pisarz1(void *pvParameters);
void Pisarz2(void *pvParameters);
void USART(void *pvParameters);
void send_char(char c);
void send_string(const char* s);

QueueHandle_t xQueue;
xSemaphoreHandle xSemaphore;
xSemaphoreHandle xSemaphoreQueue;

//******************************************************************************
int main(void)
{ 
	peripheralInit();
	xSemaphore = xSemaphoreCreateBinary(); 
	xSemaphoreGive(xSemaphore);
	xSemaphoreQueue = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphoreQueue);
	xQueue = xQueueCreate(128 / sizeof(message_t), sizeof(message_t));
	if(xQueue == NULL) return 0;
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	xTaskCreate( vLedBlink, (const signed char*)"Led Blink Task", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( Czytelnik, (const signed char*)"Czytelnik Task", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( USART, (const signed char*)"Send USART Task", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( Pisarz1, (const signed char*)"Pisarz1 Task", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( Pisarz2, (const signed char*)"Pisarz2 Task", STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	
	vTaskStartScheduler();
	return 0;
}

//******************************************************************************
void vApplicationIdleHook( void )
{}
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{}
extern void vApplicationMallocFailedHook( void )
{}	
void vApplicationTickHook( void )
{}	
//******************************************************************************	

void peripheralInit(void) {
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef USART_InitStruct;
	
//	// PLL 100MHz
//	RCC_DeInit();

//	RCC_HSEConfig(RCC_HSE_ON);
//	while(RCC_WaitForHSEStartUp() != SUCCESS);
//                                                                                                                                                                                                                              
//	RCC_PLLConfig(RCC_PLLSource_HSE, 4, 100, 2, 2);
//	RCC_HCLKConfig(RCC_SYSCLK_Div1);
//	RCC_PCLK1Config(RCC_HCLK_Div2);
//	RCC_PCLK2Config(RCC_HCLK_Div1);
//	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//	RCC_PLLCmd(ENABLE);
//	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);

	// Clocks setup
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	// PIN portu A dla LED:
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	// Piny portu A do UART2:
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);
	
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);
}
//******************************************************************************
//Task definition
void vLedBlink(void *pvParameters)
{
	for(;;)
	{
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void Czytelnik(void *pvParameters)
{
	while(1)
	{
		vTaskDelay( 500 / portTICK_RATE_MS );
		xSemaphoreGive(xSemaphore);
	}
}

void Pisarz1(void *pvParameters)
{
  uint16_t number = 0;
	message_t text;
	while(1)
	{
		xSemaphoreTake(xSemaphoreQueue, portMAX_DELAY);
		sprintf(text.data, "Jestem 1: %d\n\r", number);
		if (xQueueSend(xQueue, &text, portMAX_DELAY) == pdTRUE)
			if (++number > 999) number = 0;
		xSemaphoreGive(xSemaphoreQueue);
		vTaskDelay( 300 / portTICK_RATE_MS );
	}
}

void Pisarz2(void *pvParameters)
{
  uint16_t number = 0;
	message_t text;
	while(1)
	{
		xSemaphoreTake(xSemaphoreQueue, portMAX_DELAY);
		sprintf(text.data, "Jestem 2: %d\n\r", number);
		if (xQueueSend(xQueue, &text, portMAX_DELAY) == pdTRUE)
			if (++number > 999) number = 0;
		xSemaphoreGive(xSemaphoreQueue);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}

void USART(void *pvParameters)
{
	message_t textToSend;
	while(1)
	{	
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		if(xQueueReceive(xQueue, &textToSend, portMAX_DELAY) == pdTRUE)
		send_string(textToSend.data);
	}
}

void send_char(char c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}
	
void send_string(const char* s)
{
    while (*s)
        send_char(*s++);
}

void push_back(const char* text)
{
	while(*text)
		if(xQueueSend(xQueue, text,portMAX_DELAY) == pdTRUE)
			text++;
}