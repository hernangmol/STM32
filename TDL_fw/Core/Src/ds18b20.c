/*
 * ds18b20.c
 *
 *  Created on: 18 jul. 2024
 *      Author: HGM
 */

#include "ds18b20.h"

extern TIM_HandleTypeDef htim3;

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay_us (480);   // delay according to datasheet
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay_us (80);    // delay according to datasheet
	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN)))
		Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;
	delay_us (400); // 480 us delay totally.
	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us (1);  // wait for 1 us
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay_us (50);  // wait for 60 us
		}
		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay_us (50);  // wait for 60 us
			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read(void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_9, 0);  // pull the data pin LOW
		delay_us (2);  // wait for 2 us
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_9))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us (60);  // wait for 60 us
	}
	return value;
}


float DS18B20_Temp2Float(uint16_t number)
{
	uint16_t aux;
	float result = 0;
	float signo;
	uint16_t mask = 0b1111100000000000;
	// extracción del signo
	if((number & mask) > 0)
		signo = -1;
	else
		signo = 1;
	// calculo de la magnitud
	mask = 0b0000011111111111;
	// parte entera
	if (signo == -1)
	{                     // complemento A2
		aux = ~number & mask;
		aux+=1;
	}
	else
	{
		aux = number & mask;
	}
// parte decimal
	result = 0;
	mask = 0b0000000000000001;
	for (int i=0;i<12;i++)
	{
		 if((aux & mask)>0)
			 result = result + .0625 *pow(2, i);
		 mask = mask << 1;
	}
	return (signo * result);
}
