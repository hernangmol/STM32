/*
 * ds18b20.h
 *
 *  Created on: 18 jul. 2024
 *      Author: HGM
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "stm32f1xx_hal.h"
#include "math.h"

#define DS18B20_PORT GPIOB
#define DS18B20_PIN GPIO_PIN_9

/************************************** Public functions **************************************/
void Set_Pin_Output(GPIO_TypeDef *, uint16_t);
void Set_Pin_Input(GPIO_TypeDef *, uint16_t);
uint8_t DS18B20_Start (void);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t);
uint8_t DS18B20_Read(void);
float DS18B20_Temp2Float(uint16_t);

#endif /* INC_DS18B20_H_ */
