/*
 * ili9325.h
 *
 *  Created on: 24 апр. 2019 г.
 *      Author: Murad
 */

#ifndef ILI9325_H_
#define ILI9325_H_
#include "stm32h7xx_hal.h"
#include <stdlib.h>
//#include "fatfs.h"



#define ADDR_CMD        *(uint8_t*)0x60000000

#define ADDR_DATA        *(uint8_t*)0x60010000







#define ADDR_DATA        *(uint8_t*)0x60010000



#define swap(a,b) {int16_t t=a;a=b;b=t;}

#define  RESET_ACTIVE   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

#define  RESET_IDLE   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);



#define  BLACK 0x0000

#define  BLUE 0x001F

#define  RED 0x0F800

#define  GREEN 0x07E0

#define  CYAN 0x07FF

#define  MAGENTA 0xF81F

#define  YELLOW 0xFFE0

#define  WHITE 0xFFFF


#endif /* ILI9325_H_ */
