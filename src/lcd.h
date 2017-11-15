
#ifndef LCD_H_
#define LCD_H_

#include <stdio.h>
#include "diag/Trace.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include "cmsis/cmsis_device.h"


// LCD
#define LCD_EN (0x80)
#define LCD_DIS (0x0)
#define LCD_CMD (0x0)
#define LCD_CHAR (0x40)

#define LCD_MAX_DELAY ((uint32_t)96000)

#define GPIO_LCK_PIN (GPIO_Pin_4)

// TIM3
#define myTIM3_PRESCALE ((uint16_t) 0x0000)
#define myTIM3_PERIOD_DEFAULT (100)
#define MAX_DELAY ((uint32_t) 96000)
#define myTIM3_PERIOD ((uint32_t)0xFFFFFFFF) /* Maximum possible setting for overflow */

/*
 * Initialize the LCD display
 */
void LCD_Init(void);

void mySPI_Init();

void myGPIOB_Init();

void myTIM3_Init();

void Delay(uint32_t time);

void SPI_Write(uint8_t data);

void LCD_Word(uint8_t type, uint8_t data);

void LCD_Command(uint8_t data);

void LCD_Char(char *ch);

#endif /* LCD_H_ */
