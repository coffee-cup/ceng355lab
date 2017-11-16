
#ifndef LCD_H_
#define LCD_H_

#include "cmsis/cmsis_device.h"
#include "diag/Trace.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_spi.h"
#include <stdio.h>

// LCD
#define LCD_EN (0x80)
#define LCD_DIS (0x00)
#define LCD_CMD (0x00)
#define LCD_CHAR (0x40)

#define LCD_CLEAR_CMD (0x01)
#define LCD_RETURN_HOME (0x02)
#define LCD_FIRST_LINE (0x80)
#define LCD_SECOND_LINE (0xC0)

#define GPIO_LCK_PIN (GPIO_Pin_4)

// TIM6
#define myTIM6_PRESCALE ((uint16_t)((SystemCoreClock - 1) / 1000)) // 1ms per tick
#define myTIM6_PERIOD_DEFAULT (100) // 100 ms

/*
 * Initialize the LCD display
 */
void LCD_Init(void);

/*
 * Initialize SPI
 */
void mySPI_Init();

/*
 * Initialize the GPIOB pins
 */
void myGPIOB_Init();

/*
 * Initialize counter TIM6
 */
void myTIM6_Init();

/*
 * Wait for time in ms
 */
void Delay(uint32_t time);

/*
 * Write data over SPI
 */
void SPI_Write(uint8_t data);

/*
 * Write data to the LCD
 */
void LCD_Data(uint8_t type, uint8_t data);

/*
 * Send a command to the LCD
 */
void LCD_Command(uint8_t data);

/*
 * Send a string to the LCD
 */
void LCD_Word(char *s);

/*
 * Send a character to the LCD
 */
void LCD_Char(char ch);

/*
 * Clear the LCD
 */
void LCD_Clear();

/*
 * Write first and second lines to LCD
 */
void Write_Lines(char *first_line, char *second_line);

#endif /* LCD_H_ */
