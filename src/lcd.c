#include "lcd.h"

void myLCD_Init() {
  trace_printf("Initing LCD\n");

  myGPIOB_Init();
  myTIM6_Init();
  mySPI_Init();

  /*
   * If LCD is in 8 bit mode, this will set it in 4 bit mode
   * If LCD is in 4 bit mode, this will send the cursor home (top left)
   */
  LCD_Command(LCD_RETURN_HOME);
  Delay(2); // To account for 1.52ms execution time

  // Configure the display
  LCD_Command(0x28); // 2 lines of 8 characters
  LCD_Command(0x0C); // Cursor not displayed
  LCD_Command(0x06); // Auto increment DDRAM address after each char read

  LCD_Clear();

  Write_Lines("Hello", "World");
}

// Enable SPI
void mySPI_Init() {
  trace_printf("Initing SPI\n");

  // Enable SPI Clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI_InitTypeDef SPI_InitStructInfo;
  SPI_InitTypeDef *SPI_InitStruct = &SPI_InitStructInfo;

  SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct->SPI_CRCPolynomial = 7;

  SPI_Init(SPI1, SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);
}

void myGPIOB_Init() {
  trace_printf("Initing GPIOB\n");

  /* Enable clock for GPIOB peripheral */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  GPIO_InitTypeDef GPIO_InitStructInfo;
  GPIO_InitTypeDef *GPIO_InitStruct = &GPIO_InitStructInfo;

  // Configure PB3 and PB5 pins
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF; // alternate function mode
  GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct->GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
  GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, GPIO_InitStruct);

  // Configure LCK pin
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_OUT; // output mode
  GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct->GPIO_Pin = GPIO_LCK_PIN;
  GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct->GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, GPIO_InitStruct);
}

// Enable the TIM6 Timer
// This is used to wait for some LCD operations to complete
// e.g. clearing the display
void myTIM6_Init() {

  // Enable the clock for TIM6
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  /*
   * Settings for TIM6
         *
   * bit 7: 	Auto reload preload enable = 1
   * bit 6-5: 	Center-aligned mode selection = 00
   * bit 4:		Direction = 0 - up counter
   * bit 3:		One-pulse mode = 1 - stops counting at next update event
   * bit 2:		Update request source = 1 - only counter overflow
generates an update interrupt
   * bit 1:		Update disable = 0 - UEV enabled. The UEV event is
generated
   * bit 0:		Counter enable = 0 - counter disabled
   */
  //  TIM6->CR1 = ((uint16_t)0x008C);
  TIM6->CR1 = 0x84;

  // Set clock prescaler value
  TIM6->PSC = myTIM6_PRESCALE;

  // Set auto-reloaded delay
  TIM6->ARR = myTIM6_PERIOD_DEFAULT;

  /* Update timer registers */
  TIM6->EGR = ((uint16_t)0x0001);
}

/*
 * Wait for a delay in ms
 */
void Delay(uint32_t time) {
  // Clear timer
  TIM6->CNT = (uint32_t)0x0;

  // Set timeout
  TIM6->ARR = time;

  // Update timer registers
  TIM6->EGR |= 0x0001;

  // Start the timer
  TIM6->CR1 |= TIM_CR1_CEN;

  // Wait until interrupt occurs
  while ((TIM6->SR & TIM_SR_UIF) == 0)
    ;

  // Stop timer
  TIM6->CR1 &= ~(TIM_CR1_CEN);

  // Reset the interrupt flag
  TIM6->SR &= ~(TIM_SR_UIF);
}

void SPI_Write(uint8_t data) {
  // Do not update register output
  GPIOB->BRR = GPIO_LCK_PIN;

  // Wait until SPI is ready to send
  // 	TXE = 1 or BSY = 0
  while ((SPI1->SR & SPI_SR_BSY) && ~(SPI1->SR & SPI_SR_TXE))
    ;

  // Send the data
  SPI_SendData8(SPI1, data);

  // Wait until SPI has sent everything
  //	BSY = 0
  while (SPI1->SR & SPI_SR_BSY)
    ;

  GPIOB->BSRR = GPIO_LCK_PIN;
}

void LCD_Data(uint8_t type, uint8_t data) {
  /*
   * We need to send the data in 2 parts, high and low.
   * This is because the 4 MSBs are reserved for EN and RS registers
   *
   * We send each half of the word and disable -> enable -> disable lcd
   * 	so the word is picked up
   */

  uint8_t high_data = ((data & 0xF0) >> 4);
  uint8_t low_data = data & 0xF;

  SPI_Write(LCD_DIS | type | high_data);
  SPI_Write(LCD_EN | type | high_data);
  SPI_Write(LCD_DIS | type | high_data);

  SPI_Write(LCD_DIS | type | low_data);
  SPI_Write(LCD_EN | type | low_data);
  SPI_Write(LCD_DIS | type | low_data);
}

void LCD_Word(char *s) {
  char *ch = s;
  while (*ch != NULL) {
    LCD_Char(*ch);
    ch++;
  }
}

void LCD_Char(char ch) { LCD_Data(LCD_CHAR, (uint8_t)(ch)); }

void LCD_Command(uint8_t data) { LCD_Data(LCD_CMD, data); }

void LCD_Clear() {
  // Send the clear command
  LCD_Command(LCD_CLEAR_CMD);

  // This command takes time, wait 2ms for it to complete
  Delay(2);
}

void Write_Lines(char *first_line, char *second_line) {
  LCD_Clear();

  LCD_Command(LCD_FIRST_LINE);
  LCD_Word(first_line);

  LCD_Command(LCD_SECOND_LINE);
  LCD_Word(second_line);
}
