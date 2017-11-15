#include "lcd.h"

void myLCD_Init() {
	trace_printf("Initing LCD\n");

	myGPIOB_Init();
	mySPI_Init();
	myTIM3_Init();

	Delay(5000);
//	Delay((uint32_t)48000000);
//	Delay((uint32_t)48000000);
//	Delay((uint32_t)48000000);
//	Delay((uint32_t)48000000);

	trace_printf("After 5s\n");

	LCD_Command(ENABLE);

	SPI_Write(0x02);
	SPI_Write(0x82);
	SPI_Write(0x02);

	LCD_Command(0x28);

	LCD_Command(0x01);

	char ch = 'J';
	LCD_Char(&ch);
}

// Enable SPI
void mySPI_Init() {
	trace_printf("Initing SPI\n");

	// Enable SPI Clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

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
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIO_InitTypeDef GPIO_InitStructInfo;
	GPIO_InitTypeDef* GPIO_InitStruct = &GPIO_InitStructInfo;

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

// Enable the TIM3 Timer
// This is used to wait for some LCD operations to complete
// e.g. clearing the display
void myTIM3_Init() {

	// Enable the clock for TIM3
//	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//
//	// Settings for TIM3
//	// 	bit 7: 		Auto reload preload enable = 1
//	// 	bit 6-5: 	Center-aligned mode selection = 00
//	// 	bit 4:		Direction = 1 - downcounter
//	//	bit 3:		One-pulse mode = 1 - stops counting at the next update event
//	//	bit 2:		Update request source = 1 - only counter overflow generates an update interrupt
//	// 	bit 1:		Update disable = 0 - UEV enabled. The UEV event is generated
//	//	bit 0:		Counter enable = 0 - counter disabled
//	TIM3->CR1 = ((uint16_t) 0x08); // 0x98
//
//	// Set clock prescaler value
//	TIM3->PSC = myTIM3_PRESCALE;
//
//	// Set auto-reloaded delay
//	TIM3->ARR = myTIM3_PERIOD;
//
////	TIM3->EGR = (TIM3->EGR & ~0x5F) | (0x1 & 0x5F);
//
////	TIM3->CNT = LCD_MAX_DELAY;
////	TIM3->CR1 |= 0x1;
//
//	// Set delay value
////	TIM3->CNT = LCD_MAX_DELAY;
//
//	// Update timer registers
//	TIM3->EGR |= 0x0001;

//	TIM3->CR1 = ((uint16_t) 0x9C);
//
//	TIM3->PSC = myTIM3_PRESCALE;
//	TIM3->ARR = myTIM3_PERIOD_DEFAULT;
//
//	TIM3->EGR |= 0x0001;

	TIM3->CR1 = 0xC6;

	TIM3->PSC = myTIM3_PRESCALE;
	TIM3->ARR = myTIM3_PERIOD;

//	TIM3->EGR = (TIM3->EGR & ~0x5F) | (0x1 &0x5F);
	TIM3->EGR |= 0x0001;

	TIM3->CNT = MAX_DELAY;
//	TIM3->CR1 |= 0x1;
}

/*
 * Wait for a delay in ms
 */
void Delay(uint32_t time) {
//	// Clear counter register
////	TIM3->CNT = (uint32_t)0x0;
//
//	// Set the amount of time to delay for
//	TIM3->ARR = time;
//	TIM3->CNT = time;
////	TIM3->CNT = LCD_MAX_DELAY;
//
//	// Update timer registers
//	TIM3->EGR |= 0x0001;
//
//	// Clear interrupt flag
////	TIM3->SR &= ~(TIM_SR_UIF);
//
//	// Start timer
//	TIM3->CR1 |= TIM_CR1_CEN;
//
//	// Poll timer until timer expires
//	while((TIM3->CR1 & TIM_CR1_CEN) != 0) {
//		trace_printf("%d\n", TIM3->CNT);
//	}
////	while((TIM3->SR & TIM_SR_UIF) != 0) {
////		trace_printf("%d\n", TIM3->CNT);
////	}
//	trace_printf("Fuck\n\n");
//
//	// Stop timer
//	TIM3->CR1 &= ~(TIM_CR1_CEN);
//
//	// Clear interrupt flag
//	TIM3->SR &= ~(TIM_SR_UIF);

//	TIM3->CNT = time;
//	trace_printf("%d\n", TIM3->CNT);
//
//	TIM3->ARR = time;
//
//	TIM3->EGR |= 0x0001;
//
//	TIM3->CR1 |= TIM_CR1_CEN;
//
//	trace_printf("%d\n", TIM3->CNT);
//	while(!(TIM3->SR & TIM_SR_UIF)) {
////		trace_printf("%d\n", TIM3->CNT);
//	}
//
//	TIM3->CR1 &= ~(TIM_CR1_CEN);
//
//	TIM3->SR &= ~(TIM_SR_UIF);

	TIM3->CNT = MAX_DELAY;

	TIM3->CR1 |= TIM_CR1_CEN;

	while(TIM3->CNT > ((MAX_DELAY+1) - time)) {
		trace_printf("%d\n", TIM3->CNT);
	}

	// Stop timer
	TIM3->CR1 &= ~(TIM_CR1_CEN);
}

void SPI_Write(uint8_t data) {
	// Do not update register output
	GPIOB->BRR = GPIO_LCK_PIN;

	// Wait until SPI is ready to send
	// 	TXE = 1 or BSY = 0
	while((SPI1->SR & SPI_SR_BSY) && ~(SPI1->SR & SPI_SR_TXE));

	// Send the data
	SPI_SendData8(SPI1, data);

	// Wait until SPI has sent everything
	//	BSY = 0
	while(SPI1->SR & SPI_SR_BSY);

	GPIOB->BSRR = GPIO_LCK_PIN;
}

void LCD_Word(uint8_t type, uint8_t data) {
	/*
	 * We need to send the data in 2 parts, high and low.
	 * This is because the 4 MSBs are reserved for EN and RS registers
	 *
	 * We send each half of the word and disable -> enable -> disable lcd
	 * 	so the word is picked up
	 */

	uint8_t high_data = ((data & 0xF0) >> 4);
	uint8_t low_data = data & 0xF;

	SPI_Write(LCD_DIS 	| type | high_data);
	SPI_Write(LCD_EN 	| type | high_data);
	SPI_Write(LCD_DIS 	| type | high_data);

	SPI_Write(LCD_DIS 	| type | low_data);
	SPI_Write(LCD_EN 	| type | low_data);
	SPI_Write(LCD_DIS 	| type | low_data);
}

void LCD_Char(char *ch) {
	LCD_Word(LCD_CHAR, (uint8_t)(*ch));
}

void LCD_Command(uint8_t data) {
	LCD_Word(LCD_CMD, data);
}

