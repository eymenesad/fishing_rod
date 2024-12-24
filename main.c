#include <stdint.h>

// Structs (From PS)
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	uint32_t reserved1;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	uint32_t reserved2[2];
	volatile uint32_t BDTR;
}TIM15_General;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	uint32_t reserved;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
} TIM;

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	uint32_t reserved;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	uint32_t reserved1[3];
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
} TIMxBasicType;

typedef struct{
	volatile uint32_t ISR;
	volatile uint32_t IER;
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CFG2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	uint32_t reserved2;
	volatile uint32_t TR1;
	volatile uint32_t TR2;
	volatile uint32_t TR3;
	uint32_t reserved3;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t SQR4;
	volatile uint32_t DR;
	uint32_t reserved4[2];
	volatile uint32_t JSQR;
	uint32_t reserved5[4];
	volatile uint32_t OFR1;
	volatile uint32_t OFR2;
	volatile uint32_t OFR3;
	volatile uint32_t OFR4;
	uint32_t reserved6[4];
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
	uint32_t reserved7[4];
	volatile uint32_t AWD2CR;
	volatile uint32_t AWD3CR;
	uint32_t reserved8[2];
	volatile uint32_t DIFSEL;
	volatile uint32_t CALFACT;
} ADCType;

typedef struct{
	volatile uint32_t CSR;
	uint32_t reserved1;
	volatile uint32_t CCR;
	volatile uint32_t CDR;
} ADCCommon;

typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t STH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	volatile uint32_t GTPR;
	volatile uint32_t RTOR;
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
	volatile uint32_t PRESC;
}USARTType;

typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t BRR;
	uint32_t reserved[2];
	volatile uint32_t RQR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t RDR;
	volatile uint32_t TDR;
	volatile uint32_t PRESC;
}LPUARTType;



// Macros

volatile uint16_t next_head = 0;
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)		// Bus for GPIO (From PS)
#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)		// Bus for TIM15 (From PS)
#define RCC_APB1ENR1 *((volatile uint32_t *) (0x40021000 + 0x58))
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)

#define ISER2 *((volatile uint32_t *) 0xE000E108)			// Interrupt enable (From PS)
#define ISER1 *((volatile uint32_t *) 0xE000E104)

#define USART2 ((USARTType *)0x40004400)
#define LPUART ((LPUARTType *) 0x40008000)

#define ADC1 ((ADCType *) 0x42028000)
#define ADC ((ADCCommon *) 0x42028300)

#define GPIOA ((GPIO *) 0x42020000)							// GPIO pin A (From PS)
#define TIM15 ((TIM15_General *) 0x40014000)				// TIM15 pin (From PS)
#define GPIOB ((GPIO *) 0x42020400)
#define TIM2 ((TIM *) 0x40000000)
#define TIM6 ((TIMxBasicType *) 0x40001000)
#define TIM7 ((TIMxBasicType *) 0x40001400)
#define GPIOD ((GPIO *) 0x42020C00)

// Globals
int vibration_count = 0;		// Storing the vibration count for IC
int cycle_count = 0;			// Storing the cycle count for seeing the effect of OC
int cycle_count_strong = 0;
//int two_cycle = 0;				// Counting the vibrations for two cycles
int tempo = 0;					// Storing the value of duty cycle for OC
int casting_cycle_count = 0;
int clocks_enabled=1;
int lux_value = 0;
int buzzer_tempo = 0;
#define TX_BUFFER_SIZE 64
volatile char tx_buffer[TX_BUFFER_SIZE];  // Circular buffer
volatile uint16_t tx_head = 0;
volatile uint16_t tx_tail = 0;
volatile uint8_t tx_busy = 0;  // 1 if transmitting, 0 if idle

// Function vibration
void vibration_handler(void);	// The function will be implemented in next steps.

// IRQ
void TIM15_IRQHandler(void){	// Interrupt handler for TIM15 (From PS)
	if(TIM15->SR & (1 << 1)){		// Input interrupt handling
		TIM15->SR &= ~(1 << 1);		// Reset the value to zero
		vibration_count++;			// Increase vibration counter
	}

	if(TIM15->SR & (1 << 0)){		// Timer overflow interrupt handling (From PS)
		TIM15->SR &= ~(1 << 0);		// Reset the value to zero (From PS)
//		two_cycle++;				// Increment two cycle variable
//		if(two_cycle >= 2){			// If two, handle vibration results
//			two_cycle = 0;			// Reset it to zero for next vibration count
			vibration_handler();	// Handle vibration
//		}
	}
}

void TIM6_IRQHandler(void) // NEW! ISR for TIM6
{
	if(TIM6->SR & (1 << 0)){
		TIM6->SR=0;
		casting_cycle_count++;
		if(casting_cycle_count <= 3){
			TIM2->CCR3 = tempo;
		}
		else{
			TIM2->CCR3 = 0;
			casting_cycle_count = 0;
		}
	}
}

void TIM7_IRQHandler(void) // NEW! ISR for TIM7
{
	if(TIM7->SR & (1 << 0)){
		TIM7->SR &= ~(1<<0);


		//ADC1->CR &= ~(1 << 29); //Powering ADC Module
		//ADC1->CR |= (1 << 0);

		ADC1->CR |= (1<<0);
		while((ADC1->ISR & 1) == 0) {}

		ADC1->CR |= (1 << 2);
	}
}

// GPIO
void init_GPIO_ICOC(void){
	RCC_AHB2ENR |= 1;			// Bus enable for GPIO A (From PS)

	GPIOA->MODER &= ~(0b1111 << (2 * 2));		// Reset PA2 and PA3 to zero (From PS)
	GPIOA->MODER |= (0b1010 << (2 * 2));		// Mode PA2 and PA3 to alternate function (From PS)
	// INPUT = PA2, OUTPUT = PA3
	GPIOA->AFRL &= ~(0b11111111 << (4 * 2));	// Clear alternate function bits for PA2, PA3 (From PS)
	GPIOA->AFRL |= (0b11101110 << (4 * 2));		// Set alternate function configuration for PA2, PA3 (From PS)
}

void init_GPIO_SERVO(void){
	// Turn on the GPIOB port.
	RCC_AHB2ENR |= (1 << 1);

	GPIOB->MODER &= ~(0x03 << (10 * 2));   	// PB10 clear bits
	GPIOB->MODER |= (0x02 << (10 * 2));    	// Set PB10 as alternate function
}

void init_GPIO_USART(void) {
	RCC_AHB2ENR |= (1 << 3);

   // Configure PA2 and PA3 as alternate function for USART2
	GPIOD->MODER &= ~(3 << (5 * 2)); // Clear mode for PD5
	GPIOD->MODER |= (2 << (5 * 2));  // Set PD5 to alternate function
	GPIOD->MODER &= ~(3 << (6 * 2)); // Clear mode for PD6
	GPIOD->MODER |= (2 << (6 * 2));  // Set PD6 to alternate function

}

void init_GPIO_ADC(void){ //Initilazing GPIO
	//Enabling clock for GPIOA
	RCC_AHB2ENR |= 1;
	RCC_AHB2ENR |= 1 << 1;
	GPIOA->MODER |= 0b11 << 2; //Setting PA1 as analog mode
	RCC_AHB2ENR |= 1 << 13; //Enabling the ADC clock

	GPIOA->MODER &= ~(1 << 17); //PA8 is output
}

void init_ADC(void) //REFERENCE: PS10 Slide
{
	ADC1->CR &= ~(1 << 29); //Powering ADC Module
	ADC1->CR |= (1 << 28); //Enabling ADC Voltage Regulator
	RCC_CCIPR1 |= 3 << 28; //main ADC clock is system clock
	ADC->CCR |= 3 << 16; //Selecting ADC Clock mode (1Mhz)
	ADC1->SMPR1 |= 0b111 << 18; //sampling time is 640.5 ADC12_IN6 clock cycles to decrease sampling error
	ADC1->SQR1 &= ~(0b1111 << 0); // Number of conversion is 1
	ADC1->SQR1 |= 6 << 6; // 1st conversion is ADC12_IN6
	ADC1->CR |= (1 << 31); //calibrate ADC
	while((ADC1->CR & (1 << 31)) != 0) {

	}
	//Wait until Calibration
	ADC1->CR |= 1; //turn on ADC module
	while((ADC1->ISR & 1) == 0) {

	}
	//Waiting until ADC is enabled
	//ADC1->CR |= 1 << 2; //Starting Conversion
	ADC1->IER |= 1 << 2; //turn on EOC interrupts
	ISER1 |= 1 << 5;//Enabling global signals for ADC Interrupt
}

void disable_ADC(void) {
    ADC1->CR &= ~(1 << 0);            //Disable ADC module
    ADC1->CR |= (1 << 29);     //Enable deep power-down mode
}
void light_handler(void){
	if(lux_value > 400){
		GPIOA->ODR &= ~(1 << 8);
	}
	else{
		GPIOA->ODR |= (1 << 8);
	}
}
void ADC1_2_IRQHandler(void)	//REFERENCE: PS 10 Slide
{
	if((ADC1->ISR & 1<<2) != 0)
	{
		lux_value = ADC1->DR; //Reading light intensity value
		//ADC1->CR |= 1<<2; //Starting conversion
		light_handler();
		ADC1->CR &=~(1<<0);
				//disable_ADC();         //Disable ADC
	}
}

void init_USART(void) {
	RCC_CCIPR1 &= ~(1 << 3);
	RCC_CCIPR1 |= 1 << 2;
	// Enable GPIOA clock for USART2 pins (PD5 - TX, PD6 - RX)
	RCC_APB1ENR1 |= (1 << 17); // Enable USART2 clock

	// Set alternate function 7 (AF7) for PD5 and PD6
	GPIOD->AFRL |= (7 << (5 * 4)); // Set PD5 (TX) to AF7
	GPIOD->AFRL |= (7 << (6 * 4)); // Set PD6 (RX) to AF7

	// Configure USART2
	USART2->BRR = 0x1A1;            // Set baud rate to 9600 for 4 MHz clock
	USART2->CR1 |= (1 << 2) | (1 << 3); // Enable RX and TX
	USART2->CR1 |= (1 << 29);        // Enable FIFO
	USART2->CR1 |= (1 << 5);        // Enable RXNE interrupt
	//USART2->CR1 |= (1 << 7);        // Enable TXNE interrupt
	USART2->CR1 |= (1 << 0);        // Enable USART

	// Enable USART2 interrupt in NVIC
	ISER1 |= (1 << 30);
}

void uart_send_char(char c) {
   //while (!(USART2->ISR & (1 << 7))); // Wait until TXE is set
   //USART2->TDR = c;
	 if (tx_head < TX_BUFFER_SIZE) {
	        tx_buffer[tx_head++] = c;

	        // If USART is idle, enable TXE interrupt to start sending
	        if (!tx_busy) {
	            tx_busy = 1;
	            USART2->CR1 |= (1 << 7);  // Enable TXE interrupt
	        }
	    }
}

void uart_send_string(const char *str) {
    while (*str) {
        uart_send_char(*str++); // Send each character in the string
    }
}

void enable_clocks(void) {
    if(!clocks_enabled){
    	TIM15->CR1 |= 1;
    	TIM6->CR1 |= 1;
    }
}
void disable_clocks(void) {
	if(clocks_enabled){
	    	TIM15->CR1 &= ~(1<<0);
	    	TIM6->CR1 &= ~(1<<0);
	    }

}

void USART2_IRQHandler(void) {
	if ((USART2->ISR & (1 << 7)) && (tx_head != tx_tail)) {
		USART2->TDR = tx_buffer[tx_tail++];

		        // If newline '\n' is sent, reset the buffer
		        if (tx_buffer[tx_tail - 1] == '\n') {
		            tx_head = 0;
		            tx_tail = 0;
		            tx_busy = 0;
		            USART2->CR1 &= ~(1 << 7);  // Disable TXE interrupt
		        }
	    }

    if (USART2->ISR & (1 << 5)) { // RXNE (Receive Not Empty)
        char received_char = USART2->RDR; // Read received character

        if (received_char == 'L') {
			uart_send_string("LEFT\r\n");
			TIM15->DIER &= ~(0b11);
			disable_clocks();
			clocks_enabled=0;
			vibration_count=0;
			cycle_count=0;
			TIM15->CCR2=0;
			TIM2->CCR3 = 20;

        } else if(received_char == 'R') {
        	uart_send_string("RIGHT\r\n");
			TIM15->DIER &= ~(0b11);
        	disable_clocks();
        	clocks_enabled=0;
			vibration_count=0;
			cycle_count=0;
			TIM15->CCR2=0;


        	TIM2->CCR3 = 85;
        }
        else if(received_char == 'S'){
        	uart_send_string("STOPPED\r\n");
        	TIM2->CCR3 = 10;
			TIM15->DIER &= ~(0b11);
        	disable_clocks();
			vibration_count=0;
			cycle_count=0;
			TIM15->CCR2=0;

        	clocks_enabled=0;
        }
        else if(received_char == 'F'){
			uart_send_string("FISHING ");
			uart_send_string("MODE ");
			uart_send_string("ON\r\n");


			TIM2->CCR3 = 10;
			TIM15->CCR2=0;
			TIM15->DIER |= (0b11);


			enable_clocks();
			clocks_enabled=1;
		}

        else{
        	uart_send_string("INVALID\r\n");
        }
	}
}

// Timer
void init_TIMER15_ICOC(void){
	RCC_APB2ENR |= (1 << 16);			// Enable TIM15 clock (From PS) (From PS)

	TIM15->PSC = 3999;					// Set prescaler to divide clock frequency (From PS)
	TIM15->ARR = 499;					// Set auto-reload value for desired timer period (From PS)

	TIM15->CCMR1 &= ~(0b111 << 12);		// Reset output compare mode for channel 2 (From PS)
	TIM15->CCMR1 |= (0b110 << 12);		// Configure PWM mode 1 for channel 2 (From PS)
	TIM15->CCER |= (1 << 4);			// Enable output for channel 2 (From PS)
	TIM15->CCR2 = 2;					// Set initial duty cycle for channel 2 (From PS)

	TIM15->CCMR1 &= ~(0b11);			// Reset input capture settings for channel 1 (From PS)
	TIM15->CCMR1 |= (0b01);				// Configure input capture for channel 1 (From PS)
	TIM15->CCER |= (0b1011);			// Enable input and output settings for channels (From PS)

	TIM15->DIER |= (0b11);				// Enable interrupts for input capture and timer overflow (From PS)

	TIM15->BDTR |= (1 << 15);			// Enable break and dead-time configuration (From PS)

	TIM15->CR1 |= 1;					// Start the timer (From PS)
}

void init_TIMER2_SERVO() {

    RCC_APB1ENR1 |= 0x01;        // Turn on Timer2

    GPIOB->AFRH &= ~(0x0F << ((10 - 8) * 4)); // AFRH reset
    GPIOB->AFRH |= (0x01 << ((10 - 8) * 4)); // AF1(TIM2_CH3) selection for PB10

    TIM2->PSC = 110 - 1;		// Reduce 110 MHz system frequency to 1 MHz
    TIM2->ARR = 200 - 1;		// Set 20 ms period (50 Hz)

    TIM2->CCMR2 &= ~(0x07 << 4);      // CCMR2 Clear bits
    TIM2->CCMR2 |= (0x06 << 4);       // PWM mode 1 selection for CH3
    TIM2->CCER |= (0x01 << 8);           // Enable CH3 output

    TIM2->CR1 |= 0x01; // Start timer2
}

void init_TIMER6_CASTING(void)
{
	 RCC_APB1ENR1 |= 1 << 4; //TIM6x_CLK is enabled, running at 4MHz
	 TIM6->PSC = 3999; //Set Prescaler
	 TIM6->ARR = 999; //Set Delay
	 TIM6->CR1 &= ~(1<<1); //OVF will generate an event
	 //
	 TIM6->CR1 |= (1<<7); //Enable autoreload of ARR register
	 TIM6->DIER |= 1; //NEW! enable UIF to generate an interrupt
	 ISER1 |= 1 << 17;//NEW! enable global signaling for TIM6 interrupt
	 // ISER1 position is inferred from page 530 in rm0438 PDF
}

void init_TIMER7_ADC(void)
{
	 RCC_APB1ENR1 |= 1 << 5; //TIM6x_CLK is enabled, running at 4MHz
	 TIM7->PSC = 3999; //Set Prescaler
	 TIM7->ARR = 4999;//Set Delay
	 TIM7->CR1 &= ~(1<<1); //OVF will generate an event
	 //
	 TIM7->CR1 |= (1<<7); //Enable autoreload of ARR register
	 TIM7->DIER |= 1; //NEW! enable UIF to generate an interrupt
	 ISER1 |= 1 << 18;//NEW! enable global signaling for TIM6 interrupt
	 // ISER1 position is inferred from page 530 in rm0438 PDF
	 TIM7->CR1 |= 1;
}

// Interrupt
void init_INTERRUPT(void){
	ISER2 |= (1 << 5);				// Enable TIM15 interrupt in NVIC (From PS)
	__asm volatile(
				"mov r0, #0 \n\t"
				"msr primask, r0 \n\t"
	);
}

// Vibration
void vibration_handler(void){
	if(vibration_count > 130){		// High-intensity vibration detected
		uart_send_string("FISH DETECTED\r\n");
		TIM15->DIER &= ~(0b11);
//		vibration_count = 0;
		tempo = 85;					// Set high duty cycle. Maximum strength for motor and buzzer.
		TIM6->CR1 |= 1; 			//TIM6_CNT is enabled (clocked)
		TIM15->CR1 &= ~(1 << 0);
		buzzer_tempo = 0;
	}
	else if(vibration_count > 50){
		uart_send_string("STRONG VIBRATIONS PRESENT\r\n");
		buzzer_tempo = 499;
	}

	else if(vibration_count > 10){
		uart_send_string("WEAK VIBRATIONS PRESENT\r\n");
		buzzer_tempo = 99;
	}

	else{
		tempo = 0;					// No vibration detected, set duty cycle to 0. Deactivate buzzer, deactivate motor.
		buzzer_tempo = 0;
	}
	vibration_count = 0;
	TIM2->CCR3 = tempo;
	TIM15->CCR2 = buzzer_tempo;
}



// Main
int main(void){
	init_GPIO_ICOC();			// Initialize GPIO
	init_TIMER15_ICOC();			// Initialize Timer

	init_GPIO_SERVO();
	init_TIMER2_SERVO();

	init_TIMER6_CASTING();

	init_GPIO_USART();
	init_USART();

	init_GPIO_ADC();
	init_ADC();
	init_TIMER7_ADC();

	init_INTERRUPT();// Initialize Interrupts

	while(1){
		__asm volatile("wfi");		// Wait for interrupt (low-power mode) (From PS)
	}
	return 0;		// Program should never reach here
}