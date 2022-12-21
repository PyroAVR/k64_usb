// Hardfault handler and setup functions for error reporting.
#include <MK64F12.h>
#define __GPIO_CLEAR(thing) thing->PCOR
#define __GPIO_SET(thing) thing->PSOR
#define SET_LED(base, which) do { __GPIO_CLEAR(GPIO##base) |= (1 << which); } while(0)
#define CLEAR_LED(base, which) do { __GPIO_SET(GPIO##base) |= (1 << which); } while(0)

#if CLOCK_SETUP == 1
#undef DEFAULT_SYSTEM_CLOCK
#define DEFAULT_SYSTEM_CLOCK  (SystemCoreClock >> 3)
#endif

void LED_Init(void){
	// Enable clocks on Ports B and E for LED timing
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	// Configure the Signal Multiplexer for GPIO
	// Table 6, Chapter 10, K64 User Guide
	PORTB->PCR[22] = PORT_PCR_MUX(1); // RED
	PORTB->PCR[21] = PORT_PCR_MUX(1); // BLUE
	PORTE->PCR[26] = PORT_PCR_MUX(1); // GREEN
	
	// Switch the GPIO pins to output mode
  GPIOB->PDDR = (1 << 22) | (1 << 21);
	GPIOE->PDDR = (1 << 26);

	// Turn off the LEDs
	GPIOB->PDOR = ~0;
	GPIOE->PDOR = ~0;
}


static void block_wait(int millis) {
	while(millis > 0) {
			for(int i = 0; i < DEFAULT_SYSTEM_CLOCK/1000; i++) {};
			millis--;
	}
}


__attribute__((naked))
void HardFault_Handler(void) {
    __asm volatile (
     " movs r0,#4       \n"
     " movs r1, lr      \n"
     " tst r0, r1       \n"
     " beq _MSP         \n"
     " mrs r0, psp      \n"
     " b _HALT          \n"
     "_MSP:               \n"
     " mrs r0, msp      \n"
     "_HALT:              \n"
     " ldr r1,[r0,#24]  \n" // this one is the PC
     // FIXME FINISH HANDLER, SETUP UART/SDHC DUMP
    );
    LED_Init();
    CLEAR_LED(E, 26);
    CLEAR_LED(B, 21);
    SystemCoreClockUpdate();
    for(;;) {
        for(int i = 0; i < 3; i++) {
            SET_LED(B, 22);
            block_wait(100);
            CLEAR_LED(B, 22);
            block_wait(100);
        }
        for(int i = 0; i < 3; i++) {
            SET_LED(B, 22);
            block_wait(300);
            CLEAR_LED(B, 22);
            block_wait(300);
        }
        for(int i = 0; i < 3; i++) {
            SET_LED(B, 22);
            block_wait(100);
            CLEAR_LED(B, 22);
            block_wait(100);
        }
        block_wait(2000);
    }
    asm("b .");
}
