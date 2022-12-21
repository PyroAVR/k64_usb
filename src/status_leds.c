#include <MK64F12.h>
#include <status_leds.h>

/*************
 * FUNCTIONS
 *************/
int status_leds_init() {
    // Enable clocks on Ports B and E for LED timing
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    // Configure the Signal Multiplexer for GPIO
    // Table 6, Chapter 10, K64 User Guide
    PORTB->PCR[22] = PORT_PCR_MUX(1); // RED
    PORTB->PCR[21] = PORT_PCR_MUX(1); // BLUE
    PORTE->PCR[26] = PORT_PCR_MUX(1); // GREEN

    /* set gpios to turn off the leds */
    GPIOB->PSOR |= (3 << 21);
    GPIOE->PSOR |= (1 << 26);

    /* set the GPIO to be outputs */
    GPIOB->PDDR = (1 << 22) | (1 << 21);
    GPIOE->PDDR = (1 << 26);

    return 0;
}

