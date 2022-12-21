#include <MK64F12.h>

/**********
 * MACROS
 **********/
#define GREEN_LED_ON()  GPIOE->PCOR |= (1 << 26)
#define GREEN_LED_OFF() GPIOE->PSOR |= (1 << 26)
#define RED_LED_ON()    GPIOB->PCOR |= (1 << 22)
#define RED_LED_OFF()   GPIOB->PSOR |= (1 << 22)

/**************
 * PROTOTYPES
 **************/

/**
 * Enables the Status LEDs located on PTC16, PTC17
 * @return 0
 */
int status_leds_init();
