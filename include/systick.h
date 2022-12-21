#pragma once
#include <stdint.h>

/**
 * Enable the systick timer
 * @param reload_val 24-bit value to reload timer to after countdown complete
 * @param clksource 1 for core clock, 0 for reference (vendor) clock
 * @param tickint whether a SysTick interrupt should be requested on countdown
 * @param en 1 to enable the SysTick timer, 0 to disable.
 * @return 0 on success, -1 if vendor clock was requested and is not present.
 */
int systick_init(uint32_t reload_val, int clksource, int tickint, int en);

/**
 * Initialize the systick timer to a 10ms period
 * @param clksource 1 for core clock 0 for reference (vendor) clock
 * @param tickint whether to signal an interrupt when the counter reaches zero
 * @param en 1 to enable the SysTick timer, 0 to disable
 * @return 0 on success, 1 if the calibration is inexact, -1 if the
 * calibration value is not known, and -2 if the vendor clock was requested and
 * is not present.
 */
int systick_try_tenms(int clksource, int tickint, int en);
