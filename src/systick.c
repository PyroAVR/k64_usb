#include <systick.h>
#include <stdint.h>
#include <MK64F12.h>

int systick_init(uint32_t reload_val, int clksource, int tickint, int en) {
    int status = 0;
    SysTick->LOAD = reload_val & SysTick_LOAD_RELOAD_Msk;
    SysTick->CTRL |= (clksource << SysTick_CTRL_CLKSOURCE_Pos) |
        (tickint << SysTick_CTRL_TICKINT_Pos) | en;
    if(clksource == 0 && SysTick->CALIB & SysTick_CALIB_NOREF_Msk) {
        status = -1;
    }
    return status;
}

int systick_try_tenms(int clksource, int tickint, int en) {
    int status = 0;
    int reload_val = SysTick->CALIB & SysTick_CALIB_TENMS_Msk;
    if(reload_val == 0) {
        status = -1;
        goto done;
    }
    if(clksource == 0 && SysTick->CALIB & SysTick_CALIB_NOREF_Msk) {
        status = -2;
    }
    if(SysTick->CALIB & SysTick_CALIB_SKEW_Msk) {
        status = 1;
    }
    else {
        systick_init(reload_val, clksource, tickint, en);
    }
done:
    return status;
}
