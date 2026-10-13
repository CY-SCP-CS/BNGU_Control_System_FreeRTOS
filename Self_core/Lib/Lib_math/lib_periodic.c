#include "lib_periodic.h"
uint32_t lib_periodic_skip_expired(uint32_t now,uint32_t period,uint32_t *next) {
    int32_t lateness=(int32_t)(now-*next);
    if(lateness<=0 || !period)return 0;
    uint32_t skipped=(uint32_t)lateness/period+1;
    *next+=skipped*period;
    return skipped;
}
