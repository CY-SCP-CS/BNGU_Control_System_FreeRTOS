#ifndef LIB_PERIODIC_H
#define LIB_PERIODIC_H
#include <stdint.h>
/* All deadlines are less than 2^31 ticks apart. Equal means due, not expired. */
uint32_t lib_periodic_skip_expired(uint32_t now,uint32_t period,uint32_t *next);
#endif
