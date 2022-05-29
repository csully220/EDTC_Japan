/*
 * USB Tutorial
 * 
 * Copyright (c) 2020 Manuel Bleichenbacher
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 * 
 * Commmon functions
 */

#ifndef COMMON_H
#define COMMON_H

#include <libopencmsis/core_cm3.h>

/**
 * @brief Initializes systick services
 */
void systick_init();

/**
 * @brief Gets the time.
 * 
 * @return number of milliseconds since a fixed time in the past
 */
uint32_t millis();

/**
 * @brief Delays execution (busy wait)
 * @param ms delay length, in milliseconds
 */
void delay(uint32_t ms);


// maps integer x from domain [a1,a2] onto range [b1,b2]
// where a2 > a1 and b2 > b1
// and a1 > 0 and b1 > 0
// returns -1 if fail
/*
int csdf_map(int x, int a1, int a2, int b1, int b2) {
    if(x < a1 || x > a2) return -1;
    if(a1 > a2 || b1 > b2) return -1;
    if(a1 < 0 || b1 < 0) return -1;

    int x1 = (x - a1) * (b2 - b1);
    float x2 = (float)b1 + ((float)(x1)/(float)(a2 - a1));
    return (int)x2;
};
*/


#endif