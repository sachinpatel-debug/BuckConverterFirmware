#pragma once

/**
 * Type 3 Compensator paramater values scaled
 * Scaling is done so that only integer math is used as that helps computation speed tremendously
 */
#define K (+496) //scaling factor defined by the arr register, the resistor divider, and the ADC GAIN
#define COMP_SCALE (100000000)
#define B0 (+072843308 * K)
#define B1 (-68984401 * K)
#define B2 (-72799692 * K)
#define B3 (+69028018 * K)
#define A0 (0)
#define A1 (+100346303 * K)
#define A2 (+4660361 * K)  //I have scaled everything by 1/10
#define A3 (-5006664 * K)
#define MIN_COMP_VAL (0)
#define MAX_COMP_VAL (uint32_t)(COMP_SCALE * 0.9f)
/**
 *
 */
#define VREF_VOLTS (5U)

#define PARAM_VREF 2482
