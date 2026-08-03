#pragma once
#include <stdint.h>
/**
 * Type 3 Compensator paramater values scaled
 * Scaling is done so that only integer math is used as that helps computation speed tremendously
 */
#define COMP_SCALE (1 << 20)
#define PWM_ARR_VAL (100U)
#define K (0.2014651) //scaling factor defined by the arr register, the resistor divider, and the ADC GAIN
#define B0 (int32_t)(+0.44377462 * K * COMP_SCALE) //B coeffs take an adc error (the x array) and map it to a scaled duty cycle so that b*x range is (0-100)*COMP_SCALE
#define B1 (int32_t)(-0.31693807 * K * COMP_SCALE)
#define B2 (int32_t)(-0.43473636 * K * COMP_SCALE)
#define B3 (int32_t)(+0.32597733 * K * COMP_SCALE)
#define A0 (0U)
#define A1 (int32_t)(-0.0341864 * COMP_SCALE) //A coeffs take a duty cycle  between 0 and 100 (the y array) and scale it so that a*y range is (0-100)*COMP_SCALE. That's the same range as b*x which is good because a*y is added to B*x
#define A2 (int32_t)(+0.76680178 * COMP_SCALE)
//#define A3 (int32_t)(+0.26738619 * COMP_SCALE)
#define A3 (int32_t)(COMP_SCALE - A1 - A2) //enforce A1+A2+A3 equals 1 * COMP_SCALE
#define MIN_COMP_VAL (0)
#define MAX_COMP_VAL (uint32_t)(PWM_ARR_VAL * 0.9f)
/**
 *
 */
#define VREF_VOLTS (5U)

#define PARAM_VREF (2482)
