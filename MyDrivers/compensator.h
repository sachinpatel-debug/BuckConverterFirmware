#pragma once

#include <stdint.h>
#include "control_params.h"
/*
 * type 3 compensator struct including the coefficients, min and max vals, and
 */

struct type3_COMP {
	int32_t a[4];
	int32_t b[4]; //
	int32_t x[4]; //Error input (param_vref - meas_Vref)
	int32_t y[4]; //duty cycle output
	uint32_t min_val;
	uint32_t max_val;
	uint32_t float_to_int_scaler;
};

/*
 * @brief initialize the compensator with its a and b coefficients and their scalings and zeroes for inputs and outputs
 * @param the type 3 compensator with the necessary inputs, outputs, and coefficients
 */
void compensator_init(struct type3_COMP* comp);
/*
 * @brief reset the compensator error inputs and duty outputs to 0
 */
void compensator_reset(struct type3_COMP* comp);

/*
 * @brief perform computation to find next output based on old outputs and duty cycles
 * @param the type 3 compensator with the necessary inputs, outputs, and coefficients
 * @param the error signal (vref - vout) on the adc scale
 * @return the duty cycle
 */
uint32_t compensator_step(struct type3_COMP* comp, int32_t error);
