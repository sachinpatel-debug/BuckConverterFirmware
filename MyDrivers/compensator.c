#include "compensator.h"

void compensator_init(struct type3_COMP* comp){
	a[0] = A0;
	a[1] = A1;
	a[2] = A2;
	a[3] = A3;
	b[0] = B0;
	b[1] = B1;
	b[2] = B2;
	b[3] = B3;
	comp->x[0] = 0;
	comp->x[1] = 0;
	comp->x[2] = 0;
	comp->x[3] = 0;
	comp->y[0] = 0;
	comp->y[1] = 0;
	comp->y[2] = 0;
	comp->y[3] = 0;
//	comp->min_val =
}

void compensator_reset(struct type3_COMP* comp){
	comp->x[0] = 0;
	comp->x[1] = 0;
	comp->x[2] = 0;
	comp->x[3] = 0;
	comp->y[0] = 0;
	comp->y[1] = 0;
	comp->y[2] = 0;
	comp->y[3] = 0;
}

float compensator_step(struct type3_COMP* comp, int32_t error){
	comp->x[0] = error;
	comp->y[0] = comp->b[0] * comp->x[0] +
				 comp->b[1] * comp->x[1] +
				 comp->b[2] * comp->x[2] +
				 comp->b[3] * comp->x[3] +
				 comp->a[1] * comp->y[1] +
				 comp->a[2] * comp->y[2] +
				 comp->a[3] * comp->y[3]; //here we are performing the differential equation, but remember, all the A and B coeffs are scaled by COMP_SCALE
	if(y_n[0]>DUTY_TICKS_MAX){
		y_n[0]=DUTY_TICKS_MAX;
	}
	else if(y_n[0] < DUTY_TICKS_MIN){
		y_n[0] = DUTY_TICKS_MIN;
	}
	else{
		//nothing, MISRA compliance is a good thing ig
	}
	//next, update values so that current error error one cycle ago and so on
	e_n[3] = e_n[2];
	e_n[2] = e_n[1];
	e_n[1] = e_n[0];
	y_n[3] = y_n[2];
	y_n[2] = y_n[1];
	y_n[1] = y_n[0];
	return y_n[0];
}
