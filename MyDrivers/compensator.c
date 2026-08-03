#include "compensator.h"

void compensator_init(struct type3_COMP* comp){
	comp->a[0] = A0;
	comp->a[1] = A1;
	comp->a[2] = A2;
	comp->a[3] = A3;
	comp->b[0] = B0;
	comp->b[1] = B1;
	comp->b[2] = B2;
	comp->b[3] = B3;
	comp->x[0] = 0;
	comp->x[1] = 0;
	comp->x[2] = 0;
	comp->x[3] = 0;
	comp->y[0] = 0;
	comp->y[1] = 0;
	comp->y[2] = 0;
	comp->y[3] = 0;
	comp->min_val = MIN_COMP_VAL;
	comp->max_val = 0; //max will be incremented and initialized during startup
	comp->rem = 0;
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
	comp->rem = 0;
}

uint32_t compensator_step(struct type3_COMP* comp){
//	 comp->x[0] = error;

	    /* prefer int64 for MAC if you can; if strict int32, keep products small */
	    int32_t acc =
	        comp->b[0] * comp->x[0] +
	        comp->b[1] * comp->x[1] +
	        comp->b[2] * comp->x[2] +
	        comp->b[3] * comp->x[3] +
	        comp->a[1] * comp->y[1] +
	        comp->a[2] * comp->y[2] +
	        comp->a[3] * comp->y[3];

	    acc += comp->rem;   /* carry fractional duty */

	    comp->y[0] = acc / COMP_SCALE; //the next duty cycle, but remember, it will get
	    comp->rem = acc - comp->y[0] * COMP_SCALE;   /* same as acc % COMP_SCALE for >=0 */

	    if (comp->y[0] > comp->max_val) {
	    	comp->y[0] = comp->max_val;
	        comp->rem = 0;   /* simple anti-windup on rail */
	    } else if (comp->y[0] < comp->min_val) {
	    	comp->y[0] = comp->min_val;
	        comp->rem = 0;
	    }

	    comp->x[3] = comp->x[2];
	    comp->x[2] = comp->x[1];
	    comp->x[1] = comp->x[0];
	    comp->y[3] = comp->y[2];
	    comp->y[2] = comp->y[1];
	    comp->y[1] = comp->y[0];

	    return comp->y[0];
}
