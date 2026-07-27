The compensator parameters will be taken from the tustin\_coeffs calculation file. 

They will then be scaled by K so that the output of the compensator is a number from 0 to the ARR value. 

In addition to being scaled by K, they will be scaled by 2^30 (which is around 10^9) so that only integer division is needed. This is for speed. The A and B coeffs will be scaled by 2^30, and the resulting output of each step will be scaled down by 2^30 to provide the output duty cycle. 

