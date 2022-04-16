#include <stdint.h>

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1)
				+ out_min;
	// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
