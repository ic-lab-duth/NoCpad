#ifndef __DUTH_HELPER_NON_SYNTH__
#define __DUTH_HELPER_NON_SYNTH__

unsigned int my_log2c(unsigned int val) {
	/// ceil(log2) integer calculation. Returns -1 for log2(0)
	unsigned int ret = -1;
	while (val != 0) {
		val >>= 1;
		ret++;
	}
	return ret;
}


#endif // __DUTH_HELPER_NON_SYNTH__
