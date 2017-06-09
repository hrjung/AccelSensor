/* 
 * Free FFT and convolution (C)
 * 
 * Copyright (c) 2014 Project Nayuki
 * https://www.nayuki.io/page/free-small-fft-in-multiple-languages
 * 
 * (MIT License)
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * - The above copyright notice and this permission notice shall be included in
 *   all copies or substantial portions of the Software.
 * - The Software is provided "as is", without warranty of any kind, express or
 *   implied, including but not limited to the warranties of merchantability,
 *   fitness for a particular purpose and noninfringement. In no event shall the
 *   authors or copyright holders be liable for any claim, damages or other
 *   liability, whether in an action of contract, tort or otherwise, arising from,
 *   out of or in connection with the Software or the use or other dealings in the
 *   Software.
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "AccelSensorTask.h"

#define STATIC_COEFF_TABLE

#ifndef M_PI
#define	M_PI		3.14159265358979323846	/* pi */
#endif
#define SIZE_MAX ((size_t)-1)

#define FFT_SIZE	(FFT_SAMPLE_MAX/2)


// Private function prototypes
static size_t reverse_bits(size_t x, unsigned int n);

#ifdef STATIC_COEFF_TABLE
// cos, sin table for 128 samples
#if (FFT_SAMPLE_MAX == 128)
const static float cos_table[FFT_SIZE] = {
 1.000000, 0.998795, 0.995185, 0.989177,
 0.980785, 0.970031, 0.956940, 0.941544,
 0.923880, 0.903989, 0.881921, 0.857729,
 0.831470, 0.803208, 0.773010, 0.740951,
 0.707107, 0.671559, 0.634393, 0.595699,
 0.555570, 0.514103, 0.471397, 0.427555,
 0.382683, 0.336890, 0.290285, 0.242980,
 0.195090, 0.146730, 0.098017, 0.049068,
 0.000000, -0.049068, -0.098017, -0.146730,
 -0.195090, -0.242980, -0.290285, -0.336890,
 -0.382683, -0.427555, -0.471397, -0.514103,
 -0.555570, -0.595699, -0.634393, -0.671559,
 -0.707107, -0.740951, -0.773010, -0.803208,
 -0.831470, -0.857729, -0.881921, -0.903989,
 -0.923880, -0.941544, -0.956940, -0.970031,
 -0.980785, -0.989177, -0.995185, -0.998795,
};

const static float sin_table[FFT_SIZE] = {
 0.000000, 0.049068, 0.098017, 0.146730,
 0.195090, 0.242980, 0.290285, 0.336890,
 0.382683, 0.427555, 0.471397, 0.514103,
 0.555570, 0.595699, 0.634393, 0.671559,
 0.707107, 0.740951, 0.773010, 0.803208,
 0.831470, 0.857729, 0.881921, 0.903989,
 0.923880, 0.941544, 0.956940, 0.970031,
 0.980785, 0.989177, 0.995185, 0.998795,
 1.000000, 0.998795, 0.995185, 0.989177,
 0.980785, 0.970031, 0.956940, 0.941544,
 0.923880, 0.903989, 0.881921, 0.857729,
 0.831470, 0.803208, 0.773010, 0.740951,
 0.707107, 0.671559, 0.634393, 0.595699,
 0.555570, 0.514103, 0.471397, 0.427555,
 0.382683, 0.336890, 0.290285, 0.242980,
 0.195090, 0.146730, 0.098017, 0.049068,
};
#elif (FFT_SAMPLE_MAX == 256)
// cos, sin table for 128 samples
const static float cos_table[FFT_SIZE] = {
 1.000000, 0.999699, 0.998795, 0.997290,
 0.995185, 0.992480, 0.989177, 0.985278,
 0.980785, 0.975702, 0.970031, 0.963776,
 0.956940, 0.949528, 0.941544, 0.932993,
 0.923880, 0.914210, 0.903989, 0.893224,
 0.881921, 0.870087, 0.857729, 0.844854,
 0.831470, 0.817585, 0.803208, 0.788346,
 0.773010, 0.757209, 0.740951, 0.724247,
 0.707107, 0.689541, 0.671559, 0.653173,
 0.634393, 0.615232, 0.595699, 0.575808,
 0.555570, 0.534998, 0.514103, 0.492898,
 0.471397, 0.449611, 0.427555, 0.405241,
 0.382683, 0.359895, 0.336890, 0.313682,
 0.290285, 0.266713, 0.242980, 0.219101,
 0.195090, 0.170962, 0.146730, 0.122411,
 0.098017, 0.073565, 0.049068, 0.024541,
 0.000000, -0.024541, -0.049068, -0.073565,
 -0.098017, -0.122411, -0.146730, -0.170962,
 -0.195090, -0.219101, -0.242980, -0.266713,
 -0.290285, -0.313682, -0.336890, -0.359895,
 -0.382683, -0.405241, -0.427555, -0.449611,
 -0.471397, -0.492898, -0.514103, -0.534998,
 -0.555570, -0.575808, -0.595699, -0.615232,
 -0.634393, -0.653173, -0.671559, -0.689541,
 -0.707107, -0.724247, -0.740951, -0.757209,
 -0.773010, -0.788346, -0.803208, -0.817585,
 -0.831470, -0.844854, -0.857729, -0.870087,
 -0.881921, -0.893224, -0.903989, -0.914210,
 -0.923880, -0.932993, -0.941544, -0.949528,
 -0.956940, -0.963776, -0.970031, -0.975702,
 -0.980785, -0.985278, -0.989177, -0.992480,
 -0.995185, -0.997290, -0.998795, -0.999699,
};
const static float sin_table[FFT_SIZE] = {
 0.000000, 0.024541, 0.049068, 0.073565,
 0.098017, 0.122411, 0.146730, 0.170962,
 0.195090, 0.219101, 0.242980, 0.266713,
 0.290285, 0.313682, 0.336890, 0.359895,
 0.382683, 0.405241, 0.427555, 0.449611,
 0.471397, 0.492898, 0.514103, 0.534998,
 0.555570, 0.575808, 0.595699, 0.615232,
 0.634393, 0.653173, 0.671559, 0.689541,
 0.707107, 0.724247, 0.740951, 0.757209,
 0.773010, 0.788346, 0.803208, 0.817585,
 0.831470, 0.844854, 0.857729, 0.870087,
 0.881921, 0.893224, 0.903989, 0.914210,
 0.923880, 0.932993, 0.941544, 0.949528,
 0.956940, 0.963776, 0.970031, 0.975702,
 0.980785, 0.985278, 0.989177, 0.992480,
 0.995185, 0.997290, 0.998795, 0.999699,
 1.000000, 0.999699, 0.998795, 0.997290,
 0.995185, 0.992480, 0.989177, 0.985278,
 0.980785, 0.975702, 0.970031, 0.963776,
 0.956940, 0.949528, 0.941544, 0.932993,
 0.923880, 0.914210, 0.903989, 0.893224,
 0.881921, 0.870087, 0.857729, 0.844854,
 0.831470, 0.817585, 0.803208, 0.788346,
 0.773010, 0.757209, 0.740951, 0.724247,
 0.707107, 0.689541, 0.671559, 0.653173,
 0.634393, 0.615232, 0.595699, 0.575808,
 0.555570, 0.534998, 0.514103, 0.492898,
 0.471397, 0.449611, 0.427555, 0.405241,
 0.382683, 0.359895, 0.336890, 0.313682,
 0.290285, 0.266713, 0.242980, 0.219101,
 0.195090, 0.170962, 0.146730, 0.122411,
 0.098017, 0.073565, 0.049068, 0.024541,
};
#elif (FFT_SAMPLE_MAX == 512)
const static float cos_table[FFT_SIZE] = {
 1.000000, 0.999925, 0.999699, 0.999322,
 0.998795, 0.998118, 0.997290, 0.996313,
 0.995185, 0.993907, 0.992480, 0.990903,
 0.989177, 0.987301, 0.985278, 0.983105,
 0.980785, 0.978317, 0.975702, 0.972940,
 0.970031, 0.966976, 0.963776, 0.960431,
 0.956940, 0.953306, 0.949528, 0.945607,
 0.941544, 0.937339, 0.932993, 0.928506,
 0.923880, 0.919114, 0.914210, 0.909168,
 0.903989, 0.898674, 0.893224, 0.887640,
 0.881921, 0.876070, 0.870087, 0.863973,
 0.857729, 0.851355, 0.844854, 0.838225,
 0.831470, 0.824589, 0.817585, 0.810457,
 0.803208, 0.795837, 0.788346, 0.780737,
 0.773010, 0.765167, 0.757209, 0.749136,
 0.740951, 0.732654, 0.724247, 0.715731,
 0.707107, 0.698376, 0.689541, 0.680601,
 0.671559, 0.662416, 0.653173, 0.643832,
 0.634393, 0.624860, 0.615232, 0.605511,
 0.595699, 0.585798, 0.575808, 0.565732,
 0.555570, 0.545325, 0.534998, 0.524590,
 0.514103, 0.503538, 0.492898, 0.482184,
 0.471397, 0.460539, 0.449611, 0.438616,
 0.427555, 0.416430, 0.405241, 0.393992,
 0.382683, 0.371317, 0.359895, 0.348419,
 0.336890, 0.325310, 0.313682, 0.302006,
 0.290285, 0.278520, 0.266713, 0.254866,
 0.242980, 0.231058, 0.219101, 0.207111,
 0.195090, 0.183040, 0.170962, 0.158858,
 0.146730, 0.134581, 0.122411, 0.110222,
 0.098017, 0.085797, 0.073565, 0.061321,
 0.049068, 0.036807, 0.024541, 0.012272,
 0.000000, -0.012272, -0.024541, -0.036807,
 -0.049068, -0.061321, -0.073565, -0.085797,
 -0.098017, -0.110222, -0.122411, -0.134581,
 -0.146730, -0.158858, -0.170962, -0.183040,
 -0.195090, -0.207111, -0.219101, -0.231058,
 -0.242980, -0.254866, -0.266713, -0.278520,
 -0.290285, -0.302006, -0.313682, -0.325310,
 -0.336890, -0.348419, -0.359895, -0.371317,
 -0.382683, -0.393992, -0.405241, -0.416430,
 -0.427555, -0.438616, -0.449611, -0.460539,
 -0.471397, -0.482184, -0.492898, -0.503538,
 -0.514103, -0.524590, -0.534998, -0.545325,
 -0.555570, -0.565732, -0.575808, -0.585798,
 -0.595699, -0.605511, -0.615232, -0.624860,
 -0.634393, -0.643832, -0.653173, -0.662416,
 -0.671559, -0.680601, -0.689541, -0.698376,
 -0.707107, -0.715731, -0.724247, -0.732654,
 -0.740951, -0.749136, -0.757209, -0.765167,
 -0.773010, -0.780737, -0.788346, -0.795837,
 -0.803208, -0.810457, -0.817585, -0.824589,
 -0.831470, -0.838225, -0.844854, -0.851355,
 -0.857729, -0.863973, -0.870087, -0.876070,
 -0.881921, -0.887640, -0.893224, -0.898674,
 -0.903989, -0.909168, -0.914210, -0.919114,
 -0.923880, -0.928506, -0.932993, -0.937339,
 -0.941544, -0.945607, -0.949528, -0.953306,
 -0.956940, -0.960431, -0.963776, -0.966976,
 -0.970031, -0.972940, -0.975702, -0.978317,
 -0.980785, -0.983105, -0.985278, -0.987301,
 -0.989177, -0.990903, -0.992480, -0.993907,
 -0.995185, -0.996313, -0.997290, -0.998118,
 -0.998795, -0.999322, -0.999699, -0.999925,
};
const static float sin_table[FFT_SIZE] = {
 0.000000, 0.012272, 0.024541, 0.036807,
 0.049068, 0.061321, 0.073565, 0.085797,
 0.098017, 0.110222, 0.122411, 0.134581,
 0.146730, 0.158858, 0.170962, 0.183040,
 0.195090, 0.207111, 0.219101, 0.231058,
 0.242980, 0.254866, 0.266713, 0.278520,
 0.290285, 0.302006, 0.313682, 0.325310,
 0.336890, 0.348419, 0.359895, 0.371317,
 0.382683, 0.393992, 0.405241, 0.416430,
 0.427555, 0.438616, 0.449611, 0.460539,
 0.471397, 0.482184, 0.492898, 0.503538,
 0.514103, 0.524590, 0.534998, 0.545325,
 0.555570, 0.565732, 0.575808, 0.585798,
 0.595699, 0.605511, 0.615232, 0.624860,
 0.634393, 0.643832, 0.653173, 0.662416,
 0.671559, 0.680601, 0.689541, 0.698376,
 0.707107, 0.715731, 0.724247, 0.732654,
 0.740951, 0.749136, 0.757209, 0.765167,
 0.773010, 0.780737, 0.788346, 0.795837,
 0.803208, 0.810457, 0.817585, 0.824589,
 0.831470, 0.838225, 0.844854, 0.851355,
 0.857729, 0.863973, 0.870087, 0.876070,
 0.881921, 0.887640, 0.893224, 0.898674,
 0.903989, 0.909168, 0.914210, 0.919114,
 0.923880, 0.928506, 0.932993, 0.937339,
 0.941544, 0.945607, 0.949528, 0.953306,
 0.956940, 0.960431, 0.963776, 0.966976,
 0.970031, 0.972940, 0.975702, 0.978317,
 0.980785, 0.983105, 0.985278, 0.987301,
 0.989177, 0.990903, 0.992480, 0.993907,
 0.995185, 0.996313, 0.997290, 0.998118,
 0.998795, 0.999322, 0.999699, 0.999925,
 1.000000, 0.999925, 0.999699, 0.999322,
 0.998795, 0.998118, 0.997290, 0.996313,
 0.995185, 0.993907, 0.992480, 0.990903,
 0.989177, 0.987301, 0.985278, 0.983105,
 0.980785, 0.978317, 0.975702, 0.972940,
 0.970031, 0.966976, 0.963776, 0.960431,
 0.956940, 0.953306, 0.949528, 0.945607,
 0.941544, 0.937339, 0.932993, 0.928506,
 0.923880, 0.919114, 0.914210, 0.909168,
 0.903989, 0.898674, 0.893224, 0.887640,
 0.881921, 0.876070, 0.870087, 0.863973,
 0.857729, 0.851355, 0.844854, 0.838225,
 0.831470, 0.824589, 0.817585, 0.810457,
 0.803208, 0.795837, 0.788346, 0.780737,
 0.773010, 0.765167, 0.757209, 0.749136,
 0.740951, 0.732654, 0.724247, 0.715731,
 0.707107, 0.698376, 0.689541, 0.680601,
 0.671559, 0.662416, 0.653173, 0.643832,
 0.634393, 0.624860, 0.615232, 0.605511,
 0.595699, 0.585798, 0.575808, 0.565732,
 0.555570, 0.545325, 0.534998, 0.524590,
 0.514103, 0.503538, 0.492898, 0.482184,
 0.471397, 0.460539, 0.449611, 0.438616,
 0.427555, 0.416430, 0.405241, 0.393992,
 0.382683, 0.371317, 0.359895, 0.348419,
 0.336890, 0.325310, 0.313682, 0.302006,
 0.290285, 0.278520, 0.266713, 0.254866,
 0.242980, 0.231058, 0.219101, 0.207111,
 0.195090, 0.183040, 0.170962, 0.158858,
 0.146730, 0.134581, 0.122411, 0.110222,
 0.098017, 0.085797, 0.073565, 0.061321,
 0.049068, 0.036807, 0.024541, 0.012272,
};
#else
#error FFT_SAMPLE_MAX is not correct value
#endif

#else
static int coeff_initialized=0;
static float cos_table[FFT_SIZE], sin_table[FFT_SIZE]; // about 1k
static void generate_coeff_table(void)
{
	int i;

	for (i = 0; i < FFT_SIZE; i++) {
		cos_table[i] = cos(2 * M_PI * i / FFT_SAMPLE_MAX);
		sin_table[i] = sin(2 * M_PI * i / FFT_SAMPLE_MAX);
	}
	coeff_initialized = 1;
}
#endif

int fft_transform(float *real, float *imag) {
	// Variables
	int status = 0;
	unsigned int levels;
	size_t size;
	size_t i;

	// Compute levels = floor(log2(n))
	{
		size_t temp = FFT_SAMPLE_MAX;
		levels = 0;
		while (temp > 1) {
			levels++;
			temp >>= 1;
		}
		if (1u << levels != FFT_SAMPLE_MAX)
			return status;  // n is not a power of 2
	}
	
#if 0 //check power of 2
	// Trignometric tables
	if (SIZE_MAX / sizeof(float) < FFT_SIZE)
		return false;

	size = (n / 2) * sizeof(float);
	cos_table = malloc(size);
	sin_table = malloc(size);
	if (cos_table == NULL || sin_table == NULL)
		goto cleanup;
#endif

#ifndef STATIC_COEFF_TABLE
	if(!coeff_initialized)
	{
		generate_coeff_table();
	}
#endif
	
	// Bit-reversed addressing permutation
	for (i = 0; i < FFT_SAMPLE_MAX; i++) {
		size_t j = reverse_bits(i, levels);
		if (j > i) {
			float temp = real[i];
			real[i] = real[j];
			real[j] = temp;
			temp = imag[i];
			imag[i] = imag[j];
			imag[j] = temp;
		}
	}
	
	// Cooley-Tukey decimation-in-time radix-2 FFT
	for (size = 2; size <= FFT_SAMPLE_MAX; size *= 2) {
		size_t halfsize = size / 2;
		size_t tablestep = FFT_SAMPLE_MAX / size;
		for (i = 0; i < FFT_SAMPLE_MAX; i += size) {
			size_t j;
			size_t k;
			for (j = i, k = 0; j < i + halfsize; j++, k += tablestep) {
				float tpre =  real[j+halfsize] * cos_table[k] + imag[j+halfsize] * sin_table[k];
				float tpim = -real[j+halfsize] * sin_table[k] + imag[j+halfsize] * cos_table[k];
				real[j + halfsize] = real[j] - tpre;
				imag[j + halfsize] = imag[j] - tpim;
				real[j] += tpre;
				imag[j] += tpim;
			}
		}
		if (size == FFT_SAMPLE_MAX)  // Prevent overflow in 'size *= 2'
			break;
	}
	status = 1;
	
//cleanup:
//	free(cos_table);
//	free(sin_table);
	return status;
}

void fft_getMagnitude(uint16_t *mag, float *real, float *img)
{
	int i;
	double sum=0, calc=0;

	for(i=0; i<FFT_SIZE; i++)
	{
		sum = (double)(real[i]*real[i] + img[i]*img[i]);

		 calc = sqrt(sum)/(double)FFT_SIZE;
		 mag[i] = (uint16_t)round(calc);
	}
}

static size_t reverse_bits(size_t x, unsigned int n) {
	size_t result = 0;
	unsigned int i;
	for (i = 0; i < n; i++, x >>= 1)
		result = (result << 1) | (x & 1);
	return result;
}

