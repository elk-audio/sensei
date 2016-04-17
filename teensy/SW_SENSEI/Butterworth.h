#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#include "Arduino.h"
#include "common.h"
#include <stdio.h>
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

typedef std::complex<FilterType> Complex;

class Butterworth {

public:

	void createFilterCoefficients(int order, FilterType Fs, FilterType Fcut, FilterType* a, FilterType* b);

private:

	Complex* p_poles;
	Complex* z_poles;
	Complex* b_coeffs;
	Complex* z_zeros;
	Complex* a_coeffs;

	void zeros2coeffs(Complex* zeros, Complex* coeffs, int size);
	void create_z_poles(Complex* poles, int order, FilterType cutoff);
	void create_z_zeros(Complex* zeros, int order);
	FilterType warp_freq(FilterType freq, FilterType Fs);
	void p2z(Complex* p, Complex* z, int size, FilterType Fs);
	void inverse_poly(Complex* coeffs, int size);
};

#endif // BUTTERWORTH_H
