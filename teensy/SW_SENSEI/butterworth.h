#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#include "common.h"
#include <stdio.h>
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

typedef std::complex<sensei::FilterType> Complex;

class Butterworth {

public:
	void createFilterCoefficients(int order, sensei::FilterType Fs, sensei::FilterType Fcut, sensei::FilterType* a, sensei::FilterType* b);

private:
	Complex* _p_poles;
	Complex* _z_poles;
	Complex* _z_zeros;
	Complex* _a_coeffs;
	Complex* _b_coeffs;
	void zeros2coeffs(Complex* zeros, Complex* coeffs, int size);
	void create_z_poles(Complex* poles, int order, sensei::FilterType cutoff);
	void create_z_zeros(Complex* zeros, int order);
	sensei::FilterType warp_freq(sensei::FilterType freq, sensei::FilterType Fs);
	void p2z(Complex* p, Complex* z, int size, sensei::FilterType Fs);
	void inverse_poly(Complex* coeffs, int size);
};

#endif // BUTTERWORTH_H
