#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H

#include "Arduino.h"
#include "common.h"
#include <stdio.h>
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

typedef std::complex<type_filter_var> Complex;

class Butterworth {

public:
	Butterworth();
	~Butterworth();
	void createFilterCoefficients(int order, type_filter_var Fs, type_filter_var Fcut, type_filter_var* a, type_filter_var* b);

private:

	Complex* p_poles;
	Complex* z_poles;
	Complex* b_coeffs;
	Complex* z_zeros;
	Complex* a_coeffs;

	void zeros2coeffs(Complex* zeros, Complex* coeffs, int size);
	void create_z_poles(Complex* poles, int order, type_filter_var cutoff);
	void create_z_zeros(Complex* zeros, int order);
	type_filter_var warp_freq(type_filter_var freq, type_filter_var Fs);
	void p2z(Complex* p, Complex* z, int size, type_filter_var Fs);
	void inverse_poly(Complex* coeffs, int size);
};





#endif
