#include "butterworth.h"

void Butterworth::createFilterCoefficients(int order, FilterType Fs, FilterType Fcut, FilterType* a, FilterType* b)
{
	p_poles = new Complex[order + 1];
	z_poles = new Complex[order + 1];
	b_coeffs = new Complex[order + 1];
	z_zeros = new Complex[order + 1];
	a_coeffs = new Complex[order + 1];

	FilterType cutoff = warp_freq(Fcut, Fs);
	create_z_poles(p_poles, order, cutoff);
	p2z(p_poles, z_poles, order, Fs);
	zeros2coeffs(z_poles, a_coeffs, order);
	create_z_zeros(z_zeros, order);
	zeros2coeffs(z_zeros, b_coeffs, order);

	FilterType k0_numer = 0.0;
	FilterType k0_denom = 0.0;

	for (int i = 0; i < order + 1; ++i){
		k0_numer += b_coeffs[i].real();
		k0_denom += a_coeffs[i].real();
	}
	FilterType k0 = k0_numer / k0_denom;

	for (int i = 0; i < order + 1; i++)
	{
		a[i] = a_coeffs[order - i].real();
		b[i] = b_coeffs[i].real() / k0;
	}

	delete(p_poles);
	delete(z_poles);
	delete(b_coeffs);
	delete(z_zeros);
	delete(a_coeffs);
}

void Butterworth::zeros2coeffs(Complex * zeros, Complex * coeffs, int size)
{
	int * b = new int[size + 1];
	memset(b, 0, sizeof(int)*(size + 1));
	int i = 0;
	while (!b[size]){
		i = 0;
		while (b[i]) b[i++] = 0;
		b[i] = 1;

		int subset_size = 0;
		Complex summand = 1.0;
		for (i = 0; i < size; i++){
			if (b[i]){
				summand *= zeros[i];
				++subset_size;
			}
		}
		coeffs[size - subset_size] += summand;
	}

	delete[] b;

	for (int i = (size + 1) % 2; i < size; i += 2){
		coeffs[i] = -coeffs[i];
	}

	coeffs[size] = Complex(1.0, 0.0);
}

void Butterworth::create_z_poles(Complex * poles, int order, FilterType cutoff)//cutoff Hz
{
	FilterType arg;
	for (int i = 0; i < order; ++i){
		arg = M_PI*(2 * i + order + 1) / 2 / order;

		poles[i] =  Complex(std::cos(arg), std::sin(arg));

		poles[i] *= 2 * M_PI*cutoff;
	}
}

void Butterworth::create_z_zeros(Complex * zeros, int order)
{
	for (int i = 0; i < order; ++i){
		zeros[i] = Complex(-1.0, 0.0);
	}
}

FilterType Butterworth::warp_freq(FilterType freq, FilterType Fs)
{
	return Fs*tan(M_PI*freq / Fs) / M_PI;
}

void Butterworth::p2z(Complex * p, Complex * z, int size, FilterType Fs)
{
	for (int i = 0; i < size; ++i){
		z[i] = (Complex(2 * Fs, 0.0) + p[i]) / (Complex(2 * Fs, 0.0) - p[i]);
	}
}

void Butterworth::inverse_poly(Complex * coeffs, int size)
{
	Complex denom = coeffs[size - 1];
	for (int i = 0; i < size; ++i){
		coeffs[i] /= denom;
	}
	reverse(coeffs, coeffs + size);
}
