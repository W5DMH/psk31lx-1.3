#ifndef __HANSIS_FFT_H
#define __HANSIS_FFT_H

#ifndef CMPLX_DEFINED
#define CMPLX_DEFINED
typedef struct {
    double x,y;
} cmplx;
#endif

class FFTer
{
public:
    FFTer( int l, int mod=0 );
    ~FFTer();

#if 0
    Vector<Complex> operator*( const Vector<Complex>& fd );
    Vector<Complex> operator*( const Vector<double>& fd );
    Vector<Complex> operator*( const Vector<short>& fd );

    friend Vector<Complex> operator*( const Vector<Complex>& fd,
                                      const FFTer& fft );
    friend Vector<Complex> operator*( const Vector<double>& fd,
                                      const FFTer& fft );
    friend Vector<Complex> operator*( const Vector<short>& fd,
                                      const FFTer& fft );
    friend Vector<Complex> operator/( const Vector<Complex>& fd,
                                      const FFTer& fft );
#endif

protected:
    int len, M;
    int mode;
    double *real_a, *imag_a;


public:
    // Dumm-DFT: Hilfsfunktionen und -tabellen
    cmplx *Dm;
#if 0
    Vector<Complex> dft( const Vector<Complex>, int invers ) const;
    Vector<Complex> dft( const Vector<double>, int invers ) const;
    Vector<Complex> dft( const Vector<short>, int invers ) const;
    void dft_init();
    void dft_remove();
#endif

    // Split-Radix FFT mit Hilfstabellen (Base 2) und -funktionen
    void fftsr_init();
    void fftsr_remove();
    void fftsr_bitrevers(double *x, double *y) const;
    double *CT1, *CT3, *ST1, *ST3;
    int *itab;

    // a) normale Transformation (komplex)
    cmplx* fftsr( const cmplx* ) const;
    void fftsr_helper( int, int, int, double *, double *,
                       double *, double *, double *, double *,
                       double *, double *) const;
    void fftsr_main( double *, double *) const;
    // b) inverse Split-Radix (auch komplex)
    cmplx* ifftsr( const cmplx* ) const;
    void ifftsr_main( double *, double *) const;
    void ifftsr_helper( int, int, double *, double *,
                        double *, double *, double *, double *,
                        double *, double *) const;
    // c) reele Transformation (Input=Vector<double>)
    cmplx* rfftsr( const double* ) const;
    cmplx* rfftsr( const short* ) const;
    cmplx* rfftsr_wrapper( double *) const;
    void rfftsr_bitrevers(double *x) const;
    void rfftsr_main( double *) const;
    void rfftsr_helper( int, int, double *, double *, double *,
                        double *) const;
};

#endif
