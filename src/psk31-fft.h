
#ifndef PSK31_FFT_H_INCLUDED
#define PSK31_FFT_H_INCLUDED
#include "stdio.h"
#include "hansis_fft.h"

class psk31_fft {
public:
    static const int MODE_RXDATA=0;
    static const int MODE_NARROWIF=1;
    void set_parameters(int len, int _overlap, int _mode);
    void get_parameters(int *len, int *over, int *mode);
    void add_rx_sample(int sample);
    void add_if_sample(float I, float Q);
    int has_new_data();
    int get_abs_data(float *data, int start, int len, int delta);
    void Psk31_fft() {
        rxdata = NULL;
        Idata = Qdata = NULL;
        fft = NULL;
        window = NULL;
        N = valid_counter=-1;
    }
private:
    inline float sqr(float x) {
        return x * x;
    }
    int N;
    int overlap;
    int mode;
    int *rxdata;
    float *Idata, *Qdata;
    int cnt;
    int valid_counter;
    FFTer *fft;
    float *window;
};
#endif
