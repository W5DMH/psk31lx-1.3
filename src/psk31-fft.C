#include <math.h>

#include "psk31-fft.h"
#include "hansis_fft.h"

#include <stdio.h>

void psk31_fft::set_parameters(int len, int _delta, int _mode) {
    if(len!=N) {
        if(fft) delete fft;
        fft = new FFTer(len);
        N=len;
        if(Idata) delete[] Idata;
        Idata = new float[N];
        if(Qdata) delete[] Qdata;
        Qdata = new float[N];
        if(rxdata) delete[] rxdata;
        rxdata = new int[N];
        if(window) delete[] window;
        window = new float[N];
        for(int i=0; i<N; i++) {
            // init window... (simple triangluar window)
            if(i<N/2) window[i]=2.0*i/N;
            else window[i]=2.0*(N-i)/N;
        }
        cnt=0;
    }
    overlap = _delta;
    mode = _mode;
    valid_counter=N;
}

void psk31_fft::get_parameters(int *len, int *_delta, int *_mode) {
    if(len) *len=N;
    if(_delta) *_delta = overlap;
    if(_mode) *_mode = mode;
}

void psk31_fft::add_rx_sample(int sample) {
    if(mode!=MODE_RXDATA) return;
    rxdata[cnt++] = sample;
    if(cnt>=N) cnt=0;
    if(valid_counter) valid_counter--;
}

void psk31_fft::add_if_sample(float I, float Q) {
    if(mode!=MODE_NARROWIF) return;
    Idata[cnt]=I;
    Qdata[cnt++]=Q;
    if(cnt>=N) cnt=0;
    if(valid_counter) valid_counter--;
}

int psk31_fft::has_new_data() {
    return valid_counter==0;
}

/* Just copy out those values needed for display...
 * the maximum number of values has been specified by set_paramters
 * The copy out value starts at start (lower end == 0),
 * writes count values to data buffer,
 * where it compines step FFT values to one entry...
 * condition: start+count*step < MAXVALUES
 */

int psk31_fft::get_abs_data(float *data, int start, int count, int step)
{
    if( start<0||count<0||step<0 ) return -1;
    if( start + count*step > N ) return -1;

    cmplx * fftval; // = new cmplx[N];
    //Vector<Complex> fftval(N);

    // switch on type; copy source; performe fft
    if(mode==MODE_RXDATA) {
        double *mydata = new double[N];
        for(int i=0; i<N; i++) {
            mydata[i]=(double)(rxdata[(cnt+i)%N])/32767.0
                      *window[i];
        }
#if 0
        fprintf(stderr,"Input data:\n");
        for(int i=0; i<N; i++)
            fprintf(stderr,"%f ",mydata[i]);
#endif
        fftval = fft->rfftsr_wrapper(mydata);
        delete[] mydata;
    } else {
        cmplx * input = new cmplx[N];
        for(int i=0; i<N; i++) {
            input[i].x = Idata[(cnt+i)%N] * window[i];
            input[i].y = Qdata[(cnt+i)%N] * window[i];
        }
        fftval = fft->fftsr(input);
        delete[] input;
    }

    int index = start;
    //fprintf(stderr,"power value results:\n");
    for(int i=0; i<count; i++) {
        // calculate power
        float v = 0;
        for(int j=index; j<index+step; j++) {
            v += sqr(fftval[j].x)+sqr(fftval[j].y);
        }
        // copy to data...
        data[i] = sqrt(v);
        //fprintf(stderr,"%f ",v);
        index += step;
    }

    // without Vector class, this has the be freed explicitly
    delete[] fftval;

    // reset  ready counter
    valid_counter=overlap;
    return 0;
};
