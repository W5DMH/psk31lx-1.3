/* PSK31  -- receiver classs */

#ifndef PSK31_RECEIVER_H_INCLUDED
#define PSK31_RECEIVER_H_INCLUDED

#define FIR1TAPS 64
#define FIR2TAPS 64

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include "psk31-coder.h"
#include "psk31-fft.h"

/* receiver channel interface class */
class psk31_receiver {
private:
    int qpsk;        /* QPSK mode on */
    int lsb;         /* inverted mode */
    float _rxfreq;
    int rxfreq;
    int rxphase;
    int strength;    /* logarithmic RX signal strength; 0..40 */
    int dcd;         /* dcd flag */
    int dcdlevel;    /* -1=ignore dcd, 0=plain pre/postamble dcd,
			    otherwise: level 1..100 for dcd sensitivity */
    int afc;         /* AFC flag */
    int noiselevel;  /* test only: simulated addition noise */
    int dofft;       /* freq scan via fft activation */
    int lastdelta;
    psk31_coder coder;

    /* for DCD calculation... */
    unsigned long dcdshfreg;
    float is,qs;              // signal quality estimator

    /* fft stuff */
    float fft_result;
    float fftbufI[1024], fftbufQ[1024];
    int fftind;

    /* receive filters */
    float ibuf1_I[FIR1TAPS];
    float ibuf1_Q[FIR1TAPS];
    float ibuf2_I[FIR2TAPS];
    float ibuf2_Q[FIR2TAPS];
    int amp_buf[16];
    float amp_buf_new[16];
    int ibuf1_cur, ibuf2_cur; /*index of oldest sample/place for new one */
    int lastphase; /* phase of last rx symbol... */
    int bitclk;    /* phase synchronisation */

    int stat;        /* cyclic state counter for receiver */
    char *decode_buffer;

    float do_fir(float *base, int cur, float *coeff, int len);

    psk31_fft *pskfft;

    int process_500();
    int process_rx_symbol(cmplx rxsymb);
    void dodcd(int diffphase);
    void doafc(int symb);
    void pdistr(float *I, float *Q);
    void perform_fft(float I, float Q);
    int diffphase2symbol(int diffphase);
    FILE *writefile;
public:
    psk31_receiver(psk31_fft *fft) {
        pskfft = fft;
        stat=0;
        ibuf1_cur=ibuf2_cur=0;
        rxphase=0;
        for(int i=0; i<16; i++) amp_buf[i]=0;
        set_freq(1000);
        dofft=0;
        lastdelta=0;
        for(int i=0; i<FIR1TAPS; i++) ibuf1_I[i]=ibuf1_Q[i]=0;
        for(int i=0; i<FIR2TAPS; i++) ibuf2_I[i]=ibuf2_Q[i]=0;
        coder.prime_decoder();
        dcd = qpsk = lsb = dcdlevel = afc = lastdelta = strength = 0;
        rxfreq = 1000;
        _rxfreq = 1000.0; // just to NOT have unitialized values!
        is=qs=0;
    }
    psk31_fft *get_fft() {
        return pskfft;
    }
    void set_fft(psk31_fft *fft) {
        pskfft = fft;
    }
    int set_mode(int qpsk_flag, int lsb_flag=0) {
        qpsk=qpsk_flag;
        lsb=lsb_flag;
        coder.setmode(qpsk, lsb);
        return 0;
    }
    void set_afc(int afc_flag) {
        afc=afc_flag;
    }
    void set_dcdlevel(int dcd_flag) {
        dcdlevel=dcd_flag;
    }
    void set_dcd(int dcd_flag) {
        dcd=dcd_flag;
    }
    void set_freq(float f) {
        _rxfreq=f;
        rxfreq=(int)(65536.0/SAMPLE_RATE*f);
    }
    void get_info(int *q, int *l, float *f, int *dlevel, int *a, int *d,
                  int *stat, int *streng) {
        if(q) *q=qpsk;
        if(l) *l=lsb;
        if(f) *f=_rxfreq;
        if(dlevel) *dlevel=dcdlevel;
        if(a) *a=afc;
        if(d) *d=dcd;
        if(stat) *stat=lastdelta;
        if(streng) *streng=strength;
    }
    float get_freq() {
        return _rxfreq;
    }
    void enable_fft(int f) {
        dofft=f;
    }
    float get_fft_result() {
        return fft_result;
    }
    int process_rx_sample(short sample); /* return: 0=no new char;
                                                1=new char -1=buffer full */
};


#endif
