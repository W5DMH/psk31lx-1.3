/* PSK31  -- receiver classs
 * partly derived from the work of Andrew Senior G0TJZ and Peter Martinez G3PLX
 * (C) 1998,1999 Hansi Reiser DL9RDZ
 * subject to GPL -- see LICENSE for details
 */

#include "psk31-receiver.h"
#include "coeff.h"   /* float fir1c[], fir2c[] */

/*
 * returns: -1: new sym,now rxbuf full; 0=no new sym; 1=new sym
 * currently: returns: new char, or NO_CHAR
 */
int psk31_receiver::process_rx_sample(short sample)
{
    float s,c,scarg,smpl;
    int retval;

    if(pskfft) pskfft->add_rx_sample(sample);

    smpl=sample*(1/32767.0);  /* scale to -1.0 .. 1.0 */

    /* Compute current rx LO value */
    rxphase=(rxphase+rxfreq)&0xFFFF;
    scarg=rxphase*(1.0/65536*2*M_PI);
    s=sin(scarg);
    c=cos(scarg);

    /* "mixer" to create I/Q out of Sample and LO */
    ibuf1_I[ibuf1_cur]=s*smpl;
    ibuf1_Q[ibuf1_cur]=c*smpl;
    ibuf1_cur = (ibuf1_cur+1)%FIR1TAPS;

    /* Every 16 samples run 500Hz-RX-Process */
    if( (stat&0x0F) == 0 ) {
        retval=process_500();
    }
    else retval=NO_CHAR;
    stat=(stat+1)&0xFF;
    return retval;
}

extern int OPTnew;

/* returns received character (or NO_CHAR) */
int psk31_receiver::process_500()
{
    float Ival, Qval;
    float amp, ftemp, ampsum;
    int cor, xxx, index, i;
    unsigned int temp;

    /* do the downsampling decimation filter into second buffer */
    Ival=ibuf2_I[ibuf2_cur]=do_fir(ibuf1_I, ibuf1_cur, fir1c, FIR1TAPS);
    Qval=ibuf2_Q[ibuf2_cur]=do_fir(ibuf1_Q, ibuf1_cur, fir1c, FIR1TAPS);
    if( (++ibuf2_cur) == FIR2TAPS ) ibuf2_cur=0;

    if(pskfft) pskfft->add_if_sample(Ival, Qval);

    /* And do the second FIR filtering */
    if(OPTnew&1) {
        Ival=do_fir(ibuf2_I, ibuf2_cur, fir2cnew, FIR2TAPS);
        Qval=do_fir(ibuf2_Q, ibuf2_cur, fir2cnew, FIR2TAPS);
    } else {
        Ival=do_fir(ibuf2_I, ibuf2_cur, fir2c, FIR2TAPS);
        Qval=do_fir(ibuf2_Q, ibuf2_cur, fir2c, FIR2TAPS);
    }

    float power = Ival * Ival + Qval * Qval;
    if(power>0) {
        strength = (unsigned int)(10 * log10(65536*power));
        if(strength>40) strength=40;
        if(strength<0) strength=0;
    } else strength=0;

    if(!(OPTnew&2)) {
        /* Compute amplitude in dB, shift it to the left in 'temp' */
        /* i guess simply calling 'log' would be faster on modern
           machines? */
        temp= (unsigned int)(65536* ( Ival*Ival + Qval*Qval ));
        if(temp) {
            xxx=31;
            while( xxx && !(temp&0x80000000) ) {
                xxx--;
                temp<<=1;
            }
            temp=(xxx<<20)|((temp&0x7FFFFFFF)>>11);
        }
        /* Now to the bit synchronisation. */
        index=bitclk>>20;
        amp_buf[index]=temp;
        cor=0;
        for(i=0; i<8; i++) cor+=amp_buf[i];
        for(   ; i<16; i++) cor-=amp_buf[i];
        cor/=500;    /* syngain=0.002 */
        bitclk-=cor;
    } else {
        amp = sqrt( Ival*Ival + Qval*Qval );
        index=bitclk>>20;
        amp_buf_new[index] = amp;

        ftemp = do_fir(amp_buf_new, (index+1)%16, ampcof, 16);
        ampsum=0;
        for(i=0; i<16; i++) ampsum+=amp_buf_new[i];

        if(ampsum!=0) ftemp=ftemp/ampsum;
        else ftemp=0;
        ftemp  *= 0.06;
        cor = (int)(ftemp*0x100000);
        bitclk-=cor;
    }
    bitclk+=0x100000;
    if(bitclk&0xFF000000) {
        /* here we are at the center of the bit... */
        bitclk&=0xFFFFFF;
        cmplx IQval;
        IQval.x=Ival;
        IQval.y=Qval;
        return process_rx_symbol(IQval);
    }
    else
        return NO_CHAR;
}


int psk31_receiver::process_rx_symbol(cmplx rxsymb)
{
    int diffphase, rxphase, symbol;

    rxphase=psk31_coder::IQ2iphase(rxsymb)>>8;
    diffphase=(rxphase-lastphase)&0xFF;
    lastphase=rxphase;

    doafc(diffphase);	/* Do automatic frequency correction */

    lastdelta=diffphase;

    symbol=diffphase2symbol(diffphase);
    dodcd(diffphase);	/* Do DCD */

    /* rxsymb is used for viterbi, symbol for BPSK (needn't recompute)*/
    int res=coder.decode(rxsymb, symbol);
    return res;
}

void psk31_receiver::doafc(int dp)
{
    if( !dcd || !afc)
        return; /* Don't correct if: DCD is off, sending, or AFC disabled */

    if(qpsk)
        dp = (int)((char)((dp<<2)&0xFF));   /* remove 2 MSBs */
    else
        dp = (int)((char)((dp<<1)&0xFF));   /* remove 1 MSB */
    if(dp == -128) /* eliminate bias */
        dp = 0;

#if 0 /* slow */
    _rxfreq += dp * 0.0002; /* apply correction */
#else
    _rxfreq += dp * 0.0005; /* apply correction */
#endif
    rxfreq = (int)(65536.0/SAMPLE_RATE * _rxfreq);
}


void psk31_receiver::dodcd(int diffphase)
{
    int symbol;
    symbol=diffphase2symbol(diffphase);

#if 0
    fprintf(writefile,"%d %f,%f\n",diffphase,is,qs);
#endif
    int n;
    if(qpsk) n=8;
    else n=4;
    is = 0.05*cos(n*M_PI*diffphase/256) + 0.95*is;
    qs = 0.05*sin(n*M_PI*diffphase/256) + 0.95*qs;

    dcdshfreg <<= 2; /* Shift register left */
    dcdshfreg += symbol; /* Insert latest symbol */
    if(dcdshfreg == 0xAAAAAAAAL) {
        // DCD on by Preamble
        dcd = 1;
        is=1;
        qs=0;
    }
    else if(dcdshfreg == 0L) {
        // DCD off by Postamble
        dcd = 0;
        is=qs=0;
    } else if(dcdlevel>0) {
        // TODO: for QPSK it would be better to use viterbi decoder
        // minimal distance as a decision metric!
        if( (is*is+qs*qs)*100 > dcdlevel*(dcd?0.50:1) ) {
            dcd = 1;
        } else {
            dcd = 0;
        }
    }
    // TODO: G0TJZ uses is,qs for a sponaneos frequency correction
    // based on that estimation... maybe we could do that here too?
#if 0
    fprintf(writefile,"%d %f,%f->%f %d\n",diffphase,is,qs,is*is+qs*qs,dcd);
#endif
}


float psk31_receiver::do_fir(float *base, int cur, float *coeff, int len)
{
    int c=0, i;
    float sum=0;

    for(i=cur; i<len; i++,c++) sum+=coeff[c]*base[i];
    for(i=0; i<cur-1; i++,c++) sum+=coeff[c]*base[i];
    return sum;
}


int psk31_receiver::diffphase2symbol(int diffphase)
{
    /* Convert diffphase to symbol number (0...3) */
    if(qpsk)
        diffphase = ((diffphase + 0x20) & 0xC0) >> 6;
    else
        diffphase = ((diffphase + 0x40) & 0x80) >> 6;
    if(lsb)
        diffphase = (4 - diffphase) & 3;
    return diffphase;
}

