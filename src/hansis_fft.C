
///////////////////////////////////////////////////////////////////////////
// Hansi's FFT-Klasse
//
// (C) 1997 Hansi Reiser, dl9rdz, hsreiser@stud.informatik.uni-erlangen.de
//
// C++ class for a fast computation of DFT-transforms
//
// partly derived from:
// - my own FFT-routines, (C) 1995 Hansi Reiser
// - split-radix-algorithm by H.V. Sorensen, University of Pennsylavania,
//                            hvs@ee.upenn.edu (1987)
// - C++ - interface inspired by Jan Ernst's dft.C
//
// Reference:
// - Sanjit K. Mitra, James F. Kaiser: Handbook for Digital Signal
//   Processing, Jung Wiley&Sons Inc.
// - Richard E. Blahut, Algebraic Methods for Signal Processing and
//   Communications Coding, Springer Verlag
// - Sorensen, Heideman, Burrus: "On Computing the split-radix FFT",
//   IEEE Trans. ASSP, ASSP-34, No. 1, pp 152-156
// - Sorensen, Jones, Heideman, Burrus: "Real-Valued fast Fourier
//   transform algorithms", IEEE Trans. ASSP, ASSP-35, No. 6, pp 849-864
//
// This FFT class is provided AS IS and WITHOUT any WARRENTY or FITNESS
// FOR A PARTICULAR PURPOSE
// This program may be used and distributed freely provided this header
// is included and left intact.
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// Usage:
// The constructor creates an FFT-object for a specific transform length,
// several precomputations of often-used values are done upon creation,
// which may take some time. therefor, if several transforms of the same
// lengths are needed, it is advisable to instantiate on FFT-object for
// this length once, and then use that one for all transforms being
// perforemd.
//
// All Transform length NOT a power of 2 are done by the stupid O(N^2)
// DFT algorithm. In most cases you could do much better there by other
// methods, like the Good-Thomas or Winograd FFT algorithms, mostly based
// on the Chinese Remainder Theorem. This package only has been optimized
// for transform of length 2^N. Those are perfomed by an highly efficient
// split-radix algorithm.
// Further-optimized algorithms for computing the FFT of real-valued input
// are provided, requiring half the time of complex FFT of same length.
//
// Basic operations provided:
//       Vector<Complex> * FFTer -> Vector<Complex>   performs FFT
//       Vector<Complex> / FFTer -> Vector<Complex>   performs inverse FFT
//       Vector<double> * FFTer  -> Vector<Complex>   performs FFT optimized
//                                                    for real input
//       FFTer.fft( Vector<Complex> )      performs in-place FFT
//       FFTer.ifft( Vector<Complex> )     performs in-place inverse FFT
//       FFTer.rfft( Vector<Complex> )     performs in-place real FFT, imag
//                                         parts of input assumed being 0
// Additional operations provided:
//       Vector<short> * FFTer    converted to Vector<double> * FFTer
//       FFTer * Vector<...>      converted to Vector<...> * FFTer
//
///////////////////////////////////////////////////////////////////////////

// vielleicht kann ja jemand was damit anfagen. wenn ned... macht a
// nix :) ich hoff, daß keine allzu groben Bugs mehr drin sind. Der
// große C++ - programmierer bin ich ned, drum kannma an dem dem klassen-
// interface sicher no vieles besser machn, wenn jemandn was ned passt
// kanners mir ja sagn. das ganze wurde unter linux mit der gnu ansi-c++
// libg++ entwicklet (klassen Complex und Vektor). gnu verwendet für
// die exponentation statt dem XOR-Operator eine funktion namens pow,
// und vector wird klein statt groß geschrieben, sonst sollts keine
// größeren Unterschiede geben. (i hab jez nix gegen "`unsere"' complex-
// klasse, aber i wolltmi ned lang damit beschäftigen wie man dem g++
// sagt die eigene NICHT zu verwenden :-)
#include "hansis_fft.h"

#include "stdio.h"
#include "math.h"
#include "stdlib.h"


FFTer::FFTer( int l, int mod )
{
    int loop=2;

    // Testen, ob l=2^M ist, und ggf dieses M berechnen
    len=l;
    for( loop=2,M=1; loop<len; loop<<=1 ) M++;

    // je nachdem, ob wir die schnelle split-radix-fft verwenden dürfen...
    if( mod==0 && loop==len ) {
        mode=0;
        fftsr_init();
    }
    else {
        //mode=1;
        //dft_init();
        fprintf(stderr,"mode %d with len %d is not supported\n",
                mod, l);
    }
}

FFTer::~FFTer()
{
    if(mode==0)
        fftsr_remove();
    else
        ; //dft_remove();
}


#if 0
//////////////////////////////////////////////////////////////////////
// diverse Definitionen für den '*' und '/' - Operator

Vector<Complex> FFTer::operator*( const Vector<Complex>& fd )
{
    return fd * (*this);
}
Vector<Complex> FFTer::operator*( const Vector<double>& fd )
{
    return fd * (*this);
}
Vector<Complex> FFTer::operator*( const Vector<short>& fd )
{
    return fd * (*this);
}

#define CHECK_SIZE(fd,fft)                     \
	if( int((fd).size()) < fft.len ) {          \
		fprintf(stderr, "DFT-Vector too small\n");  \
		exit(1);                   \
	}

Vector<Complex> operator*( const Vector<Complex>& fd, const FFTer& fft )
{
    Vector <Complex> fr(fft.len);
    CHECK_SIZE(fd,fft);
    if(fft.mode) return fft.dft( fd, 0 );    // normale DFT
    return fft.fftsr( fd );     // schnelle FFT
}
Vector<Complex> operator*( const Vector<double>& fd, const FFTer& fft )
{
    Vector <Complex> fr(fft.len);
    CHECK_SIZE(fd,fft);
    if(fft.mode) return fft.dft( fd, 0 );    // normale DFT
    return fft.rfftsr( fd );     // schnelle FFT
}
Vector<Complex> operator*( const Vector<short>& fd, const FFTer& fft )
{
    Vector <Complex> fr(fft.len);
    CHECK_SIZE(fd,fft);
    if(fft.mode) return fft.dft( fd, 0 );    // normale DFT
    return fft.rfftsr( fd );     // schnelle FFT
}

Vector<Complex> operator/( const Vector<Complex>& fd, const FFTer& fft )
{
    Vector <Complex> fr(fft.len);
    CHECK_SIZE(fd, fft);
    if(fft.mode) return fft.dft( fd, 1 );    // normale inverse DFT
    return fft.ifftsr( fd );    // schnelle inverse FFT
}
#endif


//////////////////////////////////////////////////////////////////////
//// private Hilfsfunktionen primitiv-DFT, für len != pow(2,M)
//////////////////////////////////////////////////////////////////////
#if 0
void FFTer::dft_init()
{
    Complex m(cos(2*M_PI/len), -sin(2*M_PI/len));
    int i;

    Dm=new Complex[len];
    for(i=0; i<len; i++) Dm[i]=pow(m,i);
}
void FFTer::dft_remove()
{
    if( Dm ) delete [] Dm;
}
cmplx* FFTer::dft( cmplx* input, int invers ) const
{
    int i, j;
    cmplx* fr = new cmplx[len];
    for(i=0; i<len; i++) {
        fr[i]=0;
        if(invers) {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(len*len-i*j)%len] * input[j];
            }
            fr[i] /= len;
        }
        else {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(i*j)%len] * input[j];
            }
        }
    }
    return fr;
}
cmplx* FFTer::dft( const double* input, int invers ) const
{
    int i, j;
    Vector<Complex> fr(len);
    for(i=0; i<len; i++) {
        fr[i]=0;
        if(invers) {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(len*len-i*j)%len] * input[j];
            }
            fr[i] /= len;
        }
        else {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(i*j)%len] * input[j];
            }
        }
    }
    return fr;
}
Vector<Complex> FFTer::dft( const Vector<short> input, int invers ) const
{
    int i, j;
    Vector<Complex> fr(len);
    for(i=0; i<len; i++) {
        fr[i]=0;
        if(invers) {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(len*len-i*j)%len] * (int)input[j];
            }
            fr[i] /= len;
        }
        else {
            for(j=0; j<len; j++) {
                fr[i] += Dm[(i*j)%len] * (int)input[j];
            }
        }
    }
    return fr;
}
#endif

//////////////////////////////////////////////////////////////////////
//// private Hilfsfunktionen Split-Radix FFT
//// plain C, häßlich, aber (hoffentlich) umso effizienter
//// schönes C++ - Interface durch Klassenfunktion fftsr(..)
//////////////////////////////////////////////////////////////////////
void FFTer::fftsr_init()
{
    // (bei meinen privaten Variablen gilt hier: len == 2^M)
    int i, m2, imax, lbss;

    // sin/cos-Tabellen
    double tmp=2*M_PI/len;
    CT1 = new double[len];
    if(!CT1) {
        len=M=0;
        return;
    }
    CT3 = new double[len];
    if(!CT3) {
        delete [] CT1;
        len=M=0;
        return;
    }
    ST1 = new double[len];
    if(!ST1) {
        delete[] CT1;
        delete[] CT3;
        len=M=0;
        return;
    }
    ST3 = new double[len];
    if(!ST3) {
        delete[] CT1;
        delete[] CT3;
        delete[] ST1;
        len=M=0;
        return;
    }
    for(i=1; i<=len/8-1; i++) {
        CT1[i]=cos(tmp*i);
        CT3[i]=cos(tmp*i*3);
        ST1[i]=sin(tmp*i);
        ST3[i]=sin(tmp*i*3);
    }
    // temporärer Speicher...
    real_a=new double[len];
    imag_a=new double[len];

    // Bit-Reversal-Tabelle (a bisserl tricky bei split-radix)
    m2=(int)ceil(0.5*M);
    itab = new int[2*len];    // wieviel brauchi wirklich??
    itab[0]=0;
    itab[1]=1;
    imax=1;
    for(lbss=2; lbss<=m2; lbss++) {
        imax=2*imax;
        for(i=0; i<imax; i++) {
            itab[i]=itab[i]<<1;
            itab[i+imax]=1+itab[i];
        }
    }
}

void FFTer::fftsr_remove()
{
    if(!len) return;
    delete [] CT1;
    delete [] CT3;
    delete [] ST1;
    delete [] ST3;
    delete [] itab;
    delete [] real_a;
    delete [] imag_a;
}

void FFTer::fftsr_bitrevers(double *x, double *y) const
{
    int m2, nbit, k, l, j0, i, j;
    double tmp;
    // Bit-Reversal
    m2=M/2;
    nbit=1<<m2;
    for(k=1; k<nbit; k++) {
        j0=nbit*itab[k];
        i=k;
        j=j0;
        for(l=1; l<=itab[k]; l++) {
            tmp=x[i];
            x[i]=x[j];
            x[j]=tmp;
            tmp=y[i];
            y[i]=y[j];
            y[j]=tmp;
            i+=nbit;
            j=j0+itab[l];
        }
    }
}

void FFTer::rfftsr_bitrevers(double *x) const
{
    int m2, nbit, k, l, j0, i, j;
    double tmp;
    // Bit-Reversal
    m2=M/2;
    nbit=1<<m2;
    for(k=1; k<nbit; k++) {
        j0=nbit*itab[k];
        i=k;
        j=j0;
        for(l=1; l<=itab[k]; l++) {
            tmp=x[i];
            x[i]=x[j];
            x[j]=tmp;
            i+=nbit;
            j=j0+itab[l];
        }
    }
}

void FFTer::fftsr_helper( int n2, int n4, int its,
                          double *x1, double *x2, double *x3, double *x4,
                          double *y1, double *y2, double *y3, double *y4 )
const
{
    int curi, it, j, jn, i, i_start, i_delta, n8=n4/2;
    double tmp1, tmp2, tmp3, tmp4, tmp5;

    // "`Butterfly"' für Mantschfaktor (w_n)^0  [==1]
    // im Schleifeninneren wird effektiv berechnet:
    // Z1'=Z1+Z3; Z2'=Z2+Z4; Z3'=(Z1-Z3)-i(Z2-Z4); Z4'=(Z1-Z3)+i(Z2-Z4);
    // [ mit Zk := Xk + i Yk]
    i_start=0;
    i_delta=n2<<1;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            tmp1=x1[i]-x3[i];
            x1[i]+=x3[i];
            tmp2=y2[i]-y4[i];
            y2[i]+=y4[i];
            x3[i]=tmp1+tmp2;
            tmp2=tmp1-tmp2;
            tmp1=x2[i]-x4[i];
            x2[i]+=x4[i];
            x4[i]=tmp2;
            tmp2=y1[i]-y3[i];
            y1[i]+=y3[i];
            y3[i]=tmp2-tmp1;
            y4[i]=tmp2+tmp1;
        }
        i_start = 2*i_delta-n2;
        i_delta=4*i_delta;
    } while(i_start<=len);

    if(n4<=1) return;

    // "`Butterfly"' auf N/8-Positionen (Mantschfaktor e^{2*k*PI/8})
    // hier spart spart man sich noch die Hälfte der Multiplikationen,
    // da bei Re(W)=Im(W) dieser Faktor ausgeklammert werden kann...
    i_start=0;
    i_delta=n2<<1;
    do {
        for(i=i_start+n8; i<len; i+=i_delta) {
            tmp1=x1[i]-x3[i];
            x1[i]+=x3[i];
            tmp2=x2[i]-x4[i];
            x2[i]+=x4[i];
            tmp3=y1[i]-y3[i];
            y1[i]+=y3[i];
            tmp4=y2[i]-y4[i];
            y2[i]+=y4[i];
            tmp5=(tmp4-tmp1)*(M_SQRT2/2);
            tmp1=(tmp4+tmp1)*(M_SQRT2/2);
            tmp4=(tmp3-tmp2)*(M_SQRT2/2);
            tmp2=(tmp3+tmp2)*(M_SQRT2/2);
            x3[i]=tmp4+tmp1;
            y3[i]=tmp4-tmp1;
            x4[i]=tmp5+tmp2;
            y4[i]=tmp5-tmp2;
        }
        i_start=2*i_delta-n2;
        i_delta*=4;
    } while(i_start < len);

    if( n8<=1 ) return;

    // und jetzt der ganz normale "`Butterfly"'. Vielleicht könnte man ja
    // noch weiter optimieren bei machen Spezielfällen, aber das brauchts
    // jetzt dann doch ned...
    // Dafür machma weils so schön is immer gleich 2 Stück in einem
    // Schleifendurchlauf. Spart ein paar Operationen auf den Schleifen-
    // variablen

    i_start=0;
    i_delta=2*n2;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            it=0;
            jn=i+n4;
            for(j=1; j<=n8-1; j++) {
                it+=its;
                // der erste...
                curi=i+j;
                tmp1=x1[curi]-x3[curi];
                x1[curi]+=x3[curi];
                tmp2=x2[curi]-x4[curi];
                x2[curi]+=x4[curi];
                tmp3=y1[curi]-y3[curi];
                y1[curi]+=y3[curi];
                tmp4=y2[curi]-y4[curi];
                y2[curi]+=y4[curi];
                tmp5=tmp1-tmp4;
                tmp1+=tmp4;
                tmp4=tmp2-tmp3;
                tmp2+=tmp3;
                x3[curi]= tmp1*CT1[it] - tmp4*ST1[it];
                y3[curi]=-tmp4*CT1[it] - tmp1*ST1[it];
                x4[curi]= tmp5*CT3[it] + tmp2*ST3[it];
                y4[curi]= tmp2*CT3[it] - tmp5*ST3[it];
                // und der zweite...
                curi=jn-j;
                tmp1=x1[curi]-x3[curi];
                x1[curi]+=x3[curi];
                tmp2=x2[curi]-x4[curi];
                x2[curi]+=x4[curi];
                tmp3=y1[curi]-y3[curi];
                y1[curi]+=y3[curi];
                tmp4=y2[curi]-y4[curi];
                y2[curi]+=y4[curi];
                tmp5=tmp1-tmp4;
                tmp1+=tmp4;
                tmp4=tmp2-tmp3;
                tmp2+=tmp3;
                x3[curi]= tmp1*ST1[it] - tmp4*CT1[it];
                y3[curi]=-tmp4*ST1[it] - tmp1*CT1[it];
                x4[curi]=-tmp5*ST3[it] - tmp2*CT3[it];
                y4[curi]=-tmp2*ST3[it] + tmp5*CT3[it];
            }
        }
        i_start = 2*i_delta - n2;
        i_delta = 4*i_delta;
    } while ( i_start < len );
}

void FFTer::fftsr_main( double *x, double *y ) const
{
    int its, k, n2, n4, i_start, i_delta, i;
    double tmp;

    // Split-Radix-"`Butterflies"' (diese L-förmigen Dinger...)
    its=1;
    n2=2*len;
    for( k=0; k<M-1; k++ ) {
        n2=n2>>1;
        n4=n2>>2;
        fftsr_helper( n2, n4, its, x, x+n4, x+2*n4, x+3*n4,
                      y, y+n4, y+2*n4, y+3*n4 );
        its<<=1;
    }

    // End-"`Butterflies"' (die noch fehlenden Länge-2-Transformationen)
    i_start=0;
    i_delta=4;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            tmp=x[i];
            x[i]+=x[i+1];
            x[i+1]=tmp-x[i+1];
            tmp=y[i];
            y[i]+=y[i+1];
            y[i+1]=tmp-y[i+1];
        }
        i_start=i_delta*2-2;
        i_delta*=4;
    } while( i_start<len );

    fftsr_bitrevers(x,y);
    // done.
}

void FFTer::ifftsr_main( double *x, double *y ) const
{
    int n2,n4,k,i_start,i_delta,i;
    double tmp;
    // "`L-shaped Butterfly"'
    n2=len<<1;
    for(k=0; k<M-1; k++) {
        n2=n2>>1;
        n4=n2>>2;
        ifftsr_helper(n2, n4, x, x+n4, x+2*n4, x+3*n4,
                      y, y+n4, y+2*n4, y+3*n4);
    }
    // Letzte 2er-Stufe
    i_start=0;
    i_delta=4;
    do {
        for( i=i_start; i<len; i+=i_delta ) {
            tmp=x[i];
            x[i]+=x[i+1];
            x[i+1]=tmp-x[i+1];
            tmp=y[i];
            y[i]+=y[i+1];
            y[i+1]=tmp-y[i+1];
        }
        i_start = 2*i_delta - 2;
        i_delta *= 4;
    } while(i_start<len);

    // rumgemantsche in die richtige Reihenfolge
    fftsr_bitrevers(x,y);

    // und dann noch durch len teilen:
    tmp=1.0/len;
    for(i=0; i<len; i++) {
        x[i]*=tmp;
        y[i]*=tmp;
    }
}

void FFTer::ifftsr_helper( int n2, int n4, double *x1, double *x2,
                           double *x3, double *x4, double *y1, double *y2,
                           double *y3, double *y4 ) const
{
    int j, jn, i1, i, i_start, i_delta, n8=n4/2;
    int it, its;
    double tmp1, tmp2, tmp3, tmp4, tmp5;

    // "`Butterfly"' für Mantschfaktor (w_n)^0  [==1]
    // fast wie die normale transformation...
    i_start=0;
    i_delta=n2<<1;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            tmp1=x1[i]-x3[i];
            x1[i]+=x3[i];
            tmp2=y2[i]-y4[i];
            y2[i]+=y4[i];
            x3[i]=tmp1-tmp2;
            tmp2=tmp1+tmp2;
            tmp1=x2[i]-x4[i];
            x2[i]+=x4[i];
            x4[i]=tmp2;
            tmp2=y1[i]-y3[i];
            y1[i]+=y3[i];
            y3[i]=tmp2+tmp1;
            y4[i]=tmp2-tmp1;
        }
        i_start = 2*i_delta-n2;
        i_delta=4*i_delta;
    } while(i_start<len);

    if(n4<=1) return;

    // "`Butterfly"' auf N/8-Positionen (Mantschfaktor e^{2*k*PI/8})
    // hier spart spart man sich noch die Hälfte der Multiplikationen,
    // da bei Re(W)=Im(W) dieser Faktor ausgeklammert werden kann...
    i_start=0;
    i_delta=n2<<1;
    do {
        for(i=i_start+n8; i<len; i+=i_delta) {
            tmp1=x1[i]-x3[i];
            x1[i]+=x3[i];
            tmp2=x2[i]-x4[i];
            x2[i]+=x4[i];
            tmp3=y1[i]-y3[i];
            y1[i]+=y3[i];
            tmp4=y2[i]-y4[i];
            y2[i]+=y4[i];

            tmp5=(tmp1-tmp4);
            tmp1+=tmp4;
            tmp4=(tmp2-tmp3);
            tmp2+=tmp3;

            x3[i] = (tmp5-tmp2)*(M_SQRT2/2);
            y3[i] = (tmp5+tmp2)*(M_SQRT2/2);
            x4[i] = (tmp4-tmp1)*(M_SQRT2/2);
            y4[i] = (tmp4+tmp1)*(M_SQRT2/2);
        }
        i_start=2*i_delta-n2;
        i_delta*=4;
    } while(i_start < len);

    if( n8<=1 ) return;
    // und jetzt der ganz normale "`Butterfly"'.
    // Ebenfalls immer gleich 2 Stück in einem Schleifendurchlauf.
    it=0;
    its=len/n2;

    for(j=1; j<n8; j++) {
        it+=its;

        i_start=0;
        i_delta=2*n2;
        jn = n4 - 2*j;
        do {
            for( i=i_start+j; i<len; i+=i_delta ) {  // !!
                // die erste
                tmp1 = x1[i]-x3[i];
                x1[i]+=x3[i];
                tmp2 = x2[i]-x4[i];
                x2[i]+=x4[i];
                tmp3 = y1[i]-y3[i];
                y1[i]+=y3[i];
                tmp4 = y2[i]-y4[i];
                y2[i]+=y4[i];
                tmp5 = tmp1 - tmp4;
                tmp1+=tmp4;
                tmp4 = tmp2 - tmp3;
                tmp2+=tmp3;
                x3[i] = tmp5*CT1[it] - tmp2*ST1[it];
                y3[i] = tmp2*CT1[it] + tmp5*ST1[it];
                x4[i] = tmp1*CT3[it] + tmp4*ST3[it];
                y4[i] =-tmp4*CT3[it] + tmp1*ST3[it];
                // die zweite
                i1 = i+jn;
                tmp1 = x1[i1]-x3[i1];
                x1[i1]+=x3[i1];
                tmp2 = x2[i1]-x4[i1];
                x2[i1]+=x4[i1];
                tmp3 = y1[i1]-y3[i1];
                y1[i1]+=y3[i1];
                tmp4 = y2[i1]-y4[i1];
                y2[i1]+=y4[i1];
                tmp5 = tmp1 - tmp4;
                tmp1+=tmp4;
                tmp4 = tmp2 - tmp3;
                tmp2+=tmp3;
                x3[i1] = tmp5*ST1[it] - tmp2*CT1[it];
                y3[i1] = tmp2*ST1[it] + tmp5*CT1[it];
                x4[i1] =-tmp1*ST3[it] - tmp4*CT3[it];
                y4[i1] = tmp4*ST3[it] - tmp1*CT3[it];
            }
            i_start = 2*i_delta - n2;
            i_delta *= 4;
        } while(i_start<=len);
    }
}

void FFTer::rfftsr_main( double *x ) const
{
    int i_start, i_delta, i, n2, n4;
    double tmp;

    // hier machen wir das ganze anderes herum.... also erst bitshuffling
    rfftsr_bitrevers(x);

    // dann die Länge-2-Butterflies
    i_start=0;
    i_delta=4;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            tmp=x[i];
            x[i]+=x[i+1];
            x[i+1]=tmp-x[i+1];
        }
        i_start = 2*i_delta - 2;
        i_delta *= 4;
    } while (i_start < len);

    // und dann die anderen "`L-förmigen"'
    n2=2;
    for(i=1; i<M; i++) {
        n2<<=1;
        n4=n2>>2;
        rfftsr_helper(n2, n4, x, x+n4, x+2*n4, x+3*n4);
    }
}

void FFTer::rfftsr_helper( int n2, int n4, double *x1,
                           double *x2, double *x3, double *x4) const
{
    int n8=n2/8,i_start,i_delta,i,j,i2,jn;
    double tmp1,tmp2,tmp3,tmp4,tmp5;
    double E, SS1, SS3, SD1, SD3, CC1, CC3, CD1, CD3;

    i_start=0;
    i_delta=n2*2;
    do {
        for(i=i_start; i<len; i+=i_delta) {
            tmp1 = x4[i]+x3[i];
            x4[i]-=x3[i];
            x3[i] = x1[i] - tmp1;
            x1[i] += tmp1;
        }
        i_start=2*i_delta-n2;
        i_delta*=4;
    } while(i_start<len);

    if(n4<=1) return;

    i_start=0;
    i_delta=n2*2;
    do {
        for(i=i_start+n8; i<len; i+=i_delta) {
            tmp1 = (x3[i]+x4[i])*(M_SQRT2/2);
            tmp2 = (x3[i]-x4[i])*(M_SQRT2/2);
            x4[i] = x2[i] - tmp1;
            x3[i] = -x2[i] - tmp1;
            x2[i] = x1[i] - tmp2;
            x1[i]+=tmp2;
        }
        i_start=2*i_delta-n2;
        i_delta*=4;
    } while( i_start<len);

    if(n8<=1) return;

    E = 2*M_PI/n2;
    SD1 = SS1 = sin(E);
    SS3 = SD3 = (3.0*SD1) - (4.0*SD1*SD1*SD1);
    CC1 = CD1 = cos(E);
    CC3 = CD3 = (4.0*CD1*CD1*CD1) - (3.0*CD1);
    for( j=1; j<n8; j++ ) {
        i_start=0;
        i_delta=2*n2;
        jn = n4 - 2*j;
        do {
            for( i=i_start+j; i<len; i+=i_delta ) {
                i2 = i + jn;
                tmp1 = x3[i]*CC1 + x3[i2]*SS1;
                tmp2 = x3[i2]*CC1 - x3[i]*SS1;
                tmp3 = x4[i]*CC3 + x4[i2]*SS3;
                tmp4 = x4[i2]*CC3 - x4[i]*SS3;
                tmp5 = tmp1 + tmp3;
                tmp3 = tmp1 - tmp3;
                tmp1 = tmp2 + tmp4;
                tmp4 = tmp2 - tmp4;
                x3[i] = tmp1 - x2[i2];
                x4[i2] = tmp1 + x2[i2];
                x3[i2] = -x2[i] - tmp3;
                x4[i] = x2[i] - tmp3;
                x2[i2] = x1[i] - tmp5;
                x1[i] = x1[i] + tmp5;
                x2[i] = x1[i2] + tmp4;
                x1[i2] = x1[i2] - tmp4;
            }
            i_start = 2*i_delta - n2;
            i_delta = 4*i_delta;
        } while( i_start < len );
        tmp1 = CC1*CD1 - SS1*SD1;
        SS1 = CC1*SD1 + SS1*CD1;
        CC1 = tmp1;
        tmp3 = CC3*CD3 - SS3*SD3;
        SS3 = CC3*SD3 + SS3*CD3;
        CC3 = tmp3;
    }
}

cmplx* FFTer::fftsr( const cmplx* input ) const
{
    cmplx * retvec = new cmplx[len];
    int i;

    for(i=0; i<len; i++) {
        real_a[i] = (double)input[i].x;
        imag_a[i] = (double)input[i].y;
    }
    fftsr_main( real_a, imag_a );

    for(i=0; i<len; i++) {
        retvec[i].x = real_a[i];
        retvec[i].y = imag_a[i];
    }
    return retvec;
}

cmplx* FFTer::ifftsr( const cmplx* input ) const
{
    cmplx * retvec = new cmplx[len];
    int i;

    for(i=0; i<len; i++) {
        real_a[i] = (double)input[i].x;
        imag_a[i] = (double)input[i].y;
    }
    ifftsr_main( real_a, imag_a );

    for(i=0; i<len; i++) {
        retvec[i].x = real_a[i];
        retvec[i].y = imag_a[i];
    }
    return retvec;
}

cmplx * FFTer::rfftsr( const double * input ) const
{
    int i;

    for(i=0; i<len; i++) {
        real_a[i] = input[i];
    }
    return rfftsr_wrapper(real_a);
}
cmplx * FFTer::rfftsr( const short * input ) const
{
    int i;

    for(i=0; i<len; i++) {
        real_a[i] = (double)input[i];
    }
    return rfftsr_wrapper(real_a);
}

cmplx * FFTer::rfftsr_wrapper( double *input ) const
{
    cmplx * retvec = new cmplx[len];
    int i;
    rfftsr_main( input );

    retvec[0].x = input[0];
    retvec[0].y = 0;
    retvec[len/2].x = input[len/2];
    retvec[len/2].y = 0;

    for(i=1; i<len/2; i++) {
        retvec[i].x = input[i];
        retvec[i].y = input[len-i];
        retvec[len-i].x = input[i];
        retvec[len-i].y = -input[len-i];
    }
    return retvec;
}

#if 0
#define LEN 32
#define DISP
main()
{
    FFTer myfft(LEN), janfft(LEN,1);
    Vector<Complex> v(LEN), v2;
    Vector<short> d(LEN);
    int i;

    printf("initalisieren...\n");
    for(i=0; i<LEN; i+=4) {
        v[i]=Complex(1,0);
        v[i+1]=Complex(2+i,0);
        v[i+2]=Complex(3-1,0);
        v[i+3]=Complex(4,0);
        d[i]=1;
        d[i+1]=2+i;
        d[i+2]=3-1;
        d[i+3]=4;
    }
    printf("\n");


#if 0
    printf("Performance-Test...\n\n");
    for(i=0; i<1000; i++) {
        v2 = d * myfft;
    }
    exit(1);
#endif

    printf("meine reelle FFT...\n");
    v2 = v * myfft;
#ifdef DISP
    for(i=0; i<LEN; i++) {
        printf("%f+i%f\t", v2[i].real(), v2[i].imag());
    }
    printf("\n");
#endif

    printf("und wieder invers:\n");
    v2 = v2 / myfft;
#ifdef DISP
    for(i=0; i<LEN; i++) {
        printf("%f+i%f\t", v2[i].real(), v2[i].imag());
    }
    printf("\n");
#endif

#ifndef DISP
    for(i=0; i<20; i++) {
        printf("%f+i%f\t", v2[i].real(), v2[i].imag());
    }
    printf("\n");
    exit(1);
#endif

    printf("die zweite DFT\n");
    v2 = v * janfft;
#ifdef DISP
    for(i=0; i<LEN; i++) {
        printf("%f+i%f\t", v2[i].real(), v2[i].imag());
    }
    printf("\n");
#endif

    printf("und wieder invers:\n");
    v2 = v2 / janfft;
#ifdef DISP
    for(i=0; i<LEN; i++) {
        printf("%f+i%f\t", v2[i].real(), v2[i].imag());
    }
    printf("\n");
#endif

    printf("done.\n");
}

#endif
