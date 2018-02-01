/* PSK31 -- Viterbi and Varicode encoder / decoder
 * based on PSK31-Code by Andrew Senior, G0TJZ
 * (C) 1998,1999 Hansi Reiser, DL9RDZ
 * subject to GPL -- see LICENSE for details
 */

#ifndef __PSK31_CODER_INCLUDED
#define __PSK31_CODER_INCLUDED

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef CMPLX_DEFINED
#define CMPLX_DEFINED
typedef struct {
    double x,y;
} cmplx;
#endif

#define SAMPLE_RATE 8000

/* codeword-flags */
#define CODE_USE_QPSK 0x010000       /* use qpsk for this codeword flag */
#define CODE_TONE_ON  0x020000       /* tone on flag */
#define CODE_CW_ECHO  0x040000       /* cw char for echo only */
#define CODE_PLAIN    0x080000       /* not yet encoded byte */

/* Send- and Echo-Char flags. Also used in the codeword buffer */
#define TX_MODE       0x200000
#define TXM_BPSK      0x000000
#define TXM_BPSK_L    0x000001
#define TXM_QPSK      0x000002
#define TXM_QPSK_L    0x000003
#define TXM_LSB       0x000001
#define TXM_CW        0x000004       // setting CW or TUNE does not change
#define TXM_TUNE      0x000008       // QPSK or LSB setting!

#define TX_FREQ       0x400000       /* 3xxxxx xx 0.01Hz */
#define TX_START      0x100001       /* start new transmission */
#define TX_END        0x100002       /* indicate end of transmission */
#define TX_URGENT   0x40000000       /* flag to execute cmds immediately*/
#define TX_DUMMY    0x001FFFFF       /* "unused" codeword */

#define NO_CHAR       0x800000
#define TX_BUSY       0x800001
#define TX_ERROR      0x800002

#define EMPTY_SAMPLE  0x100000


/* Viterbi and Varicode encoding and decoding funktions */
class psk31_coder {
private:
    /* Varicode data tables */
    static int hallo;
    static int decotab[2048];
    static int encotab[256];
    /* Convolutional coder/Viterbi coder data */
    static unsigned char symbols[32];
    static const unsigned char poly1 = 0x19, poly2 = 0x17;

    int qpsk, lsb;
    struct state {
        float dist;
        int last_abs_phase;
        long estimate;
    };
    int lastphase;

    struct state states[16];     /* QPSK decoder trellis */
    float agc,ampl;
    int vlerror, rxreg;   /* Varicode decoder state */

    unsigned int encode_qpsk(unsigned int sreg);
    unsigned int encode_bpsk(unsigned int sreg);
    int decode_qpsk(cmplx rxsymbol);
    int decode_bpsk(cmplx rxsymbol);
    float x_distance(cmplx rxsymb, int lastphase, int delta);
    int decode_varicode(int bit);
    static unsigned char parity(unsigned char u);
public:
    void prime_decoder();
    static int init_tables(char *datadir);
    psk31_coder() {
        agc=ampl=vlerror=rxreg=0;
        lastphase=0;
        qpsk=lsb=0;
        for(int i=0; i<16; i++) {
            states[i].dist=states[i].last_abs_phase=0;
            states[i].estimate=0;
        }
    }
    void setmode(int qpskfl, int lsbfl) {
        qpsk=qpskfl;
        lsb=lsbfl;
    }
    /* encodes shift register data in phase increment (0..3) */
    unsigned int encode(unsigned int sreg, int use_qpsk);
    int encode_varicode(int symb);
    int decode(cmplx rxsymbol, int symb);
    static int IQ2iphase(cmplx IQval);
};

#endif
