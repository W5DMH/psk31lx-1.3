/* PSK31 -- Viterbi and Varicode encoder / decoder
 * based on PSK31-Code by Andrew Senior, G0TJZ
 * (C) 1998,1999 Hansi Reiser, DL9RDZ
 * Subject to GPL -- see LICENSE for details
 */

#include "psk31-coder.h"

/**********************************************************************/

int psk31_coder::decotab[2048];
int psk31_coder::encotab[256];
unsigned char psk31_coder::symbols[32];

unsigned char distance[4][4]= { {0, 2, 3, 2},
    {2, 0, 2, 3},
    {3, 2, 0, 2},
    {2, 3, 2, 0}
};

/* BPSK/QPSK-Encoder */
unsigned int psk31_coder::encode_qpsk(unsigned int sreg)
{
    unsigned int symb;

    symb =  psk31_coder::symbols[~sreg & 0x1F];
    if(!lsb) symb = (4 - symb) & 3; /* Reverse 90/270 exchanges if on LSB */
    return symb;
}


unsigned int psk31_coder::encode_bpsk(unsigned int sreg)
{
    if(sreg & 1) return 0;   /* 1 -> no change */
    else         return 2;   /* 0 -> reversal */
}
unsigned int psk31_coder::encode(unsigned int sreg, int use_qpsk)
{
    if(use_qpsk) return encode_qpsk(sreg);
    else         return encode_bpsk(sreg);
}

/* BPSK/QPSK-decoder */

/**********************************************************************/
/* functions used for receiving */

int psk31_coder::IQ2iphase(cmplx IQval)
{
    float phase;
    int iphase;

    phase=atan2(IQval.y, IQval.x);
    iphase=(int)(phase*32767/M_PI);
    return iphase;
}


int psk31_coder::encode_varicode(int symb)
{
    return encotab[symb];
}

int psk31_coder::decode_varicode(int bit)
{
    unsigned char c;

    bit = !bit;	/* Invert received bit */
    //fprintf(stderr,"%d",bit);

    rxreg >>= 1;
    if((rxreg & 0x0002) > 0)
        vlerror = 1;
    if(bit)
        rxreg |= 0x8000;
    if(((rxreg & 0xC000) == 0) && (rxreg != 0)) {
        while((rxreg & 0x0001) == 0)
            rxreg >>= 1;
        rxreg >>= 1;

        //TJW c=vlerror?'¥':decotab[rxreg]; if(c==13) c='\n';
        c=vlerror?'_':decotab[rxreg];
        if(c==13) c='\n';
        vlerror = 0;
        rxreg = 0;

        return c;
    }
    return NO_CHAR; /* no new char generated */
}

float psk31_coder::x_distance(cmplx rxsymb, int lastphase, int delta)
{
    int ph[4]= {0, 0x40, 0x80, 0xC0};
    float d;

    if(!lsb) {
        ph[1]=0xC0;    // change 90<>270 deg
        ph[3]=0x40;
    }

    /* Wir sollten jetzt bei lastphase+delta landen, sind aber bei
       rxsymb... */
    /* we should be at: */
    float x,y;
    int tmp=-(ph[delta]-lastphase);
    x=cos( tmp/128.0*M_PI );
    y=sin( tmp/128.0*M_PI );

    d=sqrt( (x-rxsymb.x)*(x-rxsymb.x) + (y-rxsymb.y)*(y-rxsymb.y) );

    return d*1.5; /* scaled to original scale -- unnecesary */
}


int psk31_coder::decode_qpsk(cmplx rxsymb)
{
    float dists[32]= {0}, min;
    long ests[32];
    int newabs[32];
    unsigned char select, vote;
    int i;
    int rxphase;


    ampl=sqrt(rxsymb.x*rxsymb.x+rxsymb.y*rxsymb.y);
    if(ampl>1e-5) {
        agc=1/ampl;
        rxsymb.x*=agc;
        rxsymb.y*=agc;
    }
    // rxsymbol now scaled to absolute value of 1
    //fprintf(stderr," %f'%f ",rxsymb.x,rxsymb.y);

    rxphase=IQ2iphase(rxsymb)>>8;

    int diffphase= (rxphase-lastphase)&0xFF;
    unsigned char rcvd = ((diffphase + 0x20) & 0xC0 ) >> 6;
    if(lsb) rcvd = (4 - rcvd) & 3;

    lastphase=rxphase;

    min = 1e6; /* Preset minimum = huge */
    for(i = 0; i < 32; i++) {
#if 0
        dists[i] = states[i/2].dist + distance[rcvd][symbols[i]];
#else
        /* Contrary to the original version use a "soft" decoder... */
        dists[i] = states[i / 2].dist +
                   x_distance(rxsymb,states[i/2].last_abs_phase,symbols[i]);
#endif
        newabs[i]=rxphase;

        if(dists[i] < min) min = dists[i];
        ests[i] = ((states[i / 2].estimate) << 1) + (i & 1);
    }
    //fprintf(stderr," min=%f ",min);
    for(i = 0; i < 16; i++) {
        if(dists[i] < dists[16 + i])
            select = 0;
        else
            select = 16;
        states[i].dist = dists[select + i] - min;
        states[i].estimate = ests[select + i];
        states[i].last_abs_phase=newabs[select + i];
    }
    vote = 0;
    for(i = 0; i < 16; i++)
        if(states[i].estimate & (1L << 20))
            vote++;
    if(vote == 8)
        return random()/(RAND_MAX>>1);
    else
        return (unsigned char)(vote > 8);
}

int psk31_coder::decode(cmplx rxsymb, int symbol)
{
    if(qpsk)
        return decode_varicode(decode_qpsk(rxsymb));
    else
        return decode_varicode(symbol);
}




void psk31_coder::prime_decoder() {
    cmplx prime= {1,0};
    /* Prime the Viterbi decoder */
    for(int k = 0; k < 20; k++) {
        decode_qpsk(prime);
        prime.x=-prime.x;
    }
}


unsigned char psk31_coder::parity(unsigned char u)
{
    unsigned char k, p;
    p=0;
    for(k=0; k<8; k++) {
        p ^= u & 1;
        u >>= 1;
    }
    return p;
}

int psk31_coder::init_tables(char *datadir)
{
    /* Generate convolutional coder symbols table */
    int k;
    for(k = 0; k < 32; k++) {
        symbols[k] = 2 * parity(k & poly1) + parity(k & poly2);
    }

    /* Initialize the Varicode tables */
    FILE *fp;
    int n;
    char *rtn;
    unsigned int codeword, mask;
    char buf[15], codfile[1024], *ptr;

    strncpy(codfile, datadir, 1000);
    if(codfile[strlen(codfile)-1]!=' ') strcat(codfile, "/");
    strcat(codfile, "psk31.cod");
    fp = fopen(codfile, "r");
    if(!fp) {
        ptr = getenv("HOME");
        if(ptr) {
            strncpy(codfile, ptr, 1000);
            strcat(codfile, "/psk31.cod");
            fp = fopen(codfile, "r");
        }
    }
    if(!fp)
        fp = fopen("psk31.cod", "r");
    if(!fp) {
        fprintf(stderr,"Can't open code table file 'psk31.cod'\n");
        exit(1);
    }
    for(k = 0; k < 2048; k++)
        decotab[k] = 248;	/* 'ø' indicates invalid codeword */
    for(k = 0; k < 256; k++) {
        codeword = 4;	/* Preset 100 pattern */
        rtn = fgets(buf, 15, fp);
        n=strlen(buf);
        while( buf[n]!='1' && buf[n]!='0' && n ) n--;
        for( ; n >= 0; n--) {
            codeword <<= 1;
            if(buf[n] == '1')
                codeword++;
        }
        encotab[k] = codeword;
        mask = 0x8000;	/* Start at MSB */
        while((mask & codeword) == 0)
            mask >>= 1;	/* Find dummy '1' MSB added earlier */
        codeword &= ~mask;	/* Remove it */
        codeword >>= 1;	/* Remove (always '1') LSB */
        decotab[codeword] = k;
    }
    fclose(fp);
    return 0;
}

