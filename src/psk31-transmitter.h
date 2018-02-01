/* PSK31 -- transmitter class
 * (C) 1998 Hansi Reiser DL9RDZ
 */

#ifndef __PSK31_TRANSMITTER_INCLUDED
#define __PSK31_TRANSMITTER_INCLUDED

#include "psk31-coder.h"

#include <math.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/soundcard.h>
#include <sys/ioctl.h>
#include "psk31-coder.h"
#include "psk31-transmitter.h"
#include <assert.h>
#include "server.h"


#define DMA_BUF_BITS 10

enum { TS_INACTIVE, TS_TRANSMIT, TS_WAITFOREND };

class psk31_transmitter {
private:
    static unsigned int cwtab[58];
    psk31_coder coder;
    int transmit_state;
    //int saved_echo_char;
    //int saved_echo_timestamp;
    int audiofd;
    int cwmode;
    int qpsk;
    int lsb;
    float _txfreq;
    int txfreq;
    int txphase;
    /* process31 / filter_tx_sample */
    float txIold,txQold,txInew,txQnew;
    int stat;
    /* getnextsymbol */
    unsigned char shiftreg;     /* Transmit shift register */
    unsigned int codewordbuf;   /* Current codeword buffer */
    int useqpsk;
    int keydown;
    int echo_char, pending_sample;
    int sendchar_cwmode;
#define TXBUFSIZE 2048
    unsigned int txbuf[TXBUFSIZE], *txbuf_sta, *txbuf_end;
    char *strupr(char *str);
    int process_31();
    int filter_tx_sample(int stat);  /* 8000Hz-Process TX */
    int getnextsymbol();
    int get_buffer_entry();
    void send_cw_char(int ch);
    void putcodeword(int cw);
    void frontputcodeword(int c);
    int getcodeword();
    void unqueue_end();
    void add_echo_char( int ch, int timestamp );
    int get_echo_char( int timestamp );
    struct echo_chars {
        int ch;
        int timestamp;
    };
#define ECHO_BUFFER_LEN 64
    struct echo_chars echo_buffer[ECHO_BUFFER_LEN], *echo_start, *echo_end;

    // mode parameters: 0 or 1; -1==no change
    void set_mode(int q, int l, int cw) {
        if(q>=0) qpsk=q;
        if(l>=0) lsb=l;
        if(cw>=0) cwmode=cw;
        coder.setmode(q, l);
    }
    void set_freq(float f)
    {
        _txfreq=f;
        txfreq=(int)(65536.0/SAMPLE_RATE*f);
    }
    int execute_cmd(int c);
    void start_execute_cmd();
    int bufferspace();

public:
    psk31_transmitter() {
        txIold=txInew=1;
        txQold=txQnew=0;
        txbuf_sta=txbuf;
        txbuf_end=txbuf;
        stat=255;
        echo_char=0;
        pending_sample=EMPTY_SAMPLE;
        set_freq(1000);
        //saved_echo_char=0;
        //saved_echo_timestamp=0;
        transmit_state=TS_INACTIVE;
        echo_start=echo_end=echo_buffer;
    }
    void get_info(int *q, int *l, int *cw, float *f) {
        if(q) *q=qpsk;
        if(l) *l=lsb;
        if(f) *f=_txfreq;
        if(cw) *cw=cwmode;
    }
    void set_audiofd(int fd) {
        audiofd=fd;
    }
    int send_char(int c);
    int get_tx_state(void) {
        return transmit_state;
    }
    int send_string(char *str);
    int send_cw_string(int need_postamble, char *str);
    void sendpreamble();
    void sendpostamble();
    int processor(); /* returns TX_BUSY / NO_CHAR / sent character */
};

#endif
