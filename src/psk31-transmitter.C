/* PSK31 -- transmitter class
 * partly derived from the work of Andrew Senior G0TJZ and Peter Martinez G3PLX
 * (C) 1998,1999 Hansi Reiser DL9RDZ
 * subject to GPL -- see LICENSE for details
 */

#if 0
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
#include <assert.h>
#include "server.h"
#endif
#include "psk31-transmitter.h"

extern int full_duplex; // from psk31-main.C
extern int chans;       // from server-main.C
extern int bits;

// Thread synchronization
extern pthread_cond_t cond_tx_start, cond_tx_stop;
extern pthread_mutex_t mutex_tx_start, mutex_tx_stop;

/* Morse code encoding table */
/* 0=space, 1=dot; from msb to lsb; lowest 1 = end of code */
unsigned int psk31_transmitter::cwtab[58] =
{   0x14, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x7A, 0xB4,
    0xB6, 0x00, 0x54, 0xCE, 0x86, 0x56, 0x94, 0xFC,
    0x7C, 0x3C, 0x1C, 0x0C, 0x04, 0x84, 0xC4, 0xE4,
    0xF4, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00,
    0x60, 0x88, 0xA8, 0x90, 0x40, 0x28, 0xD0, 0x08,
    0x20, 0x78, 0xB0, 0x48, 0xE0, 0xA0, 0xF0, 0x68,
    0xD8, 0x50, 0x10, 0xC0, 0x30, 0x18, 0x70, 0x98,
    0xB8, 0xC8
};


int psk31_transmitter::execute_cmd(int c)
{
    if(c&TX_MODE)
    {
        if(c&TXM_TUNE)
            set_mode(-1,-1,2);  // Tune on
        else if(c&TXM_CW)
            set_mode(-1,-1,1);  // CW on
        else
            set_mode( (c&TXM_QPSK)?1:0, (c&TXM_LSB)?1:0, 0); // Normal mode
    }
    else if (c&TX_FREQ)
        set_freq(0.01 * (c&0xFFFFF));
    return 0;
}

int psk31_transmitter::send_char(int c)
{
    if(c==TX_START)
    {
        /* Start: send preamble in psk mode,
         * don't send it if we want to tune or send cw...
         * remove "TX_END" from buffer!
         */
        unqueue_end();
        if(transmit_state==TS_INACTIVE) {
            pthread_mutex_lock(&mutex_tx_stop);
            transmit_state=TS_TRANSMIT;
            pthread_cond_signal(&cond_tx_stop);
            pthread_mutex_unlock(&mutex_tx_stop);
        }
        else
        {
            if(transmit_state==TS_TRANSMIT)
                ;
            else if(transmit_state==TS_WAITFOREND)
            {
                pthread_mutex_lock(&mutex_tx_stop);
                transmit_state=TS_TRANSMIT;
                pthread_cond_signal(&cond_tx_stop);
                pthread_mutex_unlock(&mutex_tx_stop);
            }
        }
        start_execute_cmd();
        if(cwmode==0)
        {
            sendpreamble();
            // txIold=txInew=1; txQold=txQnew=0;
            txIold=txInew=0;
            txQold=txQnew=0;
        }
        else
        {
            txIold=txInew=0;
            txQold=txQnew=0;
        }
        return 0;
    }
    if( (c==TX_END) || (c==(TX_END|TX_URGENT)) )
    {
        if(c&TX_URGENT)
        {   // falls urgent: buffer weghaun...
            txbuf_sta=txbuf_end=txbuf;
        }
        if(transmit_state==TS_TRANSMIT)
        {
            if(sendchar_cwmode==0)
                sendpostamble();
            putcodeword(TX_END);
            transmit_state=TS_WAITFOREND;
        }
        return 0;
    }

    if( (c&TX_MODE) )
    {
        if( c&TXM_TUNE )
            sendchar_cwmode=2;
        else if( c&TXM_CW )
            sendchar_cwmode=1;
        else
            sendchar_cwmode=0;
    }

    if(bufferspace()<=128)
        return -1;  // no space in buffer...
    if(c>=0 && c<256)
    {
        if(sendchar_cwmode)
        {
            putcodeword( c|CODE_CW_ECHO );
            send_cw_char(c);
        }
        else
        {
            putcodeword(c|CODE_PLAIN);
        }
    }
    else if ((c&TX_FREQ) || (c&TX_MODE))
    {
        putcodeword(c);
    }
    return 0;
}


/* string should not contain control char like TX_END, TX_FREQ, ... */
int psk31_transmitter::send_string(char *str)
{
    if (bufferspace() < (sendchar_cwmode?5:1)*(int)strlen(str)+100)
        return -1;

    while(*str)
    {
        int c = (int)((unsigned char)*str);
        if(sendchar_cwmode)
        {
            putcodeword( c|CODE_CW_ECHO );
            send_cw_char(*str);
        }
        else
        {
            putcodeword( c|CODE_PLAIN );
        }
        str++;
    }
    return 0;
}


int psk31_transmitter::send_cw_string(int need_postamble, char *str)
{
    /* just a rough approximation of utilized space!! */
    if (bufferspace() < 5*(int)strlen(str))
        return -1;

    if(need_postamble)
        sendpostamble();
    while(*str)
    {
        putcodeword( ((unsigned char)*str)|CODE_CW_ECHO);
        send_cw_char(*str);
        str++;
    }
    //putcodeword(TX_END);
    return 0;
}

char *psk31_transmitter::strupr(char *str)
{
    char *ptr=str;

    while(*ptr)
    {
        *ptr=toupper(*ptr);
        ptr++;
    }
    return str;
}


int psk31_transmitter::getnextsymbol(void)
{
    unsigned char symb;

    symb = coder.encode(shiftreg, useqpsk);
    if(keydown)
        symb |= 4; /* Tone on */

    /* Add new bit from codeword */
    shiftreg <<= 1;
    if(codewordbuf & 1)
        shiftreg |= 1;
    codewordbuf >>= 1;

    if(codewordbuf <= 1)
    {
        /* Need to get new codeword */
        codewordbuf = 0;
        /* Get next codeword */
        codewordbuf = get_buffer_entry();

        /* Read flag bits in LSBs of codeword */
        if(codewordbuf & CODE_USE_QPSK)
            useqpsk = 1; /* Encode this one using QPSK */
        else
            useqpsk = 0; /* Encode using BPSK */
        if(codewordbuf & CODE_TONE_ON)
            keydown = 1; /* Tone should be on */
        else
            keydown = 0; /* Tone off */
        codewordbuf &= 0xffff; /* remove flag bits */
    }
    return symb;
}

int psk31_transmitter::bufferspace()
{
    int used = (txbuf_end-txbuf_sta+TXBUFSIZE)%TXBUFSIZE;
    return TXBUFSIZE - used;
}


/* Putting stuff into the circular tx buffer... */
void psk31_transmitter::putcodeword(int cword)
{
    if ((cword==(8|CODE_PLAIN)) && (txbuf_sta!=txbuf_end))
    {
        // backspace -> remove previous char if it is still in buffer!
        unsigned int *temp;
        temp = txbuf_end-1;
        if(temp<txbuf)
            temp=txbuf+TXBUFSIZE-1;
        if ((*temp)&CODE_PLAIN)
        {
            if (((*temp)&255) != 8)
            {
                txbuf_end = temp;
                return;
            }
        }
    }
    *txbuf_end = cword;
    txbuf_end++;
    if(txbuf_end>=txbuf+TXBUFSIZE)
        txbuf_end=txbuf;
    if(txbuf_sta==txbuf_end)
    {
        txbuf_sta++;
        if (txbuf_sta>=txbuf+TXBUFSIZE)
            txbuf_sta=txbuf;
    }
}


void psk31_transmitter::frontputcodeword(int c)
{
    txbuf_sta--;
    if(txbuf_sta<txbuf)
        txbuf_sta+=TXBUFSIZE;
    *txbuf_sta = c;
    if (txbuf_sta==txbuf_end)
    {
        txbuf_end--;
        if(txbuf_end<txbuf)
            txbuf_end+=TXBUFSIZE;
    }
}


void psk31_transmitter::unqueue_end()
{
    unsigned int *ptr=txbuf_sta;

    while (ptr!=txbuf_end)
    {
        if(*ptr==TX_END)
        {
            fprintf(stderr,"removing TX_END!\n");
            *ptr=TX_DUMMY;
        }
        ptr++;
        if (ptr>=txbuf+TXBUFSIZE)
            ptr=txbuf;
    }
}


int psk31_transmitter::getcodeword()
{
    int cword;
    if (transmit_state==TS_TRANSMIT||transmit_state==TS_WAITFOREND)
    {
        if (txbuf_end==txbuf_sta)
        {
            if (cwmode==2)
            {   // Tuning... -> carrier
                cword = CODE_TONE_ON | 0xFFFF;
            }
            else
            {
                if (cwmode==1)
                {   // CW send  -> no idle signal
                    cword = 0x10;
                }
                else
                {
                    cword = CODE_TONE_ON| 0x2000; // idle signal
                    if (qpsk)
                        cword|=CODE_USE_QPSK;
                }
            }
        }
        else
        {
            cword=*txbuf_sta;
            txbuf_sta++;
            if (txbuf_sta>txbuf+TXBUFSIZE)
                txbuf_sta=txbuf;
        }
    }
    else
    {
        /* STATE=TS_INACTIVE */
        cword=0x0002;  // one bit with TONE OFF!
    }
    return cword;
}


/* puts the preamble AT THE BEGINNING of the internal buffer */
void psk31_transmitter::sendpreamble(void)
{
    /* Sends 32 reversals at start of over */
    frontputcodeword(CODE_TONE_ON|0x0040);  // 6
    frontputcodeword(CODE_TONE_ON|0x2000);  // 13
    frontputcodeword(CODE_TONE_ON|0x2000);  // 13
}


void psk31_transmitter::sendpostamble(void)
{
    /*Flushes shift register and sends 32 bits of carrier at end of over*/
    if(qpsk)
    {
        /* Flush with 20 zeroes */
        putcodeword(CODE_TONE_ON|CODE_USE_QPSK|0x2000);   //13
        putcodeword(CODE_TONE_ON|CODE_USE_QPSK|0x0080);   //7
    }
    else
    {
        putcodeword(CODE_TONE_ON|0x0004); /*Flush with 2 zeroes */
    }

    /* 32 bits of carrier */
    putcodeword(CODE_TONE_ON|0x3FFF);   // 13
    putcodeword(CODE_TONE_ON|0x3FFF);   // 13
    putcodeword(CODE_TONE_ON|0x007F);   //  6
    putcodeword(0x0001);   /* Tone off */
}


void psk31_transmitter::send_cw_char(int c)
{
    unsigned char u;

    c = toupper(c);
    if (c < ' ' || c > 'Z')
        return;
    if (c == ' ')
        /* Inter-word space */
        putcodeword( 0x0008 ); /* 3 bits key-up */
    else
    {
        u = cwtab[c - 33];
        while (u > 0)
        {
            if(u == 0x80)
            {
                /* 2 dots space:
                * Combines with inter-element space to give
                * inter-character space */
                putcodeword(0x0004); /* 2 bits key-up */
            }
            else if (u & 0x80)
            {
                /* 3 dots mark = 1 dash */
                putcodeword(CODE_TONE_ON|0x000F);
                /* 3 bits key-down */
            }
            else
            {
                /* 1 dot mark */
                putcodeword(CODE_TONE_ON|0x003);
                /* 1 bit key-down */
            }
            /* 2 dot inter-element space */
            putcodeword(0x0004); /* 2 bits key-up */
            u = u << 1;
        }
    }
}


/**********************************************************************/
/* tx functions */

/* new version:
 * - verify if there is space for a whole bit (256 samples)
 * - if not, return with TX_BUSY
 * - otherwise:
 *   read current output counter (ocnt)
 *   generate new bit and write it to device
 *   if a new character was needed, schedule it for echo at ocnt+odelay
 *   echo next character if ocnt is sufficiently high
 */

//#define BLOCKSIZE (1<<DMA_BUF_BITS)
#define BLOCKSIZE 512

int psk31_transmitter::processor()
{
    static int wcnt = 0;

    int res, odelay;
    static int write_pending;
    static int len, buflen;
    static int  odd = 0;
    static short val, obuf[BLOCKSIZE];
    count_info cinfo;
    audio_buf_info ospace;

    // Get free space in write buffer....
    res=ioctl(audiofd, SNDCTL_DSP_GETOSPACE, &ospace);
    if(res)
    {
        if(errno==EBUSY)
        {
            // simplex card in read mode
            // assume that there is enough space!
            // (i.e. avoid check...)
            // set odelay (this is not the real value, this
            // echo will be too fast, but thats better than
            // nothing...
            odelay = 0;
        }
        else
        {
            perror("ERROR: GETOSPACE failed");
            return TX_ERROR;
        }
    }
    else
    {
        // ToDo: adjustable buffer limit --- a small buffer increases the
        // possibility of buffer underruns, but reduces the delay between
        // typing and actual transmission
        if(ospace.fragments<ospace.fragstotal-32)
        {
            usleep(5000);  // avoid 100% load!
            return TX_BUSY;
        }
#if 1
        // Emulation of GETODELAY via GETOSPACE value...
        // odelay value obtained this way might be larger than the
        // real ODELAY by up to one fragment size....
        odelay = (ospace.fragstotal*ospace.fragsize-ospace.bytes);
#endif
    }

#if 0
    // Using GETODELAY.....
    // does not work on all (esp. older) systems... -- requires OSS!
    res=ioctl(audiofd, SNDCTL_DSP_GETODELAY, &odelay);
    if(res)
    {
        perror("ERROR: GETODELAY failed");
        return TX_ERROR;
    }
#endif
    // odelay is set above
    ioctl(audiofd, SNDCTL_DSP_GETOPTR, &cinfo);

    if(!write_pending)
    {
        echo_char=0;  /* set by process_31() */

        if (bits == 16)
            buflen = 256;
        else
            buflen = 64;

        for (len=0; len<buflen; len++)
        {
            /* run 31.25Hz-TX-Process each 256 samples*/
            stat=(stat+1)&0xFF;
            if ((stat&0xFF)==0)
            {
                process_31();
            }
            val = filter_tx_sample(stat);
            if(echo_char==TX_END)
            {
                pthread_mutex_lock(&mutex_tx_start);
                transmit_state=TS_INACTIVE;
                pthread_cond_signal(&cond_tx_start);
                pthread_mutex_unlock(&mutex_tx_start);
            }
            if(echo_char)
            {
                add_echo_char(echo_char, cinfo.bytes+odelay+len*sizeof(short));
                echo_char=0;
            }

            /******* for signed 16 bit stereo *******/
            if (chans == STEREO && bits == 16)
            {
                obuf[len] = val;
                len++;
                /* same thing for both channels */
                obuf[len] = val;
            }

            /******* for signed 16 bit mono *******/
            if (chans == MONO && bits == 16)
            {
                obuf[len] = val;
            }

            /******* for unsigned 8 bit stereo *******/
            if (chans == STEREO && bits == 8)
            {
                /* signed short to an unsigned 8 bit */
                val = val/256 + 128;
                /* same thing for both channels */
                obuf[len] = (val << 8) | val;
            }

            /******* for unsigned 8 bit mono *******/
            if (chans == MONO && bits == 8)
            {
                /* signed short to an unsigned 8 bit */
                val = val/256 + 128;

                if ((odd & 1) == 1)
                {
                    /* lo bytes */
                    obuf[len] = obuf[len] | val;
                    odd++;
                }
                else
                {
                    /* high bytes */
                    obuf[len] = val << 8;
                    len--;  /* use this index agn for the lo bytes */
                    odd++;
                    continue;
                }
            }
            /***************************************/
        }
    }

    res=write(audiofd, &obuf, len*sizeof(short));

    if(res>0)
        wcnt+=res;

    if(res!=(int)(len*sizeof(short)))
    {
        if(res<0 && (errno==EINTR||errno==EAGAIN))
        {
            write_pending=1;
            return TX_BUSY;
        }
        if(res==0 || (res<0 && (errno==EBUSY)))
        {
            write_pending=1;
            return TX_BUSY;
        }
        if(res<0)
        {
            fprintf(stderr,"tx: write error... res=%d\n",res);
            return TX_ERROR;
        }
        fprintf(stderr,"tx: partial write (%d/%d)...\n",
                res,BLOCKSIZE*sizeof(short));
        return TX_ERROR;
    }
    write_pending=0;
    res = get_echo_char( cinfo.bytes );

#if 0
    if(res!=TX_BUSY)
        fprintf(stderr,"ECHO: returning %c (%x)\n", (unsigned char)res,res);
#endif

    if(res==TX_END)
    {
        if(transmit_state!=TS_INACTIVE)
        {
            /* This happens if the TX_END already has been moved
             * from the main queue to the echo queue and some
             * one starts to transmit... */
            fprintf(stderr,"WARNING: psk31-transmitter: ignoring "
                    "TX_END in tx state %d\n",transmit_state);
            return TX_BUSY;
        }
    }
    return res;
}


void psk31_transmitter::add_echo_char (int ch, int timestamp)
{
    echo_end->timestamp = timestamp;
    echo_end->ch = ch;
    echo_end++;
    if (echo_end >= &echo_buffer[ECHO_BUFFER_LEN])
        echo_end=&echo_buffer[0];
    if (echo_end==echo_start)
    {
        fprintf(stderr,"Warning: ECHO char buffer overflow!\n");
    }
}


int psk31_transmitter::get_echo_char (int timestamp)
{
    if(echo_start!=echo_end)
    {
        if (timestamp > echo_start->timestamp)
        {
            int x = echo_start->ch;
            echo_start++;
            if (echo_start >= &echo_buffer[ECHO_BUFFER_LEN])
                echo_start = echo_buffer;
            return x;
        }
    }
    return TX_BUSY;
}


int psk31_transmitter::process_31()
{
    char txsymb;   /* 0 or 2 for bspk, 0..3 for qpsk */
    float tmp;

    txsymb=getnextsymbol();

    txIold=txInew;
    txQold=txQnew;

    /* on/off */
    if (txsymb&4)
    {
        if (txInew==0 && txQnew==0)
            txInew=1;
    }
    else
    {
        txInew=txQnew=0;
    }

    if (txsymb&1)
    {
        /* -I>Q, Q>I at 90, 270 */
        tmp=-txInew;
        txInew=txQnew;
        txQnew=tmp;
    }
    if (txsymb&2)
    {
        /* -I>I, -Q>Q at 180,270 */
        txInew=-txInew;
        txQnew=-txQnew;
    }
    return 0;
}


/* returns sample value */
int psk31_transmitter::filter_tx_sample(int stat)  /* 8000Hz-Process TX */
{
    float shape, Itx, Qtx, scarg, s, c;

    /* Compute current tx LO value */
    txphase=(txphase+txfreq)&0xFFFF;
    scarg=txphase*(1.0/65536*2*M_PI);
    s=sin(scarg);
    c=cos(scarg);

    shape=cos( (M_PI/512)*stat );
    shape*=shape;                /* raised cosine shape */

    Itx = txIold*shape + txInew*(1-shape);
    Qtx = txQold*shape + txQnew*(1-shape);

    return (int)(16000*(Itx*s + Qtx*c));
}

/* this function is executed when a TX_START token is received.
 * it scans the start of the code word buffer. if TX_MODE or TX_FREQ
 * tokens are found, they are handled before the "real work" starts.
 * This help to have the correct cwmode setting when deciding about
 * preamble and idle signal...
 */
/* ToDo: There is one problem: If someone sends a TX_START before
 * the previous TX_END has been completely executed, the mode changes
 * affect the previos transmission. This is not a really big problem
 * (normally you don't start to transmit immediately after stopping to
 * to so), but this might be fixed some time... */
void psk31_transmitter::start_execute_cmd ()
{
    int s;
    if (txbuf_sta==txbuf_end)
        return;
    while (1)
    {
        s=*txbuf_sta;
        if ((s&TX_MODE) || (s&TX_FREQ))
        {
            // remove mode set command from buffer...
            txbuf_sta++;
            if (txbuf_sta>txbuf+TXBUFSIZE)
                txbuf_sta=txbuf;
            // and execute..
            execute_cmd(s);
        }
        else
            break;
    }
}


/* sets echo_char! */
int psk31_transmitter::get_buffer_entry ()
{
    int s;

    while(1)
    {
        s=getcodeword();
        if(s==TX_DUMMY)
            continue;
        if(s&CODE_CW_ECHO)
        {
            echo_char=s;
        }
        else if ((s&TX_MODE) || (s&TX_FREQ))
        {
            execute_cmd(s);
        }
        else
            break;
    }
    /* CW_ECHO, TX_MODE and TX_FREQ are handled in the loop. all other
     * symbols in the codeword buffer are treated now:
     */
    if (s&CODE_PLAIN)
    {
        /* encode character */
        echo_char=s;
        unsigned int codeword;

        s=s&255;

        codeword = coder.encode_varicode((unsigned char)s);
        codeword |= CODE_TONE_ON;
        if (qpsk)
            codeword |= CODE_USE_QPSK;
        return codeword;
    }
    if (s==TX_END)
    {
        echo_char=s;
        s=0;      /* TONE OFF  */
    }
    return s;
}


