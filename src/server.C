/* HF amateur radio communicator
 *
 * digital HF modes (PSK31, RTTY, MT63, ...)
 * primary server module
 */


/* servermain:
 *
 * opens and initializes sound card
 *
 * initializer encoder and decoder classes (modes/psk31*.C)
 *
 * starts new thread for encoding and decoding
 *
 * offers interface functions for the user interface
 *
 * mutex operations for accesing en-/decoder classes from en/decoding
 * thread and from user interface thread (==interface functions)
 */

#include "server.h"

#ifdef USE_PTHREAD
#include <pthread.h>
#else
#define pthread_mutex_init(x,y)
#define pthread_mutex_lock(x)
#define pthread_mutex_unlock(x)
#endif

extern int use_stereo;          /* [0]=mono [1]=stereo */

//#define USEREALTIME
#define TX 1
#define RX 0

// receive / transmit state information
int full_duplex;
int chans;
int size;
int bits;
static char *audio_dev=NULL;
static char *ptt_dev=NULL;
static int audiofd=-1, pttfd=-1;
static int ptt_invert=0;
static int ctl_ptt(int onoff);
int scconfig[2] = {1, 1};   /* [0]=8bit/16bit [1]=mono/stereo */
int lastMode;


// schedule_transmit: 0=no 1=when dcd goes off, 2=immediately
static int schedule_transmit=0;
static int trDir = RX;

static int dcdlevel=0;

/* information: mutex usage:
 * mutex_rx is used for accessing the psk31_receiver objects as well as
 *          the psk31_fft object. this lock is set by the GUI when
 *          changing configuration parameters.
 * mutex_tx is similar for the psk31_transmitter object.
 *          the buffer mutexes are used for the RX buffers, when copying
 *          data to the GUI and when transfering data from the
 *          psk31 objects in the master loop
 *
 * secondary RX decoders: [limit: N_BUFFERS-3]
 * a secondary can be started by commControl on an unused channel (3..254)
 * with command COMM_MODE and value set to MO_NORMAL
 * it can be disabled by COMM_MODE with value MO_DISABLED
 * if the channel is disable, no other command than COMM_MODE(MO_NORMAL) is
 * accepted.
 */

psk31_transmitter *psk31tx;
static psk31_fft *psk31fft;

#ifdef USE_PTHREAD
static pthread_t master_thr;
static pthread_mutex_t mutex_rx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_tx = PTHREAD_MUTEX_INITIALIZER;
#ifdef PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP
static pthread_mutex_t mutex_fd = PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP;
#else
static pthread_mutex_t mutex_fd = PTHREAD_MUTEX_INITIALIZER;
#endif
#endif
/* static pthread_mutex_t mutex_fft= PTHREAD_MUTEX_INITIALIZER; */
/* FFT uses RX mutex! Thus psk31rx also can access the fft */

typedef struct
{
    int len;
    int rpos;
    int wpos;
    char *mem;
    psk31_receiver *psk31rx;
#ifdef USE_PTHREAD
    pthread_mutex_t mutex;
#endif
} buffer_t;

// Buffer==Channel: 0=TXdata-Echo 1=FFT-data 2,...=RX-data
#define N_BUFFERS 6
static buffer_t buffers[N_BUFFERS];


static void init_buffer()
{
    int i;
    for (i=0; i<N_BUFFERS; i++)
    {
        buffers[i].len = 16384;
        buffers[i].rpos = buffers[i].wpos = 0;
        buffers[i].mem = (char *)malloc(16384);
        buffers[i].psk31rx = NULL;
        pthread_mutex_init(&buffers[i].mutex, NULL);
    }
}


static int write_buffer(int bufnr, char *data, int cnt)
{
    int n, error;
    buffer_t *b = buffers+bufnr;
    if (cnt>b->len)
        return -1;  // too much data!
    pthread_mutex_lock( &b->mutex);
    if (cnt>((b->wpos-b->rpos)%b->len))
        error=-1;
    else
        error=0;
    if (b->wpos+cnt > b->len)
        n=b->len-b->wpos;
    else
        n=cnt;
    bcopy (data, b->mem+b->wpos, n);
    b->wpos += n;
    cnt-=n;
    if(cnt>0)
    {
        bcopy( data+n, b->mem, cnt);
        b->wpos = cnt;
    }
    pthread_mutex_unlock( &b->mutex);
    return error;
}


static int read_buffer(int bufnr, char *data, int cnt)
{
    int n, buflen;
    buffer_t *b = buffers+bufnr;

    pthread_mutex_lock( &b->mutex );
    buflen = (b->wpos-b->rpos)%b->len;
    if (buflen<0)
        buflen+=b->len;

    if (cnt>buflen)
        cnt=buflen;
    if (b->rpos+cnt > b->len)
        n=b->len-b->rpos;
    else
        n=cnt;
    bcopy (b->mem+b->rpos, data, n);
    b->rpos += n;
    cnt-=n;
    if (cnt>0)
    {
        bcopy (b->mem, data+n, cnt);
        b->rpos = cnt;
    }
    pthread_mutex_unlock ( &b->mutex);
    return n;
}


/*
 * Initialize the soundcard bits per sample and mono or stereo
 * via the soundcard parameter
 */
int init_audio ()
{
    int val;
    int speed = SAMPLE_RATE;
    int frags;

    pthread_mutex_lock(&mutex_fd);  //get the mutex for soundcard fd
    if (audiofd > 2)
    {
        close (audiofd);
        usleep (50000);   //strange
        audiofd = -1;
    }

    /* Open the soundcard */
    if (audiofd < 0)
    {
        if (trDir == RX)
        {
            audiofd = open(audio_dev, O_RDONLY|O_NONBLOCK);
        }
        else
        {
            audiofd = open(audio_dev, O_WRONLY|O_NONBLOCK);
        }
    }

    if (audiofd < 0)
    {
        fprintf (stderr, "init_audio: can't open %s\n", audio_dev);
        return -1;
    }

    /* set soundcard to 16 bit mode */
    /* return 1 if set 16 bits format failed */
    val=AFMT_S16_NE;
    if (ioctl(audiofd, SNDCTL_DSP_SETFMT, &val) <0)
    {
        /* ioctl failed */
        fprintf (stderr, "init_audio: ioctl SETFMT 16 failed\n");
        return -1;
    }

    /* see if 16 bit worked */
    if (val == AFMT_S16_NE)
    {
        /* we can use 16 bit */
        size = 2;
        bits = 16;
    }
    else /* set 16 bit failed */
    {
        fprintf (stderr, "init_audio: failed to set to 16 bits\n");
        return 1;
    }

    /* check parameter to see if MONO or STEREO was requested */
    /* return 2 or 3 if set mono or stereo format failed */
    switch (scconfig[1])
    {
    case 0:  /* mono requested */
        val=MONO;
        if( ioctl(audiofd, SNDCTL_DSP_STEREO, &val)<0 )
        {
            /* ioctl failed */
            fprintf (stderr,"\ninit_audio: ioctl failed - MONO\n");
            return -1;
        }
        /* see if mono worked */
        if (val == MONO)
        {
            /* we can use mono - say so and add the \n */
            fprintf (stderr, "mono format\n");
            chans = MONO;
        }
        else
        {
            /* set mono failed */
            fprintf (stderr, "\ninit_audio: request for mono failed\n");
            return 2;
        }
        break;

    case 1:   /* stereo request */
        val = STEREO;
        if( ioctl(audiofd, SNDCTL_DSP_STEREO, &val)<0 )
        {
            fprintf (stderr, "\ninit_audio: ioctl failed - STEREO\n");
            return -1;
        }
        /* see if stereo worked */
        if (val == STEREO)
        {
            /* we can use stereo - say so and add the \n */
            size = size * 2;
            chans = STEREO;
        }
        else
        {
            /* set stereo failed */
            fprintf (stderr, "\ninit_audio: request for stereo failed\n");
            return 3;
        }
        break;

    default:
        fprintf (stderr, "\ninit_audio: invalid mono or stereo request\n");
        return -1;
    }

    /* setup the rest */
    val = speed;
    if (ioctl(audiofd, SNDCTL_DSP_SPEED, &val) < 0)
    {
        fprintf (stderr, "init_audio: ioctl SPEED failed\n");
        return -1;
    }


    if (val != speed)
    {
        fprintf(stderr,"inexact sampling rate: "
                "request for %d resulted in %d\n",speed,val);
    }

    frags = 0x0002000A;
    if (chans == STEREO)
        frags++;

    val = frags;
    ioctl(audiofd, SNDCTL_DSP_SETFRAGMENT, &val);

    /*****************************************************/
#if 0
    audio_buf_info inInfo, outInfo;
    if (ioctl(audiofd, SNDCTL_DSP_GETISPACE, &inInfo))
        perror ("init_audio: SNDCTL_DSP_GETOSPACE");
    if (ioctl(audiofd, SNDCTL_DSP_GETOSPACE, &outInfo))
        perror ("init_audio: SNDCTL_DSP_GETOSPACE");

    printf ("In  fragments=0x%08x fragstotal=0x%08x fragsize=0x%08x\n",
            inInfo.fragments, inInfo.fragstotal, inInfo.fragsize);
    printf ("Out fragments=0x%08x fragstotal=0x%08x fragsize=0x%08x\n",
            outInfo.fragments, outInfo.fragstotal, outInfo.fragsize);
#endif
    /*****************************************************/

    // Check if the device is operating in full duplex mode
    if( ioctl(audiofd, SNDCTL_DSP_GETCAPS, &val)<0 )
        perror("Warning: GETCAPS on audio device failed");
    else if(val&DSP_CAP_DUPLEX)
        full_duplex=1;

    /*****************************************************/
#if 0
    val = 0;
    if (ioctl(audiofd, SNDCTL_DSP_SETTRIGGER, &val) == -1)
        perror("ioctl: SNDCTL_DSP_SETTRIGGER");
    val = PCM_ENABLE_INPUT;
    if (ioctl(audiofd, SNDCTL_DSP_SETTRIGGER, &val) == -1)
        perror("ioctl: SNDCTL_DSP_SETTRIGGER");
#endif
    /*****************************************************/

    pthread_mutex_unlock(&mutex_fd);
    return 0;
}


/*
 * init_ptt - initialize the ptt device
 */
static int init_ptt(void)
{
    pttfd = open (ptt_dev, O_RDWR);
    if (pttfd>0)
    {
        if (ctl_ptt(0)<0)
            return -1;
    }
    return pttfd;
}


static int ctl_ptt(int onoff)
{
    //fprintf(stderr,"pttctl[%d]\n",onoff);
    if (pttfd<0)
        return 0;  // PTT control disabled, nothing to do...
    if (ptt_invert)
        onoff=!onoff;
    int arg = TIOCM_RTS|TIOCM_DTR;
    if (ioctl(pttfd, onoff?TIOCMBIS:TIOCMBIC, &arg)<0)
        return -1;
    return 0;
}


#ifdef USE_PTHREAD
void master_handler(void);

static void *master_thr_func(void *dummy)
{
    while(1)
    {
        while (1)
        {
            master_handler();
        }
    }
}
#endif

void master_handler(void)
{
    int res;
    fd_set rset, wset, eset;
    struct timeval tm;
    int dcd;

    buffers[COMM_RXCH].psk31rx->get_info(NULL,NULL,NULL,NULL,
                                         NULL,&dcd,NULL,NULL);
    if ((schedule_transmit==1&&dcd==0) || schedule_transmit==2)
    {
        ctl_ptt(1);
        psk31tx->send_char(TX_START);
        trDir = TX;
        schedule_transmit=0;
    }

    if (trDir != lastMode)
    {
        res = init_audio ();
        if (res == -1)
        {
            perror("init_audio");
            exit (1);
        }
        psk31tx->set_audiofd(audiofd);
        lastMode = trDir;
    }

    FD_ZERO(&rset);
    FD_ZERO(&wset);
    FD_ZERO(&eset);
    FD_SET(audiofd, &rset);
    FD_SET(audiofd, &eset);
    if (trDir == TX)
        FD_SET(audiofd, &wset);
    tm.tv_sec=0;
    tm.tv_usec=50000; /* 50ms */
    res=select(audiofd+1, &rset, &wset, &eset, &tm);

    if (trDir == RX)
    {
        int sample[2];

        for (;;)
        {
            pthread_mutex_lock (&mutex_fd);  // grab the mutex for the fd
            res = read(audiofd, &sample, size);
            if (res == 0)
            {
                break;
            }
            else if (res != size)
            {
                if (errno == EINTR)
                {
                    break;
                }
                if (errno == EAGAIN || errno == EBUSY)
                {
                    break;
                }
                perror ("Audio read failed");
                exit (1);
            }
            pthread_mutex_unlock (&mutex_fd);  // release mutex for fd

            /******* for S16_NE stereo *******/
            if (chans == STEREO && bits == 16)
            {
                /* two signed 16 bits to one */
                sample[0] = sample[0];
            }

            /******* for S16_NE mono *******/
            if (chans == MONO && bits == 16)
            {
                /* nothing to do for this one */
            }

            for(int r=COMM_RXCH; r<N_BUFFERS; r++)
            {
                pthread_mutex_lock(&mutex_rx);
                psk31_receiver *rx = buffers[r].psk31rx;
                if(rx==NULL)
                {
                    pthread_mutex_unlock(&mutex_rx);
                    continue;
                }
                res = rx->process_rx_sample (sample[0]);
                pthread_mutex_unlock(&mutex_rx);
                if(res!=NO_CHAR)
                {
                    rx->get_info(NULL,NULL,NULL,NULL, NULL,&dcd,NULL,NULL);
                    if (dcd||(dcdlevel==-1))
                    {
                        char cd = (char)res;
                        write_buffer(r, &cd, 1);
                    }
                }
            }
        }
        pthread_mutex_unlock(&mutex_fd);  // release mutex for fd if we break
        // out at the for loop
    }

    if (trDir == TX)
    {
        char cd;
        for(;;)
        {
            if(full_duplex)
            {
                // clear buffer if necessary
                char buffer[128];
                for(;;)
                {
                    res=read(audiofd, buffer, 128);
                    if(res!=128) break;
                }
            }

            pthread_mutex_lock (&mutex_fd);
            pthread_mutex_lock(&mutex_tx);
            res=psk31tx->processor();
            pthread_mutex_unlock(&mutex_tx);
            pthread_mutex_unlock (&mutex_fd);
            // echo-characters, including TX_END,
            // are delivered from psk31tx->processor at the
            // exact end of the transmission!
            if(res==TX_BUSY)
            {
                break;
            }
            if(res==TX_ERROR)
            {
                perror("tx error:");
                break;
            }
            if(res==TX_END)
            {
                trDir = RX;
                ctl_ptt(0);
                break;
            }
            cd = (char)(res&0xFF);
            write_buffer(0, &cd, 1);
        }
    }
}


#ifdef USE_PTHREAD
static int run_master_thread()
{
    pthread_attr_t attr;
#ifdef USEREALTIME
    struct sched_param schp;
#endif

    fprintf (stderr,"run_master_thread: starting master thread\n");
    if (pthread_attr_init(&attr))
    {
        perror("Cannot initialize pthread attributes");
        exit(4);
    }

#ifdef USEREALTIME
    memset (&schp, 0, sizeof(schp));
    if (pthread_attr_setschedpolicy(&attr, SCHED_RR))
    {
        perror("Cannot set realtime scheduling attribute");
        exit(5);
    }
    schp.sched_priority = sched_get_priority_min(SCHED_RR)+1;
    if (pthread_attr_setschedparam(&attr, &schp))
    {
        perror("Cannot set realtime scheduling priority");
        exit(6);
    }
#endif
    if (pthread_create(&master_thr, &attr, master_thr_func, NULL))
    {
        perror("Cannot create IO processing thread");
        exit(7);
    }

    if (pthread_attr_destroy(&attr))
    {
        perror("Failed to destroy attribute");
    }
    return 0;
}
#endif

/* audio, ptt:  full path to device
 * data: directory of coder data file (psk31.cod)
 * ptt may be something like "/dev/ttyS0,i"  for inverted PTT
 */

int server_main(char *audio, char *ptt, char *datadir)
{
    init_buffer();
    int rtn;

    if(audio)
    {
        audio_dev = strdup(audio);
        printf ("using %s for audio\n", audio_dev);
    }
    if(ptt)
    {
        ptt_dev = strdup(ptt);
    }

    if(ptt_dev != NULL)
    {
        int len = strlen(ptt_dev);
        if(len>2 && ptt_dev[len-2]==',')
        {
            ptt_invert = (ptt_dev[len-1]!='0');
            ptt_dev[len-2]=0;
        }
        else
        {
            ptt_invert=0;
        }
    }
    rtn = init_audio();
    switch (rtn)
    {
    case -1:
        fprintf (stderr, "Cannot initialize audio device: %s %d\n",
                 audio_dev, rtn);
        return -1;

    case 0:
        fprintf (stderr, "using 16 bit %s format\n",
                 scconfig[1] == 0 ? "mono" : "stereo");
        break;

    case 1:
        fprintf (stderr, "set 16 bit format failed\n");
        return -1;

    case 2:
    case 3:
        fprintf (stderr, "set %s format failed\n",
                 scconfig[1] == 0 ? "mono" : "stereo");
    }

    if (ptt_dev && ((pttfd=init_ptt())<0))
    {
        fprintf(stderr, "Cannot open PTT device ");
        perror (ptt_dev);
    }
    printf ("using %s for ptt.\n", ptt_dev);

    psk31_coder::init_tables(datadir);
    psk31fft = new psk31_fft();
    psk31fft->set_parameters(SAMPLES, SAMPLES, psk31_fft::MODE_RXDATA);
    buffers[COMM_RXCH].psk31rx = new psk31_receiver(psk31fft);
    psk31tx = new psk31_transmitter();
    psk31tx->set_audiofd(audiofd);

#ifdef USE_PTHREAD
    if (run_master_thread() == 0)
    {
        return 0;
    }
#endif
    return 1;
}


int commWaitUpdate (unsigned long timeout)
{
    return -1;
}


int commGetData (int channel, char *buffer, int buflen)
{
    int len;

    if (channel<0||channel>=N_BUFFERS)
        return -1;

    if (channel==COMM_FFTCH)
    {
        if (psk31fft->has_new_data())
        {
            psk31fft->get_parameters(&len, NULL, NULL);
            if(buflen<(int)(len*sizeof(float)))
            {
                fprintf(stderr,"no space in buffer for FFT data\n");
                return 0;
            }
            psk31fft->get_abs_data((float *)buffer, 0, len, 1);
            return len;
        }
        else
            return 0;
    }
    else
    {
        return read_buffer(channel, buffer, buflen);
    }
}


int commPutData (char *buffer, int buflen)
{
    if (buflen<=0) buflen=strlen(buffer);
    pthread_mutex_lock(&mutex_tx);
    for (int i=0; i<buflen; i++)
    {
        psk31tx->send_char((unsigned char)(buffer[i]));
    }
    pthread_mutex_unlock(&mutex_tx);
    return buflen;
}


static int txControl (int spec, int value)
{
    int retval=0;
    static int mq=0, ml=0, mcw=0;

    pthread_mutex_lock(&mutex_tx);
    switch(spec)
    {
    case COMM_PTT:
        switch(value)
        {
        case PTTON:
            // schedule TX start on DCD off
            schedule_transmit=1;
            break;
        case PTTON|PTTFORCE:
            // schedule TX start now
            schedule_transmit=2;
            break;
        case PTTOFF:
            // schedule TX end!
            psk31tx->send_char(TX_END);
            break;
        case PTTOFF|PTTFORCE:
            // shall we just clear the internal buffer and send
            // a postamble before aborting? should be the right
            // thing to do! TX_END|TX_URGENT does this job!
            // However, this does not allow "instant" abort
            // for emergency cases when something really goes
            // wrong... (TODO: clarify this matter...)
            psk31tx->send_char(TX_END|TX_URGENT);
#if 0
            trDir = RX;
            ctl_ptt(0);
#endif
            break;
        }
        break;
    case COMM_QPSK:
        mq=value;
        psk31tx->send_char(TX_MODE|(value?TXM_QPSK:0)|(ml?TXM_LSB:0));
        break;
    case COMM_LSB:
        ml=value;
        psk31tx->send_char(TX_MODE|(mq?TXM_QPSK:0)|(value?TXM_LSB:0));
        break;
    case COMM_MODE:
        //fprintf(stderr,"mode: value=%d\n",value);
        if(value==MO_TUNE)
        {
            psk31tx->send_char(TX_MODE|TXM_TUNE);
            mcw = 2;
        }
        else if(value==MO_CWSEND)
        {
            psk31tx->send_char(TX_MODE|TXM_CW);
            mcw = 1;
        }
        else
        {
            psk31tx->send_char( TX_MODE | (mq?TXM_QPSK:0) | (ml?TXM_LSB:0) );
            mcw = 0;
        }
        break;
    case COMM_FREQ:
        psk31tx->send_char( TX_FREQ|(0xFFFFF&value) );
        break;
    default:
        retval=-1;
    }
    pthread_mutex_unlock(&mutex_tx);
    return retval;
}


static int rxControl (int channel, int spec, int value)
{
    int retval=0;
    int mqpsk, mlsb, mafc, musedcd, mdcd;
    float mfreq;
    int lastdelta, strength;

    psk31_receiver *rx = buffers[channel].psk31rx;
    if (rx==NULL)
    {
        if (spec==COMM_MODE && value==MO_NORMAL)
        {
            // start new decoder object
#if 0
            fprintf(stderr,"info: starting new decoder object"
                    "on channel %d\n",channel);
#endif
            buffers[channel].psk31rx = new psk31_receiver(NULL);
            return 0;
        }
        fprintf(stderr,"warning: rxControl on disabled RX channel\n"
                "channel=%d spec=%d value=%d\n",channel,spec,value);
        return -1;
    }

    pthread_mutex_lock(&mutex_rx);
    rx->get_info(&mqpsk, &mlsb, &mfreq, &musedcd, &mafc, &mdcd,
                 &lastdelta, &strength);
    switch (spec)
    {
    case COMM_SWAP:
        /* exchange psk31_receiver objects of current channel and
         * channel <value>
         * TODO- keep IQ FFT data reference on COMM_RXCH!!
         * (currently not used, so not a big issue)
         */
        if (value<COMM_RXCH||value>=N_BUFFERS)
        {
            retval=-1;
            break;
        }
        {
            psk31_receiver *rx2 = buffers[value].psk31rx;
            buffers[channel].psk31rx = rx2;
            buffers[value].psk31rx = rx;
            if (value==COMM_RXCH)
            {
                rx->set_fft(psk31fft);
                rx2->set_fft(NULL);
            }
            else if (channel==COMM_RXCH)
            {
                rx2->set_fft(psk31fft);
                rx->set_fft(NULL);
            }
            break;
        }
    case COMM_DCD:
        rx->set_dcd(value);
        break;
    case COMM_DCDLEVEL:
        dcdlevel = value;
        rx->set_dcdlevel(value);
        break;
    case COMM_QPSK:
        rx->set_mode(value, mlsb);
        break;
    case COMM_LSB:
        rx->set_mode(mqpsk, value);
        break;
    case COMM_AFC:
        rx->set_afc(value);
        break;
    case COMM_MODE:
        if (value==MO_DISABLED)
        {
            if (channel==COMM_RXCH)
            {
                // do not delete master RX channel. won't work
                // as base FFT depends on this master channel
                retval=-1;
                break;
            }
#if 0
            fprintf(stderr,"info: deleting decoder object"
                    "on channel %d\n",channel);
#endif
            delete rx;
            buffers[channel].psk31rx=NULL;
            buffers[channel].rpos = buffers[channel].wpos = 0;
        }
        else
        {
            fprintf(stderr,"warning: active channel %d: "
                    "COMM_MODE %d\n",channel,value);
        }
        break;
    case COMM_FREQ:
        rx->set_freq(0.01*value);
        break;
    default:
        retval=-1;
    }
    pthread_mutex_unlock(&mutex_rx);
    return retval;
}


int fftControl (int spec, int value)
{
    int len, overlap, mode;
    int retval=0;

    pthread_mutex_lock(&mutex_rx);
    psk31fft->get_parameters(&len, &overlap, &mode);
    switch(spec)
    {
    case COMM_FFTN:
        len = value;
        psk31fft->set_parameters(len, overlap, mode);
        break;
    case COMM_FFTOVERLAP:
        overlap = value;
        psk31fft->set_parameters(len, overlap, mode);
        break;
    default:
        retval=-1;
    }
    pthread_mutex_unlock(&mutex_rx);
    return retval;
}


int commControl(int channel, int spec, int value)
{

    if(channel==255)
    {
        // transmit channel
        return txControl(spec, value);
    }
    else if (channel==0)
    {
        // echo channel
        return 0;  // no control here!
    }
    else if (channel==1)
    {
        // fft channel
        return fftControl(spec, value);
    }
    else if (channel>=COMM_RXCH&&channel<N_BUFFERS)
    {
        // rx data chennal
        return rxControl(channel, spec, value);
    }
    else
    {
        return -1;  // invalid channel
    }
}


int commGetInfo(int channel, void *buffer, int buflen)
{
    float freq;
    PSK31info *info;

    if(channel==COMM_TXCH)
    {
        // transmit channel
        if(buflen<(int)sizeof(PSK31info))
            return -1;
        info = (PSK31info *)buffer;
        info->ptt = trDir;
        pthread_mutex_lock(&mutex_tx);
        psk31tx->get_info(&info->qpsk, &info->lsb, &info->cw, &freq);

        info->freq = (int)(100*freq);
        pthread_mutex_unlock(&mutex_tx);
    }
    else if(channel==COMM_ECHOCH)
    {
        // echo channel
        return -1;  // no info on echo channel
    }
    else if (channel==COMM_FFTCH)
    {
        // fft channel
        return -1;  // no info in fft channel (TODO)
    }
    else if (channel>=COMM_RXCH&&channel<N_BUFFERS)
    {
        // rx channel
        int retval=0;
        if(buflen<(int)sizeof(PSK31info))
            return -1;
        info = (PSK31info *)buffer;
        info->ptt = trDir;
        pthread_mutex_lock(&mutex_rx);
        psk31_receiver *rx = buffers[channel].psk31rx;
        if (rx==NULL)
        {
            // TODO: different return value for invalid channel
            // vs disabled channel
            retval=-1;
        }
        else
        {
            rx->get_info(&info->qpsk, &info->lsb, &freq,
                         &info->dcdlevel, &info->afc, &info->dcd,
                         &info->phdelta, &info->strength );
            info->freq = (int)(100*freq);
        }
        pthread_mutex_unlock(&mutex_rx);
        return retval;
    }
    else
    {
        return -1;  // invalid channel
    }
    return 0;   // or what should we return here?
}
