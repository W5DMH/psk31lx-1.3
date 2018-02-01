/* psk31lx:  PSK31/soundcard  for linux
 * Copyright (C) 1998-2000 Hansi Reiser, dl9rdz (dl9rdz@amsat.org)
 *
 * term-psk31.C
 * text-base terminal for the PSK31 core
 * Version: beta-1.0 22 Feb 2000
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139,
 * USA.
 *
 */
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <dirent.h>

#include <curses.h>
#include <ncurses.h>

#include "../config.h"
#include "text-window.h"
#include "server.h"
#include "psk31-coder.h"

#define TXSTART 0
#define TXLINES 6
#define LINE1 TXSTART + TXLINES
#define RXSTART LINE1 + 1
#define RXLINES 9
#define LINE2 RXSTART + RXLINES
#define STATSTART LINE2 + 1
#define STATLINES 7

#define TX_POLL_INTERVAL 30
#define CRC_POLYNOMIAL 0x2f // x^8 + x^5 + x^3 + x^2 + x + 1
#define RX_MAX_WAIT 1250000

#define RX_BEACON_FILENAME "pskreceived.txt"
#define RX_LOG_FILENAME "received.log"

typedef enum { CFG_NONE, CFG_PSK, CFG_BEACON, CFG_UNKNOWN } cfg_section_t;

/* Receiver context */
typedef struct {
    int ptr;
    char buf[256];
    int timer;
    FILE *log_fp;
} rx_ctx_t;

/* Thread syncronization variables */
pthread_cond_t cond_tx_start = PTHREAD_COND_INITIALIZER;
pthread_cond_t cond_tx_stop = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex_tx_start = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_tx_stop = PTHREAD_MUTEX_INITIALIZER;

extern psk31_transmitter *psk31tx;

/* Device Paths */
static char audio_path[20] = "/dev/dsp";
static char ptt_path[20] = "/dev/ttyS1";
static char messages_path[48] = PKG_DATA_DIR;
DIR *messages_dir;

/* Window' management */
static window *rxwin, *txwin;
window *statwin;

PSK31info rxinfo, txinfo;

/* waterfall data */
float fftbuf[SAMPLES];

/* Current settings... */
static char callsign[40] = "NOCALL";     /* User's callsign */
static char beacon_callsign[40] = "NOCALL";
static int txtrack = 0;   /* Tx frequency follows rx frequency flag */
static float rxfreq = 0.0, txfreq = 0.0; /* Rx/Tx tone freqs (Hz) */
static int transmit=0, qpsk=0, lsb=0, dcd=0, dcdlevel=25, afc=1, phdelta=0, showgui=1;
static int tuning=0;

int OPTnew=0, exitfl=0;

static char macro[10][40];

void out_status(int force);

/* Use this variable to remember original terminal attributes. */
struct termios saved_attributes;


void reset_input_mode (void)
{
    tcsetattr (STDIN_FILENO, TCSANOW, &saved_attributes);
}


void set_input_mode (void)
{
    struct termios tattr;

    /* Make sure stdin is a terminal. */
    if (!isatty (STDIN_FILENO))
    {
        fprintf (stderr, "Not a terminal.\n");
        exit (EXIT_FAILURE);
    }
    /* Save the terminal attributes so we can restore them later. */
    tcgetattr (STDIN_FILENO, &saved_attributes);
    atexit (reset_input_mode);
    /* Set the funny terminal modes. */
    tcgetattr (STDIN_FILENO, &tattr);
    tattr.c_lflag &= ~(ICANON|ECHO); /* Clear ICANON and ECHO. */
    tattr.c_cc[VMIN] = 1;
    tattr.c_cc[VTIME] = 0;
    tcsetattr (STDIN_FILENO, TCSAFLUSH, &tattr);
}


void set_tx_freq(float freq)
{
    txfreq=freq;
    commControl(COMM_TXCH, COMM_FREQ, (int)(txfreq*100));
}


void set_rx_freq(float freq)
{
    rxfreq=freq;
    commControl(COMM_RXCH, COMM_FREQ, (int)(rxfreq*100));
    if( txtrack )
    {
        set_tx_freq(rxfreq);
    }
}


void change_freq(float offset)
{
    set_rx_freq(rxfreq+offset);
}


void setmode(int q, int l, int a, int d)
{
    qpsk=q;
    lsb=l;
    dcd=d;
    afc=a;
    commControl(COMM_TXCH, COMM_QPSK, qpsk);
    commControl(COMM_RXCH, COMM_QPSK, qpsk);
    commControl(COMM_TXCH, COMM_LSB, lsb);
    commControl(COMM_RXCH, COMM_LSB, lsb);
    commControl(COMM_RXCH, COMM_AFC, afc);
    commControl(COMM_RXCH, COMM_DCD, dcd);
}


static void readconfig(void)
{
    int cfgerr=0, len;
    FILE *fp=NULL;
    char cfgfile[256], section[40], buf[80], call[40], *ptr;
    float f;
    cfg_section_t cfgsection = CFG_NONE;

    /*
     * The ini file in $HOME/ overrides the one in $PKG_DATA_DIR file and
     * a psk31lx.ini in the current directory. So look for $HOME/.psk31lx.ini
     * first.  If not found, look in PKG_DATA_DIR/psk31lx.
     * If those two failed, look in the current directory.
     * Exit if none is found.
     */
    ptr=getenv("HOME");
    if(ptr != NULL)
    {   /* try $HOME/.psk31lx.ini first */
        strncpy(cfgfile, ptr, 256);
        strcat(cfgfile, "/.psk31lx.ini");
        fp = fopen(cfgfile, "r");
    }

    if(fp == NULL)
    {   /* if above open failed, try PKG_DATA_DIR/psk31lx.ini */
        strncpy(cfgfile, PKG_DATA_DIR, 256);
        if(cfgfile[strlen(cfgfile)-1] != '/')
        {
            strcat(cfgfile,"/");
        }
        strcat(cfgfile, "psk31lx.ini");
        fp = fopen(cfgfile, "r");
    }

    if(fp == NULL)
    {   /* if above two opens failed, try ./psk31lx.ini */
        strncpy(cfgfile, "./psk31lx.ini", 256);
        fp = fopen(cfgfile, "r");
    }

    if(fp == NULL)
    {   /* everything failed, so go away */
        fprintf(stderr, "\nConfiguration file %s/psk31lx.ini,\n"
                "~/.psk31lx.ini or ./psk31lx.ini was not found.\n"
                "Did you run \"make install\" as root?\n\n",
                PKG_DATA_DIR);
        exit(1);
    }
    fprintf (stderr, "cfgfile = %s\n", cfgfile);

    while(fgets(buf, sizeof(buf), fp) != NULL)
    {
        // Skip empty lines and comments.
        if (strlen(buf) == 0 || strchr(buf, ';') == buf || strchr(buf, '#') == buf) {
            continue;
        }
        if (index(buf, '[') == buf) {
            // Section heading
            char *stop = strrchr(buf, ']');
            if (stop == NULL) {
                fprintf(stderr,"Unterminated section heading in config file %s:\n%s\n", cfgfile, buf);
                continue;
            }
            len = stop-buf-1;
            strncpy(section, buf+1, len);
            section[len] = '\0';
            if (strcasecmp(section, "psk31lx") == 0) {
                cfgsection = CFG_PSK;
            }
            else if (strcasecmp(section, "flbeacon") == 0) {
                cfgsection = CFG_BEACON;
            }
            else {
                cfgsection = CFG_UNKNOWN;
            }
            continue;
        }
        if (cfgsection == CFG_NONE || cfgsection == CFG_UNKNOWN) {
            // We are not in a relevant section of the configuration file. Keep going.
            continue;
        }
        if(sscanf(buf, "CALL=\"%[^\"]", call) == 1) {
            switch (cfgsection) {
            case CFG_PSK:
                strcpy(callsign, call);
                break;
            case CFG_BEACON:
                strcpy(beacon_callsign, call);
                break;
            default:
                break;
            }
        }
        else if(sscanf(buf, "FREQ=%f", &f) == 1)
            rxfreq = txfreq = f;
        else if(sscanf(buf, "LSB=%d", &lsb) == 1)
            /**/;
        else if(sscanf(buf, "AFC=%d", &afc) == 1)
            /**/;
        else if(sscanf(buf, "DCDLEVEL=%d", &dcdlevel) == 1)
            /**/;
        else if(sscanf(buf, "NET=%d", &txtrack) == 1)
            /**/;
        else if(sscanf(buf, "GUI=%d", &showgui) == 1)
            /**/;
        else if(sscanf(buf, "MACRO1=\"%[^\"]", macro[0]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO2=\"%[^\"]", macro[1]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO3=\"%[^\"]", macro[2]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO4=\"%[^\"]", macro[3]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO5=\"%[^\"]", macro[4]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO6=\"%[^\"]", macro[5]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO7=\"%[^\"]", macro[6]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO8=\"%[^\"]", macro[7]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO9=\"%[^\"]", macro[8]) == 1 )
            /**/;
        else if(sscanf(buf, "MACRO10=\"%[^\"]", macro[9]) == 1 )
            /**/;
        else if(sscanf(buf, "DSPDEV=\"%[^\"]", audio_path) == 1 )
            /**/;
        else if(sscanf(buf, "PTTDEV=\"%[^\"]", ptt_path) == 1 )
            /**/;
        else if(sscanf(buf, "MESSAGEDIR=\"%[^\"]", messages_path) == 1 )
            /**/;
        else
        {
            fprintf(stderr,"Illegal line in config file%s:\n%s",
                    cfgfile, buf);
            cfgerr=1;
        }
    }
    fclose(fp);
    if(cfgerr) exit(1);
}


void usage_exit(char *argv0)
{
    fprintf(stderr,
            "%s\n\n"
            "Usage: %s [-a audiodevice] [-t serialdevice] [-m messagedirectory]"
            "[-v] [-q] [-l]] [-t] [-x] [-h]\n"
            "    -a pathname:   path to audio device\n"
            "    -t pathname:   path to PTT device\n"
            "    -m pathname:   path to message directory\n"
            "    -v:   print current version number\n"
            "    -q:   Enable QPSK mode\n"
            "    -l:   Invert phase shift for LSB mode\n"
            "    -x:   TX freq follows RX freq\n"
            "    -h:   print this help message\n"
            "\n", PACKAGE_STRING, argv0);
    exit(1);
}


void out_tuning(int);


void out_status(int force)
{
    static int oq,ol,odcd,ot,otr,oaf;
    static float orx,otx;
    char buf[128];

    out_tuning(force);

    if( orx!=rxfreq || otx!=txfreq || oq!=qpsk || ol!=lsb || odcd!=dcd
            || ot!=transmit || otr!=txtrack || oaf!=afc)
    {
        force=1;
    }
    if(!force) return;

    sprintf(buf,"  %4s%4s   DCD:%3s   RX=%6.1f   AFC:%3s   TX=%6.1f   NET:%3s"
            "  %2s", /* %c%c */
            qpsk?"QPSK":"BPSK",
            lsb?"/LSB":"/USB",
            dcd?"ON ":"OFF",
            rxfreq,
            afc?"ON ":"OFF",
            txfreq,
            txtrack?"ON ":"OFF",
            transmit?"TX":"RX"/*,*/
            /*OPTnew&1?'F':'-',*/
            /*OPTnew&2?'S':'-'*/);
    orx=rxfreq;
    otx=txfreq;
    oq=qpsk;
    ol=lsb;
    odcd=dcd;
    ot=transmit;
    otr=txtrack;
    oaf=afc;
    window::outputline(STATSTART, buf);
}


void out_tuning(int force)
{
    static int oldp=-1;
    char wfout[22];
    float level[] = { 15.0, 10.0, 5.0, 3.0, 1.5 };
    //float level[] = { 10.0, 8.0, 6.0, 2.0, 2.0 };
    int offset;
    int l, p, i, j;
    int spaces = 10;
    char vert[] = "|";                     /* use with x offset of 10 */
    char base[] = "---------------------";

    /* output spectrum analyzer */
    offset = rxfreq / SAMPLE_RATE * SAMPLES - 10;
    l = commGetData (COMM_FFTCH, (char *)fftbuf, sizeof(fftbuf));
    i = 5;
    if (l == SAMPLES)
    {
        for (i=0; i<5; i++)
        {
            for (j=0; j<21; j++)
            {
                if (fftbuf[offset + j] > level[i])
                {
                    wfout[j] = '.';
                }
                else
                {
                    wfout[j] = ' ';
                }
            }
            wfout[j] = '\0';
            window::putsyx(STATSTART + i + 1, spaces, wfout);      /* wf data  */
            window::putsyx(STATSTART + i + 1, spaces + 10, vert);  /* vert bar */
        }
    }
    window::putsyx(STATSTART + i + 1, spaces, base);

    /*output the phase scope */
    if(oldp==-1||force)
    {
        window::putsyx(STATSTART + 2, 0, "  .....  ");
        window::putsyx(STATSTART + 3, 0, " .     . ");
        window::putsyx(STATSTART + 4, 0, ".       .");
        window::putsyx(STATSTART + 5, 0, " .     . ");
        window::putsyx(STATSTART + 6, 0, "  .....  ");
    }
    /* phdelta: 0..255 */
    /* Position:     14 15 0 1 2 ... */
    int pos[][2]= {
        {2,4}, {2,5}, {2,6}, {3,7}, {3,7}, {4,8}, {5,7}, {5,7},{6,6},{6,5},
        {6,4}, {6,3}, {6,2}, {5,1}, {5,1}, {4,0}, {3,1}, {3,1},{2,2},{2,3}
    };
    p=(int)((phdelta/256.0)*20+0.5);   /* 0..19 */
    p=(int)((phdelta/256.0)*20.5);     /* 0..19 */
    if(p>19) p=0;
    if(p!=oldp || force)
        window::putsyx(STATSTART+pos[p][0], pos[p][1], "O");
    if(p!=oldp && oldp!=-1) {
        window::putsyx(STATSTART+pos[oldp][0], pos[oldp][1], ".");
        oldp=p;
    }
}


void out_macroinfo()
{
    char buf[128];
    int spaces = 33;

    sprintf(buf," F1: %-15s  F2: %-15s\n", macro[0], macro[1]);
    window::putsyx(STATSTART+2, spaces, buf);
    sprintf(buf," F3: %-15s  F4: %-15s\n", macro[2], macro[3]);
    window::putsyx(STATSTART+3 ,spaces, buf);
    sprintf(buf," F5: %-15s  F6: %-15s\n", macro[4], macro[5]);
    window::putsyx(STATSTART+4, spaces, buf);
    sprintf(buf," F7: %-15s  F8: %-15s\n", macro[6], macro[7]);
    window::putsyx(STATSTART+5, spaces, buf);
    sprintf(buf," F9: %-15s  F10:%-15s\n", macro[8], macro[9]);
    window::putsyx(STATSTART+6, spaces, buf);
}


/* If we are not transmitting:
 *   - turn on PTT, transmit CW data, turn off PTT
 * if PTT is already on:
 *   - switch mode to CW, transmit CW data
 *   - if pttoff==0:  leave PTT enabled, switch back to PSK31 mode
 *   - if pttoff==1:  turn of PTT after finishing transmisson
 */
void cwsend(char *s, int pttoff)
{
    if(tuning) tuning=0;
    commControl(COMM_TXCH, COMM_MODE, MO_CWSEND);
    if(!transmit)
    {
        pttoff=1;
        commControl(COMM_TXCH, COMM_PTT, PTTON|PTTFORCE);
    }
    commPutData(s, 0);
    if(pttoff)
    {
        fprintf(stderr,"turning off PTT!\n");
        commControl(COMM_TXCH, COMM_PTT, PTTOFF);
    }
    commControl(COMM_TXCH, COMM_MODE, MO_NORMAL);
}


void toggle_tune()
{
    if(tuning)
    {
        commControl(COMM_TXCH, COMM_PTT, PTTOFF);
        commControl(COMM_TXCH, COMM_MODE, MO_NORMAL);
        tuning = 0;
    }
    else
    {
        commControl(COMM_TXCH, COMM_MODE, MO_TUNE );
        if(!transmit)
            commControl(COMM_TXCH, COMM_PTT, PTTON|PTTFORCE);
        tuning = 1;
    }
}


void update_crc(char u, char *crc) {
    int update, j;

    for (j=1; j<=8; j++) {
        update = *crc & 0x80;
        *crc <<= 1;
        *crc |= (u >> (8-j)) & 0x01;
        if (update) {
            *crc ^= CRC_POLYNOMIAL;
        }
    }
}


int tx_append_file_to_buffer(char *file_name) {
    char c;
    char crc = 0x00;
    char footer[3];

    int fd = open(file_name, O_RDONLY);
    if (fd == -1) {
        perror("open");
        return 1;
    }
    /* Wait until TX is idle. */
    pthread_mutex_lock(&mutex_tx_start);
    while (psk31tx->get_tx_state() != TS_INACTIVE) {
        pthread_cond_wait(&cond_tx_start, &mutex_tx_start);
    }
    pthread_mutex_unlock(&mutex_tx_start);
    // Enable transmit.
    commControl(COMM_TXCH, COMM_PTT, PTTON);
    while (read(fd, &c, 1)) {
        if (c == 0x0a || c == 0x0ad) {
            continue;
        }
        commPutData(&c, 1);
        update_crc(c, &crc);
    }
    close(fd);
    update_crc(0x00, &crc);
    footer[0] = ';';
    footer[1] = crc;
    footer[2] = '\0';
    commPutData(footer, 0);
    printf("Sending file %s. CRC: 0x%02hhx\n", file_name, crc);
    /* Wait until transmission has started before sending instruction to disable it again. */
    pthread_mutex_lock(&mutex_tx_stop);
    while (psk31tx->get_tx_state() != TS_TRANSMIT) {
        pthread_cond_wait(&cond_tx_stop, &mutex_tx_stop);
    }
    commControl(COMM_TXCH, COMM_PTT, PTTOFF);
    pthread_mutex_unlock(&mutex_tx_stop);

    return 0;
}


int tx_dir_filter(const struct dirent *entry) {
    if (strstr(entry->d_name, ".txt") != NULL) {
        return 1;
    }
    return 0;
}


void tx_check_outgoing(void) {
    struct dirent **text_files;
    char cwd[48];
    char file_name[32];
    int n, rc;

    getcwd(cwd, sizeof(cwd));
    chdir(messages_path);
    chdir("TX");
    n = scandir("./", &text_files, tx_dir_filter, alphasort);
    if (n < 0) {
        chdir(cwd);
        return;
    }
    while (n--) {
        strcpy(file_name, text_files[n]->d_name);
        free(text_files[n]);
        rc = tx_append_file_to_buffer(file_name);
        if (rc) {
            fprintf(stderr, "Error transmitting file %s\n", file_name);
        }
        if (unlink(file_name)) {
            perror("unlink");
        }
    }
    free(text_files);
    chdir(cwd);
}

void *tx_monitor(void *sigset) {
    siginfo_t siginfo;
    int s;

    /* Wait for ALRM signal - which will be delivered whenever the timer expires. */
    while (!exitfl) {
        s = sigwaitinfo((sigset_t *)sigset, &siginfo);
        if (s == -1) {
            fprintf(stderr, "Error waiting for timer signal. ");
            perror("sigwaitinfo");
        }
        else {
            tx_check_outgoing();
        }
    }

    return NULL;
}

int rx_init(rx_ctx_t *rx) {
    char cwd[48];

    rx->ptr = 0;
    memset(rx->buf, 0, sizeof(rx->buf));
    rx->timer = 0;
    getcwd(cwd, sizeof(cwd));
    chdir(messages_path);
    chdir("RX");
    rx->log_fp = fopen("received.log", "a");
    if (rx->log_fp == NULL) {
        fprintf(stderr, "Could not open receive log file!\n");
        return 1;
    }

    chdir(cwd);
    return 0;
}

void rx_deinit(rx_ctx_t *rx) {
    fclose(rx->log_fp);
}

void rx_save_message(char *msg) {
    char cwd[48];
    int rc;
    
    getcwd(cwd, sizeof(cwd));
    chdir(messages_path);
    chdir("RX");
    FILE *fp = fopen(RX_BEACON_FILENAME, "w");
    if (fp == NULL) {
        fprintf(stderr, "Could not save beacon message to file. ");
        perror("fopen");
        chdir(cwd);
        return;
    }
    rc = fprintf(fp, "%s\n", msg);
    if (rc < 0) {
        fprintf(stderr, "Could not save beacon message to file. ");
        perror("fprintf");
    }
    fclose(fp);

    chdir(cwd);
}

void rx_log_message(rx_ctx_t *rx, char *msg, int crc_present, int crc_ok, char crc) {
    char timestamp[24];
    int rc;
    time_t now;
    struct tm tm_now;

    time(&now);
    localtime_r(&now, &tm_now);

    sprintf(timestamp, "%04d-%02d-%02d:%02d:%02d:%02d", 
                tm_now.tm_year+1900, tm_now.tm_mon+1, tm_now.tm_mday,
                tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    if (crc_present) {
        rc = fprintf(rx->log_fp, "%s - %s - %s (0x%02hhx)\n", timestamp, msg, crc_ok ? "CRC OK" : "CRC failed", crc);
    }
    else {
        rc = fprintf(rx->log_fp, "%s - %s (no CRC)\n", timestamp, msg);
    }

    fflush(rx->log_fp);
    if (rc < 0) {
        fprintf(stderr, "Could not save message to log file. ");
        perror("fprintf");
    }
}

void rx_handle_packet(rx_ctx_t *rx) {
    int len = rx->ptr;
    int i;
    char crc_sent, crc_calc = 0x00;
    if (len >= sizeof(rx->buf)) {
        len = sizeof(rx->buf)-1;
    }
    rx->buf[len] = '\0'; // Null-terminate string before printing.
    if (!strstr(rx->buf, beacon_callsign)) {
        printf("Ignoring received message with unknown callsign: %s\n", rx->buf);
        return;
    }
    if (len >= 2 && rx->buf[len-2] == ';') {
        crc_sent = rx->buf[len-1];
        len -= 2;
        rx->buf[len] = '\0';
        for (i=0; i<len; i++) {
            update_crc(rx->buf[i], &crc_calc);
        }
        update_crc(0x00, &crc_calc);
        if (crc_calc == crc_sent) {
            printf("Received: %s (CRC OK: 0x%02hhx)\n", rx->buf, crc_sent);
            rx_save_message(rx->buf);
            rx_log_message(rx, rx->buf, 1, 1, crc_calc);
        }
        else {
            printf("Received: %s (CRC not OK. Sent: 0x%02hhx. Expected: 0x%02hhx)\n", rx->buf, crc_sent, crc_calc);
            rx_log_message(rx, rx->buf, 1, 0, crc_sent);
        }
    }
    else {
        printf("Received: %s (no CRC)\n", rx->buf);
        rx_log_message(rx, rx->buf, 0, 0, crc_calc);
    }
}

int main(int argc, char **argv)
{
    int i, res, c, cnt;
    fd_set rset, wset, eset;
    struct timeval tm;
    char cwd[48];
    pthread_t tx_monitor_t;
    sigset_t sigset;
    rx_ctx_t rx;

    srandom(time((time_t *)0));
    readconfig();
    /* parse command line arguments */
    //
    //TJW while( (c=getopt(argc, argv, "vFSa:m:t:n:iqlxdT:?"))!=-1 )
    //
    while( (c=getopt(argc, argv, "a:t:m:vqlxh"))!=-1 )
    {
        switch(c)
        {
        case 'a':
            strncpy(audio_path, optarg, 20);
            break;
        case 't':
            strncpy(ptt_path, optarg, 20);
            break;
        case 'm':
            strncpy(messages_path, optarg, 48);
            break;
        case 'v':
            printf (PACKAGE_STRING"\n");
            exit (1);

#if 0 //TJW not using these
        case 'F':
            OPTnew|=1;  // neuer filter
            break;
        case 'S':
            OPTnew|=2;  // neue sync.
            break;
        case 'i':
            io_file=1;
            break;
        case 'T':
            transmit=1;
            break;
#endif

        case 'q':
            qpsk=1;
            break;
        case 'l':
            lsb=1;
            break;
        case 'x':
            txtrack=1;
            break;
        case 'h':
            usage_exit(argv[0]);
            break;
        default:
            fprintf(stderr,"illegal option: %c\n",c);
            usage_exit(argv[0]);
            break;
        }
    }

    // We don't want to receive ALRM signals in this thread - so block them.
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGALRM);
    res = pthread_sigmask(SIG_BLOCK, &sigset, NULL);
    if(res) {
        fprintf(stderr, "Could not set signal mask for main thread. ");
        perror("pthread_sigmask");
        exit(1);
    }

    if (server_main(audio_path, ptt_path, PKG_DATA_DIR))
    {
        exit(0);
    }

    /* Ensure we can open the folder where messages are stored */
    if (!(messages_dir = opendir(messages_path))) {
        fprintf(stderr, "Could not open message directory %s. ", messages_path);
        perror("opendir");
        exit(1);
    }
    /* ... and the TX and RX subfolders. */
    getcwd(cwd, sizeof(cwd));
    chdir(messages_path);
    if (mkdir("TX", S_IRWXU) == -1) {
        if (errno != EEXIST) {
            fprintf(stderr, "Could not create TX sub-folder in message directory %s ! Aborting ...\n", messages_path);
            exit(1);
        }
    }
    if (mkdir("RX", S_IRWXU) == -1) {
        if (errno != EEXIST) {
            fprintf(stderr, "Could not create RX sub-folder in message directory %s ! Aborting ...\n", messages_path);
            exit(1);
        }
    }
    chdir(cwd);

    /* Install timer to periodically check for outgoing messages */
    struct itimerval it = {
        .it_interval = { .tv_sec = TX_POLL_INTERVAL, .tv_usec = 0},
        .it_value = { .tv_sec = TX_POLL_INTERVAL, .tv_usec = 0}
    };
    setitimer(ITIMER_REAL, &it, NULL);


    if (!showgui) {
        fprintf(stderr, "Starting thread for TX monitor ...\n");
        res = pthread_create(&tx_monitor_t, NULL, &tx_monitor, &sigset);
        if (res) {
            fprintf(stderr, "Could not create thread for TX monitor. ");
            perror("pthread_create");
            exit(1);
        }
        /* Initialize receiver context */
        res = rx_init(&rx);
        if (res) {
            fprintf(stderr, "Failed to initialize receiver context. Exiting ...\n");
            exit(1);
        }
    }

    // Set default parameters
    setmode(qpsk,lsb,afc,dcd);
    set_rx_freq(rxfreq);
    set_tx_freq(txfreq);
    /* TJW set dcdlevel to something nice */
    commControl (COMM_RXCH, COMM_DCDLEVEL, dcdlevel);

    printf("Starting up ...\n");
    printf("Parameters: AFC=%d, GUI=%d, DCD level=%d, RX frequency=%.2f, TX freq=%.2f\n",
            afc, showgui, dcdlevel, rxfreq, txfreq);

    if (showgui) {
        /* init screen */
        window::init_window();

        atexit(window::end_window);
        txwin=new window(TXSTART, TXLINES, 80, 1);

        window::outputline(LINE1,"________________________________________________________________________________");

        rxwin=new window(RXSTART, RXLINES, 80, 0);
        window::outputline(LINE2,"________________________________________________________________________________");
        statwin=new window(STATSTART, STATLINES, 80, 0);
        out_status(1);
        out_macroinfo();
    }

    /* go into main loop */
    while(!exitfl)
    {
        fflush(stdout);
        FD_ZERO(&rset);
        FD_ZERO(&wset);
        FD_ZERO(&eset);
        FD_SET(STDIN_FILENO, &rset);
        tm.tv_sec=0;
        tm.tv_usec=50000; /* 50ms */
        res=select(17, &rset, &wset, &eset, &tm);

        if (showgui) {
            if(FD_ISSET(STDIN_FILENO, &rset))
            {
                /* handle keyboard input */
                c=getch();
                switch(c)
                {
                    case 0:
                    case 3:
                    case 4:
                    case 5:
                    case 6:
                    case 15:
                    case 16:
                    case 17:
                    case 19:
                    case 22:
                    case 25:
                    case 26:
                    case 28:
                    case 29:
                    case 30:
                    case 31:
                        c=0; /* ignore */
                        break;
                    case KEY_F(9):
                    case KEY_F(1):
                    case KEY_F(2):
                    case KEY_F(3):
                    case KEY_F(4):
                    case KEY_F(5):
                    case KEY_F(6):
                    case KEY_F(7):
                    case KEY_F(8):
                        {
                            char *txt = macro[c-KEY_F0-1];
                            commPutData( txt, 0 );
                            txwin->put_string( txt );
                            c=0;
                            break;
                        }
                        //exit (0);/* c=0; break; */
                    case 1:
                        /* ^A -> afc on/off */
                        setmode(qpsk,lsb,!afc,dcd);
                        c=0;
                        break;
                    case 2:
                        /* ^B -> QPSK <> BPSK */
                        setmode(!qpsk,lsb,afc,dcd);
                        c=0;
                        break;
                    case 7:
                    case 9:
                    case 10:
                    case 13:
                        break;            /* ok (BEL,TAB,NL,CR) */
                    case 8:
                    case 127:
                    case KEY_BACKSPACE:
                        c='\b';
                        break;    /* Backspace */
                    case 11:
                        /* ^K -> txtrack on/off */
                        txtrack=!txtrack;
                        out_status(0);
                        c=0;
                        break;
                    case 12:
                        /* ^L -> LSB <> USB */
                        setmode(qpsk, !lsb, afc, dcd);
                        c=0;
                        break;
#if 0
                    case 14:
                        /* ^N -> toggle OPTnew (new filter, bitsync)*/
                        OPTnew++;
                        if(OPTnew>=4) OPTnew=0;
                        out_status(1);
                        c=0;
                        break;
#endif
                    case 18: /* ^R -> Receive */
                        if(tuning)
                            toggle_tune();
                        else
                            commControl(COMM_TXCH, COMM_PTT, PTTOFF);
                        c=0;
                        transmit = 0;
                        break;
                    case 20:  /* ^T -> Transmit */
                        if(tuning)
                        {
                            commControl(COMM_TXCH, COMM_MODE, MO_NORMAL);
                            tuning = 0;
                        }
                        else
                        {
                            commControl(COMM_TXCH, COMM_PTT, PTTON|PTTFORCE);
                        }
                        c=0;
                        transmit = 1;
                        break;
                    case 21:
                        /* ^U -> Tune on/off */
                        toggle_tune();
                        c=0;
                        break;
                    case 23:
                        /* ^W -> CWID */
                        char buffer[128];
                        sprintf(buffer," de %s",callsign);
                        cwsend(buffer, 1);
                        c=0;
                        break;
                    case 24:
                        /* ^X -> clear windows the hard way */
                        for (i=0; i<RXLINES; i++)
                        {
                            rxwin->put_char('\n');
                        }
                        for (i=0; i<TXLINES; i++)
                        {
                            txwin->put_char('\n');
                        }
                        window::outputline(LINE1, "________________________________________________________________________________");
                        window::outputline(LINE2,"________________________________________________________________________________");
                        out_macroinfo();
                        c=0;
                        break;
                    case 27:
                        /* ESC -> Exit */
                        exit (0);
                    case KEY_UP:
                        change_freq(1);
                        c=0;
                        break;
                    case KEY_DOWN:
                        change_freq(-1);
                        c=0;
                        break;
                    case KEY_RIGHT:
                        change_freq(8);
                        c=0;
                        break;
                    case KEY_LEFT:
                        change_freq(-8);
                        c=0;
                        break;
                }
                if(c)
                {
                    char ch = (char)c;
                    commPutData(&ch, 1);
                }
                if(c>0&&c<256)
                    txwin->put_char(c);
            }
        }

        commGetInfo(COMM_TXCH, &txinfo, sizeof(txinfo));
        commGetInfo(COMM_RXCH, &rxinfo, sizeof(rxinfo));
        rxfreq = 0.01 * rxinfo.freq;
        dcd = rxinfo.dcd;
        phdelta = rxinfo.phdelta;

        if (showgui) {
            // handle echo!  we could use a different color?
            char echoBuf[256], rxBuf[256];
            int echoLen = commGetData(COMM_ECHOCH, echoBuf, sizeof echoBuf);
            // handle receive!
            int rxLen = commGetData(COMM_RXCH, rxBuf, sizeof rxBuf);

            out_status(1);
            if(echoLen>0)
            {
                for(int i=0; i<echoLen; i++) {
                    rxwin->put_char(echoBuf[i]);
                }
            }
            if(rxLen>0)
            {
                for(int i=0; i<rxLen; i++) {
                    rxwin->put_char(rxBuf[i]);
                }
            }
        }
        else {
            // Leave space for null terminator.
            cnt = commGetData(COMM_RXCH, rx.buf+rx.ptr, sizeof(rx.buf)-rx.ptr-1);
            if (cnt > 0) {
                // New data has been received
                rx.ptr = rx.ptr + cnt;
                rx.timer = 0;
            }
            else if (rx.ptr > 0) {
                // Reception has started but no new data on this iteration.
                rx.timer += 50000;
                if (rx.timer >= RX_MAX_WAIT) {
                    rx_handle_packet(&rx);
                    rx.ptr = 0;
                    rx.timer = 0;
                }
            }
        }

    }

    rx_deinit(&rx);
    pthread_join(tx_monitor_t, NULL);

    return 0;
}



