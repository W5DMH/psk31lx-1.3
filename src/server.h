#ifndef _USERIF_H_INCLUDED
#define _USERIF_H_INCLUDED

/* server-api:
 * auf senden schalten:
 *    modus angeben   PSK(preamble,idle) CW(nix,ruhe), TUNE(nix,carrier)
 *    sofort oder bei DCD off angeben
 * senden ausschalten:
 *    mit postamble oder ohne...
 *    nach tx buffer ende
 *    txbuffer leeren, ggf. postamble, dann abschalten
 *    sofort abschalten
 */

#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/soundcard.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include "psk31-receiver.h"
#include "psk31-transmitter.h"
#include "psk31-fft.h"
#include "psk31-coder.h"

#define STEREO 1
#define MONO   0

#define COMM_PTT	0x01
#define COMM_DCD	0x02
#define COMM_DCDLEVEL	0x03
#define COMM_QPSK	0x04
#define COMM_LSB	0x05
#define COMM_AFC	0x06
#define COMM_MODE	0x07
#define COMM_FREQ	0x08
#define COMM_FFTN	0x20
#define COMM_FFTOVERLAP 0x21
#define COMM_SWAP       0x40

#define COMM_TXCH	0xFF
#define COMM_ECHOCH     0x00
#define COMM_FFTCH	0x01
#define COMM_RXCH	0x02

#define PTTOFF		0
#define PTTON		1
#define PTTFORCE	0x10

#define MO_NORMAL	0x01     /* PSK31 RX or TX */
#define MO_CWSEND	0x02     /* only TX channel: send CW */
#define MO_TUNE		0x04     /* only TX channel: send tune carrier */
#define MO_DISABLED	0x80     /* disable channel. only supp. for 2nd RX */
#define SAMPLES         1024     /* fft samples - put in ini file? */

// spec:
// COMM_PTT       ON OFF PTT_FORCE
// COMM_DCD       ON OFF
// COMM_DCDLEVEL  0..100 (?)
// COMM_QPSK      ON OFF
// COMM_LSB       ON OFF
// COMM_AFC       ON OFF
// COMM_MODE      MO_DISABLED MO_NORMAL (TXonly:) MO_CWSEND  MO_TUNE
// COMM_FFTN        128..4096
// COMM_FFTOVERLAP  0..FFTN, evtl größer
// channel: -1=TX, sonst RX
// channel:  TXCH, RXCH, FFTCH
// value: -1= nix ändern, sonst value; return: aktueller value oder -1


typedef struct {
    int freq;       // IF frequence (RX,TX)
    int qpsk;         // QPSK <-> BPSK (RX,TX)
    int lsb;          // LSB(inverted)  (RX,TX)
    int cw;           // 0=normal PSK mode  1=transmit as CW  2=tuning(TX)
    int dcd;          // DCD active (RX)
    int dcdlevel;     // DCD sensitifity (-1=disable DCD,  (RX)
    // 0=only preamble/postamble DCD)
    int strength;     // range: 0..40
    int afc;          // use AFC (RX)
    int mode;         // ??
    int ptt;          // PTT on??? (transmitting?)  (all CH)
    int phdelta;      // last phase change (RX)
    float qty[4];     // quality (current,bpsk,qpsk,qpsk-inverted)
} PSK31info;

int commWaitUpdate(unsigned long timeout);
int commGetData(int channel, char *buffer, int buflen);
int commPutData(char *buffer, int buflen);
int commControl(int channel, int spec, int value);
int commGetInfo(int channel, void *buffer, int buflen);

// for the non-socket server...
int server_main(char *audio, char *ptt, char *datadir);

#endif
