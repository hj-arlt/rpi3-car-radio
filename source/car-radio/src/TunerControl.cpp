#include <QDebug>
#include <QCoreApplication>

#include "TunerControl.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>             /* getopt_long() */
#include <errno.h>              /* errno */
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/rtc.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <pthread.h>

#include "RdsList.h"

#if defined(__cplusplus) || defined(c_plusplus)
};
#endif

namespace
{

#include "TunerDefines.h"

#define TYPE_UNKNOWN           0
#define TYPE_TDA77XX           1
#define TYPE_SI4703            2

#define TDA77XX_DEVICE         "/dev/spidev0.0"
#define SI4703_DEVICE          "/dev/i2c-1"
#define SI4703_ADDRESS         0x10
#define SI4703_CHIPID          0x1242

#define FREQ_DEFAULT           90400

typedef struct {
        uint8_t      band;
        uint32_t     freq;
        int8_t       fstRF;  // RF fieldstrenth       // int8_t dByV
        int8_t       fstBB;  // BaseBand fieldstrenth // int8_t dByV
        uint8_t      mp_adj; // multipath adjust      // uint8_t %
        uint16_t     pi;
        char         name[32];
        bool         activ;
} radio_station;

#define MAX_STATIONS (64*4)

typedef struct {
        int          type;
        uint16_t     chipid;
        char        *TDA7707Image;
        int          fd;
        char         device[20];
        int          state;
        uint32_t     freq;
        uint8_t      mute;
        uint8_t      band;
        uint32_t     band_low;
        uint32_t     band_high;
        int8_t       thres_fstRF;
        int8_t       thres_fstBB;
        uint8_t      thres_mp_adj;
        int32_t      thres_det;
        int8_t       fstRF[2];  // forg/back RF fieldstrenth        dByV
        int8_t       fstBB[2];  // forg/back BaseBand fieldstrenth  dByV
        radio_station  station[MAX_STATIONS];
        int          nb_stations;
        int          last_station;
} radio_param;

typedef struct {
        uint32_t     notify;
        uint16_t     pi;
        uint8_t      ta;
        uint8_t      tmc;
        uint8_t      utc;
        uint16_t     data[4];
        int32_t      freq;
        char         name[32];
        char         text[65];
        char         save[65];
} rds_buffer;

static radio_param radio;
static rds_buffer  rds;

#define BUFFER_MAX  128

static unsigned char txBuffer[0x5000];     // size !!
static unsigned char rxBuffer[BUFFER_MAX];

uint16_t buf[16];
uint16_t reg[16];

/* Just be aware that both usleep() and nanosleep() can be interrupted by a signal. nanosleep() lets
 * you pass in an extra timespec pointer where the remaining time will be stored if that happens. So
 * if you really need to guarantee your delay times, you'll probably want to write a simple wrapper
 *  around nanosleep().
 */
int sDelay (unsigned int value)
{
        struct timespec req;
        req.tv_sec = 0;//sec;
        req.tv_nsec = value*100000;//nanosec;   0 to 999.999.999

        usleep(value *1000);
        return 0;

        /* Loop until we've slept long enough */
        do
        {
                /* Store remainder back on top of the original required time */
                if( 0 != nanosleep( &req, &req ) )
                {
                        /* If any error other than a signal interrupt occurs, return an error */
                        if(errno != EINTR)  return -1;
                }
                else
                {
                        /* nanosleep succeeded, so exit the loop */
                        break;
                }
        } while( req.tv_sec > 0 || req.tv_nsec > 0 );

        return 0;
}

/* ***************************************************************
 * GPIO
 * ***************************************************************/

int gpioExport (int pin, const char *mode)
{
  FILE *fd ;
  char fName [128] ;

  if (pin == 0)
    return -1;
  if ((fd = fopen ("/sys/class/gpio/export", "w")) == NULL)
    return -1;
  fclose (fd) ;

  sprintf (fName, "/sys/class/gpio/gpio%d/direction", pin) ;
  if ((fd = fopen (fName, "w")) == NULL)
    return -1;

  if ((strcasecmp (mode, "in")   == 0) || (strcasecmp (mode, "input")  == 0))
    fprintf (fd, "in\n") ;
  else if ((strcasecmp (mode, "out")  == 0) || (strcasecmp (mode, "output") == 0))
    fprintf (fd, "out\n") ;
  else if ((strcasecmp (mode, "high") == 0) || (strcasecmp (mode, "up")     == 0))
    fprintf (fd, "high\n") ;
  else if ((strcasecmp (mode, "low")  == 0) || (strcasecmp (mode, "down")   == 0))
    fprintf (fd, "low\n") ;
  else
    return -1;

  fclose (fd) ;
  return 0;
}

int gpioWrite(int pin, int value)
{
    FILE *fd ;
    char fName [128] ;

    sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
    if ((fd = fopen (fName, "w")) == NULL)
    {
      qDebug() << " pin " << pin << " wr in use";
      return -1;
    }
    if (value == 0)
        fprintf (fd, "0\n") ;
    else if (value == 1)
        fprintf (fd, "1\n") ;
    else
        goto gerr;

    fclose (fd) ;
    return 0;
gerr:
    fclose (fd) ;
    return -1;
}

void gpioOut(int pin, int val)
{
    gpioExport (pin, "out");
    gpioWrite(pin, val);
}

/* ***************************************************************
 * SPI
 * ***************************************************************/

uint8_t spimode;
uint32_t spispeed;
uint8_t  spibits;
static uint32_t dwParam[16];

static int spiInit(char *dev, uint8_t mode, uint32_t speed, uint8_t bits)
{
        int fd;
        if ((fd = open (dev, O_RDWR)) < 0) {
//              qDebug() << "ERR: dev " << dev << " open";
                return -1;
        }
        mode    &= 3 ;    // Mode is 0, 1, 2 or 3
        spimode = mode;
        spispeed = speed;
        spibits = bits;
        if (ioctl (fd, SPI_IOC_WR_MODE, &mode)  < 0) {
                return -2;
        }

        if (ioctl (fd, SPI_IOC_RD_MODE, &mode)  < 0) {
                return -2;
        }

        if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
                return -3;
        }

        if (ioctl (fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
                return -3;
        }

        if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
                return -4;
        }

        if (ioctl (fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
                return -4;
        }
        return fd;
}

int spiDataRW (int fd, uint8_t *dataout, int lenout, uint8_t *datain, int lenin)
{
        int ret;
        struct spi_ioc_transfer spi ;

        memset (&spi, 0, sizeof (spi)) ;

        spi.tx_buf        = (unsigned long)dataout ;
        spi.rx_buf        = (unsigned long)datain ;
        spi.len           = lenout + lenin ;
        spi.delay_usecs   = 0;
        spi.speed_hz      = spispeed ;
        spi.bits_per_word = spibits ;
        spi.cs_change     = 0; // CS high ok.
        ret = ioctl (fd, SPI_IOC_MESSAGE(1), &spi) ;
        if (ret < 0) {
                return -1;
        }
        return 0;
}

/* ***************************************************************
 * I2C
 * ***************************************************************/

/*
* RD automatic from reg 10 ..15, 0,1..9
*/
int i2cReadRegisters(void)
{
    int r, i, err = 0;

    radio.fd = open(SI4703_DEVICE, O_RDWR);
    if(radio.fd < 0){
        qDebug() << "TunerControl: ERR opening i2c device " << SI4703_DEVICE;
        return -1;
    }
    r = ioctl(radio.fd, I2C_SLAVE, SI4703_ADDRESS);
    if(r < 0) {
        qDebug() << "ERR: selecting i2c device on" << SI4703_ADDRESS;
        err = -1;
        goto rdex;
    }
    r = read(radio.fd, buf, 32);

    //reorg the values to match the spec sheet
    //also swap the first 8 bits with the last 8 bits

    reg[0]=(buf[6]>>8)|(buf[6]<<8);
    reg[1]=(buf[7]>>8)|(buf[7]<<8);
    reg[2]=(buf[8]>>8)|(buf[8]<<8);
    reg[3]=(buf[9]>>8)|(buf[9]<<8);
    reg[4]=(buf[10]>>8)|(buf[10]<<8);
    reg[5]=(buf[11]>>8)|(buf[11]<<8);
    reg[6]=(buf[12]>>8)|(buf[12]<<8);
    reg[7]=(buf[13]>>8)|(buf[13]<<8);
    reg[8]=(buf[14]>>8)|(buf[14]<<8);
    reg[9]=(buf[15]>>8)|(buf[15]<<8);
    reg[10]=(buf[0]>>8)|(buf[0]<<8);
    reg[11]=(buf[1]>>8)|(buf[1]<<8);
    reg[12]=(buf[2]>>8)|(buf[2]<<8);
    reg[13]=(buf[3]>>8)|(buf[3]<<8);
    reg[14]=(buf[4]>>8)|(buf[4]<<8);
    reg[15]=(buf[5]>>8)|(buf[5]<<8);

    radio.chipid = reg[0];

    //qDebug() << "SI4703: id " << hex << radio.chipid;
    for (i=0; i<16; i++) {
        //qDebug() << "SI4703: reg " << i << " val " << hex << reg[i];
    }

    err = 0;
rdex:
    close(radio.fd);
    return err;
}

/*
* WR automatic from reg2 ..15, 0,1
*/
int i2cWriteRegisters(void)
{
    int r, i, err = 0;
    uint16_t newreg[6];

    radio.fd = open(SI4703_DEVICE, O_RDWR);
    if(radio.fd < 0){
        qDebug() << "ERR: opening i2c device node";
    }
    r=ioctl(radio.fd, I2C_SLAVE, SI4703_ADDRESS);
    if(r < 0) {
        qDebug() << "ERR: selecting i2c device";
    }
    r=0;
    // convert back to big endian
    for(i=0x02;i<0x08;i++) {
        newreg[r]=(reg[i]>>8)|(reg[i]<<8);
        r++;
    }
    if(write(radio.fd,newreg,12) < 12) {
        qDebug() << "could not write to device\n";
        err = -1;
    }
    close(radio.fd);
    return err;
}

/* ***************************************************************
 * Tuner common
 * ***************************************************************/
int RadioReset(void)
{
        gpioOut(16, 0);
        sDelay(5); // 5ms
        gpioOut(16, 1);

        qDebug() << " reset..";
        sDelay(100); // 100ms
        return 0;
}

void clearStations(void)
{
    /* clear scan tables of stations */
    memset(radio.station, 0, sizeof(radio_station) * MAX_STATIONS);
    radio.nb_stations = 0;
}

int addStation(radio_station *station)
{
    int i;

    for (i=0; i<radio.nb_stations; i++) {
        /* actualize */
        if (radio.station[i].freq == station->freq) {
            radio.station[i].fstRF = station->fstRF;
            radio.station[i].fstBB = station->fstBB;
            radio.station[i].activ = true;
            break;
        }
    }
    /* add new station, pi and name unknown now */
    if ((i == radio.nb_stations) && (i < MAX_STATIONS)) {
        radio.station[i].freq = station->freq;
        radio.station[i].fstRF = station->fstRF;
        radio.station[i].fstBB = station->fstBB;
        if (! radio.station[i].name)
            strcpy(radio.station[i].name, "NONAME");
        radio.station[i].activ = true;
        radio.nb_stations++;
        qDebug() << "TunerControl: new station " << radio.station[i].freq << " " << radio.station[i].name;
        return 1;
    }
    return 0;
}

char *getNameOfId(uint16_t id)
{
        int i = 0;
        while(pi_data[i].pi) {
                if (pi_data[i].pi == id)
                        return (char*)pi_data[i].name;
                i++;
        }
        return NULL;
}

void setActualStation(void)
{
    int i;

    if (rds.pi == 0)
        return;

    for (i=0; i<radio.nb_stations; i++) {
        if (radio.station[i].freq == radio.freq) {
            radio.station[i].pi = rds.pi;
//            if (rds.name[0])
//                strcpy(radio.station[i].name, rds.name); // could be the short name only !!
            strcpy(radio.station[i].name, getNameOfId(rds.pi));
            break;
        }
   }
//   qDebug() << "Tuner::set station " << i << " " << radio.station[i].pi << " " << radio.station[i].name;
}

/* ***************************************************************
 * Tuner SI4703 Breakout sparfun
 * ***************************************************************/
void SI4703_Init(void)
{
    RadioReset();

    i2cReadRegisters();
    reg[0x07] = 0x8100; // enable the oscillatore
    i2cWriteRegisters();
    sDelay(500);          // 500ms

    i2cReadRegisters();
    reg[0x02] = 0x0001;   // pwrup
    reg[0x02] |= (1<<15); // enable soft unmute
    reg[0x02] |= (1<<14); // enable unmute
    reg[0x04] |= (1<<12); // enable RDS
    reg[0x04] |= (1<<11); // european config
    reg[0x05]  = 0x0011;  // 100kHz spacing, 87..108 EU, vol=1
//  reg[0x06] |= (1<<8);  // ext. volume +30dB
    i2cWriteRegisters();
    sDelay(120);          // >110ms

    SI4703_Tune( FREQ_DEFAULT );
    SI4703_Volum(15);      // 0..15

    SI4703_Status();
}

void SI4703_Volum(int vol)
{
    int volume = vol;
    i2cReadRegisters();
    if(vol < 0) volume = 0;
    if(vol > 15) volume = 15;
    reg[0x05] &= 0xFFF0;  // clear volume bits
    reg[0x05] |= volume;  // set new volume
    i2cWriteRegisters();
}

void SI4703_Status(void)
{
    int val;
    i2cReadRegisters();
    val = ((reg[0x0B] & 0x1FF)*100) + 87500;
    qDebug() << "Si4703: freq\t= " << val << "KHz";
    val = reg[0x0A] & 0x0100;
    qDebug() << "Si4703: stereo\t= " << ((val==0)?"on":"off");
    val = (reg[0x0A] & 0xFF);
    radio.fstRF[0] = val;
    radio.fstBB[0] = val;
    qDebug() << "Si4703: signal\t= " << val;
    val = (reg[0x05] & 0xF);
    qDebug() << "Si4703: volume\t= " << val;
    val = ((reg[0x0A] & (1<<15))>>15);
    qDebug() << "Si4703: rds\t= " << val;
}

/* freq in KHz */
int SI4703_Tune(int freq)
{
    /* 90400 - 87500 = */
    int i, channel = (freq - 87500) / 100;

    qDebug() << "Si4703: frq" << freq << "chn=" << channel;

    i2cReadRegisters();
    reg[3] &= 0xFE00;  // set channel to 00
    reg[3] |= channel; // set the new channel
    reg[3] |= (1<<15); // set the tune bit
    i2cWriteRegisters();
    i = 0;
    while(1)
    {
        i2cReadRegisters();
        if((reg[0x0A] & (1<<14)) != 0)
            break;
        sDelay(1);          // 1ms
        i++;
    }
    if((reg[0x0A] & (1<<14)) == 0) {
        qDebug() << "Si4703: Err: STC 0";
    }
    i2cReadRegisters();
    reg[3] &= ~(1<<15); // clear the tune bit
    i2cWriteRegisters();

    // wait for the radio to clear the STC bit
    i = 0;
    while(1)
    {
        i2cReadRegisters();
        if((reg[0x0A] & (1<<14)) == 0)
            break;
        sDelay(1);          // 1ms
        i++;
    }
    if((reg[0x0A] & (1<<14)) !=0) {
        qDebug() << "Si4703: Err: STC 1";
    }
    SI4703_Status();
    return 0;
}

int SI4703_Seek(bool up)
{
    int i, ns = 0;
    uint32_t freq;

    qDebug() << "Si4703: seek" << ((up==1)?"up":"down" );

    i2cReadRegisters();
    reg[0x02] |= (1<<10); // allow wrap
    if (up)
         reg[0x02] |= (1<<9);  // set seek direction up
    else reg[0x02] &= ~(1<<9);
    reg[0x02] |= (1<<8);  // start seek
    reg[0x06] &= 0xFF00;  // reset (seek, Imp) threshold
    reg[0x06] |= (3<<4);  // set min seek threshold 1..7
    reg[0x06] |= (4<<1);  // set min FM Imp. threshold 1..15
    i2cWriteRegisters();

    // watch for STC == 1
    i = 0;
    while(1)
    {
        i2cReadRegisters();
        if((reg[10] & (1<<14)) != 0)
            break;          //tuning finished
        sDelay(1);          // 1ms
        i++;
    }
    if((reg[0x0A] & (1<<14)) == 0) {
        qDebug() << "Si4703: Err: STC 0";
    }
    i2cReadRegisters();
    int SFBL = reg[0x0A] & (1<<13);
    reg[0x02] &= ~(1<<8); //clear the seek bit
    i2cWriteRegisters();

    sDelay(80);

    i = 0; // 1sec
    while (1)
    {
        i2cReadRegisters();
        if((reg[0x0A] & (1<<14)) == 0)
            break;
        sDelay(1);          // 1ms
        i++;
    }
    if((reg[0x0A] & (1<<14)) != 0) {
        qDebug() << "Si4703: Err: STC 1";
    }
    if(SFBL) {
        qDebug() << "Si4703: Err: end of band!";
        return -1;
    }
    SI4703_Status();

    radio.band = FM_MODE;
    freq = ((reg[0x0B] & 0x1FF)*100) + 87500;

    /* station found */
    if (radio.freq != freq) {
        radio_station station;
        memset(&station, 0, sizeof(station));
        station.band  = radio.band;
        station.freq  = freq;
        station.fstRF = (reg[0x0A] & 0xFF);
        station.fstBB = station.fstRF;
        station.mp_adj = 0;
        station.activ = true;

        radio.freq = freq; // save actual freq.

        ns = addStation(&station);

        qDebug() << " Next: Station " << (radio.nb_stations-1) << station.freq  << " RF " << station.fstRF;
    }
    /* return new stations */
    return ns;
}

/*
 * timeout in usec
 */
void SI4703_ReadRDS(char* buffer, long timeout)
{
    long endTime = timeout/50;
    bool completed[] = {false, false, false, false};
    int completedCount=0;

    while ((completedCount < 4) && endTime--) {
        i2cReadRegisters();
        if(reg[0x0A] & (1<<15)) {
            uint16_t b = reg[0x0D];
            int index = b & 0x03;

            if (!completed[index] && b < 500) {
                completed[index] = true;
                completedCount++;
                char Dh = (reg[0x0F] & 0xFF00)>>8;
                char Dl = (reg[0x0F] & 0x00FF);
                buffer[index *2] = Dh;
                buffer[index *2+1]=Dl;
            }
            usleep(50); //originally was set to 40
        } else {
            usleep(30);
        }
    }
    if (endTime == 0) {
        buffer[0]='\0';
        return;
    }
    buffer[8]='\0';
}

/* ***************************************************************
 * Tuner TDA7707
 * ***************************************************************/

const unsigned char TabTuCmdSeq[] = {
//len, cmd args....                            CSR             NBR
  3,TUNER_SET_RDS_BUFF,   0x00,0x00,0x01, 0x00,0x02,0x61, 0x00,0x01,0x04,    // RDS fore ground, polling, CSR=0x0265, 4 blocks notify
  3,TUNER_SET_RDS_BUFF,   0x00,0x00,0x02, 0x00,0x02,0x61, 0x00,0x01,0x04,    // back ground
  2,TUNER_SET_AUD_IF,     0x00,0x00,0x13, 0x00,0x04,0x50,    // DAC & SAI on, WS pulse, clk inv., 16L/16R
  2,TUNER_CONF_BB_SAT,    0x00,0x00,0x00, 0x00,0x00,0x01,    // Tuner_Conf_BB_SAI -> Mode 0 Master
  2,TUNER_SET_BB_IF,      0x00,0x00,0x01, 0x00,0x00,0x00,    // Tuner_Set_BB_IF  -> SAI BB CMOS Level, disable AM filter
  1,TUNER_BB_IF_ONOFF,    0x00,0x00,0x01,                    // DEVICE_BB_IF_ON
  1,TUNER_FM_STEREO_MOD,  0x00,0x00,0x00,                    // Stereo auto
   0,0,0,0 // Ende
};

const unsigned char TabTuPhysSeq[] = {
//len, data....
  8, 0x80,0x01,0x40,0x17, 0x18,0x00,0x18,0x18,  // aus Conf_SAI_PINS_TSAT.txt von ST
  8, 0x80,0x01,0x40,0x18, 0x02,0x00,0x00,0x18,
  8, 0x80,0x01,0x40,0x19, 0x00,0x00,0x1D,0x1D,
  8, 0x80,0x01,0x40,0x08, 0x00,0x70,0x00,0x00,
   0,0,0,0 // Ende
};

int TDA77x7_cmdWr(int fd, uint8_t cmd, uint32_t *param, int inlen, int rxlen)
{
    int err, i, outlen;
    uint32_t crc = 0;
    uint8_t *pTx = txBuffer;

    outlen = ((inlen+2) * 3) + 4;
    memset(txBuffer, 0, outlen);

    /* 24bit WR burst autoinc to addr 0x020180 */
    *pTx++ = (uint8_t)((WR24_BURST_AUTO + CMD_BUF_ADDR) >> 24);
    *pTx++ = (uint8_t)((WR24_BURST_AUTO + CMD_BUF_ADDR) >> 16);
    *pTx++ = (uint8_t)((WR24_BURST_AUTO + CMD_BUF_ADDR) >> 8);
    *pTx++ = (uint8_t)((WR24_BURST_AUTO + CMD_BUF_ADDR) & 0xFF);

//	*pTx++ = (uint8_t)((CMD_ID(cmd) + inlen) >> 24);
    *pTx++ = (uint8_t)((CMD_ID(cmd) + inlen) >> 16);
    *pTx++ = (uint8_t)((CMD_ID(cmd) + inlen) >> 8);
    *pTx++ = (uint8_t)((CMD_ID(cmd) + inlen) & 0xFF);

    crc = CMD_ID(cmd) + inlen;
    for (i=0; i<inlen; i++)
    {
//		*pTx++ = (uint8_t)((param[i]) >> 24);
        *pTx++ = (uint8_t)((param[i]) >> 16);
        *pTx++ = (uint8_t)((param[i]) >> 8);
        *pTx++ = (uint8_t)((param[i]) & 0xFF);
                crc += param[i];
    }
    crc = crc &0xFFFFFF;
//	*pTx++ = (uint8_t)(crc >> 24);
    *pTx++ = (uint8_t)(crc >> 16);
    *pTx++ = (uint8_t)(crc >> 8);
    *pTx++ = (uint8_t)(crc & 0xFF);

    err = spiDataRW (fd, txBuffer, outlen, NULL, 0);
    sDelay(1);

        /* read back */
    pTx = txBuffer;
    *pTx++ = (uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 24);
    *pTx++ = (uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 16);
    *pTx++ = (uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 8);
    *pTx++ = (uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) & 0xFF);

    err = spiDataRW (fd, txBuffer, 4, (uint8_t*)rxBuffer, rxlen*3);

    crc = (rxBuffer[4] >> 5) & 7;
    if (crc) {
//       printf("ERR: read - %s %s %s\n", (crc & ERR_CRC)?"CRC":"", (crc & ERR_CID)?"CID":"", (crc & ERR_COLL)?"COLL":"");
    }
    if (rxBuffer[5] != cmd) {
//       printf("ERR: cmd 0x%02X\n", rxBuffer[5]);
    }
    return (err);
}

int TDA77x7_cmdRd(int fd, uint8_t cmd, uint8_t *indata, int inlen)
{
    int err, outlen;

    memset(txBuffer, 0, 8 + inlen);

    if (cmd < TUNER_ANSWER_ONLY) {
        /* 32bit RD burst autoinc to addr 0x020180 */
        txBuffer[0] = (uint8_t)((RD32_BURST_AUTO + CMD_BUF_ADDR) >> 24);
        txBuffer[1] = (uint8_t)((RD32_BURST_AUTO + CMD_BUF_ADDR) >> 16);
        txBuffer[2] = (uint8_t)((RD32_BURST_AUTO + CMD_BUF_ADDR) >> 8);
        txBuffer[3] = (uint8_t)((RD32_BURST_AUTO + CMD_BUF_ADDR) & 0xFF);

        txBuffer[4] = (uint8_t)((CMD_ID(cmd) + inlen) >> 24);
        txBuffer[5] = (uint8_t)((CMD_ID(cmd) + inlen) >> 16);
        txBuffer[6] = (uint8_t)((CMD_ID(cmd) + inlen) >> 8);
        txBuffer[7] = (uint8_t)((CMD_ID(cmd) + inlen) & 0xFF);
        outlen = 8;
    }
    else
    {
        /* 24bit RD burst autoinc to addr 0x020180 78020180 */
        txBuffer[0] = 0x78; //(uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 24);
        txBuffer[1] = 0x02; //(uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 16);
        txBuffer[2] = 0x01; //(uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) >> 8);
        txBuffer[3] = 0x80; //(uint8_t)((RD24_BURST_AUTO + CMD_BUF_ADDR) & 0xFF);
        outlen = 4;
    }
    err = spiDataRW (fd, txBuffer, outlen, indata, inlen);
    memcpy(indata, &indata[outlen], inlen);
    if (err < 0) {
 //       printf("ERR: ioctl..!\n");
    }
    return (err);
}

int TDA77x7_versionRd(int fd, uint8_t *indata, int len)
{
    int err;
    /* 32bit RD burst autoinc to addr 0x1401E */
    txBuffer[0] = (uint8_t)((RD32_BURST_AUTO + CHIP_VERSION_ADDR) >> 24);
    txBuffer[1] = (uint8_t)((RD32_BURST_AUTO + CHIP_VERSION_ADDR) >> 16);
    txBuffer[2] = (uint8_t)((RD32_BURST_AUTO + CHIP_VERSION_ADDR) >> 8);
    txBuffer[3] = (uint8_t)((RD32_BURST_AUTO + CHIP_VERSION_ADDR) & 0xFF);
    err = spiDataRW (fd, txBuffer, 4, indata, len);
    memcpy(indata, &indata[4], len);

    return (err);
}

int TDA77x7_boot(int fd, char *fwImage)
{
        int    err, len, i, remlen, cnt;
        uint32_t cmdAddr;
        FILE  *f;
        size_t nread;

        if (! fwImage) {
                qDebug() << "ERR: no boot file found!\n";
                return -1;
        }
        if ((f = fopen(fwImage, "rb")) == NULL) {
                qDebug() << "ERR: open file " << fwImage;
                return -1;
        }
        qDebug() << "Boot: file " << fwImage;

        /* file: len, data[len], len, .. */
        nread = fread(&len, 4, 1, f);
        if (! nread) {
                qDebug() << "ERR: empty file " << fwImage;
                return -1;
        }
        while(len)
        {
                nread = fread(txBuffer, 1, len, f);
                if ((int)nread < len) len = 0;
                /*
                 * len, addr, data,...
                 * txbuffer: addr[4], data..
                 */
                cmdAddr = (txBuffer[0] << 24) + (txBuffer[1] << 16) + (txBuffer[2] << 8) + (txBuffer[3] & 0xFF);
                cnt = 4;
                remlen = len-4;
                while (remlen) {

                        if (remlen < 4092)
                                len = remlen;
                        else    len = 4092;

                        txBuffer[0] = (cmdAddr >> 24) & 0xFF;
                        txBuffer[1] = (cmdAddr >> 16) & 0xFF;
                        txBuffer[2] = (cmdAddr >>  8) & 0xFF;
                        txBuffer[3] = (cmdAddr >>  0) & 0xFF;
                        for (i=0; i<len; i++)
                                txBuffer[i+4] = txBuffer[cnt+i];

                        err = spiDataRW (fd, txBuffer, len+4, NULL, 0);
                        if (err) {
                                qDebug() << "ERR: spiRW!";
                                return -1;
                        }

                        sDelay(20);
                        remlen  -= len;
                        cnt     += len;
                        cmdAddr += len >> 2; // div 4, 32 bit
                }

                nread = fread(&len, 4, 1, f);
                if(nread < 1) len = 0;
        }
        fclose(f);

        qDebug() << " fw loaded..";

        /* wait on ready */
        len = 200;
        do {
                err = TDA77x7_cmdRd(fd, TUNER_ANSWER_ONLY, rxBuffer, 3);
                if (err) {
                        len = 0;
                        break;
                }
                cmdAddr = (rxBuffer[0] << 16) + (rxBuffer[1] << 8) + rxBuffer[2];
                sDelay(100);
                len--;

        } while (len && (cmdAddr != 0xAFFE42));

        if (! len) {
                qDebug() << "ERR: timeout to ready!";
                return -1;
        }
        return (err);
}

int TDA77XX_Init(void)
{
    int i, len, rep;

    radio.fd = spiInit(radio.device, SPI_MODE, SPI_SPEED, SPI_BITS);
    if (radio.fd >= 0) {
        /* ready? */
        if (TDA77XX_GetQuality() < 0)
        {
            /* load firmware */
            rep = 3;
            while (rep)
            {
                RadioReset();

                /* read access */
                TDA77x7_versionRd(radio.fd, rxBuffer, 8);

                qDebug() << " Version: ";
                for (i=2; i<8; i++) {
                        //qDebug() << rxBuffer[i];
                }
                qDebug() << "\n";

                if ((rxBuffer[2] != 'B') || (rxBuffer[3] != 'A') || (rxBuffer[4] != 'V') ||
                    (rxBuffer[5] != '7') || (rxBuffer[6] != '6') || (rxBuffer[7] != '6'))
                {
                    qDebug() << "ERR: Version of TDA7707 wrong!\n";
                    rep--;
                    continue;
                }
                sDelay(50); // 5ms

                if (TDA77x7_boot(radio.fd, radio.TDA7707Image) == 0)
                {
                    qDebug() << " boot o.k.";
                    break;
                }
                else if (rep <= 0)
                {
                    qDebug() << "ERR: TDA7706 loading!";
                    close(radio.fd);
                    return -1;
                }
                else rep--;
            }
        }
        if (! rep) {
            return -1;
        }

        /* cmd table out */
        i = 0;
        len = TabTuCmdSeq[i++];
        while(len) {
                int p;
                unsigned char cmd = TabTuCmdSeq[i++];
                for(p=0; p<len; p++) {  // words
                        dwParam[p]  = (TabTuCmdSeq[i++] << 16);
                        dwParam[p] += (TabTuCmdSeq[i++] <<  8);
                        dwParam[p] += (TabTuCmdSeq[i++] & 0xFF);
                }
                TDA77x7_cmdWr(radio.fd, cmd, dwParam, len, 1);
                sDelay(100); // 10ms
                len = TabTuCmdSeq[i++];
        }
        /* default fm */
        radio.state = 'i';

//      SetThreshold(1, 30, 255, 255); // BB, det_th, mp_adj

        TDA77XX_Mute(0);

        return 0;
    }
    return -1;
}

int TDA77XX_Tune(int dev, char *band, int freq)
{
    /* select band */
    if (strcmp(band, "DAB3") == 0)
        radio.band = DAB_MODE3;
    else if (strcmp(band, "DABL") == 0)
        radio.band = DAB_MODEL;
    else if (strcmp(band, "AM") == 0)
        radio.band = AM_MODE_EU;
    else
        radio.band = FM_MODE;

    /* band init */
    switch (radio.band)
    {
    case AM_MODE_EU: radio.band_low = AM_BAND_LOW;
             radio.band_high = AM_BAND_HIGH;
             break;
    case DAB_MODE3: radio.band_low = DAB_BAND3_LOW;
             radio.band_high = DAB_BAND3_HIGH;
             break;
    case DAB_MODEL: radio.band_low = DAB_BANDL_LOW;
             radio.band_high = DAB_BANDL_HIGH;
             break;
    case FM_MODE:
    default:
             radio.band = FM_MODE;
             radio.band_low = FM_BAND_LOW;
             radio.band_high = FM_BAND_HIGH;
             break;
    }
    /* actual Freq init */
    radio.freq = freq;

    qDebug() << " Tune: (" << dev << ") Band " << ((radio.band == FM_MODE)?"FM": ((radio.band == AM_MODE_EU)?"AM":"DAB")) << "Frq " << radio.freq;

    if (radio.band) {
            dwParam[0] = 0x03;      // band change fore & back ground
            dwParam[1] = radio.band;      // band
            if (radio.band == FM_MODE)
                    dwParam[2] = 0x02;          // Antenna diversity on
            else if (radio.band == DAB_MODE3)
                    dwParam[2] = 0x30;          // HD filter, Digital BB
            else {                              // AM
                    dwParam[2] = 0x00;          // FM, HD off
            }
            dwParam[3] = radio.band_low;  // low limit
            dwParam[4] = radio.band_high; // high limit
            TDA77x7_cmdWr(radio.fd, TUNER_BAND, dwParam, 5, 1);

            sDelay(500); // 50ms
    }
    if (radio.freq) {
            dwParam[0] = 0x03;      // tune change fore & back ground
            dwParam[1] = radio.freq;      // khz
            TDA77x7_cmdWr(radio.fd, TUNER_TUNE, dwParam, 2, 2);
    }
    sDelay(500); // 10ms

    /* VPA fore & back ground, seamless switch */
    if (radio.band == FM_MODE) {

            dwParam[0] = 0x01;
            TDA77x7_cmdWr(radio.fd, TUNER_SET_FM, dwParam, 1, 1);

            sDelay(100); // 10ms

            /* SAI_BB_CONF_NEW = ((SAI_BB_CONF & 0xFFFFFF3F) | 0x00000080)
             *  - bit[7:6] have to be set to 0x10.
             */
            txBuffer[0] = 0x18;
            txBuffer[1] = 0;
            txBuffer[2] = 0xb0;
            txBuffer[3] = 0;
            spiDataRW (radio.fd, txBuffer, 4, (uint8_t*)rxBuffer, 4);

            txBuffer[0] = 0x98;  //write
            txBuffer[1] = 0;
            txBuffer[2] = 0xb0;
            txBuffer[3] = 0;
            txBuffer[4] = rxBuffer[4];
            txBuffer[5] = rxBuffer[5];
            txBuffer[6] = rxBuffer[6];
            txBuffer[7] = (rxBuffer[7] & 0x3f) | 0x80;
            spiDataRW (radio.fd, txBuffer, 4, (uint8_t*)rxBuffer, 4);
    }
    else if (radio.band == AM_MODE_EU) {
            dwParam[0] = 0x00;
            dwParam[1] = 0x01;
            TDA77x7_cmdWr(radio.fd, TUNER_SET_BB_IF, dwParam, 2, 1);

            sDelay(100); // 10ms
    }
    dwParam[0] = 0x01; // unmute
    TDA77x7_cmdWr(radio.fd, TUNER_AUDIO_MUTE, dwParam, 1, 1);

    return 0;
}

int TDA77XX_Seek(int dev, bool up)
{
    int i;
    int ns = 0;
    dev = dev;

    dwParam[0] = 0x01; // seek fore or !! back=2 ground
    if (up)
           dwParam[1] = (3 << 4) | 0x01;  // seek auto up, unmute, 3 additional measurments
    else   dwParam[1] = (3 << 4) | 0x03;  // seek auto down , unmute, 3 additional measurments
    TDA77x7_cmdWr(radio.fd, TUNER_SEEK_START, dwParam, 2, 1);
    sDelay(500); // 50ms
    i = 0;
    while(1) {
            dwParam[0] = 0x01;   // return fore or back=2 ground
            TDA77x7_cmdWr(radio.fd, TUNER_GET_SEEK_STAT, dwParam, 1, 3);
            /* station found */
            if (rxBuffer[7] & 0x80) {
                radio_station station;
                memset(&station, 0, sizeof(station));
                station.band = radio.band;
                station.freq = (rxBuffer[7] << 16) + (rxBuffer[8] << 8) + (rxBuffer[9] & 0xFF);
                station.freq &= 0x1FFFFF;
                station.fstRF = rxBuffer[10];
                station.fstBB = rxBuffer[11];
                station.mp_adj = rxBuffer[13];
                station.activ = true;

                radio.freq = station.freq; // save actual freq.

                ns = addStation(&station);

                qDebug() << " Next: Station " << (radio.nb_stations-1) << station.freq  << " RF " << station.fstRF;
                break;
            }
            sDelay(500); // 50ms
            if (i++ > 100) // timeout 5sec
                    break;
    }
    /* end of seek, RDS reset */
    dwParam[0] = 0x01;  // seek fore (back=2) ground
    dwParam[1] = 0x00;  // seek stop, unmute
    TDA77x7_cmdWr(radio.fd, TUNER_SEEK_END, dwParam, 2, 1);

    return ns;
}

int TDA77XX_Scan(int dev, char *band)
{
    int ns, i = 0;
    dev = dev;

    if (strcmp(band, "DAB3") == 0)
        radio.band = DAB_MODE3;
    else if (strcmp(band, "DABL") == 0)
        radio.band = DAB_MODEL;
    else if (strcmp(band, "AM") == 0)
        radio.band = AM_MODE_EU;
    else
        radio.band = FM_MODE;

    switch (radio.band)
    {
    case AM_MODE_EU: radio.band_low = AM_BAND_LOW;
             radio.band_high = AM_BAND_HIGH;
             break;
    case DAB_MODE3: radio.band_low = DAB_BAND3_LOW;
             radio.band_high = DAB_BAND3_HIGH;
             break;
    case DAB_MODEL: radio.band_low = DAB_BANDL_LOW;
             radio.band_high = DAB_BANDL_HIGH;
             break;
    case FM_MODE:
    default:
             radio.band = FM_MODE;
             radio.band_low = FM_BAND_LOW;
             radio.band_high = FM_BAND_HIGH;
             break;
    }
    qDebug() << " Tune: Seek/Scan " << band;

    /* set band */
    if (radio.band) {
            dwParam[0] = 0x03;            // band change fore & back ground
            dwParam[1] = radio.band;      // band
            if (radio.band == FM_MODE)
                    dwParam[2] = 0x02;          // Antenna diversity on
            else if (radio.band == DAB_MODE3)
                    dwParam[2] = 0x30;          // HD filter, Digital BB
            else {                              // AM
                    dwParam[2] = 0x00;          // FM, HD off
            }
            dwParam[3] = radio.band_low;  // low limit
            dwParam[4] = radio.band_high; // high limit
            TDA77x7_cmdWr(radio.fd, TUNER_BAND, dwParam, 5, 1);

            sDelay(500); // 50ms
    }
    dwParam[0] = 0x01;  // seek fore (back=2) ground
    dwParam[1] = 0x00;  // seek stop, unmute
    TDA77x7_cmdWr(radio.fd, TUNER_SEEK_END, dwParam, 2, 1);
    sDelay(100); // 50ms

    dwParam[0] = 0x01;          // seek fore (back=2) ground
    dwParam[1] = (1 << 8) | (3 << 4) | 0x05;  // seek auto, stay mute, 3 additional measurments
    TDA77x7_cmdWr(radio.fd, TUNER_SEEK_START, dwParam, 2, 1);
    sDelay(500); // 50ms

    i = 0;
    while(1)
    {
        dwParam[0] = 0x01;          // return fore (back=2) ground
        TDA77x7_cmdWr(radio.fd, TUNER_GET_SEEK_STAT, dwParam, 1, 3);
//      dump("seek", &rxBuffer[4], 4);

        /* 23:20,19:16,15:8,7:0
         *     1F.FFFF
         */
        /* station found */
        if (rxBuffer[7] & 0x80) {
            radio_station station;
            memset(&station, 0, sizeof(station));
            station.band = radio.band;
            station.freq = (rxBuffer[7] << 16) + (rxBuffer[8] << 8) + (rxBuffer[9] & 0xFF);
            station.freq &= 0x1FFFFF;
            station.fstRF = rxBuffer[10];
            station.fstBB = rxBuffer[11];
            station.mp_adj = rxBuffer[13];
            station.activ = true;

            ns = addStation(&station);

            qDebug() << " Scan::Station " << i << ns << " frq " << station.freq << " RF " << (int8_t)station.fstRF;

            i += ns;

            /* seek next */
            dwParam[0] = 0x01;          // seek fore (back=2) ground
            TDA77x7_cmdWr(radio.fd, TUNER_SEEK_START, dwParam, 1, 1);
        }
        /* full seek reached */
        if ((i>MAX_STATIONS) || (rxBuffer[7] & 0x40)) {

                qDebug() << " Tune: Seek/Scan end with " << i << " stations";
                radio.nb_stations = i;
                break;
        }
        sDelay(500); // 50ms
    }
    /* end of seek, RDS reset */
    dwParam[0] = 0x03;  // seek fore (back=2) ground
    dwParam[1] = 0x00;  // seek stop, unmute
    TDA77x7_cmdWr(radio.fd, TUNER_SEEK_END, dwParam, 2, 1);

    return i;
}

int TDA77XX_GetQuality(void)
{
    dwParam[0] = 0x01; // fore ground
    if (TDA77x7_cmdWr(radio.fd, TUNER_GET_REC_QLY, dwParam, 1, 2))
        return -1;

    if ((rxBuffer[7] == 255) && (rxBuffer[8] == 255) && (rxBuffer[9] == 255))
        return -1;

    if ((rxBuffer[7] == 0) && (rxBuffer[9] == 0))
        return -1;

    radio.fstRF[0] = rxBuffer[7];
    radio.fstBB[0] = rxBuffer[8];

    //qDebug() << " QTY foregnd: " << rxBuffer[7] << rxBuffer[8] << rxBuffer[9] << " " << rxBuffer[10] << rxBuffer[11] << rxBuffer[12];

    dwParam[0] = 0x02; // back ground
    TDA77x7_cmdWr(radio.fd, TUNER_GET_REC_QLY, dwParam, 1, 2);

    radio.fstRF[1] = rxBuffer[7];
    radio.fstBB[1] = rxBuffer[8];

    //qDebug() << " QTY backgnd: " << rxBuffer[7] << rxBuffer[8] << rxBuffer[9] << " " << rxBuffer[10] << rxBuffer[11] << rxBuffer[12];
    return 0;
}

void TDA77XX_Mute(int mute)
{
    if (mute)
         radio.mute = 0;
    else radio.mute = 1;
    dwParam[0] = radio.mute; // 0 mute
    TDA77x7_cmdWr(radio.fd, TUNER_AUDIO_MUTE, dwParam, 1, 1);
}

/* ***************************************************************
 * RDS
 * ***************************************************************/

int rdsCheckText(char *txt)
{
        if (*txt == 0x0D) { // end of message
//                printf("eo msg\n");
                *txt = ' ';
                return 0;
        } else if (*txt == 0x0A) { // line break
//                printf("line break\n");
                *txt = ' ';
                return 0;
        } else if (*txt == 0x0B) { // end of headline
//                printf("eo head\n");
                *txt = ' ';
                return 0;
        } else if (*txt == 0x1F) { // soft hyph
//                printf("hyphen\n");
                *txt = ' ';
                return 0;
        }
        return 1;
}

int rdsDecode(uint16_t *rdata)
{
#if 0
    int i;
#endif
    int AF[2], PTY, print = 0;
    uint8_t GTYPE, TMC, TA, TP, MS, DI, C10;
    char PS[4];

    GTYPE = (rdata[1] >> 11) & 0x1F;
    TP   = (rdata[1] >> 10) & 0x1;
    PTY  = (rdata[1] >> 5) & 0x1F;
    TA   = (rdata[1] >> 4) & 0x1;
    MS   = (rdata[1] >> 3) & 0x1;
    DI   = (rdata[1] >> 2) & 0x1;
    C10  = rdata[1] & 3;
    TMC = 0;

    if (TP || MS || DI)
    {

    }

    switch (GTYPE)
    {
    case 0: // 0A
            TMC = TA;
            PS[0] = (rdata[3] >> 8) & 0xFF;
            PS[1] = rdata[3] & 0xFF;
            rds.name[C10<<1] = (char)PS[0];
            rds.name[(C10<<1)+1] = (char)PS[1];
            AF[0] = rdata[2] & 0xFF;        // 1. freq
            AF[1] = (rdata[2] >> 8) & 0xFF; // 2. freq
            if ((int)AF[0] < 204)
                    rds.freq = 87500 + (int32_t)(AF[0]*100);
            print = 1;
            break;
    case 1: // 0B
            TMC = TA;
            PS[0] = (rdata[3] >> 8) & 0xFF;
            PS[1] = rdata[3] & 0xFF;
            rds.name[C10<<1] = (char)PS[0];
            rds.name[(C10<<1)+1] = (char)PS[1];
            rds.name[8] = 0;
            print = 1;
            break;
    case 4: // 2A
            if (rds.ta != TA) { // clear buffer
                    strcpy(rds.save, rds.text);
                    memset(rds.text, 0, sizeof(rds.text));
            }
            rds.ta = TA;
            C10  = rdata[1] & 0xF;
            PS[0] = (rdata[2] >> 8) & 0xFF;
            PS[1] = rdata[2] & 0xFF;
            PS[2] = (rdata[3] >> 8) & 0xFF;
            PS[3] = rdata[3] & 0xFF;
            rdsCheckText(&PS[0]);
            rds.text[C10<<2] = (char)PS[0];
            rdsCheckText(&PS[1]);
            rds.text[(C10<<2)+1] = (char)PS[1];
            rdsCheckText(&PS[2]);
            rds.text[(C10<<2)+2] = (char)PS[2];
            rdsCheckText(&PS[3]);
            rds.text[(C10<<2)+3] = (char)PS[3];
            print = 1;
            break;
    case 5: // 2B
            if (rds.ta != TA) { // clear buffer
                    strcpy(rds.save, rds.text);
                    memset(rds.text, 0, sizeof(rds.text));
            }
            rds.ta = TA;
            C10  = rdata[1] & 0xF;
            PS[0] = (rdata[3] >> 8) & 0xFF;
            PS[1] = rdata[3] & 0xFF;
            rdsCheckText(&PS[0]);
            rds.text[C10<<1] = (char)PS[0];
            rdsCheckText(&PS[1]);
            rds.text[(C10<<1)+1] = (char)PS[1];
            print = 1;
            break;
    case 8: // 4A time, date
            rds.utc = rdata[3] & 0x2F; // B5=+/-

            qDebug() << "UTC: " << ((rds.utc & 0x20)?"-":"+") << (rds.utc & 0x1F);

            break;
    case 9: // 4B
            break;
    }

    if (print) {

//        qDebug() << rds.pi   << " ->  " << getNameOfId(rds.pi);
        return 1;

        if (getNameOfId(rds.pi))
                qDebug() << rds.pi   << " ->  " << getNameOfId(rds.pi)
                                     << "- " << pty[(int)PTY].name << " / " << rds.freq << " / "
                                     << rds.name << " - " << rds.text;
        else    qDebug() << rds.pi   << " ->  " << rdata[0]
                                     << "- " << pty[(int)PTY].name << " / " << rds.freq << " / "
                                     << rds.name << " - " << rds.text;
#if 0
        if (rds.save[0])
                qDebug() << rds.save;
#endif
        if (rds.tmc != TMC) {
                qDebug() << "TMC " << ((TMC == 1)?"on":"off") << "\n";
                rds.tmc = TMC;
        }
        return 1;
    }
    return 0;
}

} ///< namespace

/* ********************************************************************
 * class RdsThread
 ********************************************************************** */
/*
 * RDA thread
 *
 * radio->state: o opened
 *             i initialized
 *             q quit thread and TunerControl
 *             t tuned
 *             n new,next
 *             r rds receiption
 */
void RdsThread::run()
{
    int  i, id, len;

    qDebug() << "TunerControl: rds thread start - state:" << (char)radio.state;
    while(1)
    {
        switch (radio.state)
        {
        case 'o':
            mutex.lock();
            emit rdsAvailable();
            mutex.unlock();
            sleep(5);
            break;

        case 'q':
            qDebug() << "TunerControl: rds thread stop - state:q";
            break;

        case 's':
            radio.state = 't'; // goto tuned
            break;

        case 't':
            qDebug() << "TunerControl: rds thread - state:t";
            memset(&rds, 0, sizeof(rds_buffer));
            radio.state = 'r'; // goto rds receive
            break;

        case 'r':
            {
//              qDebug() << "TunerControl: rds thread - state:r";
                /* get RDS - RNR notify */
                dwParam[0] = 0x00;   // reset fore (back=2) ground
                /* read out without param */
                TDA77x7_cmdWr(radio.fd, TUNER_READ_RDS_BUFF, dwParam, 0, 0x1F); // max.20 blocks+status
                len = rxBuffer[6] & 0x1F;
                if (len)
                {
                    mutex.lock();
                    /* sync, no over */
                    rds.notify = (rxBuffer[7] << 16) + (rxBuffer[8] << 8) + (rxBuffer[9] & 0xFF);

                    if (!(rds.notify & RDS_BOFL)
                     && ((rds.notify & (RDS_DATARDY | RDS_SYNC)) == (RDS_DATARDY | RDS_SYNC)))
                    {
                        int grp, block;
                        /* rds spy format */
                        id = 10;
                        grp = 0;
                        for (i=0; i<len-2; i++)
                        {
                            if ((rxBuffer[id] & 0x70) < 0x20)
                            {
                                block = rxBuffer[id++] & 3;
                                /* pi */
                                if (block == 0)
                                {
                                        rds.pi = (rxBuffer[11] << 8) + rxBuffer[12];
                                }
                                grp |= 1 << block;
                                rds.data[block]  = (rxBuffer[id++] << 8);
                                rds.data[block] += rxBuffer[id++];
                                if (grp == 0xF)
                                {
                                    grp = 0;
                                    if (rdsDecode(rds.data))
                                    {
                                        emit rdsAvailable();
                                        if (rds.tmc)
                                            emit tmsAvailable();
                                    }

                                }
                            }
                        }
                    }
                    mutex.unlock();
                }
                usleep(300000); // 300ms
            }
            break;

        /* i,n,.. */
        default:
//          qDebug() << "TunerControl: rds thread - state:" << (char)radio.state;
            sleep(1);
            break;
        }

        if(radio.state == 'q')
            break;
    }
    qDebug() << "TunerControl: close rds thread - state= " << radio.state;
}

/* ********************************************************************
 * class TunerControl
 ********************************************************************** */
TunerControl::TunerControl()
    : m_fileutil()
{
    memset(&radio, 0, sizeof(radio_param));
    radio.state = 'o';

    /* clear scan tables of stations */
    memset(radio.station, 0, sizeof(radio_station) * MAX_STATIONS);
    radio.nb_stations = 0;
    radio.type = TYPE_UNKNOWN;

    /* i2c si4703 */
    if ((i2cReadRegisters() == 0) && (radio.chipid == SI4703_CHIPID)) {
        strcpy(radio.device, SI4703_DEVICE);
        SI4703_Init();
        radio.type = TYPE_SI4703;
        qDebug() << "TunerControl: SI4703 connected..";
        radio.state = 't';
    }
    else /* spi taf7707 */
    {
        strcpy(radio.device, TDA77XX_DEVICE);
        /* default */
        QString str = m_fileutil.getSystemBase() + "/" + TDA77XX_FIRMWARE_PATH;
        qDebug() << "TunerControl:" << (char*)str.toStdString().c_str();
        radio.TDA7707Image = (char*)str.toStdString().c_str();

        if (TDA77XX_Init() == 0) {
            radio.type = TYPE_TDA77XX;
            qDebug() << "TunerControl: TDA7707 connected..";
            radio.state = 't';
        }
        else
        {
            qDebug() << "TunerControl: no HW -> emulation..";
            return;
        }
    }
    if (radio.type == TYPE_TDA77XX) {
        /* instantiate thread object */
//        rdsThread.start();

//        connect(&rdsThread, SIGNAL(rdsAvailable()), this, SLOT(rdsReceived()) );
//        connect(&rdsThread, SIGNAL(tmsAvailable()), this, SLOT(tmsReceived()) );
    }
}

TunerControl::~TunerControl()
{
    qDebug() << "TunerControl::close";
    if (radio.state != 'o')
    {
        radio.state = 'q';
//        rdsThread.wait();
//        rdsThread.terminate();
    }
}

int TunerControl::getquality(int dev)
{
    int ret = 0;

    if (radio.state == 'o')
        return -3;

    if ((dev < 1) || (dev > 3))
        return -2;

    if (radio.type == TYPE_TDA77XX) {
        if (TDA77XX_GetQuality() < 0)
            return -1;
        ret = radio.fstRF[dev-1];
    }
    else if (radio.type == TYPE_SI4703) {
        SI4703_Status();
        ret = radio.fstRF[dev-1];
    }
    return ret;
}

int TunerControl::setThreshold(int dev, int thres_bb, int thres_det, int thres_adj)
{
    if (radio.state == 'o')
        return -3;

    if ((dev < 1) || (dev > 3))
        return -2;

    radio.thres_fstBB = thres_bb;
    radio.thres_det = thres_det;
    radio.thres_mp_adj = thres_adj;

    /* set threshold */
    if (radio.type == TYPE_TDA77XX) {
        dwParam[0] = 0x03;  // seek thres fore & (back=2) ground
        dwParam[1] = (radio.thres_fstBB << 8) + (radio.thres_det & 0xFF);
    //  dwParam[2] = (radio.thres_mp_adj << 16);
        TDA77x7_cmdWr(radio.fd, TUNER_SET_SEEK_THRES, dwParam, 2, 1);
    }
    else if (radio.type == TYPE_SI4703) {

    }

    qDebug() << "TunerControl: set thres " << radio.thres_fstBB;

    return 0;
}

int TunerControl::tune(int dev, const QString &b, const QString &f)
{
    char *band = (char*)b.toStdString().c_str();
    int freq = strtol((char*)f.toStdString().c_str(), NULL, 10);

    if (radio.type == TYPE_SI4703) {
        SI4703_Tune(freq);
        return 0;
    }

    if (radio.state == 'o') // simulation
        return -3;

    if ((dev < 1) || (dev > 3))
        return -2;

    if (radio.type == TYPE_TDA77XX) {
        radio.state = 'n';
        TDA77XX_Tune(dev, band, freq);
        radio.state = 't';
    }
    return 0;
}

int TunerControl::scan(int dev, const QString &b)
{
    int i = 0;
    char *band = (char*)b.toStdString().c_str();

    if (radio.type == TYPE_SI4703) {
        rdsThread.mutex.lock();
        rdsThread.mutex.unlock();
        return 0;
    }

    if (radio.state == 'o')
        return 0;

    if ((dev < 1) || (dev > 3))
        return 0;

    if (radio.type == TYPE_TDA77XX) {
        rdsThread.mutex.lock();
        radio.state = 'n';
        i = TDA77XX_Scan(dev, band);
        radio.state = 's';
        rdsThread.mutex.unlock();
    }
    return i;
}

int TunerControl::nextup()
{
    int ns = next(1, true);
    return ns;
}

int TunerControl::nextdown()
{
    int ns = next(1, false);
    return ns;
}

int TunerControl::next(int dev, bool up)
{
    int i = 0;

    qDebug() << " Tune: next " << (up?" up":" down") << "on" << dev;

    if (radio.type == TYPE_SI4703) {
        rdsThread.mutex.lock();
        i = SI4703_Seek(up);
        rdsThread.mutex.unlock();
        return i;
    }

    if (radio.state == 'o')
        return -3;

    if ((dev < 1) || (dev > 3))
        return -2;

    if (radio.type == TYPE_TDA77XX) {
        rdsThread.mutex.lock();
        radio.state = 'n';
        i = TDA77XX_Seek(dev, up);
        radio.state = 't';
        rdsThread.mutex.unlock();
    }
    /* count of new stations */
    return i;
}

void TunerControl::mute()
{
    qDebug() << "TunerControl mute";

    if (radio.state == 'o')
        return;

    if (radio.type == TYPE_TDA77XX) {
        TDA77XX_Mute(1);
    }
}

void TunerControl::unmute()
{
    qDebug() << "TunerControl unmute";

    if (radio.state == 'o')
        return;

    if (radio.type == TYPE_TDA77XX) {
        TDA77XX_Mute(0);
    }
}

void TunerControl::volume(double volume)
{
//  qDebug() << " Volume: " << volume;

    if (radio.state == 'o')
        return;
    volume = volume;
}

/* -------------------------------------------
 * service functions
 --------------------------------------------- */
void TunerControl::clearstations(void)
{
    if (radio.state == 'o') // simulation
        return;

    rdsThread.mutex.lock();
    clearStations();
    rdsThread.mutex.unlock();
}

/*
 * get rds text, present(0) and last(1)
 */
QString TunerControl::getrdstext(int line)
{
    QString text;
    if (radio.state == 'o')
         text = "rds daten kommen hier..";
    else if (line == 0)
         text = QString::fromLocal8Bit(rds.text);
    else text = QString::fromLocal8Bit(rds.save);
    return text;
}

/*
 * get actual station logo of rds pi, from pi_data table
 */
QString  TunerControl::getnamelogo(QString station)
{
   QString name;
   int i = 0;
   while(pi_data[i].pi) {
       if (station == pi_data[i].name) {
           name = m_fileutil.getLogoBase() + "/" + QString::fromLocal8Bit(pi_data[i].icon);
           return name;
       }
       i++;
   }
   if (radio.state == 'o') {
       return (m_fileutil.getLogoBase() + "/logo-radio.png");
   }
   return NULL;
}

/*
 * get actual station logo of rds pi, from pi_data table
 */
QString  TunerControl::getstationlogo()
{
   QString name;
   int i = 0;
   if (rds.pi == 0)
       return NULL;

// qDebug() << "Tuner::logo " << rds.pi;
   while(pi_data[i].pi) {
           if (pi_data[i].pi == rds.pi) {
              name = m_fileutil.getLogoBase() + "/" + QString::fromLocal8Bit(pi_data[i].icon);
//              qDebug() << "Tuner::logo " << name;
              return name;
           }
           i++;
   }
   return NULL;
}

void TunerControl::rdsReceived()
{
//    qDebug() << "TunerControl: rds available\n";
    rdsThread.mutex.lock();
    /* set pi, name, logo */
    setActualStation();
    rdsThread.mutex.unlock();
}

void TunerControl::tmsReceived()
{
    qDebug() << "TunerControl: tms available\n";
}

int TunerControl::gettmsstatus()
{
    return rds.tmc;
}

int TunerControl::getrdspi()
{
    return(rds.pi);
}

/*
 * get actual rds station name
 */
QString  TunerControl::getrdsname()
{
   QString name;
   if (rds.pi == 0)
       return "";

   name = QString::fromLocal8Bit(getNameOfId(rds.pi));

//   qDebug() << "Tuner::pi name" << rds.pi << " " + name;
//   name = QString::fromLocal8Bit(rds.name);
//   qDebug() << "Tuner::rds name" << rds.pi << " " + name;
   if (name.isEmpty())
       return "";

   return name;
}

int TunerControl::getactualindex(void)
{
   int i, idx = 0;
   for (i=0; i<radio.nb_stations; i++) {
      /* active entrys displayed */
      if (radio.station[i].activ) {
         if (radio.freq == radio.station[i].freq) {
            radio.last_station = idx;

            qDebug() << "Tuner::station now" << radio.last_station << "f" << radio.freq;
            break;
         }
         idx++;
      }
   }
   if (i == radio.nb_stations) {
       qDebug() << "Tuner::station not found f" << radio.freq;
       return -1;
   }
   return radio.last_station; //-1;
}

/* ---------------------------------------------------------
 * Stationlist to QString and file
 *
 * ---------------------------------------------------------
 */
/*
 * Convert radio scan entry to QString
 * FM,88400,PI,MDR FIGARO,36,36
 */
QString TunerControl::getStationString(int entry)
{
    QString line;

    if (entry >= radio.nb_stations)
        return NULL;
    if (radio.station[entry].band == FM_MODE) line = "FM";
    else if (radio.station[entry].band == AM_MODE_EU) line = "AM";
    else if (radio.station[entry].band == DAB_MODE3) line = "DAB3";
    else if (radio.station[entry].band == DAB_MODEL) line = "DABL";
    else line = "FM";
    line.append(",");
    // freq
    line.append(QString::number(radio.station[entry].freq, 10));
    line.append(",");
    // activ
    if (radio.station[entry].activ)
        line.append("T,");
    else line.append("F,");
    // rds.pi
    line.append(QString::number(radio.station[entry].pi, 16));
    line.append(",");
    // name
    if (radio.station[entry].name[0] == 0)
        line.append("NONAME");
    else  line.append(QString::fromLocal8Bit(radio.station[entry].name));
    line.append(",");
    // quality
    line.append(QString::number(radio.station[entry].fstRF, 10));
    line.append(",");
    line.append(QString::number(radio.station[entry].fstBB, 10));

    return line;
}

/*
 * Write station text file
 */
int TunerControl::storestationlist(QString storefile)
{
    int i;
    QString line;
    QString filename = m_fileutil.getRadioBase() + "/" + storefile;
    QFile file( filename );

    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
       QTextStream stream( &file );
       for (i=0; i<radio.nb_stations; i++) {
           line = getStationString(i);
//         qDebug() << line;
           stream << line << (char)0x0A;
       }
       file.close();
       return 0;
    }
    else {
        qDebug() << "Error: open.." << filename;
    }
    return -1;
}

/* ---------------------------------------------------------
 * QString of Stationlist to radio_station  station[MAX_STATIONS]
 *
 * ---------------------------------------------------------
 */
/*
 * FM,107400,T,14C2,R.SA,22,22 ->
 */
int TunerControl::qstring2station(QString line, int entry)
{
   int i;
   bool ok;
   QString str;

   str = m_fileutil.getLineItem(line, 1);
   if (str == "FM")         radio.station[entry].band = FM_MODE;
   else if (str == "AM")    radio.station[entry].band = AM_MODE_EU;
   else if (str == "DAB3")  radio.station[entry].band = DAB_MODE3;
   else if (str == "DABL")  radio.station[entry].band = DAB_MODEL;
   else return -1;

   str = m_fileutil.getLineItem(line, 2);
   i = str.toInt(&ok, 10);
   if (! ok) return -1;
   radio.station[entry].freq = i;

   str = m_fileutil.getLineItem(line, 3);
   if (str == "T")  radio.station[entry].activ = true;
   else radio.station[entry].activ = false;

   str = m_fileutil.getLineItem(line, 4);
   i = str.toInt(&ok, 16);
   if (! ok) return -1;
   radio.station[entry].pi = i;

   str = m_fileutil.getLineItem(line, 5);
   strcpy(radio.station[entry].name, str.toStdString().c_str());

   str = m_fileutil.getLineItem(line, 6);
   i = str.toInt(&ok, 10);
   if (! ok) return -1;
   radio.station[entry].fstRF = i;

   str = m_fileutil.getLineItem(line, 7);
   i = str.toInt(&ok, 10);
   if (! ok) return -1;
   radio.station[entry].fstBB = i;

   radio.nb_stations = entry + 1;
#if 0
   qDebug() << " B  " << radio.station[entry].band
            << " F  " << radio.station[entry].freq
            << " pi " << radio.station[entry].pi
            << " NM " << radio.station[entry].name
            << " RF " << radio.station[entry].fstRF
            << " BB " << radio.station[entry].fstBB;
#endif
   return 0;
}
