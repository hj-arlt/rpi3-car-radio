#ifndef TUNERDEFINES_H
#define TUNERDEFINES_H

/*
 * TDA7707 Eval.boaed
 */

/* tuner command */
#define TUNER_READ              0x1E
#define TUNER_WRITE             0x1F
#define TUNER_PING              0x00
#define TUNER_TUNE              0x08
#define TUNER_BAND              0x0A
#define TUNER_SET_FM            0x11
#define TUNER_GET_REC_QLY       0x12
#define TUNER_GET_CHN_QLY       0x13
#define TUNER_GET_AF_QLY        0x24
#define TUNER_SET_BB_IF         0x04
#define TUNER_CONF_BB_SAT       0x05
#define TUNER_CONF_JESD204      0x06
#define TUNER_SET_AUD_IF        0x0D
#define TUNER_SET_FIR2          0x0E
#define TUNER_SET_NOTCH         0x34
#define TUNER_SET_BLEND         0x14
#define TUNER_CONF_HD_BLD       0x15
#define TUNER_AUDIO_MUTE        0x16
#define TUNER_FM_STEREO_MOD     0x17
#define TUNER_AF_START          0x20
#define TUNER_AF_END            0x21
#define TUNER_AF_CHECK          0x22
#define TUNER_AF_SWITCH         0x23
#define TUNER_SEEK_START        0x26
#define TUNER_SEEK_END          0x27
#define TUNER_GET_SEEK_STAT     0x28
#define TUNER_SET_SEEK_THRES    0x29
#define TUNER_SET_RDS_BUFF      0x1A
#define TUNER_READ_RDS_BUFF     0x1B
#define TUNER_SET_VICS_OUT      0x1C
#define TUNER_SET_BB_PROC       0x30
#define TUNER_SET_DISS          0x31
#define TUNER_GET_WSP_STAT      0x32
#define TUNER_BB_IF_ONOFF       0x33
#define TUNER_TEST_BB           0x35
#define TUNER_ANSWER_ONLY       0x7F

/* control */
#define RD_ACCESS               (0 << 31)
#define WR_ACCESS               (1 << 31)
#define DATA32                  (0 << 29)
#define DATA24                  (3 << 29)
#define BURST_MODE              (1 << 28)
#define AUTO_INC                (1 << 27)

#define CMD_BUF_ADDR            0x20180
#define CHIP_VERSION_ADDR       0x1401E
#define SYS_STATUS_R0           0x20100
#define SYS_STATUS_R1           0x20101
#define SYS_STATUS_R2           0x20102

#define TUNER_READY             0xAFFE42

#define WR32_BURST_AUTO         (WR_ACCESS | BURST_MODE | AUTO_INC)
#define RD32_BURST_AUTO         (            BURST_MODE | AUTO_INC)
#define WR24_BURST_AUTO         (WR_ACCESS | DATA24 | BURST_MODE | AUTO_INC)
#define RD24_BURST_AUTO         (            DATA24 | BURST_MODE | AUTO_INC)

/* cmd header */
#define CMD_STAT(x)             ((x >> 21) & 7)
#define CMD_ID(x)               (x << 8)

#define ERR_CRC                 (4)
#define ERR_CID                 (2)
#define ERR_COLL                (1)

#define SPI_SPEED               1000000
#define SPI_MODE                3
#define SPI_BITS                8

#ifndef TDA77XX_FIRMWARE_PATH
/* rayleigh */
//#define TDA77XX_FIRMWARE_PATH   "STA710_OM_v1.1.7.bin"
//#define TDA77XX_FIRMWARE_PATH   "TDA7707_OM_v3.8.0.bin"
/* rsb */
#define TDA77XX_FIRMWARE_PATH   "TDA7707_BC_v3.9.0.bin"

//#define TDA77XX_FIRMWARE_PATH   "TDA7707_OM_v4.0.6.bin"
//#define TDA77XX_FIRMWARE_PATH   "TDA7707_OM_v4.0.9.bin"
//#define TDA77XX_FIRMWARE_PATH   "TDA7707_OM_v4.0.11.bin"
#endif

#define AM_MODE_EU              0x05
#define AM_BAND_LOW                144
#define AM_BAND_HIGH             30000

#define FM_MODE                 0x01
#define FM_BAND_LOW              87000
#define FM_BAND_HIGH            108000

#define DAB_MODE3               0x03
#define DAB_BAND3_LOW           168160
#define DAB_BAND3_HIGH          239200

#define DAB_MODEL               0x04
#define DAB_BANDL_LOW           1452960
#define DAB_BANDL_HIGH          1490624

/* CSR of RDS */
#define RDS_RESET               (1 << 9)
#define RDS_ENABLE              1
#define RDS_BNE                 (1 << 20)
#define RDS_BOFL                (1 << 21)
#define RDS_SYNC                (1 << 22)
#define RDS_DATARDY             (1 << 23)

int  TDA77XX_GetQuality(void);
void TDA77XX_Mute(int mute);

/*
 * Tuner common
 */
int RadioReset(void);

/*
 * SI4702 Sparkfun breakout
 */

void SI4703_Init(void) ;
void SI4703_Status(void) ;
int  SI4703_Tune(int freq);
void SI4703_Volum(int vol);
int  SI4703_Seek(bool up);
void SI4703_ReadRDS(char* buffer, long timeout);

#endif // TUNERDEFINES_H
