/*
 * NvMemory.cpp
 *
 *  Created on: 23.11.2017
 *      Author: hj.arlt@online.de
 */

#include <QDebug>
#include <QCoreApplication>
#include "NvMemory.h"

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>
#if defined(__cplusplus) || defined(c_plusplus)
};
#endif

namespace
{

#define EEPROM_I2C_ADDR   0x54   // 7bit , 0xA8 8bit
static char device[] = "/dev/i2c-1";

#define DEVICE_SIZE       8192   // at24C64
#define BLOCK_SIZE_READ   64
#define BLOCK_SIZE_WRITE  32

static struct i2c_msg msg[2];
/* block buffer */
static uint8_t txBuffer[BLOCK_SIZE_READ];
static uint8_t rxBuffer[BLOCK_SIZE_WRITE+2];

/* ***************************************************************
 * I2C
 * ***************************************************************/

static int i2cInit(const char *device, int devId)
{
    int fd;

    if ((fd = open(device, O_RDWR | O_NONBLOCK)) < 0)
        return -1;
    if (ioctl(fd, I2C_SLAVE, devId) < 0)
    {
        close(fd);
        return -1;
    }
#if 0
    uint64_t funcs;
    if (ioctl(fd, I2C_FUNCS, &funcs) >= 0) {
        if (funcs & I2C_FUNC_I2C)
            printf("I2C ioctl_write support\n");
        if (funcs & I2C_FUNC_SMBUS_WORD_DATA)
            printf("I2C ioctl_smbus_write support\n");
    }
    else {
        printf("ERR. I2C funcs support\n");
    }
#endif
    return fd;
}

static int i2cWrRd(int fd, uint8_t addr, uint8_t*txbuf, int tlen, uint8_t*rxbuf, int rlen)
{
    int err;
    struct i2c_rdwr_ioctl_data i2cdata;

    i2cdata.nmsgs = 1;
    if (tlen)
    {
        msg[0].addr = addr;
        msg[0].flags = 0;
        msg[0].len = tlen;
        msg[0].buf = txbuf;
        if (rlen)
        {
            msg[1].addr = addr;
            msg[1].flags = I2C_M_RD;
            msg[1].len = rlen;
            msg[1].buf = rxbuf;
            i2cdata.nmsgs = 2;
        }
    }
    else
    {
        msg[0].addr = addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = rlen;
        msg[0].buf = rxbuf;
    }
    i2cdata.msgs = msg;

    err = ioctl(fd, I2C_RDWR, &i2cdata);
    if (err < 0)
    {
//        qDebug() << "NvMemory: err: i2c on 0x%x " << msg[0].addr;
    }
    return err;
}

/* ***************************************************************
 * Eeprom
 * ***************************************************************/

static int eepCheck()
{
    int fd = i2cInit(device, EEPROM_I2C_ADDR);
    if (fd < 0)
        return -1;
    close(fd);
    return 0;
}

int eepRead(int addr, uint8_t *pdata, int len)
{
    int fd, size, rem = len;
    uint8_t *pAddr = pdata;

    if ((addr + len) > DEVICE_SIZE) {
        qDebug() << "NvMemory: WARN size too big " << (addr + len) << " > " << DEVICE_SIZE;
        return -1;
    }
    fd = i2cInit(device, EEPROM_I2C_ADDR);
    if (fd < 0) {
        qDebug() << "NvMemory: error init!!";
        return -1;
    }
    /* set startaddress */
    txBuffer[0] = (addr >> 8) & 0xFF; // Addr 15:8 0x0000
    txBuffer[1] = addr &0xFF;         // Addr  7:0
    if (i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, 2, rxBuffer, 0) < 0) {
        qDebug() << "NvMemory: error on WR addr!!";
        close(fd);
        return -1;
    }
    /* Current Address Read */
    while (rem)
    {
        if (rem > BLOCK_SIZE_READ)
             size = BLOCK_SIZE_READ;
        else size = rem;
        if (i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, 0, rxBuffer, size) < 0) {
            qDebug() << "NvMemory: error on RD data!!";
            close(fd);
            return -1;
        }
        memcpy(pAddr, rxBuffer, size);
        rem -= size;
        pAddr += size;
    }
    close(fd);
    return 0;
}

int eepWrite(int addr, uint8_t *pdata, int len)
{
    int size, baddr = addr, rem = len;
    uint8_t *pAddr = pdata;

    if ((addr + len) > DEVICE_SIZE) {
        qDebug() << "NvMemory: WARN size too big " << (addr + len) << " > " << DEVICE_SIZE;
        return -1;
    }
    int fd = i2cInit(device, EEPROM_I2C_ADDR);
    if (fd < 0) {
        qDebug() << "NvMemory: error init!!";
        return -1;
    }
    /* next block addr */
    baddr = (addr / BLOCK_SIZE_WRITE) * BLOCK_SIZE_WRITE;
    baddr += BLOCK_SIZE_WRITE;
    size = baddr - addr; // rest of block
    if (size > len)
        size = len;

    /* wr first block */
    txBuffer[0] = (addr >> 8) & 0xFF; // Addr 15:8 0x0000
    txBuffer[1] = addr &0xFF;         // Addr  7:0
    memcpy(&txBuffer[2], pAddr, size);
    if (i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, size+2, rxBuffer, 0) < 0) {
        qDebug() << "NvMemory: error on WR data!!";
        close(fd);
        return -1;
    }

    /* wait on ready */
    while(i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, 0, rxBuffer, 1) < 0) {
        usleep(1);
    }
    rem   -= size;
    pAddr += size;
    while (rem)
    {
        if (rem > BLOCK_SIZE_WRITE)
             size = BLOCK_SIZE_WRITE;
        else size = rem;

        txBuffer[0] = (baddr >> 8) & 0xFF; // Addr 15:8
        txBuffer[1] = baddr &0xFF;         // Addr  7:0
        memcpy(&txBuffer[2], pAddr, size);
        if (i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, size+2, rxBuffer, 0) < 0) {
            qDebug() << "NvMemory: error on WR data!!";
            close(fd);
            return -1;
        }
        baddr += BLOCK_SIZE_WRITE;
        rem   -= size;
        pAddr += size;
        /* wait on ready */
        while(i2cWrRd(fd, EEPROM_I2C_ADDR, txBuffer, 0, rxBuffer, 1) < 0) {
            usleep(1);
        }
    }
    close(fd);
    return 0;
}

void eepTest(void)
{
    uint8_t *pBuffer;
    int i;
    pBuffer = (uint8_t *)malloc(DEVICE_SIZE);
    if (! pBuffer) {
        qDebug() << "NvMemory: error malloc!!";
        return;
    }
    memset(pBuffer, 0x55, DEVICE_SIZE);
    eepWrite(0, pBuffer, DEVICE_SIZE);

    eepRead(0, pBuffer, DEVICE_SIZE);
    for (i=0; i<DEVICE_SIZE; i++) {
        if (pBuffer[i] != 0x55)
            break;
    }
    if (i<DEVICE_SIZE) {
        qDebug() << "NvMemory: error test 55 on addr " << i;
        goto eeperr;
    }

    memset(pBuffer, 0xAA, DEVICE_SIZE);
    eepWrite(0, pBuffer, DEVICE_SIZE);

    eepRead(0, pBuffer, DEVICE_SIZE);
    for (i=0; i<DEVICE_SIZE; i++) {
        if (pBuffer[i] != 0xAA)
            break;
    }
    if (i<DEVICE_SIZE) {
        qDebug() << "NvMemory: error test AA on addr " << i;
        goto eeperr;
    }

eeperr:
    free(pBuffer);
}

int eepGetTag(int tag, int *value)
{
    (void)tag;
    if (value) {
        *value = 0xFF;
        return 0;
    }
    return -1;
}

int eepSetTag(int tag, int value)
{
    (void)tag;
    (void)value; // suppres compiler warning
    return 0;
}

} // namespace

/* ********************************************************************
 * class NvMemory
 ********************************************************************** */

NvMemory::NvMemory()
    : m_fileutil()
    , m_mutex()
{
    m_found = false;
    if (eepCheck() < 0) {
        qDebug() << "NvMemory: no HW -> emulation..";
        return;
    }
    m_found = true;

    eepTest();
}

NvMemory::~NvMemory()
{
    qDebug() << "NvMemory::close";
}

int NvMemory::readEeprom(int addr, quint8 *pdata, int len)
{
    if (!m_found)
        return -1;
    QMutexLocker lock(&m_mutex);
    return eepRead(addr, pdata, len);
}

int NvMemory::writeEeprom(int addr, quint8 *pdata, int len)
{
    if (!m_found)
        return -1;
    QMutexLocker lock(&m_mutex);
    return eepWrite(addr, pdata, len);
}

int NvMemory::getEeprom(int tag, int *value)
{
    return eepGetTag(tag, value);
}

int NvMemory::setEeprom(int tag, int value)
{
    return eepSetTag(tag, value);
}
