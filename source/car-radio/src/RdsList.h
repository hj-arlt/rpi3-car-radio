/*
 * rds.h
 *
 *  Created on: 18.03.2017
 *      Author: hj.arlt@online.de
 */

#ifndef RDS_H_
#define RDS_H_

#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {
#endif

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <linux/types.h>

typedef struct {
        uint16_t     pi;
        char         name[32];
        char         icon[32];
} pi_struct;

typedef struct {
        char         name[18];
} pty_struct;

typedef struct {
        char         name[5];
        uint32_t     freq;
} dab_channels;

/* tables */
extern const dab_channels dab[];
extern const pi_struct pi_data[];
extern const pty_struct pty[];

#if defined(__cplusplus) || defined(c_plusplus)
};
#endif

#endif /* RDS_H_ */
