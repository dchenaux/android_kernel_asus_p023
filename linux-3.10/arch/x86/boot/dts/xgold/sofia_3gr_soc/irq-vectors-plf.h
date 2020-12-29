/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* Some vectors may be platform dependent such as EINT
 * Move those vectors from irq-vectors to there :g!/EXI/d
 * use #ifdef MY_PLATFORM in case of shared IRQ
 * */

#ifndef _IRQ_VECTORS_PLF_H
#define _IRQ_VECTORS_PLF_H
#if defined(CONFIG_Z300CG) || defined(CONFIG_Z300C)
#define EXI3 0
#define EXI4 40
#define EXI5 41
#define EXI7 0
#define EXI13 55
#define EXI15 0
#define EXI9 61
#define EXI10 122
#define EXI11 121
#define EXI8 118
#define EXI14 0

#define EXI0 56
#define EXI1 57
#define EXI2 58
#define EXI6 60
#define EXI12 0
#endif


#if defined(CONFIG_Z380C)
#define EXI3 121
#define EXI4 40
#define EXI5 41
#define EXI7 0
#define EXI13 55
#define EXI15 0
#define EXI9 61
#define EXI10 122
#define EXI11 0
#define EXI8 118
#define EXI14 0

#define EXI0 56
#define EXI1 57
#define EXI2 58
#define EXI6 60
#define EXI12 0
#endif


#endif /*_IRQ_VECTORS_PLF_H */
