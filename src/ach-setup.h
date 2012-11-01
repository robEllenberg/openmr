/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include "hubo.h"

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                           as the priority of kernel tasklets
                           and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                                      guaranteed safe to access without
                                      faulting */

// Timing info
#define NSEC_PER_SEC    1000000000

#ifndef _HUBO_ACH_NAMESPACE_
#define _HUBO_ACH_NAMESPACE_

namespace Hubo{

    struct timeb {
        time_t   time;
        unsigned short millitm;
        short    timezone;
        short    dstflag;
    };

    struct hubo_channels {
        // ach channels
        ach_channel_t chan_hubo_ref;      // hubo-ach
        ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
        ach_channel_t chan_hubo_state;    // hubo-ach-state
        ach_channel_t chan_hubo_param;    // hubo-ach-param
    };

    void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset( dummy, 0, MAX_SAFE_STACK );
    };

    static inline void tsnorm(struct timespec *ts){

        //	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
        // calculates the next shot
        while (ts->tv_nsec >= NSEC_PER_SEC) {
            //usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
            ts->tv_nsec -= NSEC_PER_SEC;
            ts->tv_sec++;
        }
    };

    hubo_channels setup_channels(){
        /* RT */
        struct sched_param param;

        hubo_channels chan;
        /* Declare ourself as a real time task */
        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            perror("sched_setscheduler failed");
            //TODO Make these openrave exceptions / asserts?
            exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            perror("mlockall failed");
            //TODO Make these openrave exceptions / asserts?
            exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

        /* open ach channel */
        int r = ach_open(&chan.chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
        assert( ACH_OK == r );

        r = ach_open(&chan.chan_hubo_param, HUBO_CHAN_PARAM_NAME , NULL);
        assert( ACH_OK == r );

        r = ach_open(&chan.chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
        assert( ACH_OK == r );
        return chan;
    }
}
#endif
