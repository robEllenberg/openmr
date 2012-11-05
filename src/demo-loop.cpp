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
#include "ach-setup.h"
using namespace Hubo;

int debug = 0;
int hubo_debug = 1;
hubo_channels chan;

void huboLoop() {
    // get initial values for hubo
    struct hubo_ref H_ref;
    struct hubo_state H_state;
    struct hubo_param H_param;
    
    setup_memory<hubo_ref>(&H_ref,&(chan.hubo_ref));
    setup_memory<hubo_state>(&H_state,&(chan.hubo_state));
    setup_memory<hubo_param>(&H_param,&(chan.hubo_param));

    // time info
    struct timespec t;
    int interval = 10000000; // 100 hz (0.01 sec)

    // get current time
    clock_gettime( 0,&t);
    double f = 0.2;		// frequency
    double A = 0.3;// 1.0;
    double dir = -1.0;
    int jnt = WST;
    ach_status_t r;
    size_t fs;
    while(1) {
        // wait until next shot
        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

        /* Get latest ACH message */
        r = (ach_status_t)ach_get( &(chan.hubo_ref), &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(hubo_debug) {
                printf("Ref r = %s\n",ach_result_to_string((ach_status_t)r));}
        }
        else{   assert( sizeof(H_ref) == fs ); }
        r = (ach_status_t)ach_get( &(chan.hubo_state), &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(hubo_debug) {
                printf("State r = %s\n",ach_result_to_string((ach_status_t)r));}
        }
        else{   assert( sizeof(H_state) == fs ); }

        double jntDiff = H_state.joint[jnt].pos - H_ref.ref[jnt];
        printf("\033[2J");
        printf("%s: Cur = %f \t  Diff = %f \t State = %f \t Ref = %f\n",H_param.joint[jnt].name,H_state.joint[jnt].cur, jntDiff, H_state.joint[jnt].pos, H_ref.ref[jnt]);	

        double jntTmp = A*(-cos(f*2.0*pi*t.tv_sec)-1.0)/2.0;
        H_ref.ref[jnt] = dir*jntTmp;

        ach_put( &(chan.hubo_ref), &H_ref, sizeof(H_ref));
        t.tv_nsec+=interval;
        tsnorm(&t);
    }

}
int main(int argc, char **argv) {

    int i = 1;
    while(argc > i) {
        if(strcmp(argv[i], "-d") == 0) {
            debug = 1;
        }
        i++;
    }

    /* RT */
    struct sched_param param;
    /* Declare ourself as a real time task */
    param.sched_priority = MY_PRIORITY;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
        perror("mlockall failed");
        exit(-2);
    }

    /* Pre-fault our stack */
    stack_prefault();
        

    /* open ach channel */
    int r = ach_open(&(chan.hubo_ref), HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan.hubo_param, HUBO_CHAN_PARAM_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan.hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == r );

    huboLoop();
    pause();
    return 0;

}
