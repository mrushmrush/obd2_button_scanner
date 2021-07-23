//****************************************************************************
//  
//  Copyright (c) 2019  Lede Engineering, Inc.
//  All Rights Reserved
//  No portions of this material may be reproduced in any form without the
//  written permission of:
//          Lede Engineering, Inc.
//          8007 Sleepy Lagoon Way
//          Flowery Branch, GA 30542
//  All information contained in this document is Lede Engineering
//  company private, proprietary, and trade secret.
//  
//  
//****************************************************************************
//  doxygen header
//  
//  Filename:       obd2_button_scanner.c
//  Author:         Mark Rush
//  Creation Date:  Apr 3, 2019
//  
//  Description:
//  
//  
// gcc obd2_button_scanner.c serial_in_thd.c serial_in_thd.h -lpthread
//
//****************************************************************************


#define _GNU_SOURCE
//********************** Include Files ***************************************
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>  //for getopt, symlink
#include <string.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <syslog.h>
#include <termios.h>

#include "serial_in_thd.h"
#include "uart.h"


//********************** Local Constants *************************************

//main-loop status flag states
#define STOP_MAINLOOP               0x00000001
#define SEND_STRING2OBD2            0x00000002
#define DELIMITER_FOUND             0x00000004

//********************** Local Typedefs **************************************
#define SERIAL_PORT_FQN "/dev/ttyUSB0"


//********************** Local Variables *************************************

//signalling for main loop
static pthread_cond_t  mainloop_state;
static pthread_mutex_t es_mutex = PTHREAD_MUTEX_INITIALIZER;
static unsigned int    mainloop_state_flags = 0;

//********************** Global extern instances ************************************

//****************** Local Function Prototypes *******************************


//*****************************************************************************
//*****************************************************************************



//****************************************************************************
// main()
//  
//****************************************************************************
int main (int argc, char *argv[])
{

    // We want to run in the background
    //daemon (0, 0);

    serial_t *obd2_port = serial_create ();

    if (!obd2_port)
    {
        printf ("Error creating obd2_port\n");
        exit -1;
    }

    if (0 > serial_connect (obd2_port, SERIAL_PORT_FQN, 500000))
    {
        printf ("Error connecting to obd2_port\n");
        exit -1;
    }



    // Send setup messages
    //    serial_send (obd2_port, "atl0\n", 5); //linefeeds off
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?
    // add '>' as delimiter?????

    //    serial_send (obd2_port, "ath0\n", 5); //headers off
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //    serial_send (obd2_port, "ats1\n", 5); //print spaces   <-- //TODO: Look into removing spaces to speed up
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //    serial_send (obd2_port, "atal\n", 5); // allow long messages
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //    serial_send (obd2_port, "ate0\n", 5); //echo off
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //    serial_send (obd2_port, "atsp0\n", 6); //automatic protocol search
    //    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //TODO: use to mask responses
    // fputs ("atar\n", serial_port_fp); fflush(serial_port_fp); //
    //fputs ("atma\n", serial_port_fp); fflush(serial_port_fp);


    //**************************************************************************
    // Begin main loop
    //**************************************************************************
    struct timespec timeout;
    pthread_condattr_t pt_attr;  //attribs to set clock in cond_wait
    int rval;

    //initialize mainloop syncronization variables
    //(Note: mutex must be locked when we enter the pthread_cond_timedwait)
    pthread_mutex_lock (&es_mutex);
    pthread_condattr_init(&pt_attr);
    pthread_condattr_setclock(&pt_attr, CLOCK_MONOTONIC);
    pthread_cond_init (&mainloop_state, &pt_attr);
    clock_gettime (CLOCK_MONOTONIC, &timeout);
    //timeout.tv_sec += ;
    timeout.tv_nsec += 0;

    mainloop_state_flags = mainloop_state_flags; //TODO: Until supervisor mainloop has to handle events

    printf ("Entering main loop");fflush(stdout);
    //Main Loop
    int run = 1; //TODO: cleanup
    while (run)
    {
        //wait here for event or for timeout
        rval = pthread_cond_timedwait (&mainloop_state, &es_mutex, &timeout);
        switch (rval)
        {
            case 0:  //awakened by signal flag
                //
                if (STOP_MAINLOOP & mainloop_state_flags)
                {
                    printf ("STOPPING MAINLOOP");fflush(stdout);
                    run = 0;
                    mainloop_state_flags &= ~STOP_MAINLOOP;
                }

                if (SEND_STRING2OBD2 & mainloop_state_flags)
                {
                    mainloop_state_flags &= ~SEND_STRING2OBD2;
                }

                if (DELIMITER_FOUND & mainloop_state_flags)
                {
                    mainloop_state_flags &= ~DELIMITER_FOUND;
                }
                break;

            case ETIMEDOUT:
                {
                //XXX: Manual method of stopping the system.  Simply 'touch'
                // 'stop' in root directory.
                struct stat   buffer; 
                static int r;
                if (0 == stat ("/stop", &buffer))
                {
                    printf ("Manual 'Stop' file found");fflush(stdout);
                    unlink ("/stop");
                    run = 0;
                }
                else
                {
                    while (serial_available(obd2_port))
                    {
                        printf ("%c", serial_get(obd2_port));
                    }

                    //calculate next timeout (cond_timedwait accepts timespec)
                    clock_gettime (CLOCK_MONOTONIC, &timeout);
                    //timeout.tv_sec += timeout;
                }
                }

                break;

            default:
                printf ("External Signal detected in main loop");fflush(stdout);
                break;
        }
    }

    return 0;
}  //End Main

void delimiter_notify(char delim)
{
    //switch on delimiter?
    pthread_mutex_lock (&es_mutex);
    mainloop_state_flags |= DELIMITER_FOUND;
    pthread_mutex_unlock (&es_mutex);
    pthread_cond_signal (&mainloop_state);
}

void send_string2obd2 (char *buffer)
{

}
