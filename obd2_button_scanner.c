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

#include "console_in_thd.h"
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

static char *console_input_buffer;

// TODO: Collection of all monitored vehicle variables
static struct vehicle_struct {
    int speed;  //mph
    int gear;

    // Registered APIs TODO: Separate out to other struct
    void (*send_string2obd2_fp)(char **buffer); // pointer to sendstring function

} vehicle;

int log_fd = 0;
bool logging_on = false;
bool record = false;
bool waiting_for_delimiter = false;
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

    if (0 > serial_connect (obd2_port, SERIAL_PORT_FQN, 38400))
    {
        printf ("Error connecting to obd2_port\n");
        exit -1;
    }

//    serial_add_delimiter (obd2_port, '>');

    // Send setup messages
        serial_send(obd2_port, "atz\r", 4);
        while ('>' != serial_blocking_get(obd2_port));
    printf ("0\r");

        serial_send (obd2_port, "atl1\r", 5); //linefeeds off
        while ('>' != serial_blocking_get(obd2_port));
    printf ("1\r");

        serial_send (obd2_port, "atsp0\r", 6); //automatic protocol search
        while ('>' != serial_blocking_get(obd2_port));
    printf ("2\r");

        serial_send (obd2_port, "ath0\r", 5); //headers off
        while ('>' != serial_blocking_get(obd2_port));
    printf ("3\r");

        serial_send (obd2_port, "ats1\r", 5); //print spaces   <-- //TODO: Look into removing spaces to speed up
        while ('>' != serial_blocking_get(obd2_port));
    printf ("4\r");

        serial_send (obd2_port, "atal\r", 5); // allow long messages
        while ('>' != serial_blocking_get(obd2_port));
    printf ("5\r");

        serial_send (obd2_port, "ate0\r", 5); //echo off
        while ('>' != serial_blocking_get(obd2_port));
    printf ("6\n");

    //TODO: use to mask responses
    // fputs ("atar\n", serial_port_fp); fflush(serial_port_fp); //
    //fputs ("atma\n", serial_port_fp); fflush(serial_port_fp);

    /*
       1) Find capabilities
       2) 
     */

    start_console_handler_thread (NULL);




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

    mainloop_state_flags = mainloop_state_flags; //Keep compiler happy

    //Main Loop
    printf ("Entering main loop\n");fflush(stdout);
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
                    if (console_input_buffer[0] == '*')
                    {
                        //Scanner command
                        switch (console_input_buffer[1])
                        {
                            case 'l':
                                if (console_input_buffer[2] == '1')
                                {
                                    //Logging on
                                    log_fd = open ("./log_file", O_CREAT | O_RDWR, S_IRWXU | S_IRWXG | S_IRWXO);
                                    if (-1 == log_fd)
                                    {
                                        perror("Opening log file");
                                    }
                                    else
                                    {
                                        logging_on = true;
                                    }
                                }
                                else
                                {
                                    //Logging off
                                    logging_on = false;
                                    if (log_fd)
                                        close (log_fd);
                                }
                                break;
                            case 'r':
                                record = true;
                                serial_put(obd2_port, '\r');
                                break;
                            case 'q':
                                //if logging is on, close files
                                run = 0;
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        //printf("Send str: ");fflush(stdout);//TESTTESTTEST
                        int i = 0;
                        struct timespec pause;
                        pause.tv_sec = 0;
                        pause.tv_nsec = 30000000;

                        if (logging_on)
                        {
                            char ch[] = {'-', '>'};
                            write (log_fd, &ch, 2);
                        }

                        while (console_input_buffer[i])
                        {
                            serial_put (obd2_port, console_input_buffer[i]);
                            if (logging_on)
                            {
                                write (log_fd, &console_input_buffer[i], 1);
                            }
                            //printf ("0x%x ", console_input_buffer[i]);fflush(stdout);//TESTTESTTEST
                            //printf ("%c", console_input_buffer[i]);fflush(stdout);//TESTTESTTEST
                            i++;
                            nanosleep (&pause, NULL); //TODO: Try without delay, now that input is working...
                        }
                        serial_put (obd2_port, 0x0d);
                        printf ("\n");fflush(stdout);//TESTTESTTEST

                        if (logging_on)
                        {
                            char ch[] = {'\r', '<', '-'};
                            write (log_fd, &ch, 3);
                        }
                        //serial_put (obd2_port, '\n');fflush(stdout);//TESTTESTTEST
                    }

                    free (console_input_buffer);
                    mainloop_state_flags &= ~SEND_STRING2OBD2;
                }

                if (DELIMITER_FOUND & mainloop_state_flags)
                {
                    mainloop_state_flags &= ~DELIMITER_FOUND;
                }
                break;

            case ETIMEDOUT:
                {
                    // Manual method of stopping the system.  Simply 'touch /stop' in other console.
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
                        //Normal operation
                        if (waiting_for_delimiter)
                        {
                            /*
                            char ch;
                            while (serial_available(obd2_port))
                            {
                                ch = serial_get(obd2_port);

                            // Wait for delimiter
                            char *found = strchr (ch, s->delimiters);
                            if (found)
                            {
                                char *buffer
                            }
                            */
                        }
                        else
                        {
                            // Not waiting for delimiter
                            char ch;
                            while (serial_available(obd2_port))
                            {
                                ch = serial_get(obd2_port);

                                if (record && '>' == ch)
                                {
                                    static int count = 0;
                                    if (0 == count)
                                    serial_send(obd2_port, "0104\r", 5); //Engine Load
                                    else if (1 == count)
                                    serial_send(obd2_port, "0105\r", 5); //Coolant Temp
                                    else if (2 == count)
                                    serial_send(obd2_port, "010c\r", 5); //Engine speed
                                    else if (3 == count)
                                    serial_send(obd2_port, "010d\r", 5); //Vehicle speed

                                    count++;
                                    count &= 0x03;
                                }

                                printf ("%c", ch);fflush(stdout);
                                if (logging_on)
                                {
                                    write (log_fd, &ch, 1);
                                }
                                // TODO: If logging is on, write to file
                            }
                        }
                    }
                }

                //calculate next timeout (cond_timedwait accepts timespec)
                clock_gettime (CLOCK_MONOTONIC, &timeout);
                timeout.tv_nsec += 100000000; //1ms
                if (timeout.tv_nsec > 999999999)
                {
                    timeout.tv_sec++;
                    timeout.tv_nsec -= 1000000000; 
                }

                break;

            default:
                printf ("External Signal detected in main loop");fflush(stdout);
                break;
        }
    }

    // TODO: Close log files
    if (log_fd) close(log_fd);

    // TODO: Stop threads

    // TODO: Close serial port
    serial_clear_delimiters (obd2_port);

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

void send_string2obd2 (char **buffer)
{
    if ((console_input_buffer = strdup (*buffer)))
    {
        pthread_mutex_lock (&es_mutex);
        mainloop_state_flags |= SEND_STRING2OBD2;
        pthread_mutex_unlock (&es_mutex);
        pthread_cond_signal (&mainloop_state);
    }
    else
    {
        printf ("ERROR: could not allocate space for obd2send\n");fflush(stdout);
    }
}
