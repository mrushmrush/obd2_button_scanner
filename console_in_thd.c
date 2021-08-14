//****************************************************************************
//  console_handler_thd.c
//  
//  Description:
//  
//****************************************************************************


#define _GNU_SOURCE 
//********************** Include Files ***************************************
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <pthread.h>
#include <syslog.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sched.h>
#include <fcntl.h>

#include "console_in_thd.h"

//********************** Local Constants *************************************
//loop_state flags
#define STOP_THREAD       0x00000001
#define RECEIVED_CHAR     0x00000002   //received a character from console port
#define RECEIVED_PING     0x00000004   //received a PING from console port

//Mainloop conditions
#define RECEIVED_DATA 0x00000001

//********************** Local Typedefs **************************************

//********************** Local Variables *************************************
static bool stop_thread_flag = false;



//********************** Global instances ************************************

//****************** Local Function Prototypes *******************************
static void *console_handler_thread(void *dummy);

//****************** Extern Function Prototypes *******************************
extern void send_string2obd2 (char **buffer);

//****************************************************************************
//  start_console_handler_thread()
//  _mutex_recursive
//  Encapsulate thread startup features
//****************************************************************************
pthread_t start_console_handler_thread (void *arg)
{
    pthread_t console_handler_tid;
    pthread_attr_t attr;
    struct sched_param sched_param_structure;

    printf ("Starting Console Handler thread\n");fflush(stdout); //TESTTESTTEST
    syslog (LOG_DEBUG, "Starting Console Handler thread");

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);   
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    sched_param_structure.sched_priority = sched_get_priority_max(SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &sched_param_structure);
    stop_thread_flag = false;
    pthread_create (&console_handler_tid, &attr, console_handler_thread, arg);

    return (console_handler_tid);
}

//****************************************************************************
//  stop_console_handler_thd()
//  
//  Thread API: Encapsulate thread shutdown features
//****************************************************************************
int stop_console_handler_thd(pthread_t tid)
{
    stop_thread_flag = true;
    //and wait...
    return (pthread_join(tid, NULL)); //returns with ETIMEDOUT
    //if unable to join
}

#define CONSOLE_ACTIVITY_TIMEOUT 5
//****************************************************************************
//  console_handler_thread()
//  
//****************************************************************************
static void *console_handler_thread(void *data)
{

    printf ("Console Handler Thread started\n");fflush(stdout); //TESTTESTTEST

    // Open the console port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)

    char *input_buffer = NULL;
    int num_chars;
    size_t input_buffer_size = 0;

    size_t num;

        int i = 0;
    while (!stop_thread_flag)
    {
        num = getline (&input_buffer, &input_buffer_size, stdin);
        input_buffer[num-1] = 0; //get rid of /n

#if 0        //TESTTESTTEST
        printf ("console sending:\n");
        i = 0;
        while (input_buffer[i])
        {
            printf ("%x\n", (int)input_buffer[i]);fflush(stdout);
            i++;
        }
#endif        //TESTTESTTEST


        send_string2obd2 (&input_buffer);
    } //end while

    pthread_exit(NULL);

    return NULL;
}
