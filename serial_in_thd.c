//****************************************************************************
//  serial_handler_thd.c
//  
//  Description:
//  
//****************************************************************************


//********************** Include Files ***************************************
#define _GNU_SOURCE 
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

#include "serial_in_thd.h"

//********************** Local Constants *************************************
//loop_state flags
#define STOP_THREAD       0x00000001
#define RECEIVED_CHAR     0x00000002   //received a character from serial port
#define RECEIVED_PING     0x00000004   //received a PING from serial port

//Mainloop conditions
#define RECEIVED_DATA 0x00000001

//********************** Local Typedefs **************************************

//********************** Local Variables *************************************
static bool stop_thread_flag = false;



//********************** Global instances ************************************

//****************** Local Function Prototypes *******************************
static void *serial_handler_thread(void *dummy);
static int read_elm327(int fd, char *buf, int n);

//****************************************************************************
//  start_serial_handler_thread()
//  _mutex_recursive
//  Encapsulate thread startup features
//****************************************************************************
pthread_t start_serial_handler_thread (void *arg)
{
    pthread_t serial_handler_tid;
    pthread_attr_t attr;
    struct sched_param sched_param_structure;

    printf ("Starting Serial Handler thread\n");fflush(stdout); //TESTTESTTEST
    syslog (LOG_DEBUG, "Starting Serial Handler thread");

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);   
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    sched_param_structure.sched_priority = sched_get_priority_max(SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &sched_param_structure);
    stop_thread_flag = false;
    pthread_create (&serial_handler_tid, &attr, serial_handler_thread, arg);

    return (serial_handler_tid);
}

//****************************************************************************
//  stop_serial_handler_thd()
//  
//  Thread API: Encapsulate thread shutdown features
//****************************************************************************
int stop_serial_handler_thd(pthread_t tid)
{
    stop_thread_flag = true;
    //and wait...
    return (pthread_join(tid, NULL)); //returns with ETIMEDOUT
    //if unable to join
}

#define SERIAL_ACTIVITY_TIMEOUT 5
//****************************************************************************
//  serial_handler_thread()
//  
//****************************************************************************
static void *serial_handler_thread(void *data)
{

    printf ("Serial Handler Thread started\n");fflush(stdout); //TESTTESTTEST

    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    FILE *serial_port =  data;

    char *input_buffer = malloc(256);
    int num_chars;
    size_t input_buffer_size = 256;



    while (!stop_thread_flag)
    {
        getline (&input_buffer, &input_buffer_size, serial_port);
        printf ("%s", input_buffer);
//        fputs (input_buffer, obd_file_out);
    } //end while
    // Do whatever work remains before exiting
    // ...

    pthread_exit(NULL);

    return NULL;
}


#if 0
/// Collect data up to the next prompt
/** Reads up to the next '>'
   \param buf buffer to fill
   \param n size of buf
   \return number of bytes put in buf, or -1 on error
*/
int read_elm327(int fd, char *buf, int n) {
	char *bufptr = buf; // current position in buf

	struct timeval start,curr; // For timing out
	if(0 != gettimeofday(&start, NULL)) {
		perror("Couldn't gettimeofday");
		return -1;
	}
	memset((void *)buf, '\0', n);
	int retval = 0; // Value to return
	int nbytes; // Number of bytes read
	do {
		nbytes = read(fd, bufptr, buf+n-bufptr-1);
		if(-1 == nbytes && EAGAIN != errno) {
			perror("Error in readserialdata");
		}
		if(-1 != nbytes) {
			// printf("Read bytes '%s'\n", bufptr);
			retval += nbytes; // Increment bytecount
			bufptr += nbytes; // Move pointer forward
		}
		if(0 != gettimeofday(&curr, NULL)) {
			perror("Couldn't gettimeofday");
			return -1;
		}
//		if(OBDCOMM_TIMEOUT < 1000000l*(curr.tv_sec - start.tv_sec) +
//			(curr.tv_usec - start.tv_usec)) {
        if (2 < (curr.tv_sec - start.tv_sec))
        {
			printf("Timeout!\n");
			return -1;
		}
	} while (retval == 0 || bufptr[-1] != '>');

	return retval;
}

void blindcmd(int fd, const char *cmd, int no_response) {
	char outstr[1024];
	snprintf(outstr, sizeof(outstr), "%s%s\0", cmd, OBDCMD_NEWLINE);
	appendseriallog(outstr, SERIAL_OUT);
	write(fd,outstr, strlen(outstr));
	if(0 != no_response) {
		sleep(1);
		readtonextprompt(fd);
	}
}
#endif
