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

#include "serial_handler_thd.h"

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
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return NULL;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B500000);
    cfsetospeed(&tty, B500000);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return NULL;
    }

    FILE *obd_file_out = fopen ("./obdout.txt", "w");
    char input_buffer[256];
    char output_buffer[256];
    int num_chars;


    strcpy (output_buffer, "atsp0\n");
    write (serial_port, output_buffer,strlen(output_buffer));
    num_chars = read_elm327(serial_port, input_buffer, 256);
    fwrite (input_buffer, 1, num_chars, obd_file_out);

#if 0



    printf ("%s", input_buffer);

    fputs ("atl1\n", serial_stream); fflush(serial_stream);
    getline (&input_buffer, &input_buffer_size, serial_stream);
    fwrite (obd_file_out, input_buffer, 
    printf ("%s", input_buffer);

    fputs ("atal\n", serial_stream); fflush(serial_stream);
    getline (&input_buffer, &input_buffer_size, serial_stream);
    printf ("%s", input_buffer);

    fputs ("atma\n", serial_stream); fflush(serial_stream);

    while (!stop_thread_flag)
    {
        getline (&input_buffer, &input_buffer_size, serial_stream);
        printf ("%s", input_buffer);
        fputs (input_buffer, obd_file_out);
    } //end while
#endif
    // Do whatever work remains before exiting
    // ...

    fclose (obd_file_out);
    close (serial_port);
    pthread_exit(NULL);

    return NULL;
}


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

#if 0
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
