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


//********************** Local Constants *************************************


//********************** Local Typedefs **************************************
#define SERIAL_PORT_FQN "/dev/ttyUSB0"


//********************** Local Variables *************************************

//********************** Global extern instances ************************************
    struct termios tty;
    int serial_port; 

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

    int serial_port = open("/dev/ttyUSB0", O_RDWR);
    //if serial port is opened, continue

    // Set up serial port parameters
    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
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
        return -1;
    }



    // FIXME: CANNOT OPEN BOTH RDONLY AND WRONLY IN SAME PROCESS
    pthread_t input_tid = start_serial_handler_thread(&serial_port);
    // Wait for signal

#if 0
    strcpy (output_buffer, "atsp0\n");
    write (serial_port, output_buffer,strlen(output_buffer));
    num_chars = read_elm327(serial_port, input_buffer, 256);
    fwrite (input_buffer, 1, num_chars, obd_file_out);


    // Send setup messages
    printf ("%s", input_buffer);

    fputs ("atl1\n", serial_stream); fflush(serial_stream);
    getline (&input_buffer, &input_buffer_size, serial_stream);
    fwrite (obd_file_out, input_buffer, 
    printf ("%s", input_buffer);

    fputs ("atal\n", serial_stream); fflush(serial_stream);
    getline (&input_buffer, &input_buffer_size, serial_stream);
    printf ("%s", input_buffer);

    fputs ("atma\n", serial_stream); fflush(serial_stream);
#endif
    
    char *input_buffer = malloc(256);
    size_t size_of_buffer = 256;
    int num_read;

    while (num_read = getline (&input_buffer, &size_of_buffer, stdin))
    {
        if (strstr("quit", input_buffer))
        {
            stop_serial_handler_thd(input_tid);
            close (serial_port);
            exit(-1);
        }

//        write (stdin, input_buffer, strlen(input_buffer));
        write (serial_port, input_buffer, strlen(input_buffer));
    }

    return 0;
}  //End Main

