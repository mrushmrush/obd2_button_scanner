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


//********************** Local Typedefs **************************************
#define SERIAL_PORT_FQN "/dev/ttyUSB0"


//********************** Local Variables *************************************

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



#if 0
    strcpy (output_buffer, "atsp0\n");
    write (serial_port, output_buffer,strlen(output_buffer));
    num_chars = read_elm327(serial_port, input_buffer, 256);
    fwrite (input_buffer, 1, num_chars, obd_file_out);


    // Send setup messages
    printf ("%s", input_buffer);

    fputs ("atl1\n", serial_port_fp); fflush(serial_port_fp);
    getline (&input_buffer, &input_buffer_size, serial_port_fp);
    fwrite (obd_file_out, input_buffer, 
    printf ("%s", input_buffer);

    fputs ("atal\n", serial_port_fp); fflush(serial_port_fp);
    getline (&input_buffer, &input_buffer_size, serial_port_fp);
    printf ("%s", input_buffer);

    fputs ("atma\n", serial_port_fp); fflush(serial_port_fp);
#endif
    
    serial_send (obd2_port, "atl0\n", 5); //linefeeds off
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?
    // add '>' as delimiter?????

    serial_send (obd2_port, "ath0\n", 5); //headers off
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    serial_send (obd2_port, "ats1\n", 5); //print spaces   <-- //TODO: Look into removing spaces to speed up
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    serial_send (obd2_port, "atal\n", 5); // allow long messages
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    serial_send (obd2_port, "ate0\n", 5); //echo off
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    serial_send (obd2_port, "atsp0\n", 6); //automatic protocol search
//    sleep(3);
    //insert wait for correct response.  Add trigger to continue?

    //TODO: use to mask responses
   // fputs ("atar\n", serial_port_fp); fflush(serial_port_fp); //



    //// SET UP STDIN FOR OUR USE
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    newt.c_cc[VTIME] = 1;
    newt.c_cc[VMIN] = 0;
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    //USE fctl to set NDELAY?  Check delay value first?https://www.cmrr.umn.edu/~strupp/serial.html#CONTENTS

    char c;

    for(;;)
    {
        // if available, get char from stdin
        //     send char to obd2_port
        if ((c=getchar()) > 0)
        {
            putchar(c);
            serial_put(obd2_port, c);
        }
printf (".");
        // if available, get char from obd2_port
        //     print to console (adding lf?)
        while (serial_available(obd2_port))
        {
            printf ("%c", serial_get(obd2_port));
        }
    }
    
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

    return 0;
}  //End Main
