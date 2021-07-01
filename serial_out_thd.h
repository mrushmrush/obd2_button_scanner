//****************************************************************************
//  serial_handler_thd.h
//
//  Description:  Low priority thread to pulse heart beat led
//

#ifndef  __SERIAL_HANDLER_THREAD_H__
#define  __SERIAL_HANDLER_THREAD_H__

#include <pthread.h>

//****************** Public Function Prototypes *******************************
pthread_t start_serial_handler_thread (void *arg);
int stop_serial_handler_thd(pthread_t tid);

#endif // __SERIAL_HANDLER_THREAD_H__
