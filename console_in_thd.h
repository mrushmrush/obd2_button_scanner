//****************************************************************************
//  console_handler_thd.h
//
//  Description:  Low priority thread to pulse heart beat led
//

#ifndef  __CONSOLE_HANDLER_THREAD_H__
#define  __CONSOLE_HANDLER_THREAD_H__

#include <pthread.h>

//****************** Public Function Prototypes *******************************
pthread_t start_console_handler_thread (void *arg);
int stop_console_handler_thd(pthread_t tid);

#endif // __CONSOLE_HANDLER_THREAD_H__
