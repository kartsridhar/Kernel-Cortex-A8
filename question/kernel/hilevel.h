/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#ifndef __HILEVEL_H
#define __HILEVEL_H

// Include functionality relating to newlib (the standard C library).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>
// Include functionality relating to the platform.

#include   "GIC.h"
#include "PL011.h"
#include "SP804.h"    //timer interrupt

// Include functionality relating to the   kernel.

#include "lolevel.h"
#include     "int.h"

typedef int pid_t;

typedef enum { 
  STATUS_CREATED,
  STATUS_READY,
  STATUS_EXECUTING,
  STATUS_WAITING,
  STATUS_TERMINATED
} status_t;

typedef struct {
  uint32_t cpsr, pc, gpr[ 13 ], sp, lr;
} ctx_t;

typedef struct {
           pid_t    pid;         // process identifier
        status_t status;
           ctx_t    ctx;
       bool isAvailable;
         pid_t priority;
 pid_t changed_priority;
      pid_t incPriority;        // increase priority by this val
} pcb_t;

typedef struct {
    pid_t pipeID;
    status_t status;
    pid_t send;       // sending end PID
    pid_t rec;         // receiving end PID
    uint32_t data;   //pipe contents
} pipe_t;

#endif


