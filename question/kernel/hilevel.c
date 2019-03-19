/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

//------------------------FROM lab-3_q-------------------------------------

#define N 3      // number of processes

pcb_t pcb[ N ]; 
pcb_t* current = NULL;
size_t count = 3;

// Function to create a new process identifier
pid_t newPid() {
    for ( pid_t p = 1; p < N; p++ ) {
        bool check = false;       // to check if pid exists
        for ( size_t i = 0; i < count; i++ ) {
            if ( p == pcb[ i ].pid ) {
                check = true;     // exists.
                break;
            }
        }
        if ( check == false )
            return p;
    }
    return -1;
}

// Function to create a new process control block 
pcb_t* newProcess( uint32_t sp, uint32_t pc ) {
    count += 1;
    pcb_t *newProcess = &pcb[ count ];
    memset( newProcess, 0, sizeof( pcb_t ) );
    newProcess->pid = newPid();
    newProcess->status = STATUS_CREATED;
    newProcess->ctx.cpsr = 0x50;
    newProcess->ctx.pc = pc;
    newProcess->ctx.sp = sp;
    newProcess->priority = 0;
    
    return newProcess;
}

// Function to choose the next availaible process 
int findMaxPriority() {
    int maxPriority = 0;              // set to the highest priority of the process 
    int temp;
    current->changed_priority = current->priority;
    for ( size_t i = 0; i < count; i++ ) {       // iterating through processors to find process with highest priority
        if ( pcb[ i ].pid != current->pid ) {
            pcb[ i ].changed_priority += pcb[ i ].incPriority;                 // incrementing every process by 1 and resetting the age of the current process to 0
            int priority = pcb[ i ].changed_priority + pcb[ i ].priority;      // priority =  changed priority + old priority

            if ( maxPriority < priority ) {
                maxPriority = priority;
                temp = i;
            }
        }
    }
    return temp;
}

// initially 
void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    current = next;                             // update   executing index   to P_{next}

  return;
}

/* Scheduler function (algorithm). Currently special purpose for 3 user
programs. It checks which process is active, and performs a context
switch to suspend it and resume the next one in a simple round-robin
scheduling. 'Memcpy' is used to copy the associated execution contexts
into place, before updating 'current' to refelct new active PCB.
*/

void schedule( ctx_t* ctx ) {
            
    int maxP = findMaxPriority();

    dispatch( ctx, current, &pcb[ maxP ]);
    current->status = STATUS_READY;
    pcb[ maxP ].status = STATUS_EXECUTING;
       
    return;
}

extern void     main_P3();
extern uint32_t tos_P3;
extern void     main_P4();
extern uint32_t tos_P4;
extern void     main_P5();                   
extern uint32_t tos_P5;
extern void     main_console();
extern uint32_t tos_console;

//-------------------------------------------------------------------------------

void hilevel_handler_rst( ctx_t* ctx ) {
    
//       PL011_putc( UART0, 'R', true ); 
//       Initialise 2 PCBs = User Processes (from lab-3_q)
      memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_3
      pcb[ 0 ].pid      = 3;
      pcb[ 0 ].status   = STATUS_CREATED;
      pcb[ 0 ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
      pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_P3 );
      pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_P3  );
      pcb[ 0 ].priority = 5;
      pcb[ 0 ].changed_priority = 5;
      pcb[ 0 ].incPriority = 3;

      memset( &pcb[ 1 ], 0, sizeof( pcb_t ) );     // initialise 1-st PCB = P_4
      pcb[ 1 ].pid      = 4;
      pcb[ 1 ].status   = STATUS_CREATED;
      pcb[ 1 ].ctx.cpsr = 0x50;
      pcb[ 1 ].ctx.pc   = ( uint32_t )( &main_P4 );
      pcb[ 1 ].ctx.sp   = ( uint32_t )( &tos_P4  );
      pcb[ 1 ].priority = 6;
      pcb[ 1 ].changed_priority = 6;
      pcb[ 1 ].incPriority = 4;
    
      memset( &pcb[ 2 ], 0, sizeof( pcb_t ) );     // initialise 2-nd PCB = P_5
      pcb[ 2 ].pid      = 5;
      pcb[ 2 ].status   = STATUS_CREATED;
      pcb[ 2 ].ctx.cpsr = 0x50;
      pcb[ 2 ].ctx.pc   = ( uint32_t )( &main_P5 );
      pcb[ 2 ].ctx.sp   = ( uint32_t )( &tos_P5  );
      pcb[ 2 ].priority = 7;
      pcb[ 2 ].changed_priority = 7;
      pcb[ 2 ].incPriority = 5;
    
//       pcb_t *console = newProcess( main_console, tos_console );
//       dispatch( ctx, current, &console->ctx );
      dispatch( ctx, current, &pcb[ findMaxPriority() ] );
      
      // Configuring the timer interrupt (from lab-4_q)
      TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
      TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
      TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
      TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
      TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

      GICC0->PMR          = 0x000000F0; // unmask all            interrupts
      GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
      GICC0->CTLR         = 0x00000001; // enable GIC interface
      GICD0->CTLR         = 0x00000001; // enable GIC distributor
      
      // Enabling the IRQ interrupt
      int_enable_irq();
      
      return;
}

// From lab-4_q
void hilevel_handler_irq( ctx_t* ctx ) {
    // Step 2: read  the interrupt identifier so we know the source.

    uint32_t id = GICC0->IAR;

    // Step 4: handle the interrupt, then clear (or reset) the source.

    if( id == GIC_SOURCE_TIMER0 ) {
//    PL011_putc( UART0, 'T', true ); 
      TIMER0->Timer1IntClr = 0x01;
      schedule( ctx );   // Switch context between process control blocks
    }

    // Step 5: write the interrupt identifier to signal we're done.

    GICC0->EOIR = id;
    return;
}

// From lab-3_q
void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
   /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */
    switch( id ) {
        case 0x00 : { // 0x00 => yield()
          schedule( ctx );
          break;
        }

        case 0x01 : { // 0x01 => write( fd, x, n )
          int   fd = ( int   )( ctx->gpr[ 0 ] );
          char*  x = ( char* )( ctx->gpr[ 1 ] );
          int    n = ( int   )( ctx->gpr[ 2 ] );

          for( int i = 0; i < n; i++ ) {
            PL011_putc( UART0, *x++, true );
          }

          ctx->gpr[ 0 ] = n;

          break;
        }

        default   : { // 0x?? => unknown/unsupported
          break;
        }
    }
    return;
}

