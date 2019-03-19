/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

//------------------------FROM lab-3_q-------------------------------------

#define N 1      // number of processes

pcb_t pcb[ N ]; 
pcb_t* current = NULL;
size_t count = 1;

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
pcb_t* newProcess( pid_t id, uint32_t sp, uint32_t pc, pid_t priority ) {
    count += 1;
    pcb_t *newProcess = &pcb[ count ];
    memset( newProcess, 0, sizeof( pcb_t ) );
    newProcess->pid = id;
    newProcess->status = STATUS_CREATED;
    newProcess->ctx.cpsr = 0x50;
    newProcess->ctx.pc = pc;
    newProcess->ctx.sp = sp;
    newProcess->priority = priority;
    newProcess->changed_priority = newProcess->priority;          // initialising changed priority to initial priority
    newProcess->incPriority = id * 2;              // initialising priority increment as twice the pid, for simplicity

    return newProcess;
}

// Function to choose the process with highest priority
// Priority = initial priority + the change in priority
// Reset the current process' changed priority to the original priority
// Increase the changed priority of every process by the increment priority value
// returns the index of the process with highest priority  
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

extern void     main_console();
extern uint32_t tos_console;

//-------------------------------------------------------------------------------

void hilevel_handler_rst( ctx_t* ctx ) {
    
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
	
	PL011_putc( UART0, 'R',      true );
    
	memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_3
	pcb[ 0 ].pid      = 3;
	pcb[ 0 ].status   = STATUS_CREATED;
	pcb[ 0 ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
	pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
	pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console );
	pcb[ 0 ].priority = 5;
	pcb[ 0 ].changed_priority = 5;
	pcb[ 0 ].incPriority = 3;
	
	dispatch( ctx, current, &pcb[ 0 ] );
	
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
        case 0x00 : { // 0x00 => yield() = timer forcibly transfer the control to another process. 
			schedule( ctx );
			break;
        }

        case 0x01 : { // 0x01 => write( fd, x, n )
			int   fd = ( int   )( ctx->gpr[ 0 ] );     // file descriptor
			char*  x = ( char* )( ctx->gpr[ 1 ] );
			int    n = ( int   )( ctx->gpr[ 2 ] );

			for( int i = 0; i < n; i++ ) {
				PL011_putc( UART0, *x++, true );
			}

			ctx->gpr[ 0 ] = n;

			break;
        }
		
		case 0x02 : { // 0x02 => read( fd, x, n )
			break;
		}
		
		case 0x03 : { // 0x03 => fork()
			pid_t _id_ = newPid();
			uint32_t _sp_ = ( ( _id_ + 1 ) * 0x00001000 );
			uint32_t _pc_ = ( ( uint32_t )( &main_console ) );
			pcb_t *new = newProcess( _id_, _sp_, _pc_, 5 );
            break;
		}
			
		case 0x04 : { // 0x04 => exit()
			break;
		}
		
		case 0x05 : { // 0x05 => exec()
			
		}

        default   : { // 0x?? => unknown/unsupported
			break;
        }
    }
    return;
}

