/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

//------------------------FROM lab-3_q-------------------------------------

#define N 10      // number of processes

pcb_t pcb[ N ]; 
pcb_t* current = NULL;
int count = 0;

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
// initialising changed priority to initial priority
// initialising priority increment as twice the pid, for simplicity
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
    newProcess->changed_priority = newProcess->priority;          
    newProcess->incPriority = id * 2;              

    return newProcess;
}

// Function to choose the process with highest priority
// Priority = initial priority + the change in priority
// Reset the current process' changed priority to the original priority
// Increase the changed priority of every process by the increment priority value
// returns the index of the process with highest priority  
int findMaxPriority() {
    int maxPriority = 0;              
    int temp = 0;
    current->changed_priority = current->priority;
    for ( int i = 0; i < count; i++ ) {       
        if ( pcb[ i ].pid != current->pid  && pcb[i].pid != -1) {
            pcb[ i ].changed_priority += pcb[ i ].incPriority;                 
            int priority = pcb[ i ].changed_priority + pcb[ i ].priority;      

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
scheduling. 'memcpy' is used to copy the associated execution contexts
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
	
	PL011_putc( UART0, 'R', true );
    PL011_putc( UART0, 'E', true );
    PL011_putc( UART0, 'S', true );
    PL011_putc( UART0, 'E', true );
    PL011_putc( UART0, 'T', true );
    
    for(int i = 0; i < N; i++){
        pcb[i].pid = -1;
    }
    
	memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_3
	pcb[ 0 ].pid      = 0;
	pcb[ 0 ].status   = STATUS_CREATED;
	pcb[ 0 ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
	pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
	pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console );
	pcb[ 0 ].priority = 5;
	pcb[ 0 ].changed_priority = 5;
	pcb[ 0 ].incPriority = 3;
//     pcb* console = newProcess( newPid(), ( uint32_t )( &tos_console ), ( uint32_t )( &main_console ), 3 );
	
	dispatch( ctx, NULL, &pcb[ 0 ] );
	
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
    pcb_t* new = NULL;
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
		
// fork():
//  - create new child process with unique PID,
//  - replicate state (e.g., address space) of parent in child,
//  - parent and child both return from fork, and continue to execute after the call point,
//  - return value is 0 for child, and PID of child for parent.
            
		case 0x03 : { // 0x03 => fork()
			pid_t new_id = newPid();
			uint32_t new_sp = (ctx->sp);
			uint32_t new_pc = ctx->pc;
			new = newProcess( new_id, new_sp, new_pc, 5 );
            
            new->ctx.gpr[ 0 ] = 0;     // storing the address of the child
            
			ctx->gpr[ 0 ] = new_id;
			
            break;
		}
			
		case 0x04 : { // 0x04 => exit()
			break;
		}
		
		case 0x05 : { // 0x05 => exec()
            uint32_t address = ( uint32_t ) ( ctx->gpr[ 0 ] );
            dispatch( ctx, current, current );
            current->ctx.pc = ( uint32_t ) ( address );
            
			break;
		}

        default   : { // 0x?? => unknown/unsupported
			break;
        }
    }
    return;
}

