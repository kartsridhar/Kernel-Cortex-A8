/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

//------------------------FROM lab-3_q-------------------------------------

#define N 10      // max number of processes

pcb_t pcb[ N ]; 
pcb_t* current = NULL;
int count = 0;     // number of processes existing
int next;          // to store the index of the next available space

int getNextAvailableSpace( ) {
    for ( int i = 0; i < count; i++ ) {
        if ( pcb[ i ].isAvailable ) {
            return i;
        }
    }
    return -1;
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

	memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise 0-th PCB = P_3
	pcb[ 0 ].pid      = 0;
	pcb[ 0 ].status   = STATUS_CREATED;
	pcb[ 0 ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
	pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
	pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console );
    pcb[ 0 ].isAvailable = false;
	pcb[ 0 ].priority = 10;
	pcb[ 0 ].changed_priority = 5;
	pcb[ 0 ].incPriority = 3;

    count += 1;
    
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
        schedule( ctx );   // Switch context between process control blocks
        TIMER0->Timer1IntClr = 0x01;
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
		
// fork():
// 1  - create new child process with unique PID,
// 2  - replicate state (e.g., address space) of parent in child,
// 3  - parent and child both return from fork, and continue to execute after the call point,
// 4  - return value is 0 for child, and PID of child for parent.
            
		case 0x03 : { // 0x03 => fork()
            count += 1;
            next = getNextAvailableSpace();
            memset( &pcb[ next ], 0, sizeof( pcb_t ) );
            pcb[ next ].pid      = next + 1;
            pcb[ next ].status   = STATUS_CREATED;
            pcb[ next ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
            pcb[ next ].ctx.pc   = ( uint32_t )( &main_console );
            pcb[ next ].ctx.sp   = ( uint32_t )( &tos_console + ( next + 1 ) * 0x00001000 );
           
            ctx->gpr[ 0 ] = 0;

            break;
		}
			
		case 0x04 : { // 0x04 => exit()
			break;
		}
		
		case 0x05 : { // 0x05 => exec()
            memcpy( ctx, &pcb[ next ], sizeof( ctx_t ) );
            ctx->pc = ctx->gpr[ 0 ];
            dispatch( ctx, current, &pcb[ next ] );
			break;
		}

        default   : { // 0x?? => unknown/unsupported
			break;
        }
    }
    return;
}

