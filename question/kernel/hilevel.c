/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

#define PROCESSES 20               // max number of processes
#define SIZE_OF_STACK 0x00001000   // defining the size of stack
#define PIPES 60                   // max number of pipes

pcb_t pcb[ PROCESSES ];
pcb_t* current = NULL;
int noOfPCB = 0;                   // number of processes existing
int availableSpaceIndex;           // to store the index of the next available space

int priorities[ PROCESSES ];       // to store the init priorities of each pcb

pipe_t pipes[ PIPES ];
int noOfPipes = 0;

// Function to print to console, making things easier
void pprint( char* str ) {
    for ( int i = 0; i < strlen( str ); i++ ) {
        PL011_putc( UART0, str[ i ], true );
    }
}

// Function to return the index of the available space
int getAvailableSpace( ) {
    for ( int i = 0; i < noOfPCB; i++ ) {
        if ( pcb[ i ].isAvailable || pcb[ i ].status == STATUS_TERMINATED ) {
            return i;
        }
    }
    return -1;
}

// Function to return the index of the available pipe
int getAvailablePipe() {
    for ( int i = 0; i < PIPES; i++ ) {
        if ( pipes[ i ].status == STATUS_TERMINATED )
            return i;
    }
    return -1;
}

// Function to get a respective pipe by pipe ID
int getPipeIndex( pid_t id ) {
    for ( int i = 0; i < PIPES; i++ ) {
        if ( id == pipes[ i ].pipeID )
            return i;
    }
    return -1;
}

// // Function to assign priority for console
// int assignPriorityConsole() {
//     priorities[ 0 ] = 20;
//     return priorities[ 0 ];
// }
//
// // Function to assign a random priority value to each process
// void assignPriority() {
//     for ( int i = 1; i < noOfPCB; i++ ) {
//         int p = rand() % 18;               // 20 for console, want to keep it even
//         priorities[ i ] = p;
//     }
// }

// Function to choose the process with highest priority
// Priority = initial priority + the change in priority
// Reset the current process' changed priority to the original priority
// Increase the changed priority of every process by the increment priority value
// returns the index of the process with highest priority
int findMaxPriority() {
    int maxPriority = 0;
    int temp = 0;

    current->changed_priority = current->priority;

    for ( int i = 0; i < noOfPCB; i++ ) {

        if ( pcb[ i ].pid != current->pid && pcb[ i ].status != STATUS_TERMINATED ) {

            pcb[ i ].changed_priority += pcb[ i ].incPriority;
            int priority = pcb[ i ].changed_priority + pcb[ i ].priority;

            if ( maxPriority < priority ) {
                maxPriority = priority;
                temp = i;
            }
            // else if ( maxPriority == priority ) {
            //     if ( pcb[ i ].changed_priority > pcb[ availableSpaceIndex ].changed_priority ) {
            //         maxPriority = priority;
            //         temp = i;
            //     }
            // }
        }
    }
    return temp;
}

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

void roundRobinSchedule( ctx_t* ctx ) {

    for ( int i = 0; i < noOfPCB; i++ ) {
        if ( current->pid == pcb[ i ].pid ) {
            int nextProcessIndex = ( i + 1 ) % noOfPCB;
            dispatch( ctx, &pcb[ i ], &pcb[ nextProcessIndex ] );
            pcb[ i ].status = STATUS_READY;
            pcb[ nextProcessIndex ].status = STATUS_EXECUTING;
            return;
        }
    }
    return;
}

void prioritySchedule( ctx_t* ctx ) {

    int maxP = findMaxPriority();

    dispatch( ctx, current, &pcb[ maxP ] );

    current->status = STATUS_READY;
    pcb[ maxP ].status = STATUS_EXECUTING;

    return;
}

extern void main_console();
extern void main_philosopher();
extern void main_waiter();
extern uint32_t tos_console;
extern uint32_t tos_USR;          // for user programs

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

    pprint("RESET");

    // Setting all processes in N to available
    for ( int i = 0; i < PROCESSES; i++ ){
        pcb[ i ].isAvailable = true;
    }

    // Setting all pipes in PIPES to available
    for ( int i = 0; i < PIPES; i++ ) {
        pipes[ i ].status = STATUS_TERMINATED;
    }

	memset( &pcb[ 0 ], 0, sizeof( pcb_t ) );     // initialise the console
	pcb[ 0 ].pid      = 0;
	pcb[ 0 ].status   = STATUS_CREATED;
	pcb[ 0 ].ctx.cpsr = 0x50;                    // processor is switched into USR mode
	pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
	pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_console );
  pcb[ 0 ].isAvailable = false;                // setting the process space to be unavailable
	pcb[ 0 ].priority = 20;
	pcb[ 0 ].changed_priority = 20;
 	// pcb[ 0 ].priority = assignPriorityConsole();
	// pcb[ 0 ].changed_priority = assignPriorityConsole();
	pcb[ 0 ].incPriority = 3;

    noOfPCB += 1;                                // increasing the process count by 1

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

        // Switch context between process control blocks
        prioritySchedule( ctx );
//         roundRobinSchedule( ctx );
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
            prioritySchedule( ctx );
            // roundRobinSchedule( ctx );
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

            pprint("FORK");

            noOfPCB += 1;
            availableSpaceIndex = getAvailableSpace();

            // Setting everything in the stack to 0
            memset( &pcb[ availableSpaceIndex ], 0, sizeof( pcb_t ) );

            // Copying the contents of stack into the child process
            memcpy( &pcb[ availableSpaceIndex ].ctx, ctx , sizeof( ctx_t ) );

            // Creating a new child process with unique available PID
            pcb[ availableSpaceIndex ].pid      = availableSpaceIndex;
            pcb[ availableSpaceIndex ].status   = STATUS_CREATED;
            pcb[ availableSpaceIndex ].isAvailable   = false;
            pcb[ availableSpaceIndex ].priority = ( availableSpaceIndex * 3 );
            pcb[ availableSpaceIndex ].changed_priority = ( availableSpaceIndex * 3 );
            // pcb[ availableSpaceIndex ].priority = priorities[ availableSpaceIndex ];
            // pcb[ availableSpaceIndex ].changed_priority = priorities[ availableSpaceIndex ];
            pcb[ availableSpaceIndex ].incPriority = 2;

            // Storing the stack pointer of the current process
            uint32_t currentStack = ( uint32_t ) ( &tos_USR + ( current->pid ) * SIZE_OF_STACK );

            // Child process' stack pointer
            uint32_t childStack = ( uint32_t ) ( &tos_USR + ( availableSpaceIndex ) * SIZE_OF_STACK );

            // getting the stack needed for copying
            uint32_t offsetStack = currentStack - ( ctx->sp );

            // Setting the stack pointer of the child process to the required
            pcb[ availableSpaceIndex ].ctx.sp = childStack - offsetStack;

            // memcpy copies the stack downwards.
            // (void *) is used to indicate src and dest are pointers
            memcpy( ( void * ) childStack - SIZE_OF_STACK, ( void * ) currentStack - SIZE_OF_STACK, SIZE_OF_STACK );

            ctx->gpr[ 0 ] = availableSpaceIndex;           // returning pid to the parent
            pcb[ availableSpaceIndex ].ctx.gpr[ 0 ] = 0;   // return 0 to the child
            break;
		    }

		    case 0x04 : { // 0x04 => exit()

            pprint("EXIT");

            // Simply terminating the process
            current->status = STATUS_TERMINATED;
			      break;
		    }

        case 0x05 : { // 0x05 => exec()

            pprint("EXEC");

            ctx->pc = ( uint32_t ) ctx->gpr[ 0 ];          // loading the address from fork
            ctx->sp = ( uint32_t ) ( &tos_console + ( ( availableSpaceIndex ) * SIZE_OF_STACK ));
            prioritySchedule( ctx );
		      	break;
		    }

        case 0x06 : { // 0x06 => kill()

            pprint("KILL");

            uint32_t kill = ( uint32_t ) ( ctx->gpr[ 0 ] );
            noOfPCB -= 1;
            pcb[ kill ].isAvailable = true;
            pcb[ kill ].status = STATUS_TERMINATED;

            break;
        }

        case 0x08 : { // 0x08 => pipe( pid_t send, pid_t rec )

//             pprint("CREATING_PIPE");

            noOfPipes += 1;
            int availablePipeIndex = getAvailablePipe();
            // Setting everything in the pipe to 0
            memset( &pipes[ availablePipeIndex ], 0, sizeof( pipe_t ) );

            // Initiliasing a new pipe
            pipes[ availablePipeIndex ].pipeID = availablePipeIndex;
            pipes[ availablePipeIndex ].status = STATUS_READY;
            pipes[ availablePipeIndex ].send = ( pid_t ) ctx->gpr[ 0 ];
            pipes[ availablePipeIndex ].rec = ( pid_t ) ctx->gpr[ 1 ];
            pipes[ availablePipeIndex ].data = -1;

            // Returning the pipeID
            ctx->gpr[ 0 ] = pipes[ availablePipeIndex ].pipeID;
            break;
        }

        case 0x09 : { // 0x09 => writePipe( int pipeID, uint32_t data )

//             pprint("WRITING");

            // Getting the pipe ID from ctx
            pid_t id = ctx->gpr[ 0 ];
            uint32_t data = ctx->gpr[ 1 ];

            // Getting the index of the pipe ID received
            int get = getPipeIndex( id );

            pipes[ get ].data = data;

            break;
        }

        case 0x10 : { // 0x10 => readPipe( int pipeID )

//             pprint("READING");

            // Getting the pipe ID from ctx
            pid_t id = ctx->gpr[ 0 ];
            ctx->gpr[ 0 ] = -1;  //resetting

            int get = getPipeIndex( id );
            ctx->gpr[ 0 ] = pipes[ get ].data;

            pipes[ get ].data = -1;

            break;
        }

        case 0x11 : { // 0x11 => closePipe( int pipeID )

//             pprint("CLOSING");

            // Getting the pipe ID from ctx
            pid_t id = ctx->gpr[ 0 ];

            int get = getPipeIndex( id );

            pipes[ get ].status = STATUS_TERMINATED;
            noOfPipes -= 1;
            break;
        }

        case 0x12 : { //0x12 => getProcessID()
            ctx->gpr[ 0 ] = current->pid;
            break;
        }

        default   : { // 0x?? => unknown/unsupported
			      break;
        }
    }
    return;
}
