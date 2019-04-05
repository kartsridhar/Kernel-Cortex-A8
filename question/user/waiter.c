/* ARBITRATOR SOLUTION
 * In order to pick up the forks,
 * a philosopher must ask permission of the waiter.
 * The waiter gives permission to only one philosopher at a time
 * until the philosopher has picked up both of their forks.
 *
 * IMPLEMENTING THE WAITER AS A MUTEX
 *
 * */

#include "waiter.h"

#define PHILS 16
#define EAT 1
#define THINK 0

extern void main_philosopher();

/* 1. Init child processes and pipes for every philosopher
 * 2. Choose even philosophers to eat first, then switch with odd
 * 3. increase the count
 * */

int pipeIDS[ PHILS ];
int philIDS[ PHILS ];

void main_waiter() {
    pid_t currPID = getProcessID();     // get current process ID

    // 1.
    for ( int i = 0; i < PHILS; i++ ) {
        philIDS[ i ] = fork();                             // fork a child process for each philosopher
        pipeIDS[ i ] = pipe( currPID, philIDS[ i ] );      // create a pipe from current process
                                                           // to every philID

        if ( 0 == philIDS[ i ] )
            exec( &main_philosopher );
    }

    // 2.
    int count = 0;
    while ( 1 ) {
        for ( int i = 0; i < PHILS; i++ ) {
            if ( count%2 == 0 ) {                         // checking if round is even
                if ( i%2 == 0 ) writePipe( i, EAT );      // making even philosophers eat
                else writePipe( i, THINK );               // making odd philosophers think
            }
            else {                                        // checking if round is odd
                if ( i%2 != 0 ) writePipe( i, EAT );      // making odd philosophers eat
                else writePipe( i, THINK );               // making even philosophers think
            }
        }
        // 3.
        count = ( count + 1 ) % PHILS;
    }
    exit( EXIT_SUCCESS );
}
