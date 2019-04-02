/* ARBITRATOR SOLUTION
 * In order to pick up the forks,
 * a philosopher must ask permission of the waiter.
 * The waiter gives permission to only one philosopher at a time
 * until the philosopher has picked up both of their forks.
 * Putting down a fork is always allowed.
 *
 * IMPLEMENTING THE WAITER AS A MUTEX
 *
 * */

#include "waiter.h"

#define PHILS 16

extern void main_philosopher();

/*
 * 1. Init a pipe for every philosopher
 * 2. Allow alternate philosophers to eat initially
 * 3. Wait for waiter to give permission. Reads pipe one at a time
 *    and allows philosopher to eat if forks are available
 */

void main_waiter() {
    int pipes[ PHILS ];   // array for pipes
    int id[ PHILS ];      // array for philosopher PIDs
    
    // 1.
    for( int i = 0; i < PHILS; i++ ) {
        id[ i ] = fork();   // forking a child process for each philosopher
        if ( 0 == id[ i ] ) {
            exec( &main_philosopher );
            break;
        }
        pipes[ i ] = pipe( id[ i ] );
    }
    
    // 2.
    for( int j = 0; j < PHILS; j++ ) {
        if( j%2 == 0 ) 
            writePipe( j, 1 );    // make them eat first
        else 
            writePipe( j, 2 );    // make them think first
    }
    yield();
    
    // 3.
    while( 1 ) {
        for( int k = 0; k < PHILS; k++ ) {
            int x = readPipe( k + 3 );
            if( x == 1 )       // if philo finished eating, make him think
                writePipe( k, 2 );
            if( x == 2 )       // make the philo who was thinking, eat
                writePipe( k, 1 );
        }
        write(STDOUT_FILENO, "\n", 2);
        yield();               // sys call yield
    }
    exit(EXIT_SUCCESS);
}

