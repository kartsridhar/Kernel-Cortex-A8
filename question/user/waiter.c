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
#define PIPES ( PHILS * 2 )

extern void main_philosopher();

int curr = 3;

// Getting the PID of the current philosopher
int find_philosopher_pid() {
    int pid = curr;
    curr++;
    return pid;
}

// Single philosopher eating solution.
void main_waiter() {
    // init the pipes array and philosopher PIDs array
    int pipes[ PIPES ];
    int pids[ PHILS ];

    // Calling fork for each philosopher
    for ( int i = 0; i < PHILS; i++ ) {
        pids[ i ] = fork();   // forking a child process for each philosopher
        if ( 0 == pids[ i ] ) {
            exec( &main_philosopher );
            break;
        }
    }
    exit( EXIT_SUCCESS );
}
