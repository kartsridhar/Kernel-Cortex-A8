#include "philosopher.h"

#define PHILS 16

extern int find_philo_pid();
// action => defining the action the philosopher must perform

// Philosopher who wants to use the fork, so asks the person next
void main_philosopher() {
    int pipeID;

    int pid = find_philo_pid();

    if ( pid == PHILS + 2 ) {
        pipeID = pipe( 3 );       // choosing the end of the first pipe if we
                                  // receive the pid of the last pipe
    }
    else pipeID = pipe( pid + 1 );

    int action = 2;

    if ( pid == 3 || pid == 9 ) {
        action = 1;
    }

    while ( 1 ) {
        if ( action == 1 ) { // if philosopher is eating now, then make him think
            write( STDOUT_FILENO, " THINKING ", 10 );

            writePipe( pipeID, 1 );
            action = 2;     // making him think

            if ( pipeID == PHILS - 1) // checking last process
               write( STDOUT_FILENO, "\n", 2 );
            yield();
        }
        else {
            if ( pid == 3 )
                action = readPipe( PHILS + 2 );
            else
                action = readPipe( pid - 1 );

            if ( action == 1) { // if philo can eat, make him eat
                write( STDOUT_FILENO, " EATING ", 8 );
                writePipe( pipeID, 2 );
            }
            else { // make him think otherwise
                write( STDOUT_FILENO, " THINKING ", 10 );
                writePipe( pipeID, 2 );
            }
            if ( pipeID == PHILS - 1 )
                write( STDOUT_FILENO, "\n", 2 );
            yield();
        }
    }
    exit( EXIT_SUCCESS );
}
