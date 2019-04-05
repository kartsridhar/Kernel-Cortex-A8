#include "philosopher.h"

#define THINK 0
#define EAT   1

void main_philosopher() {
    int currPID = getProcessID();
    int philID = currPID - 2;          // console = 0, waiter = 1

    int action;                        // to read the action from the pipe
    while ( 1 ) {
        action = readPipe( philID );   // get philID

        write(STDOUT_FILENO, " PHILOSOPHER ", 13 );

        char* str = "  ";
        itoa( str, philID + 1 );

        write(STDOUT_FILENO, str, 2);  // writing out the philosopherID

        if ( action == EAT ) {              // eat == 1 (EAT)
            write( STDOUT_FILENO, " IS EATING \n", 13 );
        }
        else {
            write( STDOUT_FILENO, " IS THINKING \n", 15 );
        }
    }
    exit( EXIT_SUCCESS );
}
