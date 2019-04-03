#include "philosopher.h"

void main_philosopher() {
    int currPID = getProcessID();
    int philID = currPID - 3;      // console = 0, waiter = 1
    
    int eat;
    while ( 1 ) {
        eat = readPipe( philID );   // get philID
        
        write(STDOUT_FILENO, " PHILOSOPHER ", 13 );
        
        char* str = "  ";
        itoa( str, philID + 1 );
        
        write(STDOUT_FILENO, str, 2);  // writing out the philosopherID
        
        if ( eat ) {
            write( STDOUT_FILENO, " IS EATING \n", 13 );
        }
        else {
            write( STDOUT_FILENO, " IS THINKING \n", 15 );
        }
    }
    exit( EXIT_SUCCESS );
}


