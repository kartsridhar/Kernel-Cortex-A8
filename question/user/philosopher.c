#include "philosopher.h"

#define PHILS 16

// action => defining the action the philosopher must perform

void main_philosopher() {
    int action = 0;
    while( 1 ) {
        action = readPipe( 2 );   
        
        char* str = "  ";
        itoa(str, action);
        
        if(action == 2) write( STDOUT_FILENO, " THINKING ", 10 );
        if(action == 1) write( STDOUT_FILENO, " EATING ", 8 );
        yield();
    }
    exit(EXIT_SUCCESS);
}

