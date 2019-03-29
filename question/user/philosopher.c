// #include "philosopher.h"

// #define PHILS 16

// int action; // defining the action the philosopher must perform

// // 1. Philopher waiting for his turn
// void waitingPhilosopher() {
//     action = 0;       // wait 
//     while ( 1 ) {
//         action = readPipe( 2 )     // if you receive from the sender
//         char* str = " ";
//         itoa( action, str );       // converting int to ASCII
        
//         if ( action == 1 )
//             write( STDOUT_FILENO, " EATING ", 8 );
//         if ( action == 2 )
//             write( STDOUT_FILENO, " THINKING ", 10 );
//         yield();
//     }
//     exit( EXIT_SUCCESS );
// }

// // 2. Philosopher who wants to use the fork, so asks the person next 
// void askingPhilosopher() {
//     int pid, pipeID;
//     if ( pid == PHILS + 2 ) {
//         pipeID = pipe(3);       // choosing the end of the first pipe if we 
//                                 // receive the pid of the last pipe
//     }
//     else pipeID = pipe( pid + 1 );
// }
