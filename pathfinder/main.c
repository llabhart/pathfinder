//
//  main.c
//  pathfinder
//
//  Created by Lukas Labhart on 7/16/13.
//  Copyright (c) 2013 Lukas Labhart. All rights reserved.
//

#include <stdio.h>
#include "lookup.h"
#include "cpucycles.h"
#include "mapgen.h"
#include "posgen.h"
#include "pathfinder.h"
#define sign(x) (x > 0 ? 1:(x < 0 ? -1:0))
int main()
{
    int x1, x2, x3;
    x1 = sign(10);
    x2 = sign(-10);
    x3 = sign(0);
    
    printf('sgn(10) %i \n sgn(-10) %i \n sgn(0) %i\n', x1, x2, x3);
//{
//    LOOKUP_init();
//    
//    POSGEN_init(START_X, START_Y, 45);
//    
//    
//    MAPGEN_init();
//    unsigned long long int cycle_init = rdtsc();
//    MAPGEN_visibility(&myMap, POS_x, POS_y, alpha);
//    cycle_init = rdtsc()-cycle_init;
//    printf("Initialitaion took %f ms\n", (float)(cycle_init)/50000);
//    
//    //Loop until the destination is reached
//    while (POS_x != ENDX && POS_y != ENDY)
//    {
//        MAPGEN_visibility(&myMap, POS_x,  POS_y, alpha);
//        PF_astar(&myMap);
//        PF_reconstruct(&myMap);
//                                                
//}
//    printf("Finished");
}

