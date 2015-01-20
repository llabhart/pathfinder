//
//  posgen.c
//  pathfinder
//
//  Created by Lukas Labhart on 7/25/13.
//  Copyright (c) 2013 Lukas Labhart. All rights reserved.
//

#include "posgen.h"

void POSGEN_init(float startx, float starty, int start_alpha)
{
    POS_x = startx;
    POS_y = starty;
    alpha = start_alpha;
    FILE * file;
    file = fopen("coordinates.txt", "w");
    fclose(file);
}

void POSGEN_update(float destx, float desty, int alpha_dest)
{
    float dx = destx-POS_x;
    float dy = desty-POS_y;
    //printf("Destination: (%f, %f)\n", destx, desty);
    //printf("Old Pos: (%f, %f)\n", POS_x, POS_y);
    
    //Update the position towards the next point in the shortest path algorithm
    POS_x += 50.0*dx/(float)(sqrt(square(dx)+square(dy)));
    POS_y += 50.0*dy/(float)(sqrt(square(dx)+square(dy)));
    alpha = arctan(dx, dy);
    
    //Make sure we don't overshoot
    if(POS_x >= destx && POS_y >= desty)
    {
        POS_x = destx;
        POS_y = desty;
    }
    
    //printf("New Pos: (%f, %f)\n", POS_x, POS_y);
    
    //Write points to file to directly plot via MATLAB
    FILE * file;
    file = fopen("coordinates.txt", "a+");
    fprintf(file, "%f %f; ", POS_x, POS_y);
    fclose(file);
}