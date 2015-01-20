//
//  pathfinder.h
//  
//
//  Created by Lukas Labhart on 7/15/13.
//
//

#ifndef _pathfinder_h
#define _pathfinder_h

#include "mapgen.h"
#include "posgen.h"
#include "cpucycles.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "lookup.h"

#define ALPHA_MAX 60

//void PF_init();

//Calculate the shortest path given the current visibility range
void PF_astar(struct map* mapa);

//Reconstruct the calculated path and update the position
void PF_reconstruct(struct map* mapa);

#endif
