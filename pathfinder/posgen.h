//
//  posgen.h
//  pathfinder
//
//  Created by Lukas Labhart on 7/25/13.
//  Copyright (c) 2013 Lukas Labhart. All rights reserved.
//

#ifndef _posgen_h
#define _posgen_h
#include <stdio.h>
#include <math.h>
#include "mapgen.h"
#include "lookup.h"

void POSGEN_init(float startx, float starty, int start_alpha);

void POSGEN_update(float destx, float desty, int alpha_dest);

float POS_x, POS_y;
int alpha;

#endif
