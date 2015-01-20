//
//  mapgen.h
//  
//
//  Created by Lukas Labhart on 7/15/13.
//
//

#ifndef _mapgen_h
#define _mapgen_h

#include <math.h>
#include <stdio.h>
//#include "lookup.h"

# define N_DISC 18
# define N_OBS 20
# define N_POINTS (N_DISC * N_OBS + 2)
# define PI 3.1415
# define INF 2147483647
# define SAFETY_MARGIN 20
# define START_P 0
# define END_P 1
# define VISIBILITY 300 
# define START_X 500
# define START_Y 0
# define ENDX 500
# define ENDY 1000
# define square(a) (a)*(a)
# define RMIN 64
//# define INT

int visible_objects;


//Variables for A* Algorithm
struct map
{
#ifdef INT
    int x[N_POINTS];
    int y[N_POINTS];
    int is_in_bin[N_POINTS];
    int parent[N_POINTS];
    int dI[N_POINTS];
    int h[N_POINTS];
    int delta[N_POINTS][N_POINTS];
#else
    float x[2*N_POINTS];
    float y[2*N_POINTS];
    int is_in_bin[2*N_POINTS];
    int parent[2*N_POINTS];
    float dI[2*N_POINTS];
    float h[2*N_POINTS];
    float delta[2*N_POINTS][2*N_POINTS];
    int alpha[2*N_POINTS][2*N_POINTS];
    int alphastart;
#endif
};

//Struct to save obstacles
struct obstacles
{
    int x[N_OBS];
    int y[N_OBS];
    int r[N_OBS];
    int visible[N_OBS];
};

struct map myMap;

struct obstacles myObstacles;

void MAPGEN_init();

void MAPGEN_visibility(struct map* mapv, float cLocx, float cLocy, int calpha);

int MAPGEN_collision(float* xs, float* ys, float* xe, float* ye);

void MAPGEN_clear();


#endif
