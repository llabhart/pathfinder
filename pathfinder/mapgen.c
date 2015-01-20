//
//  mapgen.c
//  
//
//  Created by Lukas Labhart on 7/15/13.
//
//

#include "mapgen.h"
#include "lookup.h"



void MAPGEN_init()
{
    //Hard coded map
    memmove(myObstacles.x, (int[]){ 370, 490, 610, 730, 850, 970,   8, 128, 248, 368, 630, 750, 870, 990, 370, 490, 610, 730, 850, 970}, sizeof(myObstacles.x));
    memmove(myObstacles.y, (int[]){ 352, 352, 352, 352, 352, 352, 500, 500, 500, 500, 500, 500, 500, 500, 646, 646, 646, 646, 646, 646}, sizeof(myObstacles.y));
    memmove(myObstacles.r, (int[]){ 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45  }, sizeof(myObstacles.r));
    
    for (short i=0; i<N_OBS;i++)
    {
        myObstacles.r[i]+= SAFETY_MARGIN;
        //myObstacles.x[i]-=500;
        //myObstacles.y[i]-=500;
        myObstacles.visible[i] = 0;
    }  
    
    //printf("Point 43 to 67: %f\n", myMap.delta[67][43]);
    
    //for (int i=0; i<N_POINTS; i++)
    //    printf("%d: x=%f, y=%f\n", i, myMap.x[i], myMap.y[i]);
}

int MAPGEN_collision(float* xs, float* ys, float* xe, float* ye)
{
    float lambda;
    float gamma;
    
    //Some reused variables
    float dx = *xe-*xs;
    float dy = *ye-*ys;
    float det = (dx*dx+dy*dy);
    
    //Scan through all the obstacles
    for (short i=0; i<N_OBS; i++)
    {
        
        //Only check the ones visible for the vehicle
        if (myObstacles.visible[i] > 0)
        {
            //Geometric check whether the shortest distance is smaller than the radius of the observed obstacle
            // return 1 on collision, else 0
            lambda = dx*(myObstacles.x[i]-*xs)+dy*(myObstacles.y[i]-*ys);
            //gamma = 1000000/(float)(det)*((float)(dx)/1000*(float)(myObstacles.y[i]-myMap.y[start])/1000-(float)(dy)/1000*(float)(myObstacles.x[i]-myMap.x[start])/1000);
            gamma = ((float)(dx)*((float)(myObstacles.y[i])-(float)(*ys))-(float)(dy)*((float)(myObstacles.x[i])-(float)(*xs)))/(float)(det);
            //printf("obstacle: %d, %d, %e, det: %d\n", i, lambda, gamma*gamma*det, (myObstacles.r[i]*myObstacles.r[i]));
            if (lambda <=det && lambda >=0 && gamma*gamma*(float)(det)<=(float)(myObstacles.r[i]*myObstacles.r[i]))
            {
                
                //printf("collision, lambda=%d, gamma=%d\n", start, end);
                return 1;
            }
        }
        
    }
    //printf("no collision\n");
    return 0;
}

void MAPGEN_visibility(struct map* mapv, float cLocx, float cLocy, int calpha)
{
    //Calculate new state variables
    for (int i=0; i<N_POINTS*2; i++)
    {
        mapv->x[i] = 0;
        mapv->y[i] = 0;
        mapv->is_in_bin[i] = 0;
        mapv->parent[i] = 0;
        mapv->dI[i] = 0;
        mapv->h[i] = 0;
    }
    
    mapv->x[END_P] = ENDX;
    mapv->y[END_P] = ENDY;
    mapv->is_in_bin[END_P] = 0;
    mapv->parent[END_P] = -1;
    mapv->dI[END_P] = INF;
    mapv->h[END_P] = 0;
    
#ifdef INT
    myMap.x[START_P] = (int)cLocx;
    myMap.y[START_P] = (int)cLocy;
    myMap.h[START_P] = (int)(sqrt(square(myMap.x[END_P]-myMap.x[START_P])+square(myMap.y[END_P]-myMap.y[START_P])));
#else
    mapv->x[START_P] = cLocx;
    mapv->y[START_P] = cLocy;
    mapv->h[START_P] = sqrt(square(mapv->x[END_P]-mapv->x[START_P])+square(mapv->y[END_P]-mapv->y[START_P]));
#endif
    mapv->is_in_bin[START_P] = 1;
    mapv->parent[START_P] = START_P;
    mapv->dI[START_P] = 0;
    mapv->alpha[START_P][START_P] = calpha;
    mapv->delta[START_P][START_P] = 10000;
    mapv->alphastart = calpha;
    visible_objects = 0;
    
    for (short i = 0; i<N_OBS;i++)
    {
        if (square(cLocx-myObstacles.x[i])+square(cLocy-myObstacles.y[i]) < VISIBILITY*VISIBILITY)
        {
            myObstacles.visible[i] = (myObstacles.r[i] >= RMIN) ? 1 : 2;
            visible_objects += myObstacles.visible[i];
        }
        else
            myObstacles.visible[i] = 0;
    }
    
    short c;
    short g = 0;
    for (short i = 0; i<N_OBS; i++)
    {
        for (short p = 0; p < myObstacles.visible[i]; p++)
        {
            //Calculate the outlying radius of the polyhedra surrounding obstacle i
            float r_tilde = (p < 1) ? (myObstacles.r[i])*cos_l[N_DISC]+10 : (RMIN)*cos_l[N_DISC]+5;
            
            //printf("%d, %d\n", myObstacles.x[i], myObstacles.y[i]);
            //Discretize obstacle i with a polyhedra
            for (short j=0;j<N_DISC;j++)
            {
                c = g*N_DISC+j+2;
                //printf("%d\n", c);
                mapv->x[c] = (cos_l[j] > 0 ? (myObstacles.x[i] + (int)(r_tilde*cos_l[j])+1) : (myObstacles.x[i] + (int)(r_tilde*cos_l[j])));
                mapv->y[c] = (sin_l[j] > 0 ? (myObstacles.y[i] + (int)(r_tilde*sin_l[j])+1) : (myObstacles.y[i] + (int)(r_tilde*sin_l[j])));
                                
                mapv->is_in_bin[c] = 0;
                mapv->parent[c] = -1;
                mapv->dI[c] = INF;
#ifdef INT
                mapv->h[c] = (int)(sqrt(square(mapv->x[END_P]-mapv->x[c])+square(mapv->y[END_P]-mapv->y[c])));
                //printf("x of point %d: %d\n", c, mapv->h[c]);
                if(c != 0)
                {
                    for (short k=0;k<c;k++)
                    {
                        //On collision, set the distance to infinity
                        mapv->delta[c][k] = (MAPGEN_collision(c,k) == 0 ? (int)(sqrt(square(mapv->x[c]-mapv->x[k])+square(mapv->y[c]-mapv->y[k]))) : INF);
                        //printf("x of point %d: %d\n", c, mapv->delta[c][k]);
                    }
                }
#else
                mapv->h[c] = sqrt(square(mapv->x[END_P]-mapv->x[c])+square(mapv->y[END_P]-mapv->y[c]));
                //printf("x of point %d: %f\n", c, mapv->h[c]);
                if(c != 0)
                {
                    for (short k=0;k<c;k++)
                    {
                        //On collision, set the distance to infinity
                        //mapv->delta[c][k] = (MAPGEN_collision(c,k) == 0 ? sqrt(square(mapv->x[c]-mapv->x[k])+square(mapv->y[c]-mapv->y[k])) : INF);
                        /*if (MAPGEN_collision(c,k) == 0)
                        {
                            mapv->delta[c][k] = sqrt(square(mapv->x[c]-mapv->x[k])+square(mapv->y[c]-mapv->y[k]));
                            mapv->alpha[c][k] = arctan(mapv->x[c]-mapv->x[k], mapv->y[c]-mapv->y[k]);
                            //printf("alpha between point %d and %d is: %d\n", c, k, mapv->alpha[c][k]);
                        }
                        else
                            mapv->delta[c][k] = INF;*/
                        //printf("x of point %d: %f\n", c, mapv->delta[c][k]);
                    }
                }
#endif
            }
            g++;
        }
    }
    
 /*   //Check collision also for the start and end node
#ifdef INT
    mapv->delta[END_P][START_P] = (MAPGEN_collision(END_P,START_P) == 0 ? (int)(sqrt(square(mapv->x[END_P]-mapv->x[START_P])+square(mapv->y[END_P]-mapv->y[START_P]))) : INF);
#else
    mapv->delta[END_P][START_P] = (MAPGEN_collision(END_P,START_P) == 0 ? sqrt(square(mapv->x[END_P]-mapv->x[START_P])+square(mapv->y[END_P]-mapv->y[START_P])) : INF);
    mapv->alpha[END_P][START_P] = arctan(mapv->x[END_P]-mapv->x[START_P], mapv->y[END_P]-myMap.y[START_P]);
#endif*/
    
}

void MAPGEN_clear()
{
    //Clear the discrete map
    memset(myMap.x, 0, sizeof(myMap.x));
    memset(myMap.y, 0, sizeof(myMap.y));
    memset(myMap.is_in_bin, 0, sizeof(myMap.is_in_bin));
    memset(myMap.parent, 0, sizeof(myMap.parent));
    memset(myMap.h, 0, sizeof(myMap.h));
    memset(myMap.dI, 0, sizeof(myMap.dI));
    memset(myMap.delta, 0, sizeof(myMap.delta));
}
