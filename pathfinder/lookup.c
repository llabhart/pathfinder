//
//  lookup.c
//  pathfinder
//
//  Created by Lukas Labhart on 7/29/13.
//  Copyright (c) 2013 Lukas Labhart. All rights reserved.
//

#include "lookup.h"
#include "math.h"

void LOOKUP_init()
{
    //Initialize arctan lookuptable
    /*for (short i=0; i <1000; i++)
    {
        for (short j = 0; j < 1000; j++)
        {
            arctangens[i][j] = (int)(180.*atan((float)(j)/(float)(i))/PI);
        }
    }*/
    int i;
    for (i=0; i<1000000; i++)
    {
        arctangens[i] = (int)(180.*atan((float)(i%1000)/(float)((int)(i/1000.)))/PI);
        //printf("x: %d, y: %d, atan: %d\n", (int)(i/1000.), i%1000, arctangens[i]);
    }
    //Initialize tan lookup
    for (short i=0; i<90; i++)
    {
        tangens[i] = tan((float)(i)*PI/180.);
    }
#if N_DISC == 6
#elif N_DISC == 12
#elif N_DISC == 24
//#elif N_DISC == 18
#else
    for (short i = 0; i<N_DISC; i ++)
    {
        cos_l[i] = cos(2*PI/(float)(N_DISC)*i);
        sin_l[i] = sin(2*PI/(float)(N_DISC)*i);
        //printf("%f, %f, %f\n", cos_l[i], sin_l[i], 360/(float)(N_DISC));
    }
    cos_l[N_DISC] = cos(PI/(float)(N_DISC));
#endif
}

int arctan(int x, int y)
{
    if (x == 0)
        return (y > 0) ? 90 : 270;
    if (x > 0)
    {
        //return (y >= 0) ? arctangens[x][y] : (-arctangens[x][-y]+360);
        return (y >= 0) ? arctangens[x*1000+y] : (-arctangens[x*1000-y]+360);
    }
    if (x < 0)
    {
        //return (y >= 0) ? (-arctangens[-x][y]+180) : arctangens[-x][-y]+180;
        return (y >= 0) ? (-arctangens[-x*1000+y]+180) : arctangens[-x*1000-y]+180;
    }
    return INF;
}

float tang(int x)
{
    return (x >= 0) ? tangens[x] : -tangens[-x];
}
