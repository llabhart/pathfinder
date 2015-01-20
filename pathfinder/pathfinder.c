//
//  pathfinder.c
//  
//
//  Created by Lukas Labhart on 7/15/13.
//
//

#include "pathfinder.h"

void PF_astar(struct map* mapa)
{
    const int n_points = visible_objects*N_DISC+2;     //Number of visible obstacles*number of discrete points per obstacle
    //printf("Number of points: %d\n", visible_objects);
    
    //Some storage variables for the A* algorithm
    int bin[n_points];
    bin[0] = START_P;
    int bin_elements = 1;
    int removed;
#ifdef INT
    int delta;
    int* delta_check;
#else
    float delta;
    float delta_check;
    float delta_prev;
#endif
    int alpha_k, alpha_k1, r_current;
    unsigned long long cycle_start = rdtsc();
    //clock_t t = clock();
    
    //Scan through all points until no possible option is available
    while (bin_elements!=0)
    {
        bin_elements --;
        //Remove an element from the bin which is to be observed
        removed = bin[bin_elements];
        //printf("removed: %d\n", removed);
        
        //Flag this element as no longer in the bin
        mapa->is_in_bin[removed] = 0;
        
        //Iterate through all points to find the shortest distance between the removed and the observed point
        for (short i=0;i<n_points;i++)
        {
            //No need to observe the removed point
            if (i != removed)
            {
                //Get precalculated distance
                delta_check = MAPGEN_collision(&mapa->x[removed], &mapa->y[removed], &mapa->x[i], &mapa->y[i]) == 0 ? sqrt(square(mapa->x[i]-mapa->x[removed])+square(mapa->y[i]-mapa->y[removed])) : (float)(INF);
                //printf("%d, %d, %f\n", removed, i, *delta_check);
                //printf("%f\n", mapa->dI[END_P]);
#ifdef INT
                if (delta_check != INF)
#else
                if (delta_check != (float)(INF))
#endif
                {
                    //Update distance
                    delta = mapa->dI[removed]+delta_check;
                    
                    //printf("Angle between point %d and %d is: %d\n", i, removed, alpha_k1-alpha_k);
                    //Compare whether the computed distance is shorter or not
                    if ((delta+mapa->h[i])<(mapa->dI[i]+mapa->h[i]) && (delta+mapa->h[i])< mapa->dI[END_P])
                    {
                        alpha_k =  (removed != myMap.parent[removed]) ? arctan(myMap.x[removed]-myMap.x[myMap.parent[removed]], myMap.y[removed]-myMap.y[myMap.parent[removed]]) : myMap.alphastart;
                        //alpha_k = (removed >= mapa->parent[removed]) ? mapa->alpha[removed][mapa->parent[removed]] : (mapa->alpha[removed][mapa->parent[removed]] < 180 ? mapa->alpha[removed][mapa->parent[removed]]+180 : mapa->alpha[removed][mapa->parent[removed]]-180);
                        alpha_k1 = arctan(myMap.x[i]-myMap.x[removed], myMap.y[i]-myMap.y[removed]);
                        //alpha_k1 = (i >= removed) ? mapa->alpha[i][removed] : (mapa->alpha[removed][i] < 180 ? mapa->alpha[removed][i] + 180 : mapa->alpha[removed][i] - 180);
                        //printf("%d\n", alpha_k1-alpha_k);
                        if ((alpha_k1-alpha_k) <= ALPHA_MAX && (alpha_k1-alpha_k) >= -ALPHA_MAX)
                        {
                        //printf("%d: delta=%d\n ", i, mapa->dI[i]);
                        //printf("%d\n", i);
                            delta_prev = (removed != myMap.parent[removed]) ? sqrtf(square(myMap.x[removed]-myMap.x[myMap.parent[removed]])+square(myMap.y[removed]-myMap.y[myMap.parent[removed]])) : 1000.;

                            float tang1 = (alpha_k1-alpha_k) >= 0 ? tang((int)((alpha_k1-alpha_k)/2.)) : tang((int)((alpha_k-alpha_k1)/2.));
                            r_current = (tang1 != 0) ? (int)(delta_prev/(2.*tang1)) : RMIN;
                            //if(i == END_P)
                                
                           //if (r_current >= RMIN)
                            if (r_current >= RMIN)
                            {
                                
                                //Set new shortest distance to point i
                                mapa->dI[i] = delta;
                                //Make the removed the parent node of point i
                                mapa->parent[i] = removed;
                                
                                //Add point i to bin, if not yet there
                                if (mapa->is_in_bin[i] == 0)
                                {
                                    //printf("%d, %d, %f, %d\n", removed, i, tang1, r_current);
                                    //printf("new item in bin: %d\n", i);
                                    bin[bin_elements] = i;
                                    bin_elements++;
                                    mapa->is_in_bin[i] = 1;
                                }
                           }
                        }
                    }
                }
                
            }
        }
    }
    
    unsigned long long cycle_end = rdtsc();
    //t = clock()-t;
    unsigned int cycles = (unsigned int)(cycle_end-cycle_start);
    //printf("Cycles for the complete algorithm: %d\n", cycles);
    printf("Amount of time on a 50MHz CPU in ms: %f\n", (float)(cycles)/50000.0);
    //printf("Amount of time on a 2.4GHz CPU in ms: %f\n", (float)t/CLOCKS_PER_SEC*1000);
    //printf("x_end = %d\n", mapa->x[END_P]);
    //printf("%d\n", PF_collision(57,60));
}

void PF_reconstruct(struct map* mapa)
{
    float path_length = 0;
    int node = END_P;
    int prev_node = 0;
    int n_nodes = 0;
    //printf("Visible Objects: %d\n", visible_objects);
    while (node != START_P)
    {
        if (node > mapa->parent[node])
            path_length += mapa->delta[node][mapa->parent[node]];
        else
            path_length += mapa->delta[mapa->parent[node]][node];
        prev_node = node;
        node = mapa->parent[node];
        n_nodes++;
    }
    
    int PATH_xArr[n_nodes], PATH_yArr[n_nodes], PATH_speedArr[n_nodes];
    int i;
    node = END_P;
    //printf("%d\n", n_nodes);
    for (i = n_nodes-1;i>=0;i--)
    {
        PATH_xArr[i] = (int)(mapa->x[node]*1000.);
        PATH_yArr[i] = (int)(mapa->y[node]*1000.);
        PATH_speedArr[i] = 20000;
        printf("x: %f, y: %f, node: %d\n", mapa->x[node], mapa->y[node], node);
        node = mapa->parent[node];
    }
    printf("x: %f, y: %f, node: %d\n", mapa->x[node], mapa->y[node], node);
    //Update the position towards the next point
    for (i=0;i<n_nodes;i++)
    {
        printf("x: %d, y: %d\n", PATH_xArr[i]/1000, PATH_yArr[i]/1000);
    }
    POSGEN_update(mapa->x[prev_node], mapa->y[prev_node], mapa->alpha[prev_node][START_P]);
    
    printf("Shortest Path Length: %f\n Number of nodes: %d\n",path_length, i);
    //float path_matlab = mapa->delta[END_P][55]+mapa->delta[55][32]+mapa->delta[33][32]+mapa->delta[34][33]+mapa->delta[35][34]+mapa->delta[120][35];
    //printf("Shortest Path Matlab: %f\n", path_matlab);
    //printf("%f", (float)(INF+INF));
}



