#include "parallel_planning_3d/planner.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>


float planner::h_calculation(planner::Node* Node1, planner::Node* Node2){

        return sqrt(pow((Node1->x-Node2->x),2) + pow((Node1->y-Node2->y),2) +  pow((Node1->z-Node2->z),2) );
    }

bool planner::sortcol(const std::vector<float>& v1, const std::vector<float>& v2)
    {
        return v1[1] > v2[1];
    }


void planner::map_generation(planner::Node* graph, int n, int start, int goal, std::vector<int> & obstacles){
    //Assign coordinate
    for (int z=0; z<n; z++){
        for (int y =0; y<n; y++){

            for (int x=0; x<n; x++){

                graph[z*n*n + y*n+x].x = x;
                graph[z*n*n + y*n+x].y = y;
                graph[z*n*n + y*n+x].z = z;
                
            }
        }
    }
    graph[start].start = true;
    graph[start].g = 0;
    graph[start].h = h_calculation(&graph[start], &graph[goal]);
    graph[start].f = graph[start].h + graph[start].g;
    // graph[start].explored = true;

    graph[goal].goal = true;
    // graph[goal].h = 0;

    for (int i =0; i<obstacles.size(); i++){
        graph[obstacles[i]].obstacle = true;
    }


}

int planner::coordtoindex(std::vector<int>& coordinate, int n){

    int index = coordinate[0] + coordinate[1]*n + coordinate[2]*n*n;
    return index;

}

std::vector<int> planner::indextocoord(int index, int n){


    std::vector<int> coordinate(3);

    coordinate[3] = index/(n*n);

    int a = index%(n*n);

    coordinate[0] = a%n;
    coordinate[1] = a/n;

    return coordinate;

}
