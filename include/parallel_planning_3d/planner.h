#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <math.h>



namespace planner
{
    //The data struction for the grip map
    struct Node
    
    {
        int x,y,z,parent= 0;
        bool start = false, goal = false, path = false;
        bool obstacle = false, hidden_obstacle = false, explored = false, frontier = false;
        // cost from start and estimate cost to goal
        
        float h = INFINITY, g = INFINITY, f = INFINITY;
        

    };

    struct priority_queue_compare
    {   
        // queue elements are vectors so we need to compare those
        bool operator()(std::vector<float> const& a, std::vector<float> const& b) const
        {

            // reverse sort puts the lowest value at the top    
            return a[1] > b[1];
        }
    };

    //Calculate the heuristic value for newly explored node
    float h_calculation(Node* Node1, Node* Node2);

    //Sort the nodes in the q list
    bool sortcol(const std::vector<float>& v1, const std::vector<float>& v2);
    
    // //Generate the map array for future calculation
    void map_generation(Node* graph, int n, int start, int goal, std::vector<int> & obstacles);

    void add_hidden_obstacles(Node* graph, std::vector<int> & hidden_obstacles);

    //Traditional A* sequential planning
    void sequential_explore(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& path_to_goal);

    bool edge_detection(int explored_index, int n, int i, int* neighbor);

    int coordtoindex(std::vector<int>& coordinate, int n);

    std::vector<int> indextocoord(int index, int n);

    //Check if there is a hidden obstracle nearby
    void obstacle_detection(int current, planner::Node* graph, int n);

    void random_obstacle_block(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& obstacle);

    void clear_obstacle_block(planner::Node* graph, int n, int current, int goal,  std::vector<int>& obstacle);


}















#endif