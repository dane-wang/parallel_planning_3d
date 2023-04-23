#include "parallel_planning_3d/planner.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <math.h>
#include <random>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <cuda.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/merge.h>



float planner::h_calculation(planner::Node* Node1, planner::Node* Node2){

        return sqrt(pow((Node1->x-Node2->x),2) + pow((Node1->y-Node2->y),2) +  pow((Node1->z-Node2->z),2) );
        // int a = std::abs(Node1->x-Node2->x);
        // int b = std::abs(Node1->y-Node2->y);
        // int c = std::abs(Node1->z-Node2->z);
        // std::vector<float> coordinate_difference =  {(float) a, (float) b, (float) c};

        // float d = *max_element(coordinate_difference.begin(),coordinate_difference.end());

        // return 10.0*d;
    }

float planner::h_calculation(planner::BiNode* Node1, planner::BiNode* Node2){

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

    // for (int i =0; i<obstacles.size(); i++){
    //     if (graph[obstacles[i]].start==false && graph[obstacles[i]].goal==false){
    //         graph[obstacles[i]].obstacle = true;
    //     }
        
    // }
    planner::random_obstacle_block(graph, n, start, goal, obstacles);


}


void planner::map_generation(planner::BiNode* graph, int n, int start, int goal, std::vector<int> & obstacles){
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
    graph[goal].b_g = 0;
    graph[goal].b_h = h_calculation(&graph[start], &graph[goal]);
    graph[goal].b_f = graph[goal].b_h + graph[goal].b_g;
    // graph[goal].h = 0;

    // for (int i =0; i<obstacles.size(); i++){
    //     if (graph[obstacles[i]].start==false && graph[obstacles[i]].goal==false){
    //         graph[obstacles[i]].obstacle = true;
    //     }
        
    // }
    planner::random_obstacle_block(graph, n, start, goal, obstacles);


}

int planner::coordtoindex(std::vector<int>& coordinate, int n){

    int index = coordinate[0] + coordinate[1]*n + coordinate[2]*n*n;
    return index;

}

std::vector<int> planner::indextocoord(int index, int n){


    std::vector<int> coordinate(3);

    coordinate[2] = index/(n*n);

    int a = index%(n*n);

    coordinate[0] = a%n;
    coordinate[1] = a/n;

    return coordinate;

}

// __device__ int planner::coordtoindex_gpu(thrust::device_vector<int>& coordinate, int n){

//     int index = coordinate[0] + coordinate[1]*n + coordinate[2]*n*n;
//     return index;
// }

// __device__ thrust::device_vector<int> planner::indextocoord_gpu(int index, int n){

//     thrust::device_vector<int> coordinate(3);

//     coordinate[2] = index/(n*n);

//     int a = index%(n*n);

//     coordinate[0] = a%n;
//     coordinate[1] = a/n;

//     return coordinate;


// }


void planner::add_hidden_obstacles(Node* graph, std::vector<int> & hidden_obstacles){
    for (int i =0; i<hidden_obstacles.size(); i++){
        graph[hidden_obstacles[i]].hidden_obstacle = true;
    }
}


void planner::sequential_explore(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& path_to_goal){

    //Initialize everything
    bool path_found = false;
    std::priority_queue< std::vector<float>, std::vector< std::vector<float> >, planner::priority_queue_compare > q_list;
    graph[start_index].g = 0;
    graph[start_index].h = (float) planner::h_calculation(&graph[start_index], &graph[goal_index]);
    graph[start_index].f = graph[start_index].g + graph[start_index].h;

    std::cout << graph[start_index].h << std::endl;

    q_list.push({(float) start_index, graph[start_index].f});
    int neighbors[][3] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, {1, 0, 1}, {1, 0, -1}, {1, 1, 0}, {1, -1, 0}, {-1, 0, 1} , {-1, 0, -1} , {-1, 1, 0} , {-1, -1, 0} , {1, 1, 1} , {1, 1, -1} , {1, -1, 1} , {1, -1, -1} , {-1, -1, -1} , {-1, -1, 1} , {-1, 1, -1} , {-1, 1, 1}   };

    int neighbor[26*3];

    for (int i =0; i< 26; i++){
        for (int j=0; j<3; j++){

        neighbor[3*i+j] = neighbors[i][j];



        }

    }

    while(q_list.size()!=0 && !path_found){
        // pop the node with smallest node
        auto smallest_node = q_list.top();
        q_list.pop();

        int explored_index = smallest_node[0];

        if (graph[explored_index].explored) continue;


        int explored_coord[3];
        explored_coord[2] = explored_index/(n*n);

        int a = explored_index%(n*n);

        explored_coord[0] = a%n;
        explored_coord[1] = a/n;



        //std::cout << explored_index << std::endl;

        graph[explored_index].explored = true;
        graph[explored_index].frontier = false;

        // if we found the path
        if (explored_index == goal_index){
            std::cout << "found" << std::endl;
            // planner::Node* temp_node = graph[explored_index].parent;
            // while (!temp_node->start){
                
            //     temp_node->path = true;
            //     temp_node = temp_node->parent;
            // }
            path_found = true;
        }

        //std::cout << new_explore_index << std::endl;
        if (!path_found){
            for (int i=0; i<26; i++)
            {
                int neighbor1[3];
                neighbor1[0] = neighbor[3*i];
                neighbor1[1] = neighbor[3*i+1];
                neighbor1[2] = neighbor[3*i+2];

                
                // printf("Checking %d, %d, %d\n", (int) neighbor[0], (int) neighbor[1], (int) neighbor[2]);


                int new_coord[3];
                new_coord[0] = explored_coord[0] + neighbor1[0];
                new_coord[1] = explored_coord[1] + neighbor1[1];
                new_coord[2] = explored_coord[2] + neighbor1[2];

                
                int new_index = new_coord[0] + new_coord[1]*n + new_coord[2]*n*n;
    

                if (new_index<0 || new_index >= n*n*n) continue;

                float cost;

                if (i<6){
                    cost = 1;
                }
                else if (i<18)
                {
                    cost = sqrt(2.0);
                }
                else {
                    cost = sqrt(3.0);
                }
                

                //Check if the new index possible (like if it will go out of the map)
                bool edge_detect = true;

                 if ((new_coord[0] >= n) || (new_coord[0] < 0)  || (new_coord[1] >= n) || (new_coord[1] <0 ) || (new_coord[2] >= n) || (new_coord[2] < 0)){

                    edge_detect = false;
                }

                if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                {
                    graph[new_index].g = graph[explored_index].g + cost;
                    if (graph[new_index].h==INFINITY) {
                        graph[new_index].h = (float) planner::h_calculation(&graph[new_index], &graph[goal_index]);
                        // std::cout << "calculating " << new_index << std::endl;

                    } 
                    graph[new_index].f = graph[new_index].h + graph[new_index].g;
                    graph[new_index].parent = explored_index;
                    graph[new_index].frontier = true;

                    q_list.push({(float) new_index, graph[new_index].f});
                }
                else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
                {
                    if (graph[new_index].g > graph[explored_index].g + cost)
                    {
                        graph[new_index].g = graph[explored_index].g + cost;
                        graph[new_index].f = graph[new_index].h + graph[new_index].g;
                        graph[new_index].parent = explored_index;

                        if (graph[new_index].explored == true) {
                            graph[new_index].explored == false;
                            q_list.push({(float) new_index, graph[new_index].f});
                        }
                    }
                }
                

            }
        
            // std::sort(q_list.begin(), q_list.end(), planner::sortcol);
        }
        else{
            int path1 = goal_index;
            while (path1 != start_index)
            {
                path_to_goal.push_back(path1);
                graph[path1].path = true;
                path1 = graph[path1].parent;
            }
        
        }
        //std::cout << q_list.size() << std::endl;
        
    }
    if (q_list.size()==0) std::cout<< "NO PATH IS FOUND" <<std::endl;        
    
}


//Check if there is a hidden obstracle nearby
void planner::obstacle_detection(int current, planner::Node* graph, int n){
    
    // std::cout <<"checking" <<std::endl;
    std::vector<int> curr_coordinate = planner::indextocoord(current, n);
    // std::cout<< curr_coordinate[0]<<curr_coordinate[1]<<curr_coordinate[2]<<std::endl;

    for (int i = -2; i<3; i++){
        for (int j = -2; j<3; j++){
            for (int k=-2; k<3; k++){

                

                std::vector<int> new_coordinate{(curr_coordinate[0]+i), (curr_coordinate[1]+j), (curr_coordinate[2]+k) };
                
                // std::cout<< new_coordinate[0]<<new_coordinate[1]<<new_coordinate[2]<<std::endl;

                
                if (new_coordinate[0]>=n || new_coordinate[0]<0 || new_coordinate[1]>=n || new_coordinate[1]<0 || new_coordinate[2]>=n || new_coordinate[2]<0  ) continue;

                int new_index = planner::coordtoindex(new_coordinate, n);

                if (graph[new_index].hidden_obstacle){

                    // std::cout<< new_coordinate[0]<<new_coordinate[1]<<new_coordinate[2]<<std::endl;

                    std::cout <<"checking" << std::endl;

                    graph[new_index].hidden_obstacle = false;
                    graph[new_index].obstacle = true;

                } 
                new_coordinate.clear();
            }
        }
    }

}


void planner::random_obstacle_block(planner::Node* graph, int n, int start_index, int goal_index, std::vector<int>& obstacle){

    for (int i=0; i<obstacle.size(); i++){

        int obstacle_index = obstacle[i];
        std::vector<int> obstacle_coord = planner::indextocoord(obstacle_index, n);
        std::vector<int> start_coord = planner::indextocoord(start_index, n);
        std::vector<int> goal_coord = planner::indextocoord(goal_index, n);
        std::vector<int> obstacle_size(3);

        std::random_device rnd_device;
        // Specify the engine and distribution.
        std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
        std::uniform_int_distribution<int> dist {3, (int) n/5};
        
        auto gen = [&dist, &mersenne_engine](){
                    return dist(mersenne_engine);
                };

        
        std::generate(std::begin(obstacle_size), std::end(obstacle_size), gen);

        std::vector<int> start_dis(3), goal_dis(3);

        bool appropriate_obstacle = true;

        for (int j=0; j<3; j++){
            int a,b;
            a = start_coord[j] - obstacle_coord[j];
            b = goal_coord[j] - obstacle_coord[j];

            if ((a>0 && a<= obstacle_size[j]) || (b>0 && b<= obstacle_size[j])) appropriate_obstacle = false;

        }

        if (!appropriate_obstacle) continue;

        for (int x=0; x<obstacle_size[0]; x++){
            for (int y=0; y<obstacle_size[1]; y++){
                for (int z=0; z<obstacle_size[2]; z++){

                    if ((obstacle_coord[0]+x < n) && (obstacle_coord[1]+y < n) && (obstacle_coord[2]+z < n)){

                        std::vector<int> new_coord = {obstacle_coord[0]+x, obstacle_coord[1]+y, obstacle_coord[2]+z};
                        int new_index = planner::coordtoindex(new_coord, n);

                        graph[new_index].obstacle = true;

                    }


                }
            }
        }




    }



}

void planner::random_obstacle_block(planner::BiNode* graph, int n, int start_index, int goal_index, std::vector<int>& obstacle){

    for (int i=0; i<obstacle.size(); i++){

        int obstacle_index = obstacle[i];
        std::vector<int> obstacle_coord = planner::indextocoord(obstacle_index, n);
        std::vector<int> start_coord = planner::indextocoord(start_index, n);
        std::vector<int> goal_coord = planner::indextocoord(goal_index, n);
        std::vector<int> obstacle_size(3);

        std::random_device rnd_device;
        // Specify the engine and distribution.
        std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
        std::uniform_int_distribution<int> dist {3, (int) n/5};
        
        auto gen = [&dist, &mersenne_engine](){
                    return dist(mersenne_engine);
                };

        
        std::generate(std::begin(obstacle_size), std::end(obstacle_size), gen);

        std::vector<int> start_dis(3), goal_dis(3);

        bool appropriate_obstacle = true;

        for (int j=0; j<3; j++){
            int a,b;
            a = start_coord[j] - obstacle_coord[j];
            b = goal_coord[j] - obstacle_coord[j];

            if ((a>0 && a<= obstacle_size[j]) || (b>0 && b<= obstacle_size[j])) appropriate_obstacle = false;

        }

        if (!appropriate_obstacle) continue;

        for (int x=0; x<obstacle_size[0]; x++){
            for (int y=0; y<obstacle_size[1]; y++){
                for (int z=0; z<obstacle_size[2]; z++){

                    if ((obstacle_coord[0]+x < n) && (obstacle_coord[1]+y < n) && (obstacle_coord[2]+z < n)){

                        std::vector<int> new_coord = {obstacle_coord[0]+x, obstacle_coord[1]+y, obstacle_coord[2]+z};
                        int new_index = planner::coordtoindex(new_coord, n);

                        graph[new_index].obstacle = true;

                    }


                }
            }
        }




    }



}

void planner::clear_obstacle_block(planner::Node* graph, int n, int current, int goal,  std::vector<int>& obstacle){


    for (int i=0; i<obstacle.size(); i++){

        int obstacle_index = obstacle[i];
        std::vector<int> obstacle_coord = planner::indextocoord(obstacle_index, n);
        int size = (int) n/5;

        for (int x=0; x<size; x++){
            for (int y=0; y<size; y++){
                for (int z=0; z<size; z++){

                    if ((obstacle_coord[0]+x < n) && (obstacle_coord[1]+y < n) && (obstacle_coord[2]+z < n)){

                        std::vector<int> new_coord = {obstacle_coord[0]+x, obstacle_coord[1]+y, obstacle_coord[2]+z};
                        int new_index = planner::coordtoindex(new_coord, n);

                        graph[new_index].obstacle = false;

                    }


                }
            }
        }


    }

}

void planner::clear_obstacle_block(planner::BiNode* graph, int n, int current, int goal,  std::vector<int>& obstacle){


    for (int i=0; i<obstacle.size(); i++){

        int obstacle_index = obstacle[i];
        std::vector<int> obstacle_coord = planner::indextocoord(obstacle_index, n);
        int size = (int) n/5;

        for (int x=0; x<size; x++){
            for (int y=0; y<size; y++){
                for (int z=0; z<size; z++){

                    if ((obstacle_coord[0]+x < n) && (obstacle_coord[1]+y < n) && (obstacle_coord[2]+z < n)){

                        std::vector<int> new_coord = {obstacle_coord[0]+x, obstacle_coord[1]+y, obstacle_coord[2]+z};
                        int new_index = planner::coordtoindex(new_coord, n);

                        graph[new_index].obstacle = false;

                    }


                }
            }
        }


    }

}



std::vector<int> planner::goal_generation( int start, int goal, int n){

    float cost = 0;

    

    std::random_device rnd_device;
    // Specify the engine and distribution.
    std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
    std::uniform_int_distribution<int> dist {0, n*n*n-1};
    
    auto gen = [&dist, &mersenne_engine](){
                return dist(mersenne_engine);
            };

    
    while (cost<1.2*n)
    {
        std::vector<int> start_and_goal(2);
        std::generate(std::begin(start_and_goal), std::end(start_and_goal), gen);
        start = start_and_goal[0];
        goal = start_and_goal[1];

        std::vector<int> start_coordinate = planner::indextocoord(start, n);
        std::vector<int> goal_coordinate = planner::indextocoord(goal, n);


        cost = sqrt(pow((start_coordinate[0]-goal_coordinate[0]),2) + pow((start_coordinate[1]- goal_coordinate[1]),2) + pow((start_coordinate[2]-goal_coordinate[2]),2));
        start_and_goal.clear();
        
        
    }

        std::vector<int> start_coordinate = planner::indextocoord(start, n);
        std::vector<int> goal_coordinate = planner::indextocoord(goal, n);

        std::cout << "Start "<< start_coordinate[0] << " " << start_coordinate[1]<< " " << start_coordinate[2]  << std::endl;
        std::cout << "Goal "<< goal_coordinate[0] << " " << goal_coordinate[1] << " " << goal_coordinate[1] << std::endl;
        // std::cout << start << " " << goal << " " << cost << std::endl;

        

        std::vector<int> position = {start, goal};
        return position;



}
