#include "parallel_planning_3d/planner.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <math.h>
#include <random>

float planner::h_calculation(planner::Node* Node1, planner::Node* Node2){

        return sqrt(pow((Node1->x-Node2->x),2) + pow((Node1->y-Node2->y),2) +  pow((Node1->z-Node2->z),2) );
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
    graph[start_index].h = planner::h_calculation(&graph[start_index], &graph[goal_index]);
    graph[start_index].f = graph[start_index].g + graph[start_index].h;
    q_list.push({(float) start_index, graph[start_index].f});
    int neighbor[26] = {1, -1, n, -n, n*n, -n*n, n+1, n-1, -n+1, -n-1, n*n+1, n*n-1, n*n+n, n*n-n, -n*n+1, -n*n-1, -n*n+n, -n*n-n, n*n + n + 1, n*n + n- 1,  n*n - n + 1, n*n - n -1, -(n*n + n + 1), -(n*n + n- 1), -(n*n - n + 1), -(n*n - n -1) };

    while(q_list.size()!=0 && !path_found){
        // pop the node with smallest node
        auto smallest_node = q_list.top();
        q_list.pop();

        int explored_index = smallest_node[0];
        int floor_index = explored_index%(n*n);
        int vertical_index = explored_index/(n*n);
        int row_index = floor_index / n;


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
                int new_index = explored_index + neighbor[i];
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

                //int neighbor[26] = {1, -1, n, -n, n*n, -n*n, n+1, n-1, -n+1, -n-1, n*n+1, n*n-1, n*n+n, n*n-n, -n*n+1, -n*n-1, -n*n+n, -n*n-n, n*n + n + 1, n*n + n- 1,  n*n - n + 1, n*n - n -1, -(n*n + n + 1), -(n*n + n- 1), -(n*n - n + 1), -(n*n - n -1) };
                
                bool left_edge_out = (floor_index%n ==0) && (i==1|| i==7 || i==9 || i==11 || i==15 || i==19 || i==21 || i==22 || i==24);

                bool right_edge_out = ((floor_index+1)%n ==0) && (i==0 || i==6 || i==8 || i==10 || i==14 || i==18 || i==20 || i==23 || i==25);

                bool front_edge_out = ((row_index+1)%n ==0) && (i==2 || i==6 || i==7 || i==12 || i==16 || i==18 || i==19 || i==24 || i==25);

                bool back_edge_out = (row_index == 0) && (i==3 || i==8 || i==9 || i==13 || i==17 || i==20 || i==21 || i==22 || i==23);

                bool top_edge_out = ((vertical_index+1)%n ==0) && (i==4 || i==10 || i==11 || i==12 || i==13 || i==20 || i==21 || i==18 || i==19);

                bool bot_edge_out = (vertical_index == 0) && (i==5 || i==14 || i==15 || i==16 || i==17 || i==24 || i==25 || i==22 || i==23);

                if (left_edge_out || right_edge_out || front_edge_out || back_edge_out || top_edge_out || bot_edge_out){
                    edge_detect = false;
                }

                if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                {
                    graph[new_index].g = graph[explored_index].g + cost;
                    graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal_index]);
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
                        q_list.push({(float) new_index, graph[new_index].f});

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

