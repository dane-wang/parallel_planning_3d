#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include <queue>
#include "parallel_planning_3d/planner.h"
#include "std_msgs/Int32MultiArray.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stdio.h>
#include <iostream>
#include <queue>
#include "parallel_planning_3d/parallel_explore.cuh"
#include <random>

#include <chrono>


extern "C" void gpu_warmup();

extern "C" void parallel_bi_explore(planner::BiNode* graph, int n, int start, int goal, int max_thread_size, std::vector<int>& path);



int main(int argc, char** argv){
    ros::init(argc, argv, "parallel_planning_timing");
    
    ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray> ("planning_info", 100, ros::init_options::AnonymousName);
    ros::Rate loop_rate(5);

    std_msgs::Int32MultiArray map;
    
    //generate map info from the config file
    int n, max_thread_size, use_parallel_planning, use_random_obstacles, dynamtic_obstacles;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    std::vector<int> hidden_obstacles;
    
    XmlRpc::XmlRpcValue xml_hidden_obstacles;

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
  
    ros::param::get("hidden_obstacles", xml_hidden_obstacles);
    ros::param::get("max_thread", max_thread_size);
    ros::param::get("use_parallel", use_parallel_planning);
    ros::param::get("use_random_obstacles", use_random_obstacles);
    ros::param::get("dynamtic_obstacles", dynamtic_obstacles);

    if(use_random_obstacles){

        float ratio;
        ros::param::get("random_obstacles_ratio", ratio);

        int obstacle_size = ratio * n;

        // First create an instance of an engine.
        std::random_device rnd_device;
        // Specify the engine and distribution.
        std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
        std::uniform_int_distribution<int> dist {0, n*n*n-1};
        
        auto gen = [&dist, &mersenne_engine](){
                    return dist(mersenne_engine);
                };

        
        std::vector<int> vec(obstacle_size);
        std::generate(std::begin(vec), std::end(vec), gen);
        
        obstacles = vec;
        

        

    }
    else{
        XmlRpc::XmlRpcValue xml_obstacles;
        ros::param::get("obstacles", xml_obstacles);
        for(int i=0; i< xml_obstacles.size(); i++){
            int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n + (int)xml_obstacles[i][2] * n * n;
            obstacles.push_back(obstacles_index);
        }
    }
    

    // Initialize the start and goal node
    int start = planner::coordtoindex(start_coord, n);
    int goal = planner::coordtoindex(goal_coord, n);
    int current = start;
    bool path_found = false;
    bool no_path = false;
    int step = 0;
    bool waiting_for_new_obstacles = false;

  
    // Initialize the obstacles list
    // for(int i=0; i< xml_obstacles.size(); i++){
    //     int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n +   (int)xml_obstacles[i][2] * n * n;
    //     obstacles.push_back(obstacles_index);
    // }

    // // Initialize the hidden obstacles list
    // for(int i=0; i< xml_hidden_obstacles.size(); i++){
    //     int hidden_obstacles_index =  (int)xml_hidden_obstacles[i][0] +  (int)xml_hidden_obstacles[i][1] * n + (int)xml_hidden_obstacles[i][2] * n * n;
    //     hidden_obstacles.push_back( hidden_obstacles_index);
    // }

    planner::BiNode* graph = new planner::BiNode[n*n*n];

 
    
    

	planner::map_generation(graph, n, start, goal, obstacles);

    // planner::Node graph_copy[n*n*n];
   
    // planner::add_hidden_obstacles(&graph[0], hidden_obstacles);

    
    
    std::vector<int> path;

    if (use_parallel_planning) gpu_warmup();

  

    while (ros::ok()){
        while (ros::ok() && current!=goal && !no_path)
        {
           
            if (!path_found && !waiting_for_new_obstacles){
                
                planner::BiNode* graph_copy = new planner::BiNode[n*n*n];
                std::copy(graph, graph+n*n*n, graph_copy);
                auto start_time = std::chrono::high_resolution_clock::now();
                // std::cout << current << std::endl;
            
                parallel_bi_explore(graph_copy, n, current, goal, max_thread_size, path);
          
                auto stop = std::chrono::high_resolution_clock::now();
                float duration = std::chrono::duration<float, std::milli>(stop - start_time).count();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_time);
                std::cout << "Exectuation time is " << duration << std::endl;

                delete[] graph_copy;
                
                if (path.size()==0){
                    if( dynamtic_obstacles){
                        std::cout<< "Waiting for new obstacles "<< std::endl;
                        waiting_for_new_obstacles = true;
                    }
                    else{
                        no_path = true;
                    }
                    
                }
                else{
                    std::cout<< "Path length is "<<path.size()<< std::endl;
                    path_found = true;

                }
                
            }

            
            
            // planner::obstacle_detection(current, &graph[0], n);
            // std::cout<< "Current "<<current<< std::endl;

            

            std::vector<int32_t> v(n*n*n, 0);

            if (path_found){
                current = path.back();
                path.pop_back();
                
                for (int k =0; k< path.size(); k++){
                    v[path[k]] = 3;
                }

            }
            step++;

           
            
            for (int z=0; z<n; z++)
            {
                for (int y =0; y<n; y++)
                { 
                    for (int x=0; x<n; x++){

                        if (graph[z*n*n + y*n+x].start) {
                        
                            v[z*n*n + y*n+x] = 1;
                            }
                        else if (graph[z*n*n + y*n+x].obstacle){
                            v[z*n*n + y*n+x] = 4;
                            }

                        else if (z*n*n + y*n+x == current){
                            v[z*n*n + y*n+x] = 6;
                            }
                        else if (graph[z*n*n + y*n+x].goal)
                        {
                            v[z*n*n + y*n+x] = 2;
                            }                                         
                    }
                }
            }
       
                
            map.data = v;
            
        
            pub.publish(map);
            // ros::spinOnce(); 
            loop_rate.sleep(); 

            if(step>10 && dynamtic_obstacles){

                // delete[] graph;
                // planner::Node* graph1 = new planner::Node[n*n*n];
                planner::clear_obstacle_block(graph, n, current, goal, obstacles);
                obstacles.clear();

                float ratio;
                ros::param::get("random_obstacles_ratio", ratio);

                int obstacle_size = ratio * n;

                // First create an instance of an engine.
                std::random_device rnd_device;
                // Specify the engine and distribution.
                std::mt19937 mersenne_engine {rnd_device()};  // Generates random integers
                std::uniform_int_distribution<int> dist {0, n*n*n-1};
        
                auto gen = [&dist, &mersenne_engine](){
                            return dist(mersenne_engine);
                        };

                std::vector<int> vec1(obstacle_size);
                std::generate(std::begin(vec1), std::end(vec1), gen);
        
                obstacles = vec1;

	            // planner::map_generation(graph1, n, current, goal, obstacles);
                // // graph1[current].start = false;
                // // graph1[start].start = true;

                // graph = graph1;
                planner::random_obstacle_block(graph, n, current, goal, obstacles);

                waiting_for_new_obstacles = false;
                step = 0;

                std::cout<< "new obstacle" << std::endl;


            }


            if (!path.empty()){
                //Check if we will hit obstacles on our path
                for (int i =(path.size()-1); i> (path.size()-5); i--){
                    int path_index = path[i];
                    if (graph[path_index].obstacle) {
                        path_found = false;
                        std::cout <<"replanning" << std::endl;
                        path.clear();
                    
                        }

                    }
            }
            
        }


        pub.publish(map);
        ros::spinOnce(); 
        loop_rate.sleep(); 

            

        

    }
    
    

    return 0;
}