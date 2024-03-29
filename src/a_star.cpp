#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include <queue>
#include "std_msgs/Int32MultiArray.h"

#include "parallel_planning_3d/planner.h"
#include "parallel_planning_3d/parallel_explore.cuh"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cmath>
#include <random>

extern "C" void parallel_dijkstra(planner::Node* graph, int n, int goal, int resolution_size);

int main (int argc, char **argv) 
{
	// 初始化ROS节点 节点名字
	ros::init (argc, argv, "planner"); 
	 // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray> ("planning_info", 100, ros::init_options::AnonymousName);
    
    //geometry_msgs::Point start_goal;
    std_msgs::Int32MultiArray map;
	
    //generate map info from the config file
    int n, use_random_obstacles, use_dijkstra;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
    ros::param::get("use_random_obstacles", use_random_obstacles);
    ros::param::get("use_dijkstra", use_dijkstra);

    if(use_random_obstacles){

        float ratio;
        ros::param::get("random_obstacles_ratio", ratio);

        int obstacle_size = ratio * n;

        std::cout<<"obstacle "<< obstacle_size<< std::endl;

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
    int start = start_coord[0]+ start_coord[1] * n + start_coord[2] * n * n;
    int goal = goal_coord[0] + goal_coord[1] * n + goal_coord[2] * n * n;

	

	planner::Node* graph = new planner::Node[n*n*n];

	planner::map_generation(graph, n, start, goal, obstacles);

    if (use_dijkstra){

        int resolution_size;
        ros::param::get("resolution_size", resolution_size);
        parallel_dijkstra(graph, n, goal, resolution_size);
        
    }

	bool path_found = false;
	// std::cout << graph[start].f << std::endl;

	// std::vector<std::vector<float> > q_list;
    std::priority_queue< std::vector<float>, std::vector< std::vector<float> >, planner::priority_queue_compare > q_list;

    q_list.push({(float) start, graph[start].f});

	int neighbors[][3] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, {1, 0, 1}, {1, 0, -1}, {1, 1, 0}, {1, -1, 0}, {-1, 0, 1} , {-1, 0, -1} , {-1, 1, 0} , {-1, -1, 0} , {1, 1, 1} , {1, 1, -1} , {1, -1, 1} , {1, -1, -1} , {-1, -1, -1} , {-1, -1, 1} , {-1, 1, -1} , {-1, 1, 1}   };

    int neighbor[26*3];

    for (int i =0; i< 26; i++){
        for (int j=0; j<3; j++){

        neighbor[3*i+j] = neighbors[i][j];



        }

    }
	// std::cout << sizeof(neighbor)/sizeof(int) << std::endl;

	while (ros::ok()) {

        while(ros::ok() && q_list.size()!=0 && !path_found){

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

			if (explored_index == goal){
                std::cout << "found" << std::endl;
                // planner::Node* temp_node = graph[explored_index].parent;
                // while (!temp_node->start){
                   
                //     temp_node->path = true;
                //     temp_node = temp_node->parent;
                // }
                path_found = true;
            }

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

                    //int neighbor[26] = {1, -1, n, -n, n*n, -n*n, n+1, n-1, -n+1, -n-1, n*n+1, n*n-1, n*n+n, n*n-n, -n*n+1, -n*n-1, -n*n+n, -n*n-n, n*n + n + 1, n*n + n- 1,  n*n - n + 1, n*n - n -1, -(n*n + n + 1), -(n*n + n- 1), -(n*n - n + 1), -(n*n - n -1) };
                    
                    if ((new_coord[0] >= n) || (new_coord[0] < 0)  || (new_coord[1] >= n) || (new_coord[1] <0 ) || (new_coord[2] >= n) || (new_coord[2] < 0)){

                        edge_detect = false;
                    }


                    if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
                    {
                        graph[new_index].g = graph[explored_index].g + cost;
                        if (graph[new_index].h==INFINITY){
                            graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal]);
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
                
                int path1 = goal;
                while (path1 != start)
                {
                    graph[path1].path = true;
                    path1 = graph[path1].parent;
                }
            
                
            }

            std::vector<int32_t> v(n*n*n, 0);
            for (int z=0; z<n; z++){
                for (int y =0; y<n; y++){

                    for (int x=0; x<n; x++){

                        if (graph[z*n*n + y*n+x].start) {
                            v[z*n*n + y*n+x] = 1;
                        }
                        else if (graph[z*n*n + y*n+x].goal)
                        {
                            v[z*n*n + y*n+x] = 2;
                        }
                        else if (graph[z*n*n + y*n+x].path){
                            v[z*n*n + y*n+x] = 3;
                        }
                        else if (graph[z*n*n + y*n+x].obstacle){
                            v[z*n*n + y*n+x] = 4;
                        }
                        else if (graph[z*n*n + y*n+x].frontier){
                            v[z*n*n + y*n+x] = 5;
                        }
                        else if (graph[z*n*n + y*n+x].explored){
                            v[z*n*n + y*n+x] = 6;
                        }

                        // v[z*n*n + y*n+x] = graph[z*n*n + y*n+x].h;
                        
                        
                    }
                }
            }
            
            ros::Rate loop_rate(30);
            
            map.data = v;
        
            // 广播
            pub.publish(map);
            loop_rate.sleep(); 

            if (q_list.size()==0) std::cout<<"NO PATH IS FOUND" <<std::endl;
            




		}

        // std::vector<int32_t> v(n*n*n, 0);
        // for (int z=0; z<n; z++){
        //     for (int y =0; y<n; y++){

        //         for (int x=0; x<n; x++){

                   
        //             int x1 = (int)graph[z*n*n + y*n+x].h;
            
        //             v[z*n*n + y*n+x] = x1;
                    
        //         }
        //     }
        // }
        // map.data = v;

        // std::cout<< "Origion " <<  graph[1].h  <<std::endl;
        // std::cout<< "Origion " <<  v[1]  <<std::endl;


        

                        

       
        ros::Rate loop_rate(5);
        
        pub.publish(map);
        loop_rate.sleep(); 

        

        

	}





	return 0; 
}
