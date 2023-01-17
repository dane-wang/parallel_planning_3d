#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include <queue>
#include "std_msgs/Int8MultiArray.h"

#include "parallel_planning_3d/planner.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cmath>

int main (int argc, char **argv) 
{
	// 初始化ROS节点 节点名字
	ros::init (argc, argv, "planner"); 
	 // 节点句柄
	ros::NodeHandle nh; 

    // 发布消息 话题名字 队列大小
	ros::Publisher pub = nh.advertise<std_msgs::Int8MultiArray> ("planning_info", 100, ros::init_options::AnonymousName);
    
    //geometry_msgs::Point start_goal;
    std_msgs::Int8MultiArray map;
	
    //generate map info from the config file
    int n;
    std::vector<int> start_coord, goal_coord;
    std::vector<int> obstacles;
    XmlRpc::XmlRpcValue xml_obstacles;

    ros::param::get("map_size", n);
    ros::param::get("start_position", start_coord);
    ros::param::get("goal_position", goal_coord);
    ros::param::get("obstacles", xml_obstacles);
    

    // Initialize the start and goal node
    int start = start_coord[0]+ start_coord[1] * n + start_coord[2] * n * n;
    int goal = goal_coord[0] + goal_coord[1] * n + goal_coord[2] * n * n;

	for(int i=0; i< xml_obstacles.size(); i++){
        int obstacles_index =  (int)xml_obstacles[i][0] +  (int)xml_obstacles[i][1] * n + (int)xml_obstacles[i][2] * n * n;
        obstacles.push_back(obstacles_index);
    }

	planner::Node* graph = new planner::Node[n*n*n];

	planner::map_generation(graph, n, start, goal, obstacles);

	bool path_found = false;
	// std::cout << graph[start].f << std::endl;

	// std::vector<std::vector<float> > q_list;
    std::priority_queue< std::vector<float>, std::vector< std::vector<float> >, planner::priority_queue_compare > q_list;

    q_list.push({(float) start, graph[start].f});

	int neighbor[26] = {1, -1, n, -n, n*n, -n*n, n+1, n-1, -n+1, -n-1, n*n+1, n*n-1, n*n+n, n*n-n, -n*n+1, -n*n-1, -n*n+n, -n*n-n, n*n + n + 1, n*n + n- 1,  n*n - n + 1, n*n - n -1, -(n*n + n + 1), -(n*n + n- 1), -(n*n - n + 1), -(n*n - n -1) };

	// std::cout << sizeof(neighbor)/sizeof(int) << std::endl;

	while (ros::ok()) {

        while(ros::ok() && q_list.size()!=0 && !path_found){

			auto smallest_node = q_list.top();
            q_list.pop();

			int explored_index = smallest_node[0];
            int floor_index = explored_index%(n*n);
            int vertical_index = explored_index/(n*n);
            int row_index = floor_index / n;

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
                        graph[new_index].h = planner::h_calculation(&graph[new_index], &graph[goal]);
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
                
                int path1 = goal;
                while (path1 != start)
                {
                    graph[path1].path = true;
                    path1 = graph[path1].parent;
                }
            
                
            }

            std::vector<int8_t> v(n*n*n, 0);
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
                        
                        
                    }
                }
            }
            
            ros::Rate loop_rate(5);
            
            map.data = v;
        
            // 广播
            pub.publish(map);
            loop_rate.sleep(); 

            




		}

        if (path_found){
            ros::Rate loop_rate(5);
            pub.publish(map);
            loop_rate.sleep(); 

        }

        

	}





	return 0; 
}
