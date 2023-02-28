#include <ros/ros.h> 
//#include <geometry_msgs>
// #include "geometry_msgs/Point.h"
#include <time.h>  
#include <vector>
#include "parallel_planning_3d/planner.h"
#include "std_msgs/Int32MultiArray.h"
#include <algorithm>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <cuda.h>

#include <iostream>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/merge.h>
#include <queue>
#include <random>


__device__ bool path_found_gpu;
__device__ int neighbor_gpu[3*26];
__device__ int goal_gpu;


struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



template <typename T, typename T1> 
__global__ void get_f(T* q,  planner::Node* graph, T1* h, int q_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < q_size){
    int node = q[tid];

    h[tid] = graph[node].f;

    // printf("%d", q[tid]);
  }

}

template <typename T>
__global__ void explore(T* q,  planner::Node* graph, T* new_q, int q_size , int n )
{
  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid<q_size) {

    int explored_index = q[tid];
  
    int explored_coord[3];
    explored_coord[2] = explored_index/(n*n);

    int a = explored_index%(n*n);

    explored_coord[0] = a%n;
    explored_coord[1] = a/n;


    graph[explored_index].explored = true;
    graph[explored_index].frontier = false;

    if (graph[explored_index].goal){
      printf("FOUND");
      printf("Hello from thread %d, I am exploring %d\n", tid, explored_index);
      // planner::Node* temp_node = graph[explored_index].parent;
      // while (!temp_node->start){
        
      //     temp_node->path = true;
      //     temp_node = temp_node->parent;
      // }
      path_found_gpu = true;
    }

    if (!path_found_gpu){
      for (int i=0; i<26; i++)

      { 

         
        int neighbor[3];
        neighbor[0] = neighbor_gpu[3*i];
        neighbor[1] = neighbor_gpu[3*i+1];
        neighbor[2] = neighbor_gpu[3*i+2];  

        int new_coord[3];
        new_coord[0] = explored_coord[0] + neighbor[0];
        new_coord[1] = explored_coord[1] + neighbor[1];
        new_coord[2] = explored_coord[2] + neighbor[2];

        // printf("Checking %d, %d, %d\n", (int) new_coord[0], (int) new_coord[1], (int) new_coord[2]);

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

        bool edge_detect = true;
     
        if ((new_coord[0] >= n) || (new_coord[0] < 0)  || (new_coord[1] >= n) || (new_coord[1] <0 ) || (new_coord[2] >= n) || (new_coord[2] < 0)){
            edge_detect = false;
        }

        


        if (graph[new_index].obstacle == false && graph[new_index].frontier == false && graph[new_index].explored == false && edge_detect)
        {
          graph[new_index].g = graph[explored_index].g + cost;
            
          float h_1 = sqrt(pow((graph[new_index].x-graph[goal_gpu].x),2) + pow((graph[new_index].y-graph[goal_gpu].y),2) + pow((graph[new_index].z-graph[goal_gpu].z),2) );
            // printf("%f", h_1);
          graph[new_index].h = h_1;

            
          graph[new_index].f = graph[new_index].h + graph[new_index].g;
          graph[new_index].parent = explored_index;
          graph[new_index].frontier = true;
          
          new_q[26*tid+i] = new_index;
        }
        else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].frontier == true || graph[new_index].explored == true))
        {
          if (graph[new_index].g > graph[explored_index].g + cost)
          {
            graph[new_index].g = graph[explored_index].g + cost;
            graph[new_index].f = graph[new_index].h + graph[new_index].g;
            graph[new_index].parent = explored_index;
          }
        }
      }

    }
  }

}




  


int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel_planning");
  ros::NodeHandle nh; 

  // 发布消息 话题名字 队列大小
  ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray> ("planning_info", 100, ros::init_options::AnonymousName);
    
  //geometry_msgs::Point start_goal;
  std_msgs::Int32MultiArray map;
  

  //generate map info from the config file
  int n, max_thread_size, use_random_obstacles;
  std::vector<int> start_coord, goal_coord;
  std::vector<int> obstacles;
 

  ros::param::get("map_size", n);
  ros::param::get("start_position", start_coord);
  ros::param::get("goal_position", goal_coord);
  ros::param::get("use_random_obstacles", use_random_obstacles);
  ros::param::get("max_thread", max_thread_size);

  // Initialize the start and goal node
  int start = start_coord[0]+ start_coord[1] * n + start_coord[2] * n * n;
  int goal = goal_coord[0] + goal_coord[1] * n + goal_coord[2] * n * n;

  if(use_random_obstacles){

        float ratio;
        ros::param::get("random_obstacles_ratio", ratio);

        int obstacle_size = ratio * n;

        // std::cout<<"obstacle "<< obstacle_size<< std::endl;

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


  planner::Node* graph = new planner::Node[n*n*n];

	planner::map_generation(graph, n, start, goal, obstacles);

  int path1 = goal;
  bool path_found = false;

  // Start to work with CUDA
  thrust::host_vector<int> q_lists;

  q_lists.push_back(start);

  // Start to allocate memory on gpu:
  
  const int map_size = n*n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  int neighbors[][3] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, {1, 0, 1}, {1, 0, -1}, {1, 1, 0}, {1, -1, 0}, {-1, 0, 1} , {-1, 0, -1} , {-1, 1, 0} , {-1, -1, 0} , {1, 1, 1} , {1, 1, -1} , {1, -1, 1} , {1, -1, -1} , {-1, -1, -1} , {-1, -1, 1} , {-1, 1, -1} , {-1, 1, 1}   };

  int neighbor[26*3];
  for (int i =0; i< 26; i++){
    for (int j=0; j<3; j++){

      neighbor[3*i+j] = neighbors[i][j];



    }

  }

  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, graph, map_size, cudaMemcpyHostToDevice);

  cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  3*26*sizeof(int));
  cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));


  thrust::device_vector<int> q_lists_gpu = q_lists;

  // parallel_explore(&graph[0], n, path_found, start, max_thread_size);
  
  


  while (ros::ok()) {
    while(ros::ok() && q_lists_gpu.size()!=0 && !path_found){

      int q_size = q_lists_gpu.size();
      // std::cout << "q size is" << q_size << std::endl;

      
      

      //Determine how many thread should be launched
      int thread_size_needed = min(max_thread_size, q_size);
      int block_size, thread_size;

      if (thread_size_needed <=1024){
        block_size = 1;
        thread_size = thread_size_needed;
      }
      else{
        block_size = (thread_size_needed/1024) + 1;
        thread_size = 1024;
      }

      //new_q is the list store the frontier generated from this step of exploration
      thrust::device_vector<int> new_q_lists_gpu(26*thread_size_needed);
      thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);


 
      //Launch the kernel to explore the map
      explore<<<block_size,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()), thread_size_needed, n);
      cudaDeviceSynchronize();
      cudaMemcpyFromSymbol(&path_found, path_found_gpu,  sizeof(bool), 0, cudaMemcpyDeviceToHost );

      
      cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );


      // Remove all element that is not used during the exploration and repeated value
      
      new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
      thrust::sort(new_q_lists_gpu.begin(), new_q_lists_gpu.end());
      new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );

      // std::cout << "new q size is" << new_q_lists_gpu.size() << std::endl;
      
      // Create new q list based on origional and updated q
      if (q_size <= max_thread_size) {
        q_lists_gpu.clear();
        q_lists_gpu = new_q_lists_gpu;
        new_q_lists_gpu.clear();
      }
      else {
        
        q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+max_thread_size );
        q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
        thrust::sort(q_lists_gpu.begin(), q_lists_gpu.end());
        q_lists_gpu.erase(thrust::unique(q_lists_gpu.begin(), q_lists_gpu.end()), q_lists_gpu.end() );
        new_q_lists_gpu.clear();

        // //sort the q_list based on the f value
        thrust::device_vector<float> f_value(q_lists_gpu.size());
        get_f<<<1, q_lists_gpu.size()>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(f_value.data()), q_lists_gpu.size() );
        thrust::sort_by_key(f_value.begin(), f_value.end(), q_lists_gpu.begin() );
      }

      
      //q_size = q_lists_gpu.size();
      // thrust::device_vector<float> h_value(q_size);

      // if (q_size > 1024) {
      //   int block = q_size / 1024 + 1;
        
      //   get_h<<<block, 1024>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(&h_value[0]), q_size );

      //   thrust::sort_by_key(h_value.begin(), h_value.end(), q_lists_gpu.begin() );

      // }

      if (path_found){
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
                    
                    
                }
            }
        }
      // thrust::host_vector<int> q_lists1= q_lists_gpu;
      // thrust::sort(q_lists1.begin(), q_lists1.end());
      // const bool hasDuplicates = std::adjacent_find(q_lists1.begin(), q_lists1.end()) != q_lists1.end();
      // std::cout << "Duplicates" << hasDuplicates << std::endl;
      
      // for (int k =0; k< q_lists1.size(); k++){

      //   v[q_lists1[k]] = 2;

      // }
      ros::Rate loop_rate(5);
            
      map.data = v;

      // map.points[10] = 120;
      // map.points[125] = 140;
  
  
      // 广播
      pub.publish(map);
      loop_rate.sleep(); 

      if (q_lists_gpu.size()==0) std::cout<<"NO PATH IS FOUND" <<std::endl;
            


    }
    
    ros::Rate loop_rate(5);
    pub.publish(map);
    loop_rate.sleep(); 

  }


  


  return 0;
}