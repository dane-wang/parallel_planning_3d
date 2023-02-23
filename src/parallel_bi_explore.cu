#include <time.h>  
#include <vector>
#include "parallel_planning_3d/planner.h"
#include "parallel_planning_3d/parallel_explore.cuh"
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


__device__ bool path_found_gpu;
__device__ int neighbor_gpu[26];
__device__ int goal_gpu;
__device__ int start_gpu;


struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



template <typename T, typename T1> 
__global__ void get_f(T* q,  planner::BiNode* graph, T1* h, int q_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < q_size){
    int node = q[tid];

    h[tid] = graph[node].f;

    // printf("%d", q[tid]);
  }

}

template <typename T, typename T1> 
__global__ void get_b_f(T* b_q,  planner::BiNode* graph, T1* h, int q_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < q_size){
    int node = b_q[tid];

    h[tid] = graph[node].b_f;

    // printf("%d", q[tid]);
  }

}

template <typename T, typename T1> 
__global__ void get_total_g(T* meeting_node,  planner::BiNode* graph, T1* h, int meeting_size )
{

  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  if (tid < meeting_size){
    int node = meeting_node[tid];

    h[tid] = graph[node].b_g + graph[node].g ;

    // printf("%d", q[tid]);
  }

}




template <typename T>
__global__ void explore(planner::BiNode* graph, T* q,  T* new_q, T* b_q, T* new_b_q, T* meeting_nodes, int q_thread, int b_q_thread  )
{
  int tid = blockIdx.x *blockDim.x + threadIdx.x;

  if (tid<q_thread) {

    int explored_index = q[tid];
    int n = neighbor_gpu[2];

    int floor_index = explored_index%(n*n);
    int vertical_index = explored_index/(n*n);
    int row_index = floor_index / n;

    graph[explored_index].explored = true;
    graph[explored_index].frontier = false;

    if (graph[explored_index].b_explored){
    //   printf("FOUND");
      // printf("Hello from thread %d, I am exploring %d\n", tid, explored_index);
      // planner::Node* temp_node = graph[explored_index].parent;
      // while (!temp_node->start){
        
      //     temp_node->path = true;
      //     temp_node = temp_node->parent;
      // }
      meeting_nodes[tid] = explored_index;
      path_found_gpu = true;
    }

    if (!path_found_gpu){
      for (int i=0; i<26; i++)
      {   
        
        
        int new_index = explored_index + neighbor_gpu[i];
        
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
  else if (tid<(q_thread+b_q_thread))
  {
    int explored_index = b_q[tid-q_thread];
    int n = neighbor_gpu[2];

    int floor_index = explored_index%(n*n);
    int vertical_index = explored_index/(n*n);
    int row_index = floor_index / n;

    graph[explored_index].b_explored = true;
    graph[explored_index].b_frontier = false;

    if (graph[explored_index].explored){
    //   printf("FOUND");
      // printf("Hello from thread %d, I am exploring %d\n", tid, explored_index);
      // planner::Node* temp_node = graph[explored_index].parent;
      // while (!temp_node->start){
        
      //     temp_node->path = true;
      //     temp_node = temp_node->parent;
      // }
      meeting_nodes[tid] = explored_index;
      path_found_gpu = true;
    }

    if (!path_found_gpu){
      for (int i=0; i<26; i++)
      {   
        
        
        int new_index = explored_index + neighbor_gpu[i];
        
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

        
                  
        bool left_edge_out = (floor_index%n ==0) && (i==1|| i==7 || i==9 || i==11 || i==15 || i==19 || i==21 || i==22 || i==24);

        bool right_edge_out = ((floor_index+1)%n ==0) && (i==0 || i==6 || i==8 || i==10 || i==14 || i==18 || i==20 || i==23 || i==25);

        bool front_edge_out = ((row_index+1)%n ==0) && (i==2 || i==6 || i==7 || i==12 || i==16 || i==18 || i==19 || i==24 || i==25);

        bool back_edge_out = (row_index == 0) && (i==3 || i==8 || i==9 || i==13 || i==17 || i==20 || i==21 || i==22 || i==23);

        bool top_edge_out = ((vertical_index+1)%n ==0) && (i==4 || i==10 || i==11 || i==12 || i==13 || i==20 || i==21 || i==18 || i==19);

        bool bot_edge_out = (vertical_index == 0) && (i==5 || i==14 || i==15 || i==16 || i==17 || i==24 || i==25 || i==22 || i==23);

        if (left_edge_out || right_edge_out || front_edge_out || back_edge_out || top_edge_out || bot_edge_out){
            edge_detect = false;
        }

        


        if (graph[new_index].obstacle == false && graph[new_index].b_frontier == false && graph[new_index].b_explored == false && edge_detect)
        {
          graph[new_index].b_g = graph[explored_index].b_g + cost;
            
          float h_1 = sqrt(pow((graph[new_index].x-graph[start_gpu].x),2) + pow((graph[new_index].y-graph[start_gpu].y),2) + pow((graph[new_index].z-graph[start_gpu].z),2) );
            // printf("%f", h_1);
          graph[new_index].b_h = h_1;

            
          graph[new_index].b_f = graph[new_index].b_h + graph[new_index].b_g;
          graph[new_index].b_parent = explored_index;
          graph[new_index].b_frontier = true;
          
          new_b_q[26*(tid-q_thread)+i] = new_index;
        }
        else if (edge_detect && graph[new_index].obstacle == false && (graph[new_index].b_frontier == true || graph[new_index].b_explored == true))
        {
          if (graph[new_index].b_g > graph[explored_index].b_g + cost)
          {
            graph[new_index].b_g = graph[explored_index].b_g + cost;
            graph[new_index].b_f = graph[new_index].b_h + graph[new_index].b_g;
            graph[new_index].b_parent = explored_index;
          }
        }
      }
    }

  }
  

}








extern "C"
void parallel_bi_explore(planner::BiNode* graph, int n, int start, int goal, int max_thread_size, std::vector<int>& path){

    bool path_found = false;

    thrust::host_vector<int> q_lists;
    thrust::host_vector<int> b_q_lists;
    q_lists.push_back(start);
    b_q_lists.push_back(goal);

    const int map_size = n*n*n*sizeof(planner::BiNode);

    planner::BiNode *map_gpu;

    int neighbor[26] = {1, -1, n, -n, n*n, -n*n, n+1, n-1, -n+1, -n-1, n*n+1, n*n-1, n*n+n, n*n-n, -n*n+1, -n*n-1, -n*n+n, -n*n-n, n*n + n + 1, n*n + n- 1,  n*n - n + 1, n*n - n -1, -(n*n + n + 1), -(n*n + n- 1), -(n*n - n + 1), -(n*n - n -1) };

    cudaMalloc( (void**)&map_gpu, map_size );
    cudaMemcpy(map_gpu, graph, map_size, cudaMemcpyHostToDevice);

    cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
    cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  26*sizeof(int));
    cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));
    cudaMemcpyToSymbol(start_gpu, &start,  sizeof(int));


    thrust::device_vector<int> q_lists_gpu = q_lists;
    thrust::device_vector<int> b_q_lists_gpu = b_q_lists;

   
    while( q_lists_gpu.size()!=0 && b_q_lists_gpu.size()!=0  && !path_found){

        int q_size = q_lists_gpu.size();
        int b_q_size = b_q_lists_gpu.size();
        // std::cout << "q size is" << q_size << std::endl;
        // std::cout << "b q size is" << b_q_size << std::endl;

        
        

        //Determine how many thread should be launched
        int thread_size_needed = min(max_thread_size, (q_size+b_q_size));
        int block_size, thread_size;
        int q_thread, b_q_thread;

        if (thread_size_needed <=1024){
            block_size = 1;
            thread_size = thread_size_needed;
        }
        else{
            block_size = (thread_size_needed/1024) + 1;
            thread_size = 1024;
        }

        //new_q is the list store the frontier generated from this step of exploration
        if ((q_size+b_q_size)<=max_thread_size){

            q_thread = q_size;
            b_q_thread = b_q_size;

        }
        else{

            float ratio = q_size / (q_size+b_q_size);
            q_thread = (int) (ratio * max_thread_size);
            q_thread = max(q_thread, 300);
            b_q_thread = max_thread_size - q_thread;

            if (b_q_thread > b_q_size) {
                b_q_thread = b_q_size;
                q_thread = max_thread_size - b_q_thread;
            }


        }

        // std::cout << "q size is" << q_size << std::endl;
        // std::cout << "b q size is" << b_q_size << std::endl;

        // std::cout << "q thread is" << q_thread << std::endl;
        // std::cout << "b q thread is" << b_q_thread << std::endl;

        thrust::device_vector<int> new_q_lists_gpu(q_thread*26);
        thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);

        thrust::device_vector<int> new_b_q_lists_gpu(b_q_thread*26);
        thrust::fill(new_b_q_lists_gpu.begin(), new_b_q_lists_gpu.end(), -1);

        thrust::device_vector<int> meeting_nodes(q_thread + b_q_thread);
        thrust::fill(meeting_nodes.begin(), meeting_nodes.end(), -1);




        //Launch the kernel to explore the map
        explore<<<block_size,thread_size>>>(map_gpu, thrust::raw_pointer_cast(q_lists_gpu.data()), thrust::raw_pointer_cast(new_q_lists_gpu.data()),  thrust::raw_pointer_cast(b_q_lists_gpu.data()), thrust::raw_pointer_cast(new_b_q_lists_gpu.data()), thrust::raw_pointer_cast(meeting_nodes.data()), q_thread, b_q_thread);
        cudaDeviceSynchronize();
        cudaMemcpyFromSymbol(&path_found, path_found_gpu,  sizeof(bool), 0, cudaMemcpyDeviceToHost );

        
        // cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );


        // Remove all element that is not used during the exploration and repeated value
        
        new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
        thrust::sort(new_q_lists_gpu.begin(), new_q_lists_gpu.end());
        new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );

        new_b_q_lists_gpu.erase(thrust::remove_if(new_b_q_lists_gpu.begin(), new_b_q_lists_gpu.end(), is_negative()),  new_b_q_lists_gpu.end() );
        thrust::sort(new_b_q_lists_gpu.begin(), new_b_q_lists_gpu.end());
        new_b_q_lists_gpu.erase(thrust::unique(new_b_q_lists_gpu.begin(), new_b_q_lists_gpu.end()), new_b_q_lists_gpu.end() );


        // std::cout << "new q size is" << new_q_lists_gpu.size() << std::endl;
        
        // Create new q list based on origional and updated q
        if (q_size <= q_thread) {
            q_lists_gpu.clear();
            q_lists_gpu = new_q_lists_gpu;
            new_q_lists_gpu.clear();
        }
        else {
            
            q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+q_thread );
            q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
            thrust::sort(q_lists_gpu.begin(), q_lists_gpu.end());
            q_lists_gpu.erase(thrust::unique(q_lists_gpu.begin(), q_lists_gpu.end()), q_lists_gpu.end() );
            new_q_lists_gpu.clear();

            // //sort the q_list based on the f value
            thrust::device_vector<float> f_value(q_lists_gpu.size());
            get_f<<<1, q_lists_gpu.size()>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(f_value.data()), q_lists_gpu.size() );
            thrust::sort_by_key(f_value.begin(), f_value.end(), q_lists_gpu.begin() );
            // thrust::reverse(thrust::device, q_lists_gpu.begin(), q_lists_gpu.end());
            
        }

        if (b_q_size <= b_q_thread) {
            b_q_lists_gpu.clear();
            b_q_lists_gpu = new_b_q_lists_gpu;
            new_b_q_lists_gpu.clear();
        }
        else {
            
            b_q_lists_gpu.erase(b_q_lists_gpu.begin(), b_q_lists_gpu.begin()+b_q_thread );
            b_q_lists_gpu.insert(b_q_lists_gpu.end(), new_b_q_lists_gpu.begin(), new_b_q_lists_gpu.end() );
            thrust::sort(b_q_lists_gpu.begin(), b_q_lists_gpu.end());
            b_q_lists_gpu.erase(thrust::unique(b_q_lists_gpu.begin(), b_q_lists_gpu.end()), b_q_lists_gpu.end() );
            new_b_q_lists_gpu.clear();

            // //sort the q_list based on the f value
            thrust::device_vector<float> b_f_value(b_q_lists_gpu.size());
            get_b_f<<<1, b_q_lists_gpu.size()>>>(thrust::raw_pointer_cast(b_q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(b_f_value.data()), b_q_lists_gpu.size() );
            thrust::sort_by_key(b_f_value.begin(), b_f_value.end(), b_q_lists_gpu.begin() );
            // thrust::reverse(thrust::device, b_q_lists_gpu.begin(), b_q_lists_gpu.end());
            
        }

        
        //q_size = q_lists_gpu.size();
        // thrust::device_vector<float> h_value(q_size);

        // if (q_size > 1024) {
        //   int block = q_size / 1024 + 1;
            
        //   get_h<<<block, 1024>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(&h_value[0]), q_size );

        //   thrust::sort_by_key(h_value.begin(), h_value.end(), q_lists_gpu.begin() );

        // }

        if (path_found){

            cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );

            std::cout<< "checking for path" << std::endl;

            // std::cout<< "Possible middle point number before remove" << meeting_nodes.size() << std::endl;
            meeting_nodes.erase(thrust::remove_if(meeting_nodes.begin(), meeting_nodes.end(), is_negative()),  meeting_nodes.end() );
            thrust::sort(meeting_nodes.begin(), meeting_nodes.end());
            meeting_nodes.erase(thrust::unique(meeting_nodes.begin(), meeting_nodes.end()), meeting_nodes.end() );

            
            // std::cout<< "Possible middle point number" << meeting_nodes.size() << std::endl;
            
            thrust::device_vector<float> total_g_value(meeting_nodes.size());
            get_total_g<<<1, meeting_nodes.size()>>>(thrust::raw_pointer_cast(meeting_nodes.data()),  map_gpu, thrust::raw_pointer_cast(total_g_value.data()), meeting_nodes.size() );

            auto it = std::min_element(std::begin(total_g_value), std::end(total_g_value));

            int middle_point = meeting_nodes[std::distance(std::begin(total_g_value), it)];

            int path1 = middle_point;
            int path2 = graph[middle_point].b_parent;

            std::vector<int> actual_path1, actual_path2;

            while (path1 != start)
            {
                actual_path1.push_back(path1);
                graph[path1].path = true;
                path1 = graph[path1].parent;
            }

            while (path2 != goal)
            {
                actual_path2.push_back(path2);
                graph[path2].path = true;
                path2 = graph[path2].b_parent;
            }
            actual_path2.push_back(goal);

            std::reverse(actual_path2.begin(), actual_path2.end());
            actual_path2.insert(actual_path2.end(), actual_path1.begin(), actual_path1.end());


            path = actual_path2;
            
        
            
        }    
    }

    if (q_lists_gpu.size()==0 || b_q_lists_gpu.size()==0) std::cout<< "NO PATH IS FOUND" <<std::endl;        
    




};
    

