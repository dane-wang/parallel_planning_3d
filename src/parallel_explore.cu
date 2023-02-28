

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
__global__ void explore(T* q,  planner::Node* graph, T* new_q, int q_size, int n  )
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


__global__ void warmup(int* a)
{
    a = a+1;
}


extern "C"
void parallel_explore(planner::Node* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path_to_goal){

  //Setup everything for planning
  // graph[start_index].g = 0;
  // graph[start_index].h = h_calculation(&graph[start_index], &graph[goal_index]);
  // graph[start_index].f = graph[start_index].g + graph[start_index].h;
  bool path_found = false;
  int goal = goal_index;
  thrust::host_vector<int> q_lists;
  q_lists.push_back(start_index);

  const int map_size = n*n*n*sizeof(planner::Node);

  planner::Node *map_gpu;

  int neighbors[][3] = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, {1, 0, 1}, {1, 0, -1}, {1, 1, 0}, {1, -1, 0}, {-1, 0, 1} , {-1, 0, -1} , {-1, 1, 0} , {-1, -1, 0} , {1, 1, 1} , {1, 1, -1} , {1, -1, 1} , {1, -1, -1} , {-1, -1, -1} , {-1, -1, 1} , {-1, 1, -1} , {-1, 1, 1}   };

  int neighbor[26*3];
  for (int i =0; i< 26; i++){
    for (int j=0; j<3; j++){

      neighbor[3*i+j] = neighbors[i][j];

    }

  }
  //Copy all needed variables to gpu
  cudaMalloc( (void**)&map_gpu, map_size );
  cudaMemcpy(map_gpu, graph, map_size, cudaMemcpyHostToDevice);

  cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  3*26*sizeof(int));
  cudaMemcpyToSymbol(goal_gpu, &goal,  sizeof(int));

  //q list on gpu
  thrust::device_vector<int> q_lists_gpu = q_lists;

  while(q_lists_gpu.size()!=0 && !path_found){
    int q_size = q_lists_gpu.size();

    //Determine how many thread should be launched
    int thread_size_needed = min(max_thread, q_size);
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
    // cudaMemcpy(&graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );


    // Remove all element that is not used during the exploration and repeated value
    
    new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
    thrust::sort(new_q_lists_gpu.begin(), new_q_lists_gpu.end());
    new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );
    
    
    // Create new q list based on origional and updated q
    if (q_size <= max_thread) {
      q_lists_gpu.clear();
      q_lists_gpu = new_q_lists_gpu;
      new_q_lists_gpu.clear();
    }
    else {
      
      q_lists_gpu.erase(q_lists_gpu.begin(), q_lists_gpu.begin()+max_thread );
      q_lists_gpu.insert(q_lists_gpu.end(), new_q_lists_gpu.begin(), new_q_lists_gpu.end() );
      thrust::sort(q_lists_gpu.begin(), q_lists_gpu.end());
      q_lists_gpu.erase(thrust::unique(q_lists_gpu.begin(), q_lists_gpu.end()), q_lists_gpu.end() );
    
      new_q_lists_gpu.clear();

      // //sort the q_list based on the f value
      thrust::device_vector<float> f_value(q_lists_gpu.size());
      get_f<<<1, q_lists_gpu.size()>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(f_value.data()), q_lists_gpu.size() );
      cudaDeviceSynchronize();
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
      cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );
      int path1 = goal;
      while (path1 != start_index)
        {  
          path_to_goal.push_back(path1);
          graph[path1].path = true;
          // path.push_back(path1);
          path1 = graph[path1].parent;
        }
      // cudaFree(map_gpu);


    }
  }

  if (q_lists_gpu.size()==0) std::cout<< "NO PATH IS FOUND" <<std::endl;        
    

 
}

extern "C"
void gpu_warmup() {

    //GPU warm up
    int a = 0;
    int* a_gpu;
    cudaMalloc( (void**)&a_gpu, sizeof(int) );
    cudaMemcpy(a_gpu, &a, sizeof(int), cudaMemcpyHostToDevice);
    warmup<<<1,1>>>(a_gpu);
    cudaDeviceSynchronize();
}
