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



// __device__ bool path_found_gpu;
__device__ int neighbor_gpu[3*26];
// __device__ int g_gpu;

struct is_negative
{
  __host__ __device__
  bool operator()(int x)
  {
    return x ==-1;
  }
};



template <typename T>
__global__ void dijkstra_explore(T* q,  planner::Node* graph, T* new_q , int q_size, int resolution_size, int n )
{
  int tid = blockIdx.x *blockDim.x + threadIdx.x;
  // printf("Q size %d \n", q_size);

  if (tid<q_size){
    int explored_index = q[tid];
    // graph[explored_index].explored = true;
  
    int explored_coord[3];
    explored_coord[2] = explored_index/(n*n);

    int a = explored_index%(n*n);

    explored_coord[0] = a%n;
    explored_coord[1] = a/n;

    

    for (int i=-(resolution_size-1)/2; i<(resolution_size-1)/2+1; i++){
      for(int j=-(resolution_size-1)/2; j<(resolution_size-1)/2+1; j++ ){
          for(int k=-(resolution_size-1)/2; k<(resolution_size-1)/2+1; k++ ){

              if ((explored_coord[0]+i < n) && (explored_coord[0]+i >= 0)  && (explored_coord[1]+j < n) && (explored_coord[1]+j >= 0) && (explored_coord[2]+k < n) && (explored_coord[2]+k >= 0)){

                          // printf("Hello");
                          int sur_coord[3] = {0, 0, 0};
                          sur_coord[0] = explored_coord[0]+i; 
                          sur_coord[1] = explored_coord[1]+j; 
                          sur_coord[2] = explored_coord[2]+k; 
                          int sur_index = sur_coord[0] + sur_coord[1]*n + sur_coord[2]*n*n;

                          graph[sur_index].h = graph[explored_index].h;

              }
          }
        }
      }



    for (int i=0; i<26; i++)
    {   
      
      // int new_index = explored_index + resolution_size * neighbor_gpu[i];
      // thrust::device_vector<int> new_coord = planner::indextocoord_gpu(new_index, n);

      int neighbor[3];
      neighbor[0] = neighbor_gpu[3*i];
      neighbor[1] = neighbor_gpu[3*i+1];
      neighbor[2] = neighbor_gpu[3*i+2];

      
      // printf("Checking %d, %d, %d\n", (int) neighbor[0], (int) neighbor[1], (int) neighbor[2]);


      int new_coord[3];
      new_coord[0] = explored_coord[0] + neighbor[0] * resolution_size ;
      new_coord[1] = explored_coord[1] + neighbor[1] * resolution_size;
      new_coord[2] = explored_coord[2] + neighbor[2] * resolution_size;

      // printf("Checking %d, %d, %d\n", (int) new_coord[0], (int) new_coord[1], (int) new_coord[2]);

      int new_index = new_coord[0] + new_coord[1]*n + new_coord[2]*n*n;
      

      float cost = 5.0;
          
      // if (i<6){
      //   cost = 1;
      // }
      // else if (i<18)
      // {
      //   cost = sqrt(2.0);
      // }
      // else {
      //   cost = sqrt(3.0);
      // }

      if ((new_coord[0] >= n) || (new_coord[0] < 0)  || (new_coord[1] >= n) || (new_coord[1] <0 ) || (new_coord[2] >= n) || (new_coord[2] < 0)){

        //we know it is not a valid point

        for (int i=-(resolution_size-1)/2; i<(resolution_size-1)/2+1; i++)
        {
          for(int j=-(resolution_size-1)/2; j<(resolution_size-1)/2+1; j++ )
          {
            for(int k=-(resolution_size-1)/2; k<(resolution_size-1)/2+1; k++ )
            {

              if ((new_coord[0]+i < n) && (new_coord[0]+i >= 0)  && (new_coord[1]+j < n) && (new_coord[1]+j >= 0) && (new_coord[2]+k < n) && (new_coord[2]+k >= 0)){

                          // thrust::device_vector<int> new_coord1(3);

                          // printf("Hello");
                          int new_coord1[3] = {0, 0, 0};
                          new_coord1[0] = new_coord[0]+i; 
                          new_coord1[1] = new_coord[1]+j; 
                          new_coord1[2] = new_coord[2]+k; 
                          int new_index1 = new_coord1[0] + new_coord1[1]*n + new_coord1[2]*n*n;

                          graph[new_index1].h = graph[explored_index].h + cost;

                      }
            }
          }
        }
        // printf("Wrong");
        continue;
        

      }
      else if (graph[new_index].obstacle == false && graph[new_index].h == INFINITY)
      {
        graph[new_index].h = graph[explored_index].h + cost;
        // graph[new_index].frontier = true;
        // printf("assinging");
        // graph[new_index].parent = explored_index;
        // graph[new_index].frontier = true;

        // if (q_size>800) printf("Still working %d/n", new_index);
        
        new_q[26*tid+i] = new_index;
      }
      else if (graph[new_index].obstacle == false && (graph[new_index].h != INFINITY))
      {
        if (graph[new_index].h > graph[explored_index].h + cost)
        {
          // printf("assinging1");
          graph[new_index].h = graph[explored_index].h + cost;
          new_q[26*tid+i] = new_index;
        }
      }
    }
  }

  
}






extern "C" 
void parallel_dijkstra(planner::Node* graph, int n, int goal, int resolution_size=1){

  // Use dijkstra to initialize the heuristic values for all the node.


  //The heuristic for goal is 0
  // graph[goal_index].h = 0;
  graph[goal].g = 0;
  graph[goal].h = 0;
  graph[goal].f = graph[goal].g + graph[goal].h;
  
  
  // bool path_found = false;
  // int goal = goal_index;

  thrust::host_vector<int> q_lists;
  q_lists.push_back(goal);

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

  // cudaMemcpyToSymbol(path_found_gpu, &path_found,  sizeof(bool));
  cudaMemcpyToSymbol(neighbor_gpu, &neighbor,  3*26*sizeof(int));
//   cudaMemcpyToSymbol(g_gpu, &g,  sizeof(int));

  //q list on gpu
  thrust::device_vector<int> q_lists_gpu = q_lists;

  while (q_lists_gpu.size()!=0)
  {
  
  
  // for(int x =0; x<3; x++){


    int q_size = q_lists_gpu.size();
    std::cout << "Q size " << q_lists_gpu.size() << std::endl;



    //new_q is the list store the frontier generated from this step of exploration
   

  
    int block_size, thread_size;

    if (q_size <=1024){
      block_size = 1;
      thread_size = q_size;
    }
    else{
      block_size = (q_size/1024) + 1;
      thread_size = 1024;
    }

    thrust::device_vector<int> new_q_lists_gpu(26*q_size);
    thrust::fill(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), -1);


    
 

    //Launch the kernel to explore the map
    dijkstra_explore<<<block_size,thread_size>>>(thrust::raw_pointer_cast(q_lists_gpu.data()),  map_gpu, thrust::raw_pointer_cast(new_q_lists_gpu.data()), q_size ,resolution_size, n);
    cudaDeviceSynchronize();
    

    

    // // Remove all element that is not used during the exploration and repeated value

    
    new_q_lists_gpu.erase(thrust::remove_if(new_q_lists_gpu.begin(), new_q_lists_gpu.end(), is_negative()),  new_q_lists_gpu.end() );
    thrust::sort(new_q_lists_gpu.begin(), new_q_lists_gpu.end());
    new_q_lists_gpu.erase(thrust::unique(new_q_lists_gpu.begin(), new_q_lists_gpu.end()), new_q_lists_gpu.end() );

    // std::cout<< new_q_lists_gpu.size() << std::endl;

    q_lists_gpu.clear();
 
    q_lists_gpu = new_q_lists_gpu;
    new_q_lists_gpu.clear();

    

    
  
    

  }

  cudaMemcpy(graph, map_gpu,  map_size, cudaMemcpyDeviceToHost );

    





}