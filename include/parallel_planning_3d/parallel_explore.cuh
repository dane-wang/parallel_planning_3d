#ifndef PARALLEL_H
#define PARALLEL_H
#include <time.h>  
#include <vector>
#include "parallel_planning_3d/planner.h"
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

// #ifdef __cplusplus
//   extern "C"
// #endif
extern "C" 
void parallel_explore(planner::Node* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path);

extern "C"
void gpu_warmup();

extern "C" 
void parallel_bi_explore(planner::BiNode* graph, int n, int start_index, int goal_index, int max_thread, std::vector<int>& path);


#endif