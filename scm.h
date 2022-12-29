// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================
// 
// Declaration of two scan matching algorithms for ego vehicle localization
// using Lidar point clouds:
// 1. Normal Distributions Transform (NDT)
// 2. Iterative Closest Point (ICP)

#ifndef SCM_H
#define SCM_H

#include <Eigen/Geometry>
#include <iostream>     // std::cout, std::fixed
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include <vector>

#include "helper.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


// Normal Distributions Transform (NDT) Scan Matching Algorithm
Eigen::Matrix4d scanMatchingByNDT(
	PointCloudT::Ptr target, PointCloudT::Ptr source, Pose initialPose,
	double epsilon, int maxIter);

// Iterative Closest Point (ICP) Algorithm
Eigen::Matrix4d scanMatchingByICP(
	PointCloudT::Ptr target, PointCloudT::Ptr source, Pose initialPose,
	double epsilon, int maxIter);

#endif  // SCM_H
