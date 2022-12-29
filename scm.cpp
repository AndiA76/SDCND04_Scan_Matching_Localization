// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================
// 
// Implementation of two scan matching algorithms for ego vehicle localization
// using Lidar point clouds:
// 1. Normal Distributions Transform (NDT)
// 2. Iterative Closest Point (ICP)

#include "scm.h"


// Normal Distributions Transform (NDT) Scan Matching Algorithm
Eigen::Matrix4d scanMatchingByNDT(
	PointCloudT::Ptr target, PointCloudT::Ptr source, Pose initialPose,
	double epsilon = 1e-6, int maxIter = 60) {

	// Set timer to measure processing time
	pcl::console::TicToc time;
	time.tic();

	// Initialize final transformation matrix (rotation and translation) as identiy matrix
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

	// Align source point cloud with initial pose to obtain the initial guess to start NDT iterations
	Eigen::Matrix4f initialGuess = transform3D(
		initialPose.rotation.yaw, initialPose.rotation.pitch, initialPose.rotation.roll,
		initialPose.position.x, initialPose.position.y, initialPose.position.z
	).cast<float>();

	// Initialize Normal Distributions Transform (NDT)
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;

	// Set scale dependend NDT parameters
	// Set minimum transformation difference for termination conditions
	ndt.setTransformationEpsilon(epsilon);
	// Set maximum step size for More-Thuente line search
	ndt.setStepSize(0.1);  // 0.01; 0.1; 0.5; 1.0;
	// Set resolution of NDT grid structure (VoxelGridCovariance)
	ndt.setResolution(1.0);  // 0.1; 1.0; 2.0;
	// Set maximum number of registration iterations
	ndt.setMaximumIterations(maxIter);

	// Set point cloud to be aligned
	ndt.setInputSource(source);

	// Set target point cloud to align the source cloud to
	ndt.setInputTarget(target);

	// Initialize output point cloud after applying NDT
	PointCloudT::Ptr outputCloud (new PointCloudT);
	// Apply NDT to get the final transform - starting iterations with the initial transform from above
	ndt.align(*outputCloud, initialGuess);
	
	// Check for convergence of the NDT algorithm
	if (ndt.hasConverged()) {
		// Get the final transformation matrix cast to double precision
		transformationMatrix = ndt.getFinalTransformation().cast<double>();

		// Show convergence, fitness score and processing time
		std::cout << "NDT has convereged in " << time.toc() <<
			" ms with a fitness score of " << ndt.getFitnessScore() << std::endl;
		std::cout << "Returning final NDT transformation matrix!" << std::endl;
	} else {
		std::cout << "WARNING: NDT did not converge!" << std::endl;
		std::cout << "Returning identity matrix!" << std::endl;
	}

	// Return the final transformation matrix obtained by NDT
	return transformationMatrix;
}

// Iterative Closest Point (ICP) Algorithm
Eigen::Matrix4d scanMatchingByICP(
	PointCloudT::Ptr target, PointCloudT::Ptr source, Pose initialPose,
	double epsilon = 1e-4, int maxIter = 16) {
	
	// Set timer to measure processing time
	pcl::console::TicToc time;
	time.tic();

	// Initialize final transformation matrix (rotation and translation) as identiy matrix
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();

	// Align source with initial pose to obtain the initial transform to transform the source point cloud
	Eigen::Matrix4d initialTransform = transform3D(
		initialPose.rotation.yaw, initialPose.rotation.pitch, initialPose.rotation.roll,
		initialPose.position.x, initialPose.position.y, initialPose.position.z
	);

	// Initialize transformed source point cloud as input point cloud to ICP
	PointCloudT::Ptr transformedSource (new PointCloudT);
	pcl::transformPointCloud (*source, *transformedSource, initialTransform);

	// Initialize Iterative Clostes Point (ICP)
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	// Set scale dependend NDT parameters
	// Set minimum transformation difference for termination conditions
	icp.setTransformationEpsilon(epsilon);
	// Set maximal corresponding distance
	icp.setMaxCorrespondenceDistance(2.0);  // 0.001; 0.01; 0.1; 0.5; 1.0; 2.0;
	// Set RANSAC outlier rejection threshold
	icp.setRANSACOutlierRejectionThreshold(10);
	// Set maximum number of registration iterations
	icp.setMaximumIterations(maxIter);

	// Set point cloud to be aligned
	icp.setInputSource(transformedSource);

	// Set target point cloud to align the source cloud to
	icp.setInputTarget(target);	

	// Initialize output point cloud after applying ICP
	PointCloudT::Ptr outputCloud (new PointCloudT);
	// Apply ICP to align the output point cloud with the transformed source point cloud
	icp.align(*outputCloud);
	
	// Check for convergence of the ICP algorithm
	if (icp.hasConverged()) {
		// Get the final transformation matrix cast to double precision
		transformationMatrix = icp.getFinalTransformation().cast<double>();
		// Apply initial transform to final transform
		transformationMatrix = (transformationMatrix * initialTransform).cast<double>();

		// Show convergence, fitness score and processing time
		std::cout << "ICP has convereged in " << time.toc() << 
			" ms with a fitness score of " << icp.getFitnessScore() << std::endl;
		std::cout << "Returning final ICP transformation matrix!" << std::endl;
	} else {
		std::cout << "WARNING: ICP did not converge!" << std::endl;
		std::cout << "Returning identity matrix!" << std::endl;
	}

	// Return the final transformation matrix obtained by ICP
	return transformationMatrix;
}
