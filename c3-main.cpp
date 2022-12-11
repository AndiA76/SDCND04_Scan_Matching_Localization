// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht using code base/skeleton provided by Udacity
//  Source:	    https://www.udacity.com/
// 
//  Copyright © 2012 - 2021, Udacity, Inc.
//  Copyright © 2022 Andreas Albrecht
// ============================================================================

#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <chrono>
#include <cmath>
#include <ctime>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

#include "helper.h"
#include "ukf.h"

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
// Handle keyboard events
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
	//	*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

// Actuate on control command
void Actuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min(max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

// Draw 3D bounding box representing the car's current pose
void drawCar(Pose pose, int num, Color color, double alpha,
	pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
	box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
	box.cube_length = 4;
	box.cube_width = 2;
	box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

// Normal Distributions Transform (NDT) Scan Matching Algorithm
Eigen::Matrix4d scanMatchingByNDT(
	PointCloudT::Ptr target, PointCloudT::Ptr source, Pose initialPose,
	double epsilon = 1e-6, int maxIter = 4) {

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
	double epsilon = 1e-6, int maxIter = 4){
	
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

int main(int arg_cnt, char * arg_vec[]) {
	/* Main function running the carla simulation.

	Args:
		arg_cnt (int):
			argument counter
		arg_vec (char*):
			pointer to argument vector
	
	Retuns:
		error flag (int):
			return 0 if main() has finished without errors, or 1 if not.
	*/
	// Use NDT as default scan matching algorithm
	int scmAlgoId = 0;
	std::string scmAlgoName = "Normal Distributions Transform (NDT)";
	if (arg_cnt==2) {
		std::cout << arg_vec[0] << std::endl;
		std::cout << arg_vec[1] << std::endl;
		if (strcmp(arg_vec[1],"NDT")==0) {
			// Use NDT as scan matching algorithm if selected
			scmAlgoId = 0;
			scmAlgoName = "Normal Distributions Transform (NDT)";
		} else if (strcmp(arg_vec[1],"ICP")==0) {
			// Use ICP as scan matching algorithm if selected
			scmAlgoId = 1;
			scmAlgoName = "Iterative Closest Point (ICP)";
		}
		std::cout << "Using " << scmAlgoName << " (user setting) for scan matching." << std::endl;
	} else if (arg_cnt==1) {
		std::cout << arg_vec[0] << std::endl;
		// Use NDT as default scan matching algorithm if no argument is passed as input
		std::cout << "Using " << scmAlgoName << " (default) for scan matching." << std::endl;
	} else {
		std::cout << "Error: Wrong number of input arguments .. program stopped." << std::endl;
		return 1;
	}

	// Init lidar scan counter
	int scan_cnt = 0;
	
	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	// Get simulation time from a snapshot of the current frame
	auto timestamp = world.get_snapshot().timestamp;

	// Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
	lidar_bp.SetAttribute("lower_fov", "-25");
	lidar_bp.SetAttribute("channels", "32");
	lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;  // continue lidar scanning if set to true
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	// Init point cloud visualization
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	// Init initial pose of the ego vehicle
	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Create instance of an Unscented Kalman Filter for ego vehicle tracking
	UKF vehicle_ukf;

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	// Declare variables for the filtered point cloud (using a voxel grid filter) and the scan point cloud
	typename PointCloudT::Ptr cloudFiltered (new PointCloudT);
	typename PointCloudT::Ptr scanCloud (new PointCloudT);

	// Initialize listener to capture cyclic lidar measurements
	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			for (auto detection : *scan){
				if(
					(
						detection.point.x*detection.point.x + detection.point.y*detection.point.y + 
						detection.point.z*detection.point.z
					) > 8.0){ // Don't include points touching ego
					pclCloud.points.push_back(
						PointT(detection.point.x, detection.point.y, detection.point.z)
					);
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}

	});
	
	// Initialize reference for the true starting pose of the ego vehicle
	Pose poseRef(
		Point(
			vehicle->GetTransform().location.x,
			vehicle->GetTransform().location.y,
			vehicle->GetTransform().location.z
		),
		Rotate(
			vehicle->GetTransform().rotation.yaw * pi/180,
			vehicle->GetTransform().rotation.pitch * pi/180,
			vehicle->GetTransform().rotation.roll * pi/180
		)
	);

	// Initialize maximum positioning error w.r.t. true pose of the ego vehicle
	double maxError = 0;

	// Start simulation loop
	while (!viewer->wasStopped())
  	{
		// Increment simulation time as long as new_scan == true
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}

		// Get actual simulation time from Carla world object
		timestamp = world.get_snapshot().timestamp;

		// Refresh camera position
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1,
				pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}

		// Remove 3D bounding box representing the previous true pose of the ego vehicle
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		// Measure true pose of the ego vehicle w.r.t. to the pose reference
		Pose truePose = Pose(
			Point(
				vehicle->GetTransform().location.x,
				vehicle->GetTransform().location.y,
				vehicle->GetTransform().location.z
			),
			Rotate(
				vehicle->GetTransform().rotation.yaw * pi/180,
				vehicle->GetTransform().rotation.pitch * pi/180,
				vehicle->GetTransform().rotation.roll * pi/180
			)
		) - poseRef;
		// Draw a new 3D bounding box representing the current true pose of the ego vehicle
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		// Get the current yaw angle
		double theta = truePose.rotation.yaw;
		// Get the steering angle
		double stheta = control.steer * pi/4 + theta;
		// Remove previous rendering shape of the steering direction
		viewer->removeShape("steer");
		// Render the actual steering direction using a short ray
		renderRay(
			viewer,
			Point(
				truePose.position.x+2*cos(theta),
				truePose.position.y+2*sin(theta),
				truePose.position.z
			),
			Point(
				truePose.position.x+4*cos(stheta),
				truePose.position.y+4*sin(stheta),
				truePose.position.z
			),
			"steer",
			Color(0,1,0)
		);

		ControlState actuate(0, 0, 1);
		if(cs.size() > 0){
			actuate = cs.back();
			cs.clear();

			Actuate(actuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		
		if(!new_scan){

			// Set new_scan flag
			new_scan = true;

			// Initialize pose with ground truth in the first step
			if (scan_cnt==0) {
				// Initialize ego vehicle pose
				pose.position = truePose.position;
				pose.rotation = truePose.rotation;

				// Initialize Unscented Kalman Filter for ego vehicle tracking
				vehicle_ukf.InitializeState(pose, timestamp)
			}

			// Get the pose estimate from the Unscented Kalman Filter
			pose_ukf = vehicle_ukf.GetPoseEstimate()
			
			// Count the number of lidar scans
			scan_cnt++;

			// TODO: Filter scan using voxel filter
			// Declare variable for the voxel grid filter
			pcl::VoxelGrid<PointT> vgf;
			// Initialize voxel grid filter with latest scan cloud
			vgf.setInputCloud(scanCloud);
			// Set voxel grid filter resolution
			double filterResolution = 0.8; // 0.25; 0.5; 0.75; 0.8; 1.0; 2.0; 5.0;
			vgf.setLeafSize(filterResolution, filterResolution, filterResolution);
			// Apply voxel grid filter on scanCloud and return the result in cloudFiltered
			vgf.filter(*cloudFiltered);

			// Get pose matching transformation matrix depending using NDT or ICP
			Eigen::Matrix4d matchingTransform;
			if (scmAlgoId==0){  // NDT
				// Set maximum number of iterations
				int iter = 4; //25;  // 5; 10; 20; 50; 60; 100;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-6;  // 1e-1; 1e-2; 1e-3; 1e-4; 1e-5; 1e-6; 1e-7;
				// Get final transformation matrix to match pose with measurements
				matchingTransform = scanMatchingByNDT(mapCloud, cloudFiltered, pose_ukf, epsilon, iter);
			}
			else {  // ICP
				// Set maximum number of iterations
				int iter = 4; // 5; 10; 15; 20; 50;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-6;  // 1e-1; 1e-2; 1e-3; 1e-4;				
				// Get final transformation matrix to match pose with measurements
				matchingTransform = scanMatchingByICP(mapCloud, cloudFiltered, pose_ukf, epsilon, iter);
			}
			
			// TODO: Find pose transform by using ICP or NDT matching
			pose = getPose(matchingTransform);

			// Trigger Kalman Filter update cycle on the latest pose estimate obtained from scan matching
    		vehicle_ukf.UpdateCycle(pose, timestamp);

			// TODO: Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr transformedScan (new PointCloudT);
			pcl::transformPointCloud (*cloudFiltered, *transformedScan, matchingTransform);

			// Hide scan point cloud
			viewer->removePointCloud("scan");

			// TODO: Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, transformedScan, "scan", Color(1,0,0) );

			// Remove all shapes and redraw
			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);

			// Update pose error
			double poseError = sqrt(
				(truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + 
				(truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y)
			);
			if(poseError > maxError)
				maxError = poseError;
			// Update driven distance
			double distDriven = sqrt(
				(truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y)
			);
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}

			pclCloud.points.clear();
		}
  	}

	return 0;
}
