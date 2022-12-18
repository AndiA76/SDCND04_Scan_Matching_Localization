// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht using code base/skeleton provided by Udacity
//  Source:	    https://www.udacity.com/
// 
//  Copyright © 2012 - 2021, Udacity, Inc.
//  Copyright © 2022 Andreas Albrecht
// ============================================================================
// 
// Implementation of the main routine for the scan matching localization project.

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
#include "scm.h"
#include "ukf.h"

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
// Handle keyboard events
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer) {

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = 
	//	*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()) {
		cs.push_back(ControlState(0, -0.02, 0));
  	} else if (event.getKeySym() == "Left" && event.keyDown()) {
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()) {
		cs.push_back(ControlState(0.1, 0, 0));
  	} else if (event.getKeySym() == "Down" && event.keyDown()) {
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()) {
		refresh_view = true;
	}
}

// Actuate on control command
void Actuate(ControlState response, cc::Vehicle::Control& state) {

	if(response.t > 0) {
		if(!state.reverse) {
			state.throttle = min(state.throttle+response.t, 1.0f);
		} else {
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	} else if(response.t < 0) {
		response.t = -response.t;
		if(state.reverse) {
			state.throttle = min(state.throttle+response.t, 1.0f);
		} else {
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);
		}
	}
	state.steer = min(max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

// Draw 3D bounding box representing the car's current pose
void drawCar(Pose pose, int num, Color color, double alpha,
	pcl::visualization::PCLVisualizer::Ptr& viewer) {

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
	box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
	box.cube_length = 4;
	box.cube_width = 2;
	box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
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
	// Do not use Unscented Kalman Filter support by default
	bool useUKF = false;
	// Check number of input arguments
	if (arg_cnt==3) {
		std::cout << arg_vec[0] << std::endl;
		std::cout << arg_vec[1] << std::endl;
		std::cout << arg_vec[2] << std::endl;
		if (strcmp(arg_vec[1],"NDT")==0) {
			// Use NDT as scan matching algorithm if selected
			scmAlgoId = 0;
			scmAlgoName = "Normal Distributions Transform (NDT)";
		} else if (strcmp(arg_vec[1],"ICP")==0) {
			// Use ICP as scan matching algorithm if selected
			scmAlgoId = 1;
			scmAlgoName = "Iterative Closest Point (ICP)";
		} else {
			std::cout << "Error: Wrong input argument ... program stopped." << std::endl;
			std::cout << "Hint: Use either './cloud_loc ICP' or './cloud_loc NDT'." << std::endl;
			return 1;
		}
		std::cout << "Using " << scmAlgoName << " (user setting) for scan matching." << std::endl;
		if (strcmp(arg_vec[2],"UKF")==0) {
			useUKF = true;
		} else {
			std::cout << "Error: Wrong input argument ... program stopped." << std::endl;
			std::cout << "Hint: Use either './cloud_loc ICP UKF' or './cloud_loc NDT UKF'." << std::endl;
			return 1;
		}
		std::cout << "Using Unscented Kalman Filter to support tracking ego vehicle pose." << std::endl;
	} else if (arg_cnt==2) {
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
	// auto timestamp = world.GetSnapshot().GetTimestamp();
	auto world_snapshot = world.GetSnapshot();
	auto timestamp = world_snapshot.GetTimestamp();

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

	// Initialize initial pose of the ego vehicle
	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Create instance of an Unscented Kalman Filter for ego vehicle tracking
	UKF vehicle_ukf;

	// Initialize initial pose estimate of the Uncented Kalman Filter
	Pose pose_ukf(Point(0,0,0), Rotate(0,0,0));

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
					) > 8.0) {  // Don't include points touching ego
					pclCloud.points.push_back(
						PointT(detection.point.x, detection.point.y, detection.point.z)
					);
				}
			}
			if(pclCloud.points.size() > 5000) {  // CANDO: Modify this value to get different scan resolutions
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
			vehicle->GetTransform().rotation.yaw * PI/180,
			vehicle->GetTransform().rotation.pitch * PI/180,
			vehicle->GetTransform().rotation.roll * PI/180
		)
	);

	// Initialize maximum positioning error w.r.t. true pose of the ego vehicle
	double maxError = 0;
	double maxError_ukf = 0;

	// Start simulation loop
	while (!viewer->wasStopped()) {
		// Increment simulation time as long as new_scan == true
		while(new_scan) {
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}

		// Get actual simulation time from Carla world object
		world_snapshot = world.GetSnapshot();
		timestamp = world_snapshot.GetTimestamp();
		std::cout << "timestamp = " << timestamp << "." << std::endl;

		// Refresh camera position
		if(refresh_view) {
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
				vehicle->GetTransform().rotation.yaw * PI/180,
				vehicle->GetTransform().rotation.pitch * PI/180,
				vehicle->GetTransform().rotation.roll * PI/180
			)
		) - poseRef;
		// Draw a new 3D bounding box representing the current true pose of the ego vehicle
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		// Get the current yaw angle
		double theta = truePose.rotation.yaw;
		// Get the steering angle
		double stheta = control.steer * PI/4 + theta;
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
		
		if(!new_scan) {

			// Set new_scan flag
			new_scan = true;

			// Initialize pose with ground truth in the first step
			if (scan_cnt==0) {
				// Initialize ego vehicle pose
				pose.position = truePose.position;
				pose.rotation = truePose.rotation;

				// Initialize Unscented Kalman Filter for ego vehicle tracking
				if (useUKF) {
					vehicle_ukf.InitializeState(pose, timestamp.platform_timestamp);
				}
			}

			// Get the current pose estimate from the Unscented Kalman Filter
			if (useUKF) {
				pose_ukf = vehicle_ukf.GetPoseEstimate();
			}

			// Count the number of lidar scans
			scan_cnt++;

			// TODO: Filter scan using voxel filter
			// Declare variable for the voxel grid filter
			pcl::VoxelGrid<PointT> vgf;
			// Initialize voxel grid filter with latest scan cloud
			vgf.setInputCloud(scanCloud);
			// Set voxel grid filter resolution
			double filterResolution = 0.8; // 0.8; // 0.25; 0.5; 0.75; 0.8; 1.0; 2.0; 5.0;
			vgf.setLeafSize(filterResolution, filterResolution, filterResolution);
			// Set minimum number of points per voxel grid required for the grid cell to be used
			int minPointPerVoxel = 5;
			vgf.setMinimumPointsNumberPerVoxel(minPointPerVoxel);
			// Apply voxel grid filter on scanCloud and return the result in cloudFiltered
			vgf.filter(*cloudFiltered);

			// Get pose matching transformation matrix depending using NDT or ICP
			Eigen::Matrix4d matchingTransform;
			if (scmAlgoId==0) {  // NDT
				// Set maximum number of iterations
				int iter = 4; //25;  // 4; 5; 10; 20; 50; 60; 100;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-6;  // 1e-1; 1e-2; 1e-3; 1e-4; 1e-5; 1e-6; 1e-7;
				// Get final transformation matrix to match pose with measurements
				if (useUKF) {
					matchingTransform = scanMatchingByNDT(mapCloud, cloudFiltered, pose_ukf, epsilon, iter);
				} else {
					matchingTransform = scanMatchingByNDT(mapCloud, cloudFiltered, pose, epsilon, iter);
				}				
			} else {  // ICP
				// Set maximum number of iterations
				int iter = 16; // 4; 5; 10; 15; 20; 50;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-4;  // 1e-1; 1e-2; 1e-3; 1e-4; 1e-5; 1e-6; 1e-7;		
				// Get final transformation matrix to match pose with measurements
				if (useUKF) {
					matchingTransform = scanMatchingByICP(mapCloud, cloudFiltered, pose_ukf, epsilon, iter);
				} else {
					matchingTransform = scanMatchingByICP(mapCloud, cloudFiltered, pose, epsilon, iter);
				}
			}
			
			// TODO: Find pose transform by using ICP or NDT matching
			pose = getPose(matchingTransform);

			// Trigger Kalman Filter update cycle on the latest pose estimate obtained from scan matching
			if (useUKF) {
				vehicle_ukf.UpdateCycle(pose, timestamp.platform_timestamp);
			}

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

			// Update true pose error
			double poseError = sqrt(
				(truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + 
				(truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y)
			);
			if(poseError > maxError)
				maxError = poseError;
			// Update true driven distance
			double distDriven = sqrt(
				(truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y)
			);

			if (useUKF) {
				// Update Unscented Kalman Filter pose estimation error
				double poseError_ukf = sqrt(
					(truePose.position.x - pose_ukf.position.x) * (truePose.position.x - pose_ukf.position.x) + 
					(truePose.position.y - pose_ukf.position.y) * (truePose.position.y - pose_ukf.position.y)
				);
				if(poseError_ukf > maxError_ukf)
					maxError_ukf = poseError_ukf;

				// Update driven distance estimated by Unscented Kalman Filter
				double distDriven_ukf = sqrt(
					(pose_ukf.position.x) * (pose_ukf.position.x) + (pose_ukf.position.y) * (pose_ukf.position.y)
				);
				std::cout << "pose_ukf.position.x = " << pose_ukf.position.x << std::endl;
				std::cout << "pose_ukf.position.y = " << pose_ukf.position.y << std::endl;
				std::cout << "pose_ukf.position.z = " << pose_ukf.position.z << std::endl;
				std::cout << "poseError_ukf       = " << poseError_ukf << std::endl;
				std::cout << "maxError_ukf        = " << maxError_ukf << std::endl;
				std::cout << "distDriven_ukf      = " << distDriven_ukf << std::endl;
			
				// Show driven distance and (max.) pose error in the visualizer window
				viewer->removeShape("dist");
				viewer->addText(
					"Distance (UKF): "+to_string(distDriven)+" m ("+to_string(distDriven_ukf)+" m)", 100, 100, 24, 1.0, 1.0, 1.0, "dist",0
				);
				viewer->removeShape("derror");
				viewer->addText(
					"Pose error (UKF): "+to_string(poseError)+" m ("+to_string(poseError_ukf)+" m)", 100, 75, 24, 1.0, 1.0, 1.0, "derror",0
				);
				viewer->removeShape("maxE");
				viewer->addText(
					"Max Error (UKF): "+to_string(maxError)+" m ("+to_string(maxError_ukf)+" m)", 100, 50, 24, 1.0, 1.0, 1.0, "maxE",0
				);
			} else {
				// Show driven distance and (max.) pose error in the visualizer window
				viewer->removeShape("dist");
				viewer->addText(
					"Distance: "+to_string(distDriven)+" m", 100, 100, 24, 1.0, 1.0, 1.0, "dist",0
				);
				viewer->removeShape("derror");
				viewer->addText(
					"Pose error: "+to_string(poseError)+" m", 100, 75, 24, 1.0, 1.0, 1.0, "derror",0
				);
				viewer->removeShape("maxE");
				viewer->addText(
					"Max Error: "+to_string(maxError)+" m", 100, 50, 24, 1.0, 1.0, 1.0, "maxE",0
				);
			}

			// Show passed / failed test results in the visualizer window
			if(maxError > 1.2 || distDriven >= 170.0 ) {
				viewer->removeShape("eval");
			if(maxError > 1.2) {
				viewer->addText("Try Again", 100, 25, 24, 1.0, 0.0, 0.0, "eval",0);
			} else {
				viewer->addText("Passed!", 100, 25, 24, 0.0, 1.0, 0.0, "eval",0);
			}
		}

			pclCloud.points.clear();
		}
  	}

	return 0;
}
