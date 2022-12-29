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
#include <carla/geom/Vector3D.h>
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

// Geometric dimensions of the ego vehicle
constexpr double L_VEHICLE = 4.0;	// distance between front and rear axle
constexpr double W_VEHICLE = 2.0;	// vehicle width
constexpr double H_VEHICLE = 2.0;	// vehicle height


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
	box.cube_length = L_VEHICLE;  // 4
	box.cube_width = W_VEHICLE;   // 2
	box.cube_height = H_VEHICLE;  // 2
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
	std::cout << "arg_cnt = " << arg_cnt << std::endl;
	// Check number of input arguments
	if ((arg_cnt >= 2) && (arg_cnt <= 3)) {
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
			std::cout << "Hint: Use either 'ICP' or 'NDT' as first input argument." << std::endl;
			std::cout << "Example calls: './cloud_loc ICP (...)' or './cloud_loc NDT (...)'" << std::endl;
			return 1;
		}
		std::cout << "Using " << scmAlgoName << " (user setting) for scan matching." << std::endl;
		if (arg_cnt==3) {
			if (strcmp(arg_vec[2],"UKF")==0) {
				useUKF = true;
			} else {
				std::cout << "Error: Wrong input argument ... program stopped." << std::endl;
				std::cout << "Hint: Use 'UKF' as second input argument." << std::endl;
				std::cout << "Example call: './cloud_loc (...) UKF'" << std::endl;
				return 1;
			}
			std::cout << "Using Unscented Kalman Filter to support tracking 2D ego vehicle motion." << std::endl;
		}
	} else if (arg_cnt==1) {
		// Use NDT as default scan matching algorithm if no argument is passed as input
		std::cout << "Using " << scmAlgoName << " (default) for scan matching." << std::endl;
	} else {
		std::cout << "Error: Wrong number of input arguments .. program stopped." << std::endl;
		return 1;
	}

	// Declare simulation time and simulation time step
	double t, t_prev, delta_t;

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

	// Initialize current simulation time
	t = timestamp.platform_timestamp;

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

	// Initialize actor for the ego vehicle
	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);

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

	// Create instance of an Unscented Kalman Filter for 2D ego vehicle motion tracking
	UKF vehicle_ukf(L_VEHICLE/2, L_VEHICLE/2, PI/4);

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

	// Initialize true pose and predicted pose of the ego vehicle
	Pose truePose(Point(0,0,0), Rotate(0,0,0));
	Pose predPose(Point(0,0,0), Rotate(0,0,0));

	// Initialize yaw angle and orientation angle of the front wheels in global coordinates
	double theta = 0;
	double stheta = 0;

	// Initialize true and predicted steering angle of the front wheels relative to the ego vehicle axis
	double trueSteeringAngle = 0;
	double predSteeringAngle = 0;

	// Initialize true and predicted velocity of the ego vehicle
	double trueVelocity = 0;
	double predVelocity = 0;

	// Initialize (max./avg.) 2D position estimation error w.r.t. the true pose of the ego vehicle
	double posError = 0;
	double maxPosError = 0;
	EMA movAvgPosError;	// Init 1D moving average filter

	// Initialize driven distance at the current and the previous time step
	double distDriven = 0;
	double distDriven_prev = 0;

	// Initialize (max./avg.) 2D orientation estimation error
	double rotError = 0;
	double maxRotError = 0;
	EMA movAvgRotError;	// Init 1D moving average filter

	// Initialize (max./avg.) 2D velocity estimation error
	double velError = 0;
	double maxVelError = 0;
	double avgVelError = 0;
	EMA movAvgVelError;	// Init 1D moving average filter

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

		// Remember previous simulation time
		t_prev = t;
		// Update simulation time
		t = timestamp.platform_timestamp;
		// Calculate time difference
		delta_t = t - t_prev;
		std::cout << "t       = " << t << " s" << std::endl;
		std::cout << "t_prev  = " << t_prev << " s" << std::endl;
		std::cout << "delta_t = " << delta_t << " s" << std::endl;

		// Refresh camera position focusing on the predicted ego vehicle position
		if(refresh_view) {
			viewer->setCameraPosition(predPose.position.x, predPose.position.y, 60, predPose.position.x+1,
				predPose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}

		// Remove 3D bounding box representing the previous true pose of the ego vehicle
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		// Measure true pose of the ego vehicle w.r.t. to the pose reference
		truePose = Pose(
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
		// Measure the current 2D velocity of the ego vehicle
		// double trueVelocity = vehicle->GetVelocity().Length();  // 3D vector length
		trueVelocity = sqrt(
			(vehicle->GetVelocity().x * vehicle->GetVelocity().x) + 
			(vehicle->GetVelocity().y * vehicle->GetVelocity().y)
		);
		std::cout << "trueVelocity = " << trueVelocity << " m/s" << std::endl;
		// Get the current yaw angle of the ego vehicle in global cooridnates
		theta = truePose.rotation.yaw;
		// Measure the current steering angle of the front wheels
		trueSteeringAngle = control.steer * PI/4;
		std::cout << "trueSteeringAngle = " << trueSteeringAngle << " rad" << std::endl;
		// Get the current orientation angle of the front wheels in global coordinates
		stheta = trueSteeringAngle + theta;
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

		// Set initial control state
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

			// Initialize the predictions with with ground truth in the first step
			if (scan_cnt==0) {
				// Initialize the predicted pose of the ego vehicle with ground truth
				predPose = truePose;

				// Initial the predicted velocity of the ego vehicle with ground truth
				predVelocity = trueVelocity;

				// Initialize the predicted steering angle of the ego vehicle with ground truth
				predSteeringAngle = trueSteeringAngle;

				// Initialize Unscented Kalman Filter for 2D ego vehicle motion tracking
				if (useUKF) {
					vehicle_ukf.InitializeState(truePose, trueVelocity, trueSteeringAngle, t);
				}
			}

			// Get the current pose, velocity and steering angle estimate from the Unscented Kalman Filter
			if (useUKF) {
				predPose = vehicle_ukf.GetPoseEstimate();
				predVelocity = vehicle_ukf.GetVelocityEstimate();
				predSteeringAngle = vehicle_ukf.GetSteeringAngleEstimate();
			}

			// Count the number of lidar scans
			scan_cnt++;

			// Filter scan using voxel filter
			// Declare variable for the voxel grid filter
			pcl::VoxelGrid<PointT> vgf;
			// Initialize voxel grid filter with latest scan cloud
			vgf.setInputCloud(scanCloud);
			// Set voxel grid filter resolution
			double filterResolution = 1.0; // 0.8 // 0.25; 0.5; 0.75; 0.8; 1.0; 2.0; 5.0;
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
				int iter = 4; // 60 // 4; 5; 10; 20; 25; 50; 60; 100;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-4;  // 1e-6 // 1e-1; 1e-2; 1e-3; 1e-4; 1e-5; 1e-6; 1e-7;
				// Get final pose transformation matrix to match the predicted pose with Lidar measurements
				matchingTransform = scanMatchingByNDT(mapCloud, cloudFiltered, predPose, epsilon, iter);
			} else {  // ICP
				// Set maximum number of iterations
				int iter = 16; // 16 // 4; 5; 10; 15; 20; 50;
				// Set minimum transformation difference for termination conditions
				double epsilon = 1e-4;  // 1e-1; 1e-2; 1e-3; 1e-4; 1e-5; 1e-6; 1e-7;		
				// Get final pose transformation matrix to match the predicted pose with Lidar measurements
				matchingTransform = scanMatchingByICP(mapCloud, cloudFiltered, predPose, epsilon, iter);
			}
			
			// Find the current pose using the pose transform from ICP or NDT scan matching
			predPose = getPose(matchingTransform);

			// Trigger Kalman Filter update cycle on the latest measurements
			if (useUKF) {
				vehicle_ukf.UpdateCycle(predPose, trueVelocity, trueSteeringAngle, t);
			}

			// Transform scan so it aligns with ego's actual pose and render that scan
			PointCloudT::Ptr transformedScan (new PointCloudT);
			pcl::transformPointCloud (*cloudFiltered, *transformedScan, matchingTransform);

			// Hide scan point cloud
			viewer->removePointCloud("scan");

			// Change `scanCloud` below to your transformed scan
			renderPointCloud(viewer, transformedScan, "scan", Color(1,0,0) );

			// Remove all shapes and redraw
			viewer->removeAllShapes();
			drawCar(predPose, 1,  Color(0,1,0), 0.35, viewer);

			// Update 2D position estimation error
			posError = sqrt(
				(truePose.position.x - predPose.position.x) * (truePose.position.x - predPose.position.x) + 
				(truePose.position.y - predPose.position.y) * (truePose.position.y - predPose.position.y)
			);
			// Remember maximum 2D position estimation error
			if(posError > maxPosError)
				maxPosError = posError;
			// Store driven distance up to the previous time step
			distDriven_prev = distDriven;
			// Update driven distance
			distDriven = sqrt(
				(truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y)
			);
			if (useUKF) {
				std::cout << "predVelocity = " << predVelocity << " m/s" << std::endl;
			}
			// Update 2D orientation estimation error
			rotError = truePose.rotation.yaw - predPose.rotation.yaw;
			// Remember maximum 2D orientation estimation error
			if(abs(rotError) > maxRotError)
				maxRotError = abs(rotError);
			// Update 2D velocity estimation error (should be small)
			velError = trueVelocity - predVelocity;
			// Remember maximum 2D velocity estimation error
			if(abs(velError) > maxVelError)
				maxVelError = abs(velError);
			// Calculate moving average of the absolute error values
			if (scan_cnt==0) {
				// Initialize moving average filters
				movAvgPosError.initialize(posError);
				movAvgRotError.initialize(rotError);
				movAvgVelError.initialize(velError);
			} else {
				// Update moving average filters
				movAvgPosError.update(posError);
				movAvgRotError.update(rotError);
				movAvgVelError.update(velError);
			}

			// Show driven distance, current velocity and (max.) pose & velocity errors in the visualizer window
			std::string scm_algo_str = (scmAlgoId==0) ? "NDT" : "ICP";
			std::string use_ukf_str = (useUKF) ? " and UKF" : "";
			viewer->removeShape("algoInfo");
			viewer->addText(
				"Using "+scm_algo_str+use_ukf_str, 100, 140, 20, 1.0, 1.0, 1.0, "algoInfo",0
			);
			viewer->removeShape("dist");
			viewer->addText(
				"Distance: "+to_string(distDriven)+" m", 100, 120, 20, 1.0, 1.0, 1.0, "dist",0
			);
			viewer->removeShape("speed");
			viewer->addText(
				"Act. speed: "+to_string(trueVelocity)+" m/s", 100, 100, 20, 1.0, 1.0, 1.0, "speed",0
			);
			viewer->removeShape("posErr");
			viewer->addText(
				"Pos. error: "+to_string(posError)+" m (abs(max.): "+to_string(maxPosError)+" m | exp. avg.: "+
				to_string(movAvgPosError.getMovAvg())+" m)", 100, 80, 20, 1.0, 1.0, 1.0, "posErr", 0
			);
			viewer->removeShape("rotErr");
			viewer->addText(
				"Rot. error: "+to_string(rotError)+" rad (abs(max.): "+to_string(maxRotError)+" rad | exp. avg.: "+
				to_string(movAvgRotError.getMovAvg())+" rad)", 100, 60, 20, 1.0, 1.0, 1.0, "rotErr", 0
			);
			viewer->removeShape("velErr");
			viewer->addText(
				"Vel. error: "+to_string(rotError)+" m/s (abs(max.): "+to_string(maxRotError)+" m/s | exp. avg.: "+
				to_string(movAvgVelError.getMovAvg())+" m/s)", 100, 40, 20, 1.0, 1.0, 1.0, "velErr", 0
			);

			// Show passed / failed test results in the visualizer window
			if(maxPosError > 1.2 || distDriven >= 170.0 ) {
				viewer->removeShape("eval");
				if(maxPosError > 1.2) {
					viewer->addText("Try Again", 100, 20, 20, 1.0, 0.0, 0.0, "eval", 0);
				} else {
					viewer->addText("Passed!", 100, 20, 20, 0.0, 1.0, 0.0, "eval", 0);
				}
			}

			// Clear point cloud
			pclCloud.points.clear();
		}
  	}

	return 0;
}
