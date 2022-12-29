// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht using code base/skeleton provided by Udacity
//  Source:	    https://www.udacity.com/
// 
//  Copyright © 2012 - 2021, Udacity, Inc.
//  Copyright © 2022 Andreas Albrecht
// ============================================================================
//
// Implementation of data structures and helper functions for ego vehicle localization
// using Lidar point clouds and scan matching algorithms.

#include "helper.h"

// 2D coordinate transform
Eigen::Matrix4d transform2D(double theta, double xt, double yt) {

	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity ();

	matrix(0, 3) = xt;
	matrix(1, 3) = yt;

	matrix(0, 0) = cos(theta);
	matrix(0, 1) = -sin(theta);
	matrix(1, 0) = sin(theta);
	matrix(1, 1) = cos(theta);

	return matrix;
}

// 3D coordinate transform
Eigen::Matrix4d transform3D(
	double yaw, double pitch, double roll, double xt, double yt, double zt
) {

	Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

	matrix(0, 3) = xt;
	matrix(1, 3) = yt;
	matrix(2, 3) = zt;

	matrix(0, 0) = cos(yaw) * cos(pitch);
	matrix(0, 1) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	matrix(0, 2) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
	matrix(1, 0) = sin(yaw) * cos(pitch);
	matrix(1, 1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	matrix(1, 2) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
	matrix(2, 0) = -sin(pitch);
	matrix(2, 1) = cos(pitch) * sin(roll);
	matrix(2, 2) = cos(pitch) * cos(roll);

	return matrix;
}

// get the current pose (location and orientation) of a simulated object
Pose getPose(Eigen::Matrix4d matrix) {

	Pose pose(
		Point(
			matrix(0,3), matrix(1,3), matrix(2,3)
		),
		Rotate(
			atan2(matrix(1, 0),matrix(0, 0)),
			atan2(-matrix(2,0), sqrt(matrix(2,1)*matrix(2,1) + matrix(2,2)*matrix(2,2))),
			atan2(matrix(2,1),matrix(2,2))
		)
	);
	return pose;
}

// get the distance between two 3D points
double getDistance(Point p1, Point p2) {
	return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z) );
}

double minDistance(Point p1, vector<Point> points) {
	if (points.size() > 0) {
		double dist = getDistance(p1, points[0]);
		for (unsigned int index = 1; index < points.size(); index++) {
			double newDist = getDistance(p1, points[index]);
			if (newDist < dist)
				dist = newDist;
		}
		return dist;
	}
	return -1;
}

// print a 4D transformation matrix (of type double)
void print4x4Matrix(const Eigen::Matrix4d & matrix) {
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// print a 4D transformation matrix (of type float)
void print4x4Matrixf (const Eigen::Matrix4f & matrix) {
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// render a 3D point cloud
void renderPointCloud(
	pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
	std::string name, Color color, int renderSize
) {
    viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
    viewer->setPointCloudRenderingProperties (
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, renderSize, name
	);
    viewer->setPointCloudRenderingProperties (
		pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name
	);
}

// render a 3D ray
void renderRay(
	pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color
) {
	viewer->addLine(PointT(p1.x, p1.y, 0), PointT(p2.x, p2.y, 0), color.r, color.g, color.b, name);
}

// render a 3D path
void renderPath(
	pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudT::Ptr& cloud,
	std::string name, Color color
) {

	int previous = 0;
	for (unsigned int index = previous+1; index < cloud->points.size(); index++) {
		renderRay(
			viewer,
			Point(
				cloud->points[previous].x, cloud->points[previous].y, 0
			),
			Point(
				cloud->points[index].x, cloud->points[index].y, 0
			),
			name+to_string(previous),
			color
		);
		previous++;
	}

}

// angle around z axis
Eigen::Quaternionf getQuaternion(float theta) {
	Eigen::Matrix3f rotation_mat;
	rotation_mat << 
	cos(theta), -sin(theta), 0,
	sin(theta),  cos(theta), 0,
	0, 			 0, 		 1;
	
	Eigen::Quaternionf q(rotation_mat);
	return q;
}

// render a 3D bounding box
void renderBox(
	pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity
) {
    if (opacity > 1.0)
    	opacity = 1.0;
    if (opacity < 0.0)
        opacity = 0.0;
    std::string cube = "box"+std::to_string(id);
    viewer->addCube(
		box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
		cube
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube
	);

    std::string cubeFill = "boxFill"+std::to_string(id);
    viewer->addCube(
		box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
		cubeFill
	);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}


// Implementation of a time-discrete exponential moving average (EMA) filter class
// Source: https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Exponential%20Moving%20Average/Exponential-Moving-Average.html
/**
 * @brief Constructor: Initializes a new exponential moving average filter instance.
 */
EMA::EMA() {
	// Initialize moving average with zero
	x_avg_ = 0;

	// Initialize decay constant (must be greater zero)
	alpha_ = 0.1;
}

/** 
 * @brief Desctructor.
 */
EMA::~EMA() {}

/**
 * @brief Initialize exponential moving average filter.
 * 
 * @param x_0: Initial input to the exponential moving average filter.
 */
void EMA::initialize(double x_0) {
	// Initialize moving average
	x_avg_ = x_0;
}

/**
 * @brief Update exponential moving average filter.
 * 
 * @param x_n: Input to the moving average filter at time step t_n.
 */
void EMA::update(double x_n) {
	// Update exponential moving average
	x_avg_ = alpha_ * x_n + (1 - alpha_) * x_avg_;
}

/**
 * @brief Get the current moving average value.
 * 
 * @returns Current moving average.
 */
double EMA::getMovAvg() {
	// Return the current moving average value
	return x_avg_;
}
