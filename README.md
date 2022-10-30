# SDCND04_Scan_Matching_Localization
Project 4 of Udacity's "Self-Driving Car Engineer" Nanodegree Program about scan matching algorithms for object localization using lidar point clouds.

The goal of this project is to localize a virtual car driving in a simulation for at least 170m from the starting position and never exceeding a distance pose error of 1.2m. The simulation car is equipped with a lidar sensor. The simulator provides lidar scans at regular intervals. There is also a point cloud map [map.pcd](./map.pcd) already available. This point cloud map has been extracted from some demo environment of the [CARLA simulator][https://carla.org/). By using point registration matching between the map and the incoming lidar scans, localization for the car can be accomplished at every time step.

## Setup and Usage
All the work is done in a pre-set virtual environment in a Udacity workspace. It contains a running installation of [CARLA simulator](https://carla.org/). All the dependencies to run the project are provided in this workspace, though if you have a strong local system then you can get a performance/speed boost from setting up and running there. The project code is implemented in [c3-main.cpp](./c3-main.cpp). The helper functions in [helper.cpp](./helper.cpp) are provided by Udacity.

### Running the program
To run the program in the Udacity workspace the following steps need to be taken:

1. Compile the code (Terminal Tab 1)
´´´
cd /home/workspace/c3-project

cmake .

make
´´´
2. Launch the simulator in a new terminal tab (Terminal Tab 2)
´´´
su - student

cd /home/workspace

./run-carla.sh
´´´
3. Run the project code back in the first tab (Terminal Tab 1)
There is a choice between two different scan matching algorithms "Iterative Closest Point (ICP)" or "Normal Distributions Transform (NDT)" that can be selected by either adding "ICP" or "NDT" as additional argument to the function call of cloud_loc.
´´´
cd /home/workspace/

./cloud_loc ICP # choose between one of [ICP, NDT]
´´´
4. Control the car manually
After starting the simluation a window appears that visualizes the point cloud map in blue color with the regular overlayed lidar scans in red color for each time step. The virtual car appears as a green bounding box representing its estimated location and orientation. To get the car start moving press three times the arrow up key without changing the steering angle. This way, the car will start moving straight with constant speed. The remaining pose error is calculated using the selected scan matching algorithm. The visualization shows the driven distance from the starting position, the current pose estimation error and the maximum pose estimation error at each time step. The latter should never exceed 1.2 m over the a distance of 170 m.

### Controlling the car

In order to control the car, PCL has a listener in its viewer window. First click on the view window to activate the keyboard listener and then the following keys can be used to actuate the car. Note in order not to overwhelm the listener refrain from holding down keys and instead just tap keys.
Throttle ( UP Key )

Each tap will increase the throttle power. Three presses is a good amount of throttle power.
*Reverse/Stop ( Down Key )*

A single tap will stop the car and reset the throttle to zero if it is moving. If the car is not moving it will apply throttle in the reverse direction.
*Steer ( Left/Right Keys )*

Tap these keys to incrementally change the steer angle. The green line extruding out in front of the red box in the image above represents the steer value and its end point will move left/right depending on the current value.
*Center Camera ( a )*

Press this key to recenter the camera with a top down view of the car.
*Rotate Camera ( mouse left click and drag )*
*Pan Camera ( mouse middle button and drag)*
*Zoom (mouse scroll)*
*Getting Started*

## Implementation of the tasks
To fulfill the project requirements, a voxel filter and a scan matching technique using ICP or NDT are implemented to find a pose transfrom matching the current lidar scans with the point cloud map, which in turn gives a location update to the current pose of the car.

Step 1: Filter the lidar scan using voxel filter
A voxel grid filter is used to reduce the number of points from each lidar scan to be considered for further processing. This reduces the computational time, but it also reduces the available informaiton to match the current environmental scan with the exising point cloud map. In general, a lower number of lidar scan points after filter tendentially leads to higher localization errors because less points are used as reference.

Step 2: Find pose transform by using ICP or NDT matching
To find a pose transform two different scan matching methods have been implemented making use of [point cloud library]([https://pointclouds.org/) functions:
- "Iterative Closest Point (ICP)" algorithm => scanMatchingByICP()
- "Normal Distributions Transform (NDT)" => scanMatchingByNDT()
Both functions return a 4D tranformation matrix, which can be used to align the current lidar scan to a new determined pose estimation of the simualated car. The new pose of the car is obtained by using the getPose() helper function.

Step 3: Transform the scan so it aligns with ego's actual pose and render that scan
In this final step, the filtered lidar scan cloud is transformed into a new point cloud using in-built pcl functions and the calculated transform from the previous step. Afterwards the point clouds for rendering are updated accordingly.

## Results for Iterative Closest Point (ICP) Algorithms
As described above you need to pass ´´´ICP´´´ as additional parameter to the function call of ´´´cloud_loc´´´ after the Carla simulation has been started:

´´´./cloud_loc ICP´´´

The following settings have been used to meet the requirements.

Voxel filter settings:
- voxel grid filter resolution = 0.8 m

ICP settings:
- maximum number of registration iterations = 15
- minimum transformation difference for termination conditions = 1e-4
- maximal corresponding distance = 2.0
- ransac outlier rejection threshold = 10

Achieved maximum pose error over 170 m simulated driving distance using NDT and above settings:
- maximum pose error = approx. 0.64 m

The achieved results using ICP are shown below for 3 time steps at the beginning, in the middel and at the end of the simulation:
<img src="screenshots/ICP_test_start_pose_2022-10-16.png"/>
<img src="screenshots/ICP_test_mid_pose_2022-10-16.png"/>
<img src="screenshots/ICP_test_final_pose_2022-10-16.png"/>

## Results for Normal Distributions Transform (NDT) Algorithm
As described above you need to pass ´´´NDT´´´ as additional parameter to the function call of ´´´cloud_loc´´´ after the Carla simulation has been started:

´´´./cloud_loc NDT´´´

The following settings have been used to meet the requirements.

Voxel filter settings:
- voxel grid filter resolution = 0.8 m

NDT settings:
- maximum number of registration iterations = 60
- minimum transformation difference for termination conditions = 1e-6
- maximum step size for More-Thuente line search = 0.1
- NDT grid structure resolution (VoxelGridCovariance) = 1.0

Achieved maximum pose error over 170 m simulated driving distance using NDT and above settings:
- maximum pose error = approx. 0.45 m

The achieved results using ICP are shown below for 3 time steps at the beginning, in the middel and at the end of the simulation:
<img src="screenshots/NDT_test_start_pose_2022-10-16.png"/>
<img src="screenshots/NDT_test_mid_pose_2022-10-16.png"/>
<img src="screenshots/NDT_test_final_pose_2022-10-16.png"/>

## License
Remarks: Since the the code in this repository was provided by [Udacity](https://www.udacity.com/) in a Udacity student workspace, it automatically falls under the Udacity license, too:https://carla.org/

[LICENSE.md](./LICENSE.md)

This also applies to the changes, extensions or modifications implemented by the code owner, s. [CODEOWNERS](./CODEOWNERS).
