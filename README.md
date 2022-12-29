# SDCND04_Scan_Matching_Localization
This is project 4 of Udacity's "Self-Driving Car Engineer" Nanodegree Program focusing on scan matching algorithms for object localization using lidar point clouds.

The goal of this project is to localize a virtual car driving straight along a road in a simulated traffic scenario for at least 170m from the starting position never exceeding a 2D position error of 1.2 m at maximum. The virtual car in the simulation is equipped with a lidar sensor that scans its environment. The simulator provides lidar scans at regular intervals. Additionally, a point cloud map [map.pcd](./map.pcd) of the environment is provided. This point cloud map has been extracted from a demo environment of the [CARLA simulator](https://carla.org/). By using point registration matching between this point cloud map and the incoming lidar scans, localization for the car can be accomplished at every time step. Therefore, a transformation matrix is calculated that maps the lidar point cloud from the current scan in an optial way to the point cloud map. This transformation is used to correct the current pose estimate of the virtual car with resepct to the map.

## Setup and Usage
All the work is done in a pre-set virtual environment in a Udacity workspace. It contains a running installation of [CARLA simulator](https://carla.org/). All the dependencies to run the project are provided in this workspace. If you have a strong local system then you can get a performance/speed boost from setting up the simulation and running it there. The main function of the project code controlling the interaction with CARLA is implemented in the file [c3-main.cpp](./c3-main.cpp). The scan matching algorithms are implemented in [scm.cpp](./scm.cpp). The helper functions in [helper.cpp](./helper.cpp) are provided by Udacity. They have only been extended by an exponential averaging filter to monitor the pose estimation errors. As pure scan matching turns out to work only well if the initial guess for the starting pose of the scan matching iterations is rather close to the true pose of the vehicle, an Unscented Kalman Filter using an Ackermann bicycle model as 2D kinematic motion model has been additionally implemented in the file [ukf.cpp](./ukf.cpp). It can be used optionally and uses the pose estimated obtained scan matching localization as measurement updates. Furthermore, it reads the steering angle and the vehicle speed as additional measurement updates, which are assumed to be available in every vehicle, in order to support the scan matching process with better guesses of the current pose for the next scan matching iteration.

### Running the program
To run the program in the Udacity workspace the following steps need to be taken:

1. Compile the code (Terminal Tab 1)

```
cd /home/workspace/c3-project

cmake .

make
```

2. Launch the simulator in a new terminal tab (Terminal Tab 2)

```
su - student

cd /home/workspace/c3-project

./run-carla.sh
```

3. Run the project code in the first tab (Terminal Tab 1)

Two different scan matching algorithms "Iterative Closest Point (ICP)" or "Normal Distributions Transform (NDT)" have been implemented. You can choose between them by adding either ```ICP``` or ```NDT``` as additional argument to the call of ```./cloud_loc```. Optionally, you can use an additional Unscented Kalman Filter for 2D vehicle motion tracking to provide better initial guesses of the starting pose estimates of the ego car to improve or stabilize scan matching localization. Using the additional Kalman filter scan matching localization is a bit more stable and robust.

Command to use only scan matching localization based on ICP or NDT:

```
cd /home/workspace/c3-project

./cloud_loc ICP  # choose between one of [ICP, NDT] as additional argument
```

Command to use both scan matching localization (ICP or NDT) and an Unscented Kalman Filter using a kinematic bicycle model (2D):
```
cd /home/workspace/c3-project

./cloud_loc ICP UKF  # choose between one of [ICP, NDT] as first argument and add UKF as second argument
```

1. Control the car manually

After starting the simluation, a visualizer window appears. It shows the point cloud map in blue color with the regular overlayed lidar scans in red color for each time step. The virtual car appears as a green bounding box representing its estimated pose, or location and orientation, respectively. To make the car start moving press the 'arrow up key' three times without changing the steering angle. This way, the car will start moving straight with constant speed. Please be aware that you should do this rather slowly to make the virtual car speed up less abruptly. After several trials it turned out that hitting the 'arrow up key' quicker leads to slightly higher end speeds and to quicker acceleration. This can lead to loosing track of the true position using pure scan matching localization. As the optional Unscented Kalman Filter also has some dynamics, and thus, shows a transient phase before it an follow the track, hitting the 'arrow up key' with more care helps to prevent early failure. The remaining pose error is calculated using the selected scan matching algorithm with or without Kalman filter support. The visualization is updated at each time step showing the driven distance from the starting position in [m], the current velocity in [m/s], the current position estimation error in [m], the current orientation estimation error in [rad] and the velocity estimation error in [m/s] are display. Additional the absolute maximum of the errors that has occured so far and an exponential moving average are shown. The absolute maximum of the position estimation error should never exceed 1.2 m over the a driven distance of 170 m.

### Controlling the car
In order to control the car, PCL has a listener in its viewer window. First click on the view window to activate the keyboard listener, and then, the following keys can be used to actuate the car. Note in order not to overwhelm the listener refrain from holding down keys and instead just tap keys. As stated above, please do this rather slowly and with care - especially when speeding up the virtual car.

**Throttle ( UP Key )**

Each tap will increase the throttle power. Three presses is a good amount of throttle power. It is assumed that the virtual car has a wheel speed sensor that provides the overall 2D forward velocity of the virtual car.

**Reverse/Stop ( Down Key )**

A single tap will stop the car and reset the throttle to zero if it is moving. If the car is not moving it will apply throttle in the reverse direction.

**Steer ( Left/Right Keys )**

Tap these keys to incrementally change the steer angle. The green line extruding out in front of the red box in the image above represents the steer value and its end point will move left/right depending on the current value. It is assumed that the actual steering angle is available as an internal measurement obtained from a steering angle sensor.

**Center Camera ( a )**

Press this key to recenter the camera with a top down view of the car.

**Rotate Camera ( mouse left click and drag )**

**Pan Camera ( mouse middle button and drag)**

**Zoom (mouse scroll)**

## Implementation of the original tasks
To fulfill the project requirements, a voxel filter and a scan matching technique using ICP or NDT are implemented to find a pose transfrom matching the current lidar scans with the point cloud map, which in turn gives a location update to the current pose of the car. The latest  pose estimate is used as an initial guess for the next iteration of the selected scan matching algorithm.

**Step 1: Filter the lidar scan using voxel filter**

A voxel grid filter is used to reduce the number of points from each lidar scan to be considered for procesing in the scan matching step. This reduces the computational load and the processing time, but it also deminishes the extend of information about the 3D environment available in each scan point cloud that can be matched with the exising point cloud map. In general, a lower number of lidar scan points remaining in the filtered scan point cloud tendentially leads to higher localization errors because less points are avaible that can be used as a reference.

**Step 2: Find pose transform by using ICP or NDT matching**

In order to find a pose transform, two different scan matching methods are implemented in [c3-main.cpp](./c3-main.cpp) making use of [point cloud library]([https://pointclouds.org/) (PCL) functionality. You can find the implementation of ICP resp. NDT in the following functions:

- "Iterative Closest Point (ICP)" algorithm => ```scanMatchingByICP()```
- "Normal Distributions Transform (NDT)" => ```scanMatchingByNDT()```

Both functions return a 4D tranformation matrix, which can be used to align the current lidar scan to a new determined pose estimation of the simualated car. The new pose of the car is obtained by using the ```getPose()``` helper function.

**Step 3: Transform the scan so it aligns with ego's actual pose and render that scan**

In this final step, the filtered lidar scan cloud is transformed into a new point cloud using in-built PCL functions and the calculated transform from the previous step. Afterwards the input ```scanCloud``` to the point cloud rendering function ```renderPointCloud``` is updated with the newly calculated values of the transformed lidar scan cloud.

## Results for Iterative Closest Point (ICP) Algorithm
As described above you need to pass ```ICP``` as additional parameter to the function call of ```cloud_loc``` in tab 1 after the Carla simulation has been started in tab 2:

```./cloud_loc ICP```

The following settings have been used to meet the requirements.

**Voxel filter settings:**
- voxel grid filter resolution = 1.0 m
- minimum number of points per voxel = 5

**ICP settings:**
- maximum number of registration iterations = 16
- minimum transformation difference for termination conditions = 1e-4
- maximal corresponding distance = 2.0
- ransac outlier rejection threshold = 10

**Achieved 2D position error over 170 m simulated driving distance using ICT with above settings:**
- maximum 2D position error = approx. 0.77 m
- average 2D position error = approx. 0.35 m

The achieved results using ICP are shown below for 3 time steps at the beginning, in the middle and at the end of the simulation:

<img src="screenshots/ICP_test_start_pose_2022-12-28.png"/>
<img src="screenshots/ICP_test_mid_pose_2022-12-28.png"/>
<img src="screenshots/ICP_test_final_pose_2022-12-28.png"/>

## Results for Normal Distributions Transform (NDT) Algorithm
As described above you need to pass ```NDT``` as additional parameter to the function call of ```cloud_loc``` in tab 1 after the Carla simulation has been started in tab 2:

```./cloud_loc NDT```

The following settings (1) have been used to showcase how NDT fails when using less iterations and a coarser voxel grid to make NDT faster.

**Voxel filter settings (1):**
- voxel grid filter resolution = 1.0 m
- minimum number of points per voxel = 5

**NDT settings (1):**
- maximum number of registration iterations = 4
- minimum transformation difference for termination conditions = 1e-4
- maximum step size for More-Thuente line search = 0.1
- NDT grid structure resolution (VoxelGridCovariance) = 1.0

**Achieved 2D position error over 68 m simulated driving distance using NDT with above settings (1):**
- maximum 2D position error = approx. 7.93 m
- average 2D position error = approx. 6.88 m

The achieved results using NDT with parameter settings (1) are shown below for 2 time steps at the beginning and in the middle of the simulation after NDT failed to keep track of the ego vehicle's true pose:

<img src="screenshots/NDT_test_start_pose_2022-12-28.png"/>
<img src="screenshots/NDT_test_fails_2022-12-28.png"/>

Using above setting NDT failed after approx. 60 m loosing track of the ego vehicle's pose. However, every simulation is slightly different. Therefore, NDT alone might work in some cases or with different settings. For me it worked using 60 iterations, a minimum transformation difference of 1e-6 and a finer voxel grid filter resolution of 0.8, s. settings (2). However, NDT needs much more processing time with settings (2) what makes it too slow to keep track if the ego vehicle accelerates too quickly, e.g. at the start of the simulation. Therefore, one needs to accelerate the ego vehicle very carefully to make it work.

The following settings (2) have been used to meet the requirements.

**Voxel filter settings (2):**
- voxel grid filter resolution = 0.8 m
- minimum number of points per voxel = 5

**NDT settings (1):**
- maximum number of registration iterations = 60
- minimum transformation difference for termination conditions = 1e-6
- maximum step size for More-Thuente line search = 0.1
- NDT grid structure resolution (VoxelGridCovariance) = 1.0

**Achieved 2D position error over 170 m simulated driving distance using NDT with above settings (2):**
- maximum 2D position error = approx. 0.24 m
- average 2D position error = approx. 0.49 m

The achieved results using NDT with parameter settings (2) are shown below for 3 time steps at the beginning, in the middle and at the end of the simulation:

<img src="screenshots/NDT_test_start_pose_2022-12-29.png"/>
<img src="screenshots/NDT_test_mid_pose_2022-12-29.png"/>
<img src="screenshots/NDT_test_final_pose_2022-12-29.png"/>

Using other settings than (2) NDT failed in most of the cases for me - especially towards the trickier parts in the point cloud map in the middle range of the track or at its end. As mentioned before, NDT also failed when the ego car accelerated too quickly right at the start of the simulation by loosing track even with settings (2). However, when making NDT faster e.g. by reducing the number of iterations and using a less fine-grained voxel grid it fails later in the simulation, s. settings (1). To make NDT work with e.g. settings (1) the initial guess of the pose estimate at the beginning of each NDT iteration needs to be improved. To do so, a Kalman Filter and additional sensor information can be used to support scan matching localization. This is one way to mitigate this problem and improve stability of 2D vehicle motion tracking. As shown below, an additional Kalman Filter also allows to make NDT work even with less iterations, e.g. using settings (1), and thus, with a reduced processing time.

## Stabilizing Scan Matching Localization by Unscented Kalman Filter (UKF)
After several trials with different settings for ICP or NDT it turned out that scan matching localization is very sensitive towards the initial guess of the starting pose for a new scan matching iteration. If the initial guess is only a little bit too far off scan matching looses track and cannot find the true pose any more. For instance, this happens very likely if the virtual car accelerates too quick or reaches to high a speed. One reason can be that scan matching is too slow to finish the pose estimate before the ego vehicle has moved on too far. Processing time can be reduced by decreasing the number of iterations or by using less points to align with by using a larger voxel filter grid. However, with this the precision of scan matching localization decreases, too, what can also be a failure reason. The point cloud map used in the project has some rather tricky parts (e.g. approx. in the middle and at the end of the track) where scan matching has difficulties and often looses track. A rather finer voxel grid seems to be helpful in this case, but leads also to longer processing time. Somehow, a good compromise between processing speed using only few iterations and precicion is needed. This seems to work for ICP, but not so much for NDT.
To overcome the problem, a strong support for pose estimation is needed. This can be done using further available sensor information and a Kalman filter approach, for example. For this purpose, an Unscented Kalman Filter (UKF) for tracking the 2D motion of the virtual car was implemented, s. [ukf.h](./ukf.h) and [ukf.cpp](./ukf.cpp). The UKF was adapoted from another project ([SFND_Unscented_Kalman_Filter](https://github.com/AndiA76/SFND_Unscented_Kalman_Filter)) and modified to solve this task. As the virtual car in the CARLA simulation is based on an Ackermann steering model the Unscented Kalman Filter uses a [kinematic bicycle model](https://thef1clan.com/2020/09/21/vehicle-dynamics-the-kinematic-bicycle-model/). The UKF uses the poses estimates from Lidar scan matching localization for its measurement updates. A direct position measurement e.g. using GPS would certainly be a good additional source of information. However, in this project we only use the steering angle and the ego car velocity assuming that both measurements are usually available from steering angle sensor and wheel speed sensor. Using these two sensor signals as additional source of information ego motion tracking can be improved and scan matching localization can be stabilized to some extend.

Remark: As a Kalman Filter also adds its own dynamics to the motion tracking process it can also destabilize the whole setting if not properly set. Especially transient phases (e.g. starting up the ego vehicle) are critical. The behavior of Kalman Filters can be adjusted by defining process and measurement noise covariance matrices. Setting high process noise covariance values means putting less trust in the plant model of the Kalman filter and more trust in the measurement updates. Vice versa, setting high measurement noise covariance values means putting less trust in the measurement updates and more trust in the plant model.

## Results for Iterative Closest Point (ICP) with UKF
As described above you need to pass ```ICP``` and ```UKF``` as additional parameters to the function call of ```cloud_loc``` in tab 1 after the Carla simulation has been started in tab 2:

```./cloud_loc ICP UKF```

The following settings have been used to meet the requirements.

**Voxel filter settings:**
- voxel grid filter resolution = 1.0 m
- minimum number of points per voxel = 5

**ICP settings:**
- maximum number of registration iterations = 16
- minimum transformation difference for termination conditions = 1e-4
- maximal corresponding distance = 2.0
- ransac outlier rejection threshold = 10

**UKF settings:**
Process noise:
- standard deviation for longitudinal acceleration = 2.0 m/s^2
- standard deviation for yaw rate change = 2.0 rad/s^2
- standard deviation for steering rate change = 0.1 rad/s^2
Measurement noise:
- standard deviation for Lidar scan matching localization of x-position = 0.1 m
- standard deviation for Lidar scan matching localization of y-position = 0.1 m
- standard deviation for Lidar scan matching localization of yaw angle = 0.2 rad
- standard deviation for velocity measurement from wheel speed sensor = 0.2 m/s
- standard deviation for steering angle measurement from steering angle sensor = 0.01 rad

**Achieved 2D position error over 170 m simulated driving distance using ICT and UKF with above settings:**
- maximum 2D position error = approx. 0.56 m
- average 2D position error = approx. 0.38 m

The achieved results using ICP and UKF are shown below for 3 time steps at the beginning, in the middle and at the end of the simulation:

<img src="screenshots/ICP_UKF_test_start_pose_2022-12-28.png"/>
<img src="screenshots/ICP_UKF_test_mid_pose_2022-12-28.png"/>
<img src="screenshots/ICP_UKF_test_final_pose_2022-12-28.png"/>

## Results for Normal Distributions Transform (NDT) with UKF
As described above you need to pass ```NDT``` and ```UKF``` as additional parameters to the function call of ```cloud_loc``` in tab 1 after the Carla simulation has been started in tab 2:

```./cloud_loc NDT```

The following settings have been used to meet the requirements.

**Voxel filter settings:**
- voxel grid filter resolution = 1.0 m
- minimum number of points per voxel = 5

**NDT settings:**
- maximum number of registration iterations = 4
- minimum transformation difference for termination conditions = 1e-4
- maximum step size for More-Thuente line search = 0.1
- NDT grid structure resolution (VoxelGridCovariance) = 1.0

**UKF settings:**
Same as with ICP and UKF (s. above).

**Achieved maximum pose error over 170 m simulated driving distance using NDT and UKF with above settings:**
- maximum 2D position error = approx. 0.50 m
- average 2D position error = approx. 0.19 m

The achieved results using NDT and UKF are shown below for 3 time steps at the beginning, in the middle and at the end of the simulation:

<img src="screenshots/NDT_UKF_test_start_pose_2022-12-28.png"/>
<img src="screenshots/NDT_UKF_test_mid_pose_2022-12-28.png"/>
<img src="screenshots/NDT_UKF_test_final_pose_2022-12-28.png"/>

Thanks to the UKF the test passes this time although using the same settings (1) as above where NDT failed. Thus, the additional UKF helps to improve the stability of scan matching localization, e.g. using NDT. It also works well with ICP.

## License
Remarks: Since the the code in this repository was provided by [Udacity](https://www.udacity.com/) in a Udacity student workspace, it automatically falls under the Udacity license, too:

[LICENSE.md](./LICENSE.md)

This also applies to the changes, extensions or modifications implemented by the code owner, s. [CODEOWNERS](./CODEOWNERS).
