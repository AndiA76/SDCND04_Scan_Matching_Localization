// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================

// Declaration of an Unscented Kalman Filter (UKF) class for 2D vehicle motion
// tracking using a kinematic bicycle model.

#ifndef UKF_H
#define UKF_H

#include <math.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// #include "Eigen/Dense"  // https://eigen.tuxfamily.org/dox/GettingStarted.html
#include "helper.h"

class UKF
{
    public:

    /* CLASS MEMBER FUNCTIONS */

    /**
     * @brief Constructor: Initializes a new Unscented Kalman Filter instance to track 2D ego vehicle motion.
     *  
     * @param front_axle_dist Distance between center of mass and front axle of the ego vehicle.
     * 
     * @param rear_axle_dist Distance between center of mass and rear axle of the ego vehicle.
     * 
     * @param sdelta_max Maximum steering angle of the ego vehicle.
     */
    UKF(double front_axle_dist, double rear_axle_dist, double max_steering_angle);

    /**
     * @brief Destructor.
     * 
     */
    virtual ~UKF();

    /**
     * @brief Initialize the state vector of the Unscented Kalman Filter
     * 
     * @param initialPose Initial pose of the vehicle to be tracked.
     *  
     * @param initialVelocity Initial longitudinal velocity of the ego vehicle in m/s obtained from wheel speed sensor
     * 
     * @param initialSteeringAngle Initial steering angle of the ego vehicle in rad obtained from steering angle sensor
     * 
     * @param timestamp_s Current timestamp in seconds.
     * 
     */
    void InitializeState(Pose initialPose, double initialVelocity, double initialSteeringAngle, double timestamp_s);

    /**
     * @brief Unscented Kalman Filter Prediction and Measurement Update Cycle using Lidar Scan Matching.
     * 
     * @param measuredPose Measured pose of the vehicle from Lidar scan matching localization.
     * 
     * @param measuredVelocity Measured longitudinal velocity of the ego vehicle in m/s from wheel speed sensor
     * 
     * @param measuredSteeringAngle Measured steering angle of the ego vehicle in rad from steering angle sensor
     * 
     * @param timestamp_s Timestamp of the latest pose measurement in seconds.
     */
    void UpdateCycle(Pose measuredPose, double measuredVelocity, double measuredSteeringAngle, double timestamp_s);

    /**
     * @brief Generate sigma points Xsig_aug_ for the augmented state vector.
     */
    void GenerateSigmaPoints(); 

    /**
     * @brief Predict sigma points Xsig_pred_ for the augmented state vector.
     * 
     * @param delta_t Time difference between time step k and time step k+1 in s.
     */
    void PredictSigmaPoints(double delta_t);

    /**
     * @brief Predict mean and covariance of the predicted state.
     */
    void PredictStateMeanAndCovariance();

    /**
     * @brief Predict the sigma points, the state, and the state covariance for the next time step.
     * 
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * @brief Measurement update of state and state covariance matrix using Lidar scan matching localization and odometry measurement.
     * 
     * @param measuredPose Measured pose of the ego vehicle at time step k+1 (obtained from Lidar scan matching localization)
     * 
     * @param measuredVelocity Measured velocity of the ego vehicle at time step k+1 (obtained from odometry)
     * 
     * @param measuredSteeringAngle Measured steering angle of the ego vehicle at time step k+1 (obtained from odometry)
     */
    void MeasurementUpdate(Pose measuredPose, double measuredVelocity, double measuredSteeringAngle);

    /**
     * @brief Get estimate of the current ego vehicle pose from UKF.
     * 
     * @return Estimated pose of the ego vehicle.
     */
    Pose GetPoseEstimate();
    
    /**
     * @brief Get estimate of the current ego vehicle velocity from UKF.
     * 
     * @return Estimated velocity of the ego vehicle.
     */
    double GetVelocityEstimate();

    /**
     * @brief Get estimate of the current ego vehicle steering angle from UKF.
     * 
     * @return Estimated steering angle of the ego vehicle.
     */
    double GetSteeringAngleEstimate();

    /* STATIC CONSTANTS */
    
    // 2 * PI
    static constexpr double M_PI_X_2 = 2 * M_PI;

    /* CLASS MEMBER VARIALBES */

    // Initialization flag: Initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // Timestamp of the previous step in s when the state is true
    long long time_s_;

    // Geometric dimensions of the ego vehicle
    double l_f_;    // Distance between center of mass and front axle
    double l_r_;    // Distance between center of mass and rear axle
    double l_;      // Overall distance between front and rear axle

    // Maximum steering angle (symmetrix in both directions) in rad
    double max_steering_angle_;

    // State vector dimension
    int n_x_;

    // State vector in SI units and rad
    Eigen::VectorXd x_;

    // State covariance matrix
    Eigen::MatrixXd P_;

    // Number of sigma points
    int n_sig_;

    // Sigma point spreading parameter
    double lambda_;

    // Sigma point spread
    double sig_spread_;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // Predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    // Augmented state dimension
    int n_x_aug_;

    // Augmented state vector
    Eigen::VectorXd x_aug_;

    // Augmented state covariance matrix
    Eigen::MatrixXd P_aug_;

    // Augmented sigma points matrix
    Eigen::MatrixXd Xsig_aug_;

    // Process noise standard deviation and covariance for linear acceleration (longitudinal axis) in m/s^2
    double std_a_;
    double std_a_square_;

    // Process noise standard deviation and covariance for angular acceleration (yaw angle) in rad/s^2
    double std_yawdd_;
    double std_yawdd_square_;

    // Process noise standard deviation and covariance for angular acceleration (steering angle) in rad/s^2
    double std_sdeltadd_;
    double std_sdeltadd_square_;

    // Lidar scan matching localization measurement noise standard deviation for position x in m (cartesian coordinates)
    double std_lidar_scm_x_;

    // Lidar scan matching localization measurement noise standard deviation for position y in m (cartesian coordinates)
    double std_lidar_scm_y_;

    // Lidar scan matching localization measurement noise standard deviation for yaw angle in rad (cartesian coordinates)
    double std_lidar_scm_yaw_;

    // Velocity measurement noise standard deviation in m/s (cartesian coordinates)
    double std_wheel_speed_sensor_;

    // Steering angle measurement noise standard deviation in rad (cartesian coordinates)
    double std_steering_angle_sensor_;

    // Measurement noise covariance matrix
    Eigen::MatrixXd R_meas_;

    // Normalized Innovation Squared (NIS) value for measurement update
    double NIS_meas_;
};

#endif  // UKF_H
