// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================

// Implementation of an Unscented Kalman Filter (UKF) class for 2D vehicle motion
// tracking using a kinematic bicycle model.

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

//#include "Eigen/Dense"
//#include "helper.h"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


/**
 * @brief Constructor: Initializes a new Unscented Kalman Filter instance to track 2D ego vehicle motion.
 *  
 * @param front_axle_dist Distance between center of mass and front axle of the ego vehicle.
 * 
 * @param rear_axle_dist Distance between center of mass and rear axle of the ego vehicle.
 * 
 * @param sdelta_max Maximum steering angle of the ego vehicle.
 */
UKF::UKF(double front_axle_dist, double rear_axle_dist, double max_steering_angle) {
    // Initialization flag: set to true after first update cycle (default: false)
    is_initialized_ = false;

    // Set distances between ego vehicle axles
    l_f_ = front_axle_dist;
    l_r_ = rear_axle_dist;
    l_ = l_f_ + l_r_;

    // Set maximum steering angle
    max_steering_angle_ = max_steering_angle;

    // Process noise standard deviation for longitudinal acceleration (linear acceleration noise) in m/s^2
    std_a_ = 2.0;  // 1.0; 2.0 => initial guess
    std_a_square_ = std_a_ * std_a_;
    // Process noise standard deviation for yaw rate change (angular acceleration noise) in rad/s^2
    std_yawdd_ = 2.0;  // 1.0; 2.0 => initial guess
    std_yawdd_square_ = std_yawdd_ * std_yawdd_;
    // Process noise standard deviation for steering rate change (angular acceleration noise) in rad/s^2
    std_sdeltadd_ = 0.1;  // 0.1
    std_sdeltadd_square_ = std_sdeltadd_ * std_sdeltadd_;

    // Lidar scan matching localization measurement noise standard deviation for position x in m (cartesian coordinates)
    std_lidar_scm_x_ = 0.1;  // 0.15 => initial guess 0.5 => too high
    // Lidar scan matching localization measurement noise standard deviation for position y in m (cartesian coordinates)
    std_lidar_scm_y_ = 0.1;  // 0.15 => initial guess 0.5 => too high
    // Lidar scan matching localization measurement noise standard deviation for yaw angle in rad (cartesian coordinates)
    std_lidar_scm_yaw_ = 0.2;  // 0.25, 0.5 => initial guess 1.0 => too high

    // Velocity measurement noise standard deviation
    std_wheel_speed_sensor_ = 0.2;  // 0.05, 0.25, 0.3

    // Steering angle mesurement noise standard deviation
    std_steering_angle_sensor_ = 0.01;  // 0.02

    // Dimension of the original state vector
    n_x_ = 6;  // x_ = [px py vel yaw yawd sdelta]
    // Initial state vector for 2D motion
    x_ = VectorXd::Zero(n_x_);
    // Initial state covariance matrix
    P_ = MatrixXd::Identity(n_x_, n_x_);

    // Dimension of the augmented state vector
    n_x_aug_ = 9;  // x_ = [px py vel yaw yawd sdelta nu_a nu_yawdd nu_sdeltadd]
    // Number of sigmal points
    n_sig_ = 2 * n_x_aug_ + 1;
    // Sigma point spreading parameter (best practice setting: lambda_ = 3 - n_x_)
    lambda_ = 3 - n_x_;
    // Sigma point spread
    sig_spread_ = sqrt(lambda_ + n_x_aug_);
    // Initial sigma points matrix
    Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_);

    // Set initial weights of sigma points (only once)
    weights_ = VectorXd::Zero(n_sig_);
    double denominator = lambda_ + n_x_aug_;
    weights_(0) = lambda_ / denominator; // set first weight
    for (int i = 1; i < n_sig_; ++i) {
        // Set the rest of the n_sig_ = 2 * n_x_aug_ + 1 weights
        double weight = 0.5 / denominator;
        weights_(i) = weight;
    }

    // Initialize augmented state vector
    x_aug_ = VectorXd::Zero(n_x_aug_);
    // Initialize augmeted state covariance matrix
    P_aug_ = MatrixXd::Identity(n_x_aug_, n_x_aug_);
    // Initialize augmented sigma point matrix
    Xsig_aug_ = MatrixXd::Zero(n_x_aug_, n_sig_);

    // Initial measurement noise covariance matrix
    R_meas_ = MatrixXd::Zero(5, 5);
    R_meas_ << std_lidar_scm_x_ * std_lidar_scm_x_, 0, 0, 0, 0,
                0, std_lidar_scm_y_ * std_lidar_scm_y_, 0, 0, 0,
                0, 0, std_wheel_speed_sensor_ * std_wheel_speed_sensor_, 0, 0,
                0, 0, 0, std_lidar_scm_yaw_ * std_lidar_scm_yaw_, 0,
                0, 0, 0, 0, std_steering_angle_sensor_ * std_steering_angle_sensor_;

    // Initial Normalized Innovation Squared (NIS) value for measurement update
    NIS_meas_ = 0.0;
}


/** 
 * @brief Desctructor: Delete current Unscented Kalman Filter instance.
 */
UKF::~UKF() {}


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
void UKF::InitializeState(Pose initialPose, double initialVelocity, double initialSteeringAngle, double timestamp_s) {
    // Initialize Kalman Filter state vector using initial ground truth pose plus velocity and steering angle measurement
    double px   = initialPose.position.x;  // x-position of the ego vehicle
    double py   = initialPose.position.y;  // y-position of the ego vehicle
    double yaw  = initialPose.rotation.yaw;  // yaw angle of the ego vehicle
    double yawd = initialVelocity * tan(initialSteeringAngle) / l_;  // yaw rate of the ego vehicle

    // Initialize state vector x_
    x_ << px, py, initialVelocity, yaw, yawd, initialSteeringAngle;

    // Initialize time stamp in seconds [s]
    time_s_ = timestamp_s;

    // set initialization flag to true
    is_initialized_ = true;
}


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
void UKF::UpdateCycle(Pose measuredPose, double measuredVelocity, double measuredSteeringAngle, double timestamp_s) {
    // Measurement process of the Unscented Kalman Filter using Lidar Scan Matching Localization

    // Check if Kalman Filter state has been initialized
    if (!is_initialized_) {
        // Initialize Kalman Filter state using the latest pose measurement
        InitializeState(measuredPose, measuredVelocity, measuredSteeringAngle, timestamp_s);

        return;
    } // is_initialized_

    // Kalman Filter prediction step:
    // - update the time step
    // - update the state transition matrix F according to the next time step
    // - update the process noise covariance matrix P

    // Calculate the time step between the current and the previous time stamp
    double delta_t = static_cast<double>(timestamp_s - time_s_);
    // Update the time stamp
    time_s_ = timestamp_s;

    // Predict next Kalman Filter state at the next time step:
    // When applying Euler method for predicting large time steps, we assume that the state
    // derivative is constant over the whole time interval. However, this leads to errors in
    // state propagation. In order to mitigate error propagation, we divide the one big step
    // into several small ones. See detail discussion @ ...
    // https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/55
    while (delta_t > 0.1) {
        const double dt = 0.05;
        Prediction(dt);
        delta_t -= dt;
    }
    Prediction(delta_t);

    // Kalman Filter measurement update step:
    // - update the Kalman filter state using pose measurement from Lidar Scan Matching Localization,
    //   velocity measurement from wheel speed sensor and steering angle measurement from steering
    //   angle sensor
    // - update the state and measurement noise covariance matrices

    // Measurement update
    MeasurementUpdate(measuredPose, measuredVelocity, measuredSteeringAngle);
}


/**
 * @brief Generate sigma points Xsig_aug_ for the augmented state vector.
 */
void UKF::GenerateSigmaPoints() {
    // Initialize augmented state vector
    x_aug_ = VectorXd::Zero(n_x_aug_); // n_x_aug_ = n_x_ + 2
    x_aug_.head(n_x_) = x_; // original state vector

    // Initialize augumented state covariance matrix
    P_aug_ = MatrixXd::Zero(n_x_aug_, n_x_aug_); // n_x_aug_ = n_x_ + 2
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_(n_x_aug_-3, n_x_aug_-3) = std_a_square_; // covariance of linear acceleration noise
    P_aug_(n_x_aug_-2, n_x_aug_-2) = std_yawdd_square_; // covariance of angular acceleration noise
    P_aug_(n_x_aug_-1, n_x_aug_-1) = std_sdeltadd_square_; // covariance of angular acceleration noise on steering angle

    // Calculate the square root matrix of the agumented process noise covariance matrix
    MatrixXd L = P_aug_.llt().matrixL(); // use Cholesky decomposition

    // Initialize augumented sigma point matrix
    Xsig_aug_ = MatrixXd::Zero(n_x_aug_, n_sig_);
    Xsig_aug_.col(0) = x_aug_; // set first column
    for (int i = 0; i < n_x_aug_; ++i) {
        // Fill the other columns
        Xsig_aug_.col(i + 1)            = x_aug_ + sig_spread_ * L.col(i);
        Xsig_aug_.col(i + 1 + n_x_aug_) = x_aug_ - sig_spread_ * L.col(i);

        // Yaw angle sigma point normalization modulo +/- PI
        while (Xsig_aug_(3, i) > M_PI) Xsig_aug_(3, i) -= M_PI_X_2;
        while (Xsig_aug_(3, i) < -M_PI) Xsig_aug_(3, i) += M_PI_X_2;

        // Limit steering angle signa points
        if (Xsig_aug_(5, i) < -max_steering_angle_) {
            Xsig_aug_(5, i) = -max_steering_angle_;
        } else if (Xsig_aug_(5, i) > max_steering_angle_) {
            Xsig_aug_(5, i) = max_steering_angle_;
        }
    }
}


/**
 * @brief Predict sigma points Xsig_pred_ for the augmented state vector.
 * 
 * @param delta_t Time difference between time step k and time step k+1 in s.
 */
void UKF::PredictSigmaPoints(double delta_t) {
    // Loop over all sigma points
    for (int i = 0; i < n_sig_; ++i) {
        // Extract state vector elements from augmented sigma point matrix for better readability
        double px          = Xsig_aug_(0, i);   // x-position of the ego vehicle
        double py          = Xsig_aug_(1, i);   // y-position of the ego vehicle
        double vel         = Xsig_aug_(2, i);   // longitudinal velocity of the ego vehicle
        double yaw         = Xsig_aug_(3, i);   // yaw angle of the ego vehicle [-PI, +PI]
        double yawd        = Xsig_aug_(4, i);   // yaw rate of the ego vehicle
        double sdelta      = Xsig_aug_(5, i);   // steering angle of the ego vehicle' front wheels
        double nu_a        = Xsig_aug_(6, i);   // linear accelaration noise on longitudinal motion
        double nu_yawdd    = Xsig_aug_(7, i);   // angular acceleration noise on yaw angle
        double nu_sdeltadd = Xsig_aug_(8, i);   // angular acceleration noise on steering angle

        // Declare predicted state vector comnponents
        double px_pred, py_pred, vel_pred, yaw_pred, yawd_pred, sdelta_pred;

        // Precidct next velocity
        vel_pred = vel;

        // Predict next steering angle
        sdelta_pred = sdelta;

        // Limit predicted steering angle
        if (sdelta_pred < -max_steering_angle_) {
            sdelta_pred = -max_steering_angle_;
        } else if (sdelta_pred > max_steering_angle_) {
            sdelta_pred = max_steering_angle_;
        }

        // Predict angle of instantaneous rotation around velocity pole
        double beta_pred = atan(l_r_ * tan(sdelta_pred) / l_);

        // Predict next yaw rate
        yawd_pred = vel_pred * tan(sdelta_pred) * cos(beta_pred) / l_r_;

        // Predict next yaw angle
        yaw_pred = yaw + yawd_pred * delta_t;

        // Predicted yaw angle normalization modulo +/- PI
        while (yaw_pred > M_PI) yaw_pred -= M_PI_X_2;
        while (yaw_pred < -M_PI) yaw_pred += M_PI_X_2;

        // Predict next x-y-position using 2D kinematic bicycle model (avoid division by zero)
        px_pred = px + vel_pred * cos(beta_pred + yaw_pred) * delta_t;
        py_pred = py + vel_pred * sin(beta_pred + yaw_pred) * delta_t;

        // Initialize process noise (given as noise on longitudinal and angular acceleration)
        double delta_t_square_half = 0.5 * delta_t * delta_t;
        double nu_px  = delta_t_square_half * cos(beta_pred + yaw_pred) * nu_a;
        double nu_py  = delta_t_square_half * sin(beta_pred + yaw_pred) * nu_a;
        double nu_vel = delta_t * nu_a;
        double nu_yaw  = delta_t_square_half * nu_yawdd;
        double nu_yawd = delta_t * nu_yawdd;
        double nu_sdelta = delta_t_square_half * nu_sdeltadd;

        // Add process noise effect to predicted state vector
        px_pred   = px_pred + nu_px;
        py_pred   = py_pred + nu_py;
        vel_pred  = vel_pred + nu_vel;
        yaw_pred  = yaw_pred + nu_yaw;
        yawd_pred = yawd_pred + nu_yawd;
        sdelta_pred = sdelta_pred + nu_sdelta;

        // Limit predicted steering angle
        if (sdelta_pred < -max_steering_angle_) {
            sdelta_pred = -max_steering_angle_;
        } else if (sdelta_pred > max_steering_angle_) {
            sdelta_pred = max_steering_angle_;
        }

        // Predicted yaw angle normalization modulo +/- PI
        while (yaw_pred > M_PI) yaw_pred  -= M_PI_X_2;
        while (yaw_pred < -M_PI) yaw_pred += M_PI_X_2;

        // Update Xsig_pred
        Xsig_pred_(0, i) = px_pred;
        Xsig_pred_(1, i) = py_pred;
        Xsig_pred_(2, i) = vel_pred;
        Xsig_pred_(3, i) = yaw_pred;
        Xsig_pred_(4, i) = yawd_pred;
        Xsig_pred_(5, i) = sdelta_pred;
    }
}


/**
 * @brief Predict mean and covariance of the predicted state.
 */
void UKF::PredictStateMeanAndCovariance() {
    // Predict state mean
    VectorXd x_pred = VectorXd::Zero(n_x_);
    for (int i = 0; i < n_sig_; ++i) {
        x_pred += weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted yaw angle normalization modulo +/- PI
    while (x_pred(3) > M_PI) x_pred(3) -= M_PI_X_2;
    while (x_pred(3) < -M_PI) x_pred(3) += M_PI_X_2;

    // Predict state (process noise) covariance matrix
    MatrixXd P_pred = MatrixXd::Zero(n_x_, n_x_);
    for (int i = 0; i < n_sig_; ++i) {
        // State residual
        VectorXd x_diff = Xsig_pred_.col(i) - x_pred;

        // Yaw angle residual normalization modulo +/- PI
        while (x_diff(3) > M_PI) x_diff(3) -= M_PI_X_2;
        while (x_diff(3) < -M_PI) x_diff(3) += M_PI_X_2;

        // Final predicted state covarinace matrix
        P_pred += weights_(i) * x_diff * x_diff.transpose();
    }
  
    // Store predicted state vector and state covariance matrix
    x_ = x_pred;
    P_ = P_pred;
}


/**
 * @brief Predict the sigma points, the state, and the state covariance for the next time step.
 * 
 * @param delta_t Time between k and k+1 in s
 */
void UKF::Prediction(double delta_t) {
    /// Generate sigma points for the augmented state vector
    GenerateSigmaPoints();

    // Predict augmented sigma points for the next time step
    PredictSigmaPoints(delta_t);

    // Predicted state mean and covariance for the next time step
    PredictStateMeanAndCovariance();
}


/**
 * @brief Measurement update of state and state covariance matrix using Lidar scan matching localization and odometry measurement.
 * 
 * @param measuredPose Measured pose of the ego vehicle at time step k+1 (obtained from Lidar scan matching localization)
 * 
 * @param measuredVelocity Measured velocity of the ego vehicle at time step k+1 (obtained from odometry)
 * 
 * @param measuredSteeringAngle Measured steering angle of the ego vehicle at time step k+1 (obtained from odometry)
 */
void UKF::MeasurementUpdate(Pose measuredPose, double measuredVelocity, double measuredSteeringAngle) {
    // Set measurement dimension incl. 2D Lidar scan matching localization (x and y point position and yaw angle),
    // velocity measurement from wheel speed sensor and steering angle measurement from steering angle sensor.
    int n_z = 5;

    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig_);

    // Create predicted mean measurement
    VectorXd z_pred = VectorXd::Zero(n_z);

    // Create measurement covariance matrix
    MatrixXd S = MatrixXd::Zero(n_z, n_z);

    /* ----------- PREDICT MEASUREMENTS ----------- */

    // Transform sigma points into measurement space
    for (int i = 0; i < n_sig_; ++i) {
        // Measurement model in Cartesian coordinates
        Zsig(0, i) = Xsig_pred_(0, i);  // x-position (px)
        Zsig(1, i) = Xsig_pred_(1, i);  // y-position (py)
        Zsig(2, i) = Xsig_pred_(2, i);  // velocity
        Zsig(3, i) = Xsig_pred_(3, i);  // yaw angle [-PI, +PI]
        Zsig(4, i) = Xsig_pred_(5, i);  // steering angle
    }

    // Calculate'predicted mean measurement (n_sig_ = 2 * n_x_aug_+ 1 simga points)
    for (int i = 0; i < n_sig_; ++i) {
        z_pred += weights_(i) * Zsig.col(i);
    }

    // Calculate predicted measurement covariance matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
    for (int i = 0; i < n_sig_; ++i) {
        // Measurement residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // Update measurement covariance matrix
        S += weights_(i) * z_diff * z_diff.transpose();
    }

    // Add measurement noise covariance matrix
    S = S + R_meas_;

    /* ----- UPDATE STATE MEAN AND COVARIANCE ----- */

    // Create vector for true received measurements
    VectorXd z = VectorXd::Zero(n_z);

    // Get received pose, velocity and steering angle measurement
    z(0) = measuredPose.position.x;     // measured x-position of the ego vehicle
    z(1) = measuredPose.position.y;     // measured y-position of the ego vehicle
    z(2) = measuredVelocity;            // measured velocity of the ego vehicle
    z(3) = measuredPose.rotation.yaw;   // measured yaw angle of the ego vehicle
    z(4) = measuredSteeringAngle;       // measured steering angle of the ego vehicle

    // Create cross correlation matrix between sigma points in state space and in measurement space
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

    // Calculate cross correlation matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
    for (int i = 0; i < n_sig_; ++i) {
        // State residual
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // Yaw angle residual normalization modulo +/- PI
        while (x_diff(3) > M_PI) x_diff(3) -= M_PI_X_2;
        while (x_diff(3) < -M_PI) x_diff(3) += M_PI_X_2;

        // Measurement residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // Update cross-correlation matrix between sigma points in state space and in measurement space
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    // Calculate Kalman Filter gain maxtrix
    MatrixXd K = Tc * S.inverse();

    // Measurement residuals
    VectorXd z_diff = z - z_pred;

    // Update state mean and state covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // Yaw angle normalization modulo +/- PI
    while (x_(3) > M_PI) x_(3) -= M_PI_X_2;
    while (x_(3) < -M_PI) x_(3) += M_PI_X_2;

    // Limit steering angle
    if (x_(5) < -max_steering_angle_) {
        x_(5) = -max_steering_angle_;
    } else if (x_(5) > max_steering_angle_) {
        x_(5) = max_steering_angle_;
    }

    // Calculate Normalized Innovation Squared (NIS) for measurement update
    NIS_meas_ = z_diff.transpose() * S.inverse() * z_diff;
}


/**
 * @brief Get estimate of the current ego vehicle pose from UKF.
 * 
 * @return Estimated pose of the ego vehicle.
 */
Pose UKF::GetPoseEstimate() {
    // Extract x/y-position and yaw angle from current state vector x_
    double px      = x_(0); // x-position of the ego vehicle
    double py      = x_(1); // y-position of the ego vehicle
    double yaw     = x_(3); // yaw angle of the ego vehicle [-PI, +PI]

    // Set pz, pitch and roll to zero (considere only 2D motion)
    double pz = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    // Set current pose
    Pose estimatedPose(
		Point(px, py, pz),
		Rotate(yaw, pitch, roll)
	);

    // Return 
    return estimatedPose;
}


/**
 * @brief Get estimate of the current ego vehicle velocity from UKF.
 * 
 * @return Estimated velocity of the ego vehicle.
 */
double UKF::GetVelocityEstimate() {
    // Return longitudinal velocity of the ego vehicle
    return x_(2);
}


/**
 * @brief Get estimate of the current ego vehicle steering angle from UKF.
 * 
 * @return Estimated steering angle of the ego vehicle.
 */
double UKF::GetSteeringAngleEstimate() {
    // Return steering angle of the ego vehicle
    return x_(5);
}
