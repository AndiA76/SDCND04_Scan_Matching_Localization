// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================

// Implementation of an Unscented Kalman Filter class for tracking 2D vehicle
// motion using a continuous turn rate continuous velocity (CTRV) motion model.

#include "Eigen/Dense"
#include "helper.h"
#include "ukf.h"

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DEBUG 0 // Toggle debugging (0) and normal mode (1)


/**
 * @brief Constructor: Initializes a new Unscented Kalman Filter instance to track 2D ego vehicle motion.
 */
UKF::UKF() {
    // Initialization flag: set to true after first call of ProcessMeasurement (default: false)
    is_initialized_ = false;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2.0;
    std_a_square_ = std_a_ * std_a_;
    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 2.0;
    std_yawdd_square_ = std_yawdd_ * std_yawdd_;

    // Lidar scan matching localization measurement noise standard deviation position x in m (cartesian coordinates)
    std_lidar_x_ = 0.15;
    // Lidar scan matching localization measurement noise standard deviation position y in m (cartesian coordinates)
    std_lidar_y_ = 0.15;
    // Lidar scan matching localization measurement noise standard deviation yaw angle in rad (cartesian coordinates)
    std_lidar_yaw_ = 0.5;

    // Dimension of the original state vector using contant turn rate constant velocity model (CTRV)
    n_x_ = 5;
    // Initial state vector for 2D motion: x_ = [px py v yaw_angle yaw_rate]
    x_ = VectorXd::Zero(n_x_);
    // Initial state covariance matrix
    P_ = MatrixXd::Identity(n_x_, n_x_);

    // Dimension of the augmented state vector
    n_x_aug_ = 7;
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

    // Initial Lidar measurement noise covariance matrix
    R_lidar_ = MatrixXd::Zero(3, 3);
    R_lidar_ << std_lidar_x_ * std_lidar_x_, 0, 0,
                0, std_lidar_y_ * std_lidar_y_, 0,
                0, 0, std_lidar_yaw_ * std_lidar_yaw_;

    // Initial Normalized Innovation Squared (NIS) value for Lidar
    NIS_lidar_ = 0.0;
}


/** 
 * @brief Desctructor: Delete current Unscented Kalman Filter instance.
 */
UKF::~UKF() {}


/**
 * @brief Initialize the state vector of the Unscented Kalman Filter with a 2D CTRV model.
 * 
 * @param initialPose Initial pose of the vehicle to be tracked.
 * 
 * @param timestamp_s Current timestamp in seconds.
 * 
 */
void UKF::InitializeState(Pose initialPose, double timestamp_s) {
    // Initialize Kalman Filter state vector using initial ground truth pose
    double px   = pose.position.x;          // x-position of the ego vehicle
    double py   = pose.position.y;          // y-position of the ego vehicle
    double v    = 0.0;                      // velocity of the ego vehicle
    double yaw  = pose.rotation.yaw;        // yaw angle of the ego vehicle
    double yawd = 0.0;                      // yaw rate of the ego vehicle

    // Initialize state vector x
    x_ << px, py, v, yaw, yawd;

    // Initialize time stamp in seconds [s]
    time_s_ = timestamp_s;

#if DEBUG
    std::cout << "Initial Kalman Filter state vector using initial ground truth pose" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px    = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py    = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v     = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw   = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd  = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time  = " << time_s_ << " [s]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif

    // set initialization flag to true
    is_initialized_ = true;
}


/**
   * @brief Unscented Kalman Filter Prediction and Measurement Update Cycle using Lidar Scan Matching.
   * 
   * @param measuredPose Measured pose of the vehicle to be tracked obtained by Lidar scan matching.
   * 
   * @param timestamp_s Timestamp of the latest pose measurement in seconds.
   */
void UKF::UpdateCycle(Pose measuredPose, double timestamp_s) {
    // Measurement process of the Unscented Kalman Filter using Lidar Scan Matching Localization

    // Check if Kalman Filter state has been initialized
    if (!is_initialized_) {
        // Initialize Kalman Filter state using the latest pose measurement
        InitializeState(measuredPose, timestamp_s);

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

#if DEBUG
    std::cout << std::endl;
    std::cout << "Predict next Kalman Filter state vector" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px      = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py      = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v       = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw     = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd    = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time    = " << time_s_ << " [s]" << std::endl;
    std::cout << "delta_t = " << delta_t << " [s]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif

    // Kalman Filter measurement update step:
    // - update the Kalman filter state using pose measurement from Lidar Scan Matching Localization
    // - update the state and measurement noise covariance matrices

    // Measurement update
    UpdateLidar(measuredPose);

#if DEBUG
    std::cout << "Kalman Filter measurement update using Lidar" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px   = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py   = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v    = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw  = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time = " << time_s_ << " [s]" << std::endl;
    std::cout << std::endl;
    std::cout << "NIS  = " << std::fixed << std::setprecision(9) << NIS_lidar_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
}


/**
 * @brief Generate augmented sigma points Xsig_aug_.
 */
void UKF::GenerateSigmaPoints() {
    // Initialize augmented state vector
    x_aug_ = VectorXd::Zero(n_x_aug_); // n_x_aug_ = n_x_ + 2
    x_aug_.head(n_x_) = x_; // original state vector

    // Initialize augumented state covariance matrix
    P_aug_ = MatrixXd::Zero(n_x_aug_, n_x_aug_); // n_x_aug_ = n_x_ + 2
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_(n_x_aug_-2, n_x_aug_-2) = std_a_square_; // covariance of linear acceleration noise
    P_aug_(n_x_aug_-1, n_x_aug_-1) = std_yawdd_square_; // covariance of angular acceleration noise

    // Calculate the square root matrix of the agumented process noise covariance matrix
    MatrixXd L = P_aug_.llt().matrixL(); // use Cholesky decomposition

    // Initialize augumented sigma point matrix
    Xsig_aug_ = MatrixXd::Zero(n_x_aug_, n_sig_);
    Xsig_aug_.col(0) = x_aug_; // set first column
    for (int i = 0; i < n_x_aug_; ++i) {
        // Fill the other columns
        Xsig_aug_.col(i + 1)            = x_aug_ + sig_spread_ * L.col(i);
        Xsig_aug_.col(i + 1 + n_x_aug_) = x_aug_ - sig_spread_ * L.col(i);

        // Yaw angle sigma point normalization modulo +/- M_PI
        while (Xsig_aug_(3, i) > M_PI) Xsig_aug_(3, i) -= M_PI_x_2_;
        while (Xsig_aug_(3, i) < -M_PI) Xsig_aug_(3, i) += M_PI_x_2_;
    }
}


/**
 * @brief Predict augmented sigma points Xsig_pred_.
 * 
 * @param delta_t Time between k and k+1 in s
 */
void UKF::PredictSigmaPoints(double delta_t) {
    // Remark: All states are measured relative to the ego car

    // Loop over all sigma points
    for (int i = 0; i < n_sig_; ++i) {
        // Extract state vector elements from augmented sigma point matrix for better readability
        double px       = Xsig_aug_(0, i); // x-position relative to the ego car
        double py       = Xsig_aug_(1, i); // y-position relative to the ego car
        double v        = Xsig_aug_(2, i); // longitudinal velocity
        double yaw      = Xsig_aug_(3, i); // yaw angle [-M_PI, +M_PI]
        double yawd     = Xsig_aug_(4, i); // yaw rate
        double nu_a     = Xsig_aug_(5, i); // linear accelaration noise
        double nu_yawdd = Xsig_aug_(6, i); // angular acceleration noise

        // Declare predicted state vector comnponents
        double px_pred, py_pred, v_pred, yaw_pred, yawd_pred;

        // Precidct next velocity = const. (assumption)
        v_pred = v;

        // Predict next yaw angle
        yaw_pred = yaw + yawd * delta_t;

        // Predicted yaw angle normalization modulo +/- M_PI
        while (yaw_pred > M_PI) yaw_pred -= M_PI_x_2_;
        while (yaw_pred < -M_PI) yaw_pred += M_PI_x_2_;

        // Predict next yaw rate = const. (assumption)
        yawd_pred = yawd;

        // Predict next x-y-position using CTRV vehicle motion model (avoid division by zero)
        if (fabs(yawd) > 0.001) {
            // Predict next predicted position on circular path
            double v_yawd_ratio = v/yawd;
            px_pred = px + v_yawd_ratio * (sin(yaw_pred) - sin(yaw));
            py_pred = py + v_yawd_ratio * (-cos(yaw_pred) + cos(yaw));
        } else {
            // Predict next predicted position on straight path
            double delta_p = v * delta_t;
            px_pred = px + delta_p * cos(yaw);
            py_pred = py + delta_p * sin(yaw);
        }

        // Initialize process noise (given as noise on linear acceleration nu_a and noise on angular acceleration nu_yawdd)
        double delta_t_square_half = 0.5 * delta_t * delta_t;
        double nu_px   = delta_t_square_half * cos(yaw) * nu_a;
        double nu_py   = delta_t_square_half * sin(yaw) * nu_a;
        double nu_v    = delta_t * nu_a;
        double nu_yaw  = delta_t_square_half * nu_yawdd;
        double nu_yawd = delta_t * nu_yawdd;

        // Add process noise effect to predicted state vector
        px_pred   = px_pred + nu_px;
        py_pred   = py_pred + nu_py;
        v_pred    = v_pred + nu_v;
        yaw_pred  = yaw_pred + nu_yaw;
        yawd_pred = yawd_pred + nu_yawd;

        // Predicted yaw angle normalization modulo +/- M_PI
        while (yaw_pred > M_PI) yaw_pred  -= M_PI_x_2_;
        while (yaw_pred < -M_PI) yaw_pred += M_PI_x_2_;

#if DEBUG
        std::cout << "Process noise effect on the predicted state vector" << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
        std::cout << "nu_px    = 0.5 * delta_t^2 * cos(yaw) * nu_a = " << std::fixed << std::setprecision(9) << nu_px << " [m]" << std::endl;
        std::cout << "nu_py    = 0.5 * delta_t^2 * cos(yaw) * nu_a = " << std::fixed << std::setprecision(9) << nu_py << " [m]" << std::endl;
        std::cout << "nu_v     = delta_t * nu_a                    = " << std::fixed << std::setprecision(9) << nu_v << " [m/s]" << std::endl;
        std::cout << "nu_yaw   = 0.5 * delta_t_square * nu_yawdd   = " << std::fixed << std::setprecision(9) << nu_yaw << " [rad]" << std::endl;
        std::cout << "nu_yawd  = delta_t * nu_yawdd                = " << std::fixed << std::setprecision(9) << nu_yawd << " [rad/s]" << std::endl;
        std::cout << std::endl;
        std::cout << "nu_a     = " << std::fixed << std::setprecision(9) << nu_a << " [m/s^2]" << std::endl;
        std::cout << "nu_yawdd = " << std::fixed << std::setprecision(9) << nu_yawdd << " [rad/s^2]" << std::endl;
        std::cout << std::endl;
        std::cout << "time     = " << std::fixed << std::setprecision(9) << time_s_ << " [s]" << std::endl;
        std::cout << "------------------------------------------------------------------" << std::endl;
#endif

        // Update Xsig_pred
        Xsig_pred_(0, i) = px_pred;
        Xsig_pred_(1, i) = py_pred;
        Xsig_pred_(2, i) = v_pred;
        Xsig_pred_(3, i) = yaw_pred;
        Xsig_pred_(4, i) = yawd_pred;
    }
}


/**
 * @brief Predict mean and covariance of the predicted state.
 */
void UKF::PredictStateMeanAndCovariance() {
    // Remark: All states are measured relative to the ego car

    // Predict state mean
    VectorXd x_pred = VectorXd::Zero(n_x_);
    for (int i = 0; i < n_sig_; ++i) {
        x_pred += weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted yaw angle normalization modulo +/- M_PI
    while (x_pred(3) > M_PI) x_pred(3) -= M_PI_x_2_;
    while (x_pred(3) < -M_PI) x_pred(3) += M_PI_x_2_;

    // Predict state (process noise) covariance matrix
    MatrixXd P_pred = MatrixXd::Zero(n_x_, n_x_);
    for (int i = 0; i < n_sig_; ++i) {
        // State residual
        VectorXd x_diff = Xsig_pred_.col(i) - x_pred;

        // Yaw angle residual normalization modulo +/- M_PI
        while (x_diff(3) > M_PI) x_diff(3) -= M_PI_x_2_;
        while (x_diff(3) < -M_PI) x_diff(3) += M_PI_x_2_;

        // Final predicted state covarinace matrix
        P_pred += weights_(i) * x_diff * x_diff.transpose();
    }
  
    // Store predicted state vector and state covariance matrix
    x_ = x_pred;
    P_ = P_pred;
}


/**
 * @brief Prediction Predicts sigma points, the state, and the state covariance matrix.
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

#if DEBUG
    std::cout << "Prediction" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Augmented sigma points = " << std::fixed << std::setprecision(9) << Xsig_aug_ << std::endl;
    std::cout << "Predicted sigma points = " << std::fixed << std::setprecision(9) << Xsig_pred_ << std::endl;
    std::cout << "Predicted state mean = " << std::fixed << std::setprecision(9) << x_ << std::endl;
    std::cout << "Predicted state covariance = " << std::fixed << std::setprecision(9) << P_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
}


/**
 * @brief Predict and update the state and the state covariance matrix using a Lidar scan matching localization measurement.
 * 
 * @param meas_package The measurement at k+1
 */
void UKF::UpdateLidar(Pose measuredPose) {
    // Set measurement dimension for 2D Lidar scan matching localization (x and y point position and yaw angle)
    int n_z = 3;

    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig_);

    // Create predicted mean measurement
    VectorXd z_pred = VectorXd::Zero(n_z);

    // Create measurement covariance matrix
    MatrixXd S = MatrixXd::Zero(n_z, n_z);

    /* ----------- PREDICT MEASUREMENTS ----------- */

    // Remark: All states are measured relative to the ego car

    // Transform sigma points into measurement space
    for (int i = 0; i < n_sig_; ++i) {
        // Extract states
        double px   = Xsig_pred_(0, i);  // x-position of the ego car
        double py   = Xsig_pred_(1, i);  // y-position of the ego car
        double v    = Xsig_pred_(2, i);  // longitudinal velocity
        double yaw  = Xsig_pred_(3, i);  // yaw angle [-M_PI, +M_PI]
        double yawd = Xsig_pred_(4, i);  // yaw rate

        // Measurement model in Cartesian coordinates
        Zsig(0, i) = Xsig_pred_(0, i);  // x-position (px)
        Zsig(1, i) = Xsig_pred_(1, i);  // y-position (py)
        Zsig(2, i) = Xsig_pred_(3, i);  // yaw angle [-M_PI, +M_PI]
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
    S = S + R_lidar_;

    /* ----- UPDATE STATE MEAN AND COVARIANCE ----- */

    // Remark: All measurements are measured relative to the ego car

    // Get true received measurements
    VectorXd z = meas_package.raw_measurements_;  // true received measurement

    // Create cross correlation matrix between sigma points in state space and in measurement space
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

    // Calculate cross correlation matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
    for (int i = 0; i < n_sig_; ++i) {
        // State residual
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // Yaw angle residual normalization modulo +/- M_PI
        while (x_diff(3) > M_PI) x_diff(3) -= M_PI_x_2_;
        while (x_diff(3) < -M_PI) x_diff(3) += M_PI_x_2_;

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

    // Yaw angle normalization modulo +/- M_PI
    while (x_(3) > M_PI) x_(3) -= M_PI_x_2_;
    while (x_(3) < -M_PI) x_(3) += M_PI_x_2_;

    // Calculate Normalized Innovation Squared (NIS) update for Lidar
    NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}


/**
 * @brief Get current pose estimate from UKF.
 * 
 * @return Pose
 */
Pose UKF::GetPoseEstimate() {
    // Extract states from current vector x_
    double px       = x_(0); // x-position relative to the ego car
    double py       = x_(1); // y-position relative to the ego car
    double v        = x_(2); // longitudinal velocity
    double yaw      = x_(3); // yaw angle [-M_PI, +M_PI]
    double yawd     = x_(4); // yaw rate

    // Set pz, pitch and roll to zero (considere only 2D motion)
    double pz = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    // Set current pose
    Pose pose(
		Point(px, py, pz),
		Rotate(yaw, pitch, roll)
	);
	return pose;
}
