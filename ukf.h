// ============================================================================
//  Project 4:  Scan Matching Localization (Self-Driving Car Engineer Program)
//  Authors:    Andreas Albrecht
// 
//  Copyright © 2022 Andreas Albrecht
// ============================================================================

// Declaration of an Unscented Kalman Filter class for vehicle pose estimation

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
     * @brief Constructor.
     * 
     */
    UKF();

    /**
     * @brief Destructor.
     * 
     */
    virtual ~UKF();

    // Initialize Unscented Kalman Filter state.
    /**
     * @brief Initialize Unscented Kalman Filter state.
     * 
     * @param initialPose Initial pose of the vehicle to be tracked.
     * 
     * @param timestamp_s Timestamp in seconds.
     * 
     */
    void InitializeState(Pose initialPose, double timestamp_s);

    /**
     * @brief Unscented Kalman Filter Prediction and Measurement Update Cycle using Lidar Scan Matching.
     * 
     * @param measuredPose Measured pose of the vehicle to be tracked obtained by Lidar scan matching.
     * 
     * @param timestamp_s Current timestamp in seconds.
     * 
     */
    void UpdateCycle(Pose measuredPose, double timestamp_s);

    /**
     * @brief Generate sigma points for the augmented state vector.
     * 
     */
    void GenerateSigmaPoints(); // AugmentSigmaPoints()

    /**
     * @brief Predict sigma points for the augmented state vector.
     * 
     * @param delta_t Time between k and k+1 in s
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
     * @brief Predict and update the state and the state covariance matrix using a Lidar measurement
     * 
     * @param measuredPose The measurement ego vehicle pose at k+1
     */
    void UpdateLidar(Pose measuredPose);

    /**
     * @brief Get current pose estimate from UKF.
     * 
     * @return estimatedPose
     */
    Pose GetPoseEstimate();

    /* STATIC CONSTANTS */
    
    // 2 * PI
    static constexpr double M_PI_X_2 = 2 * M_PI;

    /* CLASS MEMBER VARIALBES */

    // Initialization flag: Initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // Timestamp of the previous step in s when the state is true
    long long time_s_;

    // State vector dimension
    int n_x_;

    // State vector: [x-position, y-position, longitudinal velocity, yaw angle, yaw rate] in SI units and rad
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

    // Process noise standard deviation longitudinal acceleration in m/s^2 (cartesian coordinates)
    double std_a_;
    double std_a_square_;

    // Process noise standard deviation yaw acceleration in rad/s^2 (cartesian coordinates)
    double std_yawdd_;
    double std_yawdd_square_;

    // Lidar scan matching localization measurement noise standard deviation position x in m (cartesian coordinates)
    double std_lidar_x_;

    // Lidar scan matching localization measurement noise standard deviation position y in m (cartesian coordinates)
    double std_lidar_y_;

    // Lidar scan matching localization measurement noise standard deviation yaw angle in rad (cartesian coordinates)
    double std_lidar_yaw_;

    // Lidar scan matching localization measurement noise covariance matrix
    Eigen::MatrixXd R_lidar_;

    // Normalized Innovation Squared (NIS) value for Lidar scan matching localization measurement
    double NIS_lidar_;
};

#endif  // UKF_H
