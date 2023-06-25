/*
 * brief: use eskf to fuse imu and gps data
 * reference: Sola.Quaternion kinematics for the error-state Kalman filter.2017
 * author: chennuo0125@163.com
 * date: 20201018
 */
#ifndef _IMU_UWB_FUSION_H_
#define _IMU_UWB_FUSION_H_

#include <iostream>
#include <vector>
#include "types.h"
//#include "wgs_conversions.h"

using namespace std;

namespace Fusion
{

class ImuUwbFusion
{
public:
    ImuUwbFusion();
    ~ImuUwbFusion();

    // use imu datas at start to initial initial pose
    // ref: https://github.com/daniilidis-group/msckf_mono
    void imuInit(const vector<ImuData<double>> &imu_datas);

    // config imu variance
    void cfgImuVar(double sigma_an, double sigma_wn, double sigma_aw, double sigma_ww);

    // config reference uwb position 
    void cfgRefUwb(double position_x, double position_y, double position_z);

    // nominal state update, refer formula (156)
    void
    updateNominalState(const ImuData<double> &last_imu_data, const ImuData<double> &imu_data);

    // calculate jacobi Fx, refer formula (166)
    void calcF(const ImuData<double> &imu_data, double dt);

    // update noise covariance matrix
    void updateQ(double dt);

    // eskf predict stage
    void imuPredict(const ImuData<double> &last_imu_data, const ImuData<double> &imu_data);

    // update error state jacobi
    void updateH();

    // update measure noise covariance matrix
    void updateV(const UwbData<double> &uwb_data);

    // eskf measure update stage
    // imu_datas pair: fisrt->imu_data second->dt
    void uwbUpdate(const UwbData<double> &uwb_data, const vector<ImuData<double>> &imu_datas);

    // get result
    State<double> getState();
    State<double> getNominalState();

    // set state
    void recoverState(const Fusion::State<double> &last_updated_state);

private:
    // sensor noise
    double sigma_an_2_;
    double sigma_wn_2_;
    double sigma_aw_2_;
    double sigma_ww_2_;

    // gps reference latitude and longitute
    double ref_position_x_;
    double ref_position_y_;
    double ref_position_z_;
    //WgsConversions gps_converter_;

    Eigen::Vector3d g_;                //gravity
    State<double> no_state_;           //nominal state
    State<double> ac_state_;           //actual state
    Eigen::Matrix<double, 15, 15> P_;  // actual state covariance
    Eigen::Matrix<double, 15, 15> Fx_; //predict function jacobi for state
    Eigen::Matrix<double, 15, 12> Fi_; //predict function noise jacobi for state
    Eigen::Matrix<double, 12, 12> Qi_; //covariances matrixs of perturbation impulses
    Eigen::Matrix<double, 3, 15> H_;   // measure function jacobi for error state
    Eigen::Matrix<double, 3, 16> Hx_;  //measure function jacobi for state
    Eigen::Matrix<double, 3, 3> V_;    //measure variance
};

} // namespace Fusion

#endif
