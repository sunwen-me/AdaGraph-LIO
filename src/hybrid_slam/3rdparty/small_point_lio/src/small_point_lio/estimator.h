/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "eskf.h"
#include "parameters.h"
#include "small_ivox.h"
#include <pch.h>

namespace small_point_lio {

    class Estimator {
    public:
        // for common
        Parameters *parameters = nullptr;
        eskf kf;
        // for h_point
        // `prior_ivox` is an immutable recovery seed. `ivox` is the current
        // trajectory's writable online layer; existing callers intentionally
        // keep this name for the online map to limit vendor surface churn.
        std::shared_ptr<SmallIVox> prior_ivox;
        std::shared_ptr<SmallIVox> ivox;
        Eigen::Matrix<state::value_type, 3, 1> Lidar_T_wrt_IMU;
        Eigen::Matrix<state::value_type, 3, 3> Lidar_R_wrt_IMU;
        Eigen::Vector3f point_lidar_frame;
        Eigen::Vector3f point_odom_frame;
        std::vector<Eigen::Vector3f> nearest_points;
        std::vector<Eigen::Vector3f> prior_nearest_points;
        std::vector<Eigen::Vector3f> online_nearest_points;
        // for h_imu
        Eigen::Matrix<state::value_type, 3, 1> angular_velocity;
        Eigen::Matrix<state::value_type, 3, 1> linear_acceleration;
        double imu_acceleration_scale;
        // The HybridSLAM adapter consumes the same point-to-plane observation
        // that was accepted by the filter.  Keeping the last result here
        // avoids re-running nearest-neighbour and plane fitting work.
        point_measurement_result last_point_measurement{};

        Estimator();

        void reset();

        void set_prior_map(const std::vector<Eigen::Vector3f>& points);

        void clear_prior_map();

        void clear_online_map();

        [[nodiscard]] Eigen::Matrix<state::value_type, state::DIM, state::DIM> process_noise_cov() const;

        void h_point(const state &s, point_measurement_result &measurement_result);

        void h_imu(const state &s, imu_measurement_result &measurement_result);
    };

}// namespace small_point_lio
