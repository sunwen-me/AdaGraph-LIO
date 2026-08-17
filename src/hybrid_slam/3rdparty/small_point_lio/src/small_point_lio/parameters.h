/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num = 1;
        float min_distance_squared = 0.25F;
        float max_distance_squared = 1.0e6F;
        bool space_downsample = true;
        float space_downsample_leaf_size = 0.5F;

        Eigen::Vector3d gravity{0.0, 0.0, -9.81};
        bool check_satu = true;
        bool fix_gravity_direction = true;
        double satu_acc = 3.0;
        double satu_gyro = 35.0;
        double acc_norm = 9.81;

        double map_resolution = 0.5;
        size_t init_map_size = 10;

        bool extrinsic_est_en = false;
        Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();
        Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();

        double laser_point_cov = 0.01;
        double imu_meas_acc_cov = 0.01;
        double imu_meas_omg_cov = 0.01;
        double velocity_cov = 20.0;
        double omg_cov = 1000.0;
        double acceleration_cov = 500.0;
        double bg_cov = 0.0001;
        double ba_cov = 0.0001;
        double plane_threshold = 0.1;
        double match_sqaured = 81.0;

        bool publish_odometry_without_downsample = false;

        // The algorithm receives a fully constructed value object. ROS or
        // another transport may populate it at an outer adapter boundary.
        void validate() const;
    };

}// namespace small_point_lio
