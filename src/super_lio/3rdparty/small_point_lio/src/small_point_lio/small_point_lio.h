/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

namespace small_point_lio {

    class SmallPointLio {
    public:
        struct MemoryStats {
            SmallIVox::MemoryStats ivox;
            Preprocess::MemoryStats preprocess;
            size_t frame_vector_backing_bytes = 0;
        };

        struct FrontendFrame {
            common::Odometry odometry;
            // Registered, motion-compensated points in the odom frame.
            std::vector<Eigen::Vector3f> dense_points_odom;
            // Spatially downsampled points used by Point-LIO.  Keep these in
            // odom here so the realtime estimator does not pay for a second
            // coordinate transform.  Downstream consumers transform them
            // lazily, and only when a backend keyframe needs the cloud.
            std::vector<Eigen::Vector3f> matching_points_odom;
            // Aggregated point-to-plane information in AdaGraph tangent
            // order [rotation, translation].
            Eigen::Matrix<double, 6, 6> lidar_hessian =
                    Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> lidar_gradient =
                    Eigen::Matrix<double, 6, 1>::Zero();
            std::size_t attempted_point_updates = 0;
            std::size_t effective_point_updates = 0;
        };

    private:
        Parameters parameters;
        Preprocess preprocess;
        Estimator estimator;
        double time_current = 0.0;
        std::vector<Eigen::Vector3f> pointcloud_odom_frame;
        std::vector<Eigen::Vector3f> frame_dense_points_odom;
        std::vector<Eigen::Vector3f> matching_points_odom_frame;
        std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> pointcloud_callback;
        std::function<void(const common::Odometry &odometry)> odometry_callback;
        std::function<void(FrontendFrame &&frame)> frame_callback;
        Eigen::Matrix<double, 6, 6> frame_lidar_hessian =
                Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> frame_lidar_gradient =
                Eigen::Matrix<double, 6, 1>::Zero();
        std::size_t attempted_point_updates = 0;
        std::size_t effective_point_updates = 0;
        bool is_init = false;

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(Parameters parameters);

        void reset();

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

        void set_frame_callback(std::function<void(FrontendFrame &&frame)> frame_callback);

        // Transitional construction adapter access. The framework validates
        // a ROS-independent config immediately after the vendor loader runs;
        // Phase 8 will move ROS parameter loading outside this class.
        [[nodiscard]] const Parameters &get_parameters() const noexcept {
            return parameters;
        }

        // Finish the currently accumulated source scan.  This is used during
        // an orderly shutdown so the final scan is not silently discarded.
        void flush_frame();

        [[nodiscard]] MemoryStats memory_stats() const noexcept;

    private:
        [[nodiscard]] common::Odometry make_odometry(double timestamp) const;

        void publish_odometry(double timestamp);

        void publish_frame(double timestamp);

        void reset_frame_accumulators();

        void collect_dense_point(const common::Point &point_lidar_frame);
    };

}// namespace small_point_lio
