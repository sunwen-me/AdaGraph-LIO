/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <atomic>
#include <cmath>

#include "common/common.h"
#include "estimator.h"
#include "parameters.h"
#include "preprocess.h"
#include <pch.h>

namespace small_point_lio {

    class SmallPointLio {
    public:
        struct MemoryStats {
            SmallIVox::MemoryStats prior_ivox;
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
            // Aggregated point-to-plane information in HybridSLAM tangent
            // order [rotation, translation].
            Eigen::Matrix<double, 6, 6> lidar_hessian =
                    Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> lidar_gradient =
                    Eigen::Matrix<double, 6, 1>::Zero();
            std::size_t attempted_point_updates = 0;
            std::size_t effective_point_updates = 0;
        };

        struct RecoveryState {
            double timestamp = 0.0;
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
            Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
            Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
            Eigen::Matrix<state::value_type, state::DIM, state::DIM>
                covariance = Eigen::Matrix<state::value_type, state::DIM,
                                           state::DIM>::Identity() * 0.01;
            bool use_covariance = false;
            bool reinitialize_imu = false;

            [[nodiscard]] bool valid() const noexcept {
                return std::isfinite(timestamp) && position.allFinite() &&
                       rotation.allFinite() && velocity.allFinite() &&
                       gyro_bias.allFinite() && accel_bias.allFinite() &&
                       gravity.allFinite() && gravity.norm() > 1e-6 &&
                       covariance.allFinite();
            }
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
        std::atomic<bool> map_write_enabled{true};

    public:
        Eigen::Matrix<state::value_type, state::DIM, state::DIM> Q;

        explicit SmallPointLio(Parameters parameters);

        void reset();

        // Starts a new local-estimator segment. The supplied seed remains in
        // the read-only prior layer; online insertions continue to use ivox.
        void reset_and_seed(const RecoveryState& state,
                            const std::vector<Eigen::Vector3f>& seed_points);

        void on_point_cloud_callback(const std::vector<common::Point> &pointcloud);

        void on_imu_callback(const common::ImuMsg &imu_msg);

        void handle_once();

        void set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback);

        void set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback);

        void set_frame_callback(std::function<void(FrontendFrame &&frame)> frame_callback);

        // Admission gate used by the downstream health monitor. It affects
        // future iVox insertions and never blocks estimator processing.
        void set_map_write_enabled(bool enabled) noexcept {
            map_write_enabled.store(enabled, std::memory_order_release);
        }

        // Recovery seed lifecycle. Call only while typed ingress is disabled;
        // the prior layer is queried but never receives online point inserts.
        void set_prior_map(const std::vector<Eigen::Vector3f>& points);

        void clear_prior_map();

        void clear_online_map();

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
