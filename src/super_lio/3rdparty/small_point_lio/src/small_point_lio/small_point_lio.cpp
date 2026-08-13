/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio.h"

namespace small_point_lio {

    SmallPointLio::SmallPointLio(Parameters configured_parameters)
        : parameters(std::move(configured_parameters)) {
        parameters.validate();
        preprocess.parameters = &parameters;
        estimator.parameters = &parameters;
        estimator.Lidar_T_wrt_IMU = parameters.extrinsic_T.cast<state::value_type>();
        estimator.Lidar_R_wrt_IMU = parameters.extrinsic_R.cast<state::value_type>();
        if (parameters.extrinsic_est_en) {
            estimator.kf.x.offset_T_L_I = parameters.extrinsic_T.cast<state::value_type>();
            estimator.kf.x.offset_R_L_I = parameters.extrinsic_R.cast<state::value_type>();
        }
        Q = estimator.process_noise_cov();
        estimator.imu_acceleration_scale = parameters.gravity.norm() / parameters.acc_norm;

        // init data
        reset();
    }

    void SmallPointLio::reset() {
        preprocess.reset();
        estimator.reset();
        pointcloud_odom_frame.clear();
        frame_dense_points_odom.clear();
        reset_frame_accumulators();
        is_init = false;
    }

    void SmallPointLio::reset_frame_accumulators() {
        matching_points_odom_frame.clear();
        frame_lidar_hessian.setZero();
        frame_lidar_gradient.setZero();
        attempted_point_updates = 0;
        effective_point_updates = 0;
    }

    void SmallPointLio::on_point_cloud_callback(const std::vector<common::Point> &pointcloud) {
        // Upstream publishes registered points in small chunks whenever IMU
        // data advances the filter. AdaGraph, prior-map registration, and the
        // occupancy grid need one coherent LiDAR-frame unit instead. Close
        // the previous aggregate at the next source cloud boundary.
        if (is_init && preprocess.point_deque.empty()) {
            // The upstream event loop stops as soon as the downsampled point
            // queue becomes empty, which can leave a dense tail behind. It
            // would otherwise be attached to the next LiDAR frame. Transform
            // that tail with the latest state before closing this aggregate.
            while (!preprocess.dense_point_deque.empty()) {
                collect_dense_point(preprocess.dense_point_deque.front());
                preprocess.dense_point_deque.pop_front();
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
            if (!frame_dense_points_odom.empty()) {
                publish_frame(time_current);
                frame_dense_points_odom.clear();
                reset_frame_accumulators();
            }
        }
        preprocess.on_point_cloud_callback(pointcloud);
    }

    void SmallPointLio::on_imu_callback(const common::ImuMsg &imu_msg) {
        preprocess.on_imu_callback(imu_msg);
    }

    void SmallPointLio::handle_once() {
        // we need to init small point lio
        if (!is_init) {
            if ((!preprocess.point_deque.empty() || !preprocess.imu_deque.empty()) &&
                preprocess.point_deque.size() >= parameters.init_map_size &&
                (!parameters.fix_gravity_direction || preprocess.imu_deque.size() >= 200)) {
                // init map
                for (const auto &point: preprocess.point_deque) {
                    estimator.ivox->add_point(point.position);
                }
                // fix gravity direction
                if (parameters.fix_gravity_direction) {
                    estimator.kf.x.gravity = Eigen::Matrix<state::value_type, 3, 1>::Zero();
                    for (const auto &imu_msg: preprocess.imu_deque) {
                        estimator.kf.x.gravity += imu_msg.linear_acceleration.cast<state::value_type>();
                    }
                    state::value_type scale = -static_cast<state::value_type>(parameters.gravity.norm()) / estimator.kf.x.gravity.norm();
                    estimator.kf.x.gravity *= scale;
                } else {
                    estimator.kf.x.gravity = parameters.gravity.cast<state::value_type>();
                }
                estimator.kf.x.acceleration = -estimator.kf.x.gravity;
                // init time
                if (preprocess.point_deque.empty()) {
                    time_current = preprocess.imu_deque.back().timestamp;
                } else if (preprocess.imu_deque.empty()) {
                    time_current = preprocess.point_deque.back().timestamp;
                } else {
                    time_current = std::max(preprocess.point_deque.back().timestamp, preprocess.imu_deque.back().timestamp);
                }
                estimator.kf.init_timestamp(time_current);
                // clear data
                preprocess.point_deque.clear();
                preprocess.dense_point_deque.clear();
                preprocess.imu_deque.clear();
                is_init = true;
            }
            return;
        }

        // judge we should do point update or imu update
        bool is_publish_odometry = !preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty() &&
                                   preprocess.imu_deque.front().timestamp < preprocess.point_deque.back().timestamp;
        while (!preprocess.imu_deque.empty() && !preprocess.dense_point_deque.empty() && !preprocess.point_deque.empty()) {
            const common::Point &point_lidar_frame = preprocess.point_deque.front();
            const common::Point &dense_point_lidar_frame = preprocess.dense_point_deque.front();
            const common::ImuMsg &imu_msg = preprocess.imu_deque.front();
            if (dense_point_lidar_frame.timestamp < point_lidar_frame.timestamp && dense_point_lidar_frame.timestamp < imu_msg.timestamp) {
                // collect odom frame pointcloud
                collect_dense_point(dense_point_lidar_frame);

                preprocess.dense_point_deque.pop_front();
            } else if (point_lidar_frame.timestamp < imu_msg.timestamp) {
                // point update
                if (point_lidar_frame.timestamp < time_current) {
                    preprocess.point_deque.pop_front();
                    continue;
                }
                time_current = point_lidar_frame.timestamp;

                // predict
                estimator.kf.predict_state(time_current);

                // update
                estimator.point_lidar_frame = point_lidar_frame.position;
                ++attempted_point_updates;
                const bool point_update_valid = estimator.kf.update_point();
                // h_point always computes point_odom_frame before its
                // correspondence gates, so it is safe to retain for scene
                // statistics even when the update is rejected.
                matching_points_odom_frame.push_back(estimator.point_odom_frame);
                if (point_update_valid) {
                    ++effective_point_updates;
                    const auto &measurement = estimator.last_point_measurement;
                    Eigen::Matrix<double, 6, 1> jacobian;
                    // Point-LIO stores [translation, rotation, extrinsics]
                    // while AdaGraph uses [rotation, translation].
                    jacobian.head<3>() = measurement.H.segment<3>(3).transpose();
                    jacobian.tail<3>() = measurement.H.segment<3>(0).transpose();
                    const double weight = 1.0 / std::max(
                            static_cast<double>(measurement.laser_point_cov), 1e-12);
                    frame_lidar_hessian.noalias() +=
                            jacobian * weight * jacobian.transpose();
                    frame_lidar_gradient.noalias() +=
                            jacobian * weight * static_cast<double>(measurement.z);
                }

                // publish odometry
                if (parameters.publish_odometry_without_downsample) {
                    publish_odometry(time_current);
                }

                // map incremental
                estimator.ivox->add_point(estimator.point_odom_frame);

                preprocess.point_deque.pop_front();
            } else {
                // imu update
                if (imu_msg.timestamp < time_current) {
                    preprocess.imu_deque.pop_front();
                    continue;
                }
                time_current = imu_msg.timestamp;

                // predict
                estimator.kf.predict_state(time_current);
                estimator.kf.predict_cov(time_current, Q);

                // update
                estimator.angular_velocity = imu_msg.angular_velocity.cast<state::value_type>();
                estimator.linear_acceleration = imu_msg.linear_acceleration.cast<state::value_type>();
                estimator.kf.update_imu();

                preprocess.imu_deque.pop_front();
            }
        }

        if (is_publish_odometry) {
            if (!parameters.publish_odometry_without_downsample) {
                publish_odometry(time_current);
            }
            if (!pointcloud_odom_frame.empty()) {
                if (pointcloud_callback) {
                    pointcloud_callback(pointcloud_odom_frame);
                }
                pointcloud_odom_frame.clear();
            }
        }
    }

    void SmallPointLio::set_pointcloud_callback(const std::function<void(const std::vector<Eigen::Vector3f> &pointcloud)> &pointcloud_callback) {
        this->pointcloud_callback = pointcloud_callback;
    }

    void SmallPointLio::set_odometry_callback(const std::function<void(const common::Odometry &odometry)> &odometry_callback) {
        this->odometry_callback = odometry_callback;
    }

    void SmallPointLio::set_frame_callback(std::function<void(FrontendFrame &&frame)> frame_callback) {
        this->frame_callback = std::move(frame_callback);
    }

    void SmallPointLio::collect_dense_point(const common::Point &point_lidar_frame) {
        Eigen::Matrix<state::value_type, 3, 1> point_imu_frame;
        if (parameters.extrinsic_est_en) {
            point_imu_frame = estimator.kf.x.offset_R_L_I *
                    point_lidar_frame.position.cast<state::value_type>() +
                    estimator.kf.x.offset_T_L_I;
        } else {
            point_imu_frame = estimator.Lidar_R_wrt_IMU *
                    point_lidar_frame.position.cast<state::value_type>() +
                    estimator.Lidar_T_wrt_IMU;
        }
        const Eigen::Vector3f point_odom =
                (estimator.kf.x.rotation * point_imu_frame +
                 estimator.kf.x.position).cast<float>();
        // The standalone upstream publisher and the AdaGraph frame callback
        // are independent consumers.  Avoid retaining two copies unless both
        // callbacks are actually installed.
        if (pointcloud_callback) {
            pointcloud_odom_frame.push_back(point_odom);
        }
        if (frame_callback) {
            frame_dense_points_odom.push_back(point_odom);
        }
    }

    common::Odometry SmallPointLio::make_odometry(double timestamp) const {
        common::Odometry odometry;
        odometry.timestamp = timestamp;
        odometry.position = estimator.kf.x.position.cast<double>();
        odometry.velocity = estimator.kf.x.velocity.cast<double>();
        odometry.orientation = estimator.kf.x.rotation.cast<double>();
        odometry.angular_velocity = estimator.kf.x.omg.cast<double>();
        return odometry;
    }

    void SmallPointLio::publish_odometry(double timestamp) {
        if (odometry_callback) {
            odometry_callback(make_odometry(timestamp));
        }
    }

    void SmallPointLio::publish_frame(double timestamp) {
        if (!frame_callback) {
            return;
        }

        FrontendFrame frame;
        frame.odometry = make_odometry(timestamp);
        frame.dense_points_odom = std::move(frame_dense_points_odom);
        frame.matching_points_odom = std::move(matching_points_odom_frame);
        frame.lidar_hessian = frame_lidar_hessian;
        frame.lidar_gradient = frame_lidar_gradient;
        frame.attempted_point_updates = attempted_point_updates;
        frame.effective_point_updates = effective_point_updates;
        frame_callback(std::move(frame));
    }

    void SmallPointLio::flush_frame() {
        if (!is_init) {
            return;
        }
        while (!preprocess.dense_point_deque.empty()) {
            collect_dense_point(preprocess.dense_point_deque.front());
            preprocess.dense_point_deque.pop_front();
        }
        if (!pointcloud_odom_frame.empty()) {
            if (pointcloud_callback) {
                pointcloud_callback(pointcloud_odom_frame);
            }
            pointcloud_odom_frame.clear();
        }
        if (!frame_dense_points_odom.empty()) {
            publish_frame(time_current);
            reset_frame_accumulators();
        }
    }

}// namespace small_point_lio
