/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "parameters.h"

#include <cmath>
#include <string>
#include <stdexcept>

namespace small_point_lio {

    void Parameters::validate() const {
        const auto require_finite = [](const double value,
                                       const char *name) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument(std::string(name) +
                                            " must be finite");
            }
        };
        if (point_filter_num <= 0) {
            throw std::invalid_argument("point_filter_num must be positive");
        }
        if (!std::isfinite(min_distance_squared) ||
            !std::isfinite(max_distance_squared) ||
            min_distance_squared < 0.0F ||
            max_distance_squared <= min_distance_squared) {
            throw std::invalid_argument("invalid point distance bounds");
        }
        if (space_downsample &&
            (!std::isfinite(space_downsample_leaf_size) ||
             space_downsample_leaf_size <= 0.0F)) {
            throw std::invalid_argument(
                "space_downsample_leaf_size must be positive");
        }
        if (!gravity.allFinite() || gravity.norm() <= 0.0) {
            throw std::invalid_argument("gravity must be finite and non-zero");
        }
        require_finite(acc_norm, "acc_norm");
        if (acc_norm <= 0.0) {
            throw std::invalid_argument("acc_norm must be positive");
        }
        if (map_resolution <= 0.0 || init_map_size == 0) {
            throw std::invalid_argument("invalid map initialization parameters");
        }
        if (!extrinsic_T.allFinite() || !extrinsic_R.allFinite()) {
            throw std::invalid_argument("extrinsic must be finite");
        }
        require_finite(laser_point_cov, "laser_point_cov");
        require_finite(plane_threshold, "plane_threshold");
        require_finite(match_sqaured, "match_sqaured");
        if (laser_point_cov <= 0.0 || plane_threshold <= 0.0 ||
            match_sqaured <= 0.0) {
            throw std::invalid_argument("invalid Point-LIO covariance thresholds");
        }
    }

}// namespace small_point_lio
