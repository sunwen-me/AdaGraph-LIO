#include "ros/service_bridge.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <utility>

namespace LI2Sup {

std::optional<std::string> ServiceBridge::Config::validate() const {
  const std::string* names[] = {
      &start_mapping_service, &start_localization_service,
      &finish_trajectory_service, &save_map_service, &stop_service,
      &reset_service, &manual_relocalization_service};
  for (const auto* name : names) {
    if (name->empty() || name->front() != '/') {
      return "service names must be absolute ROS names";
    }
  }
  std::set<std::string> unique_names;
  for (const auto* name : names) {
    if (!unique_names.insert(*name).second) {
      return "lifecycle service names must be unique";
    }
  }
  if (!std::isfinite(save_poll_period_sec) || save_poll_period_sec <= 0.0) {
    return "save_poll_period_sec must be finite and positive";
  }
  if (session_id.value() == 0 || trajectory_id.value() == 0) {
    return "session_id and trajectory_id must be non-zero";
  }
  if (!save_map_path.empty()) {
    const std::filesystem::path path(save_map_path);
    if (path.filename().empty()) {
      return "save_map_path must name a file";
    }
  }
  return std::nullopt;
}

ServiceBridge::ServiceBridge(
    rclcpp::Node& node, rclcpp::CallbackGroup::SharedPtr callback_group,
    adagraph::system::SlamSystem& system, Config config,
    LocalizationMapProvider localization_map_provider,
    ManualRelocalizationHandler manual_relocalization_handler)
    : node_(node),
      callback_group_(std::move(callback_group)),
      system_(system),
      config_(std::move(config)),
      localization_map_provider_(std::move(localization_map_provider)),
      manual_relocalization_handler_(
          std::move(manual_relocalization_handler)) {}

void ServiceBridge::start() {
  if (started_) {
    return;
  }
  loadParameters();
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid ServiceBridge config: " + *error);
  }

  start_mapping_service_ = node_.create_service<Trigger>(
      config_.start_mapping_service,
      std::bind(&ServiceBridge::handleStartMapping, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  start_localization_service_ = node_.create_service<Trigger>(
      config_.start_localization_service,
      std::bind(&ServiceBridge::handleStartLocalization, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  finish_trajectory_service_ = node_.create_service<Trigger>(
      config_.finish_trajectory_service,
      std::bind(&ServiceBridge::handleFinishTrajectory, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  save_map_service_ = node_.create_service<Trigger>(
      config_.save_map_service,
      std::bind(&ServiceBridge::handleSaveMap, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  stop_service_ = node_.create_service<Trigger>(
      config_.stop_service,
      std::bind(&ServiceBridge::handleStop, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  reset_service_ = node_.create_service<Trigger>(
      config_.reset_service,
      std::bind(&ServiceBridge::handleReset, this, std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
  manual_relocalization_service_ = node_.create_service<Trigger>(
      config_.manual_relocalization_service,
      std::bind(&ServiceBridge::handleManualRelocalization, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);

  save_poll_timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(config_.save_poll_period_sec),
      [this] { pollSave(); }, callback_group_);
  started_ = true;
}

void ServiceBridge::loadParameters() {
  const auto string_parameter = [this](const char* name,
                                        std::string& value) {
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<std::string>(name, value);
    }
    node_.get_parameter(name, value);
  };
  const auto double_parameter = [this](const char* name, double& value) {
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<double>(name, value);
    }
    node_.get_parameter(name, value);
  };
  const auto integer_parameter = [this](const char* name,
                                         std::uint64_t& value) {
    const auto default_value = static_cast<std::int64_t>(value);
    if (!node_.has_parameter(name)) {
      node_.declare_parameter<std::int64_t>(name, default_value);
    }
    std::int64_t loaded = default_value;
    node_.get_parameter(name, loaded);
    value = loaded > 0 ? static_cast<std::uint64_t>(loaded) : 0;
  };

  string_parameter("lio.lifecycle.start_mapping_service",
                   config_.start_mapping_service);
  string_parameter("lio.lifecycle.start_localization_service",
                   config_.start_localization_service);
  string_parameter("lio.lifecycle.finish_trajectory_service",
                   config_.finish_trajectory_service);
  string_parameter("lio.lifecycle.save_map_service", config_.save_map_service);
  string_parameter("lio.lifecycle.stop_service", config_.stop_service);
  string_parameter("lio.lifecycle.reset_service", config_.reset_service);
  string_parameter("lio.lifecycle.manual_relocalization_service",
                   config_.manual_relocalization_service);
  string_parameter("lio.lifecycle.save_map_path", config_.save_map_path);
  double_parameter("lio.lifecycle.save_poll_period_sec",
                   config_.save_poll_period_sec);

  std::uint64_t session_id = config_.session_id.value();
  std::uint64_t trajectory_id = config_.trajectory_id.value();
  integer_parameter("lio.lifecycle.session_id", session_id);
  integer_parameter("lio.lifecycle.trajectory_id", trajectory_id);
  config_.session_id = adagraph::common::SessionId(session_id);
  config_.trajectory_id = adagraph::common::TrajectoryId(trajectory_id);
  if (!node_.has_parameter("lio.lifecycle.allow_deferred_localization")) {
    node_.declare_parameter<bool>("lio.lifecycle.allow_deferred_localization",
                                  config_.allow_deferred_localization);
  }
  node_.get_parameter("lio.lifecycle.allow_deferred_localization",
                      config_.allow_deferred_localization);
}

void ServiceBridge::stop() {
  if (!started_) {
    return;
  }
  save_poll_timer_.reset();
  start_mapping_service_.reset();
  start_localization_service_.reset();
  finish_trajectory_service_.reset();
  save_map_service_.reset();
  stop_service_.reset();
  reset_service_.reset();
  manual_relocalization_service_.reset();
  std::optional<std::future<adagraph::system::SystemStatus>> pending;
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    pending = std::move(pending_save_);
    started_ = false;
  }
  // Map-save commands are reliable work. Keep the future alive and wait for
  // completion instead of abandoning a worker that may still be writing.
  if (pending) {
    try {
      (void)pending->get();
    } catch (...) {
      // SystemStatus is the normal failure channel; keep teardown noexcept.
    }
  }
}

void ServiceBridge::setResponse(
    const adagraph::system::SystemStatus& status,
    const Trigger::Response::SharedPtr& response) {
  response->success = status.ok();
  response->message = status.ok() ? "ok" : status.message;
}

void ServiceBridge::handleStartMapping(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  setResponse(system_.StartMapping({config_.session_id, config_.trajectory_id}),
              response);
}

void ServiceBridge::handleStartLocalization(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  adagraph::system::LocalizationOptions options;
  options.session_id = config_.session_id;
  options.trajectory_id = config_.trajectory_id;
  options.allow_deferred_map = config_.allow_deferred_localization;
  if (localization_map_provider_) {
    options.map = localization_map_provider_();
  }
  setResponse(system_.StartLocalization(options), response);
}

void ServiceBridge::handleFinishTrajectory(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  setResponse(system_.FinishTrajectory(), response);
}

void ServiceBridge::handleSaveMap(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    if (pending_save_ &&
        pending_save_->wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
      setResponse(adagraph::system::SystemStatus::Error(
                      adagraph::system::SystemStatusCode::kRejected,
                      "a map save is already pending"),
                  response);
      return;
    }
    if (pending_save_) {
      pending_save_->get();
      pending_save_.reset();
    }
  }

  auto request = makeSaveRequest();
  if (!request.valid()) {
    setResponse(adagraph::system::SystemStatus::Error(
                    adagraph::system::SystemStatusCode::kInvalidArgument,
                    "no valid map snapshot or save path is configured"),
                response);
    return;
  }

  auto future = system_.SaveMapAsync(std::move(request));
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    pending_save_ = std::move(future);
  }
  response->success = true;
  response->message = "map save accepted; completion is reported asynchronously";
}

void ServiceBridge::handleStop(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  setResponse(system_.Stop(), response);
}

void ServiceBridge::handleReset(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  setResponse(system_.Reset(), response);
}

void ServiceBridge::handleManualRelocalization(
    const Trigger::Request::SharedPtr /*request*/,
    const Trigger::Response::SharedPtr response) {
  if (!manual_relocalization_handler_) {
    setResponse(adagraph::system::SystemStatus::Error(
                    adagraph::system::SystemStatusCode::kNotConfigured,
                    "manual relocalization handler is not configured"),
                response);
    return;
  }
  try {
    setResponse(manual_relocalization_handler_(), response);
  } catch (const std::exception& error) {
    setResponse(adagraph::system::SystemStatus::Error(
                    adagraph::system::SystemStatusCode::kRejected,
                    error.what()),
                response);
  } catch (...) {
    setResponse(adagraph::system::SystemStatus::Error(
                    adagraph::system::SystemStatusCode::kRejected,
                    "manual relocalization handler threw an unknown exception"),
                response);
  }
}

adagraph::io::MapSaveRequest ServiceBridge::makeSaveRequest() {
  adagraph::io::MapSaveRequest request;
  if (config_.save_map_path.empty()) {
    return request;
  }
  const auto snapshot = system_.Snapshot().map;
  if (!snapshot || snapshot->keyframes.empty()) {
    return request;
  }
  request.snapshot = snapshot;
  request.output_path = config_.save_map_path;
  request.voxel_leaf_size = 0.0;
  return request;
}

void ServiceBridge::pollSave() {
  std::optional<adagraph::system::SystemStatus> result;
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    if (!pending_save_ ||
        pending_save_->wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
      return;
    }
    result = pending_save_->get();
    pending_save_.reset();
  }
  if (result->ok()) {
    RCLCPP_INFO(node_.get_logger(), "SlamSystem map save completed");
  } else {
    RCLCPP_ERROR(node_.get_logger(), "SlamSystem map save failed: %s",
                 result->message.c_str());
  }
}

}  // namespace LI2Sup
