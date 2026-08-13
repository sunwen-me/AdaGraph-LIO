#ifndef LI2SUP_SERVICE_BRIDGE_H_
#define LI2SUP_SERVICE_BRIDGE_H_

#include <chrono>
#include <cstdint>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "adagraph/io/map_io.h"
#include "adagraph/localization/localization_map.h"
#include "adagraph/system/slam_system.h"

namespace LI2Sup {

// ROS-only command adapter. All commands use std_srvs/Trigger so the adapter
// does not add a ROS interface package while the lifecycle contract is still
// transitional. The core lifecycle and map-save semantics remain in
// adagraph::system::SlamSystem.
class ServiceBridge final {
 public:
  struct Config {
    std::string start_mapping_service = "/lio/slam/start_mapping";
    std::string start_localization_service =
        "/lio/slam/start_localization";
    std::string finish_trajectory_service = "/lio/slam/finish_trajectory";
    std::string save_map_service = "/lio/slam/save_map";
    std::string stop_service = "/lio/slam/stop";
    std::string reset_service = "/lio/slam/reset";
    std::string manual_relocalization_service =
        "/lio/relocalization/trigger";
    std::string save_map_path;
    double save_poll_period_sec = 0.1;
    adagraph::common::SessionId session_id{1};
    adagraph::common::TrajectoryId trajectory_id{1};
    bool allow_deferred_localization = true;

    [[nodiscard]] std::optional<std::string> validate() const;
  };

  using LocalizationMapProvider = std::function<
      std::shared_ptr<const adagraph::localization::LocalizationMap>()>;
  using ManualRelocalizationHandler =
      std::function<adagraph::system::SystemStatus()>;

  ServiceBridge(rclcpp::Node& node,
                rclcpp::CallbackGroup::SharedPtr callback_group,
                adagraph::system::SlamSystem& system, Config config,
                LocalizationMapProvider localization_map_provider = {},
                ManualRelocalizationHandler manual_relocalization_handler = {});

  ServiceBridge(const ServiceBridge&) = delete;
  ServiceBridge& operator=(const ServiceBridge&) = delete;
  ~ServiceBridge() { stop(); }

  void start();
  void stop();

 private:
  using Trigger = std_srvs::srv::Trigger;

  void handleStartMapping(const Trigger::Request::SharedPtr request,
                          const Trigger::Response::SharedPtr response);
  void handleStartLocalization(const Trigger::Request::SharedPtr request,
                               const Trigger::Response::SharedPtr response);
  void handleFinishTrajectory(const Trigger::Request::SharedPtr request,
                              const Trigger::Response::SharedPtr response);
  void handleSaveMap(const Trigger::Request::SharedPtr request,
                     const Trigger::Response::SharedPtr response);
  void handleStop(const Trigger::Request::SharedPtr request,
                  const Trigger::Response::SharedPtr response);
  void handleReset(const Trigger::Request::SharedPtr request,
                   const Trigger::Response::SharedPtr response);
  void handleManualRelocalization(
      const Trigger::Request::SharedPtr request,
      const Trigger::Response::SharedPtr response);

  [[nodiscard]] adagraph::io::MapSaveRequest makeSaveRequest();
  void loadParameters();
  void pollSave();
  static void setResponse(const adagraph::system::SystemStatus& status,
                          const Trigger::Response::SharedPtr& response);

  rclcpp::Node& node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  adagraph::system::SlamSystem& system_;
  Config config_;
  LocalizationMapProvider localization_map_provider_;
  ManualRelocalizationHandler manual_relocalization_handler_;
  bool started_ = false;

  rclcpp::Service<Trigger>::SharedPtr start_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr start_localization_service_;
  rclcpp::Service<Trigger>::SharedPtr finish_trajectory_service_;
  rclcpp::Service<Trigger>::SharedPtr save_map_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Service<Trigger>::SharedPtr manual_relocalization_service_;
  rclcpp::TimerBase::SharedPtr save_poll_timer_;

  mutable std::mutex save_mutex_;
  std::optional<std::future<adagraph::system::SystemStatus>> pending_save_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SERVICE_BRIDGE_H_
