#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "adagraph/system/dataset_runner.h"
#include "lio/point_lio_frontend.h"
#include "lio/super_lio.h"
#include "lio/system/legacy_downstream_frame.h"

namespace {

using adagraph::common::Timestamp;
using adagraph::geometry::FrameId;
using adagraph::sensor::ImuData;
using adagraph::sensor::PointCloudBuffer;
using adagraph::sensor::PointCloudFrame;
using adagraph::sensor::PointXYZIT;
using adagraph::system::DatasetInputSource;
using adagraph::system::DatasetPacket;
using adagraph::system::DatasetRunMode;
using adagraph::system::DatasetRunOptions;
using adagraph::system::DatasetRunner;
using adagraph::system::SlamSystem;
using adagraph::system::SystemStatus;
using adagraph::system::SystemStatusCode;
using adagraph::system::TextDatasetInputSource;

class VectorSource final : public DatasetInputSource {
 public:
  explicit VectorSource(std::vector<DatasetPacket> packets)
      : packets_(std::move(packets)) {}

  std::optional<DatasetPacket> next() override {
    if (index_ >= packets_.size()) {
      return std::nullopt;
    }
    return std::move(packets_[index_++]);
  }

  void close() noexcept override { closed_ = true; }
  [[nodiscard]] bool closed() const noexcept { return closed_; }

 private:
  std::vector<DatasetPacket> packets_;
  std::size_t index_ = 0;
  bool closed_ = false;
};

PointCloudFrame MakeCloud(std::int64_t start_ns) {
  return PointCloudFrame(
      Timestamp::FromNanoseconds(start_ns),
      Timestamp::FromNanoseconds(start_ns + 10), FrameId::kLidar,
      PointCloudBuffer{{1.0F, 2.0F, 3.0F, 0.0F, 0}});
}

TEST(DatasetRunner, DispatchesOrderedPacketsAndFinishesTrajectory) {
  SlamSystem system;
  int imu_count = 0;
  int cloud_count = 0;
  system.setImuSink([&](ImuData imu) {
    ++imu_count;
    return imu.valid() ? SystemStatus::Ok()
                       : SystemStatus::Error(SystemStatusCode::kRejected,
                                             "invalid imu");
  });
  system.setPointCloudSink([&](PointCloudFrame&& cloud) {
    ++cloud_count;
    return cloud.valid() && !cloud.points().empty()
               ? SystemStatus::Ok()
               : SystemStatus::Error(SystemStatusCode::kRejected,
                                     "invalid cloud");
  });

  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(1);
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.8);
  imu.angular_velocity_radps.setZero();

  std::vector<DatasetPacket> packets;
  packets.emplace_back(DatasetPacket{imu});
  packets.emplace_back(DatasetPacket{MakeCloud(2)});
  packets.emplace_back(DatasetPacket{MakeCloud(20)});
  VectorSource source(std::move(packets));
  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_TRUE(result.status.ok()) << result.status.message;
  EXPECT_TRUE(result.reached_end_of_input);
  EXPECT_TRUE(result.finished_trajectory);
  EXPECT_EQ(result.packets_seen, 3U);
  EXPECT_EQ(result.imu_packets, 1U);
  EXPECT_EQ(result.point_cloud_packets, 2U);
  EXPECT_EQ(result.rejected_packets, 0U);
  EXPECT_EQ(imu_count, 1);
  EXPECT_EQ(cloud_count, 2);
  EXPECT_TRUE(source.closed());
}

TEST(DatasetRunner, RejectsNonMonotonicInputWithoutReordering) {
  SlamSystem system;
  system.setImuSink([](ImuData) { return SystemStatus::Ok(); });

  ImuData first;
  first.timestamp = Timestamp::FromNanoseconds(10);
  first.linear_acceleration_mps2.setZero();
  first.angular_velocity_radps.setZero();
  ImuData second = first;
  second.timestamp = Timestamp::FromNanoseconds(9);
  std::vector<DatasetPacket> packets;
  packets.emplace_back(DatasetPacket{first});
  packets.emplace_back(DatasetPacket{second});
  VectorSource source(std::move(packets));

  DatasetRunOptions options;
  options.mode = DatasetRunMode::kMapping;
  DatasetRunner runner(system);
  const auto result = runner.Run(source, options);

  EXPECT_EQ(result.status.code, SystemStatusCode::kInvalidArgument);
  EXPECT_FALSE(result.reached_end_of_input);
  EXPECT_EQ(result.packets_seen, 2U);
  EXPECT_EQ(result.rejected_packets, 1U);
  EXPECT_TRUE(source.closed());
}

TEST(DatasetRunner, PumpsOnceAfterInputEndsAndFinishesActiveSession) {
  SlamSystem system;
  int processed = 0;
  bool finished = false;
  system.setImuSink([](ImuData imu) {
    return imu.valid() ? SystemStatus::Ok()
                       : SystemStatus::Error(SystemStatusCode::kRejected,
                                             "invalid imu");
  });
  system.setProcessHandler([&] {
    ++processed;
    return SystemStatus::Ok();
  });
  system.setFinishTrajectoryHandler([&] {
    finished = true;
    return SystemStatus::Ok();
  });

  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(100);
  imu.linear_acceleration_mps2.setZero();
  imu.angular_velocity_radps.setZero();
  std::vector<DatasetPacket> packets;
  packets.emplace_back(DatasetPacket{imu});
  VectorSource source(std::move(packets));

  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_TRUE(result.status.ok()) << result.status.message;
  EXPECT_EQ(processed, 2);
  EXPECT_TRUE(finished);
  EXPECT_TRUE(result.finished_trajectory);
}

TEST(DatasetRunner, FinishesSessionAfterProcessingFailure) {
  SlamSystem system;
  bool finished = false;
  system.setImuSink([](ImuData imu) {
    return imu.valid() ? SystemStatus::Ok()
                       : SystemStatus::Error(SystemStatusCode::kRejected,
                                             "invalid imu");
  });
  system.setProcessHandler([] {
    return SystemStatus::Error(SystemStatusCode::kRejected,
                               "frontend pump failed");
  });
  system.setFinishTrajectoryHandler([&] {
    finished = true;
    return SystemStatus::Ok();
  });

  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(100);
  imu.linear_acceleration_mps2.setZero();
  imu.angular_velocity_radps.setZero();
  std::vector<DatasetPacket> packets;
  packets.emplace_back(DatasetPacket{imu});
  VectorSource source(std::move(packets));

  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_EQ(result.status.code, SystemStatusCode::kRejected);
  EXPECT_EQ(result.status.message, "frontend pump failed");
  EXPECT_TRUE(finished);
  EXPECT_TRUE(result.finished_trajectory);
}

TEST(DatasetRunner, PropagatesStartFailureBeforeReadingInput) {
  SlamSystem system;
  VectorSource source({});
  DatasetRunner runner(system);
  DatasetRunOptions options;
  options.mode = DatasetRunMode::kLocalization;
  const auto result = runner.Run(source, options);

  EXPECT_EQ(result.status.code, SystemStatusCode::kInvalidArgument);
  EXPECT_EQ(result.packets_seen, 0U);
  EXPECT_TRUE(source.closed());
}

std::filesystem::path TestDatasetPath(const std::string& suffix) {
  return std::filesystem::temp_directory_path() /
         ("adagraph_text_dataset_runner_" + suffix + ".txt");
}

TEST(TextDatasetInputSource, ParsesTypedPacketsWithoutRos) {
  const auto path = TestDatasetPath("valid");
  {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << "# typed interchange format\n"
           << "imu 1.0 0 0 9.8 0.1 0.2 0.3\n"
           << "cloud 1.1 1.2 2 1 2 3 0.5 0.0 4 5 6 0.7 0.1\n";
  }

  TextDatasetInputSource source(path);
  auto imu_packet = source.next();
  ASSERT_TRUE(imu_packet);
  ASSERT_TRUE(std::holds_alternative<ImuData>(imu_packet->payload));
  ASSERT_TRUE(imu_packet->timestamp());
  EXPECT_EQ(imu_packet->timestamp()->nanoseconds(), 1'000'000'000);

  auto cloud_packet = source.next();
  ASSERT_TRUE(cloud_packet);
  ASSERT_TRUE(std::holds_alternative<PointCloudFrame>(cloud_packet->payload));
  const auto& cloud = std::get<PointCloudFrame>(cloud_packet->payload);
  ASSERT_EQ(cloud.points().size(), 2U);
  EXPECT_FLOAT_EQ(cloud.points()[1].z, 6.0F);

  EXPECT_FALSE(source.next());
  EXPECT_FALSE(source.error());
  source.close();
  std::error_code error;
  std::filesystem::remove(path, error);
  EXPECT_FALSE(error);
}

TEST(TextDatasetInputSource, ReportsMalformedInputToRunner) {
  const auto path = TestDatasetPath("invalid");
  {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << "imu 1.0 0 0 9.8 0 0\n";
  }

  TextDatasetInputSource source(path);
  SlamSystem system;
  system.setImuSink([](ImuData) { return SystemStatus::Ok(); });
  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_EQ(result.status.code, SystemStatusCode::kInvalidArgument);
  EXPECT_FALSE(result.reached_end_of_input);
  EXPECT_EQ(result.packets_seen, 0U);
  ASSERT_TRUE(source.error());
  EXPECT_NE(source.error()->find("line 1"), std::string::npos);
  std::error_code error;
  std::filesystem::remove(path, error);
  EXPECT_FALSE(error);
}

TEST(DatasetRunner, DrivesRealSuperLioFrontendWithoutRos) {
  const auto path = TestDatasetPath("super_lio_frontend");
  {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << std::fixed << std::setprecision(2);

    // Keep the stream ordered by packet timestamp.  Each scan is emitted
    // after the IMU sample at its end time, which is the same ordering used by
    // SensorBridge -> SlamSystem in the ROS composition.
    const std::vector<PointXYZIT> points = {
        {0.0F, 0.0F, 0.0F, 0.0F, 0},
        {1.0F, 0.0F, 0.0F, 0.0F, 0},
        {0.0F, 1.0F, 0.0F, 0.0F, 0},
        {0.0F, 0.0F, 1.0F, 0.0F, 0},
        {1.0F, 1.0F, 0.0F, 0.0F, 0},
    };
    for (int tick = 0; tick <= 600; ++tick) {
      const double time = static_cast<double>(tick) / 100.0;
      output << "imu " << time << " 0 0 9.81 0 0 0\n";
      if (tick == 0 || tick % 100 != 0) {
        continue;
      }
      const int scan_index = tick / 100;
      output << "cloud " << time << " " << time << " " << points.size();
      for (const auto& point : points) {
        output << " " << point.x << " " << point.y << " " << point.z
               << " " << point.intensity << " 0.0";
      }
      output << "\n";
      ASSERT_LE(scan_index, 6);
    }
  }

  LI2Sup::SuperLIO frontend;
  std::size_t frontend_results = 0;
  std::size_t downstream_frames = 0;
  frontend.setFrontendResultCallback(
      [&frontend_results](const adagraph::frontend::FrontendResult& result) {
        ++frontend_results;
        EXPECT_EQ(result.implementation,
                  adagraph::frontend::Implementation::kSuperLio);
        EXPECT_TRUE(result.local_pose.valid());
      });
  frontend.setDownstreamFrameCallback(
      [&downstream_frames](LI2Sup::LegacySuperFrame&& frame) {
        EXPECT_TRUE(frame.valid());
        ++downstream_frames;
        LI2Sup::LegacySuperBuffers reusable;
        reusable.full_body_cloud = std::move(frame.full_body_cloud);
        reusable.downsampled_body_cloud =
            std::move(frame.downsampled_body_cloud);
        reusable.observation_terms = std::move(frame.observation_terms);
        reusable.full_body_cloud->clear();
        reusable.downsampled_body_cloud->clear();
        return reusable;
      });
  ASSERT_NO_THROW(frontend.init());

  SlamSystem system;
  system.setImuSink([&frontend](ImuData imu) {
    return frontend.enqueueImu(std::move(imu))
               ? SystemStatus::Ok()
               : SystemStatus::Error(SystemStatusCode::kRejected,
                                     "Super-LIO IMU queue rejected input");
  });
  system.setPointCloudSink([&frontend](PointCloudFrame&& cloud) {
    return frontend.enqueuePointCloud(std::move(cloud))
               ? SystemStatus::Ok()
               : SystemStatus::Error(SystemStatusCode::kRejected,
                                     "Super-LIO lidar queue rejected input");
  });
  system.setProcessHandler([&frontend] {
    frontend.process();
    return SystemStatus::Ok();
  });
  system.setMappingStartHandler(
      [&frontend](const adagraph::system::MappingOptions&) {
    frontend.setTypedInputEnabled(true);
    return SystemStatus::Ok();
  });
  system.setFinishTrajectoryHandler([&frontend] {
    frontend.setTypedInputEnabled(false);
    frontend.drainTypedInput();
    return SystemStatus::Ok();
  });

  TextDatasetInputSource source(path);
  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_TRUE(result.status.ok()) << result.status.message;
  EXPECT_TRUE(result.reached_end_of_input);
  EXPECT_TRUE(result.finished_trajectory);
  EXPECT_EQ(result.point_cloud_packets, 6U);
  EXPECT_GT(frontend_results, 0U);
  EXPECT_EQ(frontend_results, downstream_frames);

  std::error_code error;
  std::filesystem::remove(path, error);
  EXPECT_FALSE(error);
}

TEST(DatasetRunner, DrivesRealPointLioFrontendWithoutRos) {
  const auto path = TestDatasetPath("point_lio_frontend");
  {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << std::fixed << std::setprecision(3);

    const std::vector<PointXYZIT> points = {
        {0.5F, 0.0F, 0.0F, 0.0F, 0},
        {1.0F, 0.0F, 0.0F, 0.0F, 0},
        {0.0F, 0.5F, 0.0F, 0.0F, 0},
        {0.0F, 1.0F, 0.0F, 0.0F, 0},
        {0.0F, 0.0F, 0.5F, 0.0F, 0},
        {0.0F, 0.0F, 1.0F, 0.0F, 0},
        {1.0F, 1.0F, 0.0F, 0.0F, 0},
        {1.0F, 0.0F, 1.0F, 0.0F, 0},
        {0.0F, 1.0F, 1.0F, 0.0F, 0},
        {1.0F, 1.0F, 1.0F, 0.0F, 0},
        {1.5F, 0.0F, 0.0F, 0.0F, 0},
        {0.0F, 1.5F, 0.0F, 0.0F, 0},
    };

    // The vendor needs a still IMU window for gravity initialization and ten
    // map points.  Subsequent scans are deliberately sparse; this test is a
    // wiring/lifecycle proof, not a trajectory-accuracy benchmark.
    for (int tick = 0; tick <= 340; ++tick) {
      const double time = static_cast<double>(tick) / 100.0;
      output << "imu " << time << " 0 0 -9.81 0 0 0\n";
      if (tick < 210 || tick % 10 != 0) {
        continue;
      }
      output << "cloud " << time << " " << time << " " << points.size();
      for (const auto& point : points) {
        output << " " << point.x << " " << point.y << " " << point.z
               << " " << point.intensity << " 0.0";
      }
      output << "\n";
    }
  }

  LI2Sup::PointLIOFrontend::Config frontend_config;
  frontend_config.point_lio.initial_map_size = 10;
  frontend_config.point_lio.fix_gravity_direction = true;
  frontend_config.point_lio.gravity_mps2 = Eigen::Vector3d(0.0, 0.0, -9.81);
  frontend_config.point_lio.min_range_m = 0.1;
  frontend_config.point_lio.max_range_m = 10.0;
  frontend_config.point_lio.space_downsample = false;

  LI2Sup::PointLIOFrontend frontend(std::move(frontend_config));
  std::size_t frontend_results = 0;
  std::size_t downstream_frames = 0;
  frontend.setFrontendResultCallback(
      [&frontend_results](const adagraph::frontend::FrontendResult& result) {
        ++frontend_results;
        EXPECT_EQ(result.implementation,
                  adagraph::frontend::Implementation::kPointLio);
        EXPECT_TRUE(result.local_pose.valid());
      });
  frontend.setOdometryCallback(
      [](const LI2Sup::NavState&, const BASIC::V3&) {});
  frontend.setDownstreamFrameCallback(
      [&downstream_frames](LI2Sup::LegacyPointFrame&& frame,
                           const LI2Sup::IMUData&) {
        EXPECT_FALSE(frame.dense_points_odom.empty());
        ++downstream_frames;
      });
  ASSERT_NO_THROW(frontend.init());

  SlamSystem system;
  system.setImuSink([&frontend](ImuData imu) {
    return frontend.enqueueImu(std::move(imu))
               ? SystemStatus::Ok()
               : SystemStatus::Error(SystemStatusCode::kRejected,
                                     "Point-LIO IMU queue rejected input");
  });
  system.setPointCloudSink([&frontend](PointCloudFrame&& cloud) {
    return frontend.enqueuePointCloud(std::move(cloud))
               ? SystemStatus::Ok()
               : SystemStatus::Error(SystemStatusCode::kRejected,
                                     "Point-LIO lidar queue rejected input");
  });
  system.setProcessHandler([&frontend] {
    frontend.processInput();
    return SystemStatus::Ok();
  });
  system.setMappingStartHandler(
      [&frontend](const adagraph::system::MappingOptions&) {
        frontend.setInputEnabled(true);
        return SystemStatus::Ok();
      });
  system.setFinishTrajectoryHandler([&frontend] {
    frontend.finish();
    return SystemStatus::Ok();
  });

  TextDatasetInputSource source(path);
  DatasetRunner runner(system);
  const auto result = runner.Run(source);

  EXPECT_TRUE(result.status.ok()) << result.status.message;
  EXPECT_TRUE(result.reached_end_of_input);
  EXPECT_TRUE(result.finished_trajectory);
  EXPECT_EQ(result.point_cloud_packets, 14U);
  EXPECT_GT(frontend_results, 0U);
  const auto downstream = frontend.downstreamStats();
  // The Point-LIO adapter deliberately keeps a bounded newest-only queue so
  // frontend estimation never waits for downstream mapping.  Finish drains
  // every retained frame; a full queue may legitimately replace old frames.
  EXPECT_EQ(frontend_results, downstream.enqueued);
  EXPECT_EQ(downstream_frames, downstream.processed);
  EXPECT_EQ(downstream.enqueued,
            downstream.processed + downstream.dropped);
  EXPECT_LE(downstream.max_queue_depth,
            frontend_config.downstream_queue_capacity);

  std::error_code error;
  std::filesystem::remove(path, error);
  EXPECT_FALSE(error);
}

}  // namespace
