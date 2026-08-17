"""Shared ROS timestamp normalization for replayed LiDAR/IMU streams."""


class ReplayTimeNormalizer:
    """Detect bag-loop resets from LiDAR and apply one offset to both streams."""

    def __init__(self, enabled, reset_threshold_seconds=1.0):
        if reset_threshold_seconds <= 0.0:
            raise ValueError("reset_threshold_seconds must be positive")
        self.enabled = enabled
        self.reset_threshold_seconds = reset_threshold_seconds
        self.offset_seconds = 0.0
        self.last_lidar_source_seconds = None
        self.latest_output_seconds = None
        self.normalized_loops = 0

    def lidar_offset(self, source_seconds):
        if (
            self.enabled
            and self.last_lidar_source_seconds is not None
            and source_seconds
            < self.last_lidar_source_seconds - self.reset_threshold_seconds
        ):
            current = source_seconds + self.offset_seconds
            high_watermark = self.latest_output_seconds
            if high_watermark is not None and current <= high_watermark:
                self.offset_seconds += high_watermark - current + 1e-6
            self.normalized_loops += 1
        self.last_lidar_source_seconds = source_seconds
        return self.offset_seconds

    def imu_offset(self):
        return self.offset_seconds

    def observe_output(self, source_seconds, offset_seconds):
        output_seconds = source_seconds + offset_seconds
        if (
            self.latest_output_seconds is None
            or output_seconds > self.latest_output_seconds
        ):
            self.latest_output_seconds = output_seconds
