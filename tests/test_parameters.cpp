#include "qb2_ros2_driver.h"
#include "qb2_ros2_snapshot_driver.h"

#include <gtest/gtest.h>
#include <limits>

using namespace blickfeld::ros_interop;

class Parameters : public testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions options(std::vector<rclcpp::Parameter> parameters = {}) {
    rclcpp::NodeOptions result;
    result.parameter_overrides(parameters);
    return result;
  }
};

TEST_F(Parameters, MissingHostDoesNotShutDownContainer) {
  EXPECT_THROW(Qb2Driver driver(options()), std::invalid_argument);
  EXPECT_TRUE(rclcpp::ok());
  EXPECT_THROW(Qb2SnapshotDriver driver(options()), std::invalid_argument);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Parameters, RejectsPartialUnixSocketPair) {
  EXPECT_THROW(Qb2Driver driver(options({{"fqdn", "127.0.0.1"}, {"system_unix_socket", "/tmp/absent"}})),
               std::invalid_argument);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Parameters, ApplicationKeyNeedsSerialNumber) {
  EXPECT_THROW(Qb2Driver driver(options({{"fqdn", "127.0.0.1"}, {"application_key", "test-key"}})),
               std::invalid_argument);
}

TEST_F(Parameters, RejectsMismatchedSnapshotListsBeforeIndexing) {
  EXPECT_THROW(Qb2SnapshotDriver driver(options({{"fqdns", std::vector<std::string>{"127.0.0.1"}}})),
               std::invalid_argument);
  EXPECT_TRUE(rclcpp::ok());
  EXPECT_THROW(Qb2SnapshotDriver driver(options({
      {"system_unix_sockets", std::vector<std::string>{"/tmp/absent"}},
      {"core_processing_unix_sockets", std::vector<std::string>{"/tmp/absent"}}})), std::invalid_argument);
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Parameters, RejectsInvalidSnapshotRate) {
  for (const auto rate : {-0.1, 0.2, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(Qb2SnapshotDriver driver(options({{"snapshot_frame_rate", rate}})), std::invalid_argument);
    EXPECT_TRUE(rclcpp::ok());
  }
}

TEST_F(Parameters, RejectsInvalidRetryCount) {
  for (const auto retries : {0, -1, 256}) {
    EXPECT_THROW(Qb2SnapshotDriver driver(options({{"max_retries", retries}})), std::invalid_argument);
    EXPECT_TRUE(rclcpp::ok());
  }
}

TEST_F(Parameters, LiveComponentCanUnloadWithoutShuttingDownContainer) {
  {
    Qb2Driver driver(options({{"fqdn", "127.0.0.1"}}));
  }
  EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Parameters, ManualSnapshotCanLoadAndUnloadWithoutDevice) {
  {
    Qb2SnapshotDriver driver(options({
        {"fqdns", std::vector<std::string>{"127.0.0.1"}},
        {"fqdn_serial_numbers", std::vector<std::string>{""}},
        {"fqdn_application_keys", std::vector<std::string>{""}},
        {"fqdn_frame_ids", std::vector<std::string>{"lidar"}},
        {"fqdn_point_cloud_topics", std::vector<std::string>{"/bf/test_points"}},
        {"snapshot_frame_rate", 0.0}}));
  }
  EXPECT_TRUE(rclcpp::ok());
}
