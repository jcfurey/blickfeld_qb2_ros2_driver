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

#include "utility/parameters.h"
#include <fstream>
#include <unistd.h>

TEST_F(Parameters, StartupParametersRejectIneffectiveChanges) {
  auto node = std::make_shared<rclcpp::Node>("startup_parameters");
  EXPECT_EQ(startupParameter<std::string>(*node, "frame_id", "lidar"), "lidar");
  EXPECT_TRUE(node->describe_parameter("frame_id").read_only);
  const auto result = node->set_parameter(rclcpp::Parameter("frame_id", "different"));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(node->get_parameter("frame_id").as_string(), "lidar");
}

TEST_F(Parameters, PrivateKeyFilesValidateWithoutExposingValues) {
  const auto path = "/tmp/qb2-test-key-" + std::to_string(getpid());
  { std::ofstream file(path); file << "  test-key\n"; }
  EXPECT_EQ(applicationKey("", path), "test-key");
  EXPECT_THROW(applicationKey("conflicting-key", path), std::invalid_argument);
  { std::ofstream file(path); file << "\n"; }
  EXPECT_THROW(applicationKey("", path), std::invalid_argument);
  unlink(path.c_str());
  EXPECT_THROW(applicationKey("", path), std::invalid_argument);
}
