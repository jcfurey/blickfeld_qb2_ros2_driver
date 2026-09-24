#include "qb2/sensor_configuration.h"
#include "qb2/point_cloud_streamer.h"
#include "qb2/point_cloud_getter.h"
#include "qb2/scan_pattern_watcher.h"
#include <grpc++/server_builder.h>
#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <thread>
#include <unistd.h>

using namespace blickfeld::ros_interop;
using namespace std::chrono_literals;
namespace api = blickfeld_qb2_ros2_driver;
namespace sys = blickfeld::system::services;
namespace core = blickfeld::core_processing::services;

class MockScan : public sys::ScanPattern::Service {
 public:
  MockScan() {
    scan.mutable_horizontal()->set_field_of_view(1.5);
    scan.mutable_vertical()->set_field_of_view(0.8);
    scan.mutable_vertical()->set_scanlines_up(40);
    scan.mutable_vertical()->set_scanlines_down(40);
    scan.mutable_frame_rate()->set_actual(4);
    scan.mutable_pulse()->set_disable_pre_pulse(true);  // Not exposed in ROS: must survive an update.
  }
  blickfeld::system::config::ScanPattern scan;
  int writes = 0;
  std::atomic<bool> deny{false};
  bool reject_write = false, fail_readback = false;
  grpc::Status Get(grpc::ServerContext*, const google::protobuf::Empty*,
                   sys::ScanPatternGetResponse* response) override {
    if (deny) return {grpc::StatusCode::PERMISSION_DENIED, "AUTHORIZED access required"};
    if (writes && fail_readback) return {grpc::StatusCode::UNAVAILABLE, "Readback failed"};
    *response->mutable_scan_pattern() = scan;
    response->set_name("current");
    return grpc::Status::OK;
  }
  grpc::Status Set(grpc::ServerContext*, const sys::ScanPatternSetRequest* request, google::protobuf::Empty*) override {
    if (reject_write) return {grpc::StatusCode::INVALID_ARGUMENT, "Unsupported combination"};
    scan = request->scan_pattern();
    scan.mutable_frame_rate()->set_actual(5);
    ++writes;
    return grpc::Status::OK;
  }
  grpc::Status GetLimits(grpc::ServerContext*, const sys::ScanPatternGetLimitsRequest*,
                         sys::ScanPatternGetLimitsResponse* response) override {
    auto* limits = response->mutable_scan_pattern_limits();
    limits->mutable_horizontal()->mutable_field_of_view()->set_minimum(1.0);
    limits->mutable_horizontal()->mutable_field_of_view()->set_maximum(1.6);
    limits->mutable_vertical()->mutable_scanlines_up()->set_minimum(10);
    limits->mutable_vertical()->mutable_scanlines_up()->set_maximum(100);
    return grpc::Status::OK;
  }
  grpc::Status Watch(grpc::ServerContext* context, const google::protobuf::Empty*,
                     grpc::ServerWriter<sys::ScanPatternWatchResponse>* writer) override {
    sys::ScanPatternWatchResponse response;
    *response.mutable_scan_pattern() = scan;
    writer->Write(response);
    while (!context->IsCancelled()) std::this_thread::sleep_for(1ms);
    return {grpc::StatusCode::CANCELLED, "Cancelled"};
  }
};

class MockCloud : public core::PointCloud::Service {
 public:
  MockCloud() {
    filter.set_minimum_range(1);
    filter.set_maximum_range(80);
    filter.set_minimum_reflectivity(17);
  }
  blickfeld::core_processing::config::PointCloud::Filter filter;
  int writes = 0;
  grpc::Status GetFilter(grpc::ServerContext*, const google::protobuf::Empty*,
                         core::PointCloudGetFilterResponse* response) override {
    *response->mutable_filter() = filter;
    return grpc::Status::OK;
  }
  grpc::Status SetFilter(grpc::ServerContext*, const core::PointCloudSetFilterRequest* request,
                         google::protobuf::Empty*) override {
    filter = request->filter();
    ++writes;
    return grpc::Status::OK;
  }
  grpc::Status Stream(grpc::ServerContext* context, const core::PointCloudStreamRequest*,
                      grpc::ServerWriter<core::PointCloudStreamResponse>* writer) override {
    core::PointCloudStreamResponse response;
    response.mutable_frame()->set_id(42);
    writer->Write(response);
    while (!context->IsCancelled()) std::this_thread::sleep_for(1ms);
    return {grpc::StatusCode::CANCELLED, "Cancelled"};
  }
  grpc::Status Get(grpc::ServerContext* context, const core::PointCloudGetRequest*,
                   core::PointCloudGetResponse*) override {
    while (!context->IsCancelled()) std::this_thread::sleep_for(1ms);
    return {grpc::StatusCode::CANCELLED, "Cancelled"};
  }
};

class Configuration : public testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("configuration_test");
    socket = "/tmp/qb2-mock-" + std::to_string(getpid());
    grpc::ServerBuilder builder;
    builder.AddListeningPort("unix:" + socket, grpc::InsecureServerCredentials());
    builder.RegisterService(&scan);
    builder.RegisterService(&cloud);
    server = builder.BuildAndStart();
    ASSERT_TRUE(server);
    info.host.system_socket = socket;
    info.host.core_processing_socket = socket;
    config = std::make_unique<qb2::SensorConfiguration>(node, info, "~");
  }
  void TearDown() override {
    config.reset();
    server->Shutdown();
    server->Wait();
    node.reset();
    rclcpp::shutdown();
    unlink(socket.c_str());
  }
  MockScan scan;
  MockCloud cloud;
  std::string socket;
  Qb2Info info;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<grpc::Server> server;
  std::unique_ptr<qb2::SensorConfiguration> config;
};

TEST_F(Configuration, ConstructionAndReadsDoNotWrite) {
  api::srv::GetScanPattern::Response response;
  config->getScanPattern(api::srv::GetScanPattern::Request(), response);
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.configuration.actual_frame_rate, 4);
  api::srv::GetPointCloudFilter::Response filter;
  config->getFilter(api::srv::GetPointCloudFilter::Request(), filter);
  EXPECT_TRUE(filter.success);
  EXPECT_EQ(filter.configuration.minimum_reflectivity, 17u);
  EXPECT_EQ(scan.writes, 0);
  EXPECT_EQ(cloud.writes, 0);
}

TEST_F(Configuration, DryRunAndDeviceLimits) {
  api::srv::SetScanPattern::Request request;
  request.fields = {"scanlines_up"};
  request.configuration.scanlines_up = 60;
  ASSERT_TRUE(request.dry_run);
  api::srv::SetScanPattern::Response response;
  config->setScanPattern(request, response);
  EXPECT_TRUE(response.success) << response.message;
  EXPECT_FALSE(response.applied);
  EXPECT_EQ(scan.writes, 0);
  request.configuration.scanlines_up = 500;
  config->setScanPattern(request, response);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(scan.writes, 0);
}

TEST_F(Configuration, PartialUpdatePreservesVendorFieldsAndReadsActualRate) {
  api::srv::SetScanPattern::Request request;
  request.fields = {"scanlines_up"};
  request.configuration.scanlines_up = 60;
  request.dry_run = false;
  api::srv::SetScanPattern::Response response;
  config->setScanPattern(request, response);
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_TRUE(response.applied);
  EXPECT_EQ(response.configuration.actual_frame_rate, 5);
  EXPECT_EQ(scan.scan.vertical().scanlines_up(), 60u);
  EXPECT_EQ(scan.scan.vertical().scanlines_down(), 40u);
  EXPECT_TRUE(scan.scan.pulse().disable_pre_pulse());
  EXPECT_EQ(scan.writes, 1);
}

TEST_F(Configuration, ReportsRejectedWritesAndFailedReadback) {
  api::srv::SetScanPattern::Request request;
  request.fields = {"scanlines_up"};
  request.configuration.scanlines_up = 50;
  request.dry_run = false;
  scan.reject_write = true;
  api::srv::SetScanPattern::Response response;
  config->setScanPattern(request, response);
  EXPECT_FALSE(response.success);
  EXPECT_FALSE(response.applied);
  scan.reject_write = false;
  scan.fail_readback = true;
  config->setScanPattern(request, response);
  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.applied);
}

TEST_F(Configuration, AuthenticationErrorIsReturnedWithoutWrites) {
  scan.deny = true;
  api::srv::GetScanPattern::Response response;
  config->getScanPattern(api::srv::GetScanPattern::Request(), response);
  EXPECT_FALSE(response.success);
  EXPECT_NE(response.message.find("RPC 7"), std::string::npos);
  EXPECT_EQ(scan.writes, 0);
}

TEST_F(Configuration, FilterUpdatesPreserveOtherSettingsAndRejectInvalidRanges) {
  api::srv::SetPointCloudFilter::Request request;
  request.fields = {"minimum_range"};
  request.configuration.minimum_range = 2;
  api::srv::SetPointCloudFilter::Response response;
  config->setFilter(request, response);
  EXPECT_TRUE(response.success);
  EXPECT_EQ(cloud.writes, 0);
  request.dry_run = false;
  config->setFilter(request, response);
  ASSERT_TRUE(response.success) << response.message;
  EXPECT_TRUE(response.applied);
  EXPECT_EQ(response.configuration.minimum_range, 2);
  EXPECT_EQ(response.configuration.maximum_range, 80);
  EXPECT_EQ(response.configuration.minimum_reflectivity, 17u);
  request.configuration.minimum_range = 90;
  response = api::srv::SetPointCloudFilter::Response();
  config->setFilter(request, response);
  EXPECT_FALSE(response.success);
  EXPECT_FALSE(response.applied);
  EXPECT_EQ(cloud.writes, 1);
}

TEST_F(Configuration, RejectsUnknownAndNonfiniteFields) {
  api::srv::SetScanPattern::Request request;
  request.fields = {"actual_frame_rate"};
  api::srv::SetScanPattern::Response response;
  config->setScanPattern(request, response);
  EXPECT_FALSE(response.success);
  request.fields = {"vertical_field_of_view"};
  request.configuration.vertical_field_of_view = std::numeric_limits<double>::quiet_NaN();
  config->setScanPattern(request, response);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(scan.writes, 0);
}

TEST_F(Configuration, StreamCanCancelBlockedReadAndStopBeforeRecreation) {
  qb2::PointCloudStreamer reader(node, info);
  ASSERT_EQ(reader.readFrame().second->id(), 42u);
  auto future = std::async(std::launch::async, [&] { return reader.readFrame(); });
  reader.cancel(true);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().second);
  // A later context must also be cancelled, even if stop preceded its creation.
  future = std::async(std::launch::async, [&] { return reader.readFrame(); });
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().second);
}

TEST_F(Configuration, SnapshotGetterHonorsStopBeforeContextCreation) {
  qb2::PointCloudGetter reader(node, info);
  reader.cancel(true);
  auto future = std::async(std::launch::async, [&] { return reader.readFrame(); });
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().second);
}

TEST_F(Configuration, WatcherReadsActualRateAndHonorsPermanentStop) {
  qb2::ScanPatternWatcher watcher(node, info);
  const auto result = watcher.watchScanPattern();
  ASSERT_TRUE(result.second);
  EXPECT_EQ(result.second->frame_rate, 4);
  watcher.cancel(true);
  auto future = std::async(std::launch::async, [&] { return watcher.watchScanPattern(); });
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().second);
  future = std::async(std::launch::async, [&] { return watcher.watchScanPattern(); });
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get().second);
}
