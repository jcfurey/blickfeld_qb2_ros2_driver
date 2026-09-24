#include "qb2_driver_status.h"
#include <gtest/gtest.h>
#include <thread>

using namespace blickfeld::ros_interop;
using Clock = std::chrono::steady_clock;

std::string value(DriverStatus& driver, const std::string& key) {
  diagnostic_updater::DiagnosticStatusWrapper status;
  driver.updateDiagnostic(status);
  for (const auto& pair : status.values)
    if (pair.key == key) return pair.value;
  return {};
}

TEST(DriverStatus, CountsFrameZeroGapsAndReboots) {
  DriverStatus status(Qb2Info{});
  auto now = Clock::now();
  status.updateRuntimeStatus(0, CommunicationState::SUCCESS_READ, now, now);
  status.updateRuntimeStatus(4, CommunicationState::SUCCESS_READ, now, now);
  EXPECT_EQ(value(status, "total_frames_dropped"), "3");
  status.updateRuntimeStatus(0, CommunicationState::FAIL_READ, now, now);
  EXPECT_EQ(value(status, "total_reboots"), "0");
  status.updateRuntimeStatus(0, CommunicationState::SUCCESS_READ, now, now);
  EXPECT_EQ(value(status, "total_reboots"), "1");
}

TEST(DriverStatus, RetainsAuthenticationFailureThroughDisconnect) {
  DriverStatus status(Qb2Info{});
  auto now = Clock::now();
  status.updateRuntimeStatus(0, CommunicationState::FAIL_AUTHENTICATION, now, now);
  status.disconnect();
  diagnostic_updater::DiagnosticStatusWrapper result;
  status.updateDiagnostic(result);
  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(result.message, "Authentication failed");
  EXPECT_TRUE(didAuthenticationFail({grpc::StatusCode::PERMISSION_DENIED, "Access denied"}));
  EXPECT_TRUE(didAuthenticationFail({grpc::StatusCode::UNAUTHENTICATED, "Expired"}));
  EXPECT_FALSE(didAuthenticationFail({grpc::StatusCode::UNAVAILABLE, "Offline"}));
}

TEST(DriverStatus, UnknownRateAndManualModeDoNotTriggerWatchdog) {
  auto past = Clock::now() - std::chrono::hours(1);
  DriverStatus unknown(Qb2Info{});
  unknown.updateRuntimeStatus(1, CommunicationState::SUCCESS_READ, past, past);
  EXPECT_FALSE(unknown.isFrameTooOld());
  Qb2ScanPattern pattern;
  pattern.frame_rate = 4;
  unknown.updateScanPattern(pattern, CommunicationState::SUCCESS_READ);
  EXPECT_TRUE(unknown.isFrameTooOld());
  Qb2Info manual;
  manual.snapshot = {true, 0};
  DriverStatus status(manual);
  status.updateRuntimeStatus(1, CommunicationState::SUCCESS_READ, past, past);
  status.updateScanPattern(pattern, CommunicationState::SUCCESS_READ);
  EXPECT_FALSE(status.isFrameTooOld());
}

TEST(DriverStatus, ConcurrentUpdatesHaveConsistentCounts) {
  DriverStatus status(Qb2Info{});
  std::thread publisher([&] {
    for (int i = 0; i < 10000; ++i) status.onPublishingFrame();
  });
  std::thread scan([&] {
    for (int i = 0; i < 10000; ++i) status.updateScanPattern(Qb2ScanPattern{}, CommunicationState::SUCCESS_READ);
  });
  for (int i = 0; i < 1000; ++i) {
    diagnostic_updater::DiagnosticStatusWrapper result;
    status.updateDiagnostic(result);
    status.isFrameTooOld();
  }
  publisher.join();
  scan.join();
  EXPECT_EQ(value(status, "total_frames_published"), "10000");
}
