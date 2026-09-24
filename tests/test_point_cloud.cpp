#include "utility/qb2_ros2_utils.h"

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <cstring>

using namespace blickfeld::ros_interop;

namespace {
Qb2Frame makeFrame() {
  Qb2Frame frame;
  frame.set_timestamp(1234567890);
  auto* binary = frame.mutable_binary();
  binary->set_length(2);
  const std::array<float, 6> xyz{1.0f, -2.0f, 3.5f, 4.0f, 5.0f, -6.0f};
  const std::array<uint16_t, 2> intensity{123, 65535};
  const std::array<uint32_t, 2> ids{42, 100000};
  binary->set_cartesian(xyz.data(), sizeof(xyz));
  binary->set_photon_count(intensity.data(), sizeof(intensity));
  binary->set_direction_id(ids.data(), sizeof(ids));
  return frame;
}
}

class PointCloudFields : public testing::TestWithParam<std::tuple<bool, bool>> {};

TEST_P(PointCloudFields, PreservesCoordinatesAndEnabledFields) {
  Qb2Info info;
  info.point_cloud_info = {"qb2_lidar", "/bf/points_raw", std::get<0>(GetParam()), std::get<1>(GetParam())};
  auto frame = makeFrame();
  if (!info.point_cloud_info.intensity) frame.mutable_binary()->clear_photon_count();
  if (!info.point_cloud_info.point_id) frame.mutable_binary()->clear_direction_id();
  const auto cloud = convertToPointCloudMsg(frame, info);
  EXPECT_EQ(cloud->header.frame_id, "qb2_lidar");
  EXPECT_EQ(rclcpp::Time(cloud->header.stamp).nanoseconds(), 1234567890);
  EXPECT_EQ(cloud->height, 1u);
  EXPECT_EQ(cloud->width, 2u);
  EXPECT_EQ(cloud->fields.size(), 3u + info.point_cloud_info.intensity + info.point_cloud_info.point_id);
  EXPECT_EQ(cloud->point_step, 12u + 4u * (info.point_cloud_info.intensity + info.point_cloud_info.point_id));
  EXPECT_EQ(cloud->row_step, 2u * cloud->point_step);
  EXPECT_EQ(cloud->data.size(), cloud->row_step);
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
  EXPECT_FLOAT_EQ(*x, 1.0f);
  EXPECT_FLOAT_EQ(*y, -2.0f);
  EXPECT_FLOAT_EQ(*z, 3.5f);
  EXPECT_FLOAT_EQ(*(++x), 4.0f);
  EXPECT_FLOAT_EQ(*(++y), 5.0f);
  EXPECT_FLOAT_EQ(*(++z), -6.0f);
  if (info.point_cloud_info.intensity) {
    sensor_msgs::PointCloud2ConstIterator<uint32_t> intensity(*cloud, "intensity");
    EXPECT_EQ(*intensity, 123u);
    EXPECT_EQ(*(++intensity), 65535u);
  }
  if (info.point_cloud_info.point_id) {
    sensor_msgs::PointCloud2ConstIterator<uint32_t> id(*cloud, "point_id");
    EXPECT_EQ(*id, 42u);
    EXPECT_EQ(*(++id), 100000u);
  }
}

INSTANTIATE_TEST_SUITE_P(AllFieldCombinations, PointCloudFields,
                        testing::Combine(testing::Bool(), testing::Bool()));

TEST(PointCloud, TimestampOverride) {
  const auto cloud = convertToPointCloudMsg(makeFrame(), Qb2Info{}, rclcpp::Time(9876543210LL));
  EXPECT_EQ(rclcpp::Time(cloud->header.stamp).nanoseconds(), 9876543210LL);
}

TEST(PointCloud, EmptyFrame) {
  const auto cloud = convertToPointCloudMsg(Qb2Frame{}, Qb2Info{});
  EXPECT_EQ(cloud->width, 0u);
  EXPECT_TRUE(cloud->data.empty());
}

TEST(PointCloud, RejectsTruncatedBinaryFields) {
  auto frame = makeFrame();
  frame.mutable_binary()->set_cartesian("short");
  EXPECT_THROW(convertToPointCloudMsg(frame, Qb2Info{}), std::invalid_argument);
  frame = makeFrame();
  frame.mutable_binary()->clear_photon_count();
  EXPECT_THROW(convertToPointCloudMsg(frame, Qb2Info{}), std::invalid_argument);
  frame = makeFrame();
  frame.mutable_binary()->clear_direction_id();
  EXPECT_THROW(convertToPointCloudMsg(frame, Qb2Info{}), std::invalid_argument);
}
