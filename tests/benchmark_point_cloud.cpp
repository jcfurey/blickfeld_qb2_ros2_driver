#include "utility/qb2_ros2_utils.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

using namespace blickfeld::ros_interop;

int main() {
  for (const unsigned int count : {4000u, 100000u}) {
    Qb2Frame frame;
    auto* binary = frame.mutable_binary();
    binary->set_length(count);
    binary->set_cartesian(std::string(count * 12, '\x42'));
    binary->set_photon_count(std::string(count * 2, '\x17'));
    binary->set_direction_id(std::string(count * 4, '\x05'));
    for (bool fields : {false, true}) {
      Qb2Info info;
      info.point_cloud_info.intensity = fields;
      info.point_cloud_info.point_id = fields;
      std::vector<double> times;
      uint64_t checksum = 0;
      for (int trial = 0; trial < 7; ++trial) {
        const auto start = std::chrono::steady_clock::now();
        for (int iteration = 0; iteration < 200; ++iteration) {
          const auto cloud = convertToPointCloudMsg(frame, info);
          checksum += cloud->data.front() + cloud->data.back() + cloud->data.size();
        }
        const auto end = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::micro>(end - start).count() / 200);
      }
      std::sort(times.begin(), times.end());
      std::cout << count << ',' << (fields ? "xyz_intensity_id" : "xyz") << ',' << times[times.size() / 2] << ','
                << checksum << '\n';
    }
  }
}
