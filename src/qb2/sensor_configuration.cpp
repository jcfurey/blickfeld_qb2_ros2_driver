#include "qb2/sensor_configuration.h"
#include "utility/qb2_connection_utils.h"
#include "utility/scan_pattern_utils.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace blickfeld::ros_interop::qb2 {
namespace {
using Scan = system::config::ScanPattern;
using Filter = core_processing::config::PointCloud::Filter;

void range(double value, double minimum, double maximum, const std::string& field) {
  if (!std::isfinite(value) || value < minimum || value > maximum)
    throw std::invalid_argument(field + " must be in [" + std::to_string(minimum) + ", " + std::to_string(maximum) +
                                "]");
}

void fieldsValid(const std::vector<std::string>& fields, const std::set<std::string>& allowed) {
  if (fields.empty()) throw std::invalid_argument("Specify at least one field to update");
  std::set<std::string> seen;
  for (const auto& field : fields) {
    if (!allowed.count(field)) throw std::invalid_argument("Unknown or read-only field: " + field);
    if (!seen.insert(field).second) throw std::invalid_argument("Duplicate field: " + field);
  }
}

interfaces::msg::ScanPattern toRos(const Scan& scan) {
  interfaces::msg::ScanPattern out;
  out.horizontal_field_of_view = scan.horizontal().field_of_view();
  out.vertical_field_of_view = scan.vertical().field_of_view();
  out.scanlines_up = scan.vertical().scanlines_up();
  out.scanlines_down = scan.vertical().scanlines_down();
  out.angle_spacing = scan.pulse().angle_spacing();
  out.frame_mode = scan.frame_mode();
  out.target_frame_rate_enabled = scan.frame_rate().has_target();
  out.target_frame_rate = scan.frame_rate().target();
  out.actual_frame_rate = actualFrameRate(scan);
  out.foveation_enabled = scan.vertical().has_foveation();
  out.foveation_fraction = scan.vertical().foveation().fraction();
  out.foveation_density_factor = scan.vertical().foveation().density_factor();
  out.uniform = scan.pulse().has_uniform();
  out.pulse_mode = scan.pulse().uniform().pulse_mode();
  return out;
}

interfaces::msg::PointCloudFilter toRos(const Filter& filter) {
  interfaces::msg::PointCloudFilter out;
  out.maximum_returns_per_point = filter.maximum_returns_per_point();
  out.sorting = filter.sorting();
  out.minimum_reflectivity = filter.minimum_reflectivity();
  out.minimum_range = filter.minimum_range();
  out.maximum_range = filter.maximum_range();
  out.reflectivity_range_enabled = filter.has_minimum_reflectivity_range_limit();
  out.reflectivity_range_minimum = filter.minimum_reflectivity_range_limit().minimum();
  out.reflectivity_range_maximum = filter.minimum_reflectivity_range_limit().maximum();
  return out;
}

void patch(Scan& scan, const interfaces::srv::SetScanPattern::Request& request) {
  fieldsValid(request.fields, {"horizontal_field_of_view", "vertical_field_of_view", "scanlines_up", "scanlines_down",
                               "angle_spacing", "frame_mode", "target_frame_rate", "foveation"});
  const auto& config = request.configuration;
  for (const auto& field : request.fields) {
    if (field == "horizontal_field_of_view") {
      range(config.horizontal_field_of_view, 0, 3.141593, field);
      scan.mutable_horizontal()->set_field_of_view(config.horizontal_field_of_view);
    } else if (field == "vertical_field_of_view") {
      range(config.vertical_field_of_view, 0, 3.141593, field);
      scan.mutable_vertical()->set_field_of_view(config.vertical_field_of_view);
    } else if (field == "scanlines_up") {
      if (!config.scanlines_up) throw std::invalid_argument("scanlines_up must be positive");
      scan.mutable_vertical()->set_scanlines_up(config.scanlines_up);
    } else if (field == "scanlines_down") {
      if (!config.scanlines_down) throw std::invalid_argument("scanlines_down must be positive");
      scan.mutable_vertical()->set_scanlines_down(config.scanlines_down);
    } else if (field == "angle_spacing") {
      if (scan.pulse().has_uniform())
        throw std::invalid_argument("angle_spacing is controlled by the uniform scan pattern");
      range(config.angle_spacing, 0.004363323129985824, 0.017453292519943295, field);
      scan.mutable_pulse()->set_angle_spacing(config.angle_spacing);
    } else if (field == "frame_mode") {
      range(config.frame_mode, 1, 3, field);
      scan.set_frame_mode(static_cast<Scan::FrameMode>(config.frame_mode));
    } else if (field == "target_frame_rate") {
      if (config.target_frame_rate_enabled) {
        if (!std::isfinite(config.target_frame_rate) || config.target_frame_rate < 0)
          throw std::invalid_argument("target_frame_rate must be finite and nonnegative");
        scan.mutable_frame_rate()->set_target(config.target_frame_rate);
      } else {
        scan.mutable_frame_rate()->clear_target();
      }
    } else if (field == "foveation") {
      if (config.foveation_enabled) {
        range(config.foveation_fraction, 0.15, 0.85, "foveation_fraction");
        if (!std::isfinite(config.foveation_density_factor) || config.foveation_density_factor <= 0)
          throw std::invalid_argument("foveation_density_factor must be finite and positive");
        auto* foveation = scan.mutable_vertical()->mutable_foveation();
        foveation->set_fraction(config.foveation_fraction);
        foveation->set_density_factor(config.foveation_density_factor);
      } else {
        scan.mutable_vertical()->clear_foveation();
      }
    }
  }
  // These observations must not be sent back as configuration.
  clearReadOnlyScanFields(scan);
}

void validateLimits(const Scan& scan, const system::data::ScanPatternLimits& limits) {
  const auto& h = limits.horizontal();
  const auto& v = limits.vertical();
  if (h.has_field_of_view())
    range(scan.horizontal().field_of_view(), h.field_of_view().minimum(), h.field_of_view().maximum(),
          "horizontal_field_of_view");
  if (v.has_field_of_view())
    range(scan.vertical().field_of_view(), v.field_of_view().minimum(), v.field_of_view().maximum(),
          "vertical_field_of_view");
  if (v.has_scanlines_up())
    range(scan.vertical().scanlines_up(), v.scanlines_up().minimum(), v.scanlines_up().maximum(), "scanlines_up");
  if (v.has_scanlines_down())
    range(scan.vertical().scanlines_down(), v.scanlines_down().minimum(), v.scanlines_down().maximum(),
          "scanlines_down");
  if (scan.frame_rate().has_target() && limits.frame_rate().has_target())
    range(scan.frame_rate().target(), limits.frame_rate().target().minimum(), limits.frame_rate().target().maximum(),
          "target_frame_rate");
  if (scan.vertical().has_foveation() && v.has_foveation()) {
    if (v.foveation().has_fraction())
      range(scan.vertical().foveation().fraction(), v.foveation().fraction().minimum(),
            v.foveation().fraction().maximum(), "foveation_fraction");
    if (v.foveation().has_density_factor())
      range(scan.vertical().foveation().density_factor(), v.foveation().density_factor().minimum(),
            v.foveation().density_factor().maximum(), "foveation_density_factor");
  }
}

void patch(Filter& filter, const interfaces::srv::SetPointCloudFilter::Request& request) {
  fieldsValid(request.fields, {"maximum_returns_per_point", "sorting", "minimum_reflectivity", "minimum_range",
                               "maximum_range", "reflectivity_range"});
  const auto& config = request.configuration;
  for (const auto& field : request.fields) {
    if (field == "maximum_returns_per_point")
      filter.set_maximum_returns_per_point(config.maximum_returns_per_point);
    else if (field == "sorting") {
      range(config.sorting, 0, 3, field);
      filter.set_sorting(static_cast<Filter::Sorting>(config.sorting));
    } else if (field == "minimum_reflectivity") {
      range(config.minimum_reflectivity, 0, 10000, field);
      filter.set_minimum_reflectivity(config.minimum_reflectivity);
    } else if (field == "minimum_range") {
      range(config.minimum_range, 0, 100, field);
      filter.set_minimum_range(config.minimum_range);
    } else if (field == "maximum_range") {
      range(config.maximum_range, 0, 100, field);
      filter.set_maximum_range(config.maximum_range);
    } else if (field == "reflectivity_range") {
      if (config.reflectivity_range_enabled) {
        range(config.reflectivity_range_minimum, 0, 250, "reflectivity_range_minimum");
        range(config.reflectivity_range_maximum, config.reflectivity_range_minimum, 250, "reflectivity_range_maximum");
        filter.mutable_minimum_reflectivity_range_limit()->set_minimum(config.reflectivity_range_minimum);
        filter.mutable_minimum_reflectivity_range_limit()->set_maximum(config.reflectivity_range_maximum);
      } else {
        filter.clear_minimum_reflectivity_range_limit();
      }
    }
  }
  const double minimum = filter.minimum_range() == 0 ? 1 : filter.minimum_range();
  const double maximum = filter.maximum_range() == 0 ? 100 : filter.maximum_range();
  if (minimum > maximum) throw std::invalid_argument("minimum_range exceeds maximum_range");
}
}  // namespace

SensorConfiguration::SensorConfiguration(rclcpp::Node::SharedPtr node, const Qb2Info& info, const std::string& prefix)
    : node_(node), info_(info) {
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_scan_ = node_->create_service<interfaces::srv::GetScanPattern>(
      prefix + "/get_scan_pattern",
      [this](const std::shared_ptr<interfaces::srv::GetScanPattern::Request> request,
             std::shared_ptr<interfaces::srv::GetScanPattern::Response> response) {
        getScanPattern(*request, *response);
      },
      rclcpp::ServicesQoS(), group_);
  set_scan_ = node_->create_service<interfaces::srv::SetScanPattern>(
      prefix + "/set_scan_pattern",
      [this](const std::shared_ptr<interfaces::srv::SetScanPattern::Request> request,
             std::shared_ptr<interfaces::srv::SetScanPattern::Response> response) {
        setScanPattern(*request, *response);
      },
      rclcpp::ServicesQoS(), group_);
  get_filter_ = node_->create_service<interfaces::srv::GetPointCloudFilter>(
      prefix + "/get_point_cloud_filter",
      [this](const std::shared_ptr<interfaces::srv::GetPointCloudFilter::Request> request,
             std::shared_ptr<interfaces::srv::GetPointCloudFilter::Response> response) {
        getFilter(*request, *response);
      },
      rclcpp::ServicesQoS(), group_);
  set_filter_ = node_->create_service<interfaces::srv::SetPointCloudFilter>(
      prefix + "/set_point_cloud_filter",
      [this](const std::shared_ptr<interfaces::srv::SetPointCloudFilter::Request> request,
             std::shared_ptr<interfaces::srv::SetPointCloudFilter::Response> response) {
        setFilter(*request, *response);
      },
      rclcpp::ServicesQoS(), group_);
}

Qb2ChannelPtr SensorConfiguration::channel(ConnectionTarget target) {
  auto& result = target == ConnectionTarget::SYSTEM ? system_channel_ : core_channel_;
  if (!result) result = connect(info_, target, node_->get_logger());
  if (!result) throw std::runtime_error("Cannot connect to Qb2 configuration service");
  return result;
}

void SensorConfiguration::getScanPattern(const interfaces::srv::GetScanPattern::Request&,
                                         interfaces::srv::GetScanPattern::Response& response) {
  handle(response, [&] {
    auto stub = system::services::ScanPattern::NewStub(channel(ConnectionTarget::SYSTEM));
    system::services::ScanPatternGetResponse result;
    rpc([&](auto* ctx) { return stub->Get(ctx, google::protobuf::Empty(), &result); });
    response.name = result.name();
    response.configuration = toRos(result.scan_pattern());
    response.message = "Current sensor scan pattern";
  });
}

void SensorConfiguration::setScanPattern(const interfaces::srv::SetScanPattern::Request& request,
                                         interfaces::srv::SetScanPattern::Response& response) {
  handle(response, [&] {
    auto stub = system::services::ScanPattern::NewStub(channel(ConnectionTarget::SYSTEM));
    system::services::ScanPatternGetResponse current;
    rpc([&](auto* ctx) { return stub->Get(ctx, google::protobuf::Empty(), &current); });
    auto scan = current.scan_pattern();
    patch(scan, request);
    system::services::ScanPatternGetLimitsRequest query;
    *query.mutable_scan_pattern() = scan;
    system::services::ScanPatternGetLimitsResponse limits;
    rpc([&](auto* ctx) { return stub->GetLimits(ctx, query, &limits); });
    if (!limits.has_scan_pattern_limits()) throw std::runtime_error("Sensor returned no scan pattern limits");
    validateLimits(scan, limits.scan_pattern_limits());
    response.configuration = toRos(scan);
    if (request.dry_run) {
      response.message = "Validated against device limits; sensor unchanged";
      return;
    }
    system::services::ScanPatternSetRequest update;
    *update.mutable_scan_pattern() = scan;
    google::protobuf::Empty unused;
    rpc([&](auto* ctx) { return stub->Set(ctx, update, &unused); });
    response.applied = true;
    rpc([&](auto* ctx) { return stub->Get(ctx, google::protobuf::Empty(), &current); });
    response.configuration = toRos(current.scan_pattern());
    response.message = "Applied and read back from sensor";
  });
}

void SensorConfiguration::getFilter(const interfaces::srv::GetPointCloudFilter::Request&,
                                    interfaces::srv::GetPointCloudFilter::Response& response) {
  handle(response, [&] {
    auto stub = core_processing::services::PointCloud::NewStub(channel(ConnectionTarget::CORE_PROCESSING));
    core_processing::services::PointCloudGetFilterResponse result;
    rpc([&](auto* ctx) { return stub->GetFilter(ctx, google::protobuf::Empty(), &result); });
    response.configuration = toRos(result.filter());
    response.message = "Current sensor point-cloud filter";
  });
}

void SensorConfiguration::setFilter(const interfaces::srv::SetPointCloudFilter::Request& request,
                                    interfaces::srv::SetPointCloudFilter::Response& response) {
  handle(response, [&] {
    auto stub = core_processing::services::PointCloud::NewStub(channel(ConnectionTarget::CORE_PROCESSING));
    core_processing::services::PointCloudGetFilterResponse current;
    rpc([&](auto* ctx) { return stub->GetFilter(ctx, google::protobuf::Empty(), &current); });
    auto filter = current.filter();
    patch(filter, request);
    response.configuration = toRos(filter);
    if (request.dry_run) {
      response.message = "Validated locally; final validation occurs when applied by sensor";
      return;
    }
    core_processing::services::PointCloudSetFilterRequest update;
    *update.mutable_filter() = filter;
    google::protobuf::Empty unused;
    rpc([&](auto* ctx) { return stub->SetFilter(ctx, update, &unused); });
    response.applied = true;
    rpc([&](auto* ctx) { return stub->GetFilter(ctx, google::protobuf::Empty(), &current); });
    response.configuration = toRos(current.filter());
    response.message = "Applied and read back from sensor";
  });
}
}  // namespace blickfeld::ros_interop::qb2
