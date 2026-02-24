// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROUTE_HISTORY__NODE_LOGIC_HPP_
#define ROUTE_HISTORY__NODE_LOGIC_HPP_

#include "rclcpp/rclcpp.hpp"
#include "types.hpp"
#include "yaml_storage.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::route_history
{

class NodeLogic
{
public:
  explicit NodeLogic(const rclcpp::Node::SharedPtr &);
  ~NodeLogic();

  auto get_save_file_path_param() -> std::string;

  void set_save_file_path(const std::string &);

  auto get_routes(const std::vector<std::string> &) -> std::vector<UuidName>;

  void load_route(const std::string &);

  void delete_route(const std::string &);

  void set_name(const std::string &, const std::string &);

  void clear_routes();

  void read_routes();

  void append_route(const YAML::Node &);

  auto prepend_uuid_name(const std::string &) -> std::string;

  void route_set_callback(const adapi_route &);

  void save_route();

  void create_group(
    const std::string & group_name = "New group",
    const std::vector<std::string> & route_uuids = {});
  void add_to_group(const std::string & group_uuid, const std::vector<std::string> & route_uuids);
  void remove_from_group(
    const std::string & group_uuid, const std::vector<std::string> & route_uuids);
  void set_group_name(const std::string & group_uuid, const std::string & group_name);
  void set_group_file_path(const std::string & group_file_path);

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_notif_publisher_;

  uuid_route_map routes;
  adapi_route current_route;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<YamlStorage> yaml_storage_routes_ = nullptr;
  std::unique_ptr<YamlStorage> yaml_storage_groups_ = nullptr;
  std::mutex mtx_;
};

}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__NODE_LOGIC_HPP_
