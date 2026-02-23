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
#include "yaml_unmarshal.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "autoware_adapi_v1_msgs/msg/route.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <exception>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::route_history
{

class NodeLogic
{
public:
  // clang-format off
  explicit NodeLogic(const rclcpp::Node::SharedPtr & node)
  : node_(node)  // clang-format on
  {
    if (!node_->has_parameter("save_file_path")) {
      node_->declare_parameter("save_file_path", "~/.ros/route_history.yaml");
    }

    save_file_cb_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters)
        -> rcl_interfaces::msg::SetParametersResult {
        return save_file_path_callback(parameters);
      });

    // clang-format off
    route_set_subscription_ = node_->create_subscription<adapi_route>(
      "/api/routing/route", 10, [this](const adapi_route & msg) {route_set_callback(msg);});
    // clang-format on

    initial_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    goal_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 10);
    sync_notif_publisher_ = node_->create_publisher<std_msgs::msg::String>("update", 10);

    yaml_storage_ = std::make_unique<YamlStorage>();
    yaml_storage_->set_path(get_save_file_path_param());
    read_routes();
  }

  ~NodeLogic()
  {
    save_file_cb_.reset();
    route_set_subscription_.reset();
    initial_pose_publisher_.reset();
    goal_pose_publisher_.reset();
    sync_notif_publisher_.reset();

    RCLCPP_INFO(node_->get_logger(), "NodeLogic clean up successful.");
  }

  rcl_interfaces::msg::SetParametersResult save_file_path_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : parameters) {
      if (p.get_name() == "save_file_path") {
        std::string path = p.get_value<std::string>();

        if (path.empty()) {
          result.successful = false;
          result.reason = "Expanded path is empty";
          return result;
        }

        if (!yaml_storage_) {
          result.successful = false;
          result.reason = "YAML manager not initialized";
          return result;
        }

        yaml_storage_->set_path(path);
        this->read_routes();
      }
    }
    return result;
  }

  // Callback updates the yaml manager
  auto get_save_file_path_param() -> std::string
  {
    return node_->get_parameter("save_file_path").as_string();
  }

  void set_save_file_path_param(const std::string & new_path)
  {
    node_->set_parameters({rclcpp::Parameter("save_file_path", new_path)});
  }

  auto get_routes(const std::vector<std::string> & uuids) -> std::vector<UuidName>
  {
    std::vector<UuidName> fetched_routes;
    if (uuids.size() == 0) {
      for (auto & [key, value] : routes) {
        UuidName res;
        res.uuid = key;
        res.name = value.name;
        fetched_routes.push_back(res);
      }
    } else {
      for (auto & uuid : uuids) {
        if (routes.count(uuid)) {
          auto route = routes[uuid];
          UuidName res;
          res.uuid = uuid;
          res.name = route.name;
          fetched_routes.push_back(res);
        } else {
          RCLCPP_INFO(
            node_->get_logger(), "[get_routes] No route found in save file for uuid %s.",
            uuid.c_str());
        }
      }
    }
    return fetched_routes;
  }

  void load_route(const std::string & uuid)
  {
    if (uuid.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[set_route] No uuid given.");
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped initial_msg;

    initial_msg.header.frame_id = routes.at(uuid).route.header.frame_id;

    initial_msg.pose.pose.position.x = routes.at(uuid).route.data[0].start.position.x;
    initial_msg.pose.pose.position.y = routes.at(uuid).route.data[0].start.position.y;
    initial_msg.pose.pose.position.z = routes.at(uuid).route.data[0].start.position.z;

    initial_msg.pose.pose.orientation.x = routes.at(uuid).route.data[0].start.orientation.x;
    initial_msg.pose.pose.orientation.y = routes.at(uuid).route.data[0].start.orientation.y;
    initial_msg.pose.pose.orientation.z = routes.at(uuid).route.data[0].start.orientation.z;
    initial_msg.pose.pose.orientation.w = routes.at(uuid).route.data[0].start.orientation.w;

    geometry_msgs::msg::PoseStamped goal_msg;

    goal_msg.header.frame_id = routes.at(uuid).route.header.frame_id;

    goal_msg.pose.position.x = routes.at(uuid).route.data[0].goal.position.x;
    goal_msg.pose.position.y = routes.at(uuid).route.data[0].goal.position.y;
    goal_msg.pose.position.z = routes.at(uuid).route.data[0].goal.position.z;

    goal_msg.pose.orientation.x = routes.at(uuid).route.data[0].goal.orientation.x;
    goal_msg.pose.orientation.y = routes.at(uuid).route.data[0].goal.orientation.y;
    goal_msg.pose.orientation.z = routes.at(uuid).route.data[0].goal.orientation.z;
    goal_msg.pose.orientation.w = routes.at(uuid).route.data[0].goal.orientation.w;

    initial_pose_publisher_->publish(initial_msg);
    goal_pose_publisher_->publish(goal_msg);
  }

  void delete_route(const std::string & uuid)
  {
    if (uuid.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[delete_route] No uuid given.");
      return;
    }

    std::vector<YAML::Node> docs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      docs = yaml_storage_->read();
    }

    // Delete from function scope
    // clang-format off
    for (auto p = docs.begin(); p != docs.end(); ) {
      // clang-format on
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        p = docs.erase(p);
        RCLCPP_INFO(
          node_->get_logger(), "[delete_route] Deleted route with uuid: %s.", uuid.c_str());
      } else {
        ++p;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      yaml_storage_->clear();
      yaml_storage_->write(docs, true);
    }

    read_routes();
  }

  void set_name(const std::string & uuid, const std::string & new_name)
  {
    if (uuid.size() == 0 || new_name.size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "[set_name] Invalid empty uuid or name given.");
      return;
    }

    // Read and save to function scope
    std::vector<YAML::Node> docs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      docs = yaml_storage_->read();
    }

    // Delete from function scope
    // clang-format off
    for (auto p = docs.begin(); p != docs.end(); ) {
      // clang-format on
      if (!*p || !(*p)["uuid"]) {
        ++p;
        continue;
      }
      if ((*p)["uuid"].as<std::string>() == uuid) {
        (*p)["name"] = new_name;
        RCLCPP_INFO(
          node_->get_logger(), "[set_name] Name changed to \"%s\" for route with uuid: %s.",
          new_name.c_str(), uuid.c_str());
        p = docs.end();
      } else {
        ++p;
      }
    }

    // Wipe file and re-write to save
    {
      std::lock_guard<std::mutex> lock(mtx_);
      yaml_storage_->clear();
      yaml_storage_->write(docs, true);
    }

    // Update local node to sync
    read_routes();
  }

  // clang-format off
  void clear_routes() {routes = {};}
  // clang-format on

  void read_routes()
  {
    std::vector<YAML::Node> docs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      docs = yaml_storage_->read();
    }
    clear_routes();
    for (auto & doc : docs) {
      append_route(doc);
    }
  }

  void append_route(const YAML::Node & node)
  {
    uuid_route_map new_map = yaml_to_map(node);
    routes.insert(new_map.begin(), new_map.end());
  }

  auto prepend_uuid_name(const std::string & yaml_str) -> std::string
  {
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid_str = gen();

    std::ostringstream oss;
    oss << "name: \"New route"
        << "\"\n";
    oss << "uuid: \"" << uuid_str << "\"\n";
    oss << yaml_str;
    std::string new_str = oss.str();
    return new_str;
  }

  // clang-format off
  void route_set_callback(const adapi_route & msg) {current_route = msg;}
  // clang-format on

  void save_route()
  {
    if (current_route.data.empty()) {
      return;
    }

    std::string yaml_str = autoware_adapi_v1_msgs::msg::to_yaml(current_route);
    auto uuid_yaml_str = prepend_uuid_name(yaml_str);
    yaml_storage_->write(uuid_yaml_str, true);

    RCLCPP_INFO(
      node_->get_logger(), "[route_set_callback] Route written to %s.",
      get_save_file_path_param().c_str());
    read_routes();
  }

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_set_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_notif_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr save_file_cb_;

  uuid_route_map routes;
  adapi_route current_route;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<YamlStorage> yaml_storage_ = nullptr;
  std::mutex mtx_;
};

}  // namespace autoware::route_history

#endif  // ROUTE_HISTORY__NODE_LOGIC_HPP_
