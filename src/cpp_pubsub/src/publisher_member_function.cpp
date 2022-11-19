// Copyright 2016 Open Source Robotics Foundation, Inc.
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

/**
 * @file publisher_member_function.cpp
 * @author Qamar Syed (qsyed@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/string_change.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


/**
 * @brief service that takes in response and outputs default response
 * 
 * @param request takes in from client
 * @param response output to client
 */
void custom_function(const std::shared_ptr<beginner_tutorials::srv::CustomService::Request> request,    
          std::shared_ptr<beginner_tutorials::srv::CustomService::Response>       response) {
  response = "default response";                                   
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), request);                                       
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), response);
}

class MinimalPublisher : public rclcpp::Node {
 public:
    /**
      * @brief Construct a new Minimal Publisher object
      * 
      */
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    
  
    custom_serv_ = this->create_service<beginner_tutorials::srv::CustomService>(
      "custom_service", &custom_function);
    }

 private:
    /**
     * @brief publishes out the message and a bunch of logging messages from each stream type
     * 
     */
    void timer_callback() {
      auto message = std_msgs::msg::String();
      message.data = "Custom message number "
        + std::to_string(count_++) +" by Qamar Syed";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      RCLCPP_ERROR_STREAM(this->get_logger(),"Error: Sample logging" << std::endl);
      RCLCPP_INFO_STREAM(this->get_logger(),"Info" << std::endl);
      RCLCPP_WARN_STREAM(this->get_logger(),"Warning" << std::endl);
      RCLCPP_FATAL_STREAM(this->get_logger(),"Fatal" << std::endl);
      RCLCPP_DEBUG_STREAM(this->get_logger(),"Debug" << std::endl);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
