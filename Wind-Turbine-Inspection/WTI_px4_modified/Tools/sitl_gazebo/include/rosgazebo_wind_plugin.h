/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_ROSGAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_ROSGAZEBO_WIND_PLUGIN_H

#include "Wind.pb.h"
#include "gazebo/transport/transport.hh"
#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <random>
#include <string>

// ROS Topic subscriber
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <thread>

#define M_GRAVITATIONAL_CONST 9.81

namespace gazebo
{
enum OperationMode
{
    DROP,
    WIND
};

enum WindMode
{
    VELOCITY,
    FORCE
};

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultCommandSubTopic = "/servo_pos";

static const ignition::math::Vector3d kDefaultWindDirection = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirection = ignition::math::Vector3d(0, 1, 0);

static const double KDefaultMaxDropWeight = 0.5;  // in Kgs
static constexpr OperationMode KDefaultOperationMode = OperationMode::WIND;
static constexpr WindMode KDefaultWindMode = WindMode::VELOCITY;

/// \brief This gazebo plugin simulates wind acting on a model.
class RosGazeboWindPlugin : public ModelPlugin
{
public:
    RosGazeboWindPlugin()
        : ModelPlugin()
        , namespace_(kDefaultNamespace)
        , command_sub_topic_(kDefaultCommandSubTopic)
        , wind_pub_topic_("/world_wind")
        , frame_id_(kDefaultFrameId)
        , link_name_(kDefaultLinkName)
        , operation_mode_(KDefaultOperationMode)
        , wind_mode_(KDefaultWindMode)
        , max_drop_weight_(KDefaultMaxDropWeight)
        , pub_interval_(0.1)
        , node_handle_(NULL)
    {
    }

    void DropCommandCallback(const std_msgs::Float64ConstPtr& _msg);
    void WindCommandCallback(const geometry_msgs::Vector3ConstPtr& _msg);

    virtual ~RosGazeboWindPlugin();

protected:
    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Called when the world is updated.
    /// \param[in] _info Update timing information.
    void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
    void QueueThread();

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_;

    physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;

    std::string namespace_;
    std::string frame_id_;
    std::string link_name_;
    std::string command_sub_topic_;
    std::string wind_pub_topic_;

    ignition::math::Vector3d xyz_offset_;
    OperationMode operation_mode_;
    WindMode wind_mode_;

    double max_drop_weight_;
    double drop_command_;
    double drop_command_last_;
    ignition::math::Vector3d wind_command_;

    double pub_interval_;
    common::Time last_time_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr wind_pub_;

    boost::thread callback_queue_thread_;

    physics_msgs::msgs::Wind wind_msg_;

    // ROS communication
    ros::NodeHandle* ros_node_;
    ros::Subscriber ros_sub_;
    ros::CallbackQueue ros_queue_;
    std::thread ros_queue_thread_;
};
}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_ROSGAZEBO_WIND_PLUGIN_H
