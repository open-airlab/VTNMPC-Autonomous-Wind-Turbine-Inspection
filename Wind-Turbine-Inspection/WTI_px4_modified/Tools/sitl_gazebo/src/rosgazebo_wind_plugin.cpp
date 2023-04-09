/*
 * Copyright 2018 Mohit Mehndiratta, NTU, Singapore
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

#include "rosgazebo_wind_plugin.h"
#include "common.h"

namespace gazebo
{
RosGazeboWindPlugin::~RosGazeboWindPlugin()
{
    this->update_connection_.reset();

    this->ros_queue_.clear();
    this->ros_queue_.disable();
    this->ros_node_->shutdown();
    this->ros_queue_thread_.join();

    delete this->ros_node_;
}

void RosGazeboWindPlugin::DropCommandCallback(const std_msgs::Float64ConstPtr& _msg)
{
    drop_command_ = _msg->data;
}

void RosGazeboWindPlugin::WindCommandCallback(const geometry_msgs::Vector3ConstPtr& _msg)
{
    wind_command_ = {_msg->x, _msg->y, _msg->z};
}

void RosGazeboWindPlugin::QueueThread()
{
    static const double timeout = 0.01;

    while (this->ros_node_->ok())
    {
        this->ros_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void RosGazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model.
    model_ = _model;
    world_ = model_->GetWorld();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[rosgazebo_wind_plugin] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("xyzOffset"))
        xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
    else
        gzerr << "[rosgazebo_wind_plugin] Please specify a xyzOffset.\n";

    if (_sdf->HasElement("operationMode"))
    {
        std::string operation_mode;
        getSdfParam<std::string>(_sdf, "operationMode", operation_mode, "");
        if (operation_mode == "drop")
        {
            operation_mode_ = OperationMode::DROP;
        }
        else if (operation_mode == "wind")
        {
            operation_mode_ = OperationMode::WIND;
        }
        else
        {
            gzerr << "[rosgazebo_wind_plugin] Invalid operation_mode. Please specify either `drop` or `wind`.\n";
        }
    }
    if (_sdf->HasElement("windMode"))
    {
        std::string wind_mode;
        getSdfParam<std::string>(_sdf, "windMode", wind_mode, "");
        if (wind_mode == "velocity")
        {
            wind_mode_ = WindMode::VELOCITY;
        }
        else if (wind_mode == "force")
        {
            wind_mode_ = WindMode::FORCE;
        }
        else
        {
            gzerr << "[rosgazebo_wind_plugin] Invalid wind_mode. Please specify either `velocity` or `force`.\n";
        }
    }
    getSdfParam<double>(_sdf, "maxDropWeightKg", max_drop_weight_, max_drop_weight_);
    getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);

    double pub_rate = 2.0;
    getSdfParam<double>(_sdf, "publishRate", pub_rate, pub_rate);  //Wind topic publishing rates
    pub_interval_ = (pub_rate > 0.0) ? 1 / pub_rate : 0.0;

    getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
    getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
    // Get the wind params from SDF.
    if (_sdf->HasElement("commandSubTopic"))
        getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
    else
        gzerr << "[rosgazebo_wind_plugin] Please specify a commandSubTopic.\n";

    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[rosgazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&RosGazeboWindPlugin::OnUpdate, this, _1));

    wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

    // ROS Topic subscriber
    // Initialize ROS, if it has not already been initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_ros_sub", ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->ros_node_ = new ros::NodeHandle(this->namespace_);

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so;
    switch (operation_mode_)
    {
        case OperationMode::DROP:
            so = ros::SubscribeOptions::create<std_msgs::Float64>(
                command_sub_topic_,
                1,
                boost::bind(&RosGazeboWindPlugin::DropCommandCallback, this, _1),
                ros::VoidPtr(),
                &this->ros_queue_);
            break;
        case OperationMode::WIND:
            so = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
                command_sub_topic_,
                1,
                boost::bind(&RosGazeboWindPlugin::WindCommandCallback, this, _1),
                ros::VoidPtr(),
                &this->ros_queue_);
            break;
    }
    this->ros_sub_ = this->ros_node_->subscribe(so);
    this->ros_queue_thread_ = std::thread(std::bind(&RosGazeboWindPlugin::QueueThread, this));
    std::string wind_mode = (wind_mode_ == WindMode::VELOCITY) ? "velocity" : "force";
    std::cout << "[rosgazebo_wind_plugin]: Subscribe to ROS topic: " << command_sub_topic_ << " in "
              << wind_mode.c_str() << " mode." << std::endl;

#if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
#else
    last_time_ = world_->GetSimTime();
#endif
}

// This gets called by the world update start event.
void RosGazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world_->SimTime();
#else
    common::Time now = world_->GetSimTime();
#endif
    if (wind_mode_ == WindMode::VELOCITY && ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0))
    {
        return;
    }
    last_time_ = now;

    if (operation_mode_ == OperationMode::DROP)
    {
        ignition::math::Vector3d drop;
        drop = ignition::math::Vector3d(0, 0, 1) * drop_command_ * max_drop_weight_ * M_GRAVITATIONAL_CONST;

        if (drop_command_ != drop_command_last_)
        {
            std::cout << "applied force = [" << drop[0] << ", " << drop[1] << ", " << drop[2] << "] \n";
            drop_command_last_ = drop_command_;
        }

        // Apply a force from the constant wind to the link.
        link_->AddForceAtRelativePosition(drop, xyz_offset_);
    }
    else if (operation_mode_ == OperationMode::WIND)
    {
        switch (wind_mode_)
        {
            case WindMode::VELOCITY:
            {
                gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
                wind_v->set_x(wind_command_.X());
                wind_v->set_y(wind_command_.Y());
                wind_v->set_z(wind_command_.Z());

                wind_msg_.set_frame_id(frame_id_);
                wind_msg_.set_time_usec(now.Double() * 1e6);
                wind_msg_.set_allocated_velocity(wind_v);
                wind_pub_->Publish(wind_msg_);

                break;
            }
            case WindMode::FORCE:
            {
                // Apply a force from the constant wind to the link.
                link_->AddForceAtRelativePosition(wind_command_, xyz_offset_);
                break;
            }
        }
    }
    else
    {
        gzthrow("[rosgazebo_wind_plugin] Invalid specified operation_mode = " << operation_mode_ << ".\n");
    }
}

GZ_REGISTER_MODEL_PLUGIN(RosGazeboWindPlugin);
}  // namespace gazebo
