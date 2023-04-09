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


#include "rosgazebo_lidar_plugin.h"
#include "common.h"

namespace gazebo {

// Register this plugin with the simulator
 GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)

 // Constructor
 GazeboRosLaser::GazeboRosLaser()
 {
 }

 // Destructor
 GazeboRosLaser::~GazeboRosLaser()
 {
   this->rosnode_->shutdown();
   delete this->rosnode_;
 }

 // Load the controller
 void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
 {
   // load plugin
   RayPlugin::Load(_parent, this->sdf);
   // Get the world name.
   std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
   // save pointers
   this->sdf = _sdf;

   GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
   this->parent_ray_sensor_ =
     dynamic_pointer_cast<sensors::RaySensor>(_parent);

   if (!this->parent_ray_sensor_)
     gzthrow("GazeboRosLaser controller requires a Ray Sensor as its parent");

   if (!this->sdf->HasElement("robotNamespace"))
   {
     ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(), "%s plugin missing <robotNamespace>, defaults to %s",
                    this->parent_ray_sensor_->Name().c_str(), kDefaultNamespace.c_str());
     this->robot_namespace_ = kDefaultNamespace;
   }
   else
     this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace");

   if (!this->sdf->HasElement("frameName"))
   {
     ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(), "%s plugin missing <frameName>, defaults to %s",
                    this->parent_ray_sensor_->Name().c_str(), kDefaultFrameName.c_str());
     this->frame_name_ = kDefaultFrameName;
   }
   else
     this->frame_name_ = this->sdf->Get<std::string>("frameName");

   if (!this->sdf->HasElement("topicName"))
   {
     ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(), "%s plugin missing <topicName>, defaults to %s",
                    this->parent_ray_sensor_->Name().c_str(), kDefaultCommandPubTopic.c_str());
     this->topic_name_ = kDefaultCommandPubTopic;
   }
   else
     this->topic_name_ = this->sdf->Get<std::string>("topicName");

   this->laser_connect_count_ = 0;

   // ROS Topic subscriber
   // Initialize ROS, if it has not already been initialized.
   if (!ros::isInitialized())   {
     int argc = 0;
     char **argv = NULL;
     ros::init(argc, argv, "rosgazebo_lidar_plugin", ros::init_options::NoSigintHandler);
   }

   ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(), "Starting %s Plugin (ns = %s)", this->parent_ray_sensor_->Name().c_str(),
                  this->robot_namespace_.c_str() );
   // ros callback queue for processing subscription
   this->deferred_load_thread_ = boost::thread(
     boost::bind(&GazeboRosLaser::LoadThread, this));
   ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(),"Subscribe to Gazebo topic: %s", this->parent_ray_sensor_->Topic().c_str() );
   ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(),"Publish on Ros topic: %s", this->topic_name_.c_str() );
  }

  // Load the controller
  void GazeboRosLaser::LoadThread()
  {
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if(this->tf_prefix_.empty()) {
        this->tf_prefix_ = this->robot_namespace_;
        boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
    }
    ROS_INFO_NAMED(this->parent_ray_sensor_->Name().c_str(), "%s Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
                   this->parent_ray_sensor_->Name().c_str(), this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

    // resolve tf prefix
//    this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

    if (this->topic_name_ != "")
    {
      ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<sensor_msgs::Range>(
        this->topic_name_, 1,
        boost::bind(&GazeboRosLaser::LaserConnect, this),
        boost::bind(&GazeboRosLaser::LaserDisconnect, this),
        ros::VoidPtr(), NULL);
      this->pub_ = this->rosnode_->advertise(ao);
      this->pub_queue_ = this->pmq.addPub<sensor_msgs::Range>();
    }

    // Initialize the controller

    // sensor generation off by default
    this->parent_ray_sensor_->SetActive(false);
  }

  // Increment count
  void GazeboRosLaser::LaserConnect()
  {
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                      &GazeboRosLaser::OnScan, this);
  }

  // Decrement count
  void GazeboRosLaser::LaserDisconnect()
  {
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0)
      this->laser_scan_sub_.reset();
  }

  // Convert new Gazebo message to ROS message and publish it
  void GazeboRosLaser::OnScan(ConstLaserScanStampedPtr &_msg)
  {
    // We got a new message from the Gazebo sensor.  Stuff a
    // corresponding ROS message and publish it.
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
    range_msg.header.frame_id = this->frame_name_;
    range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg.field_of_view = 0.0349066f;
    range_msg.min_range = _msg->scan().range_min();
    range_msg.max_range = _msg->scan().range_max();
    range_msg.range = _msg->scan().ranges(0);
    this->pub_queue_->push(range_msg, this->pub_);
  }
}
