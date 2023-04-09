 /*
  * Copyright 2012 Open Source Robotics Foundation
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
 */


#ifndef GAZEBO_ROS_LASER_HH
#define GAZEBO_ROS_LASER_HH

#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/Param.hh>
#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

// ROS Topic subscriber
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Range.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ignition/math/Rand.hh>

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameName = "/world";
static const std::string kDefaultCommandPubTopic = "/lidar";

class GazeboRosLaser : public RayPlugin
  {
    public: GazeboRosLaser();
 
    public: ~GazeboRosLaser();
 
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
 
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();
 
    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    private: sensors::RaySensorPtr parent_ray_sensor_;
 
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<sensor_msgs::Range>::Ptr pub_queue_;
 
    private: std::string topic_name_;
 
    private: std::string frame_name_;
 
    private: std::string tf_prefix_;
 
    private: std::string robot_namespace_;
 
    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
 
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    private: PubMultiQueue pmq;
  };
}
#endif // GAZEBO_ROS_LASER_HH
