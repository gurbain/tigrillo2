/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 * https://www.humanbrainproject.eu
 *
 * The Human Brain Project is a European Commission funded project
 * in the frame of the Horizon2020 FET Flagship plan.
 * http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/

#ifndef _TIGRILLO_PLUGIN_HH_
#define _TIGRILLO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/console.h"
#include "ros/subscribe_options.h"

#include "std_msgs/String.h"
#include "tigrillo_2_plugin/Motors.h"
#include "tigrillo_2_plugin/Imu.h"
#include "tigrillo_2_plugin/Sensors.h"

namespace gazebo
{
  /// \brief A plugin to control a Tigrillo quadruped with ROS.
  class TigrilloPlugin : public ModelPlugin
  {
    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    
    /// \brief Name of the joints.
    std::string name_shoulder_L = "FLUJ";
    std::string name_shoulder_R = "FRUJ";
    std::string name_hip_L = "BLUJ";
    std::string name_hip_R = "BRUJ";
    std::string name_elbow_L = "FLDJ";
    std::string name_elbow_R = "FRDJ";
    std::string name_knee_L = "BLDJ";
    std::string name_knee_R = "BRDJ";

    /// \brief Pointers to the joints.
    private: physics::JointPtr joint_shoulder_L;
    private: physics::JointPtr joint_shoulder_R;
    private: physics::JointPtr joint_hip_L;
    private: physics::JointPtr joint_hip_R;
    private: physics::JointPtr joint_elbow_L;
    private: physics::JointPtr joint_elbow_R;
    private: physics::JointPtr joint_knee_L;
    private: physics::JointPtr joint_knee_R;

    /// \brief PID controllers for the joints.
    private: common::PID pid_shoulder_L;
    private: common::PID pid_shoulder_R;
    private: common::PID pid_hip_L;
    private: common::PID pid_hip_R;
    private: bool no_pid = false;

    /// \brief The original knee angle of the static model to add as an
    /// offset to the valuye measured by gazebo
    private: float knee_angle = 33.80731;

    /// \brief Constructor
    public: TigrilloPlugin() {}
    
    /// \brief A node use for ROS transport and its name
    private: std::unique_ptr<ros::NodeHandle> ros_node;
    private: std::string ros_node_name = "tigrillo_rob";

    /// \brief A ROS subscriber and its name
    private: std::string ros_sub_name = "uart_actuators";
    private: ros::Subscriber ros_sub;

    /// \brief A ROS publisher, its name and timer
    private: std::string ros_pub_name_s = "sim_sensors";
    private: ros::Publisher ros_pub_s;
    private: std::string ros_pub_name_m = "sim_motors";
    private: ros::Publisher ros_pub_m;
    private: std::unique_ptr<ros::Rate> pub_rate;
    private: int pub_freq;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue ros_queue;

    /// \brief A thread that keep running the ros_queue
    private: std::thread ros_queue_thread;

    /// \brief A thread to publish the sensor position over ROS
    private: std::thread ros_sen_thread;

    /// \brief A thread to publish the motor position over ROS
    private: std::thread ros_mot_thread;

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    /// \brief Set the position of all actuators
    /// \param[in] _pos Array of target position
    public: void SetPositionTarget(float _pos[]);

    /// \brief Retrieve all sensors
    /// \param[in] _sens Array of the sensor values
    public: void GetSensors(float _sens[]);

    /// \brief Retrieve all sensors
    /// \param[in] _sens Array of the sensor values
    public: void GetMotors(float _mot[]);
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the actuators 
    // positions
    public: void OnRosMsg(const tigrillo_2_plugin::MotorsConstPtr &_msg);

    /// \brief Thread that publishes the sensor messages on ROS
    private: void SendRosMsgSenThread();

    /// \brief Thread that publishes the motor messages on ROS
    private: void SendRosMsgMotThread();

    /// \brief Implement ROS initialization
    private: void RosInit();

    /// \brief ROS helper function that processes messages
    private: void QueueThread();
  };

}
#endif

