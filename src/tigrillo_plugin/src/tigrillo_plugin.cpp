#ifndef _TIGRILLO2_PLUGIN_HH_
#define _TIGRILLO2_PLUGIN_HH_

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
#include "tigrillo_ctrl/Motors.h"
#include "tigrillo_ctrl/Imu.h"
#include "tigrillo_ctrl/Sensors.h"

namespace gazebo
{
	/// \brief A plugin to control a Tigrillo2 quadruped with ROS.
	class Tigrillo2Plugin : public ModelPlugin
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
		
		/// \brief Constructor
		public: Tigrillo2Plugin() {}
		
		/// \brief A node use for ROS transport and its name
		private: std::unique_ptr<ros::NodeHandle> ros_node;
		private: std::string ros_node_name = "tigrillo_rob";

		/// \brief A ROS subscriber and its name
		private: std::string ros_sub_name = "uart_actuators";
		private: ros::Subscriber ros_sub;

		/// \brief A ROS publisher, its name and timer
		private: std::string ros_pub_name = "sim_sensors";
		private: ros::Publisher ros_pub;
		private: std::unique_ptr<ros::Rate> pub_rate;
		private: int pub_freq;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue ros_queue;

		/// \brief A thread that keep running the ros_queue
		private: std::thread ros_queue_thread;

		/// \brief A thread to send the ROS messages
		private: std::thread ros_send_thread;

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

			ROS_INFO("Loading tigrillo 2 ROS plugin");
			ROS_INFO_STREAM("Gazebo is using the physics engine: " <<
			_model->GetWorld()->GetPhysicsEngine()->GetType());

			// Set Timer parameters
			int freq = 10;
			if (_sdf->HasElement("freq")) {
				freq = _sdf->Get<double>("freq");
			}
			this->pub_freq = freq;
			ROS_INFO_STREAM("Sensor Publication frequency has been set to: " << this->pub_freq << " Hz");

			// Set PID parameters
			double p = 5;
			double i = 0;
			double d = 0;
			if (_sdf->HasElement("p")) {
				p = _sdf->Get<double>("p");
			}
			if (_sdf->HasElement("i")) {
				i = _sdf->Get<double>("i");
			}
			if (_sdf->HasElement("d")) {
				d = _sdf->Get<double>("d");
			}
			ROS_INFO_STREAM("PID values are set to: " << p << "   " << i << "   " << d);

			// When the PID is only proportional equal to 1, control the motors in position directly
			if (p == 1 && i == 0 && d == 0)
				this->no_pid = true;

			// Safety check
			if (_model->GetJointCount() == 0)
			{
				ROS_ERROR("Invalid joint count, plugin not loaded");
				return;
			}

			// Store the model pointer for convenience.
			this->model = _model;

			// // Print all joints
			// std::vector<physics::JointPtr> jvec = this->model->GetJoints();
			// for(std::vector<physics::JointPtr>::iterator it = jvec.begin(); it != jvec.end(); ++it) {
			// 	std::cerr << (*it)->GetName() << "\n";
			// }
			// std::cout << this->model->GetName() + "::" + this->name_shoulder_L << std::endl;

			// Get the four actuated joints
			this->joint_shoulder_L = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_shoulder_L);
			this->joint_shoulder_R = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_shoulder_R);
			this->joint_hip_L = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_hip_L);
			this->joint_hip_R = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_hip_R);

			// Get the four passive joints
			this->joint_elbow_L = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_elbow_L);
			this->joint_elbow_R = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_elbow_R);
			this->joint_knee_L = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_knee_L);
			this->joint_knee_R = this->model->GetJoint(this->model->GetName() +
				"::" + this->name_knee_R);

			// Setup a P-controller, with a gain of 0.1.
			this->pid_shoulder_L = common::PID(p, i, d);
			this->pid_shoulder_R = common::PID(p, i, d);
			this->pid_hip_L = common::PID(p, i, d);
			this->pid_hip_R = common::PID(p, i, d);

			// Apply the P-controller to the joint.
			this->model->GetJointController()->SetPositionPID(
				this->joint_shoulder_L->GetScopedName(), this->pid_shoulder_L);
			this->model->GetJointController()->SetPositionPID(
				this->joint_shoulder_R->GetScopedName(), this->pid_shoulder_R);
			this->model->GetJointController()->SetPositionPID(
				this->joint_hip_L->GetScopedName(), this->pid_hip_L);
			this->model->GetJointController()->SetPositionPID(
				this->joint_hip_R->GetScopedName(), this->pid_hip_R);

			// Default to zero position
			float position[4] = {0, 0, 0, 0};
			this->SetPositionTarget(position);

			// Initialize ROS
			this->RosInit();
		}

		/// \brief Set the position of all actuators
		/// \param[in] _pos Array of target position
		public: void SetPositionTarget(float _pos[])
		{
			// Set the joint's target position.
			float p_pos_0 = this->joint_shoulder_L->GetAngle(0).Degree();
			float p_pos_1 = this->joint_shoulder_R->GetAngle(0).Degree();
			float p_pos_2 = this->joint_hip_L->GetAngle(0).Degree();
			float p_pos_3 = this->joint_hip_R->GetAngle(0).Degree();
			ROS_INFO_STREAM("Updating position. Previous " << p_pos_0 << " "
				<< p_pos_1 << " " << p_pos_2 << " " << p_pos_3 << " and new: "
				<< _pos[0] << " " <<_pos[1] << " " << _pos[2] << " " << _pos[3]);

			if (this->no_pid) {
				this->model->GetJointController()->SetJointPosition(
					this->joint_shoulder_L->GetScopedName(), _pos[0] * 0.01745329251);
				this->model->GetJointController()->SetJointPosition(
					this->joint_shoulder_R->GetScopedName(), _pos[1] * 0.01745329251);
				this->model->GetJointController()->SetJointPosition(
					this->joint_hip_L->GetScopedName(), _pos[2] * 0.01745329251);
				this->model->GetJointController()->SetJointPosition(
					this->joint_hip_R->GetScopedName(), _pos[3] * 0.01745329251);
			} else {
				this->model->GetJointController()->SetPositionTarget(
					this->joint_shoulder_L->GetScopedName(), _pos[0] * 0.01745329251);
				this->model->GetJointController()->SetPositionTarget(
					this->joint_shoulder_R->GetScopedName(), _pos[1] * 0.01745329251);
				this->model->GetJointController()->SetPositionTarget(
					this->joint_hip_L->GetScopedName(), _pos[2] * 0.01745329251);
				this->model->GetJointController()->SetPositionTarget(
					this->joint_hip_R->GetScopedName(), _pos[3] * 0.01745329251);
			}
			
		}
		

		/// \brief Retrieve all sensors
		/// \param[in] _sens Array of the sensor values
		public: void GetSensors(float _sens[])
		{
			// Get joint position
			_sens[0] = this->joint_elbow_L->GetAngle(0).Degree();
			_sens[1] = this->joint_elbow_R->GetAngle(0).Degree();
			_sens[2] = this->joint_knee_L->GetAngle(0).Degree();
			_sens[3] = this->joint_knee_R->GetAngle(0).Degree();

			// Add noise?

		}

		
		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the actuators 
		// positions
		public: void OnRosMsg(const tigrillo_ctrl::MotorsConstPtr &_msg)
		{
			// Convert message to array of four floats 
			float position[4];

   			position[0] = _msg->FL;
   			position[1] = _msg->FR;
   			position[2] = _msg->BL;
   			position[3] = _msg->BR;
   			
   			this->SetPositionTarget(position);
		}


		/// \brief Thread that publishes the sensor messages on ROS
		private: void SendRosMsgThread()
		{

			while (this->ros_node->ok())
			{
				// Get sensors values
				float sensors[4];
				this->GetSensors(sensors);

	   			// Send over ROS
	   			tigrillo_ctrl::Sensors msg;
	   			msg.FL = sensors[0];
	   			msg.FR = sensors[1];
	   			msg.BL = sensors[2];
	   			msg.BR = sensors[3];
	   			msg.run_time = this->model->GetWorld()->GetSimTime().Double() ;
	   			ROS_DEBUG_STREAM("Updating sensors: FL: " << msg.FL << " FR: " << 
	   				msg.FR << " BL: " << msg.BL << " BR: " << msg.BR );
				this->ros_pub.publish(msg);

				// Wait till next time
				ros::spinOnce();
				this->pub_rate->sleep();
			}
		}


		/// \brief Implement ROS initialization
		private: void RosInit()
		{
			
			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{

			ROS_INFO_STREAM("Create ROS Node: /" << this->ros_node_name);
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, this->ros_node_name,
					ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->ros_node.reset(new ros::NodeHandle(this->ros_node_name));

			// Create a named topic, and subscribe to it
			ROS_INFO_STREAM("Subscribe to topic: " + this->ros_sub_name);
			ros::SubscribeOptions so =
			ros::SubscribeOptions::create<tigrillo_ctrl::Motors>(
				this->ros_sub_name,
				1,
				boost::bind(&Tigrillo2Plugin::OnRosMsg, this, _1),
				ros::VoidPtr(), &this->ros_queue);
			this->ros_sub = this->ros_node->subscribe(so);

			// Create a named topic for publication
			ROS_INFO_STREAM("Publish in topic: " + this->ros_pub_name);
			this->ros_pub = 
				this->ros_node->advertise<tigrillo_ctrl::Sensors>(this->ros_pub_name, 1);

			// Create a timer for publication
			this->pub_rate.reset(new ros::Rate(this->pub_freq));

			// Spin up the queue helper thread.
			this->ros_queue_thread =
				std::thread(std::bind(&Tigrillo2Plugin::QueueThread, this));

			// Publish the sensors on ROS in a thread loop
			this->ros_send_thread =
				std::thread(std::bind(&Tigrillo2Plugin::SendRosMsgThread, this));
		}
		

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->ros_node->ok())
			{
				this->ros_queue.callAvailable(ros::WallDuration(timeout));
			}
		}
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(Tigrillo2Plugin);
}
#endif
