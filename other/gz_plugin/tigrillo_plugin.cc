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

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace gazebo
{
	/// \brief A plugin to control a Tigrillo quadruped with ROS.
	class TigrilloPlugin : public ModelPlugin
	{
		/// \brief Pointer to the model.
		private: physics::ModelPtr model;
		
		/// \brief Name of the joints.
		std::string name_shoulder_L = "left_shoulder";
		std::string name_shoulder_R = "right_shoulder";
		std::string name_hip_L = "left_hip";
		std::string name_hip_R = "right_hip";
		std::string name_elbow_L = "left_elbow";
		std::string name_elbow_R = "right_elbow";
		std::string name_knee_L = "left_knee";
		std::string name_knee_R = "right_knee";

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
		
		/// \brief Constructor
		public: TigrilloPlugin() {}
		
		/// \brief A node use for ROS transport and its name
		private: std::unique_ptr<ros::NodeHandle> ros_node;
		private: std::string ros_node_name = "tigrillo_rob";

		/// \brief A ROS subscriber and its name
		private: std::string ros_sub_name = "/tigrillo_ctrl/uart_actuators";
		private: ros::Subscriber ros_sub;

		/// \brief A ROS publisher, its name and timer
		private: std::string ros_pub_name = "/tigrillo_rob/uart_sensors";
		private: ros::Publisher ros_pub;
		private: std::unique_ptr<ros::Rate> pub_rate;
		private: int pub_freq;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue ros_queue;

		/// \brief A thread the keeps running the ros_queue
		private: std::thread ros_queue_thread;

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

			ROS_INFO("Loading Tigrillo ROS plugin");
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

			// Get the four actuated joints
			this->joint_shoulder_L = this->model->GetJoint(this->name_shoulder_L);
			this->joint_shoulder_R = this->model->GetJoint(this->name_shoulder_R);
			this->joint_hip_L = this->model->GetJoint(this->name_hip_L);
			this->joint_hip_R = this->model->GetJoint(this->name_hip_R);

			// Get the four passive joints
			this->joint_elbow_L = this->model->GetJoint(this->name_elbow_L);
			this->joint_elbow_R = this->model->GetJoint(this->name_elbow_R);
			this->joint_knee_L = this->model->GetJoint(this->name_knee_L);
			this->joint_knee_R = this->model->GetJoint(this->name_knee_R);

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
			float p_pos_0 = this->joint_shoulder_L->GetAngle(0).Radian();
			float p_pos_1 = this->joint_shoulder_R->GetAngle(0).Radian();
			float p_pos_2 = this->joint_hip_L->GetAngle(0).Radian();
			float p_pos_3 = this->joint_hip_R->GetAngle(0).Radian();
			ROS_INFO_STREAM("Updating position. Previous " << p_pos_0 << " "
				<< p_pos_1 << " " << p_pos_2 << " " << p_pos_3 << " and new: "
				<< _pos[0] << " " <<_pos[1] << " " << _pos[2] << " " << _pos[3]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_L->GetScopedName(), _pos[0]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_shoulder_R->GetScopedName(), _pos[1]);
			this->model->GetJointController()->SetJointPosition(
				this->joint_hip_L->GetScopedName(), _pos[2]);
			this->model->GetJointController()->SetPositionTarget(
				this->joint_hip_R->GetScopedName(), _pos[3]);
		}
		

		/// \brief Retrieve all sensors
		/// \param[in] _sens Array of the sensor values
		public: void GetSensors(float _sens[])
		{
			// Get joint position
			_sens[0] = this->joint_shoulder_L->GetAngle(0).Radian();
			_sens[1] = this->joint_shoulder_R->GetAngle(0).Radian();
			_sens[2] = this->joint_hip_L->GetAngle(0).Radian();
			_sens[3] = this->joint_hip_R->GetAngle(0).Radian();

			// Add noise?

		}

		
		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the actuators 
		// positions
		public: void OnRosMsg(const std_msgs::StringConstPtr &_msg)
		{
			// Convert JSON to array of gour floats 
			float position[4];
			rapidjson::Document d;

   			d.Parse(_msg->data.c_str());
   			rapidjson::Value& FL = d["FL"];
   			rapidjson::Value& FR = d["FR"];
   			rapidjson::Value& BL = d["BL"];
   			rapidjson::Value& BR = d["BR"];

   			int ver = 0;
   			if (FL.IsDouble()) {
   				position[0] = FL.GetDouble();
   				ver += 1;
   			}
   			if (FR.IsDouble()) {
   				position[1] = FR.GetDouble();
   				ver += 1;
   			}
   			if (BL.IsDouble()) {
   				position[2] = BL.GetDouble();
   				ver += 1;
   			}
   			if (BR.IsDouble()) {
   				position[3] = BR.GetDouble();
   				ver += 1;
   			}

   			if (ver == 4) {
   				this->SetPositionTarget(position);
   			} else {
   				ROS_WARN_STREAM("The format stored in topic " << 
   					this->ros_sub_name << " should be a dict of floats!");
   			}
		}


		/// \brief Publish the sensor messages to ROS
		public: void SendRosMsg()
		{
			// Convert to json string
			float sensors[4];
			this->GetSensors(sensors);
			rapidjson::Document d;
			d.SetObject();
			rapidjson::Document::AllocatorType& all = d.GetAllocator();
			d.AddMember("FL", sensors[0], all);
			d.AddMember("FR", sensors[1], all);
			d.AddMember("BL", sensors[2], all);
			d.AddMember("BR", sensors[3], all);
			rapidjson::StringBuffer strbuf;
			rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
			d.Accept(writer);

   			// Send over ROS
   			std_msgs::String msg;
   			msg.data = strbuf.GetString();
   			ROS_INFO_STREAM("Updating sensors: " << msg.data);
			this->ros_pub.publish(msg);

			// Wait till next time
			ros::spinOnce();
			this->pub_rate->sleep();
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
			ros::SubscribeOptions::create<std_msgs::String>(
				this->ros_sub_name,
				1,
				boost::bind(&TigrilloPlugin::OnRosMsg, this, _1),
				ros::VoidPtr(), &this->ros_queue);
			this->ros_sub = this->ros_node->subscribe(so);

			// Create a named topic for publication
			ROS_INFO_STREAM("Publish in topic: " + this->ros_pub_name);
			this->ros_pub = 
				this->ros_node->advertise<std_msgs::String>(this->ros_pub_name, 1);

			// Create a timer for publication
			this->pub_rate.reset(new ros::Rate(this->pub_freq));

			// Spin up the queue helper thread.
			this->ros_queue_thread =
				std::thread(std::bind(&TigrilloPlugin::QueueThread, this));

			// Publish the sensors on ROS in a loop
			while (this->ros_node->ok())
			{
				this->SendRosMsg();
			}
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
	GZ_REGISTER_MODEL_PLUGIN(TigrilloPlugin);
}
#endif
