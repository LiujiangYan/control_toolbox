#ifndef ROS_CONTROLLER_HPP_
#define ROS_CONTROLLER_HPP_

#include "control/control.hpp"

#include <boost/scoped_ptr.hpp>
#include <util/unique_queue.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "setpoint_generator_following.hpp"
#include "setpoint_generator_homing.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

enum ROS_CONTROL_MODE{
	ROS_FOLLOWING,
	ROS_HOMING,
};

using namespace KDL;
class RosControllerComponent : public TaskContext{
public:
	RosControllerComponent(const std::string& name,
			SetpointGeneratorFollowing* following_generator,
			SetpointGeneratorHoming* homing_generator,
			unsigned int joint_num);
	
	~RosControllerComponent();

	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

	void SetCommandPeriod(double period){m_command_period = period;m_setpoint_generator->SetCommandPeriod(period);}
	void SetUseRosTime(bool yes_no = true){m_use_ros_time = yes_no;m_setpoint_generator->SetUseRosTime(yes_no);}
	void SetRosTimeOut(double time_out){m_ros_time_out = time_out;m_setpoint_generator->SetRosTimeOut(time_out);}

	void OutputCommand(double* out){
		for(unsigned int i=0;i<m_joint_num;i++){
			out[i] = m_command_to_send[i];
		}
	}

	void InputJntState(const JntArrayAcc& new_state){
		m_joint_state = new_state;
		m_setpoint_generator->UpdatePos(new_state.q);
		m_homing_generator->UpdatePos(new_state.q);
		// 将位置、速度、力矩发布出去
		PubJntStateRealTime(m_joint_state);
	}

	void PubJntStateRealTime(const JntArrayAcc& ja);

private:
	bool Ready(){return ros::ok() and m_joint_command_sub.getNumPublishers()>0;}
	/*
	 * @brief 输入 jntstate指令, ---> RTT::InputPort(), connPolicy = 3
	 */
	static void* ReceiveRosCommand(void*);
	void JntStateCommandCB(const sensor_msgs::JointState::ConstPtr& command);
	/*
	 * @brief 输出 ROS控制器的状态, ---> RTT::OutputPort()
	 */
	void PubRosControllerState(std::string state);

	ControllerState m_state;
	SetpointGeneratorFollowing* m_setpoint_generator;
	SetpointGeneratorHoming* m_homing_generator;
	UniqueQueue<sensor_msgs::JointState> m_joint_command_buffer;
	pthread_t m_ros_receiver;
	unsigned int m_joint_num;
	JntArrayAcc m_joint_state;
	ROS_CONTROL_MODE m_ros_control_mode;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_jntstate_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > m_rt_control_state_pub;
	ros::Subscriber m_joint_command_sub;
	double m_command_to_send[6];
	double m_command_period;
	double m_ros_time_out;
	bool m_use_ros_time;
	sensor_msgs::JointState m_jntcmd;
	bool m_received_ros_command;
	ros::Time m_ros_start_command_time;

	OutputPort<std::string> pub_controller_state;
	OutputPort<JntArrayAcc&> pub_joint_state;
};

#endif