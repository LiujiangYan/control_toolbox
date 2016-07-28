/*
 * admittance_controller.hpp
 *
 *  Created on: May 31, 2016
 *      Author: hanson
 */

#ifndef ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER_HPP_


#include "control/control.hpp"
#include <boost/scoped_ptr.hpp>
#include <util/unique_queue.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "filter/moving_average_filter.hpp"
#include "jntarrayacc.hpp"
#include "jntarrayvel.hpp"
#include "frames.hpp"
#include "robot_model.hpp"
#include "util/thread_safe_queue.h"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

using namespace KDL;

class JointControllerVelComponent {
public:
	JointControllerVelComponent(const string& name, RobotModel* m_robot);
	virtual ~JointControllerVelComponent();

	void InputJntState(const JntArrayAcc& new_state){
		input_joint_state.read(new_state);
		m_joint_state = new_state;
		// 将位置、速度、力矩发布出去
		PubJntStateRealTime(m_joint_state);
	}
	void OutputCommand(double* out){
		for(unsigned int i=0;i<m_joint_num;i++){
			out[i] = m_command_to_send[i];
		}
		output_command.write(out);
	}
	void PubJntStateRealTime(const JntArrayAcc& ja);

	void PubServoCmdRealTime(const JntArrayVel& ja);
	/*
	 * @brief 启动
	 */
	bool startHook();
	/*
	 * @brief control loop, called in realtime
	 */
	void updateHook();
	/*
	 * @brief
	 */
	void stopHook();

	/*
	 * @brief
	 */
	void reset();

private:
	bool Ready(){return ros::ok() and m_vel_command_sub.getNumPublishers()>0;}
	static void* ReceiveRosCommand(void*);
	void JointVelCommandCB(const sensor_msgs::JointState::ConstPtr& command);
	/*
	 * @brief 输出 ROS控制器的状态, ---> RTT::OutputPort()
	 */
	void PubRosControllerState(std::string state);

	bool Compute();

	void GoExeception(const std::string error_code, const std::string arglist);

	RobotModel* m_robot;

	ControllerState m_state;

	UniqueQueue<std::vector<double> > m_vel_cmd_buffer;

	std::vector<double> m_vel_command;

	unsigned int m_joint_num;

	JntArrayAcc m_joint_state;

	sensor_msgs::JointState m_vel_cmd_init;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_jntstate_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_cmd_state_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > m_rt_control_state_pub;
	ros::Subscriber m_vel_command_sub;

	double m_command_to_send[6];
	double m_ros_time_out;

	bool m_received_ros_command;

	ros::Time m_ros_start_command_time;

	ros::Duration m_duration_since_first_command;
	ros::Duration m_duration_since_start_run;

	JntArrayVel m_q_cmd;

	double m_period;

	pthread_t m_ros_receiver;

	JntArray m_vel_cap;

	long m_missing_count;

	InputPort<JntArrayAcc&> input_joint_state;
	OutputPort<double*> output_command;
	OutputPort<std::string> output_controller_state;
	OutputPort<JntArrayAcc&> output_joint_state;
	OutputPort<JntArrayAcc&> output_servo_state;
};

#endif /* ADMITTANCE_CONTROLLER_HPP_ */
