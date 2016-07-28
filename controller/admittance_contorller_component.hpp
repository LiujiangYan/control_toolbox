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
using namespace RTT;

class AdmittanceControllerComponet : public TaskContext{
public:
	AdmittanceControllerComponet(const std::string& name, RobotModel* m_robot);
	virtual ~AdmittanceControllerComponet();

	void InputJntState(const JntArrayAcc& new_state){
		m_joint_state = new_state;
		// 将位置、速度、力矩发布出去
		PubJntStateRealTime(m_joint_state);
	}
	void OutputCommand(double* out){
		for(unsigned int i=0;i<m_joint_num;i++){
			out[i] = m_command_to_send[i];
		}
	}

	void PubJntStateRealTime(const JntArrayAcc& ja);


	void PubServoCmdRealTime(const JntArrayAcc& ja);

	bool configureHook();
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
	bool Ready(){return ros::ok() and m_force_command_sub.getNumPublishers()>0;}
	static void* ReceiveRosCommand(void*);
	void ForceSensorCommandCB(const geometry_msgs::WrenchStamped::ConstPtr& command);
	/*
	 * @brief 输出 ROS控制器的状态, ---> RTT::OutputPort()
	 */
	void PubRosControllerState(std::string state);

	bool Compute();

	void GoExeception(const std::string error_code, const std::string arglist);

	RobotModel* m_robot;

	ControllerState m_state;

	UniqueQueue<std::vector<double> > m_force_sensor_buffer;

	unsigned int m_joint_num;
	std::vector<MovingAverageFilter> m_force_filter;
	std::vector<MovingAverageFilter> m_velocity_filter;

	JntArrayAcc m_joint_state;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_jntstate_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_cmd_state_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > m_rt_control_state_pub;
	ros::Subscriber m_force_command_sub;


	double m_command_to_send[6];
	double m_ros_time_out;

	bool m_received_ros_command;

	geometry_msgs::WrenchStamped m_force_init;

	Twist m_vel_cmd;

	std::vector<double> m_force_net_to_buffer;

	std::vector<double> m_force_net_filtered;

	ros::Time m_ros_start_command_time;

	ros::Duration m_duration_since_first_command;
	ros::Duration m_duration_since_start_run;

	double m_delta_vel;


	JntArrayAcc m_q_cmd;

	// last joint command
	JntArrayAcc m_last_q_cmd;

	JntArray m_vel_cap;
	JntArray m_diff_acc;

	JntArray m_max_acc;

	double m_period;

	pthread_t m_ros_receiver;

	long m_missing_count;

	Frame m_flan;

	OutputPort<std::string> pub_controller_state;
	OutputPort<JntArrayAcc&> pub_joint_state;
	OutputPort<JntArrayAcc&> pub_servo_cmd;
};

#endif /* ADMITTANCE_CONTROLLER_HPP_ */
