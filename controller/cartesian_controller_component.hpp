/*
 * cartesian_controller_pos.hpp
 *
 *  Created on: Apr 18, 2016
 *      Author: lithotek
 */

#ifndef CARTESIAN_CONTROLLER_POS_HPP_
#define CARTESIAN_CONTROLLER_POS_HPP_


#include "control/control.hpp"
#include "control/setpoint_generator_cart_following.hpp"
#ifdef ROS_INTERFACE

#include <boost/scoped_ptr.hpp>
#include <util/unique_queue.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "chain_fk_solver.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

/*
	0x6060=0001h规划位置模式（ PP）；
	0x6060=0006h寻原点模式（ HM）；
	0x6060=0008h周期同步位置模式（ CSP）；
 *
 */
//enum ROS_CONTROL_MODE{
//	ROS_FOLLOWING,
//	ROS_HOMING,
//};

using namespace KDL;
class CartesianControllerComponent : public TaskContext {
public:
	CartesianControllerComponent(const std::string& name,
			ChainFkSolverPos* fksolver,
			ChainIkSolverPos* iksover,
			SetpointGeneratorCartFollowing* sg,
			unsigned int joint_num
			);
	virtual ~CartesianControllerComponent();
	/*
	 * @brief configuration part
	 */
	void SetCommandPeriod(double period){m_command_period = period;m_setpoint_generator->SetCommandPeriod(period);}
	void SetUseRosTime(bool yes_no = true){m_use_ros_time = yes_no;m_setpoint_generator->SetUseRosTime(yes_no);}
	void SetRosTimeOut(double time_out){m_ros_time_out = time_out;m_setpoint_generator->SetRosTimeOut(time_out);}

	/*
	 * @brief input output part which shall be replaced by RTT::InputPort and RTT::OutputPort
	 */
	void OutputCommand(double* out){
		for(unsigned int i=0;i<m_joint_num;i++){
			out[i] = m_command_to_send[i];
		}
	}
	/*
	 * @brief 输入
	 */
	void InputJntState(const JntArrayAcc& new_state){
		m_joint_state = new_state;
		m_setpoint_generator->UpdatePos(new_state.q);
		// 将位置、速度、力矩发布出去
		m_fksolver->JntToCart(m_joint_state.q,m_cart_state);
		PubJntCartStateRealTime(m_joint_state,m_cart_state);

		input_joint_state.write(new_state);
	}

	/*
	 * @brief 输出 机器人的状态, 此应该由robot本身来发布
	 */
	void PubJntStateRealTime(const JntArrayAcc& ja);

//	void PubTipFrameRealTime(const Frame& tip);
	void PubJntCartStateRealTime(const JntArrayAcc& ja, const Frame& frame);

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
	bool Ready(){return ros::ok() and m_cart_command_sub.getNumPublishers()>0;}
	/*
	 * @brief 输入 jntstate指令, ---> RTT::InputPort(), connPolicy = 3
	 */
	static void* ReceiveRosCommand(void*);
	void CartCommandCB(const geometry_msgs::PoseStamped::ConstPtr& command);
	/*
	 * @brief 输出 ROS控制器的状态, ---> RTT::OutputPort()
	 */
	void PubControllerState(std::string state);

	void PoseToFrame(const geometry_msgs::Pose& pose, Frame& frame){
		frame.p = Vector(pose.position.x,pose.position.y,pose.position.z);
		frame.M = KDL::Quaternion(pose.orientation.w,
				pose.orientation.x,pose.orientation.y,pose.orientation.z).ToRotation();

	}
	void FrameToPose(const Frame& frame, geometry_msgs::Pose& pose){
		KDL::Quaternion q = frame.M.ToQuaternion();
		pose.position.x = frame.p.x();
		pose.position.y = frame.p.y();
		pose.position.z = frame.p.z();
		pose.orientation.w = q.w;
		pose.orientation.x = q.v.x();
		pose.orientation.y = q.v.y();
		pose.orientation.z = q.v.z();
	}


	ChainFkSolverPos* m_fksolver;
	ChainIkSolverPos* m_iksolver;
//	ChainFkSolverPos_recursive m_fksover;
	ControllerState m_state;
	SetpointGeneratorCartFollowing* m_setpoint_generator;
//	UniqueQueue<sensor_msgs::JointState> m_joint_command_buffer;
	UniqueQueue<geometry_msgs::PoseStamped> m_cart_command_buffer;
//	UniqueQueue<sensor_msgs::JointState> m_joint_command_buffer;
	pthread_t m_ros_receiver;
	unsigned int m_joint_num;
	JntArrayAcc m_joint_state;
	Frame m_cart_state;//todo: otgc change this

	boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > m_rt_jntstate_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::String> > m_rt_control_state_pub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > m_rt_tip_frame_pub;

	ros::Subscriber m_cart_command_sub;
	double m_command_to_send[6];
	double m_command_period;
	double m_ros_time_out;
	bool m_use_ros_time;

	geometry_msgs::PoseStamped m_cart_cmd;
//	geometry_msgs::PoseStamped m_last_cmd;

//	sensor_msgs::JointState m_jntcmd;
//
	bool m_received_ros_command;
	ros::Time m_ros_start_command_time;

	InputPort<JntArrayAcc&> input_joint_state;
	OutputPort<JntArrayAcc&> pub_joint_state;
	OutputPort<JntArrayAcc&> pub_joint_cart_state;
	OutputPort<std::string> pub_controller_state;

};

#endif /* CARTESIAN_CONTROLLER_POS_HPP_ */
#endif
