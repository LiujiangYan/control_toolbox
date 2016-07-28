/*
 * setpoint_generator_cart_following.hpp
 *
 *  Created on: Apr 18, 2016
 *      Author: lithotek
 */

#ifndef SETPOINT_GENERATOR_CART_FOLLOWING_HPP_
#define SETPOINT_GENERATOR_CART_FOLLOWING_HPP_

#include "control/control.hpp"

#ifdef ROS_INTERFACE

#include "global_data_structure.hpp"
#include "interpolation_interface.hpp"
#include <kdl/jntarrayacc.hpp>
#include "planning/simple_jntspline.hpp"
#include "util/thread_safe_queue.h"
#include "util/unique_queue.h"
#include "TimeService.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

#include <kdl/frames.hpp>
using namespace KDL;
class SetpointGeneratorCartFollowingComponent: public InterpolationInterface : public TaskContext{// RTT::OS::RunnableInterface {
public:
	SetpointGeneratorCartFollowingComponent(
			const std::string& name,
			double interpPeriod,				// 插补周期
			int axisNum,								// 轴数
			KDL::JntArrayAcc upperLimits,				// 轴限制 ---> otgc robot
			KDL::JntArrayAcc lowerLimits,				// 轴限制 ---> otgc robot
			KDL::ChainIkSolverPos* iksolver,
			int axisMap[12]);							// axis map  ---> robot
	virtual ~SetpointGeneratorCartFollowingComponent();

	/*
	 * @brief configuration part
	 */
	void SetCommandQueuePtr(UniqueQueue<geometry_msgs::PoseStamped>* queue) {m_command = queue;}
	void SetCommandPeriod(double period){m_command_period = period;}
	void SetUseRosTime(bool yes_no = true){m_use_ros_time = yes_no;}
	void SetRosTimeOut(double time_out){m_ros_time_out = time_out;}

	int Interpolate(double *pos_data);
	/*
	 * Execution Engine Part
	 */
	bool initialize();
	void finalize();
	void reset();
	void stop();

	/*
	 * TaskContext Part
	 * Running
	 */
	void UpdatePos(const JntArray& q_in){m_current_pos=q_in;m_this_state.q = m_current_pos;}
private:
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

	// TODO:otgc robot_model中方法, 应当放入robot_model里面
	/*
	 * @brief 判断是否位置超限
	 */
	bool IsJntPosWithinLimits(const KDL::JntArray& q, unsigned int& index, double eps=0.0);
	/*
	 * @brief 轴速度超限
	 */
	bool IsJntVelWithinLimits(const KDL::JntArray& q, unsigned int& index, double eps=0.0);
	/*
	 * @brief 轴加速超限
	 */
	bool IsJntAccWithinLimits(const KDL::JntArray& q, unsigned int& index, double eps=0.0);
	void GotoErrorState(const string error_code, const string arglist);

	UniqueQueue<geometry_msgs::PoseStamped>* m_command;


//	KDL::ChainFkSolverPos* m_fksolver;
	KDL::ChainIkSolverPos* m_iksolver;

	Frame m_command_frame;

	double m_interp_period;

	unsigned int m_joint_num;

	KDL::JntArrayAcc m_upper_limits;
	KDL::JntArrayAcc m_lower_limits;

	SimpleJntSpline m_traj;

	int m_axis_map[12];

	KDL::JntArrayAcc m_goal_state;
	KDL::JntArrayAcc m_this_state;
	KDL::JntArray m_last_setpoint;

//	ros::Time m_this_time;

	int m_total; //
	int m_count;

	bool m_started;

	INTERP_STATUS m_interp_state;

	bool m_commanded_stop;

	JntArray m_current_pos;

	bool m_printed;

	geometry_msgs::PoseStamped m_cart_command;

	JntArray m_last_goal_pos;

	ros::Duration m_duration_since_start_run;
//
	ros::Duration m_duration_since_start_cmd;
//
	ros::Duration m_plan_duration;
//
	ros::Time m_last_command_time;
	ros::Time m_start_command_time;

	double m_command_period;

	// for test
	std::vector<long> m_test_record;
	int loop_num;

	bool m_use_ros_time;
	double m_ros_time_out;
};

#endif /* SETPOINT_GENERATOR_CART_FOLLOWING_HPP_ */
#endif
