/*
 * setpoint_generator.hpp
 *
 *  Created on: Mar 4, 2016
 *      Author: root
 */

#ifndef SETPOINT_GENERATOR_HPP_
#define SETPOINT_GENERATOR_HPP_

#include "control/control.hpp"

#ifdef ROS_INTERFACE

#include "global_data_structure.hpp"
#include "interpolation_interface.hpp"
#include "jntarrayacc.hpp"
#include "planning/simple_jntspline.hpp"
#include "util/thread_safe_queue.h"
#include "util/unique_queue.h"
#include "TimeService.hpp"
#include "sensor_msgs/JointState.h"
using namespace KDL;

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>
/*
 * 继承InterpolationInterface,但其中的虚函数没用,所以改掉了
 *
 * 从ros_server接受下个周期的目标值,插补出来的每个毫秒的值写入电机的buffer
 *
 * 周期2ms,
 *
 */

class SetpointGeneratorFollowing: public InterpolationInterface : public TaskContext{// RTT::OS::RunnableInterface {
public:
	SetpointGeneratorFollowing(const std::string& name,
			double interpPeriod,				// 插补周期
			int axisNum,								// 轴数
			KDL::JntArrayAcc upperLimits,				// 轴限制 ---> otgc robot
			KDL::JntArrayAcc lowerLimits,				// 轴限制 ---> otgc robot
			int axisMap[12]);							// axis map  ---> robot
	virtual ~SetpointGeneratorFollowing();

	/*
	 * @brief configuration part
	 */
	void SetCommandQueuePtr(UniqueQueue<sensor_msgs::JointState>* queue) {m_command = queue;}
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

	UniqueQueue<sensor_msgs::JointState>* m_command;

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

	sensor_msgs::JointState m_command_state;

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

#endif /* SETPOINT_GENERATOR_HPP_ */
#endif
