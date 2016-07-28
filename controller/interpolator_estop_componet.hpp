/*
 * interpolator_estop.hpp
 *
 *  Created on: May 17, 2016
 *      Author: hanson
 */

#ifndef INTERPOLATOR_ESTOP_HPP_
#define INTERPOLATOR_ESTOP_HPP_

#include "simple_estop.hpp"
#include "robot_model.hpp"
#include "alarm_manage.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

class InterpolatorEStopComponent : public TaskContext{ 
public:

	/*
	 * 用于由于某种原因使得前后指令点相差过大时的简单急停．
	 * todo: 或应在机器人'运动'状态监控中做；如果某种原因造成step的阻塞，此功能也无用
	 *  ABB有一个错误报警是监控的位置与指令相差过大；RTT component有种状态是FatalError
	 *
	 */

	InterpolatorEStopComponent(const std::string& name, KDL::RobotModel* robot, double interp_period);
	virtual ~InterpolatorEStopComponent();
	/*
	 * @brief 输入当前机器人各轴的运动状态：位置／速度／力矩
	 * todo:临时只用于急停的解决方案，InputPort<JntArrayAcc>
	 */
	void InputJntState(const KDL::JntArrayAcc& new_state) {
//		cout<<"new_state "<<new_state.q.size()<<","<<new_state.qdot.size()<<","<<new_state.qdotdot.size()<<endl;
//		cout<<"rob_state "<<m_simple_robot_state.q.size()<<","<<m_simple_robot_state.qdot.size()<<","<<m_simple_robot_state.qdotdot.size()<<endl;
		m_simple_robot_state = new_state;
		input_joint_state.write(new_state);
	}
	bool configureHook();
	bool startHook(){
		return true;
	}
	void updateHook();
	void reset();

	/*
	 * @brief 用于输出指令
	 */
	void OutputJntCmd(double* pos_data);

private:
	SimpleEStop m_estop;
	KDL::JntArrayAcc m_simple_robot_state; // 简单的机器人各轴的运动状态：　位置／速度／力矩　todo: 可能有个robot_state_monitor更好
	KDL::JntArrayAcc m_estop_output;

	int m_estop_step;
	bool m_started;
	bool m_stopped;
	double m_interp_period;

	InputPort<KDL::JntArrayAcc&> input_joint_state;
	OuputPort<double*> output_pos_data;
};

#endif
