/*
 * setpoint_generator_homing.hpp
 *
 *  Created on: Apr 14, 2016
 *      Author: lithotek
 */

#ifndef SETPOINT_GENERATOR_HOMING_HPP_
#define SETPOINT_GENERATOR_HOMING_HPP_

#include "interpolation_interface.hpp"
#include "planning/simple_jntmove.hpp"
#include "jntarrayacc.hpp"

#include "control/control.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

#ifdef ROS_INTERFACE
using namespace KDL;
class SetpointGeneratorHoming: public InterpolationInterface ： public TaskContext{
public:
	SetpointGeneratorHoming(const std::string& name,
			double interpPeriod,				// 插补周期
			int axisNum,								// 轴数
			KDL::JntArrayAcc upperLimits,				// 轴限制 ---> otgc robot
			KDL::JntArrayAcc lowerLimits,				// 轴限制 ---> otgc robot
			int axisMap[12]);							// axis map
	virtual ~SetpointGeneratorHoming();

	int Interpolate(double *pos_data);

	bool MoveTo(const JntArray& current, const JntArray& home);

	void reset();
	void stop();

	bool ReachedHome(){return m_at_home;}

	void UpdatePos(const JntArray& q_in){m_current_pos=q_in;m_this_state.q =q_in;}

private:
	void GotoErrorState(const string error_code, const string arglist);

	double m_interp_period;

	unsigned int m_joint_num;

	KDL::JntArrayAcc m_upper_limits;
	KDL::JntArrayAcc m_lower_limits;

	SimpleJntMove m_jnt_move;
	JntArray m_current_pos;
	KDL::JntArrayAcc m_this_state;

	int m_axis_map[12];

	INTERP_STATUS m_interp_state;



	int m_homing_step;
	bool m_at_home;

	bool m_started;

	bool m_commanded_stop;





};

#endif /* SETPOINT_GENERATOR_HOMING_HPP_ */
#endif
