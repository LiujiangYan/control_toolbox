/*
 * setpoint_generator_homing.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: lithotek
 */

#include "control/setpoint_generator_homing.hpp"
#include "alarm_manage.hpp"
#ifdef ROS_INTERFACE
SetpointGeneratorHoming::SetpointGeneratorHoming(const std::string& name,
		double interpPeriod,
		int axisNum,
		JntArrayAcc upperLimits,
		JntArrayAcc lowerLimits,
		int axisMap[12]
)
:TaskContext(name, PreOperational),
 m_interp_period(interpPeriod),
 m_joint_num(axisNum),
 m_upper_limits(upperLimits),
 m_lower_limits(lowerLimits),
 m_jnt_move(m_joint_num, lowerLimits, upperLimits),
 m_current_pos(m_joint_num),
 m_this_state(m_joint_num){

		m_started = false;

		for(int i = 0; i < 12; i++)
		{
			m_axis_map[i] = axisMap[i];
		}
		m_commanded_stop = false;

		m_homing_step = 0;
		m_at_home = false;
//		m_printed = false;
		m_interp_state = INTERP_IDLE;

}

SetpointGeneratorHoming::~SetpointGeneratorHoming() {
}


int SetpointGeneratorHoming::Interpolate(double *pos_data){
	/*
	 * 先回到home点, 此应当是机器人的状态
	 */
	m_homing_step++;
	double time = m_homing_step * m_interp_period;

	if(time >m_jnt_move.GetDuration()){
		m_at_home = true;
		time = m_jnt_move.GetDuration();
		m_interp_state = INTERP_IDLE;
	}
	m_jnt_move.GetState(time,m_this_state);

	for(unsigned int i = 0; i < m_joint_num; i++){
		pos_data[m_axis_map[i] - 1] = m_this_state.q(i);
	}

	return 0;
}

bool SetpointGeneratorHoming::MoveTo(const JntArray& current, const JntArray& home){
	m_interp_state = INTERP_RUNNING;
	return m_jnt_move.SetStartEnd(current,home);
}

void SetpointGeneratorHoming::reset(){
	m_started = false;
	m_commanded_stop = false;
	m_homing_step = 0;
}
void SetpointGeneratorHoming::stop(){
	if(m_interp_state != INTERP_STOPPING){
		m_jnt_move.Stop(m_homing_step*m_interp_period);
		m_commanded_stop = true;
		m_interp_state = INTERP_STOPPING;
	}
}
void SetpointGeneratorHoming::GotoErrorState(const string error_code, const string arglist)
{
	m_interp_state = INTERP_ERROR;

	//上报报警日志记录
	AlarmManage::Instance()->ReportLogInfo(error_code, ALARM_SOURCE_INTERP, arglist);
	//有错误就停下
	stop();
}
#endif
