/*
 * interpolator_estop.cpp
 *
 *  Created on: May 17, 2016
 *      Author: hanson
 */

#include "control/interpolator_estop.hpp"
#include "Logger.hpp"

using namespace KDL;
using namespace RTT;

InterpolatorEStopComponent::InterpolatorEStopComponent(const string& name, KDL::RobotModel* robot, double interp_period):
		TaskContext(name, PreOperational),
		m_estop(robot->GetJointNum(),robot->GetLowerJntKineConstraints(),robot->GetUpperJntKineConstraints()),
		m_simple_robot_state(robot->GetJointNum()),
		m_estop_output(robot->GetJointNum()),
		m_estop_step(0),
		m_started(false),
		m_stopped(false),
		m_interp_period(interp_period){

	this->ports()->addPort("input_joint_state", input_joint_state);
	this->ports()->addPort("output_pos_data", output_pos_data);
}

InterpolatorEStopComponent::~InterpolatorEStopComponent() {
}

bool configureHook(){
	if(!input_joint_state.isConnected()){
		return false;
	}
	if(!output_pos_data.isConnected()){
		return false;
	}
	return true;
}
void InterpolatorEStopComponent::updateHook(){
	if(!m_started){
		m_estop.SetStart(m_simple_robot_state);
		Logger::log()<<Logger::Info<<m_estop<<Logger::endl;
		m_started = true;
	}
	if(m_stopped){
		return;
	}
	double time = m_estop_step * m_interp_period;
	if(time < m_estop.GetDuration()){
		m_estop.GetState(time, m_estop_output);
	}else{
		m_estop.GetState(m_estop.GetDuration(), m_estop_output);
		m_stopped = true;
		AlarmManage::Instance()->ReportLogInfo("50050",ALARM_SOURCE_INTERP, "急停! 排查bug!");
	}

}
void InterpolatorEStopComponent::reset(){
	m_started = false;
	m_stopped = false;
}

void InterpolatorEStopComponent::OutputJntCmd(double* pos_data){
	if(pos_data == NULL){
		return;
	}
	for(unsigned int i=0; i<m_estop_output.q.rows(); i++){
		pos_data[i] = m_estop_output.q(i);
	}

	output_pos_data.write(pos_data);
}
