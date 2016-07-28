/*
 * admittance_controller.cpp
 *
 *  Created on: May 31, 2016
 *      Author: hanson
 */

#include <sstream>

#include "Logger.hpp"
#include "TimeService.hpp"
#include "alarm_manage.hpp"
#include "control/joint_controller_vel.hpp"

using namespace KDL;
using namespace RTT;
using namespace std;

JointControllerVel::JointControllerVel(const string& name, RobotModel* robot):
		TaskContext(name, PreOperational)
		m_robot(robot),
		m_joint_num(m_robot->GetJointNum()),
		m_q_cmd(m_joint_num),
		m_joint_state(m_joint_num),
		m_vel_cap(m_joint_num){

	m_state = Stopped;
	m_ros_time_out = 0.005; //5ms
	m_received_ros_command = false;

//	for(int i=0;i<m_joint_num;i++){
//		m_force_filter[i].SetWindowLength()
//	}

	m_period = 0.002;

	m_vel_cap.data<<1.0,1.0,1.0,1.0,1.0,1.0;

	m_missing_count=0;

	this->ports()->addPort("input_joint_state", input_joint_state);
	this->ports()->addPort("output_controller_state", output_controller_state);
	this->ports()->addPort("output_command", output_command);
	this->ports()->addPort("output_joint_state", output_joint_state);
	this->ports()->addPort("output_servo_state", output_servo_state);
}

JointControllerVel::~JointControllerVel() {
}

void JointControllerVel::updateHook(){

	if(m_state == FatalError){
		return;
	}

	for(unsigned int i=0;i<m_joint_num;i++){
		m_command_to_send[i] = m_joint_state.q(i);
	}
	if(m_state == Stopped and Ready()){
		// singnal ready
		m_state = Running;
		Logger::log()<<Logger::Info<<"RosController Running "<<Logger::endl;
		PubRosControllerState("Running");
	}
	if(m_state == Running){
		if(!Compute()){
			m_state = Exception;
			Logger::log()<<Logger::Info<<"RosController Error "<<Logger::endl;
			PubRosControllerState("Error");
		}
		//		Logger::log()<<Logger::Info<<"Step "<<Logger::endl;
	}

}

bool JointControllerVel::Compute(){


	TimeService::ticks start = TimeService::Instance()->getTicks();
	if(m_vel_cmd_buffer.GetElement(m_vel_command)){

		TimeService::nsecs duration = TimeService::Instance()->ticks2nsecs(TimeService::Instance()->ticksSince(start));
		if(duration > 1e6){
			Logger::log()<<Logger::Info<<"GetElement duration is "<<duration/1e6<<Logger::endl;
		}
		m_missing_count = 0;
	}else{
		m_missing_count++;
//		Logger::log()<<Logger::Info<<"m_force_sensor_buffer.GetElement failed: size = "<<m_force_sensor_buffer.Size()<<Logger::endl;
	}

	// filter vel
	for(int i=0;i<3;i++){
//		m_vel_cmd[i] = m_velocity_filter[i].process(m_vel_cmd[i]);
	}

	/*
	 * cap:
	 * cart vel > 100mm/s
	 */
//	if(m_vel_cmd.vel.Norm() > 0.1){
//		m_vel_cmd.vel = m_vel_cmd.vel.Normalized() * 0.1;
//	}

	// transfer cart vel to joint vel
//	int res = m_robot->GetIkVel()->CartToJnt(m_joint_state.q,m_vel_cmd,m_q_cmd.qdot);

	/*
	 * cap jntvel
	 */
	double scale = m_q_cmd.qdot.data.cwiseAbs().cwiseQuotient(m_vel_cap.data).maxCoeff();
	if(scale > 1.0){
		Logger::log()<<Logger::Info<<"joint vel cap : scale = "<<scale<<Logger::endl;
		m_q_cmd.qdot.data /= scale;
	}


	m_q_cmd.q.data = m_joint_state.q.data + m_q_cmd.qdot.data * m_period;


//	m_q_cmd.qdot = m_joint_state.qdot;
//	m_q_cmd.q.data = m_joint_state.q.data + m_q_cmd.qdot.data * m_period;

	PubServoCmdRealTime(m_q_cmd);

	if(m_missing_count > 2){
		stringstream errorInfo;
		errorInfo<<"ROS通信延时超过6ms";
		Logger::log()<<Logger::Info<<"pos is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"vel is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
		GoExeception("50050",errorInfo.str());
		return false;
	}
//	if(res <0 or m_robot->IsSingular(m_q_cmd.q) or m_robot->IsSingular(m_joint_state.q)){
//		Logger::log()<<Logger::Info<<"cmd is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
//		Logger::log()<<Logger::Info<<"state is "<<m_joint_state.q.data.transpose()<<Logger::endl;
//		Logger::log()<<Logger::Info<<"inverse velocity res =  "<<res<<Logger::endl;
//		GoExeception("50026","奇异点，");
//		return false;
//	}
	unsigned int idx;
	if(!m_robot->IsJntVelWithinLimits(m_q_cmd.qdot,idx,1)){
		stringstream errorInfo;
		errorInfo<<"轴"<<idx+1<<"超过限速";
		Logger::log()<<Logger::Info<<"pos is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"vel is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
		GoExeception("50026",errorInfo.str());
		return false;
	}
	if(!m_robot->IsJntPosWithinLimits(m_q_cmd.q,idx)){
		stringstream errorInfo;
		errorInfo<<"轴"<<idx+1<<"超过限位";
		Logger::log()<<Logger::Info<<"pos is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"vel is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
		GoExeception("50050",errorInfo.str());
		return false;
	}

	for(int i=0;i<6;i++){
		m_command_to_send[i] = m_q_cmd.q(i);
	}
	m_duration_since_start_run += ros::Duration(0.002);

	return true;
}

void JointControllerVel::GoExeception(const string error_code, const string arglist){
	m_state = Exception;
	//上报报警日志记录
	AlarmManage::Instance()->ReportLogInfo(error_code, ALARM_SOURCE_INTERP, arglist);
	//有错误就停下
	stop();
}
bool JointControllerVel::startHook(){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "ROKAE_ROS_CONTROL_NODE");
	ros::NodeHandle nh;

	m_rt_jntstate_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/rokae/joint_states",1));
	m_rt_jntstate_pub->msg_.position.resize(6);
	m_rt_jntstate_pub->msg_.velocity.resize(6);
	m_rt_jntstate_pub->msg_.effort.resize(6);

	m_rt_cmd_state_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/rokae/servo_cmd",1));
	m_rt_cmd_state_pub->msg_.position.resize(6);
	m_rt_cmd_state_pub->msg_.velocity.resize(6);
	m_rt_cmd_state_pub->msg_.effort.resize(6);

	m_rt_control_state_pub.reset( new realtime_tools::RealtimePublisher<std_msgs::String>(nh,"/rokae/ros_controller_state",1));
	m_rt_control_state_pub->msg_.data = "PreOperational";

	m_vel_command_sub = nh.subscribe("/optoforce_0",1,&JointControllerVel::JointVelCommandCB,this);

	pthread_create(&m_ros_receiver,NULL,&ReceiveRosCommand,this);

	//for test
//	RosExperiment::GetInstance()->StartNode();
//	RosDiagnose::GetInstance()->StartNode();
	return true;
}

void JointControllerVel::stopHook(){
	PubRosControllerState("Stopped");
	Logger::log()<<Logger::Info<<"RosController Stopped "<<Logger::endl;
	Logger::log()<<Logger::Info<<"Count:  "<<m_missing_count<<Logger::endl;
}
void JointControllerVel::reset(){
	m_received_ros_command =false;
	m_state = Stopped;
	PubRosControllerState("Stopped");
	Logger::log()<<Logger::Info<<"RosController Stopped "<<Logger::endl;
}

void JointControllerVel::JointVelCommandCB(const sensor_msgs::JointState::ConstPtr& command){

	if(!m_received_ros_command){
		m_vel_cmd_init = *command;
		m_received_ros_command = true;
		m_ros_start_command_time=command->header.stamp;
	}

	/*
	 */
	for(unsigned int i=0;i<6;i++){
		m_vel_command[i] = command->velocity[i];
	}

	/*
	 * filter force
	 */
	for(int i=0;i<6;i++){
//		m_vel_command[i] = m_force_filter[i].process(m_force_net_to_buffer[i]);
	}

	/*
	 * relay to compute
	 */
//	m_vel_cmd_buffer.PushBack(m_force_net_to_buffer);

	m_duration_since_first_command = command->header.stamp - m_ros_start_command_time;

}

void JointControllerVel::PubRosControllerState(std::string state){

	if(m_rt_control_state_pub->trylock()){
		m_rt_control_state_pub->msg_.data = state;
		m_rt_jntstate_pub->unlockAndPublish();
	}
	output_controller_state.write(state);
}
void JointControllerVel::PubJntStateRealTime(const JntArrayAcc& ja){
	if(m_rt_jntstate_pub->trylock()){

		for(int i=0;i<6;++i){
			m_rt_jntstate_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_pub->msg_.effort[i] = ja.qdotdot(i);
		}

		m_rt_jntstate_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_pub->unlockAndPublish();

	}
	output_joint_state.write(ja);
}
void JointControllerVel::PubServoCmdRealTime(const JntArrayVel& ja){
	if(m_rt_cmd_state_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_cmd_state_pub->msg_.position[i] = ja.q(i);
			m_rt_cmd_state_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_cmd_state_pub->msg_.effort[i] = 0.0;
		}
		m_rt_cmd_state_pub->msg_.header.stamp = ros::Time::now();
		m_rt_cmd_state_pub->unlockAndPublish();
	}
	output_servo_state.write(ja);
}
void* JointControllerVel::ReceiveRosCommand(void*) {
	ros::spin();
}
