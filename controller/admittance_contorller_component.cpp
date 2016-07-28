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
#include "control/admittance_controller.hpp"
#include "control/filter/exponetial_filter.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/RTT.hpp>

using namespace KDL;
using namespace RTT;
using namespace std;

AdmittanceController::AdmittanceController(const std::string &name, RobotModel* robot):
		TaskContext(name, PreOperational)
		m_robot(robot),
		m_joint_num(m_robot->GetJointNum()),
		m_force_filter(m_robot->GetJointNum()),
		m_velocity_filter(m_robot->GetJointNum()),
		m_joint_state(m_joint_num),
		m_q_cmd(m_joint_num),
		m_last_q_cmd(m_joint_num),
		m_vel_cap(m_joint_num),
		m_diff_acc(m_joint_num),
		m_max_acc(m_joint_num){

	m_state = Stopped;
	m_ros_time_out = 0.005; //5ms
	m_received_ros_command = false;

	m_force_net_to_buffer.resize(m_joint_num);
	m_force_net_filtered.resize(m_joint_num);

//	for(int i=0;i<m_joint_num;i++){
//		m_force_filter[i].SetWindowLength()
//	}

	m_delta_vel = 0.006;
	m_period = 0.002;

	m_vel_cap.data<<1.0,1.0,1.0,1.0,1.0,1.0;

	m_max_acc.data = 5*m_vel_cap.data;

	m_missing_count=0;

	for(int i=0;i<3;i++){
		m_velocity_filter[i].SetWindowLength(50);
		m_force_filter[i].SetWindowLength(60);
	}

	this->ports->addPort("pub_controller_state", pub_controller_state);
	this->ports->addPort("pub_joint_state", pub_joint_state);
	this->ports->addPort("pub_servo_cmd", pub_servo_cmd);
}

AdmittanceController::~AdmittanceController() {
}

bool configureHook(){
	if (!pub_controller_state.isConnected()){
		return false;
	}
	if (!pub_joint_state.isConnected()){
		return false;
	}
	if (!pub_servo_cmd.isConnected()){
		return false;
	}
	return true;
}

void AdmittanceController::updateHook(){

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

bool AdmittanceController::Compute(){

	/*
	 * joint states already measured
	 */
	m_robot->GetFkPos()->JntToCart(m_joint_state.q,m_flan);

	TimeService::ticks start = TimeService::Instance()->getTicks();
	if(m_force_sensor_buffer.GetElement(m_force_net_filtered)){

		TimeService::nsecs duration = TimeService::Instance()->ticks2nsecs(TimeService::Instance()->ticksSince(start));
		if(duration > 1e6){
			Logger::log()<<Logger::Info<<"GetElement duration is "<<duration/1e6<<Logger::endl;
		}
		Twist ft_end;
		for(int i=0;i<6;i++){
			ft_end[i] = m_force_net_filtered[i];
		}

//		ft_end = Rotation::RotY(0.5*PI) * ft_end;

		// transform to base frame
		ft_end.vel = m_flan.M * ft_end.vel;

		// ignore rot
		ft_end.rot = Vector::Zero();

		// transform force to cart vel
		m_vel_cmd = ft_end * m_delta_vel;
		m_missing_count = 0;
	}else{
		m_missing_count++;
//		Logger::log()<<Logger::Info<<"m_force_sensor_buffer.GetElement failed: size = "<<m_force_sensor_buffer.Size()<<Logger::endl;
	}

	// filter vel
	for(int i=0;i<3;i++){
		m_vel_cmd[i] = m_velocity_filter[i].process(m_vel_cmd[i]);
	}

	/*
	 * cap:
	 * cart vel > 100mm/s
	 */
	if(m_vel_cmd.vel.Norm() > 0.1){
		m_vel_cmd.vel = m_vel_cmd.vel.Normalized() * 0.1;
	}

	// transfer cart vel to joint vel
	int res = m_robot->GetIkVel()->CartToJnt(m_joint_state.q,m_vel_cmd,m_q_cmd.qdot);


	m_q_cmd.qdot(3) = 0.0;
	m_q_cmd.qdot(4) = 0.0;
	m_q_cmd.qdot(5) = 0.0;

	/*
	 * cap jntvel
	 */
	double scale = m_q_cmd.qdot.data.cwiseAbs().cwiseQuotient(m_vel_cap.data).maxCoeff();
	if(scale > 1.0){
//		Logger::log()<<Logger::Info<<"joint vel cap : scale = "<<scale<<Logger::endl;
		m_q_cmd.qdot.data /= scale;
	}

	/*
	 * constrain acc
	 */
	m_diff_acc.data = (m_q_cmd.qdot.data - m_last_q_cmd.qdot.data) / m_period;


	/*
	 * pos vel
	 */
	m_q_cmd.q.data = m_joint_state.q.data + 0.5 *(m_last_q_cmd.qdot.data +m_q_cmd.qdot.data) * m_period;
	m_q_cmd.qdotdot.data = (m_q_cmd.qdot.data - m_last_q_cmd.qdot.data) / m_period;

	bool use_planning = false;
	if(use_planning){

		double jerk = 10;
		unsigned int joint_num = 3;
		for(unsigned int i=0;i<joint_num;i++){
			if(m_q_cmd.qdot(i) > m_last_q_cmd.qdot(i) + EPSILON5){
				/*
				 * determine vel direction
				 * if needs accel choose positive jerk,
				 * otherwise choose negative jerk
				 */
				double plan_acc = m_last_q_cmd.qdotdot(i) + jerk * m_period;
				bool use_plan_acc = false;
				if(plan_acc > 2){
					plan_acc =2;
					use_plan_acc = true;
				}
				if(plan_acc > 2*fabs(m_last_q_cmd.qdot(i)) and !(fabs(m_last_q_cmd.qdot(i)<0.01))){
					plan_acc = 2*fabs(m_last_q_cmd.qdot(i));
					use_plan_acc = true;
				}

				double diff_jerk = (m_diff_acc(i) - m_last_q_cmd.qdotdot(i)) / m_period;
				if(diff_jerk > jerk or diff_jerk < -0.001 or use_plan_acc){
					/*
					 * if diff_jerk larger than max_jerk, or revers sign: use plan acc
					 */
					//				Logger::log()<<Logger::Info<<"m_diff_acc is "<<m_diff_acc.data.transpose()<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"diff_jerk is "<<diff_jerk<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"m_q_cmd.qdot is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"m_last_q_cmd.qdotdot is "<<m_last_q_cmd.qdotdot.data.transpose()<<Logger::endl;
					m_q_cmd.qdotdot(i) = plan_acc;
					m_q_cmd.qdot(i) = m_last_q_cmd.qdot(i) + plan_acc * m_period;
					m_q_cmd.q(i) = m_joint_state.q(i) + 0.5 * (m_last_q_cmd.qdot(i) + m_q_cmd.qdot(i)) * m_period;
				}

			}else if(m_q_cmd.qdot(i) < m_last_q_cmd.qdot(i) - EPSILON5){
				bool use_plan_acc = false;
				// needs negative jerk
				double plan_acc = m_last_q_cmd.qdotdot(i) - jerk * m_period;
				if(plan_acc < -2){
					plan_acc = -2;
					use_plan_acc = true;
				}
				if(plan_acc < 2*fabs(m_last_q_cmd.qdot(i)) and !(fabs(m_last_q_cmd.qdot(i)<0.01))){
					plan_acc = -2*fabs(m_last_q_cmd.qdot(i));
					use_plan_acc = true;
				}
				double diff_jerk = (m_diff_acc(i) - m_last_q_cmd.qdotdot(i)) / m_period;
				if(diff_jerk < -jerk or diff_jerk > 0.001 or use_plan_acc){
					//				Logger::log()<<Logger::Info<<"m_diff_acc is "<<m_diff_acc.data.transpose()<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"diff_jerk is "<<diff_jerk<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"m_q_cmd.qdot is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
					//				Logger::log()<<Logger::Info<<"m_last_q_cmd.qdotdot is "<<m_last_q_cmd.qdotdot.data.transpose()<<Logger::endl;
					/*
					 * if diff_jerk larger than max_jerk, or revers sign: use plan acc
					 */
					m_q_cmd.qdotdot(i) = plan_acc;
					m_q_cmd.qdot(i) = m_last_q_cmd.qdot(i) + plan_acc * m_period;
					m_q_cmd.q(i) = m_joint_state.q(i) + 0.5 * (m_last_q_cmd.qdot(i) + m_q_cmd.qdot(i)) * m_period;
				}
			}

		}


	}


	scale = m_q_cmd.qdot.data.cwiseAbs().cwiseQuotient(m_vel_cap.data).maxCoeff();
	if(scale > 1.0){
//		Logger::log()<<Logger::Info<<"joint vel cap : scale = "<<scale<<Logger::endl;
		m_q_cmd.qdot.data /= scale;
	}
	// exponential smooth
	double alpha = 0.03;
	for(unsigned int i=0;i<3;i++){
		m_q_cmd.qdot(i) = exponentialSmoothing(m_q_cmd.qdot(i), m_last_q_cmd.qdot(i),alpha);
	}
	m_q_cmd.q.data = m_joint_state.q.data + 0.5 * (m_q_cmd.qdot.data + m_last_q_cmd.qdot.data) * m_period;
	m_q_cmd.qdotdot.data = (m_q_cmd.qdot.data - m_last_q_cmd.qdot.data) / m_period;



	m_last_q_cmd = m_q_cmd;

//	m_q_cmd.qdot = m_joint_state.qdot;
//	m_q_cmd.q.data = m_joint_state.q.data + m_q_cmd.qdot.data * m_period;

	PubServoCmdRealTime(m_q_cmd);

	if(m_missing_count > 3){
		stringstream errorInfo;
		errorInfo<<"ROS通信延时超过6ms";
		Logger::log()<<Logger::Info<<"pos is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"vel is "<<m_q_cmd.qdot.data.transpose()<<Logger::endl;
		GoExeception("50050",errorInfo.str());
		return false;
	}
	if(res <0 or m_robot->IsSingular(m_q_cmd.q) or m_robot->IsSingular(m_joint_state.q)){
		Logger::log()<<Logger::Info<<"cmd is "<<m_q_cmd.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"state is "<<m_joint_state.q.data.transpose()<<Logger::endl;
		Logger::log()<<Logger::Info<<"inverse velocity res =  "<<res<<Logger::endl;
		GoExeception("50026","奇异点，");
		return false;
	}
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

	for(int i=0;i<3;i++){
		m_command_to_send[i] = m_q_cmd.q(i);
	}
	m_duration_since_start_run += ros::Duration(0.002);

	return true;
}

void AdmittanceController::GoExeception(const string error_code, const string arglist){
	m_state = Exception;
	//上报报警日志记录
	AlarmManage::Instance()->ReportLogInfo(error_code, ALARM_SOURCE_INTERP, arglist);
	//有错误就停下
	stop();
}
bool AdmittanceController::startHook(){
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

	m_force_command_sub = nh.subscribe("/optoforce_0",1,&AdmittanceController::ForceSensorCommandCB,this);

	pthread_create(&m_ros_receiver,NULL,&ReceiveRosCommand,this);

	//for test
//	RosExperiment::GetInstance()->StartNode();
//	RosDiagnose::GetInstance()->StartNode();
	return true;
}

void AdmittanceController::stopHook(){
	PubRosControllerState("Stopped");
	Logger::log()<<Logger::Info<<"RosController Stopped "<<Logger::endl;
	Logger::log()<<Logger::Info<<"Count:  "<<m_missing_count<<Logger::endl;
}

void AdmittanceController::reset(){
	m_received_ros_command =false;
	m_state = Stopped;
	PubRosControllerState("Stopped");
	Logger::log()<<Logger::Info<<"RosController Stopped "<<Logger::endl;
}

void AdmittanceController::ForceSensorCommandCB(const geometry_msgs::WrenchStamped::ConstPtr& command){

	if(!m_received_ros_command){
		m_force_init = *command;
		m_received_ros_command = true;
		m_ros_start_command_time=command->header.stamp;
	}

	/*
	 * m_force_net
	 */
//	m_force_net_to_buffer[0] = command->wrench.force.x - m_force_init.wrench.force.x;
//	m_force_net_to_buffer[1] = command->wrench.force.y - m_force_init.wrench.force.y;
//	m_force_net_to_buffer[2] = command->wrench.force.z - m_force_init.wrench.force.z;

	m_force_net_to_buffer[0] = -(command->wrench.force.z - m_force_init.wrench.force.z);
	m_force_net_to_buffer[1] = command->wrench.force.x - m_force_init.wrench.force.x;
	m_force_net_to_buffer[2] = command->wrench.force.y - m_force_init.wrench.force.y;

	m_force_net_to_buffer[3] = command->wrench.torque.x - m_force_init.wrench.torque.x;
	m_force_net_to_buffer[4] = command->wrench.torque.y - m_force_init.wrench.torque.y;
	m_force_net_to_buffer[5] = command->wrench.torque.z - m_force_init.wrench.torque.z;

	/*
	 * ignore force less than 0.5N
	 */
	for(int i=0; i<6; i++){
		if(m_force_net_to_buffer[i] < 0.5){
			m_force_net_to_buffer[i] = 0.0;
		}
	}

	/*
	 * filter force
	 */
	for(int i=0;i<6;i++){
		m_force_net_to_buffer[i] = m_force_filter[i].process(m_force_net_to_buffer[i]);
	}

	/*
	 * relay to compute
	 */
	m_force_sensor_buffer.PushBack(m_force_net_to_buffer);

	m_duration_since_first_command = command->header.stamp - m_ros_start_command_time;

}

void AdmittanceController::PubRosControllerState(std::string state){

	if(m_rt_control_state_pub->trylock()){
		m_rt_control_state_pub->msg_.data = state;
		m_rt_jntstate_pub->unlockAndPublish();
	}

	pub_joint_state.write(state);
}
void AdmittanceController::PubJntStateRealTime(const JntArrayAcc& ja){
	if(m_rt_jntstate_pub->trylock()){

		for(int i=0;i<6;++i){
			m_rt_jntstate_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_pub->msg_.effort[i] = ja.qdotdot(i);
		}

		m_rt_jntstate_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_pub->unlockAndPublish();

	}

	pub_controller_state(ja);
}

void AdmittanceController::PubServoCmdRealTime(const JntArrayAcc& ja){
	if(m_rt_cmd_state_pub->trylock()){
		for(int i=0;i<6;++i){
			m_rt_cmd_state_pub->msg_.position[i] = ja.q(i);
			m_rt_cmd_state_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_cmd_state_pub->msg_.effort[i] = ja.qdotdot(i);
		}
		m_rt_cmd_state_pub->msg_.header.stamp = ros::Time::now();
		m_rt_cmd_state_pub->unlockAndPublish();
	}

	pub_servo_cmd.write(ja);
}
void* AdmittanceController::ReceiveRosCommand(void*) {
	ros::spin();
}
