/*
 * cartesian_controller_pos.cpp
 *
 *  Created on: Apr 18, 2016
 *      Author: lithotek
 */

#include <control/cartesian_controller_pos.hpp>

#include "Logger.hpp"
#include "ros/ros_experiment.hpp"

using namespace RTT;
CartesianControllerComponent::CartesianControllerComponent(const std::string& name,
		ChainFkSolverPos* fksolver,
		ChainIkSolverPos* iksolver,
		SetpointGeneratorCartFollowing* sg,
		unsigned int joint_num):
			TaskContext(name, PreOperational),
			m_fksolver(fksolver),
			m_iksolver(iksolver),
			m_setpoint_generator(sg),
			m_joint_num(joint_num),
			m_joint_state(joint_num){

	m_state = Stopped;
	m_use_ros_time = false;
	m_ros_time_out = 0.03; //30ms
	m_command_period = 0.01; //10ms
	m_received_ros_command = false;

	this->ports()->addPort(input_joint_state);
	this->ports()->addPort(pub_joint_state);
	this->ports()->addPort(pub_joint_cart_state);
	this->ports()->addPort(pub_controller_state);
}	

CartesianControllerComponent::~CartesianControllerComponent() {
}

void CartesianControllerComponent::updateHook(){

	if(m_state == FatalError){
		return;
	}

//	if(m_ros_control_mode == ROS_HOMING){
//		if(m_homing_generator->ReachedHome()){
//			// 如果已到零点，那么可以进入到跟随状态；此处控制模式是通过状态来改变的。
//			m_ros_control_mode = ROS_FOLLOWING;
//		}else{
//			m_homing_generator->Interpolate(m_command_to_send);
//		}
//	}

//	if(m_ros_control_mode == ROS_FOLLOWING){

		//	if(m_state == GoingHome and m_ros_interp_task[0]->ReachedHome() and Equal(m_joint_state.q,m_home_pos,EPSILON4)){
		//		m_state = Ready;
//				Logger::log()<<Logger::Info<<"ROS_FOLLOWING "<<Logger::endl;
		//	}
		for(unsigned int i=0;i<m_joint_num;i++){
			m_command_to_send[i] = m_joint_state.q(i);
		}
		if(m_state == Stopped and Ready()){
			// singnal ready
			m_state = Running;
			Logger::log()<<Logger::Info<<"CartesianController Running "<<Logger::endl;
			PubControllerState("Running");
		}
		if(m_state == Running){
			if(m_setpoint_generator->Interpolate(m_command_to_send) !=0 ){
				m_state = Exception;
				Logger::log()<<Logger::Info<<"CartesianController Error "<<Logger::endl;
				PubControllerState("Error");
			}
			//		Logger::log()<<Logger::Info<<"Step "<<Logger::endl;
		}

//	}
}
bool CartesianControllerComponent::start(){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "LUOSHI_NODE");
	ros::NodeHandle nh;


	m_rt_jntstate_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/rokae/joint_states",1));
	m_rt_jntstate_pub->msg_.position.resize(6);
	m_rt_jntstate_pub->msg_.velocity.resize(6);
	m_rt_jntstate_pub->msg_.effort.resize(6);

	m_rt_control_state_pub.reset( new realtime_tools::RealtimePublisher<std_msgs::String>(nh,"/rokae/ros_controller_state",1));
	m_rt_control_state_pub->msg_.data = "PreOperational";

	m_cart_command_sub = nh.subscribe("/rokae/cartesian_command_pos",1,&CartesianControllerComponent::CartCommandCB,this);

	m_rt_tip_frame_pub.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(nh,"/rokae/robot_tip_frame",1));

	pthread_create(&m_ros_receiver,NULL,&ReceiveRosCommand,this);

	m_setpoint_generator->SetCommandQueuePtr(&m_cart_command_buffer);

	//for test
//	RosExperiment::GetInstance()->StartNode();
	return true;
}

void CartesianControllerComponent::stopHook(){
	//	cout<<"PubControllerState"<<endl;
	//	RosNode::GetInstance()->PubControllerState("Stopped");
	//	cout<<"StopRosMode Gone"<<endl;
	//	pthread_cancel(m_ros_receiver);
	//#ifdef ROS_INTERFACE
	//	ros::shutdown();
		//RosNode::GetInstance()->StopNode();
	PubControllerState("Stopped");
	Logger::log()<<Logger::Info<<"CartesianController Stopped "<<Logger::endl;
	m_setpoint_generator->stop();
}

void CartesianControllerComponent::reset(){
	m_received_ros_command =false;
	m_setpoint_generator->reset();
	m_state = Stopped;
	PubControllerState("Stopped");
	Logger::log()<<Logger::Info<<"CartesianController Stopped "<<Logger::endl;
}


void CartesianControllerComponent::CartCommandCB(const geometry_msgs::PoseStamped::ConstPtr& command){
	//	cout<<"call back"<<endl;
//	if(!m_printed){
//				Logger::log()<<Logger::Info<<"command pos,vel,t = "<<
//						command->position[0]<<","<<command->velocity[0]<<","<<command->header.stamp<<Logger::endl;
//				m_printed =true;
//			}

//	if(command->position.size() != m_joint_num){
//				cout<<"size not equal"<<endl;
//		return;
//	}else{
//		Logger::log()<<Logger::Info<<"-------------------------------------------------"<<Logger::endl;
//		Logger::log()<<Logger::Info<<"ros_commad publish time = "<<command->header.stamp<<Logger::endl;

		m_cart_cmd = *command;

//		MFrame tmp;
//		PoseToFrame(m_cart_cmd.pose,tmp);
//		ConfData cf;
//		int res = m_iksolver->CartToJnt(m_joint_state.q,tmp,cf,m_this_jntcmd);
//		if(res == -1){
//
//		}
//		if(res == -2){
//
//		}

		if(!m_received_ros_command){
			m_cart_cmd.header.stamp = command->header.stamp;
			m_received_ros_command = true;
		}

//		for(unsigned int i=0; i<m_joint_num;i++){
//			m_jntcmd.position[i] = m_this_jntcmd(i);
//			m_jntcmd.velocity[i] = (m_this_jntcmd(i) - m_last_jntcmd(i))/m_command_period;
//		}

		if(!m_use_ros_time){
			//如果不使用ros时钟的话，
			//				Logger::log()<<Logger::Info<<"不使用ROS Time"<<Logger::endl;
			m_cart_cmd.header.stamp += ros::Duration(m_command_period);
//			Logger::log()<<Logger::Info<<"m_jntcmd.header.stamp += ros::Duration(m_command_period) = "
//					<<m_jntcmd.header.stamp<<", cmd_period = "<<m_command_period<<Logger::endl;
		}else{
			//				Logger::log()<<Logger::Info<<"使用ROS Time"<<Logger::endl;
			m_cart_cmd.header.stamp = command->header.stamp;
		}

		if(!m_received_ros_command){
			m_cart_cmd.header.stamp = command->header.stamp;
			m_received_ros_command = true;
		}

		//
//		ros::Time this_time = ros::Time::now();
//		ros::Duration  delay = this_time - command->header.stamp;
//
//		delay_counter(delay,m_delayed_count);
//		if(!m_printed){
//			Logger::log()<<Logger::Info<<"command pos,vel,t = "<<
//					command->position[0]<<","<<command->velocity[0]<<","<<command->header.stamp<<Logger::endl;
//			m_printed =true;
//		}
//				cout<<command->position[0]<<endl;
//		for(unsigned int i=0; i<m_joint_command.q.size();i++){
//			m_joint_command.q(i) = command->position[i];
//			m_joint_command.qdot(i) = command->velocity[i];
//			m_joint_command.qdotdot(i) = command->effort[i];
//		}
//		m_joint_command_buffer->PushBack(m_joint_command);
//		/*
//		 * TODO:
//		 * 1.这种方案可能会产生由锁带来的控制延时，可能会挤掉控制点
//		 * 2.将queue中的数据类型变成sensor_msgs::JointState时，会出现buffer非空的时候，pushback的情况
//		 */
//		if(!m_joint_command_buffer->Empty()){
//			Logger::log()<<Logger::Info<<"Buffer Not Empty"<<Logger::endl;
//		}
		m_cart_command_buffer.PushBack(m_cart_cmd);

//	}
}
void CartesianControllerComponent::PubControllerState(std::string state){

	if(m_rt_control_state_pub->trylock()){
		m_rt_control_state_pub->msg_.data = state;
		m_rt_jntstate_pub->unlockAndPublish();
	}
	pub_controller_state.write(state);
}
void CartesianControllerComponent::PubJntStateRealTime(const JntArrayAcc& ja){
	if(m_rt_jntstate_pub->trylock()){

		for(int i=0;i<6;++i){
			m_rt_jntstate_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_pub->msg_.effort[i] = ja.qdotdot(i);
		}

		m_rt_jntstate_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_pub->unlockAndPublish();

	}
	pub_joint_state.write(state);
}
void CartesianControllerComponent::PubJntCartStateRealTime(const JntArrayAcc& ja, const Frame& frame){
	if(m_rt_jntstate_pub->trylock()){

		for(int i=0;i<6;++i){
			m_rt_jntstate_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_pub->msg_.effort[i] = ja.qdotdot(i);
		}

		m_rt_jntstate_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_pub->unlockAndPublish();

	}

	if(m_rt_tip_frame_pub->trylock()){
		FrameToPose(frame, m_rt_tip_frame_pub->msg_.pose);
		m_rt_tip_frame_pub->msg_.header.stamp = m_rt_jntstate_pub->msg_.header.stamp;
		m_rt_tip_frame_pub->unlockAndPublish();
	}
	pub_joint_cart_state.write(state);
}
void* CartesianControllerComponent::ReceiveRosCommand(void*) {
	ros::spin();
}

