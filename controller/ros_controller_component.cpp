#include "control/ros_controller.hpp"
#include "Logger.hpp"
#include "ros/ros_experiment.hpp"
#include "ros/ros_diagnose.hpp"

using namespace RTT;

RosControllerComponent::RosControllerComponent(const std::string& name,
		SetpointGeneratorFollowing* setpoint_generator,
		SetpointGeneratorHoming* homing_generator,
		unsigned int joint_num):
			TaskContext(name, PreOperational)
			m_setpoint_generator(setpoint_generator),
			m_homing_generator(homing_generator),
			m_joint_num(joint_num),
			m_joint_state(joint_num){
	
	m_state = Stopped;
	m_ros_control_mode = ROS_FOLLOWING;
	m_use_ros_time = false;
	m_ros_time_out = 0.03; //30ms
	m_command_period = 0.01; //10ms
	m_received_ros_command = false;

	this->ports->addPort("pub_controller_state", pub_controller_state);
	this->ports->addPort("pub_joint_state", pub_joint_state);
}

RosControllerComponent::~RosControllerComponent() {
}

bool RosControllerComponent::configureHook(){
	if (!pub_controller_state.isConnected()){
		return false;
	}
	if (!pub_joint_state.isConnected()){
		return false;
	}
	return true;
}

bool RosControllerComponent::startHook(){
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "ROKAE_ROS_CONTROL_NODE");
	ros::NodeHandle nh;

	m_jntcmd.position.resize(6);
	m_jntcmd.velocity.resize(6);
	m_jntcmd.effort.resize(6);

	m_rt_jntstate_pub.reset( new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh,"/rokae/joint_states",1));
	m_rt_jntstate_pub->msg_.position.resize(6);
	m_rt_jntstate_pub->msg_.velocity.resize(6);
	m_rt_jntstate_pub->msg_.effort.resize(6);

	m_rt_control_state_pub.reset( new realtime_tools::RealtimePublisher<std_msgs::String>(nh,"/rokae/ros_controller_state",1));
	m_rt_control_state_pub->msg_.data = "PreOperational";

	m_joint_command_sub = nh.subscribe("/rokae/joint_command",1,&RosController::JntStateCommandCB,this);

	pthread_create(&m_ros_receiver,NULL,&ReceiveRosCommand,this);

	m_setpoint_generator->SetCommandQueuePtr(&m_joint_command_buffer);

	//for test
	RosExperiment::GetInstance()->StartNode();
//	RosDiagnose::GetInstance()->StartNode();
	return true;
}

void RosControllerComponent::updateHook(){
	if(m_state == FatalError){
		return;
	}

	if(m_ros_control_mode == ROS_HOMING){
		if(m_homing_generator->ReachedHome()){
			// 如果已到零点，那么可以进入到跟随状态；此处控制模式是通过状态来改变的。
			m_ros_control_mode = ROS_FOLLOWING;
		}else{
			m_homing_generator->Interpolate(m_command_to_send);
		}
	}

	if(m_ros_control_mode == ROS_FOLLOWING){

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
			Logger::log()<<Logger::Info<<"RosController Running "<<Logger::endl;
			PubRosControllerState("Running");
		}
		if(m_state == Running){
			if(m_setpoint_generator->Interpolate(m_command_to_send) !=0 ){
				m_state = Exception;
				Logger::log()<<Logger::Info<<"RosController Error "<<Logger::endl;
				PubRosControllerState("Error");
			}
			//		Logger::log()<<Logger::Info<<"Step "<<Logger::endl;
		}
	}
}

void RosControllerComponent::stopHook(){
	PubRosControllerState("Stopped");
	Logger::log()<<Logger::Info<<"RosController Stopped "<<Logger::endl;
	m_setpoint_generator->stop();
}

void RosControllerComponent::JntStateCommandCB(const sensor_msgs::JointState::ConstPtr& command){
	if(command->position.size() != m_joint_num){
		return;
	}
	else{
		if(!m_received_ros_command){
			m_jntcmd.header.stamp = command->header.stamp;
			m_received_ros_command = true;
		}

		for(unsigned int i=0; i<m_joint_num;i++){
			m_jntcmd.position[i] = command->position[i];
			m_jntcmd.velocity[i] = command->velocity[i];
		}

		if(!m_use_ros_time){
			m_jntcmd.header.stamp += ros::Duration(m_command_period);
		}
		else{
			m_jntcmd.header.stamp = command->header.stamp;
		}

		if(!m_received_ros_command){
			m_jntcmd.header.stamp = command->header.stamp;
			m_received_ros_command = true;
		}

		m_joint_command_buffer.PushBack(m_jntcmd);
	}
}

void RosController::PubRosControllerState(std::string state){

	if(m_rt_control_state_pub->trylock()){
		m_rt_control_state_pub->msg_.data = state;
		m_rt_jntstate_pub->unlockAndPublish();
	}

	pub_controller_state.write(state);
}

void RosController::PubJntStateRealTime(const JntArrayAcc& ja){
	if(m_rt_jntstate_pub->trylock()){

		for(int i=0;i<6;++i){
			m_rt_jntstate_pub->msg_.position[i] = ja.q(i);
			m_rt_jntstate_pub->msg_.velocity[i] = ja.qdot(i);
			m_rt_jntstate_pub->msg_.effort[i] = ja.qdotdot(i);
		}

		m_rt_jntstate_pub->msg_.header.stamp = ros::Time::now();
		m_rt_jntstate_pub->unlockAndPublish();
	}

	pub_joint_state.write(ja);
}

void* RosController::ReceiveRosCommand(void*) {
	ros::spin();
}






















