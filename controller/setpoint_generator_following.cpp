/*
 * setpoint_generator.cpp
 *
 *  Created on: Mar 4, 2016
 *      Author: root
 */
#include "setpoint_generator_following.hpp"
#ifdef ROS_INTERFACE
#include "ros/ros_node.hpp"
#include "Logger.hpp"
#include "alarm_manage.hpp"
#include "ros/ros_experiment.hpp"

#include "ros/ros_diagnose.hpp"

using namespace RTT;
using namespace KDL;
SetpointGeneratorFollowing::SetpointGeneratorFollowing(const string& name,
		double interpPeriod,
		int axisNum,
		JntArrayAcc upperLimits,
		JntArrayAcc lowerLimits,
		int axisMap[12]
):TaskContext(name, PreOperatioanl),
 m_interp_period(interpPeriod),
 m_joint_num(axisNum),
 m_upper_limits(upperLimits),
 m_lower_limits(lowerLimits),
 m_traj(axisNum),
 m_last_setpoint(m_joint_num),
 m_goal_state(m_joint_num),
 m_this_state(m_joint_num),
 m_current_pos(m_joint_num){

	m_count = 0;

	m_started = false;

	for(int i = 0; i < 12; i++)
	{
		m_axis_map[i] = axisMap[i];
	}
	m_commanded_stop = false;

	m_printed = false;

	for(int i=0;i<12;i++){
		m_test_record.push_back(0);
	}

	m_command_period = 0.01;
	m_use_ros_time = false;
	m_ros_time_out = 0.03;
	//	m_plan_duration = ros::Duration(m_command_period); //10 ms 控制延时为１０ms

	//	cout<<m_upper_limits.q.data<<endl;
	//	cout<<m_upper_limits.qdot.data<<endl;
	//	cout<<m_upper_limits.qdotdot.data<<endl;
	//	cout<<m_lower_limits.q.data<<endl;
	//	cout<<m_lower_limits.qdot.data<<endl;
	//	cout<<m_lower_limits.qdotdot.data<<endl;

	// note: 不要在构造时startNode(),因为此时如果roscore没有找到的话会失败,直接返回
	//	RosExperiment::GetInstance()->StartNode();

}
SetpointGeneratorFollowing::~SetpointGeneratorFollowing() {
}

bool SetpointGeneratorFollowing::initialize(){
	return true;
}

int SetpointGeneratorFollowing::Interpolate(double *pos_data) {
	int res = 0;
	if(m_interp_state == INTERP_ERROR)
	{
		return -1;
	}
	//
	if(m_commanded_stop){
		return -1;
	}

	/*
	 * 细插补ros指令
	 *
	 */
	//		m_this_time = ros::Time::now(); // time update with this_state;
	// 如果ros指令为空,则继续按原来轨迹走,否则从当前状态到下一状态计算多项式轨迹
	if(!m_command->GetElement(m_command_state)){
		//		if(!m_command->GetElement(m_goal_state)){
	}else{

		//		if(!m_started){
		//		Logger::log()<<Logger::Info<<"m_goal_state pos,vel = "<<
		//				m_goal_state.q(0)<<","<<m_goal_state.qdot(0)<<Logger::endl;
		//		}

		/*
		 * 记录初始时间，一是指令本身发出的时间，二是sg收到指令的时间；
		 *
		 * sg假定除sg之外都属于通信环节，包括sg读取queue的阶段，并依照此假设进行规划
		 */
		if(!m_started){ 		// like start() in TaskContext, run by ExecutionEngine using state machine
			//				m_run_start_time = ros::Time::now();
			//				m_command_start_time = m_command_state.header.stamp;
			m_duration_since_start_run = ros::Duration(0.0);
			m_duration_since_start_cmd = ros::Duration(0.0);
//			Logger::log()<<Logger::Info<<"RosController's Interpolator Started, goal is "<<m_command_state.position[0]<<Logger::endl;
			m_last_command_time = m_command_state.header.stamp - ros::Duration(m_command_period);
			/*
			 * 记录开始时间，以此作为comand时间轴和插补时间轴的共有起点。
			 */
			m_start_command_time = m_command_state.header.stamp - ros::Duration(m_command_period);
//			cout<<"m_start_command_time = "<<m_start_command_time<<endl;

			m_plan_duration = ros::Duration(m_command_period);
			m_this_state.q = m_current_pos;
			m_started = true;
		}

	/* 如果使用控制器时钟，计时放在了ros_controller
		ros::Duration cmd_time_diff = m_command_state.header.stamp - m_last_command_time;

		//*double(cmd_time_diff.toNSec() / ros::Duration(m_command_period).toNSec())
		double scale = double(cmd_time_diff.toNSec()) / double(ros::Duration(m_command_period).toNSec());

		if(Equal(scale,1.0,0.1)){
			scale = 1.0;
		}else if(Equal(scale,2.0,0.1)){
			scale = 2.0;
		}else if(Equal(scale,3.0,0.1)){
			scale = 3.0;
		}else if(Equal(scale,4.0,0.1)){
			scale = 4.0;
		}else if(Equal(scale,5.0,0.1)){
			scale = 5.0;
		}
		//			Logger::log()<<Logger::Info<<"\nscale =" <<scale<<Logger::endl;
		//			Logger::log()<<Logger::Info<<"\nscale =" <<scale<<", cmd_time_diff.toNSec() =  "
		//					<<cmd_time_diff.toNSec() << ", \nros::Duration(m_command_period).toNSec() = "<<ros::Duration(m_command_period).toNSec()
		//					<<",\ncmd_time_diff.toNSec() / ros::Duration(m_command_period).toNSec() = "<<cmd_time_diff.toNSec() / ros::Duration(m_command_period).toNSec()<<Logger::endl;

		m_duration_since_start_cmd += (ros::Duration(m_command_period) * scale) ; // 表示指令点距离指令轨迹起点的时间间隔,收到指令即说明还有一段距离

	*/
		m_duration_since_start_cmd = m_command_state.header.stamp - m_start_command_time;

//		Logger::log()<<Logger::Info<<"m_command_state.header.stamp = "<<m_command_state.header.stamp<<Logger::endl;
//		Logger::log()<<Logger::Info<<"m_duration_since_start_cmd = "<<m_duration_since_start_cmd<<Logger::endl;
//		Logger::log()<<Logger::Info<<"m_duration_since_start_run = "<<m_duration_since_start_run<<Logger::endl;

		m_plan_duration = m_duration_since_start_cmd - m_duration_since_start_run; // plan time =  N * 10ms - M * 2ms
//
//		Logger::log()<<Logger::Info<<"m_plan_duration = "<<m_plan_duration<<Logger::endl;

		m_last_command_time = m_command_state.header.stamp;


		for(unsigned int i=0; i<m_goal_state.q.size();i++){
			m_goal_state.q(i) = m_command_state.position[i];
			m_goal_state.qdot(i) = m_command_state.velocity[i];
//			m_goal_state.qdotdot(i) = m_command_state.effort[i];
		}

		// 如果有数据的话,就从当前状态规划到目标状态
		if(m_plan_duration > ros::Duration(0.0)){
			m_traj.SetGoalState(m_this_state, m_goal_state, m_plan_duration.toSec());
		}

		//			ros::Duration delay = ros::Time::now() - m_command_state.header.stamp;
		//			RosNode::GetInstance()->delay_counter(delay,m_delay_record);
		//			RosNode::GetInstance()->PubJntStateRealTimeTest(m_goal_state);
		//			Logger::log()<<Logger::Info<<"SetGoalState m_plan_duration is "<<m_plan_duration<<Logger::endl;

		m_count = 0;
	}


	if(!m_started){
		return 0;
	}

	//	cout<<"in loop "<<m_goal_state.q(0)<<endl;
	//	cout<<m_period_count*m_interp_period<<"/"<<m_traj.GetDuration()<<endl;
	m_traj.GetState((m_count + 1) * m_interp_period, m_this_state);

	RosExperiment::GetInstance()->publish_joint_states(m_this_state.q,0.002);
//	RosExperiment::GetInstance()->publish_joint_states(m_this_state);
//	RosDiagnose::GetInstance()->PubSetpointRealTime(m_this_state.q);

	m_duration_since_start_run += ros::Duration(m_interp_period);

	m_count++;

	if(m_count*m_interp_period-m_command_period > m_ros_time_out){
		Logger::log()<<Logger::Info<<"ROS通信延时超过"<<m_ros_time_out*1000<<"ms"<<Logger::endl;
//		Logger::log()<<Logger::Info<<m_c`ount*m_interp_period-m_command_period<<","
//				<<m_command_period<<Logger::endl;

		stringstream errorInfo;
		errorInfo<<"ROS通信延时超过"<<m_ros_time_out*1000<<"ms";
//		GotoErrorState("70030",errorInfo.str());// 错误信息暂不可用
		GotoErrorState("50050",errorInfo.str());
	}

	unsigned int index = m_joint_num + 1;
	if(!IsJntPosWithinLimits(m_this_state.q,index,EPSILON6)){
		Logger::log()<<"this_state and goal state: p0,v0,p1,v1 =  "<<m_this_state.q(0)<<
				m_this_state.qdot(0)<<m_goal_state.q(0)<<m_goal_state.qdot(0)<<Logger::endl;
		stringstream errorInfo;
		errorInfo<<"50050$轴"<<index+1<<"超过限位";
		GotoErrorState("50050","");
		return -1;
	}

	if(!IsJntVelWithinLimits(m_this_state.qdot,index,EPSILON6)){

		//		cout<<"this_state and goal state: t,p0,v0,p1,v1,index =  "<<m_traj.GetDuration()<<","<<m_this_state.q(index)<<","<<
		//				m_this_state.qdot(index)<<","<<m_goal_state.q(index)<<","<<m_goal_state.qdot(index)<<","<<index<<endl;
		Logger::log()<<Logger::Info<<"traj: t,p0,v0,p1,v1,vx,index, t = "<<m_traj.GetDuration()<<","
				<<m_traj.GetStart().q(index)<<","
				<<m_traj.GetStart().qdot(index)<<","
				<<m_traj.GetEnd().q(index)<<","
				<<m_traj.GetEnd().qdot(index)<<","
				<<m_this_state.qdot(index)<<","
				<<index+1<<","
				<<(m_count + 1) * m_interp_period<<","
				<<Logger::endl;
		stringstream errorInfo;
		errorInfo<<"轴"<<index+1<<"超过限速";
		GotoErrorState("50026",errorInfo.str());
		return -1;
	}

/*
	if(!IsJntAccWithinLimits(m_this_state.qdotdot,index,EPSILON6)){
		Logger::log()<<Logger::Info<<"IsJntAccWithinLimits traj: t,p0,v0,p1,v1,index, t = "<<m_traj.GetDuration()<<","
				<<m_traj.GetStart().q(index)<<","
				<<m_traj.GetStart().qdot(index)<<","
				<<m_traj.GetEnd().q(index)<<","
				<<m_traj.GetEnd().qdot(index)<<","
				<<m_this_state.qdotdot(index)<<","
				<<index+1<<","
				<<(m_count + 1) * m_interp_period<<","
				<<Logger::endl;
		stringstream errorInfo;
		errorInfo<<"轴"<<index+1<<"超过加速度限制";
		GotoErrorState("50050",errorInfo.str());
		return -1;
	}

*/
	//	JntArray v_diff(6);
	//	v_diff.data = (m_this_state.q.data - m_last_setpoint.data)/m_interp_period;
	//
	//	if(!IsJntVelWithinLimits(v_diff,index,EPSILON6)){
	////		stringstream errorInfo;
	////		errorInfo<<"50050$轴"<<index<<"超过速度限制";
	//		//TODO: 需要报警信息的完善
	//		GotoErrorState("50026");
	//		return;
	//	}

	//	m_last_setpoint = m_this_state.q;

	for(unsigned int i = 0; i < m_joint_num - 3; i++){
		pos_data[m_axis_map[i] - 1] = m_this_state.q(i);
	}
	return res;
}

void SetpointGeneratorFollowing::finalize(){

}
void SetpointGeneratorFollowing::reset(){
	m_count = 0;
	m_started = false;
	m_commanded_stop = false;
	m_interp_state = INTERP_RUNNING;
	m_this_state.qdot.data.setZero();
	m_this_state.qdotdot.data.setZero();
	m_command->Clear();
}
void SetpointGeneratorFollowing::stop(){
	m_commanded_stop = true;
}
/*
 * 判断是否位置超限
 */
bool SetpointGeneratorFollowing::IsJntPosWithinLimits(const KDL::JntArray& q, unsigned int& index, double eps){
	for(unsigned int i=0;i<m_joint_num;i++){
		if(q(i) <= (m_lower_limits.q(i) - eps) or q(i) >= (m_upper_limits.q(i) + eps)){
			index = i;
			return false;
		}
	}
	return true;
}

/*
 * 轴速度超限
 */
bool SetpointGeneratorFollowing::IsJntVelWithinLimits(const KDL::JntArray& qdot, unsigned int& index, double eps){
	for(unsigned int i=0;i<m_joint_num;i++){
		if(qdot(i) <= (m_lower_limits.qdot(i) - eps) or qdot(i) >= (m_upper_limits.qdot(i) + eps)){
			index = i;
			return false;
		}
	}
	return true;
}
/*
 * 轴加速超限
 */
bool SetpointGeneratorFollowing::IsJntAccWithinLimits(const KDL::JntArray& qdotdot, unsigned int& index, double eps){
	for(unsigned int i=0;i<m_joint_num;i++){
		if(qdotdot(i) <= (m_lower_limits.qdotdot(i) - eps) or qdotdot(i) >= (m_upper_limits.qdotdot(i) + eps)){
			index = i;
			return false;
		}
	}
	return true;
}

void SetpointGeneratorFollowing::GotoErrorState(const string error_code, const string arglist)
{
	m_interp_state = INTERP_ERROR;

	Logger::log()<<Logger::Info<<"traj: t,p0,v0,p1,v1,index, t = "<<m_traj.GetDuration()<<","
					<<m_traj.GetStart().q(0)<<","
					<<m_traj.GetStart().qdot(0)<<","
					<<m_traj.GetEnd().q(0)<<","
					<<m_traj.GetEnd().qdot(0)<<","
					<<m_this_state.qdot(0)<<","
					<<(m_count + 1) * m_interp_period<<","
					<<Logger::endl;
	//上报报警日志记录
	AlarmManage::Instance()->ReportLogInfo(error_code, ALARM_SOURCE_INTERP, arglist);
	//有错误就停下
	stop();

	//	stringstream test_res;
	//	for(unsigned int i=0;i<m_test_record.size();i++){
	//		test_res<<m_test_record[i]<<",";
	//	}
	//	Logger::log()<<Logger::Info<<test_res.str()<<Logger::endl;
	//		Logger::log()<<Logger::Info<<" Setpoint Generator statistics "<<
	//				m_delay_record[0]<<","<<
	//				m_delay_record[1]<<","<<
	//				m_delay_record[2]<<","<<
	//				m_delay_record[3]<<","<<
	//				m_delay_record[4]<<","<<
	//				m_delay_record[5]<<Logger::endl;
}

#endif
