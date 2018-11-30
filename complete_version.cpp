/** @file client.cpp
*  @version 3.1.8
*  @date July 29th, 2016
*
*  @brief
*  All the exampls for ROS are implemented here.
*
*  @copyright 2016 DJI. All rights reserved.
*
*/
#include <dji_sdk_demo/demo_mission.h>
#include <ros/ros.h>
#include <stdio.h>
#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <string.h>
#include <fstream>
#include <ostream>
#include <time.h>
#include "std_msgs/Bool.h"
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

#define stay_height 0.50
#define box_height 0.50
#define detection_num 160
#define land_num 20
#define standard_detection_err 40
#define standard_land_err 0.05

#define goujian_latitude 90.0
#define goujian_longitude 90.0
#define goujian_altitude 5

#define dajian_latitude 90.0
#define dajian_longitude 90.0
#define dajian_altitude 5

#define set_time 540  //9*60
////////////////////////////////////waypoint////////////////////////////////////////
///waypoint1        takeoff后定点      |   waypoint2       起飞后上升的点      ////////
///waypoint3        起飞后实际上升的点   |   waypoint4       搭建区的点         ////////
///waypoint5        夹取构件后的点      |   waypoint6        夹取构件后上升的点  ////////
///waypoint7        夹取构件后上升的点   |   waypoint8        构建区的点        ////////
////////////////////////////////////////////////////////////////////////////////////

using namespace std;

ros::ServiceClient     waypoint_upload_service;
ros::ServiceClient     waypoint_action_service;
ros::ServiceClient 	   set_local_pos_reference;
ros::ServiceClient     sdk_ctrl_authority_service;
ros::ServiceClient     drone_task_service;
ros::ServiceClient     query_version_service;
ros::ServiceClient     drone_activation_service;

DJI::OSDK::Control control;

geometry_msgs::Vector3 d_msg;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Joy rc;

uint8_t current_gps_health = 0;
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

float detection_err[detection_num];     //识别圆圈的平面误差

bool detection;
float land_err[land_num];
bool capture;

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
	dji_sdk::SDKControlAuthority authority;
	authority.request.control_enable = 1;
	sdk_ctrl_authority_service.call(authority);

	if (!authority.response.result)
	{
		ROS_ERROR("obtain control failed!");
		return false;
	}
	ROS_INFO("control successfully");
	return true;
}
bool release_control()
{
	dji_sdk::SDKControlAuthority authority;
	authority.request.control_enable = 0;
	sdk_ctrl_authority_service.call(authority);

	if (!authority.response.result)
	{
		ROS_ERROR("release control failed!");
		return false;
	}
	ROS_INFO("release control successfully");
	return true;
}

void initwaypoint(dji_sdk::MissionWaypoint waypoint)
{
	waypoint.latitude            = 0;//wp->latitude;
    waypoint.longitude           = 0;//wp->longitude;
    waypoint.altitude            = 0;//wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
}

dji_sdk::MissionWaypoint gps_wp(sensor_msgs::NavSatFix msg)
{
	dji_sdk::MissionWaypoint waypoint;
	initwaypoint(waypoint);
	ros::spinOnce();
	waypoint.latitude            = msg.latitude;
    waypoint.longitude           = msg.longitude;
    waypoint.altitude            = msg.altitude;
	return waypoint;
}



void
setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 2;
  waypointTask.idle_velocity      = 1;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

bool runwaypoint(dji_sdk::MissionWaypoint waypoint1)
{
	ros::spinOnce();
	dji_sdk::MissionWaypointTask waypointTask;                               
    setWaypointInitDefaults(waypointTask);
    waypointTask.mission_waypoint.push_back(waypoint1); 
    ROS_INFO("Initializing Waypoint Mission..\n");
	if (initWaypointMission(waypointTask).result)
	{
	  ROS_INFO("Waypoint upload command sent successfully");
	}
	else
	{
	  ROS_WARN("Failed sending waypoint upload command");
	  return false;
	}

    // Waypoint Mission: Start
	if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
	                  MISSION_ACTION::START)
	      .result)
	{
	    ROS_INFO("Mission start command sent successfully");
	}
	else
	{
	    ROS_WARN("Failed sending mission start command");
	    return false;
	}

	return true;
}

//**************************************************************************************************************
void
setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

ServiceAck
initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_service.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck
missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action)
{
  dji_sdk::MissionWpAction missionWpAction;
  dji_sdk::MissionHpAction missionHpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_service.call(missionWpAction);
      if (!missionWpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return { missionWpAction.response.result,
               missionWpAction.response.cmd_set,
               missionWpAction.response.cmd_id,
               missionWpAction.response.ack_data };
  }
}

ServiceAck
activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if (!activation.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
             activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return ServiceAck(activation.response.result, activation.response.cmd_set,
                    activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck
obtainCtrlAuthority()
{
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
             sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                    sdkAuthority.response.cmd_id,
                    sdkAuthority.response.ack_data);
}

ServiceAck
takeoff()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck
land()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}


void go_point_s(float d_x, float d_y, float d_z, float d_yaw)
{

	float start_x;
	float start_y;
	float start_z;
	float x;
	float y;
	float z;
	float array_x;
	float array_y;
	float array_z;
	float i = 0;
	array_x = d_x / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	array_y = d_y / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	array_z = d_z / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	if (ros::ok())
	{
		ros::spinOnce();
		start_x = local_position.point.x;
		start_y = local_position.point.y;
		start_z = local_position.point.z;
	}
	printf("%f %f %f  %f\n", d_x, d_y, d_z, sqrt(d_x*d_x + d_y * d_y + d_z * d_z));

	for (int j = 0; j < 1000000 && ros::ok(); j++)
	{
		if (i<sqrt(d_x*d_x + d_y * d_y + d_z * d_z))
		{
			x = start_x + i * array_x;
			y = start_y + i * array_y;
			z = start_z + i * array_z;
			//printf("%f %f %f\n",x,y,z);
			if (i<sqrt(d_x*d_x + d_y * d_y + d_z * d_z) / 3)
				i = i + 0.005;
			else if (i>2 * sqrt(d_x*d_x + d_y * d_y + d_z * d_z) / 3)
				i = i + 0.005;
			else
				i = i + 0.010;
			control.positionAndYawCtrl(x, y, z, d_yaw);
			usleep(5000);
		}
		else
		{
			break;
		}
	}

}

void go_point_v(float d_x, float d_y, float d_z, float d_yaw)
{
	float array_x;
	float array_y;
	float array_z;

	float v_x = 0;
	float v_y = 0;
	float v_z = 0;

	float t;


	array_x = d_x / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	array_y = d_y / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	array_z = d_z / sqrt(d_x*d_x + d_y * d_y + d_z * d_z);

	float s = sqrt(d_x*d_x + d_y * d_y + d_z * d_z);
	float index = 0.0;
	/*if(s<=4.5||(s>=6.5&&s<8.5))  //the first time
	index = 0.52;
	else if(s>=9.5)
	index = 0.54;
	else
	index = 0.53;*/

	/*if(s<=6.5)                 //the third time
	index = 0.56;
	else if(s>=8)
	index=0.58;
	else
	index=0.57;*/

	if (s<6)                 //the second time
		index = 0.47;
	else if (s>7)
		index = 0.49;
	else
		index = 0.48;
	//printf("index: %f",index);
	t = sqrt(index*s * 20) * 50;//a=0.2;20<=4/a;;20<=1000ms/50ms
	for (int j = 0; j<t&&ros::ok(); j++)
	{
		if (j<t / 2)
		{
			v_x = v_x + array_x * 0.004; //0.004<=0.2/50
			v_y = v_y + array_y * 0.004;
			v_z = v_z + array_z * 0.004;
			control.velocityAndYawRateCtrl(v_x, v_y, v_z, d_yaw);
		}
		else
		{
			v_x = v_x - array_x * 0.004;
			v_y = v_y - array_y * 0.004;
			v_z = v_z - array_z * 0.004;
			control.velocityAndYawRateCtrl(v_x, v_y, v_z, d_yaw);
		}
		usleep(20000);  //50ms
						//printf("v_x:%f v_y:%f v_z:%f\n",v_x,v_y,v_z);
	}
	printf("%f %f %f  %f\n", d_x, d_y, d_z, sqrt(d_x*d_x + d_y * d_y + d_z * d_z));
	control.velocityAndYawRateCtrl(0, 0, 0, 0);
}

void Vector3_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	ros::spinOnce();
	d_msg.y = msg->x / 100.0;    //right and left of the camera    
	d_msg.x = -msg->y / 100.0;     //front and back of the camera
								   //printf("%f %f %f\n",d_msg.x,d_msg.y,d_msg.z);
}

class Pid
{
public:
	void pid(float Kp, float Ki, float Kd)
	{
		this->err = 0;
		this->err_last = 0;
		this->Actuator = 0;
		this->integral = 0;
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}
	float PID_realize(float err)
	{
		int index;
		this->err = err;
		/*if(this->Actuator > 0.5)                                 // maximum v*********************************************
		{
		if(fabs(this->err) > 2.4) index = 0;
		else
		{
		index = 1;
		if(this->err < 0) this->integral += this->err;
		}
		}
		else if(this->Actuator < -0.5)
		{
		if(fabs(this->err) > 2.4) index = 0;
		else
		{
		index = 1;
		if(this->err > 0) this->integral += this->err;
		}
		}
		else
		{
		if(fabs(this->err) > 2.4) index = 0;
		else
		{
		index = 1;
		this->integral += this->err;
		}
		}*/
		/*if(this->Actuator > 2.3||this->Kp <-2.3)
		{
		this->Kp = 0.8;
		}*/
		this->Actuator = this->Kp*this->err + this->Kd*(this->err - this->err_last);//+index*this->Ki*this->integral

		this->err_last = this->err;
		return this->Actuator;
	}
	void go_v(float d_x, float d_y, float d_z, float d_yaw)
	{
		err = sqrt(d_x*d_x + d_y * d_y);
		float v = this->PID_realize(err);
		float array_x = d_x / sqrt(d_x*d_x + d_y * d_y);//+d_z*d_z);
		float array_y = d_y / sqrt(d_x*d_x + d_y * d_y);//+d_z*d_z);

		control.velocityAndYawRateCtrl(v*array_x, v*array_y, 0.1*d_z, d_yaw);
	}
	float veloutput()
	{
		return this->Actuator;
	}
private:
	float err;
	float err_last;
	float Kp, Ki, Kd;
	float Actuator;
	float integral;
};

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	local_position = *msg;
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	current_gps_position = *msg;
}

void rc_callback(const sensor_msgs::Joy::ConstPtr& msg) {
	rc = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
	current_gps_health = msg->data;
}
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}
void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}



bool
monitoredTakeoff()
{
	ros::Time start_time = ros::Time::now();

	if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
		return false;
	}

	ros::Duration(0.01).sleep();
	ros::spinOnce();

	// Step 1.1: Spin the motor
	while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
		display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
		ros::Time::now() - start_time < ros::Duration(5)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (ros::Time::now() - start_time > ros::Duration(5)) {
		ROS_ERROR("Takeoff failed. Motors are not spinnning.");
		return false;
	}
	else {
		start_time = ros::Time::now();
		ROS_INFO("Motor Spinning ...");
		ros::spinOnce();
	}


	// Step 1.2: Get in to the air
	while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
		(display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
		ros::Time::now() - start_time < ros::Duration(20)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (ros::Time::now() - start_time > ros::Duration(20)) {
		ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
		return false;
	}
	else {
		start_time = ros::Time::now();
		ROS_INFO("Ascending...");
		ros::spinOnce();
	}

	// Final check: Finished takeoff
	while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
		ros::Time::now() - start_time < ros::Duration(20)) {
		ros::Duration(0.01).sleep();
		ros::spinOnce();
	}

	if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
	{
		ROS_INFO("Successful takeoff!");
		start_time = ros::Time::now();
	}
	else
	{
		ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
		return false;
	}

	return true;
}

bool set_local_position()
{
	dji_sdk::SetLocalPosRef localPosReferenceSetter;
	set_local_pos_reference.call(localPosReferenceSetter);

	return (bool)localPosReferenceSetter.response.result;
}

int main(int argc, char *argv[])
{
	double init_time = ros::Time::now().toSec();
	
	detection = false;
	capture = false;
	for (int i = 0; i < detection_num; i++)
	{
		detection_err[i] = 100;
	}
	for (int i = 0; i < land_num; i++)
	{
		land_err[i] = 20;
	}
	ofstream outfile;
	char name[100] = "//home//ubuntu//catkin_ws//src//Onboard-SDK-ROS-3.2//dji_sdk_demo//log//";
	time_t timer;
	struct tm *Now;
	time(&timer);
	Now = localtime(&timer);
	strcat(name, asctime(Now));
	outfile.open(name);
	//********************************************全局变量以及初始化********************************
	d_msg.x = 0;
	d_msg.y = 0;
	d_msg.z = 0;


	int main_operate_code = 0;
	int temp32;
	int circleRadius;
	int circleHeight;
	float Phi, circleRadiusIncrements;
	int x_center, y_center, yaw_local;
	bool valid_flag = false;
	bool err_flag = false;

	ros::init(argc, argv, "client");
	ROS_INFO("sdk_service_client_test");
	ros::NodeHandle nh;
	ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
	ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
	ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
	ros::Subscriber gpsSub = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
	ros::Subscriber rcsSub = nh.subscribe("dji_sdk/rc", 10, &rc_callback);
	ros::Subscriber gpsHealth = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

	drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");

	sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
	drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
	set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

	ros::spinOnce();

	//! Setting functions to be called for Mobile App Commands mode 
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("square/centerpoint", 10, &Vector3_callback);

	while (!rc.buttons[1])
			ros::spinOnce();
	printf("now<-6000\n");
	ros::spinOnce();
	while (ros::ok() && rc.buttons[1] < -6000)
	{
		ros::spinOnce();
	}
	printf("now>-6000\n");

	bool obtain_control_result = obtain_control();
	//monitoredTakeoff();                             //起飞******************************
	takeoff();
	sleep(2);
	//***************************航点从起飞点上升******************************************************************
	dji_sdk::MissionWaypoint waypoint1 = gps_wp(current_gps_position);
	sensor_msgs::NavSatFix takeoff_gps_position;
	takeoff_gps_position = current_gps_position;
	takeoff_gps_position.altitude = current_gps_position.altitude + 1;
	dji_sdk::MissionWaypoint waypoint2 = gps_wp(takeoff_gps_position);
	runwaypoint(waypoint2);
	ros::spinOnce();
	double now_time = ros::Time::now().toSec();
	double go_through_time = now_time - init_time;
	while (ros::ok()&& go_through_time < set_time)
	{
		detection = false;
		capture = false;	
        //***************************航点 从起飞点上升********************************************************************
		//##########                    到构件区     ****************************************************************
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		dji_sdk::MissionWaypoint waypoint4;
		initwaypoint(waypoint4);
		waypoint4.latitude  =  goujian_latitude;
		waypoint4.longitude =  goujian_longitude;
		waypoint4.altitude  =  goujian_altitude;
        runwaypoint(waypoint4);
		////////////////////////////////////////////////////////////

        ros::spinOnce();
		float z = local_position.point.z;

		ros::spinOnce();
		Pid drone_pid;
		while (!rc.buttons[1])
			ros::spinOnce();
		//****************************************************找箱子中心点循环********************************88
		while (ros::ok() && rc.buttons[1] < -6000 &&!detection)
		{
			ros::spinOnce();
			float dis = sqrt(d_msg.x*d_msg.x + d_msg.y*d_msg.y + d_msg.z*d_msg.z);
			float d_err = sqrt(d_msg.x*d_msg.x + d_msg.y*d_msg.y);

			if (d_msg.y || d_msg.x)
			{
				for (int j = detection_num - 1; j >0; j--)
				{
					detection_err[j] = detection_err[j - 1];
				}
				detection_err[0] = d_err;              //存放误差*****************
				float sum = 0.0;
				for (int k = 0; k < detection_num; k++)
				{
					sum += detection_err[k];
				}

				float average_detection_err = sum / detection_num;
				if (average_detection_err < standard_detection_err)
				{
					control.velocityAndYawRateCtrl(0, 0, 0, 0);
					detection = true;
					printf("detect successfully!\n");
				}

				ros::spinOnce();
				d_msg.z = z - local_position.point.z;


				if (!detection)
				{
					if (dis<1.5)
					{
						drone_pid.pid(0.13, 0, 0.01);                                     //pid parameter*********************************
						drone_pid.go_v(d_msg.x, d_msg.y, d_msg.z, 0);
					}
					else if (dis >= 1.5)
					{
						drone_pid.pid(0.13, 0, 0.01);
						drone_pid.go_v(d_msg.x, d_msg.y, d_msg.z, 0);
					}

					outfile << d_msg.x << " " << d_msg.y << " " << d_msg.z << " "
						<< "Distance.data: " << dis << " "
						<< "velocity: " << drone_pid.veloutput() << endl;
				}
				else
				{
					printf("detect successfully \n");
					printf("start landing");
				}

				usleep(200000);
			}
			else
			{
				go_point_v(0, 0, 0.15, 0);
				sleep(2);
			}
		}
			//**********************************************进入降落到固定高度进行抓取循环**********************************8
		ros::spinOnce();
		int ori_z = local_position.point.z;
		while (detection && !capture && ros::ok())
		{
			ros::spinOnce();
			while (local_position.point.z <= 0)
			{
				ros::spinOnce();
				drone_pid.go_v(d_msg.x, d_msg.y, 0, 0);
			}
			for (int j = land_num - 1; j >0; j--)
			{
				land_err[j] = land_err[j - 1];
			}
			land_err[0] = fabs(local_position.point.z - stay_height - box_height);
			cout << "local_position.point.z" << local_position.point.z << endl;

			float Distance_sum = 0;
			for (int k = 0; k < land_num; k++)
			{
				Distance_sum += land_err[k];
			}
			float average_Distance = (float)Distance_sum / land_num;
			if (average_Distance<standard_land_err)
			{
				control.velocityAndYawRateCtrl(0, 0, 0, 0);
				drone_pid.go_v(d_msg.x, d_msg.y, 0, 0);
				capture = true;
			}
			if (!capture)
			{
				if (local_position.point.z - box_height > 2 * stay_height)
				{
					float vel_z = -0.8*(local_position.point.z - stay_height - box_height) / (float)ori_z;
					control.velocityAndYawRateCtrl(0, 0, vel_z, 0);  //25cm:堆起的箱子高度
					cout << "vel of z : " << vel_z << endl;
					drone_pid.go_v(d_msg.x, d_msg.y, 0, 0);
					usleep(200000);
				}
				else if (local_position.point.z - box_height <= 2 * stay_height && local_position.point.z - box_height >= stay_height)
				{
					float vel_z = -0.5*(local_position.point.z - box_height - stay_height) / (float)ori_z;
					control.velocityAndYawRateCtrl(0, 0, vel_z, 0);
					cout << "vel of z : " << vel_z << endl;
					drone_pid.go_v(d_msg.x, d_msg.y, 0, 0);
					usleep(200000);
				}
				else
				{
					float vel_z = -0.6*(local_position.point.z - box_height - stay_height) / (float)ori_z;
					control.velocityAndYawRateCtrl(0, 0, vel_z, 0);
					cout << "vel of z : " << vel_z << endl;
					drone_pid.go_v(d_msg.x, d_msg.y, 0, 0);
					usleep(200000);
				}
			}
		}
		//*****************************抓取成功后升到固定高度，并跑航点到搭建区************************************
		if(capture)
		{
			ros::spinOnce();
			dji_sdk::MissionWaypoint waypoint5 = gps_wp(current_gps_position);
			sensor_msgs::NavSatFix rise_gps_position;
			rise_gps_position = current_gps_position;
			rise_gps_position.altitude = current_gps_position.altitude + 1;
			dji_sdk::MissionWaypoint waypoint6 = gps_wp(rise_gps_position);
			runwaypoint(waypoint6);
			/////////////////////////////////////////////////////////////////////////
			//##########                    到搭建区     *****************************
			/////////////////////////////////////////////////////////////////////////
			dji_sdk::MissionWaypoint waypoint8;
			initwaypoint(waypoint8);
			waypoint8.latitude  =  dajian_latitude;
			waypoint8.longitude =  dajian_longitude;
			waypoint8.altitude  =  dajian_altitude;
			runwaypoint(waypoint8);
		}
		ros::spinOnce();	
		now_time = ros::Time::now().toSec();
		go_through_time = now_time - init_time;
	}
	runwaypoint(waypoint1);
	land();
	outfile.close();
	return 0;
}

