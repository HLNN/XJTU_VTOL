/** @file demo_mission.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use mission APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_mission.h>
#include <iostream>
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <ostream>
#include <math.h>
#include <time.h>
#include <cstdlib>
#include <stdlib.h>

using namespace std;
using namespace DJI::OSDK;

#define start_alt 10.0

ofstream outfile;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient query_version_service;

// global variables
ros::ServiceClient     waypoint_upload_service;
ros::ServiceClient     waypoint_action_service;
ros::ServiceClient     hotpoint_upload_service;
ros::ServiceClient     hotpoint_action_service;
ros::ServiceClient     hotpoint_update_yawRate_Service;
ros::ServiceClient     hotpoint_updateRadius_service;
ros::ServiceClient     drone_activation_service;
ros::ServiceClient     sdk_ctrl_authority_service;
ros::ServiceClient     drone_task_service;

//msg
sensor_msgs::NavSatFix gps_pos;
geometry_msgs::Quaternion current_atti;
//sensor_msgs::Joy vel;
geometry_msgs::Point local_pos;

//publisher
ros::Publisher velPub;
ros::Publisher ctrlBrakePub;

uint8_t flight_status = 255;
uint8_t display_mode  = 255;

ros::Publisher ctrlPosYawPub;

void go_point_s(float x,float y, float z);
void velocity_control(float vx,float vy, float vz);
void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void initwaypoint(dji_sdk::MissionWaypoint waypoint);
bool runWaypointMission(dji_sdk::MissionWaypoint waypoint,int responseTimeout);
void setWaypointDefaults(WayPointSettings* wp);
std::vector<DJI::OSDK::WayPointSettings> createWaypoints(dji_sdk::MissionWaypoint waypoint);
void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask);
bool runHotpointMission(int initialRadius, int responseTimeout);
void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);
void setHotpointInitDefault   (dji_sdk::MissionHotpointTask& hotpointTask);           
ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask);       
ServiceAck initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask);
ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action) ;
ServiceAck activate();
ServiceAck obtainCtrlAuthority();
ServiceAck takeoff();
ServiceAck land();
ServiceAck hotpointUpdateRadius(float radius);
ServiceAck hotpointUpdateYawRate(float yawRate, int direction);


//********************************************************************************************************************************************************
//********************************************************************************************************************************************************

int
main(int argc, char** argv)
{

  char name[100] = "//home//zxm//catkin_ws//src//Onboard-SDK-ROS-3.7//dji_sdk_demo/log//";
  time_t timer;
  struct tm *Now;
  time(&timer);
	Now=localtime(&timer);
	strcat(name,asctime(Now));
	outfile.open(name);

  ros::init(argc, argv, "sdk_demo_mission");
  ros::NodeHandle nh;

  //subscriber
   ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10, &gpsPosCallback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  //publisher
  velPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);


  // ROS stuff
  waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
    "dji_sdk/mission_waypoint_upload");
  waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
    "dji_sdk/mission_waypoint_action");
  hotpoint_upload_service = nh.serviceClient<dji_sdk::MissionHpUpload>(
    "dji_sdk/mission_hotpoint_upload");
  hotpoint_action_service = nh.serviceClient<dji_sdk::MissionHpAction>(
    "dji_sdk/mission_hotpoint_action");
  hotpoint_updateRadius_service =
    nh.serviceClient<dji_sdk::MissionHpUpdateRadius>(
      "dji_sdk/mission_hotpoint_updateRadius");
  hotpoint_update_yawRate_Service =
    nh.serviceClient<dji_sdk::MissionHpUpdateYawRate>(
      "dji_sdk/mission_hotpoint_updateYawRate");
  drone_activation_service =
    nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>(
    "dji_sdk/sdk_control_authority");
  drone_task_service =
    nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  
  // Activate
  if (activate().result)
  {
    ROS_INFO("Activated successfully");
  }
  else
  {
    ROS_WARN("Failed activation");
    return -1;
  }

  // Obtain Control Authority
  ServiceAck ack = obtainCtrlAuthority();
  if (ack.result)
  {
    ROS_INFO("Obtain SDK control Authority successfully");
  }
  else
  {
    if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
    {
      ROS_INFO("Obtain SDK control Authority in progess, "
               "send the cmd again");
      obtainCtrlAuthority();
    }
    else
    {
      ROS_WARN("Failed Obtain SDK control Authority");
      return -1;

    }
  }


monitoredTakeoff();

sleep(3);

int    responseTimeout = 1;

  
ros::spinOnce();
dji_sdk::MissionWaypoint waypoint1;
initwaypoint(waypoint1);
dji_sdk::MissionWaypoint waypoint2;
initwaypoint(waypoint2);
waypoint1.latitude  = gps_pos.latitude;                                    //******************
waypoint1.longitude = gps_pos.longitude;
waypoint1.altitude  = gps_pos.altitude;

cout <<"gps_pos.altitude"<<    gps_pos.altitude << endl; 
waypoint2.latitude  = gps_pos.latitude;                               
waypoint2.longitude = gps_pos.longitude;
waypoint2.altitude  = 5 ;
ROS_INFO("%f",waypoint2.latitude ) ;
//while(gps_pos.long)
runWaypointMission(waypoint2,responseTimeout);

sleep(1);
go_point_s(0.0,2.0,0.0);

go_point_s(2.0,0.0,0.0);
outfile << "finished 1\n";
go_point_s(0.0,0.0,-3.0);
land();
outfile.close();
return 0;
}

//********************************************************************************************************************************************************
//********************************************************************************************************************************************************

void go_point_s(float x,float y, float z)   //xyz --- NEU 
{
  ros::spinOnce();
  geometry_msgs::Point start_local_position = local_pos;
  float s = sqrt(x*x + y*y + z*z);
  float max_speed = s*1.5/5.0;
  int inbound = 0;
  int brake   = 0;
  int outbound = 0;
  float xCmd = 0;
  float yCmd = 0;
  float zCmd = 0;
  sensor_msgs::NavSatFix start_gps;
  start_gps = gps_pos;
  geometry_msgs::Vector3 current_relative_pos;

  float xoffset = 0;
	float yoffset = 0;
	float zoffset = 0;

  while(brake < 50 && ros::ok())
  {
	if(inbound < 50)
	{
		ros::spinOnce();
		localOffsetFromGpsOffset(current_relative_pos,gps_pos,start_gps);
		xoffset = x - current_relative_pos.x;
		yoffset = y - current_relative_pos.y;
		zoffset = z - (local_pos.z - start_local_position.z);

		// cout << "current_relative_pos.x : " << current_relative_pos.x << "\t"
		// 	<< "current_relative_pos.y : " << current_relative_pos.y << "\t"
		// 	<< "current_relative_pos.z : " << current_relative_pos.z << "\t" << endl;
		//cout << "gps_pos : " << gps_pos.altitude << "\t" << "start_gps : " << start_gps.altitude << endl;
		
		if (abs(xoffset) >= max_speed)
		xCmd = (xoffset>0) ? max_speed : -1 * max_speed;
		else
		xCmd = xoffset;

		if (abs(yoffset) >= max_speed)
		yCmd = (yoffset>0) ? max_speed : -1 * max_speed;
		else
		yCmd = yoffset;
		//if (abs(zoffset) >= max_speed)
		//zCmd = (zoffset>0) ? max_speed : -1 * max_speed;
		//else
		zCmd = start_local_position.z + z ;

    sensor_msgs::Joy vel;
		vel.axes.push_back(yCmd);
		vel.axes.push_back(xCmd);
		vel.axes.push_back(zCmd);
		vel.axes.push_back(0);

		velPub.publish(vel);
	}
	else if(inbound > 50 && brake == 0)
	{
		ROS_INFO("##### Route start brake....");
		brake = 1;
		sensor_msgs::Joy controlVelYawRate;
		uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
					DJISDK::HORIZONTAL_VELOCITY |
					DJISDK::YAW_RATE            |
					DJISDK::HORIZONTAL_GROUND   |
					DJISDK::STABLE_ENABLE);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(flag);

		ctrlBrakePub.publish(controlVelYawRate);
	}
	else
	{
		sensor_msgs::Joy controlVelYawRate;
		uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
					DJISDK::HORIZONTAL_VELOCITY |
					DJISDK::YAW_RATE            |
					DJISDK::HORIZONTAL_GROUND   |
					DJISDK::STABLE_ENABLE);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(0);
		controlVelYawRate.axes.push_back(flag);

		ctrlBrakePub.publish(controlVelYawRate);
		brake++;
	}
	ros::spinOnce();
  xoffset = x - current_relative_pos.x;
  yoffset = y - current_relative_pos.y;
	zoffset = z - (local_pos.z - start_local_position.z);

	if (abs(xoffset) < 0.5 &&
	abs(yoffset) < 0.5 &&
	abs(zoffset) < 0.5)
	{
		inbound ++;
	}
	else
	{
		if(inbound)
		{
			outbound ++;
		}
	}
	if(outbound > 10)
	{
		ROS_INFO("##### Route: out of bounds, reset....");
		inbound  = 0;
		outbound = 0;
	}
  ROS_INFO("yoffset  :  %f",yoffset);
  ROS_INFO("inbound  :  %d",inbound);
  ROS_INFO("outbound :  %d",outbound);
  ROS_INFO("brake    :  %d",brake);
  outfile<< "yoffset   :  " << yoffset << endl
        << "inbound   :  " << inbound << endl
        << "outbound  :  " << outbound << endl
        << "brake     :  " << brake << endl << endl;
  }
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  local_pos = msg->point;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

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
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
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
    //cout << ros::Time::now() - start_time << endl;
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
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

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
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


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = gps_pos.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      gps_pos.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

void velocity_control(float vx,float vy, float vz)
{
  sensor_msgs::Joy vel;
  vel.axes.push_back(vx);
  vel.axes.push_back(vy);
  vel.axes.push_back(vz);
  vel.axes.push_back(0.0);
  velPub.publish(vel);
}

void
gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos = *msg;
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

bool
runWaypointMission(dji_sdk::MissionWaypoint waypoint,int responseTimeout)
{
  ros::spinOnce();
  
  geometry_msgs::Vector3 target;
  sensor_msgs::NavSatFix gps_target;
  gps_target = gps_pos;
  gps_target.latitude = waypoint.latitude;
  gps_target.longitude = waypoint.longitude;
  gps_target.altitude = waypoint.altitude;
  localOffsetFromGpsOffset(target, gps_target,gps_pos);
  int inbound_num = 0;
  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
  setWaypointInitDefaults(waypointTask);

  // Waypoint Mission: Create Waypoints
  //float64_t increment = 0.000001 / C_PI * 180;
  
  ROS_INFO("Creating Waypoints..\n");
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(waypoint);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
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

  while(inbound_num < 50&&ros::ok())
  {
    ros::spinOnce();
    localOffsetFromGpsOffset(target, gps_target,gps_pos);
    if(abs(target.x) < 0.5 &&
      abs(target.y) < 0.5 &&
      abs(target.z) < 0.5 )
      {
        inbound_num ++;
      }
    cout << "target.x : " << target.x << "  "
          <<"target.y : " << target.y << "  "
          <<"target.z : " << target.z << "  "
          << endl;
  }   
  return true;
}

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

void
setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(dji_sdk::MissionWaypoint waypoint)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  WayPointSettings wp2;
  setWaypointDefaults(&start_wp);
  setWaypointDefaults(&wp2);
  start_wp.latitude  = gps_pos.latitude;
  start_wp.longitude = gps_pos.longitude;
  start_wp.altitude  = waypoint.altitude - 1.0;//gps_pos.altitude;
  wp2.latitude  = waypoint.latitude;
  wp2.longitude = waypoint.longitude;
  wp2.altitude  = waypoint.altitude;
  // ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
  //          gps_pos.longitude, start_alt);

  // std::vector<DJI::OSDK::WayPointSettings>
  // wpVector =
  //   generateWaypoints(&start_wp, waypoint);
  std::vector<DJI::OSDK::WayPointSettings> wp_list;
  start_wp.index = 0;
  wp2.index = 1;
  wp_list.push_back(start_wp);
  wp_list.push_back(wp2);
  return wp_list;
}

void
uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
             wp->longitude, wp->altitude);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}

bool
runHotpointMission(int initialRadius, int responseTimeout)
{
  ros::spinOnce();

  // Hotpoint Mission: Create hotpoint
  dji_sdk::MissionHotpointTask hotpointTask;
  setHotpointInitDefault(hotpointTask);

  // Hotpoint Mission: Initialize
  initHotpointMission(hotpointTask);

  // Takeoff
  if (takeoff().result)
  {
    ROS_INFO("Takeoff command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending takeoff command");
    return false;
  }
  ros::Duration(15).sleep();

  // Start
  ROS_INFO("Start with default rotation rate: 15 deg/s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::START).result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    return false;
  }
  ros::Duration(25).sleep();

  // Pause
  ROS_INFO("Pause for 5s");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::PAUSE).result)
  {
    ROS_INFO("Mission pause command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission pause command");
    return false;
  }
  ros::Duration(5).sleep();

  // Resume
  ROS_INFO("Resume");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::RESUME).result)
  {
    ROS_INFO("Mission resume command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission resume command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update radius, no ACK
  ROS_INFO("Update radius to 1.5x: new radius = %f", 1.5 * initialRadius);
  if (hotpointUpdateRadius(1.5 * initialRadius).result)
  {
    ROS_INFO("Hotpoint update radius command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending hotpoint update radius command");
    return false;
  }
  ros::Duration(10).sleep();

  // Update velocity (yawRate), no ACK
  ROS_INFO("Update hotpoint rotation rate: new rate = 5 deg/s");
  if (hotpointUpdateYawRate(5, 1).result)
  {
    ROS_INFO("Hotpoint update yaw rate command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending hotpoint update yaw rate command");
    return false;
  }
  ros::Duration(10).sleep();

  // Stop
  ROS_INFO("Stop");
  if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                    DJI::OSDK::MISSION_ACTION::STOP)
        .result)
  {
    ROS_INFO("Mission stop command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission stop command");
    return false;
  }

  ROS_INFO("land");
  if (land().result)
  {
    ROS_INFO("Land command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending land command");
    return false;
  }

  return true;
}

void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.x = deltaLat * deg2rad * C_EARTH;     //deg2rad : pi/180   
  deltaNed.y = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude + 100.09;
}


void
setHotpointInitDefault(dji_sdk::MissionHotpointTask& hotpointTask)
{
  hotpointTask.latitude      = gps_pos.latitude;
  hotpointTask.longitude     = gps_pos.longitude;
  hotpointTask.altitude      = 20;
  hotpointTask.radius        = 10;
  hotpointTask.angular_speed = 15;
  hotpointTask.is_clockwise  = 0;
  hotpointTask.start_point   = 0;
  hotpointTask.yaw_mode      = 0;
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
initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask)
{
  dji_sdk::MissionHpUpload missionHpUpload;
  missionHpUpload.request.hotpoint_task = hotpointTask;
  hotpoint_upload_service.call(missionHpUpload);
  return ServiceAck(
    missionHpUpload.response.result, missionHpUpload.response.cmd_set,
    missionHpUpload.response.cmd_id, missionHpUpload.response.ack_data);
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
    case DJI::OSDK::HOTPOINT:
      missionHpAction.request.action = action;
      hotpoint_action_service.call(missionHpAction);
      if (!missionHpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set,
                 missionHpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
      }
      return ServiceAck(
        missionHpAction.response.result, missionHpAction.response.cmd_set,
        missionHpAction.response.cmd_id, missionHpAction.response.ack_data);
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

ServiceAck
hotpointUpdateRadius(float radius)
{
  dji_sdk::MissionHpUpdateRadius missionHpUpdateRadius;
  missionHpUpdateRadius.request.radius = radius;
  hotpoint_updateRadius_service.call(missionHpUpdateRadius);
  if (!missionHpUpdateRadius.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i",
             missionHpUpdateRadius.response.cmd_set,
             missionHpUpdateRadius.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateRadius.response.ack_data);
  }
  return ServiceAck(missionHpUpdateRadius.response.result,
                    missionHpUpdateRadius.response.cmd_set,
                    missionHpUpdateRadius.response.cmd_id,
                    missionHpUpdateRadius.response.ack_data);
}

ServiceAck
hotpointUpdateYawRate(float yawRate, int direction)
{
  dji_sdk::MissionHpUpdateYawRate missionHpUpdateYawRate;
  missionHpUpdateYawRate.request.yaw_rate  = yawRate;
  missionHpUpdateYawRate.request.direction = direction;
  hotpoint_update_yawRate_Service.call(missionHpUpdateYawRate);
  if (!missionHpUpdateYawRate.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i",
             missionHpUpdateYawRate.response.cmd_set,
             missionHpUpdateYawRate.response.cmd_id);
    ROS_WARN("ack.data: %i", missionHpUpdateYawRate.response.ack_data);
  }
  return ServiceAck(missionHpUpdateYawRate.response.result,
                    missionHpUpdateYawRate.response.cmd_set,
                    missionHpUpdateYawRate.response.cmd_id,
                    missionHpUpdateYawRate.response.ack_data);
}