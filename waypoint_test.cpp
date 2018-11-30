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

using namespace DJI::OSDK;

// global variables
ros::ServiceClient     waypoint_upload_service;
ros::ServiceClient     waypoint_action_service;
ros::ServiceClient     hotpoint_update_yawRate_Service;
ros::ServiceClient     hotpoint_updateRadius_service;
ros::ServiceClient     drone_activation_service;
ros::ServiceClient     sdk_ctrl_authority_service;
ros::ServiceClient     drone_task_service;
sensor_msgs::NavSatFix current_gps;
ros::Subscriber        current_gps_subscriber;

void
gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;                                                  //********************************************gps 
}

//*************************************************************************************************************

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


/*ServiceAck
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
*/
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



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint");
  ros::NodeHandle nh;

  // ROS stuff
  waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
    "dji_sdk/mission_waypoint_upload");
  waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
    "dji_sdk/mission_waypoint_action");
  drone_activation_service =
    nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>(
    "dji_sdk/sdk_control_authority");
  drone_task_service =
    nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  current_gps_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
    "dji_sdk/current_gpsition", 10, &gpsPosCallback);

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
  ServiceAck ack = obtainCtrlAuthority();                 //**********************************
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

  takeoff();
  ros::spinOnce();

  dji_sdk::MissionWaypoint waypoint1;
  initwaypoint(waypoint1);
  dji_sdk::MissionWaypoint waypoint2;
  initwaypoint(waypoint2);
  waypoint1.latitude  = current_gps.latitude;                                    //******************
  waypoint1.longitude = current_gps.longitude;
  waypoint1.altitude  = 10;
  waypoint2.latitude  = current_gps.latitude+0.001;                                   
  waypoint2.longitude = current_gps.longitude+0.001;
  waypoint2.altitude  = 10;

  runwaypoint(waypoint1);//,waypoint2);

  land();
  return 0;
}
