/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, University of York Robotics Laboratory (YRL).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file prgp_ardrone.h
 *  @brief The head file for prgp_ardrone package.
 *  @details The prgp_ardrone package and its structure and initial comments are created and tested by Chengqing Liu
 *  @version 1.0
 *  @author  , , , Chengqing Liu
 *  @date 24 July 2015
 *  @copyright BSD License.
 */

#ifndef PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_
#define PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Duration.h"

#include "ardrone_autonomy/Navdata.h"
#include "tum_ardrone/filter_state.h"

///sy   taking picture headers ########################
#include <cvd/image_io.h>
#include <cvd/videodisplay.h>
#include <cvd/gl_helpers.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

#include <unistd.h>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <algorithm>
#include <pthread.h>

#define DESIRED_HEIGHT 0.8
//Rob temp
#define EXTRA_HEIGHT 0.4
/** The main class for the prgp_ardrone package.
 */
class PRGPARDrone
{
private:
  ros::NodeHandle ndh_; /**< ROS node handle */
  ros::Duration ndPause; /**< The duration to pause the node */
  ros::Time pre_time; /**< The ROS time */

  //Publishers
  ros::Publisher landPub; /**< Publisher for sending the landing command directly to AR.Drone*/
  ros::Publisher takeoffPub; /**< Publisher for sending the takeoff command directly to AR.Drone */
  ros::Publisher drone_pub; /**< Publisher for sending flight command to AR.Drone by /tum_ardrone/com */
  ros::Publisher cmdPub; /**< Publisher for sending the command to Pi-Swarm by piswarm_com */
  ros::Publisher velPub; /**< Publisher for sending the command directly to AR.Drone by cmd_vel */

  //Subscribers
  ros::Subscriber cmdSub; /**< Subscriber to get the command from Pi-Swarm by piswarm_com */
  ros::Subscriber tagSub; /**< Subscriber to get the navdata, especially the tag result by /ardrone/navdata */
  ros::Subscriber currentStateSub; /**< Subscriber to get the current state of the AR.Drone */
  ros::Subscriber imgSub; /**< Subscriber to get the image from camera by /ardrone/image_raw */
  ros::Subscriber image_cmd_sub; ///
  ros::Publisher image_cmd_pub; ///
  ros::Subscriber cmd_completed_sub;

  //Rob#
  ros::Subscriber cmdCompleteSub;

  // services
  ros::ServiceClient toggleCamSrv; /**< Service client to send empty service to toggle the camera */
  ros::ServiceClient detecttypeSrv; /**< Service client to send empty service to change the detection configuration */
  ros::ServiceClient stopCmdAndHoverSrv; /**< Service client to send empty service to stop the current flight command and then hovering */

  //messages
  //std_msgs::String s; /**< Message for sending flight command to AR.Drone by /tum_ardrone/com*/
  //std::string c;

//  std_msgs::String s_Pi; /**< Message for sending command to Pi-Swarm by piswarm_com*/
//  std::string c_Pi;

  std_srvs::Empty toggle_srvs; /**< Service from client to server to toggle the camera */
  std_srvs::Empty detect_srvs; /**< Service from client to server to change the detection configuration */
  std_srvs::Empty stopCmd_srvs; /**< Service from client to server to stop the current flight command */
  geometry_msgs::Twist velCmd; /**< Message for the cmd_vel topic to AR.Drone */

  static pthread_mutex_t send_CS;
  static pthread_mutex_t pic_mt;

  //variables
  double currentPos_x;
  double currentPos_y;
  double currentPos_z;
  bool start_flag; /**< The value will be true when AR.Drone get the recruiting command from Pi-Swarm */
  bool initialising_PTAM_flag; //Rob# /**The value will be true whilst drone is taking off and performing initial movements to configure PTAM and locate home tag */
  bool aligning_to_home_tag; //Rob# /**The value will be true until the drone has centered above home tag and set new reference */
  bool detected_flag; /**< The value will be true when AR.Drone detect the target tag during the flight */
  bool detected_flag_h; /**< The value will be true when AR.Drone detect the target tag from the front camera */
  bool centering_flag; /**< The value will be true before AR.Drone centering the tag */
  bool picture_flag; /**< The value will be true before taking the picture */ //Rob# this name is not too clear. before_picture_flag would be better
  bool return_flag; /**< The value will be true before returning the Pi-Swarm and the Drone back home*/ //Rob# this name is not too clear. before_return_flag would be better
  bool init_tag_det; /**< The value will be true to activate the tag detection at the initial stage of AR.Drone */ //Rob# Is this the target tag? Maybe use target_tag or ttag distinguish
  bool init_detected_flag; /**< The value will be true when tag detected at the initial stage */
  bool home_tag_det; /**< The value will be true to activate the tag detection at the home stage of AR.Drone */
  bool home_detected_flag; /**< The value will be true when tag detected at home stage */
  bool executing_command_flag; //Rob# /**< this value will be true whilst the drone is executing a tum command, it will be set to false after a callback */
  bool cmd_completed_flag;
  bool home;
  bool image_saved;
  CVD::VideoDisplay * window = NULL;
  const static uint32_t tag_v = 65536;
  const static uint32_t tag_h = 0;
  uint8_t lost_count;
  uint16_t current_tag; /**< change when setTargetTag() is used, 0 for black_roundel, 1 for COCARDE */
  uint16_t target_tag; /**< 0 for black_roundel, 1 for COCARDE, 2 for mixed tag type (current_tag is 0) */
  bool reference_set;
  //Rob# **liu, Initialise the private variable here will be show warning.
  float altitude;
  uint32_t tag_x_coord;
  uint32_t tag_y_coord;
  float tag_orient;

  uint32_t tag_x_coord_h;
  uint32_t tag_y_coord_h;
  float tag_orient_h;

  float offset_x;
  float offset_y;
  float offset_z;

  typedef struct drone_pose
  {
    double x;
    double y;
    double z;
    double yaw;
  } pose_type;

public:
  PRGPARDrone(void);
  ~PRGPARDrone(void);

  void run();

  // callbacks
  void piswarmCmdRevCb(const std_msgs::StringConstPtr str);
  void takePicCb(const sensor_msgs::ImageConstPtr img);
  void acquireTagResultCb(const ardrone_autonomy::Navdata &navdataReceived);
  void acquireCurrentStateCb(const tum_ardrone::filter_state &currentState);
  void cmdFeedbackCb(const std_msgs::Empty msg);

  //functions
  void sendCmdToPiswarm();
  void sendVelCmd();
  void takeOff();
  void land();
  void sendFlightCmd(std::string c);
  void sendInitCmd(std::string c);
  void toggleCam();
  void setTargetTag();
  bool initARDrone();
  bool smallRangeSearch();
  bool centeringTag(double current_height);
  bool searchForTargetTag();
  void flightToTarget();
  void flightToHome();
  void moveToPose(double x, double y, double z, double yaw);
  void stopCmdAndHover();
  void moveBy(double x, double y, double z, double yaw);
  double yawFromPos(double x_coord, double y_coord);
};

#endif /* PRGP_ARDRONE_INCLUDE_PRGP_ARDRONE_H_ */
