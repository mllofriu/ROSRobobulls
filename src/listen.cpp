/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <math.h>
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"
#include "config/team.h"
#include "config/communication.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iomanip>
#include <string>
#include "visioncomm.h"
//#include"omni4_velcalculator.h"

using std::abs;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void chatterCallback(const geometry_msgs::Twist cmd_vel)
{
    ROS_INFO("Received values: [%f, %f, %f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    double LF_offset = 144*M_PI/180; //135 Robot's x-Axis (right side) is zero
    double LB_offset = 224*M_PI/180; //225
    double RF_offset =  36*M_PI/180; //45
    double RB_offset = 316*M_PI/180; //315


#if SIMULATED
    double trans_offset = 0;
#else
    double trans_offset = 0.0149;
#endif
    const double wheel_radius = 27;
    double theta_vel = cmd_vel.angular.z/*theta_prop_mult * angle_error + theta_int_mult  * angle_error_integral*/;
    // Compute LF RF LB RB
    double y_vel_robot = cmd_vel.linear.x/*cos(theta_current)*x_vel+sin(theta_current)*y_vel*/;
    double x_vel_robot = cmd_vel.linear.y/*sin(theta_current)*x_vel-cos(theta_current)*y_vel*/;
    double vel_robot = sqrt(x_vel_robot*x_vel_robot + y_vel_robot * y_vel_robot);

    double RF =  (-sin(RF_offset) * x_vel_robot + cos(RF_offset)*y_vel_robot - trans_offset*vel_robot*cos(RF_offset) + wheel_radius*theta_vel);
    double LF = -(-sin(LF_offset) * x_vel_robot + cos(LF_offset)*y_vel_robot - trans_offset*vel_robot*cos(LF_offset) + wheel_radius*theta_vel);
    double LB = -(-sin(LB_offset) * x_vel_robot + cos(LB_offset)*y_vel_robot - trans_offset*vel_robot*cos(LB_offset) + wheel_radius*theta_vel);
    double RB =  (-sin(RB_offset) * x_vel_robot + cos(RB_offset)*y_vel_robot - trans_offset*vel_robot*cos(RB_offset) + wheel_radius*theta_vel);

    unsigned int max_mtr_spd = 100;
    if (abs(LF)>max_mtr_spd)
    {
        LB=(max_mtr_spd/abs(LF))*LB;
        RF=(max_mtr_spd/abs(LF))*RF;
        RB=(max_mtr_spd/abs(LF))*RB;
        LF=(max_mtr_spd/abs(LF))*LF;
    }
    if (abs(LB)>max_mtr_spd)
    {
        LF=(max_mtr_spd/abs(LB))*LF;
        RF=(max_mtr_spd/abs(LB))*RF;
        RB=(max_mtr_spd/abs(LB))*RB;
        LB=(max_mtr_spd/abs(LB))*LB;
    }
    if (abs(RF)>max_mtr_spd)
    {
        LF=(max_mtr_spd/abs(RF))*LF;
        LB=(max_mtr_spd/abs(RF))*LB;
        RB=(max_mtr_spd/abs(RF))*RB;
        RF=(max_mtr_spd/abs(RF))*RF;
    }
    if (abs(RB)>max_mtr_spd)
    {
        LF=(max_mtr_spd/abs(RB))*LF;
        LB=(max_mtr_spd/abs(RB))*LB;
        RF=(max_mtr_spd/abs(RB))*RF;
        RB=(max_mtr_spd/abs(RB))*RB;
    }

    //Create and return result container
   /* fourWheelVels vels;
    vels.LB = LB;
    vels.LF = LF;
    vels.RB = RB;
    vels.RF = RF;

    return vels;*/



    // Send Message
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow( (TEAM != TEAM_YELLOW) );
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    //Retrive robot information
   /* int id = 1;
    LF = robot->getLF();
    RF = robot->getRF();
    LB = robot->getLB();
    RB = robot->getRB();
    float kick = 0;
    bool  dribble = false;*/

    // Fill in simulator packet
    command->set_id(1);
    command->set_wheelsspeed(true);
    command->set_wheel1(-LF);    //Left Forward
    command->set_wheel2(-LB);    //Left Backward
    command->set_wheel3( RB);    //Right Backward
    command->set_wheel4( RF);    //Right Forward
    command->set_veltangent(0);
    command->set_velnormal(0);  // No normal velocity
    command->set_velangular(0);
    //command->set_kickspeedx(kick);
    //command->set_kickspeedz(0); // No chipper
    //command->set_spinner(dribble ? 80 : 0);

    //Send packet
    struct sockaddr_in myaddr;
    int sock;

    memset(&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family=AF_INET;
    myaddr.sin_addr.s_addr=htonl(INADDR_ANY);
    myaddr.sin_port=htons(0);

    if((sock=socket(AF_INET, SOCK_DGRAM, 0))<0) {
          perror("Failed to create socket");
          exit(EXIT_FAILURE);
       }

    if(bind(sock,( struct sockaddr *) &myaddr, sizeof(myaddr))<0) {
          perror("bind failed");
          exit(EXIT_FAILURE);
       }
    inet_pton(AF_INET, "131.247.14.100",&myaddr.sin_addr.s_addr);
    myaddr.sin_port=htons(20011);

    // Build udp packet from grSim packet
    char buf[packet.ByteSize()];
    packet.SerializeToArray(buf, packet.ByteSize());

    if(sendto(sock, buf, packet.ByteSize(), 0, (struct sockaddr *)&myaddr, sizeof(myaddr))!=packet.ByteSize()) {
          perror("Mismatch in number of bytes sent");
          exit(EXIT_FAILURE);
       }
    /*QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    udpsocket.writeDatagram(dgram, _addr, _port);

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    int s = socket(dgram,_addr,_port);*/
}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "listen");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);
  //geometry_msgs::Twist t;
  //geometry_msgs::Vector3 v;




  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
