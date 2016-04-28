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
#include "serialib.h"

using std::abs;

static float k = 0.4;
struct packet_t {
    int8_t tilde;         //!<"Tilde", '~ 'for NXT, 255 for Arduino
    int8_t id;            //!<Robot ID
    int8_t left_front;    //!<LF wheel velocity
    int8_t left_back;     //!<LB wheel velocity
    int8_t right_front;   //!<RF wheel velocity
    int8_t right_back;    //!<RB wheel velocity
    int8_t kick;          //!<Kick? 1/0 for Arduino, 'k'/0 for NXT
    int8_t chip_power;    //!<Chip kick power
    int8_t dribble_power; //!<Dribbler power
    int8_t dollar;        //!<Dollar", '$' for NXT, 255 for Arduino
};

serialib Xbee;            //!<seriallib Interface to /dev/xbee


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void sendToSimulator(double RF, double RB, double LF, double LB)
{
    // Send Message
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow( (TEAM != TEAM_YELLOW) );
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();

    //Retrive robot information
    int id = 3;

    // Fill in simulator packet
    command->set_id(id);
    command->set_wheelsspeed(true);
    command->set_wheel1(-LF);    //Left Forward
    command->set_wheel2(-LB);    //Left Backward
    command->set_wheel3( RB);    //Right Backward
    command->set_wheel4( RF);    //Right Forward
    command->set_veltangent(0);
    command->set_velnormal(0);  // No normal velocity
    command->set_velangular(0); // Not in use
    command->set_kickspeedx(0); // not in use
    command->set_kickspeedz(0); // No chipper
    command->set_spinner(0); //not in use

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
    //this if statement compiles but it can be modified further
    if(sendto(sock, buf, packet.ByteSize(), 0, (struct sockaddr *)&myaddr, sizeof(myaddr))!=packet.ByteSize()) {
        perror("Mismatch in number of bytes sent");
        exit(EXIT_FAILURE);// end of if statement
    }
}

void sendToRobot(double RF, double RB, double LF, double LB)
{
    // Create array of packets
    packet_t teamPacketBuf[1];

    // Initialize packet to zeros
    std::memset(&teamPacketBuf, 0, sizeof(packet_t));

    // Load information into the packet
    packet_t* packet = &teamPacketBuf[0];
    packet->id = 4;

    //Packet format with Arduino: 250 and 255 with k+100
    packet->tilde = char(250);
    packet->dollar = char(255);
    //Uses Min and Max values to set limits to robot
    packet->left_front  = std::min(100.0, std::max(LF, -100.0))*k + 100;
    packet->left_back   = std::min(100.0, std::max(LB, -100.0))*k + 100;
    packet->right_front = std::min(100.0, std::max(RF, -100.0))*k + 100;
    packet->right_back  = std::min(100.0, std::max(RB, -100.0))*k + 100;



    //Kick, dribble, and Chip power (no chipper, always 0)
    packet->kick = 0;//not in use
    packet->dribble_power = 0;// not in use
    packet->chip_power = 0;

    // Send Array of packets
    Xbee.Write((char*)&teamPacketBuf, sizeof(packet_t));
}

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
    double theta_vel = cmd_vel.angular.z;
    // Compute LF RF LB RB
    double y_vel_robot = cmd_vel.linear.x;
    double x_vel_robot = cmd_vel.linear.y;
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
    // If using Robot or simulator be sure to change the value from 1 or 0 . to specify which function to call
    int Real = 1;
    if(Real==0)
    {
        sendToSimulator(RF, RB, LF, LB);
    }
    else
    {
        sendToRobot(RF, RB, LF, LB);
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listen");
    ros::NodeHandle n;

    if (Xbee.Open("/dev/xbee", 57600) != 1) {
        printf ("Error while opening port. Permission problem ?\n");
        exit(1);
    } else {
        printf ("Serial port opened successfully !\n");
    }

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);

    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
    // %Tag(SPIN)%
    ros::spin();
    // %EndTag(SPIN)%
    sendToRobot(0, 0, 0, 0);
    sendToSimulator(0, 0, 0, 0);
    Xbee.Close();

    return 0;
}
// %EndTag(FULLTEXT)%
