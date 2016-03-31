#include "ros/ros.h"
#include "visioncomm.h"
#include <sstream>
#include "geometry_msgs/Pose.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;
    ros::Publisher positionpub = n.advertise<geometry_msgs::Pose>("Position", 1000);
   VisionComm visionCommunicator(positionpub);


   //GuiInterface::getGuiInterface()->show();

     /*ros::init(argc, argv, "position");

     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
     ros::Rate loop_rate(10);

     int count = 0;
     while (ros::ok())
     {
         std_msgs::String msg;
         std::stringstream ss;
         ss << "Robot 1 Position X ="<<robot.x()<< std::endl<< count;
         msg.data = ss.str();
         ROS_INFO("%s", msg.data.c_str());
         chatter_pub.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         ++count;
     }*/

   visionCommunicator.run();



    return 0;
}
