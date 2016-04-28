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

    visionCommunicator.run();



    return 0;
}
