#include "ros/ros.h"
#include "visioncomm.h"

#include "geometry_msgs/PoseStamped.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "position");
	ros::NodeHandle n;

	int robot;
	n.param("robot", robot, 1);

	ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("position",2);

	VisionComm visionCommunicator(&pub);

	visionCommunicator.run();

	return 0;
}
