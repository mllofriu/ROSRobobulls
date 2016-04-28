#include <cmath>
#include <sys/time.h>
#include "config/simulated.h"
#include "config/communication.h"
#include "visioncomm.h"
#include <iostream>
#include "utilities/point.h"
#include "config/team.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
//#include "std_msg/String.h"
#include <sstream>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"

using namespace std;
int robo;// Robot ID when using parameters

VisionComm::VisionComm(ros::Publisher pub)
{

    client = new RoboCupSSLClient(VISION_PORT, VISION_ADDRESS);
    client->open(true);
    fourCameraMode = isFourCameraMode();
    positionpub = pub;
}

VisionComm::~VisionComm(void)
{
    client->close();
    delete client;
}

bool VisionComm::isFourCameraMode()
{
    client->receive(packet);
    return (packet.geometry().calib_size() == 4);
}

/* This function processes a DetectionRobot from the vision system and fills
 * out the information in the GameModel
 */
void VisionComm::updateInfo(const SSL_DetectionRobot& robot, int detectedTeamColor)
{

    if (robot.has_robot_id())
    {
        int id = robot.robot_id();


        // Assumption: rob contains the robot with id == detected_id
        Point positionReading(robot.x(),robot.y());
        float rotationReading = robot.orientation();



#if SIDE == SIDE_POSITIVE
        positionReading *= -1;
        if(rotationReading > 0) {
            rotationReading = -(M_PI - rotationReading);
        } else {
            rotationReading = -(-M_PI - rotationReading);
        }
#endif

        //TODO: publish this x,y as a Pose msg in ROS - http://docs.ros.org/hydro/api/geometry_msgs/html/msg/Pose.html



        geometry_msgs::Pose msg;
        geometry_msgs::Point p;
        geometry_msgs::Twist v;
        p.x = robot.x();
        p.y = robot.y();

        msg.position = p;

        msg.orientation = tf::createQuaternionMsgFromYaw(rotationReading);

        //ROS_INFO("%s", msg);
        positionpub.publish(msg);

    }
}


//Predicate for max_element, determining which ball is best on confidence
static bool ballCompareFn(const SSL_DetectionBall&  a, const SSL_DetectionBall&  b) {
    return a.confidence() < b.confidence();
}

//Movement distance between detections within which the ball is said to be stationary
#define NOISE_RADIUS 15

/* Looks at all detected balls in the frame detection, and chooses
 * the best one based on confidence. Sets the GameModel's ballpoint
 * as this best one if that confidence is > CONF_THRESHOLD_BALL,
 * and it is detected on the correct side.
 */


/* Used to parse and recieve a generic Robot team and update GameModel with
 * the information, if we're confident on the Robot detection
 * Initialized to updating blue, but changes to yellow if not updating blue
 */
void VisionComm::recieveRobotTeam(const SSL_DetectionFrame& frame, int whichTeam)
{
    //for(const SSL_DetectionRobot& robot : frame.robots_blue().iterator)//*currentTeamDection changed to robot_blue AE
    for(google::protobuf::RepeatedPtrField<SSL_DetectionRobot>::const_iterator it= frame.robots_blue().begin(); it != frame.robots_blue().end(); it++)
    {
        ros::NodeHandle n("~");
        n.param("robotID", robo, 0);
        //        if(isGoodDetection(robot, frame, CONF_THRESHOLD_BOTS, fourCameraMode)) {
        int robotID = (*it).robot_id();//robot.robot_id changed from ()->robot_blue.robotID AE
        if(robotID == robo) {
            updateInfo(*it, whichTeam);
        }

        //        }
    }
}

void VisionComm::receiveIfMSPassed(int ms_limit)
{
    timeval now;
    gettimeofday(&now, NULL);
    float seconds = now.tv_sec - lastRecvTime.tv_sec;
    float useconds = now.tv_usec - lastRecvTime.tv_usec;
    float ms_passed = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    if(ms_passed > ms_limit) {
        client->receive(packet);    //Recieve packet here
        lastRecvTime = now;
    }
}

bool VisionComm::receive()
{
    //Receive a new packet if X ms has passed (0 FOR NOW)
    receiveIfMSPassed(0);

    if(packet.has_detection())
    {
        const SSL_DetectionFrame& frame = packet.detection();
        //recieveBall(frame);AE
        recieveRobotTeam(frame, TEAM_BLUE);
        //recieveRobotTeam(frame, TEAM_YELLOW);AE

        /* After we have had a chance to initially recieve all robots,
         * the RoboBulls game is run with the new information here. */

    }
    
    /* After 50 frames the "seen counts" of each team are set to 0. This prevents
     * ghost robots from appearing over time */
    if(++resetFrames > 50) {
        resetFrames = 0;
    }

    return true;
}


void VisionComm::run()
{
    while(ros::ok()){
        receive();
    }
}
