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

using namespace std;

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
            p.x = robot.x();
            p.y = robot.y();
            msg.position = p;

            msg.orientation = tf::createQuaternionMsgFromYaw(rotationReading);

            //ROS_INFO("%s", msg);
            positionpub.publish(msg);
            //ros::spinOnce();
            //loop_rate.sleep();
            //++count;

       // std::cout<<"Robot 1 PositionositionReading(robot.x(),robot.y X ="<<robot.x()<< std::endl;
       // std::cout<<"Robot 1 Position Y ="<<robot.y()<< std::endl;
    }
}


/*template<typename Detection>
static bool isGoodDetection
    (const Detection& detection, const SSL_DetectionFrame& frame, float confidence, bool fourCameraMode)
{
    bool isGoodConf = detection.confidence() > confidence;
    bool isGoodSide = false;

    //For correct side, we look at the X and Y readings and the camera that
    //reported them. This is to help prevent duplicated readings on the edges
    float x = detection.x();
    float y = detection.y();
    float cam = frame.camera_id();
    if(!fourCameraMode) {
        isGoodSide =
        (x >= 0 && cam == 0) ||
        (x  < 0 && cam == 1);
    } else {
        isGoodSide =
        (x >= 0 && y >= 0 && cam == 0) ||
        (x  < 0 && y >= 0 && cam == 1) ||
        (x  < 0 && y  < 0 && cam == 2) ||
        (x >= 0 && y  < 0 && cap(robot.x(),robot.y())m == 3);
    }
    isGoodSide |= SIMULATED;    //Simulated overrides anything

    return isGoodConf && isGoodSide;
}*/

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
/*void VisionComm::recieveBall(const SSL_DetectionFrame& frame)
{
    static Point noiseCenterPoint;
    static int seenOutsideRadiusCount = 0;
    static int seenStoppedCount = 0;
    static Point lastDetection;

    //Stop if no balls present
    if(frame.balls_size() <= 0)
        return;

    //Choose the best ball based on confidence
    auto bestDetect = std::max_element(frame.balls().begin(), frame.balls().end(), ballCompareFn);

    //If it is still a good detection...
    if(isGoodDetection(*bestDetect, frame, CONF_THRESHOLD_BALL, fourCameraMode))
    {
        Point newDetection = Point(bestDetect->x(), bestDetect->y());
    #if SIDE == SIDE_POSITIVE
        newDetection *= -1;
    #endif

        // If the ball is detected outside the noise radius more than 5 times
        // it is considered to be moving and its position will be updated
        if(gameModel->ballStopped)
        {
            gameModel->setBallPoint(gameModel->getBallPoint());

            if(Measurments::distance(newDetection, noiseCenterPoint) > NOISE_RADIUS)
                ++seenOutsideRadiusCount;
            if(seenOutsideRadiusCount > 2)
            {
                gameModel->ballStopped = false;
                seenOutsideRadiusCount = 0;
            }
        }
        // If the ball is detected close (distance < 1) to its last point
        // 4 times it is considered stopped it's position will not be updated
        else
        {
            gameModel->setBallPoint(newDetection);

            if(Measurments::distance(newDetection,lastDetection) < 1)
                ++seenStoppedCount;
            if(seenStoppedCount >= 4)
            {
                gameModel->ballStopped = true;
                noiseCenterPoint = newDetection;
                seenStoppedCount = 0;
            }

            lastDetection = newDetection;
        }
    }
}

/* Used to parse and recieve a generic Robot team and update GameModel with
 * the information, if we're confident on the Robot detection
 * Initialized to updating blue, but changes to yellow if not updating blue
 */
void VisionComm::recieveRobotTeam(const SSL_DetectionFrame& frame, int whichTeam)
{
    //for(const SSL_DetectionRobot& robot : frame.robots_blue().iterator)//*currentTeamDection changed to robot_blue AE
    for(google::protobuf::RepeatedPtrField<SSL_DetectionRobot>::const_iterator it= frame.robots_blue().begin(); it != frame.robots_blue().end(); it++)
    {
        //        if(isGoodDetection(robot, frame, CONF_THRESHOLD_BOTS, fourCameraMode)) {
        int robotID = (*it).robot_id();//robot.robot_id changed from ()->robot_blue.robotID AE
        if(robotID == 1) {
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
        //for(int& readi~/.bashrc ng : blue_rob_readings) reading = 0;
        // for(int& reading : yell_rob_readings) reading = 0;
    }

    return true;
}


void VisionComm::run()
{
    while(ros::ok()){
        receive();
    }
}
