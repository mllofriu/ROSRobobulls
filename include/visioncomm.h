#ifndef VISIONCOMM_H
#define VISIONCOMM_H
#include <string>
#include "netraw.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "robocup_ssl_client.h"
#include "ros/ros.h"
//#include "model/gamemodel.h"
//#include "model/robot.h"

//! @brief Sets the minimum confidence to consider a ball reading as valid
const float CONF_THchatter_pubRESHOLD_BALL = 0.75;

//! @brief Sets the minimum confidence to consider a robot as a valid reading
const float CONF_THRESHOLD_BOTS = 0.8;

/**
 * @brief The Visiochatter_pubnComm class recieves information from the vision cameras
 * @author Origin. Narges Ghaedi, JamesW
 * @details Detects the robots and ball and puts each robot in the corresponding team
 * based on robot's color (Blue team/ Yellow team)
 */
class VisionComm
{
public:
    VisionComm(ros::Publisher pub);
    ~VisionComm();

    //! @brief Recieves an SSL_WrapperPacket and fills in the GameModel information
    bool receive();
    
    //! @brief QThread run method
    void run();

protected:
    //! @brief Parses an SSL_DetectionFrame and fills out GameModel
    void recieveRobotTeam(const SSL_DetectionFrame& frame, int whichTeam);

    //! @brief Parses an SSL_DetectionFrame and fills out GameModel
    //void recieveBall(const SSL_DetectionFrame& frame);

    //! @brief Updates GameModel information to fill out a robot
    void updateInfo(const SSL_DetectionRobot& robot, int detectedTeamColor);

    //! @brief Recieves a new packet only if `ms_limit` ms has passed since last call
    void receiveIfMSPassed(int ms_limit);

    //! @brief Returns true if we are using four or two cameras.
    bool isFourCameraMode();

    //GameModel *gamemodel;           //! Pointer to GameModel to update
    SSL_WrapperPacket packet;       //! Packet recieved by client
    RoboCupSSLClient * client;      //! client to receive packets
    int resetFrames = 0;            //! Frames passed up remove all potential robot detections
    int totalframes = 0;            //! Total frames passed since start
    int blue_rob_readings[10]={0};  //! Number of detections of each blue robot
    int yell_rob_readings[10]={0};  //! Number of detections of each yelloe robot
    timeval lastRecvTime;           //! When did we last receive a packet? Used to not recieve every one
    bool fourCameraMode = false;    //! Are we in four-camera mode (true)? Or Two-camera mode?
    ros::Publisher positionpub;
};

#endif // VISIONCOMM_H


