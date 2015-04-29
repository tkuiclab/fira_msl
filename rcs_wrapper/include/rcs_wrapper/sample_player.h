#ifndef SAMPLE_PLAYER_H
#define SAMPLE_PLAYER_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

//#include "action_generator.h"
//#include "field_evaluator.h"
//#include "communication.h"

#include <rcsc/player/player_agent.h>
#include <vector>

class SamplePlayer
    : public rcsc::PlayerAgent {
private:
    ros::NodeHandle *nh;

    ros::Publisher raw_srv_msg_pub;
    ros::Publisher self_pose_pub;
    ros::Publisher ball_pos_pub;

    ros::Subscriber cmd_vel_sub;

    std_msgs::String raw_srv_msg;
    geometry_msgs::Pose2D self_pose;
    geometry_msgs::Pose2D ball_pos;

    void velCallback(const geometry_msgs::Twist::ConstPtr &);

    //Communication::Ptr M_communication;

    //FieldEvaluator::ConstPtr M_field_evaluator;
    //ActionGenerator::ConstPtr M_action_generator;

public:

    SamplePlayer(ros::NodeHandle*);

    virtual
    ~SamplePlayer();

protected:

    /*!
      You can override this method.
      But you must call PlayerAgent::initImpl() in this method.
    */
    virtual
    bool initImpl( rcsc::CmdLineParser & cmd_parser );

    //! main decision
    virtual
    void actionImpl();

    //! communication decision
    virtual
    void communicationImpl();

    virtual
    void handleActionStart();
    virtual
    void handleActionEnd();

    virtual
    void handleServerParam();
    virtual
    void handlePlayerParam();
    virtual
    void handlePlayerType();

    //virtual
    //FieldEvaluator::ConstPtr createFieldEvaluator() const;

    //virtual
    //ActionGenerator::ConstPtr createActionGenerator() const;

private:

    bool doPreprocess();
    bool doShoot();
    bool doForceKick();
    bool doHeardPassReceive();

    double ang_vel;
    double dir;
    double vel;
public:
    //virtual
    //FieldEvaluator::ConstPtr getFieldEvaluator() const;
};

#endif
