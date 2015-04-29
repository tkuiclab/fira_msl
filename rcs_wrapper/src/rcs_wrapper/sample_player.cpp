#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "rcs_wrapper/sample_player.h"

//#include "strategy.h"
//#include "field_analyzer.h"
//
//#include "action_chain_holder.h"
//#include "sample_field_evaluator.h"
//
//#include "soccer_role.h"
//
//#include "sample_communication.h"
//#include "keepaway_communication.h"
//
//#include "bhv_penalty_kick.h"
//#include "bhv_set_play.h"
//#include "bhv_set_play_kick_in.h"
//#include "bhv_set_play_indirect_free_kick.h"
//
//#include "bhv_custom_before_kick_off.h"
//#include "bhv_strict_check_shoot.h"
//
//#include "view_tactical.h"
//
//#include "intention_receive.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/bhv_emergency.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_intercept.h>
#include <rcsc/action/body_kick_one_step.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/view_synch.h>

#include <rcsc/formation/formation.h>
#include <rcsc/action/kick_table.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/say_message_builder.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/freeform_parser.h>

#include <rcsc/common/basic_client.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/player_param.h>
#include <rcsc/common/audio_memory.h>
#include <rcsc/common/say_message_parser.h>
// #include <rcsc/common/free_message_parser.h>

#include <rcsc/param/param_map.h>
#include <rcsc/param/cmd_line_parser.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <cmath>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::SamplePlayer(ros::NodeHandle *nh)
    : PlayerAgent(),
    nh(nh)
//      M_communication(),
//      M_field_evaluator( createFieldEvaluator() ),
//      M_action_generator( createActionGenerator() )
{
    raw_srv_msg_pub = nh->advertise<std_msgs::String>("/rcs/raw_recvice_msg", 1000);
    self_pose_pub = nh->advertise<geometry_msgs::Pose2D>("/rcs/self_pose", 1000);
    ball_pos_pub = nh->advertise<geometry_msgs::Pose2D>("/rcs/ball_pos", 1000);

    cmd_vel_sub = nh->subscribe<geometry_msgs::Twist>("/rcs/cmd_vel", 10, &SamplePlayer::velCallback, this);

    ang_vel = 0.0;
    dir = 0.0;
    vel = 0.0;
}

/*-------------------------------------------------------------------*/
/*!

 */
SamplePlayer::~SamplePlayer()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
bool
SamplePlayer::initImpl( CmdLineParser & cmd_parser )
{
    std::list<std::string> args;
    args.push_back("--use_fullstate");
    args.push_back("true");
    args.push_back("--debug_fullstate");
    args.push_back("true");
    CmdLineParser cmd_parser_fullstate(args);
    bool result = PlayerAgent::initImpl( cmd_parser_fullstate );

    if ( ! result )
    {
        return false;
    }

    return true;
}

/*-------------------------------------------------------------------*/
/*!

 */
void SamplePlayer::velCallback(const geometry_msgs::Twist::ConstPtr &twist)
{
    const geometry_msgs::Vector3 &linear = twist->linear;

    ang_vel = twist->angular.z;
    dir = atan(linear.y/linear.x);
    vel = sqrt(linear.x*linear.x + linear.y*linear.y);
}

/*-------------------------------------------------------------------*/
/*!
  main decision
  virtual method in super class
*/
void
SamplePlayer::actionImpl()
{
    if ( fullstateWorld().gameMode().type() == GameMode::BeforeKickOff
         || fullstateWorld().gameMode().type() == GameMode::AfterGoal_ )
    {
        doMove(-10.0, 0.0);
    }
    else
    {
        if(vel > 0) doDash(vel, dir);
        if(ang_vel > 0) doTurn(ang_vel);
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionStart()
{
    const WorldModel &wm = this->fullstateWorld();

    raw_srv_msg.data = M_client->message();
    raw_srv_msg_pub.publish(raw_srv_msg);

    self_pose.x = wm.self().pos().x;
    self_pose.y = wm.self().pos().y;
    self_pose.theta = wm.self().pos().th().radian();
    self_pose_pub.publish(self_pose);

    ball_pos.x = wm.ball().pos().absX();
    ball_pos.y = wm.ball().pos().absY();
    ball_pos_pub.publish(ball_pos);

    ros::spinOnce();
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleActionEnd()
{
    if ( fullstateWorld().self().posValid() )
    {
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handleServerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerParam()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
void
SamplePlayer::handlePlayerType()
{

}

/*-------------------------------------------------------------------*/
/*!
  communication decision.
  virtual method in super class
*/
void
SamplePlayer::communicationImpl()
{
}

/*-------------------------------------------------------------------*/
/*!
*/
bool
SamplePlayer::doPreprocess()
{
//    // check tackle expires
//    // check self position accuracy
//    // ball search
//    // check queued intention
//    // check simultaneous kick
//
//    const WorldModel & wm = this->world();
//
//    dlog.addText( Logger::TEAM,
//                  __FILE__": (doPreProcess)" );
//
//    //
//    // freezed by tackle effect
//    //
//    if ( wm.self().isFrozen() )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": tackle wait. expires= %d",
//                      wm.self().tackleExpires() );
//        // face neck to ball
//        this->setViewAction( new View_Tactical() );
//        this->setNeckAction( new Neck_TurnToBallOrScan() );
//        return true;
//    }
//
//    //
//    // BeforeKickOff or AfterGoal. jump to the initial position
//    //
//    if ( wm.gameMode().type() == GameMode::BeforeKickOff
//         || wm.gameMode().type() == GameMode::AfterGoal_ )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": before_kick_off" );
//        Vector2D move_point =  Strategy::i().getPosition( wm.self().unum() );
//        Bhv_CustomBeforeKickOff( move_point ).execute( this );
//        this->setViewAction( new View_Tactical() );
//        return true;
//    }
//
//    //
//    // self localization error
//    //
//    if ( ! wm.self().posValid() )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": invalid my pos" );
//        Bhv_Emergency().execute( this ); // includes change view
//        return true;
//    }
//
//    //
//    // ball localization error
//    //
//    const int count_thr = ( wm.self().goalie()
//                            ? 10
//                            : 5 );
//    if ( wm.ball().posCount() > count_thr
//         || ( wm.gameMode().type() != GameMode::PlayOn
//              && wm.ball().seenPosCount() > count_thr + 10 ) )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": search ball" );
//        this->setViewAction( new View_Tactical() );
//        Bhv_NeckBodyToBall().execute( this );
//        return true;
//    }
//
    //
    // set default change view
    //

//    this->setViewAction( new View_Tactical() );
//
//    //
//    // check shoot chance
//    //
//    if ( doShoot() )
//    {
//        return true;
//    }
//
//    //
//    // check queued action
//    //
//    if ( this->doIntention() )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": do queued intention" );
//        return true;
//    }
//
//    //
//    // check simultaneous kick
//    //
//    if ( doForceKick() )
//    {
//        return true;
//    }
//
//    //
//    // check pass message
//    //
//    if ( doHeardPassReceive() )
//    {
//        return true;
//    }
//
    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doShoot()
{
//    const WorldModel & wm = this->world();
//
//    if ( wm.gameMode().type() != GameMode::IndFreeKick_
//         && wm.time().stopped() == 0
//         && wm.self().isKickable()
//         && Bhv_StrictCheckShoot().execute( this ) )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": shooted" );
//
//        // reset intention
//        this->setIntention( static_cast< SoccerIntention * >( 0 ) );
//        return true;
//    }
//
    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doForceKick()
{
//    const WorldModel & wm = this->world();
//
//    if ( wm.gameMode().type() == GameMode::PlayOn
//         && ! wm.self().goalie()
//         && wm.self().isKickable()
//         && wm.existKickableOpponent() )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": simultaneous kick" );
//        this->debugClient().addMessage( "SimultaneousKick" );
//        Vector2D goal_pos( ServerParam::i().pitchHalfLength(), 0.0 );
//
//        if ( wm.self().pos().x > 36.0
//             && wm.self().pos().absY() > 10.0 )
//        {
//            goal_pos.x = 45.0;
//            dlog.addText( Logger::TEAM,
//                          __FILE__": simultaneous kick cross type" );
//        }
//        Body_KickOneStep( goal_pos,
//                          ServerParam::i().ballSpeedMax()
//                          ).execute( this );
//        this->setNeckAction( new Neck_ScanField() );
//        return true;
//    }
//
    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
SamplePlayer::doHeardPassReceive()
{
//    const WorldModel & wm = this->world();
//
//    if ( wm.audioMemory().passTime() != wm.time()
//         || wm.audioMemory().pass().empty()
//         || wm.audioMemory().pass().front().receiver_ != wm.self().unum() )
//    {
//
//        return false;
//    }
//
//    int self_min = wm.interceptTable()->selfReachCycle();
//    Vector2D intercept_pos = wm.ball().inertiaPoint( self_min );
//    Vector2D heard_pos = wm.audioMemory().pass().front().receive_pos_;
//
//    dlog.addText( Logger::TEAM,
//                  __FILE__":  (doHeardPassReceive) heard_pos(%.2f %.2f) intercept_pos(%.2f %.2f)",
//                  heard_pos.x, heard_pos.y,
//                  intercept_pos.x, intercept_pos.y );
//
//    if ( ! wm.existKickableTeammate()
//         && wm.ball().posCount() <= 1
//         && wm.ball().velCount() <= 1
//         && self_min < 20
//         //&& intercept_pos.dist( heard_pos ) < 3.0 ) //5.0 )
//         )
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": (doHeardPassReceive) intercept cycle=%d. intercept",
//                      self_min );
//        this->debugClient().addMessage( "Comm:Receive:Intercept" );
//        Body_Intercept().execute( this );
//        this->setNeckAction( new Neck_TurnToBall() );
//    }
//    else
//    {
//        dlog.addText( Logger::TEAM,
//                      __FILE__": (doHeardPassReceive) intercept cycle=%d. go to receive point",
//                      self_min );
//        this->debugClient().setTarget( heard_pos );
//        this->debugClient().addMessage( "Comm:Receive:GoTo" );
//        Body_GoToPoint( heard_pos,
//                    0.5,
//                        ServerParam::i().maxDashPower()
//                        ).execute( this );
//        this->setNeckAction( new Neck_TurnToBall() );
//    }
//
//    this->setIntention( new IntentionReceive( heard_pos,
//                                              ServerParam::i().maxDashPower(),
//                                              0.9,
//                                              5,
//                                              wm.time() ) );
//
    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
//FieldEvaluator::ConstPtr
//SamplePlayer::getFieldEvaluator() const
//{
//    return M_field_evaluator;
//}

/*-------------------------------------------------------------------*/
/*!

*/
//FieldEvaluator::ConstPtr
//SamplePlayer::createFieldEvaluator() const
//{
//    return FieldEvaluator::ConstPtr( new SampleFieldEvaluator );
//}


/*-------------------------------------------------------------------*/
/*!
*/
//#include "actgen_cross.h"
//#include "actgen_direct_pass.h"
//#include "actgen_self_pass.h"
//#include "actgen_strict_check_pass.h"
//#include "actgen_short_dribble.h"
//#include "actgen_simple_dribble.h"
//#include "actgen_shoot.h"
//#include "actgen_action_chain_length_filter.h"

//ActionGenerator::ConstPtr
//SamplePlayer::createActionGenerator() const
//{
//    CompositeActionGenerator * g = new CompositeActionGenerator();
//
//    //
//    // shoot
//    //
//    g->addGenerator( new ActGen_RangeActionChainLengthFilter
//                     ( new ActGen_Shoot(),
//                       2, ActGen_RangeActionChainLengthFilter::MAX ) );
//
//    //
//    // strict check pass
//    //
//    g->addGenerator( new ActGen_MaxActionChainLengthFilter
//                     ( new ActGen_StrictCheckPass(), 1 ) );
//
//    //
//    // cross
//    //
//    g->addGenerator( new ActGen_MaxActionChainLengthFilter
//                     ( new ActGen_Cross(), 1 ) );
//
//    //
//    // direct pass
//    //
//    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
//    //                  ( new ActGen_DirectPass(),
//    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );
//
//    //
//    // short dribble
//    //
//    g->addGenerator( new ActGen_MaxActionChainLengthFilter
//                     ( new ActGen_ShortDribble(), 1 ) );
//
//    //
//    // self pass (long dribble)
//    //
//    g->addGenerator( new ActGen_MaxActionChainLengthFilter
//                     ( new ActGen_SelfPass(), 1 ) );
//
//    //
//    // simple dribble
//    //
//    // g->addGenerator( new ActGen_RangeActionChainLengthFilter
//    //                  ( new ActGen_SimpleDribble(),
//    //                    2, ActGen_RangeActionChainLengthFilter::MAX ) );
//
//    return ActionGenerator::ConstPtr( g );
//}
