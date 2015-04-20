#include "ros/ros.h"
#include "rcs_wrapper/sample_player.h"

#include <rcsc/common/basic_client.h>


/**
*/
int main(int argc, char **argv)
{
    /**
    */
    ros::init(argc, argv, "rcsc_wrapper");

    /**
    */
    ros::NodeHandle n;

    //ros::Rate loop_rate(10);

    SamplePlayer agent(&n);
    rcsc::BasicClient client;
    if(!agent.init(&client, argc, argv))
    {
        ROS_INFO("%s", "agent faild");
        return 0;
    }
    client.run(&agent);
    //if ( ! client.connectTo("localhost", 6000, 1000) )
    //{
    //    ROS_INFO("%s", "agent faild");
    //}

    //if(client.isServerAlive()) {
    //    ROS_INFO("%s", "Server is alive.");
    //}

    //std::string teamname( "TeamTKU" );
    //double version = 0.1;
    //bool goalie = true;
    //rcsc::PlayerInitCommand com( teamname, version, goalie );

    //std::ostringstream os;
    //com.toStr( os ); // "(init TeamName (version 9) (goalie))"を出力
    //client.sendMessage( os.str().c_str() );

    //while (ros::ok())
    //{
        //while(client.recvMessage() > 0) {
        //    std_msgs::String msg;
        //    msg.data = client.message();
        //    chatter_pub.publish(msg);
        //    parse(client.message());
        //}

        //ros::spinOnce();

        //loop_rate.sleep();
    //}

    return 0;
}
