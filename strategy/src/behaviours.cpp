#include <ros/ros.h>
#include "behaviours/bhv_dash.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "behaviours");
    ros::NodeHandle node;

    DashServer task("Dash");

    ros::spin();

    return 0;
}
