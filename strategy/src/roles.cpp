#include <ros/ros.h>
#include "roles/role_goalie.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "roles");
    ros::NodeHandle node;

    GoalieServer task("goalie");

    ros::spin();

    return 0;
}
