/*
 *   image_class.hpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
#ifndef _IMAGE_CALSS_H_
#define _IMAGE_CALSS_H_

#include "particle_filter.hpp"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace Eigen;
using namespace cv;

class paintImageClass{
private:
    Mat image;
    bool *map;
    Mat *likelihood_map;
    int mapH;
    int mapW;
    int pNum;
    int sensorLineNum;
    double shift_x;
    double shift_y;
    double rotation;
    bool isRosInit;
    Vector2i rPosi;
    Vector2d rPos;

    char* imageName;


public:
    paintImageClass(int p_Num,int sensorlines,int map_H,int map_W);
    virtual ~paintImageClass(){};

    void readMap();
    void paint_particle(std::vector<Vector3d> pAry);
    void paint_sensorline(Vector3d robot_pos,std::vector<Vector2i> sensorWall_Pos);
    void paint_robot(Vector3d robot);
    void paint_particle_sensorwall(std::vector<Vector3d> pAry,std::vector<Vector2i> tpos_wall);

    void refresh_window();
    void paintRobot_Particle(Vector3d robot, std::vector<Vector3d> pAry);
    void paintRobot_Sensorlines_Particle(Vector3d robot, std::vector<Vector3d> pAry,std::vector<Vector2i> sensorWall_Pos);

    //to verify particle sensor
    void paintRobot_Particle_PSensor(Vector3d robot, std::vector<Vector3d> pAry,std::vector<Vector2i> sensorWall_Pos,std::vector<Vector2i> tp_wall);
};
#endif
