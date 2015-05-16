/*
 *   image_class.cpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */

#include "image_class.hpp"

paintImageClass::paintImageClass(int p_Num, int sensorlines, int map_W, int map_H)
{
    sensorLineNum = sensorlines;
    pNum = p_Num;
    mapW = map_W;
    mapH = map_H;
}

void paintImageClass::readMap()
{
    int tIndex;
    imageName = "FIRA_map";
    image = imread( "/home/iclab/FIRA_ws/devel/lib/particle_filter/Ground.png", 1 );
    cv::Mat gray_img;
    cv::Mat binary_img;
    likelihood_map=new cv::Mat(cv::Size(image.cols,image.rows),CV_8UC1,Scalar(0));
    cv::cvtColor(image,gray_img,CV_RGB2GRAY);
    cv::threshold(gray_img,binary_img,50,255,CV_THRESH_BINARY);

    mapW = image.cols;
    mapH = image.rows;
    map = new bool[mapW*mapH];

    for(int i=0;i<binary_img.rows;i++)
    {
        for(int j=0;j<binary_img.cols;j++)
        {
            if(binary_img.data[(i*binary_img.cols)+j]==0)
            {
                tIndex = i*likelihood_map->cols+j;
                map[tIndex] = true;
            }
        }
    }
    //waitKey(60);
}

void paintImageClass::paint_robot(Vector3d robot)
{
    int rawLineSize = 8;
    int h_x = robot(0)+cos(robot(2))*rawLineSize;
    int h_y = robot(1)+sin(robot(2))*rawLineSize;
    Point center(robot(0),robot(1));
    Point head(h_x,h_y);
    circle(image,center,3,Scalar( 0, 0, 255 ),-1,8);
    line(image,center,head,Scalar( 0, 0, 255 ),1,8,0);
}

void paintImageClass::paint_particle(std::vector<Vector3d> pAry)
{
    for(int i=0;i<pAry.size();i++)
    {
        int rawLineSize = 8;
        int x = pAry[i](0);
        int y = pAry[i](1);
        int h_x = pAry[i](0)+cos(pAry[i](2))*rawLineSize;
        int h_y = pAry[i](1)+sin(pAry[i](2))*rawLineSize;
        Point center(x,y);
        Point head(h_x,h_y);
        CvScalar color = CV_RGB(255,119,0);
        //CvScalar color = CV_RGB(255,0,0);
        circle(image,center,3,color,-1,8);
        line(image,center,head,color,1,8,0);
    }
}

void paintImageClass::paint_sensorline(Vector3d robot_pos,std::vector<Vector2i> sensorWall_Pos)
{
    CvScalar Color=CV_RGB(0,0,255);
    CvPoint SensorWall_pos;
    CvPoint robot_pose = cvPoint(robot_pos(0),robot_pos(1));
    //std::cout << "size:" << sensorWall_Pos.size() << std::endl;

    for(int k=0;k<sensorWall_Pos.size();k++)
    {
        //std::cout << "x:" << sensorWall_Pos[i](0) << "\t" << "y:" << sensorWall_Pos[i](1) << std::endl;
        SensorWall_pos = cvPoint(sensorWall_Pos[k](0),sensorWall_Pos[k](1));
        line(image,robot_pose,SensorWall_pos,Color,1,8);
    }
}

void paintImageClass::paint_particle_sensorwall(std::vector<Vector3d> pAry,std::vector<Vector2i> tpos_wall)
{
    CvPoint SensorWall_pos;
    CvScalar Color=CV_RGB(255,119,0);

    int x = pAry[0](0);
    int y = pAry[0](1);

    CvPoint partical_pos = cvPoint(x,y);
    //std::cout << "x:" << x << std::endl;
    //std::cout << "y:" << y << std::endl;
    for(int k=0;k<tpos_wall.size();k++)
    {
        //std::cout << "x:" << sensorWall_Pos[i](0) << "\t" << "y:" << sensorWall_Pos[i](1) << std::endl;
        SensorWall_pos = cvPoint(tpos_wall[k](0),tpos_wall[k](1));
        line(image,partical_pos,SensorWall_pos,Color,1,8);
    }
}

void paintImageClass::refresh_window()
{
    image = cv::Mat(mapW,mapH,CV_8UC3,Scalar(255,255,255));
    image = imread( "/home/iclab/FIRA_ws/devel/lib/particle_filter/Ground.png", 1 );
}
void paintImageClass::paintRobot_Particle(Vector3d robot, std::vector<Vector3d> pAry)
{
    refresh_window();
    paint_particle(pAry);
    paint_robot(robot);
    imshow(imageName,image);
    waitKey(60);
}

void paintImageClass::paintRobot_Sensorlines_Particle(Vector3d robot, std::vector<Vector3d> pAry,std::vector<Vector2i> sensorWall_Pos)
{
    refresh_window();
    paint_sensorline(robot,sensorWall_Pos);
    paint_particle(pAry);
    paint_robot(robot);
    imshow(imageName,image);
    waitKey(60);
}

//to verify particle sensor
void paintImageClass::paintRobot_Particle_PSensor(Vector3d robot, std::vector<Vector3d> pAry,std::vector<Vector2i> sensorWall_Pos,std::vector<Vector2i> tp_wall)
{
    image = cv::Mat(mapW,mapH,CV_8UC3,Scalar(255,255,255));
    image = imread( "/home/iclab/FIRA_ws/devel/lib/particle_filter/Ground.png", 1 );
    paint_sensorline(robot,sensorWall_Pos);

    int x = pAry[0](0);
    int y = pAry[0](1);

    //std::cout << "out x:" << x << std::endl;
    //std::cout << "out y:" << y << std::endl;
    paint_particle_sensorwall(pAry,tp_wall);
    paint_particle(pAry);
    paint_robot(robot);

    imshow(imageName,image);
    waitKey(60);
}
