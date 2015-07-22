/*
 * csll is for serial port (RS232)
 */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "MotorControl.h"


//====motor define=====
#define ROBOT_RADIUS 0.15
#define WheelRadius  0.0508
#define RPM_Max      6940
#define Gear         26
#define PWM_Range    127.0
#define PWM_Limit_Percent_Min   0.2
#define PWM_Limit_Percent_Max   0.9
#define RPM_Offset 0
#define PWM_MIN 20.0
#define PWM_MAX 103

//-------variable-------//
const char *motion_topic_name = "/cmd_vel";

const double mAngle1Cos(cos(5*M_PI/6));
const double mAngle2Cos(cos(M_PI/6));
const double mAngle3Cos(cos(3*M_PI/2));

const double mAngle1Sin(sin(M_PI/6));
const double mAngle2Sin(sin(5*M_PI/6));
const double mAngle3Sin(sin(3*M_PI/2));

const double rad2rpm =  60.0  / (2.0*M_PI);
const double percentRange =  PWM_Limit_Percent_Max - PWM_Limit_Percent_Min;

double w1,w2,w3;
double en1,en2,en3;

/*==============================================================================*/
//Initialize
/*==============================================================================*/
void Initialize()
{
    w1=0;
    w2=0;
    w3=0;
}
/*==============================================================================*/
//Transform wheel speed to PWM
/*==============================================================================*/
int speed2pwm(double en)
{
    int pwm_1,pwm_2,pwm_3;
    float pwmRatio_1,pwmRatio_2,pwmRatio_3;
    bool w1_dir,w2_dir,w3_dir;

    en1 = en2 = en3 = en;

    double _num = 1;

    if(w1<0)    w1_dir=false;   else    w1_dir=true;
    if(w2<0)    w2_dir=false;   else    w2_dir=true;
    if(w3<0)    w3_dir=false;   else    w3_dir=true;

    w1 = w1 * (1/WheelRadius)  * Gear/_num;
    w2 = w2 * (1/WheelRadius)  * Gear/_num;
    w3 = w3 * (1/WheelRadius)  * Gear/_num;

    //to rpm
    w1 = w1 * rad2rpm;
    w2 = w2 * rad2rpm;
    w3 = w3 * rad2rpm;

    ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);

    pwmRatio_1 = ( abs(w1) - (float)RPM_Offset) / (float)RPM_Max;  // 0.56324
    pwmRatio_2 = ( abs(w2) - (float)RPM_Offset) / (float)RPM_Max;  //-0.56324
    pwmRatio_3 = ( abs(w3) - (float)RPM_Offset) / (float)RPM_Max;  //

    //ROS_INFO("pwmRatio_1:%f\tpwmRatio_2:%f\tpwmRatio_3:%f\n",pwmRatio_1,pwmRatio_2,pwmRatio_3);

    if(pwmRatio_1 > 1.0 || pwmRatio_2 > 1.0 || pwmRatio_3 > 1.0)
    {
        double pwmMax = pwmRatio_1;
        pwmMax = pwmMax > pwmRatio_2 ? pwmMax : pwmRatio_2;
        pwmMax = pwmMax > pwmRatio_3 ? pwmMax : pwmRatio_3;
        double tRatio = 1.0/(double)pwmMax;
        pwmRatio_1 = pwmRatio_1 * tRatio;
        pwmRatio_2 = pwmRatio_2 * tRatio;
        pwmRatio_3 = pwmRatio_3 * tRatio;
    }
    if(pwmRatio_1 < 0.0 || pwmRatio_2 < 0.0 || pwmRatio_3 < 0.0)
    {
        en1=0;
        en2=0;
        en3=0;
    }
    pwm_1 = (int)(  (pwmRatio_1 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    pwm_2 = (int)(  (pwmRatio_2 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    //pwm_3 = (int)(  (pwmRatio_3 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );
    pwm_3 = (int)(  (pwmRatio_3 *percentRange + PWM_Limit_Percent_Min )*PWM_Range   );

    //ROS_INFO("pwm1:%d\tpwm2:%d\tpwm3:%d\n",pwm_1,pwm_2,pwm_3);

    if(pwm_1 <= PWM_MIN || pwm_1 >= PWM_MAX)    en1 = 0;
    if(pwm_2 <= PWM_MIN || pwm_2 >= PWM_MAX)    en2 = 0;
    if(pwm_3 <= PWM_MIN || pwm_3 >= PWM_MAX)    en3 = 0;

    if(w1_dir) w1=pwm_1;else w1=pwm_1*-1;
    if(w2_dir) w2=pwm_2;else w2=pwm_2*-1;
    if(w3_dir) w3=pwm_3;else w3=pwm_3*-1;
    
    if(abs(w1)<=25)en1=0;
    if(abs(w2)<=25)en2=0;
    if(abs(w3)<=25)en3=0;

    if(abs(w1)>103)en1=0;
    if(abs(w2)>103)en2=0;
    if(abs(w3)>103)en3=0;

}
/*==============================================================================*/
//Topic call back
/*==============================================================================*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    
    w1 = mAngle1Cos*msg->linear.y + mAngle1Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z;
    w2 = mAngle2Cos*msg->linear.y + mAngle2Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z;
    w3 = mAngle3Cos*msg->linear.y + mAngle3Sin*msg->linear.x + ROBOT_RADIUS*msg->angular.z;
    
    int times= 1;
    w1 = w1*times;
    w2 = w2*times;
    w3 = w3*times;

    double en = 1;

    ROS_INFO("<==============================>");
    ROS_INFO("w1:%f\tw2:%f\tw3:%f\n",w1,w2,w3);
    ROS_INFO("<==============================>");
    //w3=100;
    speed2pwm(en);

    mcssl_send2motor(w1,w2,w3,1,1,1);
}
/*==============================================================================*/
//Main
/*==============================================================================*/
int main(int argc, char **argv)
{
    //Initial
    ros::init(argc, argv, "FIRA_Motion");
    ros::NodeHandle n;
    if(mcssl_init()<=0){return 0;}
    Initialize();

    //motion subscriber
    ros::Subscriber sub = n.subscribe(motion_topic_name, 1000, motionCallback);
    
    ros::Rate loop_rate(5);

    while(ros::ok())
    {
	//mcssl_send2motor(w1,w2,w3);

        //spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    //RS232 finish
    mcssl_finish();

    return 0;
}
