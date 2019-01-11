#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "header.h"

class CarController
{
  public:
    CarController();
    ~CarController();
    void DriverCar(Point centerPoint, float velocity, ESignType flag);
    void turn90(int angle, int speed);

  private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;

    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float minVelocity = MIN_VELOCITY;
    float maxVelocity = MAX_VELOCITY;
    /*___ Hoang Phuc ____*/
    bool flag_turn = false;   // True: co bien bao
    bool LoR = false;         // L: False   R: True
    float minAngle = -45;
    float maxAngle = 45;
    float preError,err; 
    float _angle, delta;
    void runForwardLine(Point centerPoint, float velocity);
    void turnLeft_Right(Mat src);
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;
    /*___________________*/

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;
};

#endif
