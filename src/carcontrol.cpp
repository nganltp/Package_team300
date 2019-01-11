#include "carcontrol.h"

CarController::CarController()
{
    carPos.x = CAR_POSITION_X;
    carPos.y = CAR_POSITION_Y;
    kP = 0.3; //0.52
    kD = 0.3; //0.6
    steer_publisher = node_obj1.advertise<std_msgs::Float32>(STEER_ANGLE_PUB, 10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>(SPEED_PUB, 10);
}

CarController::~CarController() {}

float CarController::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x)
        return 0;
    if (dst.y == carPos.y)
        return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;
    if (dx < 0)
        return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarController::turn90(int _angle, int _speed){
    angle.data = _angle;
    speed.data = _speed;
    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}

void CarController::runForwardLine(Point centerPoint, float velocity){
    float _velocity = velocity;
    err = centerPoint.x - carPos.x;
    _angle = (kP * err + kD * (err - preError));
    preError = err;
    delta = abs(_angle);
    if(0) _velocity = 60;
    else if(40 <= delta < 60) _velocity = 40;
    else if(60<= delta) _velocity = 20;
    //  Limited Speed and Angle
    if(_velocity < minVelocity) _velocity = minVelocity;
    if(_angle < minAngle) _angle = minAngle;
    if(_angle > maxAngle) _angle = maxAngle;
    angle.data = _angle;
    speed.data = _velocity;
}
void CarController::turnLeft_Right(Mat src){
    Mat temp, dst;
    bool start_turn = false;
    cvtColor(temp, temp, COLOR_BGR2GRAY);
    Canny(temp, dst, 150, 255); // 150 255
    cvtColor(dst, dst, COLOR_GRAY2BGR);
    //line(dst,Point(160,0),Point(160,240), Scalar(255,255,255),2);
    //imshow("turnLEFT_RIGHT",dst);
    // for(int i = 0; i< FRAME_WIDTH; i++){
    //     Vec3b pixelValue = dst.at<Vec3b>(200,i);
    //     if (pixelValue.val[2] == 255) break;
    //     if(i == FRAME_WIDTH -1) {
    //         start_turn = true;
    //         cout << "OK" <<endl;
    //     }
    // }
    //bool start_turn = true;
    if (start_turn == true && LoR == true){
        angle.data = 30;
        speed.data = 40;
    }
    flag_turn = false;
}
void CarController::DriverCar(Point centerPoint, float velocity, ESignType flag)
{
    runForwardLine(centerPoint, velocity);
    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
}
