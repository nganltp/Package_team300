#include "header.h"
#include "imageprocessing.h"
#include "signdetect.h"
#include "lanedetect.h"
#include "carcontrol.h"

ImageProcessor *ip;
SignDetector *signDetector;
LaneDetector *laneDetector;
CarController *carController;

Mat debugImg;

//VideoWriter *writer;
//VideoCapture *capture;
Mat out;
bool flag_turn = false;
bool LoR = false;       //L: false      R: true
void show();

void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat colorImg;

    try
    {
        // From subscriber
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        waitKey(1);

        // Original
        colorImg = cv_ptr->image.clone();
        out = cv_ptr->image.clone();
        //resize(colorImg, colorImg, FRAME_SIZE);
        imshow("View", colorImg);

        // Debug
        debugImg = colorImg.clone();

        // Log to video
        //writer->write(colorImg);

        signDetector->Update(colorImg);

        ESignType signType = signDetector->GetType();
        if(signType != ESignType::NONE){
            flag_turn = true;
            if(signType == ESignType::TURN_LEFT) LoR = false;
            else LoR = true;
        }
        // &&(laneDetector->getLineLeft(laneDetector->PreProcess(colorImg))==-1 || laneDetector->getLineRight(laneDetector->PreProcess(colorImg))==-1)
        if(flag_turn == true ){
            if(laneDetector->turnLR(colorImg) == false){
                // chay theo line toi khi nao return true (bat dau be cua)
                laneDetector->Update(colorImg);
                carController->DriverCar(laneDetector->GetCenterPoint(), 20, signType);
                cout << "chay theo line" << endl;
            }
            if(laneDetector->turnLR(colorImg) == true && LoR == true){
                carController->turn90(35,15);
                cout << "CUA" << endl;
                if(laneDetector->getLineLeft(laneDetector->PreProcess(colorImg))!= -1 || laneDetector->getLineLeft(laneDetector->PreProcess(colorImg))!= -1){
                    flag_turn = false;
                }
            }
            if(laneDetector->turnLR(colorImg) == true && LoR == false){
                carController->turn90(-35,15);
                cout << "CUA" << endl;
                if(laneDetector->getLineLeft(laneDetector->PreProcess(colorImg))!= -1 || laneDetector->getLineLeft(laneDetector->PreProcess(colorImg))!= -1){
                    flag_turn = false;
                }
            }
        }
        else {   
               //chay theo line
            flag_turn = false;
            laneDetector->Update(colorImg);
            carController->DriverCar(laneDetector->GetCenterPoint(), SPEED, signType);
            show();
            imshow("TRACKING", out);
        }
        //imshow("Debug", debugImg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageProcess(const char *path)
{
    Mat src = imread(path);
    imshow("View", src);
    //laneDetector->Update(src);
    waitKey(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    //cv::namedWindow("View");
    //cv::namedWindow("Binary");
    //cv::namedWindow("Bird View");
    //cv::namedWindow("Lane Detect");
    //cv::namedWindow("Debug");
    //cv::namedWindow("Sign Binary");
    //cv::namedWindow("crop");

    SAFE_ALLOC(laneDetector, LaneDetector);
    SAFE_ALLOC(carController, CarController);

    //signDetector = new SignDetector("svm_model.xml");
    SAFE_ALLOC_P1(signDetector, SignDetector, "svm_model.xml");

    //writer = new VideoWriter("out.avi", CV_FOURCC('M','J','P','G'), 30, Size(FRAME_WIDTH, FRAME_HEIGHT));

    if (STREAM)
    {
        //cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(IMAGE_SUB, 1, ImageCallback);

        ros::spin();
    }
    else
    {
        //ImageProcess("test.png");
        //VideoProcess("inp.avi");
    }

    //writer->release();

    SAFE_FREE(signDetector);
    SAFE_FREE(laneDetector);
    SAFE_FREE(carController);

    cv::destroyAllWindows();
}
void show(){
    circle(out, Point(laneDetector->check_left.x, laneDetector->check_left.y), 5, Scalar(0,255,0), -1);
    circle(out, Point(laneDetector->check_right.x, laneDetector->check_right.y), 5, Scalar(255,255,0), -1);
    circle(out, Point(laneDetector->re_check.x, laneDetector->re_check.y), 5, Scalar(0,255,255), -1);
    circle(out, Point(laneDetector->GetCenterPoint().x, laneDetector->GetCenterPoint().y), 5, Scalar(0,0,255), -1);
    line(out,Point(160,0),Point(160,240), Scalar(255,255,255),2);
    line(out,Point(0,laneDetector->GetCenterPoint().y),Point(320,laneDetector->GetCenterPoint().y), Scalar(255,255,255),2);
}
