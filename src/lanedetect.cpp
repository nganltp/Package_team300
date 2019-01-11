#include "lanedetect.h"
#include "imageprocessing.h"

/*___ Hoang Phuc ___*/

bool LaneDetector::turnLR(const Mat &src){
    Mat temp, dst, imgshow;
    bool start_turn = false;
    cvtColor(src, temp, COLOR_BGR2GRAY);
    Canny(temp, dst, 150, 255); // 150 255
    cvtColor(dst, dst, COLOR_GRAY2BGR);
    imgshow = dst.clone();
    line(imgshow,Point(0,150),Point(319,150), Scalar(0,255,0),2);
    imshow("turnLEFT_RIGHT",imgshow);
    for(int i = 0; i< FRAME_WIDTH; i++){
        Vec3b pixelValue = dst.at<Vec3b>(150,i);
        if (pixelValue.val[2] == 255) return false;
        if(i == FRAME_WIDTH -1) {
            cout << "OK" <<endl;
            return true;        // start turn 90
        }
    }
    //flag_turn = false;
}

int LaneDetector::getLineLeft(const Mat &src){
    for(int i = FRAME_WIDTH/2; i >0; i--){
        //int pixelValue = (int)src.at<uchar>(145,i);
        Vec3b pixelValue = src.at<Vec3b>(centerPoint.y,i);
        if (pixelValue.val[2] == 255){
            check_left.x = i;
            return i;
        }
    }
    return -1;
}

int LaneDetector::reGetLineLeft(const Mat &src){
    for(int i = FRAME_WIDTH/2; i >0; i--){
        Vec3b pixelValue = src.at<Vec3b>(re_check.y,i);
        if (pixelValue.val[2] == 255){
            re_check.x = i;
            return i;
        }
    }
    return -1;
}

int LaneDetector::getLineRight(const Mat &src){
    for(int i = FRAME_WIDTH/2; i < FRAME_WIDTH; i++){
        //int pixelValue = (int)src.at<uchar>(145,i);
        Vec3b pixelValue = src.at<Vec3b>(centerPoint.y,i);
        if (pixelValue.val[2] == 255){
            check_right.x = i;
            return i;
        }
    }
    return -1;
}

int LaneDetector::reGetLineRight(const Mat &src){
    for(int i = FRAME_WIDTH/2; i < FRAME_WIDTH; i++){
        Vec3b pixelValue = src.at<Vec3b>(re_check.y,i);
        if (pixelValue.val[2] == 255){
            re_check.x = i;
            return i;
        }
    }
    return -1;
}

void LaneDetector::CalculateCenterPoint(const Mat &src)
{
        if((getLineLeft(src)!= -1) && (getLineRight(src)!= -1)){
            //cout << "detect 2 lines" << endl;
            //cout << getLineLeft(src) <<"   " << getLineRight(src) << endl;
            centerPoint.x = ((getLineLeft(src) + getLineRight(src))/2);
        }
        else if (getLineLeft(src)!= -1){
            if(reGetLineLeft(src) < getLineLeft(src)){
                //cout << "detect 1 line - right" << endl;
                //cout << getLineLeft(src) << endl;
                centerPoint.x = (getLineLeft(src) - err);
            }
            else{
               // cout << "detect 1 line - left" << endl;
               // cout << getLineLeft(src) << endl;
                centerPoint.x = (getLineLeft(src) + err);
            }
        }
        else if (getLineRight(src) != -1){
            if(reGetLineRight(src) > getLineRight(src)){
                //cout << "detect 1 line - left" << endl;
                //cout << getLineRight(src) << endl;
                centerPoint.x = (getLineRight(src) + err);
            }
            else{
                //cout << "detect 1 line - right" << endl;
                //cout << getLineRight(src) << endl;
                centerPoint.x = (getLineRight(src) - err);
            }
        }
}

/*__________________*/

/*
Mat LaneDetector::PreProcess(const Mat &src)
{
    Mat imgThresholded, imgHSV, dst;

    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, MIN_HSV_BLACK, MAX_HSV_BLACK, imgThresholded);

    dst = ip->birdViewTranform(imgThresholded);

    imshow("Bird View", dst);

    FillLane(dst);

    return dst;
}
*/

Mat LaneDetector::PreProcess(const Mat &src){
    Mat temp, dst, hl, hlP, img;
    cvtColor(src, temp, COLOR_BGR2GRAY);
    //blur( temp, temp, Size(test6+1,test6+1));
    Canny(temp, dst, 150, 255); // 150 255
    imshow("Binary", dst);
    hlP = Mat::zeros(src.size(),CV_8UC1);
    hl = Mat::zeros(src.size(),CV_8UC1);
    img = Mat::zeros(src.size(),CV_8UC1);
    {
        vector<Vec2f> lines;
        HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0); // 180 0 0
        for( size_t i = 0; i < lines.size(); i++ )
        {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line( hl, pt1, pt2, Scalar(255), 5, LINE_AA);
        }
    }
    vector<Vec4i> liness;
    HoughLinesP(dst, liness, 1, CV_PI/180, 90, 50, 5); //90 50 5
    for( size_t i = 0; i < liness.size(); i++ )
    {
        line( hlP, Point(liness[i][0], liness[i][1]), Point( liness[i][2], liness[i][3]), Scalar(255), 5, 8 );
    }
    imshow("hl", hl);
    imshow("OUTPUT", hlP);
    cvtColor(hlP, hlP, COLOR_GRAY2BGR);
    return hlP;
    bitwise_and(hl, hlP, img);
    cvtColor(img, img, COLOR_GRAY2BGR);
    imshow("result", img);
    
}

void LaneDetector::Update(const Mat &src)
{
    Mat img = PreProcess(src);
    CalculateCenterPoint(img);

}


vector<Mat> LaneDetector::SplitLayer(const Mat &src, int dir)
{
    int rowN = src.rows;
    int colN = src.cols;
    std::vector<Mat> res;

    if (dir == VERTICAL)
    {
        for (int i = 0; i < rowN - SLIDE_THICKNESS; i += SLIDE_THICKNESS)
        {
            Mat tmp;
            Rect crop(0, i, colN, SLIDE_THICKNESS);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }
    else
    {
        for (int i = 0; i < colN - SLIDE_THICKNESS; i += SLIDE_THICKNESS)
        {
            Mat tmp;
            Rect crop(i, 0, SLIDE_THICKNESS, rowN);
            tmp = src(crop);
            res.push_back(tmp);
        }
    }

    return res;
}

vector<vector<Point>> LaneDetector::CenterRoadSide(const vector<Mat> &src, int dir)
{
    vector<std::vector<Point>> res;
    int inputN = src.size();
    for (int i = 0; i < inputN; i++)
    {
        std::vector<std::vector<Point>> cnts;
        std::vector<Point> tmp;
        findContours(src[i], cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        int cntsN = cnts.size();
        if (cntsN == 0)
        {
            res.push_back(tmp);
            continue;
        }

        for (int j = 0; j < cntsN; j++)
        {
            int area = contourArea(cnts[j], false);
            if (area > 3)
            {
                Moments M1 = moments(cnts[j], false);
                Point2f center1 = Point2f(static_cast<float>(M1.m10 / M1.m00), static_cast<float>(M1.m01 / M1.m00));
                if (dir == VERTICAL)
                {
                    center1.y = center1.y + SLIDE_THICKNESS * i;
                }
                else
                {
                    center1.x = center1.x + SLIDE_THICKNESS * i;
                }
                if (center1.x > 0 && center1.y > 0)
                {
                    tmp.push_back(center1);
                }
            }
        }
        res.push_back(tmp);
    }

    return res;
}

void LaneDetector::DetectLeftRight(const vector<vector<Point>> &points)
{
    static vector<Point> lane1, lane2;
    lane1.clear();
    lane2.clear();

    leftLane.clear();
    rightLane.clear();
    for (int i = 0; i < BIRDVIEW_HEIGHT / SLIDE_THICKNESS; i++)
    {
        leftLane.push_back(POINT_ZERO);
        rightLane.push_back(POINT_ZERO);
    }

    int pointMap[points.size()][20];
    int prePoint[points.size()][20];
    int postPoint[points.size()][20];
    int dis = 10;
    int max = -1, max2 = -1;
    Point2i posMax, posMax2;

    memset(pointMap, 0, sizeof pointMap);

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            pointMap[i][j] = 1;
            prePoint[i][j] = -1;
            postPoint[i][j] = -1;
        }
    }

    for (int i = points.size() - 2; i >= 0; i--)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            int err = 320;
            for (int m = 1; m < min((int)points.size() - 1 - i, 5); m++)
            {
                bool check = false;
                for (int k = 0; k < points[i + 1].size(); k++)
                {
                    if (abs(points[i + m][k].x - points[i][j].x) < dis &&
                        abs(points[i + m][k].x - points[i][j].x) < err)
                    {
                        err = abs(points[i + m][k].x - points[i][j].x);
                        pointMap[i][j] = pointMap[i + m][k] + 1;
                        prePoint[i][j] = k;
                        postPoint[i + m][k] = j;
                        check = true;
                    }
                }
                break;
            }

            if (pointMap[i][j] > max)
            {
                max = pointMap[i][j];
                posMax = Point2i(i, j);
            }
        }
    }

    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            if (pointMap[i][j] > max2 && (i != posMax.x || j != posMax.y) && postPoint[i][j] == -1)
            {
                max2 = pointMap[i][j];
                posMax2 = Point2i(i, j);
            }
        }
    }

    if (max == -1)
        return;

    while (max >= 1)
    {
        lane1.push_back(points[posMax.x][posMax.y]);
        if (max == 1)
            break;

        posMax.y = prePoint[posMax.x][posMax.y];
        posMax.x += 1;

        max--;
    }

    while (max2 >= 1)
    {
        lane2.push_back(points[posMax2.x][posMax2.y]);
        if (max2 == 1)
            break;

        posMax2.y = prePoint[posMax2.x][posMax2.y];
        posMax2.x += 1;

        max2--;
    }

    vector<Point> subLane1(lane1.begin(), lane1.begin() + 5);
    vector<Point> subLane2(lane2.begin(), lane2.begin() + 5);

    Vec4f line1, line2;

    fitLine(subLane1, line1, 2, 0, 0.01, 0.01);
    fitLine(subLane2, line2, 2, 0, 0.01, 0.01);

    int lane1X = (BIRDVIEW_WIDTH - line1[3]) * line1[0] / line1[1] + line1[2];
    int lane2X = (BIRDVIEW_WIDTH - line2[3]) * line2[0] / line2[1] + line2[2];

    if (lane1X < lane2X)
    {
        for (int i = 0; i < lane1.size(); i++)
        {
            leftLane[floor(lane1[i].y / SLIDE_THICKNESS)] = lane1[i];
        }
        for (int i = 0; i < lane2.size(); i++)
        {
            rightLane[floor(lane2[i].y / SLIDE_THICKNESS)] = lane2[i];
        }
    }
    else
    {
        for (int i = 0; i < lane2.size(); i++)
        {
            leftLane[floor(lane2[i].y / SLIDE_THICKNESS)] = lane2[i];
        }
        for (int i = 0; i < lane1.size(); i++)
        {
            rightLane[floor(lane1[i].y / SLIDE_THICKNESS)] = lane1[i];
        }
    }
}


void LaneDetector::FillLane(Mat &src)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 1);
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 3, CV_AA);
    }
}