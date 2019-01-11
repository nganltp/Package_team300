#ifndef DETECTLANE_H
#define DETECTLANE_H

#include "header.h"

class LaneDetector
{
  public:
    LaneDetector(){
      centerPoint.y = 150;
      check_left.y = 150;
      check_right.y = 150;
      re_check.y = 140;
      err = 60; //60
    };
    ~LaneDetector() = default;

  private:
    vector<Point> leftLane, rightLane;
    float laneWidth = LANE_WIDTH;

    vector<Mat> SplitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point>> CenterRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void DetectLeftRight(const vector<vector<Point>> &points);

   
    void FillLane(Mat &src);
    /*____ Hoang Phuc ____*/
    int err;
    Point centerPoint, preCenterPoint;
    
    int reGetLineLeft(const Mat &src);
    
    int reGetLineRight(const Mat &src);
    void CalculateCenterPoint(const Mat &src);
    /*____________________*/

  public:
    Mat PreProcess(const Mat &src);
    int getLineLeft(const Mat &src);
    int getLineRight(const Mat &src);
    void Update(const Mat &src);
    Point GetCenterPoint() { return centerPoint; }
    bool turnLR(const Mat &src);
    Point check_left;
    Point check_right;
    Point re_check;
};

#endif
