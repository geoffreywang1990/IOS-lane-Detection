//
//  cannyHough.hpp
//  IOS-Lane-Detection
//
//  Created by juedoul on 12/5/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#ifndef cannyHough_hpp
#define cannyHough_hpp

#include <stdio.h>
#include "calculate.hpp"

#endif /* cannyHough_hpp */
struct CrossPoint
{
    Line  *line;
    int x;
    int y;
    double sad;
};
void get_lint_para(Line &l);
bool get_cross_point(Line &l1,Line &l2,CrossPoint &p);
int array_point(CrossPoint pair[],int p,int r);
void point_quick_sort(CrossPoint pair[],int p,int r);
cv::Mat meanshift_filter(cv::Mat img);
double cal_block(cv::Mat src, CvPoint pointA,CvPoint pointB,int size);
bool find_road_line(cv::Mat src, cv::Mat dst, cv::Mat color_dst,CvPoint center, Line *pLines, Line scalLine, int num);
cv::Mat houghDetect(cv::Mat inputImage);
void getTrueLane(cv::Mat img,cv::Mat edgeMap, cv::vector<cv::Vec4i> linesdetected);
