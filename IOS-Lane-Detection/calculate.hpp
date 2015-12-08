//
//  calculate.hpp
//  IOS-Lane-Detection
//
//  Created by juedoul on 12/5/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#ifndef calculate_hpp
#define calculate_hpp

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <vector>
#include <stdio.h>

#endif /* calculate_hpp */



typedef struct _Line
{
    CvPoint p1,p2;  //larger one is p1
    double a,b,c;  //parameters of line:ax+by+c=0
    double angle; //angle of the line
    double lenth; //length of the line
    int roadVal;
    int lefOrright;//0 is left, 1 is right
    bool nextIsNull;
    _Line * next;
}Line;

double getAngle(CvPoint pointO,CvPoint pointA);
double getDistance (CvPoint pointO,CvPoint pointA );
Line * linkListCreat(int num, Line * pcornerPair);
Line * delnode(Line * h , Line * maxp);
bool angle_ok(double ang, double angThresh);
Line * angleThresh(Line*head, double angThresh);
int listLengGet(Line*head);
Line * delNodeForSort(Line * h , Line * maxp)  ;
Line * linklenthSort(Line *st);
Line * hough_link_list_create(cv::vector<cv::Vec4i> lines);
