//
//  cannyHough.cpp
//  IOS-Lane-Detection
//
//  Created by juedoul on 12/5/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#include "cannyHough.hpp"


#define  MAX(a, b)  a > b ? a:b
#define  MIN(a, b)  a < b ? a:b
using namespace cv;

#define PTR_FREE(ptr)\
{\
    if ((ptr) != (NULL))\
    {\
        free(ptr);\
        ptr = NULL;\
    }\
}

#define IF_PTR_NULL(ptr, result)\
{\
    if ((ptr) == (NULL))\
    {\
        printf("%s:%d [%s] ",__FILE__,__LINE__,__FUNCTION__);\
        printf("ptr == NULL \n");\
        return result;\
    }\
}

void get_lint_para(Line &l)
{
    l.a = l.p1.y-l.p2.y;
    l.b = l.p2.x-l.p1.x;
    l.c = l.p1.x*l.p2.y-l.p1.y*l.p2.x;
}
//if 2 lines have a cross point, if do, calculate the coordinate
bool get_cross_point(Line &l1,Line &l2,CrossPoint &p)
{
    get_lint_para(l1);
    get_lint_para(l2);
    double D = l1.a*l2.b - l2.a*l1.b;
    
    if (D == 0)
    {
        return false;
    }
    
    p.x = (l1.b*l2.c-l2.b*l1.c)/D;
    p.y = (l1.c*l2.a-l2.c*l1.a)/D;
    
    return true;
}

int array_point(CrossPoint pair[],int p,int r)
{
    int e = rand()%(r-p+1)+p;
    CrossPoint tem;
    tem = pair[e];
    pair[e] = pair[r];
    pair[r] = tem;
    double x = pair[r].x;
    int i = p-1;
    for (int j = p; j<r; j++)
    {
        if (pair[j].x <= x)
        {
            tem = pair[i+1];
            pair[i+1] = pair[j];
            pair[j] = tem;
            i++;
        }
    }
    
    tem = pair[r];
    pair[r] = pair[i+1];
    pair[i+1] = tem;
    
    return i+1;
}
//quick sort the points according to the value of x coordinate
void point_quick_sort(CrossPoint pair[],int p,int r)
{
    if (p<r)
    {
        int q=array_point(pair,p,r);
        point_quick_sort(pair,p,q-1);
        point_quick_sort(pair,q+1,r);
    }
}

/*
Mat meanshift_filter(Mat img)
{
    Mat  rgb;
    pyrMeanShiftFiltering( img, rgb,2,40,2);
    return rgb;
}
*/
double cal_block(Mat src, CvPoint pointA,CvPoint pointB,int size)//which function?
{
    double sum = 0;
    Scalar temp1;
    Scalar temp2;
    
    int m,n,m2,n2;
    m = pointA.x - size/2;
    n = pointA.y - size/2;
    
    m2 = pointB.x - size/2;
    n2 = pointB.y - size/2;
    
    for (int i=0;i<size;i++)
    {
        for (int j=0;j<size;j++)
        {
            temp1 = Scalar(src.at<uchar>((n + j), (m + i)));
            temp2 = Scalar(src.at<uchar>((n2 + j), (m2 + i)));
            sum = sum + powf(std::abs(temp1.val[0] - temp2.val[0] ), 2);
        }
    }
    sum = 1.0*sum/(size*size);
    sum = sqrtf(sum);
    return sum;
}


bool find_road_line(Mat src, Mat dst, Mat color_dst,CvPoint center, Line *pLines, Line scalLine, int num)
{
    int lineNum = 0;
    CrossPoint *pPoints;
    pPoints = (CrossPoint *)malloc(num * sizeof(CrossPoint));
    IF_PTR_NULL(pPoints,false);
    memset(pPoints,0,(num  * sizeof(CrossPoint)));
    double sad = cal_block(src, center,center,3);
    CrossPoint *temp = pPoints;
    for (int j=0; j< num; j++)
    {
        if (true == get_cross_point(pLines[j],scalLine, *temp))
        {
            int maxP = MAX(pLines[j].p1.x,pLines[j].p2.x);
            int minP = MIN(pLines[j].p1.x,pLines[j].p2.x);
            
            if ((temp->x < maxP) && (temp->x > minP))
            {
                temp->line = pLines[j];

                circle(color_dst, cvPoint(temp->x, temp->y), 4, Scalar(0,255,255));
                lineNum ++;
                temp++;
            }
            
        }
    }
    point_quick_sort(pPoints,0,(lineNum-1));
    Line *maxLine1 = NULL, *maxLine2 = NULL;
    double sadMax=0,MaxSad = 0;
    maxLine1 = &pPoints[0].line;
    for (int j=0; j< lineNum; j++)
    {
        sad = cal_block(src, center,cvPoint(pPoints[j].x,pPoints[j].y),3);
        pPoints[j].sad = sad;
        if (sad > sadMax)
        {
            maxLine1 = &pPoints[j].line;
            sadMax = sad;
        }
    }
    
    MaxSad = 0;
    maxLine2 = maxLine1;
    
    for (int j=0; j< lineNum; j++)
    {
        if ((pPoints[j].sad > MaxSad) && (pPoints[j].sad != sadMax))
        {
            maxLine2 = &pPoints[j].line;
            MaxSad = sad;
        }
    }
    
    if (NULL != maxLine1)
    {
        maxLine1->roadVal++;
    }
    
    if (NULL != maxLine2)
    {
        maxLine2->roadVal++;
    }
    
    //PTR_FREE(pPoints);
    return true;
}


CvPoint half_point(CvPoint pointA, CvPoint pointB)
{
    CvPoint point;
    point.x = (pointA.x + pointB.x)/2;
    point.y = (pointA.y + pointB.y)/2;
    
    return point;
}

Mat houghdetect(Mat inputImage)
{
    Mat  frame;
    cv::Size size = inputImage.size() ;
    frame = inputImage;
    Mat img;
    cv::Size desize = frame.size();
    resize(frame, img, desize);
    Mat src;
         src = img.clone();
    
    Mat dst, color_dst;
    int i;
    //x and y gradient
 //   Sobel(src,src,CV_8UC1, 1, 1);
 //   Canny(src, dst, 50, 200);
    //x gradient after canny
//    Sobel(dst,dst,CV_8UC1,1,0);
    //canny again
    Canny(src,dst,100,150);
    cvtColor(dst, color_dst, CV_GRAY2BGR);

    
    //Houghp to get lines
    vector<Vec4i> linesdetected;
  
    HoughLinesP(dst, linesdetected, 1,CV_PI/180, 115, 10, 70);
    
    
    Line *angleFilt = hough_link_list_create(linesdetected);
    if ((angleFilt != NULL)){
        angleFilt = angleThresh(angleFilt, 75);
        if (angleFilt == NULL)
        {
            img = dst;
        }
        else
        {
            angleFilt = linklenthSort(angleFilt);
            if( angleFilt == NULL )
            {
                img = dst;
            }
            else
            {
                Line *pTemp = angleFilt;
                int lineNum = MIN(listLengGet(angleFilt),10);
                Line *pLines = NULL;
    
                pLines = (Line *)malloc(lineNum * sizeof(Line));
                //	IF_PTR_NULL(pLines,false);
                memset(pLines,0,(lineNum * sizeof(Line)));
    
                CvPoint center;
                center = cvPoint(src.size().width/1.8,src.size().height/1.5);
                int positionMax,positionMin;
                
                for (int j=0; j< lineNum; j++){
        
                    line(color_dst, pTemp->p1, pTemp->p2, Scalar(0,0,0));
                    pLines[j].p1 = pTemp->p1;
                    pLines[j].p2 = pTemp->p2;
                    //angle of the lines, revise to angle between 0 and 180
                    if ((pTemp->angle < 180) || (180 == pTemp->angle))
                    {
                        pLines[j].angle = pTemp->angle;
                    }
                    else
                    {
                        pLines[j].angle = pTemp->angle - 180;
                    }
                    // determine the line's position, it's on the left or the right
                    if(half_point(pLines[j].p1, pLines[j].p2).x < center.x)
                    {
                        pLines[j].lefOrright = 0;
                    }
                    else
                    {
                        pLines[j].lefOrright = 1;
                    }
                    pTemp = pTemp->next;
                }
                //again color_dst hough
                Line scalLine;
                for(int i=0;i<src.size().height;i++)
                {
                    scalLine.p1 = cvPoint(0,i);
                    scalLine.p2 = cvPoint(src.size().width,i);
                    find_road_line(src,dst,color_dst,center,pLines,scalLine, lineNum);
                }
                Line  *maxLine1 = NULL, *maxLine2 = NULL;
                double valMax1=0,valMax2 = 0;
                for (int j=0; j< lineNum; j++)
                {
                    if ((pLines[j].roadVal > valMax1) && (0 == pLines[j].lefOrright))
                    {
                        maxLine1 = &pLines[j];
                        valMax1 = pLines[j].roadVal;
                    }
        
                    if ((pLines[j].roadVal > valMax2) && (1 == pLines[j].lefOrright))
                    {
                        maxLine2 = &pLines[j];
                        valMax2 = pLines[j].roadVal;
                    }
        
                }
    
                Line line1,line2,line3;
                line1.p1 = cvPoint(0,0);
                line1.p2 = cvPoint(0,src.size().height);
                line2.p1 = cvPoint(0,0);
                line2.p2 = cvPoint(src.size().width,0);
                line3.p1 = cvPoint(src.size().width,0);
                line3.p2 = cvPoint(src.size().width,src.size().height);
                CrossPoint point1,point2;
    
                if (NULL != maxLine1)
                {
                    if ((true == get_cross_point( *maxLine1,line1,point1)) && (true == get_cross_point( *maxLine1,line2,point2)))
                    {
                        cvLine(&img,cvPoint(point1.x,point1.y),cvPoint(point2.x,point2.y),CV_RGB(0,128,192),2,CV_AA);
                    }
        
                }
                if (NULL != maxLine2)
                {
                    if ((true == get_cross_point(*maxLine2,line3,point1)) && (true == get_cross_point(*maxLine2,line2,point2)))
                    {
                        //cvLine(color_dst,cvPoint(point1.x,point1.y),cvPoint(point2.x,point2.y),CV_RGB(0,128,192),2,CV_AA);
                        cvLine(&img,cvPoint(point1.x,point1.y),cvPoint(point2.x,point2.y),CV_RGB(0,128,192),2,CV_AA);
                    }
                }
    
              //  PTR_FREE(pLines);

            }
        }
    }

    return img;
    
}
