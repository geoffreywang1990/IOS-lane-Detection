//
//  detectLane.cpp
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/27/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#include "detectLane.hpp"


cv::Mat getLines(cv::Mat frame)
{
    
    cv::Mat newFrame = cv::Mat(640,480,CV_8UC1,0.0);
    cv::resize(frame, newFrame, newFrame.size());
    cv::Mat temp = cv::Mat(newFrame.rows, newFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    int halfRows = newFrame.rows - newFrame.rows/2;          //rows in region of interest
    float maxLaneWidth = 0.08 * newFrame.cols;                     //approximate max lane width based on image size
    


    medianBlur(newFrame,newFrame,5 );
   // process ROI(bottom half)
    for(int i=newFrame.rows/2; i<newFrame.rows; i++){
        int laneWidth = 10+ maxLaneWidth*(i-newFrame.rows/2)/halfRows;
        for(int j=laneWidth; j<newFrame.cols- laneWidth; j++){
            int leftDiff = newFrame.at<uchar>(i,j) - newFrame.at<uchar>(i,j-laneWidth);
            int rightDiff = newFrame.at<uchar>(i,j) - newFrame.at<uchar>(i,j+laneWidth);
            int diff  =  leftDiff + rightDiff - abs(leftDiff-rightDiff);
            
          
            int diffThresh = newFrame.at<uchar>(i,j)/2;
            
            //both left and right differences can be made to contribute
            //at least by certain threshold (which is >0 right now)
            //total minimum Diff should be atleast more than 5 to avoid noise
            if (leftDiff>0 && rightDiff >0 && diff>5)
                if(diff>=diffThresh)
                    temp.at<uchar>(i,j)=255;
        }
        
    }
   
    cv::Mat lane =deNoise(temp, newFrame);
    return newFrame;
}

cv::Mat deNoise( cv::Mat lane,cv::Mat frame)
{
    cv::Mat temp = cv::Mat(frame.rows, frame.cols, CV_8UC1,0.0);//stores finally selected lane marks
    cv::Mat binary_image; //used for blob removal
    int longLane       = 0.3 * frame.rows;
    int minRegionSize  = 0.002 * (frame.cols*frame.rows);  //min size of any region to be selected as lane
    int smallLaneArea  = 7 * minRegionSize;
    int ratio          = 4;
    
    //lines defination
    int leftLine  = 2*frame.cols/5;
    int rightLine = 3*frame.cols/5;
    int topLine   = 2*frame.rows/3;
    
    cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::RotatedRect rotated_rect;
    
    // find all contours in image
    lane.copyTo(binary_image);
    
    
    
    
    
    findContours(binary_image, contours,hierarchy, CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    
    
    //  get real lines
    if (!contours.empty())
    {
        for (int i=0; i<contours.size(); ++i)
        {
            float contour_area = contourArea(contours[i]) ;
            float mincontour = 0.003 * (frame.cols*frame.rows);
            
            //blob size should not be less than lower threshold
            if(contour_area > mincontour)
            {
                rotated_rect    = minAreaRect(contours[i]);
                cv::Size2f sz              = rotated_rect.size;
                float contour_width  = sz.width;
                float contour_length = sz.height;
                
                
                //openCV selects length and width based on their orientation
                //so angle needs to be adjusted accordingly
                float blob_angle_deg = rotated_rect.angle;
                if (contour_width < contour_length)
                    blob_angle_deg = 90 + blob_angle_deg;
                
                // line longer than longLane must be road mark.
                if(contour_length>longLane || contour_width >longLane)
                {
                    drawContours(frame, contours,i, cvScalar(0), CV_FILLED, 8);
                    drawContours(temp, contours,i, cvScalar(255), CV_FILLED, 8);
                }
                
                //angle of orientation of blob should not be near horizontal or vertical
                //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                //length:width >= ratio for valid line segments
                //if area is very small then ratio limits are compensated
                
                else if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) &&
                         ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
                          (rotated_rect.center.y > topLine &&
                           rotated_rect.center.x > leftLine && rotated_rect.center.x <rightLine)))
                {
                    
                    if ((contour_length/contour_width)>=ratio || (contour_width/contour_length)>=ratio
                        ||(contour_area< smallLaneArea &&  ((contour_area/(contour_width*contour_length)) > .75) &&
                           ((contour_length/contour_width)>=2 || (contour_width/contour_length)>=2)))
                    {
                        drawContours(frame, contours,i, cvScalar(0), CV_FILLED, 8);
                        drawContours(temp, contours,i, cvScalar(255), CV_FILLED, 8);
                    }
                }
            }
        }
    }
    

    return temp;

};



