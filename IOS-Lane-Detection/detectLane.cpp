//
//  detectLane.cpp
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/27/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#include "detectLane.hpp"
#include "iostream"

cv::Mat getLines(cv::Mat frame)
{
    
    cv::Mat newFrame;
    cv::cvtColor(frame, newFrame, CV_RGB2GRAY);
    
    cv::Mat temp = cv::Mat(newFrame.rows, newFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    int halfRows = newFrame.rows - newFrame.rows/2;          //rows in region of interest
    float maxLaneWidth = 0.03 * newFrame.cols;

   // process ROI(bottom half)
//detect lines. pixes on line should be whitter than left and right pixs
    
    for(int i=newFrame.rows/2; i<newFrame.rows; i++){
        int laneWidth = 5+ maxLaneWidth*(i-newFrame.rows/2)/halfRows;
        for(int j=laneWidth; j<newFrame.cols- laneWidth; j++){
            int leftDiff = newFrame.at<uchar>(i,j) - newFrame.at<uchar>(i,j-laneWidth);
            int rightDiff = newFrame.at<uchar>(i,j) - newFrame.at<uchar>(i,j+laneWidth);
            int diff  =  leftDiff + rightDiff - abs(leftDiff-rightDiff);
            int diffThresh = newFrame.at<uchar>(i,j)/2;
            if (leftDiff>0 && rightDiff >0 && diff>5)
                if(diff>=diffThresh)
                    temp.at<uchar>(i,j)=255;
        }
        
    }
   
    //cv::Mat lane =deNoise(temp, newFrame);  //eliminate lines we do not want
    cv::vector<cv::Vec4i> lines;
    lines = outputLines(temp, frame);
    
    //std::cout<<lines.size()<<std::endl;
    
   /* cv::cvtColor(newFrame, newFrame, CV_GRAY2BGR);
    for(int i =0;i < lines.size();i++){
        cv::line(newFrame, cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,255,0));
    }*/
    getTrueLane(frame, temp, lines);
    return frame;
}







cv::Mat deNoise( cv::Mat lane,cv::Mat frame)
{
    cv::Mat temp = cv::Mat(frame.rows, frame.cols, CV_8UC1,0.0);//stores finally selected lane marks
    cv::Mat binaryImage; //used for blob removal
    int longLane       = 0.2 * frame.rows;
    int minRegionSize  = 0.002 * (frame.cols*frame.rows);  //min size of any region to be selected as lane
    int smallLaneArea  = 5 * minRegionSize;
    int ratio          = 4;
    
    //lines defination
    int leftLine  = 2*frame.cols/5;
    int rightLine = 3*frame.cols/5;
    int topLine   = 1*frame.rows/3;
    
    cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::vector<cv::Vec4f> Lines;
    cv::vector<cv::Vec4i> outputLines;
    
    
    
    // find all contours in image
    lane.copyTo(binaryImage);
    findContours(binaryImage, contours,hierarchy, CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    
    
    //  get real lines
    if (!contours.empty())
    {
        for (int i=0; i<contours.size(); ++i)
        {
            
            
            float contour_area = contourArea(contours[i]) ;
         
            
            //find lines larger than threshold.
            if(contour_area > minRegionSize)
            {
    
                cv::RotatedRect rotated_rect = minAreaRect(contours[i]);
                cv::Size2f sz = rotated_rect.size;
                float contour_width  = sz.width;
                float contour_length = sz.height;
                
                //adjust data according to opencv
                float blob_angle_deg = rotated_rect.angle;
                if (contour_width < contour_length)
                    blob_angle_deg = 90 + blob_angle_deg;
                
                // line longer than longLane must be road mark.
                if(contour_length>longLane || contour_width >longLane)
                {
                    drawContours(frame, contours,i, cvScalar(0), CV_FILLED, 8);
                    drawContours(temp, contours,i, cvScalar(255), CV_FILLED, 8);
                }

                
                else if((blob_angle_deg <-10 || blob_angle_deg >10 ) && ((blob_angle_deg > -70 && blob_angle_deg < 70 ) || (rotated_rect.center.y > topLine && (rotated_rect.center.x > leftLine ||rotated_rect.center.x <rightLine))))
                {
                    
                    if ((contour_length/contour_width)>=ratio || (contour_width/contour_length)>=ratio ||(contour_area< smallLaneArea &&  ((contour_area/(contour_width*contour_length)) > .75) && ((contour_length/contour_width)>=3 || (contour_width/contour_length)>=3)))
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



cv::vector<cv::Vec4i> outputLines( cv::Mat lane,cv::Mat frame)
{
    cv::Mat temp = cv::Mat(frame.rows, frame.cols, CV_8UC1,0.0);//stores finally selected lane marks
    cv::Mat binaryImage; //used for blob removal
    int minRegionSize  = 0.0005 * (frame.cols*frame.rows);  //min size of any region to be selected as lane

  
    
    cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::vector<cv::Vec4i> outputLines;
    
    
    
    // find all contours in image
    lane.copyTo(binaryImage);
    findContours(binaryImage, contours,hierarchy, CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
    
    
    //  get real lines
    if (!contours.empty())
    {
        for (int i=0; i<contours.size(); ++i)
        {
            
            cv::vector<cv::Point> cnt = contours[i];
            cv::Vec4f line;
            float contour_area = contourArea(cnt) ;
            
            
            //find lines larger than threshold.
            if(contour_area > minRegionSize)
            {
                
                cv::fitLine(cnt, line, CV_DIST_L2, 0, 0.01, 0.01);
                
                float vx,vy,k;
                vx = line[0];
                vy = line[1];
                k = vy/vx;
                cv::RotatedRect rotated_rect = minAreaRect(cnt);
                cv::Size2f sz = rotated_rect.size;
                float contour_width  = sz.width;
                float contour_length = sz.height;
                
                
                float x,y;
                x= rotated_rect.center.x;
                y = rotated_rect.center.y;
                float height;
                if(contour_length >= contour_width)
                    height= contour_length;
                else{
                    height = contour_width;
                }

                
                
                int leftx = int((-sqrt(height * height / (4 * k * k + 4)) + x));
                int lefty = int(-sqrt(height * height / (4 * k * k + 4)) * k + y);
                
                int rightx = int((sqrt(height * height / (4 * k * k + 4)) + x));
                int righty = int(sqrt(height * height / (4 * k * k + 4)) * k + y);
                
             //   float a = sqrt((leftx-rightx)*(leftx-rightx) + (lefty-righty)*(lefty-righty));
            //    std::cout<<"ping fang cha : "<<a<<std::endl;
                cv::Vec4i output;
                output[0] = leftx;
                output[1] = lefty;
                output[2] = rightx;
                output[3] = righty;
                outputLines.push_back(output);
                
                drawContours(frame, contours,i, cv::Scalar::all(0), CV_FILLED, 8);
 
            }
        }
    }

    return outputLines;
    
}




