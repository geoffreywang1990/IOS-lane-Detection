//
//  detectLane.hpp
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/27/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#ifndef detectLane_hpp
#define detectLane_hpp

#include <stdio.h>
#include "armadillo"
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include "cannyHough.hpp"


#endif /* detectLane_hpp */

cv::Mat getLines(cv::Mat frame);
cv::Mat deNoise(cv::Mat lane,cv::Mat newFrame);
cv::vector<cv::Vec4i> outputLines( cv::Mat lane,cv::Mat frame);
arma::fmat Cv2Arma(cv::Mat &cvX);
cv::Mat Arma2Cv(arma::fmat &X);
