//
//  ViewController.m
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/19/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#ifdef __cplusplus
#import "armadillo"
//#import "myfit.h"
#import "detectLane.hpp"
#import <opencv2/opencv.hpp>        // Includes the opencv library
#import "opencv2/highgui/ios.h"

#import <stdlib.h>
#import <stdio.h>
#import <iostream>
#import <sstream>

#endif
#import "ViewController.h"

using namespace std;


@interface ViewController ()

@end

@implementation ViewController
@synthesize imageView;
@synthesize startCaptureButton;

@synthesize videoCamera;

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    currentMaxAccelX = 0;
    currentMaxAccelY = 0;
    currentMaxAccelZ = 0;
    
    currentMaxRotX = 0;
    currentMaxRotY = 0;
    currentMaxRotZ = 0;
    
    self.motionManager = [[CMMotionManager alloc] init];
    self.motionManager.deviceMotionUpdateInterval=1/60; //frequnence to update,60 Hz
    [self.motionManager startDeviceMotionUpdates];
    
    self.motionManager.accelerometerUpdateInterval = .1;
    self.motionManager.gyroUpdateInterval = .1;

    
    [self.motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue]
                                             withHandler:^(CMAccelerometerData  *accelerometerData, NSError *error) {
                                                 [self outputAccelertionData:accelerometerData.acceleration];
                                                 if(error){
                                                     
                                                     NSLog(@"%@", error);
                                                 }
                                             }];
    
    [self.motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue]
                                    withHandler:^(CMGyroData *gyroData, NSError *error) {
                                        [self outputRotationData:gyroData.rotationRate];
                                    }];
    
    CMAttitude *attitude;
    attitude= self.motionManager.deviceMotion.attitude;
    rollangle=attitude.roll*180/M_PI;
    yawangle=attitude.yaw*180/M_PI;
    pitchangle=attitude.pitch*180/M_PI;
    
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:imageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationLandscapeRight;
    self.videoCamera.defaultFPS = 30;
    isCapturing = NO;
}
- (NSInteger)supportedInterfaceOrientations
{
    // Only portrait orientation
    return UIInterfaceOrientationMaskLandscapeRight;
}

- (IBAction)startCaptureButtonPressed:(id)sender {
    [videoCamera start];
    isCapturing = YES;
    framesize=cv::Size(videoCamera.imageWidth,videoCamera.imageHeight);
    std::cout<<"capturing"<<std::endl;

}

- (IBAction)stopCaptureButtonPressed:(id)sender {
    [videoCamera stop];
    isCapturing = NO;
    std::cout<<"stop"<<std::endl;
}



- (void)processImage:(cv::Mat&)image
{
    cv::Mat inputFrame = image.clone();
    cv::resize(inputFrame,inputFrame, cv::Size(640,480));
    BOOL isNeedRotation = image.size() != framesize;
    if (isNeedRotation)
        inputFrame = inputFrame.t();

 
    
   /*  // Apply filter
    cv::Mat finalFrame;
    finalFrame=inputFrame;
    if (isNeedRotation)
        finalFrame = finalFrame.t();
   
   std::ostringstream strs;
    strs << currentMaxRotZ;
    std::string str = strs.str();
    cv::putText(finalFrame, str,cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL,0.8, cv::Scalar::all(255));
    finalFrame.copyTo(image);
   std::cout<<"X rot:"<<currentMaxRotX<<"X acc:"<<currentMaxAccelX<<std::endl;
        std::cout<<"Y rot:"<<currentMaxRotY<<"Y acc:"<<currentMaxAccelY<<std::endl;
        std::cout<<"Z rot:"<<currentMaxRotZ<<"Z acc:"<<currentMaxAccelZ<<std::endl;

    
    std::ostringstream strsroll,strspitch,strsyaw;
    strsroll << rollangle;
    std::string strroll = strsroll.str();
    cv::putText(finalFrame, strroll,cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL,0.8, cv::Scalar::all(255));
    strsyaw << yawangle;
    std::string stryaw = strsyaw.str();
    cv::putText(finalFrame, stryaw,cv::Point(50, 50), cv::FONT_HERSHEY_COMPLEX_SMALL,0.8, cv::Scalar::all(255));
    strspitch << pitchangle;
    std::string strpitch = strspitch.str();
    cv::putText(finalFrame, strpitch,cv::Point(100, 100), cv::FONT_HERSHEY_COMPLEX_SMALL,0.8, cv::Scalar::all(255));
      finalFrame.copyTo(image);
     std::cout<<"X rot:"<<currentMaxRotX<<"X acc:"<<currentMaxAccelX<<std::endl;
     std::cout<<"Y rot:"<<currentMaxRotY<<"Y acc:"<<currentMaxAccelY<<std::endl;
     std::cout<<"Z rot:"<<currentMaxRotZ<<"Z acc:"<<currentMaxAccelZ<<std::endl;
    double thetaX=cosh(currentMaxAccelX);
    double thetaY=cosh(currentMaxAccelY);
    double thetaZ=cosh(currentMaxAccelZ);
     arma::fmat Rx;
     Ry,Rz,Rotation;
     Rx<<1<<0<<0<<endr
     <<0<<cos(thetaX)<<sin(thetaX)<<endr
     <<0<<-sin(thetaX)<<cos(thetaX);
     Ry<<cos(thetaY)<<0<<-sin(thetaY)<<endr
     <<0<<1<<0<<endr
     <<sin(thetaY)<<0<<cos(thetaY);
     Rz<<cos(thetaZ)<<sin(thetaZ)<<0<<endr
     <<-sin(thetaZ)<<cos(thetaZ)<<0<<endr
     <<0<<0<<1;
     Rotation=Rz*Ry*Rx;*/
    
    
    
    cv::Mat gray;
    cv::cvtColor(inputFrame, gray,CV_RGB2GRAY);
    cv::Mat finalFrame;
    finalFrame = getLines(gray);
    finalFrame.copyTo(image);

    
    
   /*
    
    
   arma::fmat Rotation;
    Rotation = getRotationMatrix(currentMaxAccelX,currentMaxAccelY,currentMaxAccelZ);
    
    std::cout<<Rotation;
    
    
    cv::Mat cvR = Arma2Cv(Rotation);
    
  //  cv::warpPerspective(finalFrame,finalFrame,cvR, framesize);
    
    arma::fmat birdView = Cv2Arma(finalFrame);
  //  LineDetection(birdView);

    finalFrame.copyTo(image);
    
   */
    
    
    
}



-(void)outputAccelertionData:(CMAcceleration)acceleration
{
    currentMaxAccelX = acceleration.x;
    currentMaxAccelY = acceleration.y;
    currentMaxAccelZ = acceleration.z;
    /*  self.maxAccX.text = [NSString stringWithFormat:@" %.2f",currentMaxAccelX];
     self.maxAccY.text = [NSString stringWithFormat:@" %.2f",currentMaxAccelY];
     self.maxAccZ.text = [NSString stringWithFormat:@" %.2f",currentMaxAccelZ];
     */
    
}
-(void)outputRotationData:(CMRotationRate)rotation
{
    currentMaxRotX = rotation.x;
    currentMaxRotY = rotation.y;
    currentMaxRotZ = rotation.z;
    /*
     self.maxRotX.text = [NSString stringWithFormat:@" %.2f",currentMaxRotX];
     self.maxRotY.text = [NSString stringWithFormat:@" %.2f",currentMaxRotY];
     self.maxRotZ.text = [NSString stringWithFormat:@" %.2f",currentMaxRotZ];*/
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}
- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
    if (isCapturing)
    {
        [videoCamera stop];
    }
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

@end
