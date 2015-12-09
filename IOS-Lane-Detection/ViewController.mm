//
//  ViewController.m
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/19/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#ifdef __cplusplus
//#import "armadillo"
//#import "myfit.h"
#import "detectLane.hpp"
#import "cannyHough.hpp"
#import <opencv2/opencv.hpp>        // Includes the opencv library
#import "opencv2/highgui/ios.h"
#import <mach/mach_time.h>
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





static double matchTimeToSecs(uint64_t time){
    mach_timebase_info_data_t timebase;
    mach_timebase_info(&timebase);
    return (double)time * (double)timebase.numer /
    (double)timebase.denom / 1e9;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    /*currentMaxAccelX = 0;
    currentMaxAccelY = 0;
    currentMaxAccelZ = 0;
    
    currentMaxRotX = 0;
    currentMaxRotY = 0;
    currentMaxRotZ = 0;
    */
    prevTime = mach_absolute_time();
    IsNew = true;
        
    
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
    rollangle=90-attitude.roll*180/M_PI;
    yawangle=90-attitude.yaw*180/M_PI;
    pitchangle=90-attitude.pitch*180/M_PI;
    
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:imageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationLandscapeRight;
    self.videoCamera.defaultFPS = 30;
    isCapturing = NO;
   // videoCamera.recordVideo = YES;
    
    
    
}
- (NSInteger)supportedInterfaceOrientations
{
    // Only LandscapeRight orientation
    return UIInterfaceOrientationMaskLandscapeRight;
}


- (IBAction)newMethodPressed:(id)sender {
    IsNew = true;
}

- (IBAction)oldMethodPressed:(id)sender {
    IsNew = false;
}
- (IBAction)startCaptureButtonPressed:(id)sender {
    [videoCamera start];
    isCapturing = YES;
    framesize=cv::Size(videoCamera.imageWidth,videoCamera.imageHeight);
    std::cout<<"capturing"<<std::endl;

}

- (IBAction)stopCaptureButtonPressed:(id)sender {
    [videoCamera stop];

    
 /*   NSString* relativePath = [videoCamera.videoFileURL relativePath];
    UISaveVideoAtPathToSavedPhotosAlbum(relativePath, nil, NULL, NULL);
    UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Status"
                                                                   message:@"Saved to the Gallery!"
                                                            preferredStyle:UIAlertControllerStyleAlert];
    UIAlertAction* defaultAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault
                                                          handler:^(UIAlertAction * action) {}];
    
    [alert addAction:defaultAction];
    [self presentViewController:alert animated:YES completion:nil];
    
    isCapturing = NO;
   // std::cout<<"stop"<<std::endl;

   */
}



- (void)processImage:(cv::Mat&)image
{
    cv::Mat inputFrame = image.clone();
   // std::cout<<inputFrame.size()<<endl;
    //process 640*480 image
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
    //If detect more then two lanes, do lk to determine the correct lane
    /*UIImage* templateImg = [UIImage imageNamed:@"template.jpg"];;
     cv::Mat templateImage;
     UIImageToMat(templateImg, templateImage);
     cv::Mat H = lk(finalFrame,templateImage);
     // arma::fmat lanepts_1, lanepts_2, warp_lane_1, warp_lane_2;
     //arma::fmat warp = Cv2Arma(H);
     //warp_lane_1 = myproj_affine(lanepts_1, warp);
     //warp_lane_2 = myproj_affine(lanepts_2, warp);
     cv::Mat imgRoi = finalFrame(cv::Rect(0,0,240-1,639));
     
     cv::Size dsize = cv::Size(240, 640);
     cv::warpPerspective(templateImage,imgRoi, H, dsize,CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS);
     
     */
    /*
     
     
     arma::fmat Rotation;
     Rotation = getRotationMatrix(currentMaxAccelX,currentMaxAccelY,currentMaxAccelZ);
     
     std::cout<<Rotation;
     
     
     cv::Mat cvR = Arma2Cv(Rotation);
     
     //  cv::warpPerspective(finalFrame,finalFrame,cvR, framesize);
     
     arma::fmat birdView = Cv2Arma(finalFrame);
     //  LineDetection(birdView);*/

    
    
    
    
    //cvt to gray

    cv::Mat resizedImage;
    cv::resize(inputFrame, resizedImage, cv::Size(640,480));


    
    
    
    cv::Mat finalFrame;
    
    
    if(IsNew)
    finalFrame = getLines(resizedImage);
    else{
        finalFrame = houghDetect(resizedImage);}
    
    
    
    
    
    
    //_________________________________________________________________
    //show fps on frame
    uint64_t currTime = mach_absolute_time();
    double timeInSeconds = matchTimeToSecs(currTime - prevTime);
    prevTime = currTime;
    double fps = 1.0 / timeInSeconds;
    finalFrame.copyTo(image);
    NSString *fpsString = [NSString stringWithFormat:@"FPS = %3.2f",fps];
    cv::putText(finalFrame, [fpsString UTF8String], cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar::all(255));
    //______________________________________________________________________________________________________________
    
    


    finalFrame.copyTo(image);
    

    
    
    
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
