//
//  ViewController.h
//  IOS-Lane-Detection
//
//  Created by geoffrey on 11/19/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//
#import <opencv2/highgui/ios.h>
#import <CoreMotion/CoreMotion.h>
#import <UIKit/UIKit.h>


double currentMaxAccelX;
double currentMaxAccelY;
double currentMaxAccelZ;
double currentMaxRotX;
double currentMaxRotY;
double currentMaxRotZ;
using namespace std;
@interface ViewController : UIViewController<CvVideoCameraDelegate>
{
    CvVideoCamera* videoCamera;
    BOOL isCapturing;
    cv::Size framesize;
}
@property (nonatomic, strong) CvVideoCamera* videoCamera;
@property (strong, nonatomic) IBOutlet UIImageView *imageView;

@property (strong, nonatomic) IBOutlet UIButton *startCaptureButton;
@property (weak, nonatomic) IBOutlet UIButton *stopCaptureButton;
- (IBAction)startCaptureButtonPressed:(id)sender;

- (IBAction)stopCaptureButtonPressed:(id)sender;


@property (strong, nonatomic) CMMotionManager *motionManager;
@end

