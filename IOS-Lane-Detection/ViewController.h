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
double rollangle;
double pitchangle;
double yawangle;

using namespace std;
@interface ViewController : UIViewController<CvVideoCameraDelegate>
{
    CvVideoCamera* videoCamera;
    BOOL isCapturing;
    cv::Size framesize;
    uint64_t prevTime;
    BOOL IsNew ;

}
@property (nonatomic, strong) CvVideoCamera* videoCamera;
@property (strong, nonatomic) IBOutlet UIImageView *imageView;

@property (strong, nonatomic) IBOutlet UIButton *startCaptureButton;
@property (weak, nonatomic) IBOutlet UIButton *stopCaptureButton;
- (IBAction)startCaptureButtonPressed:(id)sender;

- (IBAction)stopCaptureButtonPressed:(id)sender;

@property (weak, nonatomic) IBOutlet UIButton *changToNewMethod;
@property (weak, nonatomic) IBOutlet UIButton *changeToOldMethod;
- (IBAction)newMethodPressed:(id)sender;
- (IBAction)oldMethodPressed:(id)sender;

@property (strong, nonatomic) CMMotionManager *motionManager;

@end

