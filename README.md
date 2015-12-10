#IOS-lane-Detection
##Title:
Mobile Real Time Road Lane Detection
##Members:
Yujun Wang(Andrew ID: yujunwan) and Juedou Liu(Andrew ID: juedoul)
##Summary:
We want to detect road lane in real time video. Because opencv hough transformation is slow. We want to find a fast way to do it on mobile devise.
##Background:
We will use descriptors to detect yellow and white road lines.Then use hough transformation to mark the lane area in the video. We will take advantage of OpenGL ES GPUImage package to accelerate the computing speed.

##The challege:
This problem can be solve in openCv easily. However, the hough transformation basicly took a lot of time. And it's hard to do it in real time without dropping frames or resize the image to a much smal    ler one. We are trying to implement this on a mobile device at a relative fast speed.  
Sloving this problem in mobile platform means a lot. It further can be combine with GPS and IMU to do Vision SLAM, and this can be used on map navigation app. Because map navigation app, like google a    pp, sometime is ambiguity even with animations.  

##Goals & Deliverables:

###Plan to Achieve: 
####Detect road lines:
Use descriptors and hough transformation.
####Segment the road into different lanes:
Draw different lane into different and show it on the video.
####Extra goal:
Combine with GPS and IMU tracking the lane.

###Evaluation:
Based on processing speed and vision result.
####Processing speed:
Actural process speed(how much frame per second)
####Vision result:
How good the visual result looks.

###Feasibility：
It is practical. White and yellow road lines have a lot of feature. So it is easy to detect them in the image. With these lines, we can Segment them easily. So the work can be done in 4 weeks.

##Schedule:
####First Week:
Prepare train data for road lines detection. And research into GPUImage, Lane Segmentation and IOS App UI development more detail.
####Second Week:
Program app UIview, Delegate and Storyboard.
####Third Week:
Code the Computer Vision part algroithm.
####Fourth Week:
Finalize the app.

##Youtube Clip:
https://www.youtube.com/watch?v=tXhUFzwJxJ0
