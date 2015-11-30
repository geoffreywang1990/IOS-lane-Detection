//
//  myfit.cpp
//  Estimate_Homography
//
//  Created by Simon Lucey on 9/21/15.
//  Copyright (c) 2015 CMU_16432. All rights reserved.
//

#include "myfit.h"

// Use the Armadillo namespace
using namespace arma;

//-----------------------------------------------------------------
// Function to return the affine warp between 3D points on a plane
//
// <in>
// X = concatenated matrix of 2D projected points in the image (2xN)  dst
// W = concatenated matrix of 3D points on the plane (3XN)            src
//
// <out>
// A = 2x3 matrix of affine parameters
fmat myfit_affine(fmat &X,  fmat &W) {
    // Fill in the answer here.....
    
    fmat A;
    A.set_size(2,3);                    //set A size 2,3
    fmat tempA,tempw,tempx;
    tempx = X;
    long long N  = W.n_cols;
    tempw.set_size(2*N,6);
    tempx.reshape(2*N,1);               //reshape X to a col
    for(int i=0;i<N;i++){
        float a = W(0,i);
        float b = W(1,i);
        fmat c;
        c   <<a<<b<<0<<0<<1<<0<<endr
            <<0<<0<<a<<b<<0<<1;
        tempw.rows(2*i,2*i+1)=c;        //make A
    }
    tempA = solve(tempw,tempx);         //A = W/X
    A(0,0) = tempA(0,0);                //re-order A
    A(0,1) = tempA(1,0);
    A(0,2) = tempA(4,0);
    A(1,0) = tempA(2,0);
    A(1,1) = tempA(3,0);
    A(1,2) = tempA(5,0);
   // cout<<A;
    return A;
}
//-----------------------------------------------------------------
// Function to project points using the affine transform
//
// <in>
// W = concatenated matrix of 3D points on the plane (3XN)
// A = 2x3 matrix of affine parameters
//
// <out>
// X = concatenated matrix of 2D projected points in the image (2xN)
fmat myproj_affine(fmat &W, fmat &A) {

    // Fill in the answer here.....
    fmat X,tempw;
    tempw = W;
    tempw.row(2).fill(1.0); //make w = [u,v,1]'
    X = A*tempw;
    return X;
}

//-----------------------------------------------------------------
// Function to return the affine warp between 3D points on a plane
//
// <in>
// X = concatenated matrix of 2D projected points in the image (2xN)
// W = concatenated matrix of 3D points on the plane (3XN)
//
// <out>
// H = 3x3 homography matrix
fmat myfit_homography(fmat &X, fmat &W) {
    
    // Fill in the answer here.....
    fmat H,A;
    H.set_size(3,3);
    A.set_size(8,9);
    long long N  = W.n_cols;
    
    for(int i=0;i<N;i++){       //make A
        float a = W(0,i);
        float b = W(1,i);
        float x = X(0,i);
        float y = X(1,i);
        fmat c;
        c<<0<<0<<0<<-a<<-b<<-1<<y*a<<y*b<<y<<endr
        <<a<<b<<1<<0<<0<<0<<-x*a<<-x*b<<-x;
        A.rows(2*i,2*i+1) = c;
        
    }
    fmat U;
    fvec s;
    fmat V;
    svd(U,s,V,A);           //svd A
    cout<<V<<endl;
    fmat Phi;
    Phi.set_size(9,1);
    Phi =V.col(V.n_cols-1); //phi
    
   // cout<<Phi<<endl;
    Phi.reshape(3,3);
    H = Phi.t();
  //  cout<<H;
    return H;
}

//-----------------------------------------------------------------
// Function to project points using the affine transform
//
// <in>
// W = concatenated matrix of 3D points on the plane (3XN)
// H = 3x3 homography matrix
//
// <out>
// X = concatenated matrix of 2D projected points in the image (2xN)
fmat myproj_homography(fmat &W, fmat &H) {
    
    // Fill in the answer here.....
    fmat X;
    X.set_size(2,W.n_cols);
    fmat K,tempw;
    tempw = W;
    tempw.row(2).fill(1.0);
    K = H*tempw;                //use cartesian rather than homogeneous 
   // cout<<'K'<<endl<<K<<endl;
    X.row(0) = K.row(0)/K.row(2);
    X.row(1) = K.row(1)/K.row(2);
    
    return X;
}


//_________________________________________
//get rotation matrix based on IMU
//Input: IMU data


fmat getRotationMatrix(double pitchangle,double rollangle,double yawangle){

    
    fmat Rx;
    Rx  << 1.0 << 0.0 << 0.0 << endr
    << 0.0 << float(cos(-pitchangle)) << float(sin(-pitchangle)) << endr
    << 0.0 << float(-sin(-pitchangle)) << float(cos(-pitchangle));
    
    fmat Ry;
    Ry<<float(cos(rollangle))<<0.0<<float(-sin(rollangle))<<endr
    <<0.0<<1.0<<0.0<< endr
    <<float(sin(rollangle))<<0.0<<float(cos(rollangle));
    
     fmat Rz;
    Rz<<float(cos(M_PI/2- rollangle))<<float(sin(M_PI/2-rollangle))<<0.0<<endr
    <<-float(sin(M_PI/2-rollangle))<<float(cos(M_PI/2-rollangle))<<0.0<< endr
    <<0.0<<0.0<<1.0;

    
     fmat Rotation;
    Rotation=Rz;
    return Rotation;
}

// Quick function to convert to Armadillo matrix header
fmat Cv2Arma(cv::Mat &cvX)
{
     fmat X(cvX.ptr<float>(0), cvX.cols, cvX.rows, false); // This is the transpose of the OpenCV X_
    return X; // Return the new matrix (no new memory allocated)
}
//==============================================================================
// Quick function to convert to OpenCV (floating point) matrix header
cv::Mat Arma2Cv( fmat &X)
{
    cv::Mat cvX = cv::Mat(X.n_cols, X.n_rows,CV_32F, X.memptr()).clone();
    return cvX; // Return the new matrix (new memory allocated)
}
fmat M2P( fmat &M)
{
    fmat P;
    P.set_size(6,1);
    P(0,0)=1-M(0,0);
    P(1,0)=M(0,1);
    P(2,0)=M(0,2);
    P(3,0)=M(1,0);
    P(4,0)=1-M(1,1);
    P(5,0)=M(1,2);
    return P;
}
 fmat P2M( fmat P)
{
    fmat M;
    M<<1-P(0)<<P(1)<<P(2)<< endr
    <<P(3)<< 1-P(4)<<P(5)<< endr
    << 0<<0<<1;
    return M;
}






cv::Mat lk(cv::Mat targetImage,cv::Mat templateImg)
{
    fmat image = Cv2Arma(targetImage);
    fmat tempImage = Cv2Arma(templateImg);
    
    int cols = int(image.n_cols);
    int rows = int(image.n_rows);
    fmat rect,tempPts;
    //points of window in image
    rect<< rows/2 << rows/2 << rows-1 << rows-1 <<  endr
        << 0 << cols-1 << 0 << cols-1;
    //points of template window
    tempPts<< 0 << 0 << tempImage.n_rows-1 << tempImage.n_rows-1 <<  endr
            << 0 << tempImage.n_cols-1 << 0 << tempImage.n_cols-1;
    fmat tempw;
    tempw.ones(3,4);
    tempw.rows(0,1) = tempPts;
    //lk-IC
    fmat initM;
    initM.zeros(3,3);
    initM.rows(0,1) = myfit_affine(rect,tempw);
    initM(2,2) = 1;
    //matrix: template move to target position
    

    //gradient of template
    fvec x,y;
    x<<-1<<1;
    y = x.t();
    fmat Tx = conv(tempImage,x);
    fmat Ty = conv(tempImage,y);
    fmat X,Y;
    fmat tempx,tempy;
    //_________________________________________________

    tempx = linspace<fmat>(0, tempImage.n_rows-1,tempImage.n_rows);
    tempy = linspace<fmat>(0, tempImage.n_cols-1,tempImage.n_cols);
    fmat XX = repmat(tempx, tempImage.n_cols, 1);
    fmat YY = repmat(tempy.t(), tempImage.n_rows, 1);
    X = vectorise(XX);
    Y = vectorise(YY);
    fmat vzeros = zeros(X.n_rows,3);
    fmat vones = ones(X.n_rows,1);
    fmat dWx = join_rows(join_rows(-X, Y), join_rows(vones,vzeros));
    fmat dWy = join_rows(join_rows(vzeros, X), join_rows(-Y,vones));
    //_______________________________________________________________
    

    
    //jacobian
    fmat T_x = vectorise(Tx);
    fmat T_y = vectorise(Ty);
    fmat J = dWx%repmat(T_x,1,6) + dWy%repmat(T_y,1,6);
    //R
    fmat R = inv(J.t()*J)*J.t();
    //start LK iteration
    fmat P = M2P(initM);
    fmat M = initM;
  
    //size of warped image
    cv::Size dsize = cv::Size(rows/2,cols);
    //dst is the warped image
    cv::Mat dst;
    //region of interest
    cv::Mat imgRoi = targetImage(cv::Rect(rows/2-1,0,cols,rows/2));
    for(int j=1;j<50;j++)
    {
        fmat invM = inv(M);
        cv::warpPerspective(imgRoi,dst, Arma2Cv(invM), dsize,CV_INTER_LINEAR+CV_WARP_INVERSE_MAP+CV_WARP_FILL_OUTLIERS);
        
        cv::Mat err = dst-templateImg;
        err.reshape(0,1);
        fmat error = Cv2Arma(err).t();
        fmat dp = R*error;
        fmat dM = P2M(dp);
        M = M*inv(dM);
        P = M2P(M);
    }
    cv::Mat H;  //H is a matrix from templet to Image
    H = Arma2Cv(M);

    return H;
}
