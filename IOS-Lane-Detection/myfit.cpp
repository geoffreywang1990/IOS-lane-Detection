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
// X = concatenated matrix of 2D projected points in the image (2xN)
// W = concatenated matrix of 3D points on the plane (3XN)
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


arma::fmat getRotationMatrix(double currentMaxAccelX,double currentMaxAccelY,double currentMaxAccelZ){
    double thetaX=cosh(currentMaxAccelX);
    double thetaY=cosh(currentMaxAccelY);
    double thetaZ=cosh(currentMaxAccelZ);
    
    arma::fmat Rx;
    Rx  << 1.0 << 0.0 << 0.0 << arma::endr
    << 0.0 << float(cos(thetaX)) << float(sin(thetaX)) << arma::endr
    << 0.0 << float(-sin(thetaX)) << float(cos(thetaX));
    
    arma::fmat Ry;
    Ry<<float(cos(thetaY))<<0.0<<float(-sin(thetaY))<<arma::endr
    <<0.0<<1.0<<0.0<<arma::endr
    <<float(sin(thetaY))<<0.0<<float(cos(thetaY));
    
    arma::fmat Rz;
    Rz<<float(cos(thetaZ))<<float(sin(thetaZ))<<0.0<<arma::endr
    <<-float(sin(thetaZ))<<float(cos(thetaZ))<<0.0<<arma::endr
    <<0.0<<0.0<<1.0;
    
    arma::fmat Rotation;
    Rotation=Rz*Ry*Rx;
    return Rotation;
}

// Quick function to convert to Armadillo matrix header
arma::fmat Cv2Arma(cv::Mat &cvX)
{
    arma::fmat X(cvX.ptr<float>(0), cvX.cols, cvX.rows, false); // This is the transpose of the OpenCV X_
    return X; // Return the new matrix (no new memory allocated)
}
//==============================================================================
// Quick function to convert to OpenCV (floating point) matrix header
cv::Mat Arma2Cv(arma::fmat &X)
{
    cv::Mat cvX = cv::Mat(X.n_cols, X.n_rows,CV_32F, X.memptr()).clone();
    return cvX; // Return the new matrix (new memory allocated)
}

//arma::fmat lineDetection(arma::fmat birdView){
    
//}

