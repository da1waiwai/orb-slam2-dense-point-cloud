#include "myTool.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include<mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

void UndistortImage(cv::Mat image, cv::Mat intrinsic_matrix,cv::Mat distortion_coeffs)
{
//    cv::Size image_size = image.size();
//    cv::Mat mapx = cv::Mat(image_size,CV_32FC1);
//    cv::Mat mapy = cv::Mat(image_size,CV_32FC1);
//    cv::Mat R = cv::Mat::eye(3,3,CV_32F);
////    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
//    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,Mat(),Mat(),image_size,CV_32FC1,mapx,mapy);

//    cv::remap(image,image,mapx, mapy, cv::INTER_NEAREST);
////    cout<<"image.cols="<<image.cols<<",image.rows="<<image.rows<<endl;
}

void ShowUndistort(cv::Mat image, cv::Mat intrinsic_matrix,cv::Mat distortion_coeffs)
{
//    cv::Size image_size = image.size();
//    cv::Mat mapx = cv::Mat(image_size,CV_32FC1);
//    cv::Mat mapy = cv::Mat(image_size,CV_32FC1);
//    cv::Mat R = cv::Mat::eye(3,3,CV_32F);
//    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
////    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,Mat(),
////                            getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),
////                            image_size,CV_32FC1,mapx,mapy);
//    cv::Mat t = image.clone();
//    cv::remap(image,t,mapx, mapy, cv::INTER_LINEAR);
//    imshow("Undistort",t);

//    //    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
//    initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,Mat(),
//                            getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),
//                            image_size,CV_32FC1,mapx,mapy);
//    cv::remap(image,t,mapx, mapy, cv::INTER_LINEAR);
//    imshow("Undistort2",t);
//    cvWaitKey(0);

}
void ShowDepth(const cv::Mat &imRGB,const cv::Mat &imD)
{
//    cv::Mat imShow = imRGB.clone();
//    for ( int m=0; m<imD.rows; m++ )
//    {
//        for ( int n=0; n<imD.cols; n++ )
//        {
//            float d = imD.ptr<float>(m)[n];
//            if (d < 0.01 )
//            {
//                imShow.at<Vec3b>(m,n)[0]=0;
//                imShow.at<Vec3b>(m,n)[1]=0;
//                imShow.at<Vec3b>(m,n)[2]=0;
//            }
//        }
//    }
//    imshow("ShowDepth",imShow);
}

