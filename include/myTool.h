#ifndef MY_TOOL_H
#define MY_TOOL_H

#include<opencv2/opencv.hpp>

void UndistortImage(cv::Mat image, cv::Mat intrinsic_matrix,cv::Mat distortion_coeffs);

void ShowUndistort(cv::Mat image, cv::Mat intrinsic_matrix,cv::Mat distortion_coeffs);

void ShowDepth(const cv::Mat &imRGB,const cv::Mat &imD);

#endif // TRACKING_H
